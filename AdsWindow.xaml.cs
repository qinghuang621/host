using System.Diagnostics;
using System.Globalization;
using System.IO;
using System.Text;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using System.Windows.Shapes;
using System.Windows.Threading;
using Path = System.IO.Path;   // System.Windows.Shapes 里也有 Path（图形），显式指向 IO

namespace GamepadSpeedController
{
    /// <summary>
    /// 负压传感窗口 —— 读 ADS1256 采集卡（接口文档.md §6.11，只读区 0x01A8~0x01C3）。
    ///
    /// 数据链路：传感器 VOUT → AIN0~AIN3 →（SPI2，Mode1）→ C 板 AdcTask
    ///           → Modbus FC03 28 只 → 本窗口。
    /// 电压**已由固件换算**（ACOM=0V 单端、BUFEN=0：V = code × 5 / 8388608，不加 2.5V）；
    /// 压力换算放在上位机（固件 P0~P3 目前恒 0），公式 P = ±P_full × (V − V0) / 4.0。
    ///
    /// 与 MainWindow 的耦合方式跟 AttitudeWindow 一致：**本窗口不自己开轮询线程**，
    /// 由 MainWindow 的 CommLoop 在窗口打开时每 200 ms 读一次并调 <see cref="UpdateAds"/>。
    /// 好处：串口只有一个读线程在排队，不会和电机/风机/IMU 抢帧；
    /// 代价：主串口没连接时本窗口只能显示"无数据"，这符合实际。
    /// </summary>
    public partial class AdsWindow : Window
    {
        // ===== 趋势图 =====
        private const int ChartSeconds = 30;          // 横轴窗口
        private const int MaxSamples   = ChartSeconds * 8 + 40;  // 采样 5 Hz，留余量

        private static readonly Color[] CurveColors =
        {
            Color.FromRgb(0xD3, 0x2F, 0x2F),   // 红  AIN0
            Color.FromRgb(0x19, 0x76, 0xD2),   // 蓝  AIN1
            Color.FromRgb(0x2E, 0x7D, 0x32),   // 绿  AIN2
            Color.FromRgb(0xF5, 0x7C, 0x00),   // 橙  AIN3
        };

        private readonly Polyline[] _curves = new Polyline[4];
        private readonly List<(long Ticks, float[] V)> _hist = new();

        // ===== 最近一帧 + 统计 =====
        private AdsSnapshot _last;
        private bool _hasData;
        private DateTime _lastSampleAt = DateTime.MinValue;
        private DateTime _prevSeqAt = DateTime.MinValue;
        private ushort _prevSeq;
        private bool _seqPrimed;
        private double _rateHz;
        private int _stuckMs;
        private ushort _lastStatus;
        private bool _statusPrimed;

        // ===== 零点修正（kPa，四路各一份，默认 0） =====
        // 两点标定（TxtCalV1/P1/V2/P2）是"传感器型号"级的参数，四路共用；
        // 零点修正是"这一只表"的零漂，逐路独立。
        private readonly double[] _poff = { 0, 0, 0, 0 };
        private bool _ready;      // 构造完成前不响应控件事件
        private bool _syncUi;     // 回填 TxtPoff 时抑制 TextChanged
        private int _logLines;

        private readonly DispatcherTimer _uiTimer = new() { Interval = TimeSpan.FromMilliseconds(500) };
        private readonly DispatcherTimer _recTimer = new() { Interval = TimeSpan.FromSeconds(1) };

        public AdsWindow()
        {
            InitializeComponent();

            for (int i = 0; i < 4; i++) CbTareCh.Items.Add($"AIN{i}");
            CbTareCh.SelectedIndex = 0;

            for (int i = 0; i < 4; i++)
            {
                _curves[i] = new Polyline
                {
                    Stroke = new SolidColorBrush(CurveColors[i]),
                    StrokeThickness = 1.6,
                };
                CurveCanvas.Children.Add(_curves[i]);
            }

            Loaded += (_, _) => { RebuildGrid(); TxtSource.Text = "数据源：主串口 → Modbus FC03 0x01A8（只读）"; };
            _uiTimer.Tick += UiTimer_Tick;
            _uiTimer.Start();
            _recTimer.Tick += RecTimer_Tick;
            _recTimer.Start();

            _ready = true;
            UpdateCalcInfo();
            Log("窗口已打开：等主串口连接后自动开始读取（0x01A8 起 28 只）");
            Log("标定默认按 -100~50 kPa / 0.5~4.5 V：0 kPa 对应 3.167 V，不是 0.5 V");
        }

        // ==================== 数据入口（由 MainWindow.CommLoop 在 UI 线程调用） ====================

        /// <summary>
        /// 刷新一帧采集卡数据。调用方已在 UI 线程（MainWindow 用 Dispatcher.BeginInvoke）。
        /// </summary>
        public void UpdateAds(AdsSnapshot s)
        {
            var now = DateTime.Now;
            _last = s;
            _hasData = true;
            _lastSampleAt = now;

            // ---- 状态灯 ----
            if (!_statusPrimed || s.Status != _lastStatus)
            {
                if (_statusPrimed)
                    Log($"ADS_STATUS 变化：{_lastStatus} → {s.Status}（{StatusText(s.Status)}）");
                _statusPrimed = true;
                _lastStatus = s.Status;
            }
            PaintStatus(s.Status);

            // ---- SEQ 速率 + "采样停住"检测 ----
            if (_seqPrimed)
            {
                double dtMs = (now - _prevSeqAt).TotalMilliseconds;
                if (dtMs >= 60)
                {
                    int d = (s.Seq - _prevSeq) & 0xFFFF;   // uint16 回绕安全
                    double hz = d * 1000.0 / dtMs;
                    _rateHz = _rateHz <= 0 ? hz : _rateHz * 0.7 + hz * 0.3;
                    TxtRate.Text = $"{_rateHz:F1} Hz";
                    if (d == 0)
                    {
                        _stuckMs += (int)dtMs;
                        if (_stuckMs >= 1500)
                        {
                            LedStatus.Background = Brushes.OrangeRed;
                            TxtStatus.Text = $"ADS_STATUS = {s.Status}（正常，但 SEQ 已停 {_stuckMs / 1000.0:F1}s）";
                            TxtStatus.Foreground = Brushes.Firebrick;
                        }
                    }
                    else _stuckMs = 0;
                }
            }
            _prevSeq = s.Seq;
            _prevSeqAt = now;
            _seqPrimed = true;

            TxtSeq.Text  = s.Seq.ToString();
            TxtAge.Text  = "0.0s 前";

            // ---- 四路 ----
            if (s.Volts != null && s.Raw != null)
            {
                for (int i = 0; i < 4; i++)
                    UpdateChannel(i, s.Volts[i], s.Raw[i]);
            }

            // ---- 趋势 ----
            _hist.Add((now.Ticks, s.Volts ?? new float[4]));
            int drop = _hist.Count - MaxSamples;
            if (drop > 0) _hist.RemoveRange(0, drop);
            RedrawCurves();
        }

        private void PaintStatus(ushort status)
        {
            var (txt, led) = status switch
            {
                0 => ("正常", Brushes.LimeGreen),
                1 => ("DRDY 超时", Brushes.OrangeRed),
                2 => ("初始化失败", Brushes.OrangeRed),
                3 => ("SPI 错误", Brushes.OrangeRed),
                _ => ($"未知({status})", Brushes.OrangeRed),
            };
            LedStatus.Background = led;
            TxtStatus.Text = $"ADS_STATUS = {status}（{txt}）";
            TxtStatus.Foreground = status == 0 ? Brushes.Black : Brushes.Firebrick;
        }

        private static string StatusText(ushort status) => status switch
        {
            0 => "正常", 1 => "DRDY 超时", 2 => "初始化失败", 3 => "SPI 错误",
            _ => $"未知({status})",
        };

        private void UpdateChannel(int i, float v, int raw)
        {
            var txtV = i switch { 0 => TxtV1, 1 => TxtV2, 2 => TxtV3, _ => TxtV4 };
            var pb   = i switch { 0 => PbV1,  1 => PbV2,  2 => PbV3,  _ => PbV4 };
            var tRaw = i switch { 0 => TxtRaw1, 1 => TxtRaw2, 2 => TxtRaw3, _ => TxtRaw4 };
            var tP   = i switch { 0 => TxtP1, 1 => TxtP2, 2 => TxtP3, _ => TxtP4 };
            var tJ   = i switch { 0 => TxtJudge1, 1 => TxtJudge2, 2 => TxtJudge3, _ => TxtJudge4 };
            var led  = i switch { 0 => LedCh1, 1 => LedCh2, 2 => LedCh3, _ => LedCh4 };
            if (txtV == null || pb == null || tRaw == null || tP == null || tJ == null || led == null) return;

            txtV.Text = $"{v:F4} V";
            pb.Value = Math.Clamp(v, 0f, 5f);
            // 24bit 两补码显示：负向饱和 = 0x800000
            tRaw.Text = $"0x{raw & 0xFFFFFF:X6} ({raw})";

            float p = Pressure(i, v);
            tP.Text = $"{p:F1} kPa";

            // 开路偏置（≈1.2V）在本量程下会算成约 -74 kPa，看起来像"强真空" —— 数值不可信，
            // 用灰色压下去，靠右边的判读文字说明；超出标定范围太多才报红。
            bool openBias = v > 1.00f && v < 1.40f;
            GetRange(out double pLo, out double pHi);
            if (p < pLo - 3 || p > pHi + 3) tP.Foreground = Brushes.Firebrick;
            else if (openBias)              tP.Foreground = Brushes.Silver;
            else if (Math.Abs(p) < 0.15)    tP.Foreground = Brushes.DimGray;
            else                            tP.Foreground = p < 0 ? Brushes.DarkOrange : Brushes.DarkGreen;

            var (judge, jColor, ledColor) = Judge(v);
            tJ.Text = judge;
            tJ.Foreground = jColor;
            led.Background = ledColor;
        }

        /// <summary>
        /// 电压 → 压力：**两点线性标定**，适配任意量程/零点（含 -100~+50 kPa 这种双向量程）。
        ///     P = P₁ + (V − V₁) × (P₂ − P₁) / (V₂ − V₁) + 零点修正
        /// 默认值对应本机传感器（-100~50 kPa / 0.5~4.5 V）：
        ///     0.5 V ↔ -100 kPa，4.5 V ↔ +50 kPa ⇒ 斜率 37.5 kPa/V，**0 kPa 在 3.167 V**（不是 0.5 V）。
        /// 2026-11-12 实测：桌面常压读数 3.1586 V，与 3.167 V 吻合 → 传感器与接线都正常。
        /// </summary>
        private float Pressure(int ch, float v) => (float)(PressureRaw(v) + _poff[ch]);

        /// <summary>不含零点修正的原始换算值</summary>
        private double PressureRaw(double v)
        {
            double v1 = ParseDouble(TxtCalV1?.Text, 0.5), p1 = ParseDouble(TxtCalP1?.Text, -100);
            double v2 = ParseDouble(TxtCalV2?.Text, 4.5), p2 = ParseDouble(TxtCalP2?.Text, 50);
            double dv = v2 - v1;
            if (Math.Abs(dv) < 1e-6) return 0;
            return p1 + (v - v1) * (p2 - p1) / dv;
        }

        /// <summary>标定覆盖的压力区间（低/高），用于判"超出量程"</summary>
        private void GetRange(out double pLo, out double pHi)
        {
            double p1 = ParseDouble(TxtCalP1?.Text, -100);
            double p2 = ParseDouble(TxtCalP2?.Text, 50);
            pLo = Math.Min(p1, p2);
            pHi = Math.Max(p1, p2);
        }

        /// <summary>标定行下方的实时信息：斜率 / 量程 / 0 kPa 对应的电压</summary>
        private void UpdateCalcInfo()
        {
            if (TxtCalcInfo == null) return;
            double v1 = ParseDouble(TxtCalV1?.Text, 0.5), p1 = ParseDouble(TxtCalP1?.Text, -100);
            double v2 = ParseDouble(TxtCalV2?.Text, 4.5), p2 = ParseDouble(TxtCalP2?.Text, 50);
            double dv = v2 - v1;
            if (Math.Abs(dv) < 1e-6)
            {
                TxtCalcInfo.Text = "⚠ 两点的电压不能相同";
                TxtCalcInfo.Foreground = Brushes.Firebrick;
                return;
            }
            double k = (p2 - p1) / dv;
            double v0 = v1 - p1 / k;      // 0 kPa 对应的电压
            TxtCalcInfo.Foreground = Brushes.Gray;
            TxtCalcInfo.Text =
                $"P = P₁ + (V − V₁) × (P₂ − P₁)/(V₂ − V₁) + 零点修正　→　" +
                $"斜率 {k:F2} kPa/V，量程 {Math.Min(p1, p2):F0}~{Math.Max(p1, p2):F0} kPa，" +
                $"0 kPa @ {v0:F3} V";
        }

        /// <summary>
        /// 单路读数判读。
        /// 🔴 两条实测经验（2026-11-12）：
        ///  ① 该黄板 AIN 上的 2.5V 是**开路偏置**（万用表几乎不取电流才量得到），
        ///     ADS1256 在 BUFEN=0 时是开关电容输入，会从 AIN 抽电荷，把这个高内阻偏置
        ///     **拉到约一半 ≈1.2V**。三路未接通道实测 1.2028/1.2028/1.2027 V（差 &lt;1mV）。
        ///  ② **不能按固定电压阈值判量程**：本机表是 -100~50 kPa / 0.5~4.5V，
        ///     2.5V 其实是 -25 kPa（正常读数），所以越限判断必须换算成压力后再比。
        ///    ⚠ 1.2V 对应 -73.6 kPa，与"强负压"重合，所以这一档只能提示"两者之一"。
        /// </summary>
        private (string Text, Brush TextColor, Brush LedColor) Judge(float v)
        {
            if (v < -0.02f)
                return ($"负值 {v:F3}V：低于 0V，查接线/标定", Brushes.Firebrick, Brushes.OrangeRed);
            if (v < 0.05f)
                return ("≈0V：AIN 短地或未接", Brushes.DarkOrange, Brushes.OrangeRed);
            if (v > 4.95f)
                return ($"{v:F3}V：超量程 / 短到 5V", Brushes.Firebrick, Brushes.OrangeRed);
            if (v > 1.00f && v < 1.40f)
                return ($"{v:F3}V：开路偏置（未接）／也可能是强负压", Brushes.DarkOrange, Brushes.Goldenrod);

            GetRange(out double pLo, out double pHi);
            double p = PressureRaw(v);
            if (p < pLo - 2.0) return ($"低于量程下限（{p:F0} kPa）", Brushes.Firebrick, Brushes.OrangeRed);
            if (p > pHi + 2.0) return ($"高于量程上限（{p:F0} kPa）", Brushes.Firebrick, Brushes.OrangeRed);
            return ($"量程内（{p:F0} kPa）", Brushes.Green, Brushes.LimeGreen);
        }

        // ==================== 趋势图 ====================

        private void ChartHost_SizeChanged(object sender, SizeChangedEventArgs e) => RebuildGrid();

        /// <summary>重画网格（只在尺寸变化时做），曲线层叠在上面。</summary>
        private void RebuildGrid()
        {
            double w = GridCanvas.ActualWidth, h = GridCanvas.ActualHeight;
            if (w <= 1 || h <= 1) return;
            GridCanvas.Children.Clear();

            for (int i = 0; i <= 5; i++)   // 0/1/2/3/4/5 V 五格
            {
                double y = h - h * i / 5.0;
                GridCanvas.Children.Add(new Line
                {
                    X1 = 0, X2 = w, Y1 = y, Y2 = y,
                    Stroke = Brushes.Gainsboro, StrokeThickness = 1,
                });
                // 0V 不写标签：左下角是 "-30s" 时间刻度，两个标签会叠在一起
                if (i == 0) continue;
                var tb = new TextBlock { Text = $"{i}V", FontSize = 10, Foreground = Brushes.Gray };
                Canvas.SetLeft(tb, 2);
                Canvas.SetTop(tb, Math.Max(0, y - 13));
                GridCanvas.Children.Add(tb);
            }

            for (int s = 0; s <= ChartSeconds; s += 5)   // 每 5 s 一条竖线
            {
                double x = w * s / ChartSeconds;
                GridCanvas.Children.Add(new Line
                {
                    X1 = x, X2 = x, Y1 = 0, Y2 = h,
                    Stroke = Brushes.Gainsboro, StrokeThickness = 1,
                });
                var tb = new TextBlock
                {
                    Text = s == ChartSeconds ? "现在" : $"-{ChartSeconds - s}s",
                    FontSize = 10, Foreground = Brushes.Gray,
                };
                Canvas.SetLeft(tb, Math.Min(x + 2, w - 34));
                Canvas.SetTop(tb, h - 14);
                GridCanvas.Children.Add(tb);
            }

            RedrawCurves();   // 尺寸变了，曲线要按新宽高重算（否则拉伸后线会跑出边界）
        }

        private void RedrawCurves()
        {
            double w = CurveCanvas.ActualWidth, h = CurveCanvas.ActualHeight;
            if (w <= 1 || h <= 1) return;

            if (_hist.Count < 2)
            {
                foreach (var c in _curves) c.Points.Clear();
                return;
            }

            long end   = _hist[^1].Ticks;
            long start = end - ChartSeconds * TimeSpan.TicksPerSecond;
            double span = end - start;

            for (int ch = 0; ch < 4; ch++)
            {
                var pts = _curves[ch].Points;
                pts.Clear();
                foreach (var (t, v) in _hist)
                {
                    if (t < start) continue;
                    double x = (t - start) * w / span;
                    double y = h - Math.Clamp(v[ch], 0f, 5f) / 5.0 * h;
                    pts.Add(new Point(x, y));
                }
            }
        }

        // ==================== 标定 ====================

        private void CbTareCh_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (!_ready || TxtPoff == null) return;
            int i = Math.Clamp(CbTareCh.SelectedIndex, 0, 3);
            _syncUi = true;
            TxtPoff.Text = _poff[i].ToString("F1", CultureInfo.InvariantCulture);
            _syncUi = false;
        }

        /// <summary>零点修正（kPa）：编辑的是 CbTareCh 选中那一路的值，四路各存一份。</summary>
        private void Poff_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
        {
            if (!_ready || _syncUi || CbTareCh == null) return;
            int i = Math.Clamp(CbTareCh.SelectedIndex, 0, 3);
            if (double.TryParse(TxtPoff.Text, NumberStyles.Float, CultureInfo.InvariantCulture, out var v)
                && v > -500 && v < 500)
            {
                _poff[i] = v;
            }
        }

        private void Calib_TextChanged(object sender, System.Windows.Controls.TextChangedEventArgs e)
            => UpdateCalcInfo();

        private void BtnTare_Click(object sender, RoutedEventArgs e)
        {
            if (!_hasData || _last.Volts == null)
            {
                MessageBox.Show(this, "还没有读到数据：先连接主串口，确认 ADS_STATUS=0 且 SEQ 在涨。",
                    "无数据", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            int i = Math.Clamp(CbTareCh.SelectedIndex, 0, 3);
            float v = _last.Volts[i];
            // 令当前读数 = 0：只改零点修正，不动两点标定（斜率/量程保持不变）
            double raw = PressureRaw(v);
            _poff[i] = -raw;
            _syncUi = true;
            TxtPoff.Text = _poff[i].ToString("F1", CultureInfo.InvariantCulture);
            _syncUi = false;
            Log($"零点：AIN{i} 在 {v:F4} V 处置 0（原始 {raw:F1} kPa，修正 {_poff[i]:+0.0;-0.0;0.0} kPa）");
        }

        private static double ParseDouble(string? s, double fallback)
            => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out var v) ? v : fallback;

        // ==================== 定时器：数据年龄 / 1Hz 记录 ====================

        private void UiTimer_Tick(object? sender, EventArgs e)
        {
            if (!_hasData)
            {
                TxtAge.Text = "--";
                TxtRate.Text = "-- Hz";
                if (LedStatus.Background != Brushes.OrangeRed)
                {
                    LedStatus.Background = Brushes.Gray;
                    TxtStatus.Text = "ADS_STATUS = --（无数据：主串口未连接 / 固件未跑 AdcTask）";
                    TxtStatus.Foreground = Brushes.Gray;
                }
                return;
            }

            double age = (DateTime.Now - _lastSampleAt).TotalSeconds;
            TxtAge.Text = $"{age:F1}s 前";
            if (age > 1.5)
            {
                // 采集中断：清状态灯，但**保留最后电压**并显式标注，避免被当成"压力变成 0"
                LedStatus.Background = Brushes.Gray;
                TxtStatus.Text = $"ADS_STATUS = --（{age:F0}s 无数据：串口断开 / 固件卡住）";
                TxtStatus.Foreground = Brushes.Firebrick;
                TxtRate.Text = "-- Hz";
            }
        }

        private const string CsvHeader = "日期,时间,通道,电压(V),原始码,压力(kPa),ADS_STATUS";

        private void RecTimer_Tick(object? sender, EventArgs e)
        {
            if (CkRec.IsChecked != true) return;
            if (!_hasData || _last.Volts == null) return;

            try
            {
                string path = Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "ads.csv");
                bool needHeader = !File.Exists(path) || new FileInfo(path).Length == 0;
                using var sw = new StreamWriter(path, append: true, new UTF8Encoding(true));
                if (needHeader) sw.WriteLine(CsvHeader);

                var now = DateTime.Now;
                for (int i = 0; i < 4; i++)
                {
                    sw.WriteLine(string.Join(",",
                        now.ToString("yyyy-MM-dd"),
                        now.ToString("HH:mm:ss"),
                        $"AIN{i}",
                        _last.Volts[i].ToString("F5"),
                        _last.Raw[i].ToString(),
                        Pressure(i, _last.Volts[i]).ToString("F2"),
                        _last.Status.ToString()));
                }
                TxtRecStatus.Text = $"记录中 · {now:HH:mm:ss}（ads.csv）";
                TxtRecStatus.Foreground = Brushes.Green;
            }
            catch (Exception ex)
            {
                // 先停止记录（Unchecked 会改状态文字），再把错误写回去，否则错误提示会被覆盖
                CkRec.IsChecked = false;
                TxtRecStatus.Text = "写入失败：" + ex.Message;
                TxtRecStatus.Foreground = Brushes.Firebrick;
            }
        }

        private void CkRec_Checked(object sender, RoutedEventArgs e)
        {
            TxtRecStatus.Text = "记录中…（每次追加 4 行 / 秒）";
            TxtRecStatus.Foreground = Brushes.Green;
        }

        private void CkRec_Unchecked(object sender, RoutedEventArgs e)
        {
            TxtRecStatus.Text = "已停止记录";
            TxtRecStatus.Foreground = Brushes.Gray;
        }

        private void BtnOpenDir_Click(object sender, RoutedEventArgs e)
        {
            try { Process.Start("explorer.exe", AppDomain.CurrentDomain.BaseDirectory); }
            catch (Exception ex)
            {
                TxtRecStatus.Text = "打开文件夹失败：" + ex.Message;
                TxtRecStatus.Foreground = Brushes.Firebrick;
            }
        }

        // ==================== 日志 ====================

        private void Log(string msg)
        {
            TxtLog.AppendText($"[{DateTime.Now:HH:mm:ss.fff}] {msg}\n");
            _logLines++;
            if (_logLines > 200)
            {
                var text = TxtLog.Text;
                int keep = text.LastIndexOf('\n', text.Length / 2);
                if (keep > 0) TxtLog.Text = text.Substring(keep + 1);
                _logLines = TxtLog.Text.Count(c => c == '\n');
            }
            TxtLog.ScrollToEnd();
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            _uiTimer.Stop();
            _recTimer.Stop();
            base.OnClosing(e);
        }
    }
}
