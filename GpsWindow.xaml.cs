using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace GamepadSpeedController
{
    /// <summary>
    /// GPS 定位（NEO-6M / 7N / M8N 等 NMEA 0183 模块）的独立窗口。
    ///
    /// 与 ElrsWindow 的区别：GPS 是**只读传感器**，不参与电机控制，也不通过 Modbus 下发任何东西，
    /// 因此不需要宿主接口注入。端口也不做 COM 号冲突限制（可能只接一个适配器）；
    /// 若端口被主串口实际占用，SerialPort.Open 会抛异常并由错误处理统一提示。
    ///
    /// 结构：串口配置 + 定位状态 + 位置/时间 + 卫星 SNR + 原始 NMEA + 集成日志。
    /// 读取线程与 MainWindow.CommLoop 解耦（独立线程 + 独立串口句柄）。
    /// </summary>
    public partial class GpsWindow : Window
    {
        private NmeaReader? _gps;
        private bool _gpsEnabled;
        private CancellationTokenSource? _gpsCts;

        public GpsWindow()
        {
            InitializeComponent();
            Loaded += GpsWindow_Loaded;
        }

        private void GpsWindow_Loaded(object sender, RoutedEventArgs e)
        {
            CbGpsPort.ItemsSource = SerialPort.GetPortNames().OrderBy(p => p).ToList();
            if (CbGpsPort.Items.Count > 0) CbGpsPort.SelectedIndex = 0;

            // NEO-6M / M8N 出厂默认 9600；被刷过固件或改过配置的模块常见 38400 / 115200
            CbGpsBaud.ItemsSource = new[] { 4800, 9600, 19200, 38400, 57600, 115200 };
            CbGpsBaud.SelectedItem = NmeaReader.DefaultBaud;
        }

        // ==================== 启用 / 停用 ====================

        private void ChkGpsEnable_Checked(object sender, RoutedEventArgs e)
        {
            if (CbGpsPort.SelectedItem is not string portName || string.IsNullOrEmpty(portName))
            {
                ChkGpsEnable.IsChecked = false;
                Log("请先选择 GPS 模块所在串口。");
                MessageBox.Show(this, "请先在端口下拉里选一个 COM 号。",
                    "未选端口", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            int baud = CbGpsBaud.SelectedItem is int b ? b : NmeaReader.DefaultBaud;

            try
            {
                _gps = new NmeaReader(portName, baud);
                _gpsEnabled = true;
                TxtGpsState.Text = "读取中";
                TxtGpsState.Foreground = System.Windows.Media.Brushes.Green;
                Log($"GPS 已启用：{portName} @ {baud} 8N1（NMEA 0183，按后缀匹配 GGA/RMC/GSA/GSV）");

                _gpsCts = new CancellationTokenSource();
                Task.Run(() => GpsPollLoop(_gpsCts.Token), _gpsCts.Token);
            }
            catch (Exception ex)
            {
                ChkGpsEnable.IsChecked = false;
                _gpsEnabled = false;
                _gps?.Dispose();
                _gps = null;
                Log($"[串口错误] 打开 GPS 串口失败：{ex.Message}\n" +
                    "可能原因：端口被占用 / 端口选错 / USB-TTL 驱动未装 / 波特率不被适配器支持");
                MessageBox.Show(this, $"打开 GPS 串口失败：{ex.Message}",
                    "串口错误", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void ChkGpsEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            _gpsEnabled = false;
            _gpsCts?.Cancel();
            Thread.Sleep(20);
            _gpsCts?.Dispose();
            _gpsCts = null;
            _gps?.Dispose();
            _gps = null;

            TxtGpsState.Text = "未启用";
            TxtGpsState.Foreground = System.Windows.Media.Brushes.Gray;
            Log("GPS 已停用。");
        }

        // ==================== 轮询循环（独立后台线程） ====================

        /// <summary>
        /// GPS 轮询循环：GPS 通常 1Hz 输出、9600 baud，100ms tick 足够（每次约 100 字节）。
        /// 与 ELRS 的 5ms tick 不同 —— 没必要高频空转，且低频能显著降低 USB 占用。
        /// </summary>
        private void GpsPollLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested && _gps != null)
            {
                try
                {
                    _gps.Poll();
                }
                catch (Exception ex) { Log($"GPS 读错误: {ex.Message}"); continue; }

                try
                {
                    if (_gps.IsLinkLost)
                    {
                        Log($"[掉线] {NmeaReader.LinkTimeoutMs / 1000} 秒未收到任何 NMEA 句子 → 停止读取");
                        Dispatcher.BeginInvoke(new Action(() =>
                        {
                            if (_gpsEnabled) ChkGpsEnable.IsChecked = false;
                        }));
                        return;
                    }

                    var g = _gps;
                    string lat = FormatLatLon(g.Latitude, true);
                    string lon = FormatLatLon(g.Longitude, false);
                    string fixText = g.HasFix ? "已定位" : "无定位";
                    bool fix = g.HasFix;
                    int satsUsed = g.SatellitesUsed, satsView = g.SatellitesInView;
                    double hdop = g.Hdop, alt = g.AltitudeM, spd = g.SpeedKnots, crs = g.CourseDeg;
                    double pdop = g.Pdop, vdop = g.Vdop;
                    int fixType = g.FixType, fixQuality = g.FixQuality;
                    string utcTime = g.UtcTime, utcDate = g.UtcDate;
                    string satSnr = g.GetSatSnrText();
                    string rawGga = g.GetLastByType("GGA");
                    string rawRmc = g.GetLastByType("RMC");
                    string rawGsa = g.GetLastByType("GSA");
                    string rawGsv = g.GetLastByType("GSV");

                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        TxtFix.Text = fixText;
                        TxtFix.Foreground = fix
                            ? System.Windows.Media.Brushes.Green
                            : System.Windows.Media.Brushes.OrangeRed;
                        if (fix)
                        {
                            string quality = fixQuality switch
                            {
                                1 => "GPS",
                                2 => "DGPS",
                                3 => "PPS",
                                4 => "RTK 固定",
                                5 => "RTK 浮动",
                                6 => "估算",
                                _ => $"质量{fixQuality}",
                            };
                            TxtFix.Text = $"已定位 ({quality})";
                        }

                        TxtSats.Text = $"{satsUsed} / {satsView}";
                        TxtHdop.Text = fix ? hdop.ToString("F2") : "--";
                        TxtAlt.Text = fix ? $"{alt:F1} m" : "-- m";

                        TxtLat.Text = fix ? lat : "--";
                        TxtLon.Text = fix ? lon : "--";
                        TxtUtcTime.Text = FormatUtcTime(utcTime);
                        TxtUtcDate.Text = FormatUtcDate(utcDate);
                        TxtSpeed.Text = fix ? $"{spd:F2} 节 ({spd * 1.852:F2} km/h)" : "--";
                        TxtCourse.Text = fix ? $"{crs:F1}°" : "--";
                        TxtFixType.Text = fixType switch
                        {
                            1 => "无定位",
                            2 => "2D",
                            3 => "3D",
                            _ => "--",
                        };
                        TxtDop.Text = $"{pdop:F2} / {vdop:F2}";

                        TxtSatSnr.Text = satSnr;
                        TxtRawGga.Text = "GGA: " + (rawGga.Length > 0 ? rawGga : "--");
                        TxtRawRmc.Text = "RMC: " + (rawRmc.Length > 0 ? rawRmc : "--");
                        TxtRawGsa.Text = "GSA: " + (rawGsa.Length > 0 ? rawGsa : "--");
                        TxtRawGsv.Text = "GSV: " + (rawGsv.Length > 0 ? rawGsv : "--");
                    }));
                }
                catch (Exception ex)
                {
                    Log($"GpsPollLoop 异常（已忽略）: {ex.Message}");
                }

                Thread.Sleep(100);
            }
        }

        // ==================== 连接自检（3 秒） ====================

        /// <summary>
        /// GPS 连接自检：3 秒内统计字节数 / 句子数 / 校验通过数 / 句子类型分布 / 定位状态。
        /// 复用已打开的 _gps（若已启用），否则临时开一个串口跑 3 秒后关闭。
        /// </summary>
        private async void BtnGpsTest_Click(object sender, RoutedEventArgs e)
        {
            if (CbGpsPort.SelectedItem is not string portName || string.IsNullOrEmpty(portName))
            {
                Log("请先选择 GPS 端口。");
                MessageBox.Show(this, "请先在端口下拉里选一个 COM 号。",
                    "未选端口", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            int baud = CbGpsBaud.SelectedItem is int b ? b : NmeaReader.DefaultBaud;

            BtnGpsTest.IsEnabled = false;
            BtnGpsTest.Content = "测试中...";

            bool ownsReader = false;
            NmeaReader? r = _gps;
            if (r == null)
            {
                try
                {
                    r = new NmeaReader(portName, baud);
                    ownsReader = true;
                }
                catch (Exception ex)
                {
                    Log($"[串口错误] 打开失败：{ex.Message}\n" +
                        "可能：端口被占用 / 端口选错 / USB-TTL 驱动未装 / 波特率不支持");
                    MessageBox.Show(this, $"打开串口失败：{ex.Message}", "串口错误",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                    BtnGpsTest.IsEnabled = true;
                    BtnGpsTest.Content = "测试连接";
                    return;
                }
            }

            r.ResetStats();

            DateTime start = DateTime.UtcNow;
            await Task.Run(() =>
            {
                while ((DateTime.UtcNow - start).TotalMilliseconds < 3000)
                {
                    r.Poll();
                    Thread.Sleep(20);
                }
            });

            var counts = r.GetTypeCountsSnapshot();
            var sb = new StringBuilder();
            sb.AppendLine($"===== GPS 自检结果（{portName} @ {baud}，3 秒）=====");
            sb.AppendLine($"  总字节:      {r.TotalBytes}");
            sb.AppendLine($"  NMEA 句子:   {r.TotalSentences}");
            sb.AppendLine($"  校验通过:    {r.ValidSentences}");
            sb.AppendLine($"  校验失败:    {r.BadChecksum}");
            sb.AppendLine($"  格式非法:    {r.BadFormat}");
            sb.AppendLine("  句子类型统计:");
            if (counts.Count == 0)
            {
                sb.AppendLine("    （无）");
            }
            else
            {
                foreach (var kv in counts)
                    sb.AppendLine($"    {kv.Key,-4} × {kv.Value}");
            }
            sb.AppendLine($"  定位状态:    {(r.HasFix ? "已定位" : "无定位")}" +
                          $"（GGA 质量={r.FixQuality}，GSA 类型={r.FixType}）");
            sb.AppendLine($"  卫星:        {r.SatellitesUsed} 已用 / {r.SatellitesInView} 可见");
            sb.AppendLine($"  经纬度:      {FormatLatLon(r.Latitude, true)}  {FormatLatLon(r.Longitude, false)}");
            sb.AppendLine($"  海拔/HDOP:   {r.AltitudeM:F1} m / {r.Hdop:F2}");
            sb.AppendLine($"  可见卫星 SNR: {r.GetSatSnrText()}");
            sb.AppendLine();

            string verdict;
            MessageBoxImage icon;
            if (r.TotalBytes == 0)
            {
                verdict = "❌ 完全没收到字节\n\n可能原因：\n" +
                          "  1. USB-TTL TX/RX 接反了（要交叉：GPS.TX → 转换器.RX）\n" +
                          "  2. GPS 模块未供电或 GND 未共地（通电后模块 LED 应闪烁）\n" +
                          "  3. 波特率不对（NEO-6M/M8N 默认 9600，试改 38400 / 115200）\n" +
                          "  4. USB-TTL 用错电平侧（要接 3.3V TTL 侧，不是 RS232 侧）";
                icon = MessageBoxImage.Error;
            }
            else if (r.TotalSentences == 0)
            {
                verdict = $"❌ 收到 {r.TotalBytes} 字节但没有完整行\n\n" +
                          $"说明数据流里没有换行符，通常仍是波特率不匹配。\n\n" +
                          $"── 原始字节 hex dump（前 128 字节）──\n{r.DumpRaw(128)}" +
                          $"────────────────────────────";
                icon = MessageBoxImage.Error;
            }
            else if (r.ValidSentences == 0)
            {
                verdict = $"❌ 收到 {r.TotalSentences} 行，但 0 行通过校验\n\n" +
                          $"格式非法 {r.BadFormat} 行，校验失败 {r.BadChecksum} 行。\n" +
                          $"通常为波特率不匹配导致字符错位。\n\n" +
                          $"── 原始字节 hex dump（前 128 字节）──\n{r.DumpRaw(128)}" +
                          $"────────────────────────────";
                icon = MessageBoxImage.Warning;
            }
            else if (!r.HasFix)
            {
                verdict = $"✅ 数据链路正常（{r.ValidSentences} 行 / 3 秒，校验全通过）\n" +
                          $"⚠️ 但尚未定位（无 fix）\n\n" +
                          $"已看到 {r.SatellitesInView} 颗可见卫星。室内几乎无法定位，请：\n" +
                          $"  1. 到室外或窗边天空开阔处\n" +
                          $"  2. 冷启动首次定位可能需 30~60 秒（模块需下载星历）\n" +
                          $"  3. 观察『可见卫星 SNR』数值增长，SNR ≥ 30dB 后易定位";
                icon = MessageBoxImage.Warning;
            }
            else
            {
                verdict = $"✅ 定位成功：{r.SatellitesUsed} 颗卫星参与解算，HDOP={r.Hdop:F2}\n" +
                          $"  经度 {FormatLatLon(r.Longitude, false)}，纬度 {FormatLatLon(r.Latitude, true)}";
                icon = MessageBoxImage.Information;
            }
            sb.Append(verdict);

            Log(sb.ToString());
            MessageBox.Show(this, sb.ToString(), "GPS 自检", MessageBoxButton.OK, icon);

            if (ownsReader) r.Dispose();

            BtnGpsTest.IsEnabled = true;
            BtnGpsTest.Content = "测试连接";
        }

        // ==================== 格式化工具 ====================

        /// <summary>十进制度 → "48°07.038' N" 形式（度分，GPS 常用显示）。</summary>
        private static string FormatLatLon(double deg, bool isLat)
        {
            if (Math.Abs(deg) < 1e-9) return "--";
            char hemi = isLat ? (deg >= 0 ? 'N' : 'S') : (deg >= 0 ? 'E' : 'W');
            double a = Math.Abs(deg);
            int d = (int)a;
            double m = (a - d) * 60.0;
            return $"{d}°{m:00.000}' {hemi}";
        }

        /// <summary>"123519.000" → "12:35:19"。</summary>
        private static string FormatUtcTime(string t)
        {
            if (t.Length < 6) return "--:--:--";
            return $"{t.Substring(0, 2)}:{t.Substring(2, 2)}:{t.Substring(4, 2)}";
        }

        /// <summary>"230394"(ddmmyy) → "1994-09-23"。两位年份按惯例：≥80 视为 19xx，否则 20xx。</summary>
        private static string FormatUtcDate(string d)
        {
            if (d.Length < 6
                || !int.TryParse(d.Substring(4, 2), out int yy))
                return "--";
            int year = yy >= 80 ? 1900 + yy : 2000 + yy;
            return $"{year:D4}-{d.Substring(2, 2)}-{d.Substring(0, 2)}";
        }

        // ==================== 日志 ====================

        private int _logLines;
        private void Log(string msg)
        {
            Dispatcher.BeginInvoke(new Action(() =>
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
            }));
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            // 窗口关闭前停用读取，释放串口
            if (_gpsEnabled) ChkGpsEnable.IsChecked = false;
            base.OnClosing(e);
        }
    }
}