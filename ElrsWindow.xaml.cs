using System.IO.Ports;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;

namespace GamepadSpeedController
{
    /// <summary>
    /// ELRS 手柄（RadioMaster Pocket 遥控器 + ELRS 2.4G 接收机）的独立控制窗口。
    ///
    /// 从 MainWindow 分离出来：把测试/连接/轮询/失联保护全部收敛到本窗口，
    /// 日志直接写入本窗口的 TxtLog，不再回灌 MainWindow 的调试日志区。
    ///
    /// 与 MainWindow 的耦合通过 IElrsHost 注入：
    ///   - 共用速度命令缓冲区（_cmdVx/Vy/Wz + _cmdHeartbeat + _cmdLock）
    ///   - 失联时回调 MainWindow 急停
    ///   - CH6 上升沿回调 MainWindow 使能切换
    ///   - CH5 三段开关同步档位
    /// </summary>
    public partial class ElrsWindow : Window
    {
        // ====== 由 MainWindow 注入的宿主接口 ======
        // 注入前为 null，所有交互都会被静默忽略（不抛异常）。
        public IElrsHost? Host { get; set; }

        // ====== 波特率候选 ======
        // CRSF 标准是 420000；其余档位用于排查"适配器能不能真做出 420000"。
        // ⚠️ CP2102/CP2103 只有固定档位表且不含 420000 → 芯片会把 420000 映射到 460800（快 9.71%），
        //    用这类芯片时无论请求 420000 还是 460800，实际都是 460800。
        private static readonly int[] BaudCandidates = { 420000, 460800, 921600, 400000, 230400, 115200, 500000 };
        private const int DefaultCrsfBaud = 420000;

        // ====== ELRS 自身状态（本窗口独占，不与 MainWindow 共享字段） ======
        private CrsfReader? _crsf;
        private bool _crsfEnabled;
        private bool _crsfSwitchArm_prev;       // CH6 二段开关边沿检测（使能切换）
        private CancellationTokenSource? _crsfCts;

        /// <summary>取端口下拉的波特率（可手输）；非法则回退 420000。</summary>
        private int GetSelBaud()
        {
            if (CbCrsfBaud.SelectedItem is int b) return b;
            var txt = CbCrsfBaud.Text;
            return int.TryParse(txt, out var v) && v > 0 ? v : DefaultCrsfBaud;
        }

        // 与 XInput 手柄互斥由 Host.OnElrsEnabled 回调 MainWindow 关闭 XInput 实现。

        public ElrsWindow()
        {
            InitializeComponent();
            Loaded += ElrsWindow_Loaded;
        }

        private void ElrsWindow_Loaded(object sender, RoutedEventArgs e)
        {
            // 复用系统串口列表；端口下拉在窗口打开时刷新一次
            CbCrsfPort.ItemsSource = SerialPort.GetPortNames().OrderBy(p => p).ToList();
            if (CbCrsfPort.Items.Count > 0) CbCrsfPort.SelectedIndex = 0;

            // 波特率下拉（可手输）：默认 CRSF 标准 420000
            CbCrsfBaud.ItemsSource = BaudCandidates.ToList();
            CbCrsfBaud.SelectedItem = DefaultCrsfBaud;

            // 初始刷新电机状态显示
            RefreshMotorState();
        }

        // ==================== 电机控制（仅急停按钮；使能/档位由遥控器 CH5/CH6 自动控制） ====================

        private void BtnEStop_Click(object sender, RoutedEventArgs e)
        {
            Host?.TriggerEStop();
            RefreshMotorState();
        }

        /// <summary>
        /// 从 MainWindow 拉取电机使能状态 + 当前档位，刷新 UI 显示。
        /// CH6 上升沿触发 / CH5 档位变化 / 急停 后调用。
        /// </summary>
        private void RefreshMotorState()
        {
            if (Host == null) return;
            bool enabled = Host.IsMotorsEnabled;
            TxtMotorState.Text = enabled ? "已使能" : "失能";
            TxtMotorState.Foreground = enabled
                ? System.Windows.Media.Brushes.Green
                : System.Windows.Media.Brushes.Gray;

            string[] gearNames = { "慢速", "中速", "快速" };
            int gearIdx = (int)Host.CurrentGear;
            TxtGear.Text = gearIdx >= 0 && gearIdx < gearNames.Length ? gearNames[gearIdx] : "--";
        }

        // ==================== 启用 / 停用 ====================

        /// <summary>
        /// 供外部（如 MainWindow.DoEStop）停用 ELRS：等效于取消勾选启用复选框。
        /// 幂等：已停用时为 no-op。用于急停时同步中断 ELRS 轮询，避免心跳被下一帧复活。
        /// </summary>
        public void DisableElrs()
        {
            if (ChkCrsfEnable.IsChecked == true)
                ChkCrsfEnable.IsChecked = false;
        }

        private void ChkCrsfEnable_Checked(object sender, RoutedEventArgs e)
        {
            if (Host == null || !Host.IsModbusConnected)
            {
                ChkCrsfEnable.IsChecked = false;
                Log("请先在主窗口连接 STM32 主串口（C 板）后再启用 ELRS。");
                MessageBox.Show(this, "请先在主窗口连接 STM32 主串口（C 板）后再启用 ELRS。",
                    "未连接", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            if (CbCrsfPort.SelectedItem is not string portName || string.IsNullOrEmpty(portName))
            {
                ChkCrsfEnable.IsChecked = false;
                Log("请选择 ELRS 接收机所在串口。");
                MessageBox.Show(this, "请选择 ELRS 接收机所在串口。",
                    "未选端口", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            // 说明：不再限制 ELRS 端口必须与主串口不同（可能只接一个适配器）。
            // 若主串口实际已打开且占用同一 COM，Windows 串口独占机制会让 new CrsfReader 抛异常，
            // 由下面的 catch 统一提示"端口被占用"。
            try
            {
                int baud = GetSelBaud();
                _crsf = new CrsfReader(portName, baud);
                _crsfEnabled = true;
                // 通知 MainWindow：关闭 XInput + 开启心跳（C 板速度命令 500ms 无更新归零）
                Host.OnElrsEnabled();
                TxtLinkState.Text = "已启用";
                TxtLinkState.Foreground = System.Windows.Media.Brushes.Green;
                Log($"ELRS 已启用：{portName} @ {baud} 8N1（帧头 0xC8，CRC-8 poly=0xD5）");
                if (baud != DefaultCrsfBaud)
                    Log($"⚠️ 波特率不是 CRSF 标准的 {DefaultCrsfBaud}：能连上不代表数据对，请看“测试链路”的 CRC 结果。");
                if (!portName.StartsWith("COM", StringComparison.OrdinalIgnoreCase))
                    Log("⚠️ 端口名看起来不是 COM 口，请确认选对了串口。");

                // 启动独立的 CRSF 轮询线程（与 MainWindow.CommLoop 解耦，避免 USB 资源竞争）
                _crsfCts = new CancellationTokenSource();
                Task.Run(() => CrsfPollLoop(_crsfCts.Token), _crsfCts.Token);
            }
            catch (Exception ex)
            {
                ChkCrsfEnable.IsChecked = false;
                _crsfEnabled = false;
                _crsf?.Dispose();
                _crsf = null;
                Log($"[串口错误] 打开 ELRS 串口失败：{ex.Message}\n" +
                    "可能原因：端口被占用 / 端口选错 / USB-TTL 驱动未装 / USB-TTL 是 RS232 电平版本（需 3.3V TTL）");
                MessageBox.Show(this, $"打开 ELRS 串口失败：{ex.Message}\n\n" +
                                 "提示：ELRS 接收机对外输出波特率固定 420000（CRSF 协议要求），" +
                                 "若 USB-TTL 不支持 420000 会打不开。",
                    "串口错误", MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void ChkCrsfEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            _crsfEnabled = false;
            // 先停轮询线程（让它自然退出），再 Dispose _crsf，避免竞态
            _crsfCts?.Cancel();
            Thread.Sleep(20);
            _crsfCts?.Dispose();
            _crsfCts = null;
            _crsf?.Dispose();
            _crsf = null;

            // 通知 MainWindow：清速度命令 + 关心跳
            Host?.OnElrsDisabled();

            TxtCrsfRssi.Text = "-- dBm";
            TxtCrsfRssi.Foreground = System.Windows.Media.Brushes.Gray;
            TxtLinkState.Text = "未启用";
            TxtLinkState.Foreground = System.Windows.Media.Brushes.Gray;
            _crsfSwitchArm_prev = false;
            Log("ELRS 已停用，速度命令已清零。");
        }

        // ==================== 链路自检（3 秒） ====================

        /// <summary>
        /// ELRS 链路自检：3 秒内统计字节数 / 0xC8 帧头数 / 有效帧数 / 通道值。
        /// 复用已打开的 _crsf（若启用 ELRS），否则临时开一个串口跑 3 秒后关闭。
        /// 结果写入本窗口日志区 + 弹窗，方便启用前确认硬件链路正常。
        /// </summary>
        private async void BtnCrsfTest_Click(object sender, RoutedEventArgs e)
        {
            if (CbCrsfPort.SelectedItem is not string portName || string.IsNullOrEmpty(portName))
            {
                Log("请先选择 ELRS 端口。");
                MessageBox.Show(this, "请先在端口下拉里选一个 COM 号。",
                    "未选端口", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            BtnCrsfTest.IsEnabled = false;
            BtnCrsfTest.Content = "测试中...";

            int baud = GetSelBaud();
            bool ownsReader = false;
            CrsfReader? r = _crsf;
            if (r == null)
            {
                try
                {
                    r = new CrsfReader(portName, baud);
                    ownsReader = true;
                }
                catch (Exception ex)
                {
                    Log($"[串口错误] 打开失败：{ex.Message}\n" +
                        "可能：端口被占用 / 端口选错 / USB-TTL 驱动未装 / 是 RS232 电平版本（需 3.3V TTL）");
                    MessageBox.Show(this, $"打开串口失败：{ex.Message}", "串口错误",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                    BtnCrsfTest.IsEnabled = true;
                    BtnCrsfTest.Content = "测试链路";
                    return;
                }
            }
            else
            {
                // 复用已打开的端口：波特率以端口实际值为准（下拉此时改不了已开的端口）
                baud = r.BaudRate;
                Log($"（复用已启用的 ELRS 端口，实测波特率设置 = {baud}）");
            }

            // 重置统计
            r.ResetStats();

            DateTime start = DateTime.UtcNow;
            await Task.Run(() =>
            {
                while ((DateTime.UtcNow - start).TotalMilliseconds < 3000)
                {
                    r.Poll();
                    Thread.Sleep(5);
                }
            });

            // ---- 波形分析：把"收到的到底是什么"变成结论 ----
            var raw = r.RawSnapshot(512);
            double bps = r.TotalBytes / 3.0;
            string blockHex = FindPeriod(raw, out int period, out double periodMatch);
            string histo = ByteHistogram(raw, 6);
            TopByte(raw, out byte topByte, out double topShare);

            var sb = new StringBuilder();
            sb.AppendLine($"===== ELRS 自检结果（{portName} @ {baud} 8N1，3 秒）=====");
            sb.AppendLine($"  总字节:      {r.TotalBytes}   （{bps:F0} B/s）");
            sb.AppendLine($"  0xC8 帧头:   {r.SyncBytes}");
            sb.AppendLine($"  有效帧:      {r.ValidFrames}（通过 CRC 校验）");
            sb.AppendLine($"  坏帧:        {r.BadFrames}（CRC 失败）");
            sb.AppendLine($"  最近坏帧:    len=0x{r.LastBadFrameLen:X2}  type=0x{r.LastBadFrameType:X2}");
            sb.AppendLine($"  RSSI:        {(r.RssiDbm == 0 ? "未知/0" : r.RssiDbm + " dBm")}");
            sb.AppendLine($"  上行LQ:      {(r.LinkQuality == 0 ? "未知/0" : r.LinkQuality + " %")}");
            sb.AppendLine();
            sb.AppendLine("  ── 节奏核对（CRSF 500Hz：26 B/帧 ≈ 13000 B/s）──");
            sb.AppendLine($"  实测 {bps:F0} B/s = 期望 13000 B/s 的 {bps / 13000.0:F2} 倍");
            sb.AppendLine($"  实测 {bps:F0} B/s = 420000 满速(42000 B/s) 的 {bps / 42000.0:P1}");
            if (raw.Length > 0)
            {
                sb.AppendLine($"  精确周期:    {period} 字节（逐字节相似度 {periodMatch:P1}）");
                sb.AppendLine($"  主循环块:    {blockHex}");
                sb.AppendLine($"  最常见字节:  0x{topByte:X2} 占 {topShare:P1}");
                sb.AppendLine($"  直方图 Top:  {histo}");
            }
            sb.AppendLine();
            sb.AppendLine("  ── 原始字节 dump（最近 128 字节）──");
            sb.AppendLine(raw.Length == 0 ? "  （无数据）" : r.DumpRaw(128));
            sb.AppendLine("  ────────────────────────────");
            sb.AppendLine("  通道值（-1..+1，中点 0）：");
            for (int i = 0; i < 11; i++)
                sb.AppendLine($"    CH{i + 1,-2} = {r.Channels[i],8:F3}");
            sb.AppendLine();

            // ---- 结论：先判"节奏"，再判"值" ----
            string verdict;
            MessageBoxImage icon;
            bool rhythmLikeCrsf = (period >= 20 && period <= 34 && periodMatch > 0.85)
                                  || Math.Abs(bps - 13000) / 13000.0 < 0.25;

            if (r.TotalBytes == 0)
            {
                verdict = "❌ 完全没收到字节\n\n可能原因：\n" +
                          "  1. USB-TTL TX/RX 接反了（要交叉：ELRS.TX → 转换器.RX）\n" +
                          "  2. ELRS 接收机未与遥控器对频（灯应常亮）\n" +
                          "  3. USB-TTL 是 RS232 电平版本（不是 3.3V TTL）\n" +
                          "  4. 接收机没供电 / 共地没接";
                icon = MessageBoxImage.Error;
            }
            else if (r.ValidFrames > 0)
            {
                verdict = $"✅ 链路正常：{r.ValidFrames} 帧 / 3 秒（{r.ValidFrames / 3.0:F1} Hz，期望 50~500 Hz）";
                icon = MessageBoxImage.Information;
            }
            else if (topShare > 0.5 && (topByte == 0xFF || topByte == 0x00))
            {
                verdict = $"❌ 收到 {r.TotalBytes} 字节，但 {topShare:P0} 都是 0x{topByte:X2} —— 线上没有数据源\n\n" +
                          "含义：接收机 TX 没在驱动这条线（悬空 / 掉电 / 没输出），UART 只是在收空闲电平或噪声。\n" +
                          "检查：ELRS.TX 是否真接到模块的 RX；接收机供电（5V/3V3，别从模块 3V3 取电）与共地一根线。";
                icon = MessageBoxImage.Error;
            }
            else if (rhythmLikeCrsf)
            {
                verdict = $"❌ 节奏像 CRSF（约 {bps:F0} B/s，{period} 字节周期），但 0 个有效帧、0xC8 只有 {r.SyncBytes} 个\n\n" +
                          "这是【端口实际波特率 ≠ 信号波特率】的典型形态：\n" +
                          "  端口比信号快时，仍会按真实起始位沿对齐（所以字节率/周期/帧长看起来都正常），\n" +
                          "  但字节内部 8 个采样点被压缩 → 每个字节的值都错、全篇找不到 0xC8。\n\n" +
                          "⚠️ 已知坑：CP2102/CP2103 做不出 420000，芯片会把请求映射到最近的支持档\n" +
                          "   = 460800（快 +9.71%），此时 420000 与 460800 的结果会完全一样。\n" +
                          "  → 点【扫描波特率】看哪一档能出有效帧；都不行就换 CP2104/CP2105/CP2102N/FT232/CH340。\n" +
                          "  （SerialPort.BaudRate 只回显请求值，不能作证据；回环自测也测不出波特率偏差。）";
                icon = MessageBoxImage.Warning;
            }
            else if (r.SyncBytes == 0)
            {
                verdict = $"❌ 收到 {r.TotalBytes} 字节但无 0xC8 帧头\n\n不是 CRSF 协议数据。可能：\n" +
                          "  - ELRS 接收机协议不是 CRSF（去 ELRS Lua 脚本里改 Receiver Protocol → CRSF）\n" +
                          "  - 接线不对（确认 ELRS.TX → 转换器.RX）\n" +
                          "  - 波特率不对（见上方“节奏核对”，或用【扫描波特率】对比）";
                icon = MessageBoxImage.Error;
            }
            else
            {
                var badHex = new StringBuilder();
                for (int i = 0; i < Math.Min(r.LastBadFrameLen + 2, r.LastBadFrame.Length); i++)
                    badHex.Append($"{r.LastBadFrame[i]:X2} ");
                verdict = $"❌ 找到 {r.SyncBytes} 个 0xC8、{r.BadFrames} 个 CRC 失败、0 个有效\n\n" +
                          $"最近坏帧：len=0x{r.LastBadFrameLen:X2}（通道帧应为 0x18/24）  " +
                          $"type=0x{r.LastBadFrameType:X2}（通道帧应为 0x16）\n" +
                          $"完整帧：{badHex}\n\n" +
                          "可能：波特率不对 / 信号干扰导致字节翻转 / 帧被截断。\n" +
                          "先看上方“节奏核对”“精确周期”“直方图”，再点【扫描波特率】对比。";
                icon = MessageBoxImage.Warning;
            }
            sb.AppendLine(verdict);

            Log(sb.ToString());
            MessageBox.Show(this, sb.ToString(), "ELRS 自检", MessageBoxButton.OK, icon);

            if (ownsReader) r.Dispose();

            BtnCrsfTest.IsEnabled = true;
            BtnCrsfTest.Content = "测试链路";
        }

        // ==================== 波特率扫描 ====================

        /// <summary>
        /// 波特率扫描：依次用候选波特率各收 0.4 秒，报"总字节 / 0xC8 / 有效帧 / 坏帧"。
        /// 用途：一眼判断"这块适配器到底能不能做出信号所需的波特率"——
        ///   正确档位会出现 ~100 个有效帧；若 420000 与 460800 结果完全相同 ⇒ 芯片把两者当同一档
        ///   （CP2102/CP2103 的映射行为）；若所有档位都 0 有效帧 ⇒ 该适配器做不出，需换芯片。
        /// 前提：先取消勾选"启用 ELRS"（扫描要逐个打开端口）。
        /// </summary>
        private async void BtnCrsfScan_Click(object sender, RoutedEventArgs e)
        {
            if (CbCrsfPort.SelectedItem is not string portName || string.IsNullOrEmpty(portName))
            {
                MessageBox.Show(this, "请先选端口。", "未选端口", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            if (_crsfEnabled || _crsf != null)
            {
                MessageBox.Show(this, "请先取消勾选“启用 ELRS”再扫描（扫描要逐个打开端口）。",
                    "端口被占用", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }

            BtnCrsfTest.IsEnabled = false;
            BtnCrsfScan.IsEnabled = false;
            BtnCrsfScan.Content = "扫描中...";

            string report = await Task.Run(() =>
            {
                var sb = new StringBuilder();
                sb.AppendLine($"===== 波特率扫描（{portName}，每档 0.4 秒）=====");
                sb.AppendLine("  波特率     总字节   0xC8   有效帧   坏帧      B/s   说明");
                foreach (int b in BaudCandidates)
                {
                    try
                    {
                        using var rr = new CrsfReader(portName, b);
                        rr.ResetStats();
                        var t0 = DateTime.UtcNow;
                        while ((DateTime.UtcNow - t0).TotalMilliseconds < 400)
                        {
                            rr.Poll();
                            Thread.Sleep(4);
                        }
                        double bps = rr.TotalBytes / 0.4;
                        string note = rr.ValidFrames > 0
                            ? "✅ 有有效帧，这一档可用"
                            : rr.TotalBytes == 0 ? "没收到任何字节"
                            : "有字节但 CRC 全失败";
                        sb.AppendLine($"  {b,-9} {rr.TotalBytes,8} {rr.SyncBytes,7} {rr.ValidFrames,8} {rr.BadFrames,7} {bps,8:F0}   {note}");
                    }
                    catch (Exception ex)
                    {
                        sb.AppendLine($"  {b,-9} 打开/读取失败：{ex.Message}");
                    }
                }
                sb.AppendLine();
                sb.AppendLine("判读要点：");
                sb.AppendLine("  · CRSF 500Hz 时正确档位应出现约 100 个有效帧 / 0.4 s；");
                sb.AppendLine("  · 若 420000 与 460800 两行结果完全一样（总字节/0xC8 相同）");
                sb.AppendLine("    ⇒ 芯片把 420000 映射到了 460800（CP2102/CP2103 的档位表行为），这块适配器用不了；");
                sb.AppendLine("  · 所有档位都 0 有效帧 ⇒ 换 CP2104 / CP2105 / CP2102N / FT232 / CH340；");
                sb.AppendLine("  · 注意：SerialPort.BaudRate 只会回显请求值，不能作为“波特率生效”的证据。");
                return sb.ToString();
            });

            Log(report);
            MessageBox.Show(this, report, "波特率扫描", MessageBoxButton.OK, MessageBoxImage.Information);

            BtnCrsfTest.IsEnabled = true;
            BtnCrsfScan.IsEnabled = true;
            BtnCrsfScan.Content = "扫描波特率";
        }

        // ==================== 原始波形分析工具 ====================

        /// <summary>
        /// 找最小周期（1..64 字节）并返回主循环块的 hex。
        /// period = 命中的最小周期；match = 该周期的逐字节相同率（1.0 表示严格周期）。
        /// CRSF 500Hz 的"节奏对但值全错"会表现为 period=26、match≈1.0。
        /// </summary>
        private static string FindPeriod(byte[] raw, out int period, out double match)
        {
            period = 0; match = 0;
            if (raw.Length < 8) return "（数据不足）";
            for (int p = 1; p <= Math.Min(64, raw.Length / 2); p++)
            {
                int same = 0, tot = 0;
                for (int i = 0; i + p < raw.Length; i++) { tot++; if (raw[i] == raw[i + p]) same++; }
                if (tot <= 0) continue;
                double r = (double)same / tot;
                if (r > match + 1e-9) { match = r; period = p; }   // 取"最小的"那个最大相似周期
            }
            if (period == 0 || match < 0.5) { period = 0; return "（无明显周期）"; }
            var sb = new StringBuilder();
            for (int i = 0; i < Math.Min(period, raw.Length); i++) sb.Append(raw[i].ToString("X2")).Append(' ');
            return sb.ToString().TrimEnd();
        }

        /// <summary>字节直方图 TopN，返回形如 "0xFF×320  0x00×12 …"。</summary>
        private static string ByteHistogram(byte[] raw, int topN)
        {
            if (raw.Length == 0) return "（无数据）";
            var cnt = new int[256];
            foreach (var b in raw) cnt[b]++;
            var sb = new StringBuilder();
            foreach (var x in Enumerable.Range(0, 256).Select(i => new { B = i, N = cnt[i] })
                                        .Where(x => x.N > 0).OrderByDescending(x => x.N).Take(topN))
                sb.Append($"0x{x.B:X2}×{x.N}  ");
            return sb.ToString().TrimEnd();
        }

        /// <summary>出现次数最多的字节及其占比（用于判"一片 0xFF/0x00 = 线上没数据源"）。</summary>
        private static void TopByte(byte[] raw, out byte top, out double share)
        {
            top = 0; share = 0;
            if (raw.Length == 0) return;
            var cnt = new int[256];
            foreach (var b in raw) cnt[b]++;
            int best = 0;
            for (int i = 1; i < 256; i++) if (cnt[i] > cnt[best]) best = i;
            top = (byte)best;
            share = (double)cnt[best] / raw.Length;
        }

        // ==================== CRSF 轮询循环（独立后台线程） ====================

        /// <summary>
        /// 独立的 CRSF 轮询循环（后台线程），与 MainWindow.CommLoop 解耦：
        /// ELRS 420000 baud 高速接收可能抢占 USB 带宽，放独立线程避免和 Modbus 互相干扰。
        /// 5ms tick 拉取 USB 驱动 buffer（500Hz 下每 5ms 约 2 帧 × 26 字节）。
        /// </summary>
        private void CrsfPollLoop(CancellationToken token)
        {
            while (!token.IsCancellationRequested && _crsf != null)
            {
                try
                {
                    _crsf.Poll();
                }
                catch (Exception ex) { Log($"ELRS 读错误: {ex.Message}"); continue; }

                try
                {
                    // ---- 失联保护 ----
                    if (_crsf.IsLinkLost)
                    {
                        Log($"[失联] RSSI={_crsf.RssiDbm}dBm，1 秒无 CRSF 帧 → 触发急停并停用 ELRS");
                        Dispatcher.BeginInvoke(new Action(() =>
                        {
                            Host?.TriggerEStop();
                            ChkCrsfEnable.IsChecked = false;
                        }));
                        return;
                    }

                    // 通道读取（Channels 是 volatile 数组，多线程可见）
                    float ch1 = _crsf.Channels[0];  // 右X → wz
                    float ch3 = _crsf.Channels[2];  // 左Y → vy
                    float ch4 = -_crsf.Channels[3]; // 左X → vx（ELRS Pocket 方向相反，取反）
                    float ch5 = _crsf.Channels[4];  // 三段开关 → 档位
                    float ch6 = _crsf.Channels[5];  // 二段开关 → 使能

                    // ---- CH5 三段开关 → 档位（自动，无需手动覆盖）----
                    SpeedGear newGear = ch5 > 0.5f ? SpeedGear.Fast
                                      : ch5 < -0.5f ? SpeedGear.Slow
                                      : SpeedGear.Mid;

                    // ---- CH6 二段开关 → 使能切换（上升沿触发）----
                    bool swArm = ch6 > 0;
                    bool needEnableToggle = swArm && !_crsfSwitchArm_prev;
                    _crsfSwitchArm_prev = swArm;

                    // ---- 摇杆映射到车体速度（vx=右+, vy=前+, wz=逆时针+）----
                    var (vx, vy, wz) = MotionMapper.Map(leftY: ch3, leftX: ch4, rightX: ch1, newGear);

                    // ---- 写入 MainWindow 共享速度命令缓冲区（线程安全，后台线程直调）----
                    Host?.SetVelocity(vx, vy, wz);

                    // ---- 所有 UI 操作 + MainWindow 状态更新，统一切回 UI 线程 ----
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        // CH5 档位：必须在 UI 线程调 MainWindow.SetGear（它操作 CbGear.SelectedIndex）
                        Host?.SetGear(newGear);
                        string[] gearNames = { "慢速", "中速", "快速" };
                        TxtGear.Text = gearNames[(int)newGear];
                        TxtGearSource.Text = "(CH5 自动)";

                        // CH6 使能切换：MainWindow.DoEnableToggle 需要在 UI 线程
                        if (needEnableToggle)
                        {
                            Host?.TriggerEnableToggle();
                            RefreshMotorState();
                        }

                        // 摇杆 + 通道 + RSSI 显示
                        PbLY.Value = ch3;
                        PbLX.Value = ch4;
                        PbRX.Value = ch1;
                        TxtVy.Text = vy.ToString("F2");
                        TxtVx.Text = vx.ToString("F2");
                        TxtWz.Text = wz.ToString("F2");

                        TxtCh1.Text = ch1.ToString("F3");
                        TxtCh2.Text = _crsf.Channels[1].ToString("F3");
                        TxtCh3.Text = ch3.ToString("F3");
                        TxtCh4.Text = _crsf.Channels[3].ToString("F3"); // 原始 CRSF 值（未取反）
                        TxtCh5.Text = ch5.ToString("F3");
                        TxtCh6.Text = ch6.ToString("F3");
                        TxtCh7.Text = _crsf.Channels[6].ToString("F3");
                        TxtChRest.Text = string.Join(" ",
                            Enumerable.Range(7, 4).Select(i => _crsf.Channels[i].ToString("F2")));

                        int rssi = _crsf.RssiDbm;
                        TxtCrsfRssi.Text = rssi == 0 ? "-- dBm" : $"{rssi} dBm";
                        TxtCrsfRssi.Foreground = rssi == 0 || rssi < CrsfReader.RssiThreshold
                            ? System.Windows.Media.Brushes.OrangeRed
                            : rssi < -80 ? System.Windows.Media.Brushes.Goldenrod
                            : System.Windows.Media.Brushes.Green;
                    }));
                }
                catch (Exception ex)
                {
                    Log($"CrsfPollLoop 异常（已忽略）: {ex.Message}");
                }

                Thread.Sleep(5);
            }
        }

        // ==================== 日志 ====================

        private int _logLines;
        private void Log(string msg)
        {
            Dispatcher.BeginInvoke(new Action(() =>
            {
                var line = $"[{DateTime.Now:HH:mm:ss.fff}] {msg}\n";
                TxtLog.AppendText(line);
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
            // 同步回灌主窗口调试日志（便于集中排查）
            Host?.Log(msg);
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            // 窗口关闭前先停用 ELRS，释放串口
            if (_crsfEnabled)
            {
                ChkCrsfEnable.IsChecked = false;
            }
            base.OnClosing(e);
        }
    }

    /// <summary>
    /// ElrsWindow 与 MainWindow 之间的解耦接口。
    /// MainWindow 实现此接口并注入到 ElrsWindow，让 ELRS 能下发速度命令、
    /// 触发急停/使能切换、同步档位，同时保持对共享命令缓冲区的互斥访问。
    /// </summary>
    public interface IElrsHost
    {
        /// <summary>主串口（RS485→C板）是否已连接。</summary>
        bool IsModbusConnected { get; }

        /// <summary>电机当前是否已使能（用于 ElrsWindow 状态显示）。</summary>
        bool IsMotorsEnabled { get; }

        /// <summary>当前档位（用于 ElrsWindow 档位显示 + 覆盖 CH5）。</summary>
        SpeedGear CurrentGear { get; }

        /// <summary>写入共享速度命令缓冲区（加锁 + 置心跳），由 CommLoop 下发。</summary>
        void SetVelocity(float vx, float vy, float wz);

        /// <summary>ELRS 启用回调：关闭 XInput 手柄互斥 + 开启心跳。</summary>
        void OnElrsEnabled();

        /// <summary>ELRS 停用回调：清零速度命令 + 关闭心跳。</summary>
        void OnElrsDisabled();

        /// <summary>失联时触发 MainWindow 急停（清速度 + 急停电机）。</summary>
        void TriggerEStop();

        /// <summary>CH6 上升沿触发使能切换 / ElrsWindow 使能按钮点击。</summary>
        void TriggerEnableToggle();

        /// <summary>CH5 三段开关同步档位（慢/中/快）。</summary>
        void SetGear(SpeedGear gear);

        /// <summary>ElrsWindow 档位手动选择（覆盖 CH5 自动更新）。</summary>
        void OverrideGear(SpeedGear gear);

        /// <summary>回灌一条日志到 MainWindow 调试日志区。</summary>
        void Log(string msg);
    }
}
