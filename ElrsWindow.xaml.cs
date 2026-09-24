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

        // ====== ELRS 自身状态（本窗口独占，不与 MainWindow 共享字段） ======
        private CrsfReader? _crsf;
        private bool _crsfEnabled;
        private bool _crsfSwitchArm_prev;       // CH6 二段开关边沿检测（使能切换）
        private CancellationTokenSource? _crsfCts;

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

            // 关键防呆：不能跟主串口是同一个 COM 号
            if (string.Equals(portName, Host.MainPortName, StringComparison.OrdinalIgnoreCase))
            {
                ChkCrsfEnable.IsChecked = false;
                Log($"[端口冲突] ELRS 端口 {portName} 与主串口 {Host.MainPortName} 相同！" +
                    $" 需要：COMx=USB-RS485→C板，COMy=USB-TTL 3.3V→ELRS，两个适配器不能共用。");
                MessageBox.Show(this,
                    $"ELRS 端口 {portName} 与主串口相同！\n\n" +
                    $"主串口（RS485 → C 板）当前占用 {Host.MainPortName}，\n" +
                    $"ELRS 接收机必须接在**另一个 USB 转串口适配器**上。\n\n" +
                    $"物理上需要：\n" +
                    $"  COMx = USB-RS485 → C 板\n" +
                    $"  COMy = USB-TTL 3.3V → ELRS 接收机\n\n" +
                    $"两个 COM 号不能相同，也不能共用一个适配器。",
                    "端口冲突", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            try
            {
                _crsf = new CrsfReader(portName);
                _crsfEnabled = true;
                // 通知 MainWindow：关闭 XInput + 开启心跳（C 板速度命令 500ms 无更新归零）
                Host.OnElrsEnabled();
                TxtLinkState.Text = "已启用";
                TxtLinkState.Foreground = System.Windows.Media.Brushes.Green;
                Log($"ELRS 已启用：{portName} @ 420000 8N1（帧头 0xC8，CRC-8 poly=0xD5）");

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

            if (Host != null && Host.IsModbusConnected
                && string.Equals(portName, Host.MainPortName, StringComparison.OrdinalIgnoreCase))
            {
                Log($"[端口冲突] 所选 {portName} 与主串口 {Host.MainPortName} 相同，必须是两个不同适配器。");
                MessageBox.Show(this,
                    $"所选 ELRS 端口 {portName} 与主串口（RS485→C板，当前也用 {Host.MainPortName}）相同！\n\n" +
                    "必须是**两个不同的 USB 转串口适配器**。",
                    "端口冲突", MessageBoxButton.OK, MessageBoxImage.Error);
                return;
            }

            BtnCrsfTest.IsEnabled = false;
            BtnCrsfTest.Content = "测试中...";

            bool ownsReader = false;
            CrsfReader? r = _crsf;
            if (r == null)
            {
                try
                {
                    r = new CrsfReader(portName);
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

            // 重置统计
            r.TotalBytes = 0;
            r.SyncBytes = 0;
            r.ValidFrames = 0;
            r.BadFrames = 0;
            r.LastBadFrameLen = 0;
            r.LastBadFrameType = 0;
            r.RssiDbm = -128;
            r.FrameLoss = 0;
            r.LastFrameAt = DateTime.MinValue;

            DateTime start = DateTime.UtcNow;
            await Task.Run(() =>
            {
                while ((DateTime.UtcNow - start).TotalMilliseconds < 3000)
                {
                    r.Poll();
                    Thread.Sleep(5);
                }
            });

            var sb = new StringBuilder();
            sb.AppendLine($"===== ELRS 自检结果（{portName}，3 秒）=====");
            sb.AppendLine($"  总字节:    {r.TotalBytes}");
            sb.AppendLine($"  0xC8 帧头: {r.SyncBytes}");
            sb.AppendLine($"  有效帧:    {r.ValidFrames}（通过 CRC 校验）");
            sb.AppendLine($"  坏帧:      {r.BadFrames}（CRC 失败）");
            sb.AppendLine($"  RSSI:      {(r.RssiDbm == 0 ? "未知/0" : r.RssiDbm + " dBm")}");
            sb.AppendLine($"  丢帧计数:  {r.FrameLoss}");
            sb.AppendLine("  通道值（-1..+1，中点 0）：");
            for (int i = 0; i < 11; i++)
                sb.AppendLine($"    CH{i + 1,-2} = {r.Channels[i],8:F3}");
            sb.AppendLine();

            string verdict;
            MessageBoxImage icon;
            if (r.TotalBytes == 0)
            {
                verdict = "❌ 完全没收到字节\n\n可能原因：\n" +
                          "  1. USB-TTL TX/RX 接反了（要交叉：ELRS.TX → 转换器.RX）\n" +
                          "  2. ELRS 接收机未与遥控器对频（灯应常亮）\n" +
                          "  3. USB-TTL 是 RS232 电平版本（不是 3.3V TTL）\n" +
                          "  4. USB-TTL 不支持 420000 波特率";
                icon = MessageBoxImage.Error;
            }
            else if (r.SyncBytes == 0)
            {
                verdict = $"❌ 收到 {r.TotalBytes} 字节但无 0xC8 帧头\n\n" +
                          $"不是 CRSF 协议数据。可能：\n" +
                          $"  - ELRS 接收机协议不是 CRSF（去 ELRS Lua 脚本里改 Receiver Protocol → CRSF）\n" +
                          $"  - 接线不对（确认 ELRS.TX → 转换器.RX）\n\n" +
                          $"── 原始字节 hex dump（前 128 字节）──\n{r.DumpRaw(128)}\n" +
                          $"────────────────────────────";
                icon = MessageBoxImage.Error;
            }
            else if (r.ValidFrames == 0 && r.BadFrames > 0)
            {
                var badHex = new StringBuilder();
                for (int i = 0; i < Math.Min(r.LastBadFrameLen + 2, r.LastBadFrame.Length); i++)
                    badHex.Append($"{r.LastBadFrame[i]:X2} ");
                verdict = $"❌ 找到 {r.SyncBytes} 个 0xC8，{r.BadFrames} 个 CRC 失败，0 个有效\n\n" +
                          $"最近坏帧详情：\n" +
                          $"  len 字段 = {r.LastBadFrameLen}（CRSF 通道帧应为 24）\n" +
                          $"  type 字段 = 0x{r.LastBadFrameType:X2}（CRSF 通道帧应为 0x16）\n" +
                          $"  完整帧：{badHex}\n\n" +
                          $"可能原因：波特率不匹配 / ELRS 输出非标准 CRSF / 信号干扰导致字节翻转";
                icon = MessageBoxImage.Warning;
            }
            else if (r.ValidFrames == 0)
            {
                verdict = $"❌ 找到 {r.SyncBytes} 个 0xC8 但无完整帧通过 CRC\n\n" +
                          $"可能：波特率轻微偏差 / 信号干扰 / 帧被截断";
                icon = MessageBoxImage.Warning;
            }
            else
            {
                verdict = $"✅ 链路正常：{r.ValidFrames} 帧 / 3 秒 " +
                          $"（{(r.ValidFrames / 3.0):F1} Hz，期望 50~500 Hz）";
                icon = MessageBoxImage.Information;
            }
            sb.Append(verdict);

            Log(sb.ToString());
            MessageBox.Show(this, sb.ToString(), "ELRS 自检", MessageBoxButton.OK, icon);

            if (ownsReader) r.Dispose();

            BtnCrsfTest.IsEnabled = true;
            BtnCrsfTest.Content = "测试链路";
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
                try { _crsf.Poll(); }
                catch (Exception ex) { Log($"ELRS 读错误: {ex.Message}"); }

                // ---- 失联保护 ----
                if (_crsf.IsLinkLost)
                {
                    Log($"[失联] RSSI={_crsf.RssiDbm}dBm，1 秒无 CRSF 帧 → 触发急停并自动停用 ELRS（需手动重启）");
                    // 急停 + 停用 ELRS 都切回 UI 线程执行（DoEStop 会再次调用 DisableElrs，幂等）
                    Dispatcher.BeginInvoke(new Action(() =>
                    {
                        Host?.TriggerEStop();
                        ChkCrsfEnable.IsChecked = false;
                    }));
                    return;
                }

                // 通道读取（Channels 是 volatile 数组，多线程可见）
                float ch1 = _crsf.Channels[0];  // 右X → wz_raw
                float ch3 = _crsf.Channels[2];  // 左Y → vy_raw
                float ch4 = _crsf.Channels[3];  // 左X → vx_raw
                float ch5 = _crsf.Channels[4];  // 三段开关 → 档位
                float ch6 = _crsf.Channels[5];  // 二段开关 → 使能

                // 三段开关 → 档位（取最新值，无需边沿）
                SpeedGear newGear = ch5 > 0.5f ? SpeedGear.Fast
                                  : ch5 < -0.5f ? SpeedGear.Slow
                                  : SpeedGear.Mid;
                // SetGear 会写 CbGear.SelectedIndex（UI），必须切回 UI 线程
                Dispatcher.BeginInvoke(new Action(() => Host?.SetGear(newGear)));

                // 二段开关 → 使能切换（上升沿触发）
                bool swArm = ch6 > 0;
                if (swArm && !_crsfSwitchArm_prev)
                    Dispatcher.BeginInvoke(new Action(() => Host?.TriggerEnableToggle()));
                _crsfSwitchArm_prev = swArm;

                // 摇杆映射到车体速度（vx=右+, vy=前+, wz=逆时针+）
                var (vx, vy, wz) = MotionMapper.Map(leftY: ch3, leftX: ch4, rightX: ch1, newGear);

                // 写入 MainWindow 的共享速度命令缓冲区（含心跳）
                Host?.SetVelocity(vx, vy, wz);

                // 更新本窗口 UI
                Dispatcher.BeginInvoke(new Action(() =>
                {
                    PbLY.Value = ch3;
                    PbLX.Value = ch4;
                    PbRX.Value = ch1;
                    TxtVy.Text = vy.ToString("F2");
                    TxtVx.Text = vx.ToString("F2");
                    TxtWz.Text = wz.ToString("F2");

                    TxtCh1.Text = ch1.ToString("F3");
                    TxtCh2.Text = _crsf.Channels[1].ToString("F3");
                    TxtCh3.Text = ch3.ToString("F3");
                    TxtCh4.Text = ch4.ToString("F3");
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

        /// <summary>主串口 COM 号（用于 ELRS 端口冲突防呆）。</summary>
        string? MainPortName { get; }

        /// <summary>写入共享速度命令缓冲区（加锁 + 置心跳），由 CommLoop 下发。</summary>
        void SetVelocity(float vx, float vy, float wz);

        /// <summary>ELRS 启用回调：关闭 XInput 手柄互斥 + 开启心跳。</summary>
        void OnElrsEnabled();

        /// <summary>ELRS 停用回调：清零速度命令 + 关闭心跳。</summary>
        void OnElrsDisabled();

        /// <summary>失联时触发 MainWindow 急停（清速度 + 急停电机）。</summary>
        void TriggerEStop();

        /// <summary>CH6 上升沿触发使能切换。</summary>
        void TriggerEnableToggle();

        /// <summary>CH5 三段开关同步档位（慢/中/快）。</summary>
        void SetGear(SpeedGear gear);

        /// <summary>回灌一条日志到 MainWindow 调试日志区。</summary>
        void Log(string msg);
    }
}
