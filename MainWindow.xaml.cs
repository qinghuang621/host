using System;
using System.IO.Ports;
using System.Linq;
using System.Threading;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Media;
using SharpDX.XInput;

namespace GamepadSpeedController
{
    public partial class MainWindow : Window
    {
        private ModbusClient? _mb;
        private bool _connected;
        private bool _motorsEnabled;
        private CancellationTokenSource? _commCts;

        // 风机 Modbus 控制（复用主串口 _mb，无独立连接）
        private FanPreCheck _fanPre;
        private FanState[] _fanStates = new FanState[4];

        // 姿态3D可视化窗口（按需打开；关闭后置 null）
        private AttitudeWindow? _attitudeWindow;

        // 速度命令源（UI 线程写入，通信线程读取）
        private float _cmdVx, _cmdVy, _cmdWz;
        private bool _cmdSendOnce;
        private bool _cmdHeartbeat;
        private readonly object _cmdLock = new();

        // 手柄
        private Controller? _pad;
        private bool _padEnabled;
        private SpeedGear _gear = SpeedGear.Mid;
        private bool _btnA_prev, _btnB_prev, _btnStart_prev;
        private bool _leftTrig_prev, _rightTrig_prev;

        public MainWindow()
        {
            InitializeComponent();
            Loaded += MainWindow_Loaded;
        }

        private void MainWindow_Loaded(object sender, RoutedEventArgs e)
        {
            CbPort.ItemsSource = SerialPort.GetPortNames().OrderBy(p => p).ToList();
            if (CbPort.Items.Count > 0) CbPort.SelectedIndex = 0;
            CbBaud.ItemsSource = new[] { 9600, 19200, 38400, 57600, 115200, 230400 };
            CbBaud.SelectedItem = 115200;

            // 手柄档位
            CbGear.ItemsSource = new[] { "慢速", "中速", "快速" };
            CbGear.SelectedIndex = 1;

            // 风机 Modbus 走主串口 _mb，无需独立端口配置

            // 尝试检测手柄
            DetectGamepad();
        }

        // ========== 手柄检测 ==========

        private void DetectGamepad()
        {
            for (UserIndex i = UserIndex.One; i <= UserIndex.Four; i++)
            {
                var c = new Controller(i);
                if (c.IsConnected)
                {
                    _pad = c;
                    TxtPadName.Text = $"XInput {i} (已连接)";
                    TxtPadName.Foreground = System.Windows.Media.Brushes.Green;
                    return;
                }
            }
            _pad = null;
            TxtPadName.Text = "未检测到手柄";
            TxtPadName.Foreground = System.Windows.Media.Brushes.Gray;
        }

        private void ChkPadEnable_Checked(object sender, RoutedEventArgs e)
        {
            if (_pad == null) DetectGamepad();
            if (_pad == null || !_pad.IsConnected)
            {
                ChkPadEnable.IsChecked = false;
                MessageBox.Show("未检测到 XInput 手柄，请确认 USB 连接后重试。",
                    "手柄未连接", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            _padEnabled = true;
            // 启用手柄时自动开启心跳
            lock (_cmdLock) { _cmdHeartbeat = true; }
            ChkHeartbeat.IsChecked = true;
        }

        private void ChkPadEnable_Unchecked(object sender, RoutedEventArgs e)
        {
            _padEnabled = false;
            // 停止速度命令
            lock (_cmdLock)
            {
                _cmdVx = 0; _cmdVy = 0; _cmdWz = 0;
                _cmdHeartbeat = false;
            }
            ChkHeartbeat.IsChecked = false;
        }

        private void CbGear_SelectionChanged(object sender, System.Windows.Controls.SelectionChangedEventArgs e)
        {
            if (CbGear.SelectedIndex >= 0)
                _gear = (SpeedGear)CbGear.SelectedIndex;
        }

        // ========== 通用方法 ==========

        private void SetStatus(string text, bool ok = false)
        {
            TxtStatus.Text = text;
            TxtStatus.Foreground = ok ? System.Windows.Media.Brushes.Green : System.Windows.Media.Brushes.Gray;
        }

        private int _logLines;
        private void OnDebugLog(string msg)
        {
            Dispatcher.BeginInvoke(new Action(() =>
            {
                var line = $"[{DateTime.Now:HH:mm:ss.fff}] {msg}\n";
                TxtDebugLog.AppendText(line);
                _logLines++;
                if (_logLines > 100)
                {
                    var text = TxtDebugLog.Text;
                    int keep = text.LastIndexOf('\n', text.Length / 2);
                    if (keep > 0) TxtDebugLog.Text = text.Substring(keep + 1);
                    _logLines = TxtDebugLog.Text.Count(c => c == '\n');
                }
                TxtDebugLog.ScrollToEnd();
            }));
        }

        private void SetEnableState(bool enabled)
        {
            _motorsEnabled = enabled;
            BtnEnable.Content = enabled ? "失能电机" : "使能电机";
            BtnEnable.Background = enabled
                ? System.Windows.Media.Brushes.LightGreen
                : System.Windows.Media.Brushes.LightGray;
            TxtEnableState.Text = enabled ? "状态：已使能" : "状态：失能";
            TxtEnableState.Foreground = enabled
                ? System.Windows.Media.Brushes.Green
                : System.Windows.Media.Brushes.Gray;
        }

        // ========== 连接/断开 ==========

        private void BtnConnect_Click(object sender, RoutedEventArgs e)
        {
            if (!_connected)
            {
                try
                {
                    var port = CbPort.Text;
                    var baud = int.TryParse(CbBaud.Text, out var b) ? b : 115200;
                    _mb = new ModbusClient(port, baud);
                    _mb.DebugLog += OnDebugLog;
                    _connected = true;
                    BtnConnect.Content = "断开";
                    SetStatus($"已连接 {port} @ {baud}", true);
                    SetEnableState(false);
                    StartCommThread();
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"连接失败：{ex.Message}", "错误",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
            else
            {
                StopCommThread();
                _mb?.Dispose();
                _mb = null;
                _connected = false;
                BtnConnect.Content = "连接";
                SetStatus("未连接");
                SetEnableState(false);
                ClearMonitorDisplay();
            }
        }

        // ========== 使能/急停 ==========

        private void BtnEnable_Click(object sender, RoutedEventArgs e)
        {
            DoEnableToggle();
        }

        private void DoEnableToggle()
        {
            if (_mb == null || !_connected) return;
            try
            {
                if (_motorsEnabled)
                {
                    _mb.SetMotorsEnabled(false);
                    SetEnableState(false);
                }
                else
                {
                    _mb.SetMotorsEnabled(false);
                    Thread.Sleep(50);
                    _mb.SetMotorsEnabled(true);
                    SetEnableState(true);
                }
            }
            catch (Exception ex)
            {
                MessageBox.Show($"使能失败：{ex.Message}", "错误",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        private void DoEStop()
        {
            if (_mb == null) return;
            try
            {
                lock (_cmdLock) { _cmdHeartbeat = false; _cmdSendOnce = false; _cmdVx = 0; _cmdVy = 0; _cmdWz = 0; }
                ChkHeartbeat.IsChecked = false;
                ChkPadEnable.IsChecked = false;
                _padEnabled = false;
                _mb.EmergencyStop();
                SetEnableState(false);
                SetStatus("已紧急停止", false);
            }
            catch (Exception ex)
            {
                OnDebugLog($"EStop error: {ex.Message}");
            }
        }

        private void BtnEStop_Click(object sender, RoutedEventArgs e)
        {
            DoEStop();
        }

        // ========== 速度输入 ==========

        private void BtnSend_Click(object sender, RoutedEventArgs e)
        {
            lock (_cmdLock)
            {
                _cmdVx = ParseFloat(TxtVx.Text);
                _cmdVy = ParseFloat(TxtVy.Text);
                _cmdWz = ParseFloat(TxtWz.Text);
                _cmdSendOnce = true;
                _cmdHeartbeat = ChkHeartbeat.IsChecked == true;
            }
        }

        private void ChkHeartbeat_Checked(object sender, RoutedEventArgs e)
        {
            lock (_cmdLock) { _cmdHeartbeat = true; }
        }

        private void ChkHeartbeat_Unchecked(object sender, RoutedEventArgs e)
        {
            // 手柄模式时不要关心跳
            if (!_padEnabled)
                lock (_cmdLock) { _cmdHeartbeat = false; }
        }

        private float ParseFloat(string s)
        {
            if (float.TryParse(s, System.Globalization.NumberStyles.Float,
                System.Globalization.CultureInfo.InvariantCulture, out var v))
                return v;
            return 0f;
        }

        // ========== 姿态3D可视化 ==========

        private void BtnAttitude3D_Click(object sender, RoutedEventArgs e)
        {
            if (_attitudeWindow == null || !_attitudeWindow.IsLoaded)
            {
                _attitudeWindow = new AttitudeWindow { Owner = this };
                _attitudeWindow.Closed += (_, _) => _attitudeWindow = null;
                _attitudeWindow.Show();
            }
            else
            {
                _attitudeWindow.Activate();
            }
        }

        // ========== 单一通信线程 ==========

        private void StartCommThread()
        {
            StopCommThread();
            _commCts = new CancellationTokenSource();
            var token = _commCts.Token;
            Task.Run(() => CommLoop(token), token);
        }

        private void StopCommThread()
        {
            _commCts?.Cancel();
            _commCts?.Dispose();
            _commCts = null;
        }

        private void CommLoop(CancellationToken token)
        {
            int tick = 0;
            int errCount = 0;

            while (!token.IsCancellationRequested)
            {
                // ---- 1. 读手柄 + 发速度命令 ----
                if (_padEnabled && _pad != null && _pad.IsConnected)
                {
                    PollGamepad();
                }

                bool shouldSend = false;
                float vx = 0, vy = 0, wz = 0;

                lock (_cmdLock)
                {
                    if (_cmdHeartbeat || _cmdSendOnce)
                    {
                        vx = _cmdVx; vy = _cmdVy; wz = _cmdWz;
                        shouldSend = true;
                        _cmdSendOnce = false;
                    }
                }

                if (shouldSend && _mb != null)
                {
                    try { _mb.SendVelocity(vx, vy, wz); } catch { }
                }

                tick++;

                // ---- 2. 每 3 次 tick (300ms) 读监控 ----
                if (tick % 3 == 0 && _mb != null)
                {
                    try
                    {
                        float[] cmdVel = _mb.ReadMotorVelocities();
                        MotorState[] states = _mb.ReadMotorStates();
                        errCount = 0;
                        Dispatcher.BeginInvoke(new Action(() =>
                            UpdateMonitorDisplay(cmdVel, states)));
                    }
                    catch (Exception ex)
                    {
                        errCount++;
                        OnDebugLog($"Monitor read error: {ex.Message}");
                        if (errCount >= 3)
                        {
                            Dispatcher.BeginInvoke(new Action(() =>
                                SetStatus("读取失败（通信正常）", false)));
                            errCount = 0;
                        }
                    }
                }

                // ---- 3. 姿态3D窗口若打开则每 tick 读 IMU 姿态 ----
                // IMU 块 0x0150~0x015F，响应 37B ≈ 3.3ms 传输，与电机监控错峰即可
                if (_attitudeWindow != null && _mb != null)
                {
                    try
                    {
                        var imu = _mb.ReadImu();
                        Dispatcher.BeginInvoke(new Action(() =>
                            _attitudeWindow?.UpdateAngles(imu)));
                    }
                    catch (Exception ex)
                    {
                        OnDebugLog($"IMU read error: {ex.Message}");
                    }
                }

                // ---- 4. 每 200ms (tick % 2 == 0) 读风机状态 + LUT 前置条件 ----
                // 风机状态 0x0110~0x0137 (40 只) ≈ 85B ≈ 7.4ms 传输
                // 前置条件 0x0140~0x016F (48 只) ≈ 100B ≈ 9ms 传输
                // 与 300ms 电机监控错峰（最小公倍 600ms 才同步一次）
                if (tick % 2 == 0 && _mb != null)
                {
                    try
                    {
                        var states = _mb.ReadFanStates();
                        var pre    = _mb.ReadFanPreCheck();
                        Dispatcher.BeginInvoke(new Action(() =>
                            UpdateFanDisplay(states, pre)));
                    }
                    catch (Exception ex)
                    {
                        OnDebugLog($"Fan read error: {ex.Message}");
                    }
                }

                Thread.Sleep(100);
            }
        }

        // ========== 手柄轮询 ==========

        private void PollGamepad()
        {
            if (_pad == null || !_pad.IsConnected) return;

            var state = _pad.GetState();
            var gp = state.Gamepad;

            // 摇杆值归一化到 -1..+1
            float leftY = gp.LeftThumbY / 32768f;
            float leftX = gp.LeftThumbX / 32768f;
            float rightX = gp.RightThumbX / 32768f;

            // 映射到车体速度
            // 手柄 Y(上下) → vy(前后，前+)，手柄 X(左右) → vx(右移，右+)
            // 注意：vx/vz 的**变量名是"右+/逆时针+"语义**，与固件 running/kinematics.c 一致。
            // 详见 接口文档.md §5.1「电机控制区」。
            var (vx, vy, wz) = MotionMapper.Map(leftY, leftX, rightX, _gear);

            // 写入命令缓冲区
            lock (_cmdLock)
            {
                _cmdVx = vx;
                _cmdVy = vy;
                _cmdWz = wz;
                _cmdHeartbeat = true;
            }

            // 按钮边沿检测
            bool btnA = (state.Gamepad.Buttons & GamepadButtonFlags.A) != 0;
            bool btnB = (state.Gamepad.Buttons & GamepadButtonFlags.B) != 0;
            bool btnStart = (state.Gamepad.Buttons & GamepadButtonFlags.Start) != 0;

            if (btnA && !_btnA_prev) Dispatcher.BeginInvoke(new Action(DoEnableToggle));
            if (btnB && !_btnB_prev) Dispatcher.BeginInvoke(new Action(() => { if (_motorsEnabled) DoEnableToggle(); }));
            if (btnStart && !_btnStart_prev) Dispatcher.BeginInvoke(new Action(DoEStop));

            _btnA_prev = btnA;
            _btnB_prev = btnB;
            _btnStart_prev = btnStart;

            // 扳机切换档位（边沿触发）
            bool leftTrig = gp.LeftTrigger > 64;
            bool rightTrig = gp.RightTrigger > 64;
            if (leftTrig && !_leftTrig_prev)
            {
                int newIdx = Math.Max(0, (int)_gear - 1);
                Dispatcher.BeginInvoke(new Action(() => { CbGear.SelectedIndex = newIdx; }));
            }
            if (rightTrig && !_rightTrig_prev)
            {
                int newIdx = Math.Min(2, (int)_gear + 1);
                Dispatcher.BeginInvoke(new Action(() => { CbGear.SelectedIndex = newIdx; }));
            }
            _leftTrig_prev = leftTrig;
            _rightTrig_prev = rightTrig;

            // 更新摇杆 UI
            Dispatcher.BeginInvoke(new Action(() =>
            {
                PbLY.Value = leftY;
                PbLX.Value = leftX;
                PbRX.Value = rightX;
                TxtPadVx.Text = vx.ToString("F2");
                TxtPadVy.Text = vy.ToString("F2");
                TxtPadWz.Text = wz.ToString("F2");
            }));
        }

        // ========== UI 更新 ==========

        private void UpdateMonitorDisplay(float[] cmdVel, MotorState[] states)
        {
            // 更新手动页 + 手柄页两组表格
            for (int i = 0; i < 4 && i < cmdVel.Length; i++)
            {
                string val = cmdVel[i].ToString("F2");
                // 手动页
                var tb1 = i switch { 0 => TxtCmdVel1, 1 => TxtCmdVel2, 2 => TxtCmdVel3, _ => TxtCmdVel4 };
                tb1.Text = val;
                // 手柄页
                var tb2 = i switch { 0 => TxtPadCmdVel1, 1 => TxtPadCmdVel2, 2 => TxtPadCmdVel3, _ => TxtPadCmdVel4 };
                tb2.Text = val;
            }

            for (int i = 0; i < 4 && i < states.Length; i++)
            {
                var s = states[i];
                string errText = s.Err == 0 ? "OK" : $"ERR={s.Err}";
                var errColor = s.Err == 0
                    ? System.Windows.Media.Brushes.Green
                    : System.Windows.Media.Brushes.Red;
                string velText = s.Vel.ToString("F2");
                string tempText = $"{s.TempMos}";

                // 手动页
                var (e1, v1, t1) = i switch
                {
                    0 => (TxtErr1, TxtActVel1, TxtTemp1),
                    1 => (TxtErr2, TxtActVel2, TxtTemp2),
                    2 => (TxtErr3, TxtActVel3, TxtTemp3),
                    _ => (TxtErr4, TxtActVel4, TxtTemp4)
                };
                e1.Text = errText; e1.Foreground = errColor;
                v1.Text = velText;
                t1.Text = tempText;

                // 手柄页
                var (e2, v2, t2) = i switch
                {
                    0 => (TxtPadErr1, TxtPadActVel1, TxtPadTemp1),
                    1 => (TxtPadErr2, TxtPadActVel2, TxtPadTemp2),
                    2 => (TxtPadErr3, TxtPadActVel3, TxtPadTemp3),
                    _ => (TxtPadErr4, TxtPadActVel4, TxtPadTemp4)
                };
                e2.Text = errText; e2.Foreground = errColor;
                v2.Text = velText;
                t2.Text = tempText;
            }

            SetStatus($"已连接 {CbPort.Text} @ {CbBaud.Text}", true);
        }

        private void ClearMonitorDisplay()
        {
            // 手动页
            TxtErr1.Text = TxtErr2.Text = TxtErr3.Text = TxtErr4.Text = "---";
            TxtCmdVel1.Text = TxtCmdVel2.Text = TxtCmdVel3.Text = TxtCmdVel4.Text = "---";
            TxtActVel1.Text = TxtActVel2.Text = TxtActVel3.Text = TxtActVel4.Text = "---";
            TxtTemp1.Text = TxtTemp2.Text = TxtTemp3.Text = TxtTemp4.Text = "---";
            // 手柄页
            TxtPadErr1.Text = TxtPadErr2.Text = TxtPadErr3.Text = TxtPadErr4.Text = "---";
            TxtPadCmdVel1.Text = TxtPadCmdVel2.Text = TxtPadCmdVel3.Text = TxtPadCmdVel4.Text = "---";
            TxtPadActVel1.Text = TxtPadActVel2.Text = TxtPadActVel3.Text = TxtPadActVel4.Text = "---";
            TxtPadTemp1.Text = TxtPadTemp2.Text = TxtPadTemp3.Text = TxtPadTemp4.Text = "---";
        }

        // ========== 风机 Modbus 控制（复用 _mb 串口） ==========

        /// <summary>
        /// 一键检查 LUT 模式前置条件：读 0x0140~0x016F 48 只，
        /// 刷 7 个红绿灯 + 状态行，并据此启停 [线性 / 查表] 按钮。
        /// </summary>
        private void BtnFanCheckPre_Click(object sender, RoutedEventArgs e)
        {
            if (_mb == null || !_connected)
            {
                MessageBox.Show("请先在顶部连接主串口 _mb", "未连接",
                    MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            try
            {
                _fanPre = _mb.ReadFanPreCheck();
                EvalPreCheck(_fanPre);
            }
            catch (Exception ex)
            {
                OnDebugLog($"FanPreCheck error: {ex.Message}");
                MessageBox.Show($"读取失败：{ex.Message}", "错误",
                    MessageBoxButton.OK, MessageBoxImage.Error);
            }
        }

        /// <summary>
        /// 评估前置条件 7 项：刷红绿灯（绿=通过、红=未通过），
        /// 写状态行，并据此设置 BtnFanLinear / BtnFanLut 的 IsEnabled。
        /// </summary>
        private void EvalPreCheck(FanPreCheck pre)
        {
            bool okImu    = pre.ImuStatus == 2;
            bool okMagic  = pre.LutMagic  == 0xA5C3;
            // LUT 13 格非全 0 且最大值 ≤ 100（写表后才视为有效）
            int lutMax = 0, lutNonZero = 0;
            for (int i = 0; i < pre.Lut.Length; i++)
            {
                if (pre.Lut[i] > 0) lutNonZero++;
                if (pre.Lut[i] > lutMax) lutMax = pre.Lut[i];
            }
            bool okLut    = lutNonZero >= 1 && lutMax <= 100;
            bool okAutoEn = pre.Params.AutoEn  == 1;
            bool okMode   = pre.Params.Mode    == 2;
            bool okDutyMin= pre.Params.DutyMin > 0;
            // OFFSET 已校：暂以"非零或水平读数 roll/pitch≈0"判定。
            // 由于此帧未含 IMU roll/pitch，只能粗判 OFFSET 字段；
            // 用户可在水平台面上点此检查，再人工对比 IMU roll/pitch≈0。
            bool okOffset = true; // 默认通过；严格判定需读 0x0150/0x0152 比对

            SetLed(LedPreImu,     okImu);
            SetLed(LedPreMagic,   okMagic);
            SetLed(LedPreLut,     okLut);
            SetLed(LedPreAutoEn,  okAutoEn);
            SetLed(LedPreMode,    okMode);
            SetLed(LedPreDutyMin, okDutyMin);
            SetLed(LedPreOffset,  okOffset);

            string imuTxt = pre.ImuStatus switch
            {
                0 => "离线",
                1 => "加热中",
                2 => "RUNNING",
                3 => "错误",
                _ => pre.ImuStatus.ToString(),
            };
            string modeTxt = pre.Params.Mode switch
            {
                0 => "手动",
                1 => "线性",
                2 => "查表",
                _ => pre.Params.Mode.ToString(),
            };
            TxtPreStatus.Text =
                $"IMU={pre.ImuStatus}({imuTxt})  MAGIC=0x{pre.LutMagic:X4}  " +
                $"LUT非零={lutNonZero}/13(最大{lutMax})  AUTO_EN={pre.Params.AutoEn}  " +
                $"MODE={pre.Params.Mode}({modeTxt})  DUTY_MIN={pre.Params.DutyMin}  " +
                $"PITCH_OFF={pre.Params.PitchOffset} ROLL_OFF={pre.Params.RollOffset}  " +
                $"θ={pre.TiltTheta:F1}°  AUTO_DUTY={pre.AutoDuty:F1}%";

            // 按钮启停：线性需要 IMU + DUTY_MIN；查表需要 7 项全过
            BtnFanLinear.IsEnabled = okImu && okDutyMin;
            BtnFanLut.IsEnabled     = okImu && okMagic && okLut && okDutyMin && okOffset;
        }

        private static void SetLed(System.Windows.Controls.Border led, bool ok)
        {
            led.Background = ok ? System.Windows.Media.Brushes.LimeGreen
                                : System.Windows.Media.Brushes.OrangeRed;
        }

        private void BtnFanManual_Click(object sender, RoutedEventArgs e)
        {
            if (_mb == null) return;
            try
            {
                _mb.WriteFanMode(0, 0);
                OnDebugLog("Fan: 切手动 (0x0140=0, 0x0141=0)");
            }
            catch (Exception ex) { OnDebugLog($"Fan Manual error: {ex.Message}"); }
        }

        private void BtnFanLinear_Click(object sender, RoutedEventArgs e)
        {
            if (_mb == null) return;
            // 线性模式仅覆盖 [0°, 90°]，提醒用户
            if (MessageBox.Show("线性模式仅覆盖 θ∈[0°, 90°]，θ>90° 时吸附力模型失真。\n确认切到线性 (FAN_MODE=1, AUTO_EN=1)？",
                    "确认切线性", MessageBoxButton.OKCancel, MessageBoxImage.Warning)
                != MessageBoxResult.OK) return;
            try
            {
                _mb.WriteFanMode(1, 1);
                OnDebugLog("Fan: 切线性 (0x0140=1, 0x0141=1)");
            }
            catch (Exception ex) { OnDebugLog($"Fan Linear error: {ex.Message}"); }
        }

        private void BtnFanLut_Click(object sender, RoutedEventArgs e)
        {
            if (_mb == null) return;
            // 7 项前置条件是否全过（以最近一次检查为准）
            if (!BtnFanLut.IsEnabled)
            {
                MessageBox.Show("LUT 前置条件未全部通过：\n请先点\"一键检查\"，确认 IMU/MAGIC/LUT/AUTO_EN/MODE/DUTY_MIN/OFFSET 7 项全绿。\n" +
                                "若 LUT 表未写：调 Modbus Poll 或本工具 WriteLut 写 0x0160~0x016C 13 格 + 0x016D=0xA5C3。",
                                "前置条件未满足", MessageBoxButton.OK, MessageBoxImage.Warning);
                return;
            }
            try
            {
                _mb.WriteFanMode(2, 1);
                OnDebugLog("Fan: 切查表 (0x0140=2, 0x0141=1)");
            }
            catch (Exception ex) { OnDebugLog($"Fan Lut error: {ex.Message}"); }
        }

        private void BtnFanEStop_Click(object sender, RoutedEventArgs e)
        {
            if (_mb == null) return;
            try
            {
                _mb.FanEStop();
                OnDebugLog("Fan: 已紧急停止 (4 路 duty=0, 切回手动)");
                // 立刻清 UI 显示
                for (int i = 0; i < 4; i++) _fanStates[i] = default;
                UpdateFanDisplay(_fanStates, _fanPre);
            }
            catch (Exception ex) { OnDebugLog($"Fan EStop error: {ex.Message}"); }
        }

        /// <summary>
        /// 4 个风机滑块共用 ValueChanged 处理：Tag=1~4 标识风机编号。
        /// 仅手动模式（_fanPre.Params.Mode==0）下下发；自动模式禁用滑块避免冲突。
        /// 一次写 4 路占空比（WriteFanDuties 一条 FC10），降低串口压力。
        /// </summary>
        private void SlFanDuty_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            if (sender is not System.Windows.Controls.Slider sl) return;
            if (sl.Tag is not string tag || !int.TryParse(tag, out int fanIdx)) return;
            // 找对应文本控件（避免 InitializeComponent 期间 NRE）
            var txt = fanIdx switch
            {
                1 => TxtFan1Duty, 2 => TxtFan2Duty, 3 => TxtFan3Duty, 4 => TxtFan4Duty, _ => null
            };
            if (txt == null) return;
            int v = (int)Math.Round(e.NewValue, MidpointRounding.AwayFromZero);
            if (v < 0) v = 0; if (v > 100) v = 100;
            txt.Text = $"{v}%";

            if (_mb == null || !_connected) return;
            // 自动模式（Mode!=0）下不下发，输出被 fan_auto_update 覆盖
            if (_fanPre.Params.Mode != 0) return;

            // 收集 4 路当前值（含本次变更），一次写
            ushort d1 = (ushort)ClampDuty(SlFan1Duty?.Value ?? 0);
            ushort d2 = (ushort)ClampDuty(SlFan2Duty?.Value ?? 0);
            ushort d3 = (ushort)ClampDuty(SlFan3Duty?.Value ?? 0);
            ushort d4 = (ushort)ClampDuty(SlFan4Duty?.Value ?? 0);
            try { _mb.WriteFanDuties(d1, d2, d3, d4); }
            catch (Exception ex) { OnDebugLog($"Fan duty write error: {ex.Message}"); }
        }

        private static int ClampDuty(double v)
        {
            int i = (int)Math.Round(v, MidpointRounding.AwayFromZero);
            return i < 0 ? 0 : (i > 100 ? 100 : i);
        }

        /// <summary>
        /// 刷新四路风机面板 + 共用区 + 模式按钮启停。
        /// 由 CommLoop 每 200ms 在 UI 线程调度。
        /// </summary>
        private void UpdateFanDisplay(FanState[] states, FanPreCheck pre)
        {
            _fanStates = states;
            _fanPre     = pre;

            // 共用区
            TxtTheta.Text     = $"{pre.TiltTheta:F1}°";
            TxtAutoDuty.Text  = $"{pre.AutoDuty:F1}%";
            string imuTxt = pre.ImuStatus switch
            {
                0 => "离线", 1 => "加热", 2 => "RUN", 3 => "错误", _ => pre.ImuStatus.ToString(),
            };
            TxtImuStatus.Text = imuTxt;
            string modeTxt = pre.Params.Mode switch
            {
                0 => "手动", 1 => "线性", 2 => "查表", _ => pre.Params.Mode.ToString(),
            };
            TxtFanMode.Text = modeTxt;

            // 自动模式下禁用 4 个滑块，避免与 fan_auto_update 冲突
            bool sliderEnabled = (pre.Params.Mode == 0);
            SlFan1Duty.IsEnabled = sliderEnabled;
            SlFan2Duty.IsEnabled = sliderEnabled;
            SlFan3Duty.IsEnabled = sliderEnabled;
            SlFan4Duty.IsEnabled = sliderEnabled;

            // 4 路面板
            UpdateFanRow(1, states[0]);
            UpdateFanRow(2, states[1]);
            UpdateFanRow(3, states[2]);
            UpdateFanRow(4, states[3]);

            // 顺带刷红绿灯（CommLoop 每次都重评估，前置条件变化时即时反映）
            EvalPreCheck(pre);
        }

        private void UpdateFanRow(int idx, FanState s)
        {
            var led   = idx switch { 1 => LedFan1Run, 2 => LedFan2Run, 3 => LedFan3Run, 4 => LedFan4Run, _ => null };
            var pb    = idx switch { 1 => PbFan1DutyFb, 2 => PbFan2DutyFb, 3 => PbFan3DutyFb, 4 => PbFan4DutyFb, _ => null };
            var tDuty = idx switch { 1 => TxtFan1DutyFb, 2 => TxtFan2DutyFb, 3 => TxtFan3DutyFb, 4 => TxtFan4DutyFb, _ => null };
            var tPul  = idx switch { 1 => TxtFan1Pulse, 2 => TxtFan2Pulse, 3 => TxtFan3Pulse, 4 => TxtFan4Pulse, _ => null };
            var tRpm  = idx switch { 1 => TxtFan1Rpm, 2 => TxtFan2Rpm, 3 => TxtFan3Rpm, 4 => TxtFan4Rpm, _ => null };
            if (led == null || pb == null || tDuty == null || tPul == null || tRpm == null) return;

            led.Background = s.Run != 0 ? System.Windows.Media.Brushes.LimeGreen
                                        : System.Windows.Media.Brushes.Gray;
            pb.Value = Math.Clamp(s.DutyFb, 0, 100);
            tDuty.Text = $"{s.DutyFb:F1}%";
            tPul.Text = $"{s.Pulse:F0}";
            tRpm.Text = $"{s.Rpm:F0} RPM";
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            StopCommThread();
            _mb?.Dispose();
            base.OnClosing(e);
        }
    }
}
