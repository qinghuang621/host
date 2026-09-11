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

        // 风机控制
        private FanSerialClient? _fan;
        private bool _fanConnected;

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

            // 风机端口
            CbFanPort.ItemsSource = SerialPort.GetPortNames().OrderBy(p => p).ToList();
            if (CbFanPort.Items.Count > 0) CbFanPort.SelectedIndex = 0;
            CbFanBaud.ItemsSource = new[] { 9600, 19200, 38400, 57600, 115200, 230400 };
            CbFanBaud.SelectedItem = 115200;

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
            // 详见 接口文档.md §5.6「坐标与符号约定」。
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

        // ========== 风机串口 ==========

        private void BtnFanRefresh_Click(object sender, RoutedEventArgs e)
        {
            CbFanPort.ItemsSource = SerialPort.GetPortNames().OrderBy(p => p).ToList();
            if (CbFanPort.Items.Count > 0) CbFanPort.SelectedIndex = 0;
        }

        private void BtnFanConnect_Click(object sender, RoutedEventArgs e)
        {
            if (!_fanConnected)
            {
                try
                {
                    var port = CbFanPort.Text;
                    if (string.IsNullOrWhiteSpace(port))
                    {
                        MessageBox.Show("请选择风机串口", "提示", MessageBoxButton.OK, MessageBoxImage.Warning);
                        return;
                    }
                    var baud = int.TryParse(CbFanBaud.Text, out var b) ? b : 115200;
                    _fan = new FanSerialClient(port, baud);
                    _fan.DebugLog += OnFanLog;
                    _fan.ReplyReceived += OnFanReply;
                    _fanConnected = true;
                    BtnFanConnect.Content = "断开";
                    BtnFanConnect.Background = Brushes.MistyRose;
                    TxtFanStatus.Text = $"已连接 {port}";
                    TxtFanStatus.Foreground = Brushes.SeaGreen;
                    ThreadPool.QueueUserWorkItem(_ =>
                    {
                        Thread.Sleep(200);
                        try { _fan?.SendStatus(); } catch { }
                    });
                }
                catch (Exception ex)
                {
                    MessageBox.Show($"风机串口打开失败：{ex.Message}", "错误",
                        MessageBoxButton.OK, MessageBoxImage.Error);
                }
            }
            else
            {
                try
                {
                    if (_fan != null)
                    {
                        _fan.ReplyReceived -= OnFanReply;
                        _fan.DebugLog -= OnFanLog;
                        _fan.Dispose();
                    }
                } catch { }
                _fan = null;
                _fanConnected = false;
                BtnFanConnect.Content = "连接";
                BtnFanConnect.Background = Brushes.PaleGreen;
                TxtFanStatus.Text = "未连接";
                TxtFanStatus.Foreground = Brushes.Gray;
                ResetFanUi();
            }
        }

        private void BtnFanHelp_Click(object sender, RoutedEventArgs e)   => _fan?.SendHelp();
        private void BtnFanStart_Click(object sender, RoutedEventArgs e)  => _fan?.SendStart();
        private void BtnFanStop_Click(object sender, RoutedEventArgs e)   => _fan?.SendStop();
        private void BtnFanStatus_Click(object sender, RoutedEventArgs e) => _fan?.SendStatus();

        private void SlFanDuty_ValueChanged(object sender, RoutedPropertyChangedEventArgs<double> e)
        {
            // InitializeComponent 期间 Slider.Value 由 0 变为 50 会提前触发本回调，
            // 此时 TxtFanDuty 尚未创建，需要做空保护。
            if (TxtFanDuty == null) return;
            int v = (int)Math.Round(e.NewValue, MidpointRounding.AwayFromZero);
            if (v < 0) v = 0; if (v > 100) v = 100;
            TxtFanDuty.Text = $"{v}%";
        }

        private void FanPreset_Click(object sender, RoutedEventArgs e)
        {
            if (sender is System.Windows.Controls.Button btn && btn.Tag is string tag && int.TryParse(tag, out var pct))
            {
                SlFanDuty.Value = pct;
                if (_fanConnected) SendFanDuty(pct);
            }
        }

        private void SendFanDuty(int v)
        {
            if (v < 0) v = 0; if (v > 100) v = 100;
            _fan?.SendDuty(v);
        }

        private void BtnFanSendRaw_Click(object sender, RoutedEventArgs e)
        {
            string c = EntFanCmd.Text.Trim();
            if (c.Length == 0) return;
            EntFanCmd.Text = "";
            _fan?.SendRaw(c);
        }

        private void EntFanCmd_KeyDown(object sender, System.Windows.Input.KeyEventArgs e)
        {
            if (e.Key == System.Windows.Input.Key.Enter)
            {
                BtnFanSendRaw_Click(sender, new RoutedEventArgs());
            }
        }

        // ===== 风机日志与回复解析 =====

        private int _fanLogLines;
        private void OnFanLog(string msg)
        {
            Dispatcher.BeginInvoke(new Action(() =>
            {
                var line = $"[{DateTime.Now:HH:mm:ss.fff}] {msg}\n";
                TxtFanLog.AppendText(line);
                _fanLogLines++;
                if (_fanLogLines > 200)
                {
                    var text = TxtFanLog.Text;
                    int keep = text.LastIndexOf('\n', text.Length / 2);
                    if (keep > 0) TxtFanLog.Text = text.Substring(keep + 1);
                    _fanLogLines = TxtFanLog.Text.Count(c => c == '\n');
                }
                TxtFanLog.ScrollToEnd();
            }));
        }

        private void OnFanReply(string line)
        {
            Dispatcher.BeginInvoke(new Action(() =>
            {
                // STATE=IDLE DUTY=0 TARGET=0 FREQ=.. RPM=.. CNT=.. PPR=..
                var m = System.Text.RegularExpressions.Regex.Match(
                    line, @"^STATE=(\S+)\s+DUTY=(\d+)\s+TARGET=(\d+)(?:\s+FREQ=(\d+)\s+RPM=(\d+)\s+CNT=(\d+))?");
                if (m.Success)
                {
                    string state = m.Groups[1].Value;
                    int duty = int.Parse(m.Groups[2].Value);
                    int tgt  = int.Parse(m.Groups[3].Value);
                    TxtFanState.Text = $"状态: {state}";
                    PbFanDuty.Value = duty;
                    TxtFanDutyState.Text = $"{duty}% / {tgt}%";
                    var color = state switch
                    {
                        "IDLE"     => Brushes.Gray,
                        "RUN"      => Brushes.SeaGreen,
                        "STOPPING" => Brushes.DarkOrange,
                        _          => Brushes.Gray
                    };
                    TxtFanState.Foreground = color;

                    // 白线 FG 测速（固件支持时才有这几个字段）
                    if (m.Groups[4].Success)
                    {
                        TxtPulseFreq.Text = $"频率 {m.Groups[4].Value} Hz";
                        TxtPulseRpm.Text  = $"转速 {m.Groups[5].Value} RPM";
                        TxtPulseCnt.Text  = $"脉冲 {m.Groups[6].Value} 个";
                        bool hasSignal = int.Parse(m.Groups[4].Value) > 0;
                        Brush sigColor = hasSignal ? Brushes.SeaGreen : Brushes.Gray;
                        TxtPulseFreq.Foreground = sigColor;
                        TxtPulseRpm.Foreground  = sigColor;
                        TxtPulseCnt.Foreground  = sigColor;
                    }
                    return;
                }
                // Sxx 设置确认：OK S80 TARGET=80 / OK S0 -> STOP
                // 让 TARGET= 可选，使 S0 停止回复也能触发状态刷新
                var m2 = System.Text.RegularExpressions.Regex.Match(line, @"^OK S(\d+)(?:\s+TARGET=(\d+))?");
                if (m2.Success)
                {
                    ThreadPool.QueueUserWorkItem(_ =>
                    {
                        Thread.Sleep(150);
                        try { _fan?.SendStatus(); } catch { }
                    });
                }
            }));
        }

        private void ResetFanUi()
        {
            TxtFanState.Text = "状态: --";
            TxtFanState.Foreground = Brushes.Gray;
            PbFanDuty.Value = 0;
            TxtFanDutyState.Text = "--% / --%";
            if (TxtPulseFreq != null)
            {
                TxtPulseFreq.Text = "频率 -- Hz";
                TxtPulseRpm.Text  = "转速 -- RPM";
                TxtPulseCnt.Text  = "脉冲 -- 个";
                TxtPulseFreq.Foreground = Brushes.Gray;
                TxtPulseRpm.Foreground  = Brushes.Gray;
                TxtPulseCnt.Foreground  = Brushes.Gray;
            }
        }

        protected override void OnClosing(System.ComponentModel.CancelEventArgs e)
        {
            StopCommThread();
            _mb?.Dispose();
            try
            {
                if (_fan != null)
                {
                    _fan.ReplyReceived -= OnFanReply;
                    _fan.DebugLog -= OnFanLog;
                    _fan.Dispose();
                }
            } catch { }
            base.OnClosing(e);
        }
    }
}
