using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;

namespace GamepadSpeedController
{
    /// <summary>
    /// 风机板 USB CDC 文本协议客户端
    /// 协议：
    ///   发送 (每行以 \n 结尾):
    ///     START        启动到 30% 默认占空比
    ///     S0..S100     设置目标占空比 (S0=停止)
    ///     STOP         斜坡减速到 0% 并断电
    ///     STATUS       查询当前状态
    ///     HELP         命令列表
    ///
    ///   回复 (每行一条):
    ///     OK START / OK STOP / OK Sxx TARGET=xx / OK S0 -> STOP
    ///     STATE=xxx DUTY=n TARGET=n      (IDLE / RUN / STOPPING)
    ///     CMD: START / S0-100 / STOP / STATUS / HELP
    ///     ERR UNKNOWN CMD / ERR BAD_NUM / ERR NUM_TOO_LONG
    /// </summary>
    public sealed class FanSerialClient : IDisposable
    {
        private readonly SerialPort _port;
        private readonly object _lock = new();
        private readonly StringBuilder _rxBuffer = new();

        /// <summary>收到一条完整的异步回复 (已去掉 \r\n)</summary>
        public event Action<string>? ReplyReceived;

        /// <summary>调试/状态日志</summary>
        public event Action<string>? DebugLog;

        public bool IsOpen => _port.IsOpen;
        public string PortName => _port.PortName;
        public int BaudRate => _port.BaudRate;

        public FanSerialClient(string portName, int baudRate = 115200)
        {
            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One)
            {
                ReadTimeout  = 200,
                WriteTimeout = 500,
                Encoding     = Encoding.ASCII,
                NewLine      = "\n"
            };
            _port.DataReceived += Port_DataReceived;
            _port.ErrorReceived += (_, e) => Log($"串口错误：{e.EventType}");
            _port.Open();
            Log($"串口已打开：{portName}，波特率 {baudRate}");
        }

        private void Log(string msg) => DebugLog?.Invoke(msg);

        // ========= 日志中文化：把协议命令/回复翻译成通俗中文 =========

        /// <summary>把发出的协议命令翻译成通俗中文</summary>
        private static string DescribeCmd(string cmd)
        {
            switch (cmd)
            {
                case "START":  return "启动风机（默认目标 30%）";
                case "STOP":   return "停止风机（减速到 0% 后断电）";
                case "STATUS": return "查询当前状态";
                case "HELP":   return "查询命令帮助";
                default:
                    // S0..S100 设置目标占空比
                    if (cmd.Length > 1 && cmd[0] == 'S' &&
                        int.TryParse(cmd.AsSpan(1), out int duty))
                    {
                        return duty == 0
                            ? "设置目标占空比 0%（风机停止）"
                            : $"设置目标占空比 {duty}%";
                    }
                    return $"原始命令：{cmd}";
            }
        }

        /// <summary>把设备回复行翻译成通俗中文；无法识别的保留原文便于排查</summary>
        private static string DescribeReply(string line)
        {
            // 状态上报：STATE=RUN DUTY=45 TARGET=30 FREQ=1234 RPM=37000 CNT=5678 PPR=2
            var sm = Regex.Match(line, @"^STATE=(\S+)\s+DUTY=(\d+)\s+TARGET=(\d+)(?:\s+FREQ=(\d+)\s+RPM=(\d+)\s+CNT=(\d+))?");
            if (sm.Success)
            {
                string state = sm.Groups[1].Value switch
                {
                    "IDLE"     => "空闲",
                    "RUN"      => "运行中",
                    "STOPPING" => "正在减速停止",
                    _          => sm.Groups[1].Value
                };
                string desc = $"当前状态：{state}（实际 {sm.Groups[2].Value}%，目标 {sm.Groups[3].Value}%）";
                // 白线 FG 测速（旧固件没有这几个字段时 Groups[4] 为空）
                if (sm.Groups[4].Success)
                {
                    desc += $"；白线 {sm.Groups[4].Value}Hz，转速 {sm.Groups[5].Value}RPM，脉冲 {sm.Groups[6].Value}个";
                }
                return desc;
            }

            // 占空比设置确认：OK S30 TARGET=30 / OK S0 -> STOP
            var okm = Regex.Match(line, @"^OK S(\d+)(?:\s+TARGET=(\d+))?");
            if (okm.Success)
            {
                return okm.Groups[1].Value == "0"
                    ? "已确认：目标占空比 0%，风机减速停止"
                    : $"已确认：目标占空比设为 {okm.Groups[2].Value}%";
            }

            switch (line)
            {
                case "OK START":         return "已确认：风机启动";
                case "OK STOP":          return "已确认：风机停止";
                case "ERR UNKNOWN CMD":  return "错误：设备不认识该命令";
                case "ERR BAD_NUM":      return "错误：占空比数值格式不正确";
                case "ERR NUM_TOO_LONG": return "错误：数值位数过长（占空比应在 0~100 之间）";
            }

            // 帮助信息：CMD: START / S0-100 / ...
            if (line.StartsWith("CMD:"))
                return "帮助：支持的命令 " + line.Substring(4).Trim();

            return $"未识别回复：{line}";
        }

        // ========= 发送命令 =========

        /// <summary>发送任意命令 (自动附加 \n)</summary>
        public void SendRaw(string cmd)
        {
            if (string.IsNullOrEmpty(cmd)) return;
            lock (_lock) _port.WriteLine(cmd);
            Log($"发送：{DescribeCmd(cmd)}");
        }

        public void SendStart()  => SendRaw("START");
        public void SendStop()   => SendRaw("STOP");
        public void SendStatus() => SendRaw("STATUS");
        public void SendHelp()   => SendRaw("HELP");

        /// <summary>设置占空比 (0~100，自动钳位)</summary>
        public void SendDuty(int duty)
        {
            if (duty < 0) duty = 0;
            if (duty > 100) duty = 100;
            SendRaw($"S{duty}");
        }

        // ========= 异步接收 =========

        private void Port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                int n = _port.BytesToRead;
                if (n <= 0) return;
                byte[] buf = new byte[n];
                int got = _port.Read(buf, 0, n);
                if (got <= 0) return;
                string text = Encoding.ASCII.GetString(buf, 0, got);
                DispatchText(text);
            }
            catch (Exception ex)
            {
                Log($"接收数据异常：{ex.Message}");
            }
        }

        /// <summary>按 \n 切行，不完整尾部留到下次拼接</summary>
        private void DispatchText(string text)
        {
            string whole;
            lock (_rxBuffer)
            {
                _rxBuffer.Append(text);
                whole = _rxBuffer.ToString();
                _rxBuffer.Clear();
            }

            int start = 0;
            while (true)
            {
                int idx = whole.IndexOf('\n', start);
                if (idx < 0)
                {
                    if (start < whole.Length)
                    {
                        lock (_rxBuffer) _rxBuffer.Append(whole, start, whole.Length - start);
                    }
                    break;
                }
                int end = idx;
                if (end > start && whole[end - 1] == '\r') end--;
                string line = whole.Substring(start, end - start);
                start = idx + 1;
                if (line.Length > 0)
                {
                    try { ReplyReceived?.Invoke(line); } catch { }
                    Log($"收到：{DescribeReply(line)}");
                }
            }
        }

        public void Dispose()
        {
            try
            {
                _port.DataReceived -= Port_DataReceived;
                if (_port.IsOpen) _port.Close();
            }
            catch { }
            _port.Dispose();
        }
    }
}
