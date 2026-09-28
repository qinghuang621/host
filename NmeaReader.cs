using System.Globalization;
using System.IO.Ports;
using System.Text;

namespace GamepadSpeedController
{
    /// <summary>
    /// NMEA 0183 解析器，用于 GPS 模块（u-blox NEO-6M / 7N / M8N 等，带 EEPROM 的 APM 兼容模块）。
    ///
    /// 硬件接线（GPS 模块 → USB-TTL 转换器，与 ELRS 同一套接线方式）：
    ///   GPS.TX  ────  USB-TTL 的 RX（TX/RX 交叉！）
    ///   GPS.RX  ────  USB-TTL 的 TX（只读定位数据时可不接）
    ///   GPS.VCC ────  3.3V（数据手册 VCC 规格 2.7~3.6V，典型 3.0V）
    ///                 ⚠️ 裸 NEO-M8N 模块接 5V 会超规损坏（输入引脚上限也是 VCC）；
    ///                    只有带 LDO 稳压的底板（APM / GY-NEO8MV2 等）才允许接 5V。
    ///   GPS.GND ────  USB-TTL GND（必须共地）
    ///
    /// 串口参数：默认 9600 8N1（数据手册 Table 14，autobauding 默认关闭），
    ///           即模块不会自适应主机波特率，两端必须显式一致；部分模块被配置成 38400 / 115200。
    ///
    /// 数据格式：ASCII 文本行，每行 `$<字段,字段,...>*<2 位 hex 校验>` + CRLF
    ///   校验 = '$' 与 '*' 之间所有字符的 XOR
    ///
    /// 关键句子（前缀可能是 GP/GN/GL/BD，多星座并存，所以按后缀匹配）：
    ///   GGA  定位质量 / 已用卫星数 / HDOP / 海拔
    ///   RMC  经纬度 / 地速 / 航向 / UTC 日期时间
    ///   GSA  定位类型（1=无定位 2=2D 3=3D）/ PDOP / VDOP
    ///   GSV  可见卫星与 SNR
    /// </summary>
    public class NmeaReader : IDisposable
    {
        public const int DefaultBaud = 9600;   // NEO-6M / M8N 出厂默认波特率
        public const int LinkTimeoutMs = 3000; // GPS 通常 1Hz 输出，3 秒无任何句子视为掉线

        private readonly SerialPort _port;
        private readonly StringBuilder _lineBuf = new(256);

        // ---- 定位状态（后台线程写、UI 线程读）----
        // 说明：double 不能加 volatile（C# 语言限制）；即便发生撕裂读也只影响单帧显示，无安全影响。
        public volatile bool HasFix;            // GGA 定位质量 > 0（或 RMC 状态 = A）
        public volatile int FixQuality;         // GGA：0=无效 1=GPS 2=DGPS 4=RTK 5=RTK浮动 6=估算
        public volatile int FixType;            // GSA：1=无定位 2=2D 3=3D
        public volatile int SatellitesUsed;     // GGA：参与解算的卫星数
        public volatile int SatellitesInView;   // GSV：可见卫星数
        public double Latitude;                 // 十进制度，北正
        public double Longitude;                // 十进制度，东正
        public double AltitudeM;                // 海拔（相对 MSL）米
        public double GeoidM;                   // 大地水准面差距（米）
        public double Hdop, Pdop, Vdop;         // 精度因子
        public double SpeedKnots;               // 地速（节，1 节 = 1.852 km/h）
        public double CourseDeg;                // 航向（度，相对真北）
        public volatile string UtcTime = "";    // hhmmss.sss
        public volatile string UtcDate = "";    // ddmmyy
        public volatile string LastSentence = ""; // 最近一条通过校验的完整 NMEA 行
        public DateTime LastSentenceAt = DateTime.MinValue; // 最近收到句子的时刻（UTC）
        public DateTime LastFixAt = DateTime.MinValue;      // 最近一次有效定位时刻（UTC）

        // ---- 自检统计（可由 ResetStats 归零）----
        public long TotalBytes;       // 累计接收字节数
        public long TotalSentences;   // 累计按行切出的句子数（含格式错误）
        public long ValidSentences;   // 累计通过 $ / * 校验的句子数
        public long BadChecksum;      // 校验失败的句子数
        public long BadFormat;        // 格式非法（无 $ / 无 * / 太短）的句子数

        private readonly Dictionary<string, long> _typeCounts = new(); // 后缀 → 计数（GGA/RMC/GSA/GSV/其它）
        private readonly Dictionary<int, int> _snr = new();            // PRN → SNR(dBHz)
        private readonly Dictionary<string, string> _lastByType = new(); // 后缀 → 最近一条原文

        // 原始字节环形缓冲区（hex dump 诊断用，最多保留 512 字节）
        private readonly byte[] _rawRing = new byte[512];
        private int _rawWritePos;

        /// <summary>启动初期还没收到任何句子时不判失联（与 CrsfReader 一致的语义）。</summary>
        public bool IsLinkLost
        {
            get
            {
                if (LastSentenceAt == DateTime.MinValue) return false;
                return (DateTime.UtcNow - LastSentenceAt).TotalMilliseconds > LinkTimeoutMs;
            }
        }

        public NmeaReader(string portName, int baud = DefaultBaud)
        {
            _port = new SerialPort(portName, baud, Parity.None, 8, StopBits.One)
            {
                ReadBufferSize = 4096,
                ReadTimeout = 500,
            };
            _port.Open();
            try { _port.DiscardInBuffer(); } catch { }
        }

        /// <summary>
        /// 由轮询线程调用：把串口驱动 buffer 里的字节全部喂给行解析状态机。
        /// GPS 9600 baud ≈ 960 字节/秒，100ms tick 只有约 100 字节，buffer 完全够用。
        /// </summary>
        public void Poll()
        {
            int avail;
            try { avail = _port.BytesToRead; }
            catch { return; }
            if (avail <= 0) return;

            byte[] buf = new byte[avail];
            int n;
            try { n = _port.Read(buf, 0, avail); }
            catch { return; }

            TotalBytes += n;
            for (int i = 0; i < n; i++)
            {
                byte b = buf[i];
                _rawRing[_rawWritePos] = b;
                _rawWritePos = (_rawWritePos + 1) % _rawRing.Length;

                if (b == (byte)'\n')
                {
                    string line = _lineBuf.ToString().TrimEnd('\r');
                    _lineBuf.Clear();
                    if (line.Length > 0) ProcessLine(line);
                }
                else if (b == (byte)'\r')
                {
                    // 行尾 CR，忽略
                }
                else if (_lineBuf.Length < 200)
                {
                    _lineBuf.Append((char)b);
                }
                else
                {
                    // 超长且无换行 → 非 NMEA 数据（波特率不对/二进制流），丢弃重新同步
                    _lineBuf.Clear();
                }
            }
        }

        /// <summary>把统计字段归零（自检前调用）。</summary>
        public void ResetStats()
        {
            TotalBytes = 0;
            TotalSentences = 0;
            ValidSentences = 0;
            BadChecksum = 0;
            BadFormat = 0;
            LastSentenceAt = DateTime.MinValue;
            LastFixAt = DateTime.MinValue;
            lock (_typeCounts) _typeCounts.Clear();
            lock (_snr) _snr.Clear();
            lock (_lastByType) _lastByType.Clear();
        }

        /// <summary>句子类型计数快照（GGA/RMC/GSA/GSV/其它）。</summary>
        public List<KeyValuePair<string, long>> GetTypeCountsSnapshot()
        {
            lock (_typeCounts)
                return _typeCounts.OrderByDescending(kv => kv.Value).ToList();
        }

        /// <summary>最近一条指定类型句子的原文（用于界面展示原始 NMEA）。</summary>
        public string GetLastByType(string type)
        {
            lock (_lastByType)
                return _lastByType.TryGetValue(type, out var s) ? s : "";
        }

        /// <summary>可见卫星 SNR 文本（按 PRN 升序），如 "12:35dB 15:28dB"。</summary>
        public string GetSatSnrText()
        {
            lock (_snr)
            {
                if (_snr.Count == 0) return "(无)";
                var sb = new StringBuilder();
                foreach (var kv in _snr.OrderBy(k => k.Key))
                {
                    if (sb.Length > 0) sb.Append("  ");
                    sb.Append($"PRN{kv.Key}:{(kv.Value > 0 ? kv.Value + "dB" : "--")}");
                }
                return sb.ToString();
            }
        }

        /// <summary>返回原始接收字节的 hex dump（最多 count 字节，环形缓冲区中最老的）。</summary>
        public string DumpRaw(int count = 128)
        {
            int avail = (int)Math.Min(TotalBytes, _rawRing.Length);
            if (avail == 0) return "(无数据)";
            int skip = avail < _rawRing.Length ? 0 : _rawWritePos;
            count = Math.Min(count, avail);
            var sb = new StringBuilder();
            for (int i = 0; i < count; i++)
            {
                int idx = (skip + i) % _rawRing.Length;
                sb.Append($"{_rawRing[idx]:X2} ");
                if ((i + 1) % 16 == 0) sb.AppendLine();
            }
            return sb.ToString();
        }

        // ==================== 行解析 ====================

        private void ProcessLine(string line)
        {
            TotalSentences++;

            if (line.Length < 6 || line[0] != '$')
            {
                BadFormat++;
                return;
            }

            int star = line.LastIndexOf('*');
            if (star < 0 || line.Length < star + 3)
            {
                BadFormat++;
                return;
            }

            // NMEA 校验：'$' 与 '*' 之间所有字符逐字节 XOR
            byte calc = 0;
            for (int i = 1; i < star; i++) calc ^= (byte)line[i];
            if (!byte.TryParse(line.Substring(star + 1, 2), NumberStyles.HexNumber,
                    CultureInfo.InvariantCulture, out byte rx) || calc != rx)
            {
                BadChecksum++;
                return;
            }

            ValidSentences++;
            LastSentenceAt = DateTime.UtcNow;
            LastSentence = line;

            // 取 talker + 类型（如 "GPGGA" / "GNGGA" / "GLGSV"），按后 3 位分类
            int comma = line.IndexOf(',');
            if (comma < 3) return;
            string tag = line.Substring(1, comma - 1);
            if (tag.Length < 3) return;
            string type = tag.Substring(tag.Length - 3);

            lock (_typeCounts)
            {
                _typeCounts.TryGetValue(type, out long c);
                _typeCounts[type] = c + 1;
            }
            lock (_lastByType) _lastByType[type] = line;

            // 字段体（不含 $ 前缀和 * 校验后缀）
            string body = star > comma ? line.Substring(comma + 1, star - comma - 1) : "";
            string[] f = body.Split(',');

            switch (type)
            {
                case "GGA": ParseGga(f); break;
                case "RMC": ParseRmc(f); break;
                case "GSA": ParseGsa(f); break;
                case "GSV": ParseGsv(f); break;
            }
        }

        /// <summary>$xxGGA,UTC,lat,N,lon,E,quality,sats,hdop,alt,M,geoid,M,age,station*CS</summary>
        private void ParseGga(string[] f)
        {
            if (f.Length < 11) return; // 至少要 11 个字段才能安全访问 f[10](GeoidM)
            UtcTime = f[0];
            if (TryParseLatLon(f[1], f[2], out double lat)) Latitude = lat;
            if (TryParseLatLon(f[3], f[4], out double lon)) Longitude = lon;
            FixQuality = ParseInt(f[5]);
            SatellitesUsed = ParseInt(f[6]);
            Hdop = ParseDouble(f[7]);
            AltitudeM = ParseDouble(f[8]);
            GeoidM = ParseDouble(f[10]); // NMEA 4.0 起 f[10] 必存在；上面的 guard 已保证

            bool fix = FixQuality > 0;
            HasFix = fix;
            if (fix) LastFixAt = DateTime.UtcNow;
        }

        /// <summary>$xxRMC,UTC,A/V,lat,N,lon,E,speedKn,course,date,magVar,...*CS</summary>
        private void ParseRmc(string[] f)
        {
            if (f.Length < 9) return;
            if (!string.IsNullOrEmpty(f[0])) UtcTime = f[0];
            if (TryParseLatLon(f[2], f[3], out double lat)) Latitude = lat;
            if (TryParseLatLon(f[4], f[5], out double lon)) Longitude = lon;
            SpeedKnots = ParseDouble(f[6]);
            CourseDeg = ParseDouble(f[7]);
            UtcDate = f[8];

            // RMC 状态 A=有效 / V=无效；只用于置位，清零交给更权威的 GGA 定位质量
            if (f[1] == "A")
            {
                HasFix = true;
                LastFixAt = DateTime.UtcNow;
            }
        }

        /// <summary>$xxGSA,M/A,fixType,prn×12,pdop,hdop,vdop*CS</summary>
        private void ParseGsa(string[] f)
        {
            if (f.Length < 17) return;
            FixType = ParseInt(f[1]);
            Pdop = ParseDouble(f[14]);
            Vdop = ParseDouble(f[16]);
        }

        /// <summary>$xxGSV,total,index,inView,[prn,elev,azim,snr]×n*CS</summary>
        private void ParseGsv(string[] f)
        {
            if (f.Length < 3) return;
            SatellitesInView = ParseInt(f[2]);

            // 每轮 GSV 的首句（index=1）到来时清空上一轮 SNR，避免消失的卫星残留
            if (ParseInt(f[1]) == 1)
            {
                lock (_snr) _snr.Clear();
            }

            for (int i = 3; i + 3 < f.Length; i += 4)
            {
                int prn = ParseInt(f[i]);
                int snr = ParseInt(f[i + 3]);
                if (prn > 0)
                {
                    lock (_snr) _snr[prn] = snr;
                }
            }
        }

        // ==================== 字段工具 ====================

        /// <summary>
        /// ddmm.mmmm / dddmm.mmmm → 十进制度。南纬/西经取负。
        /// 度位数由小数点位置反推（纬度 2 位、经度 3 位），避免按长度硬编码。
        /// </summary>
        private static bool TryParseLatLon(string val, string hemi, out double deg)
        {
            deg = 0;
            if (string.IsNullOrEmpty(val)) return false;
            int dot = val.IndexOf('.');
            if (dot < 3) return false;

            if (!double.TryParse(val.Substring(0, dot - 2), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out double d)) return false;
            if (!double.TryParse(val.Substring(dot - 2), NumberStyles.Float,
                    CultureInfo.InvariantCulture, out double m)) return false;

            deg = d + m / 60.0;
            if (hemi == "S" || hemi == "W") deg = -deg;
            return true;
        }

        private static int ParseInt(string s)
            => int.TryParse(s, NumberStyles.Integer, CultureInfo.InvariantCulture, out int v) ? v : 0;

        private static double ParseDouble(string s)
            => double.TryParse(s, NumberStyles.Float, CultureInfo.InvariantCulture, out double v) ? v : 0;

        public void Dispose()
        {
            try { _port.Close(); } catch { }
            _port.Dispose();
        }
    }
}