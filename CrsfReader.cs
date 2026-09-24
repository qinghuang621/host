using System.IO.Ports;

namespace GamepadSpeedController
{
    /// <summary>
    /// CRSF (TBS Crossfire) 协议解析器，用于 ELRS 接收机。
    ///
    /// 硬件接线（ELRS Nano 接收机 → USB-TTL 转换器）：
    ///   ELRS RX  ────  USB-TTL 的 RX（TX/RX 交叉！）
    ///   ELRS 5V  ────  USB-TTL 5V
    ///   ELRS GND ────  USB-TTL GND
    /// ⚠️ 必须用 3.3V 逻辑 USB-TTL（CH340/FT232/CP2102），不能直接接 RS232 ±12V。
    ///
    /// 串口参数：420000 baud, 8N1（CRSF 协议硬性要求）。
    ///
    /// 帧格式：0xEE &lt;len&gt; &lt;type&gt; &lt;payload...&gt; &lt;crc&gt;
    ///   len   = type(1) + payload + crc(1)，合法值 2..63
    ///   crc   = 累加和（type 到 payload 末尾）取反
    ///   type=0x16  通道帧：11 通道 × 11 bit，22 字节 payload（每通道 172~1811，中点 992）
    ///   type=0x08  链路统计帧：12 字节 payload（含 RSSI/丢包计数）
    ///
    /// ELRS 高速模式 500Hz = 每 2ms 一帧，状态机以字节流方式解析，不怕丢字节重对齐。
    /// </summary>
    public class CrsfReader : IDisposable
    {
        private const byte SyncByte     = 0xC8;  // CRSF 帧头（不是 0xEE！那是老 Crossfire）
        private const byte TypeChannels = 0x16;
        private const byte TypeLinkStat = 0x08;

        // CRSF 通道值范围（11 bit），中点 992，向两端 ±819
        private const int ChMin = 172;
        private const int ChMax = 1811;
        private const int ChMid = 992;
        private const float ChScale = 819.0f;

        // 失联阈值
        public const int LinkTimeoutMs = 1000;   // 1 秒没收到任何 CRSF 帧
        public const int RssiThreshold = -100;   // dBm，低于此值视为链路不可用

        private readonly SerialPort _port;
        private readonly byte[] _frame = new byte[64]; // len ≤ 63 + sync，64 足够
        private int _frameLen;
        private ParseState _state = ParseState.WaitSync;

        // 暴露给外部的状态字段（volatile，多线程可见）
        public volatile float[] Channels = new float[16]; // 实际前 11 个有效，归一化到 -1..+1
        public volatile int RssiDbm = 0;                    // 上行 RSSI（dBm），0=未知（还没收到 Link Statistics 帧）
        public volatile int FrameLoss;                     // 累计丢帧计数
        public DateTime LastFrameAt = DateTime.MinValue;

        // 测试统计字段（BtnCrsfTest_Click 用，可重置归零）
        public long TotalBytes;       // 累计接收字节数
        public long SyncBytes;        // 累计 0xEE 同步字节数
        public long ValidFrames;     // 累计通过 CRC 校验的帧数
        public long BadFrames;       // 累计 CRC 失败的帧数
        public int LastBadFrameLen;  // 最近一帧 CRC 失败时的 len 字段值
        public byte LastBadFrameType;// 最近一帧 CRC 失败时的 type 字段值
        public byte[] LastBadFrame = new byte[64]; // 最近一帧完整数据（从 0xEE 开始）

        // 原始字节环形缓冲区（用于 hex dump 诊断，最多保留 512 字节）
        private readonly byte[] _rawRing = new byte[512];
        private int _rawWritePos;
        public int RawBytesQueued => (int)Math.Min(TotalBytes, 512);

        public bool IsLinkLost
        {
            get
            {
                // 从未收到过帧 → 不算失联（刚启动还没对上频 / 刚创建的 reader 还没 Poll 到帧）
                if (LastFrameAt == DateTime.MinValue) return false;
                return (DateTime.UtcNow - LastFrameAt).TotalMilliseconds > LinkTimeoutMs
                    || (RssiDbm != 0 && RssiDbm < RssiThreshold);
            }
        }

        private enum ParseState { WaitSync, ReadLen, ReadBody }

        public CrsfReader(string portName)
        {
            // ELRS 接收机对外（飞控/PC）的 CRSF 波特率：默认 420000。
            // SBUS 模式才是 115200 —— 所以如果 ELRS 设置为 SBUS 输出，要改成 115200 + 解析 SBUS 帧。
            _port = new SerialPort(portName, 420000, Parity.None, 8, StopBits.One)
            {
                ReadBufferSize = 4096,
                ReadTimeout = 500,
            };
            _port.Open();
            try { _port.DiscardInBuffer(); } catch { }
        }

        /// <summary>
        /// 在 CommLoop tick 里调用：把串口驱动 buffer 里所有可用字节喂给状态机。
        /// 50ms tick 一次，期间堆 25 帧 × 26 字节 = 650 字节，buffer 装得下。
        /// </summary>
        public void Poll()
        {
            int avail;
            try { avail = _port.BytesToRead; }
            catch { return; }
            if (avail <= 0) return;

            byte[] buf = new byte[avail];
            int n = _port.Read(buf, 0, avail);
            TotalBytes += n;
            for (int i = 0; i < n; i++)
            {
                _rawRing[_rawWritePos] = buf[i];
                _rawWritePos = (_rawWritePos + 1) % _rawRing.Length;
                FeedByte(buf[i]);
            }
        }

        /// <summary>
        /// 返回原始接收字节的 hex dump（最多 128 字节，环形缓冲区中最老的）。
        /// 用于诊断：判断收到的到底是什么协议（CRSF/SBUS/DSMX/全零/固定值等）。
        /// </summary>
        public string DumpRaw(int count = 128)
        {
            int avail = (int)Math.Min(TotalBytes, _rawRing.Length);
            if (avail == 0) return "(无数据)";
            int skip = 0; // 环形缓冲最老的位置 = 写指针（因为是回绕的）
            if (avail < _rawRing.Length) skip = 0; // 没填满，从 0 开始
            else skip = _rawWritePos; // 填满了，写指针指向下一个，即最老位置
            count = Math.Min(count, avail);
            var sb = new System.Text.StringBuilder();
            for (int i = 0; i < count; i++)
            {
                int idx = (skip + i) % _rawRing.Length;
                sb.Append($"{_rawRing[idx]:X2} ");
                if ((i + 1) % 16 == 0) sb.AppendLine();
            }
            return sb.ToString();
        }

        private void FeedByte(byte b)
        {
            switch (_state)
            {
                case ParseState.WaitSync:
                    if (b == SyncByte)
                    {
                        _frame[0] = SyncByte;
                        _frameLen = 1;
                        SyncBytes++;
                        _state = ParseState.ReadLen;
                    }
                    break;

                case ParseState.ReadLen:
                    // len = type(1) + payload + crc(1)，合法范围 2..63
                    if (b < 2 || b > 63) { _state = ParseState.WaitSync; break; }
                    _frame[1] = b;
                    _frameLen = 2;
                    _state = ParseState.ReadBody;
                    break;

                case ParseState.ReadBody:
                    // 要读的总字节数 = 2 (sync + len 字段) + len (type + payload + crc)
                    int want = _frame[1] + 2;
                    _frame[_frameLen++] = b;
                    if (_frameLen >= want)
                    {
                        // 帧收齐，校验 CRC
                        // CRSF CRC = CRC-8 (poly=0xD5, init=0x00)
                        // 计算范围：type(_frame[2]) + payload，不含 sync/len/crc 本身
                        byte crcCalc = Crc8D5(_frame, 2, _frameLen - 3);
                        byte crcRx = _frame[_frameLen - 1];

                        if (crcCalc == crcRx)
                        {
                            ValidFrames++;
                            HandleFrame(_frame[2], _frame, 3, _frame[1] - 2);
                        }
                        else
                        {
                            BadFrames++;
                            LastBadFrameLen = _frame[1];
                            LastBadFrameType = _frame[2];
                            int copyLen = Math.Min(_frameLen, LastBadFrame.Length);
                            Buffer.BlockCopy(_frame, 0, LastBadFrame, 0, copyLen);
                        }
                        _state = ParseState.WaitSync;
                    }
                    break;
            }
        }

        /// <summary>
        /// CRSF 使用的 CRC-8：多项式 x^8+x^6+x^4+x^3+x^2+1 (0xD5)，初始值 0。
        /// 等价于 CRSF 协议文档里的 crc8() 函数。
        /// </summary>
        private static byte Crc8D5(byte[] data, int offset, int count)
        {
            byte crc = 0;
            for (int i = 0; i < count; i++)
            {
                crc ^= data[offset + i];
                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 0x80) != 0)
                        crc = (byte)((crc << 1) ^ 0xD5);
                    else
                        crc <<= 1;
                }
            }
            return crc;
        }

        private void HandleFrame(byte type, byte[] buf, int off, int len)
        {
            LastFrameAt = DateTime.UtcNow;

            if (type == TypeChannels && len >= 22)
                DecodeChannels(buf, off);
            else if (type == TypeLinkStat && len >= 12)
                DecodeLinkStats(buf, off);
        }

        /// <summary>
        /// 11 通道 × 11 bit 紧凑打包，低字节在前。
        /// 用 bit 流方式解最稳：每取满 11 bit 输出一个通道。
        /// </summary>
        private void DecodeChannels(byte[] buf, int off)
        {
            uint bits = 0;
            int bitsAvail = 0;
            int p = off;
            // 取 11 通道 = 121 bit = 15.125 字节，从 22 字节 payload 里取 16 字节足够
            for (int ch = 0; ch < 11; ch++)
            {
                while (bitsAvail < 11 && p < off + 22)
                {
                    bits |= (uint)buf[p++] << bitsAvail;
                    bitsAvail += 8;
                }
                int raw = (int)(bits & 0x7FF);
                bits >>= 11;
                bitsAvail -= 11;
                Channels[ch] = Normalize(raw);
            }
        }

        private void DecodeLinkStats(byte[] buf, int off)
        {
            // payload[0] = uplink RSSI（signed int8, dBm，0 表示未知）
            int r = (sbyte)buf[off + 0];
            if (r != 0) RssiDbm = r;
            // payload[2..3] = 累计丢帧计数（uint16, little-endian）
            FrameLoss = buf[off + 2] | (buf[off + 3] << 8);
        }

        private static float Normalize(int raw)
        {
            if (raw < ChMin) raw = ChMin;
            if (raw > ChMax) raw = ChMax;
            return (raw - ChMid) / ChScale;
        }

        public void Dispose()
        {
            try { _port.Close(); } catch { }
        }
    }
}
