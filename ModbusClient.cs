using System.IO;
using System.IO.Ports;
using System.Text;
using System.Threading;

namespace GamepadSpeedController
{
    /// <summary>
    /// Modbus RTU 主站 - 使用最简串口读写方式，对齐 Modbus Poll 的行为。
    /// </summary>
    public class ModbusClient : IDisposable
    {
        private const byte SlaveId = 1;
        private const ushort RegEnable   = 0x0000;
        private const ushort RegMotorVel = 0x000A;
        /* 车体速度命令块的**起始地址**（协议地址 20 = 0x0014）。
         * ⚠️ 命名说明：0x0014 在固件里是 **VY_HI**（前后轴），不是 VX_HI。
         * 6 个寄存器依次为 VY 高/低、VX 高/低、WZ 高/低，占 0x0014~0x0019。
         * 因此这里命名为"速度块基址"，避免与固件的 VX（0x0016）混淆。
         * 详见 接口文档.md §5.6「坐标与符号约定」。 */
        private const ushort RegBodyVelBase = 0x0014;
        private const ushort RegMotorStat = 0x0064;

        private readonly SerialPort _port;
        private readonly object _lock = new();

        /// <summary>调试日志输出事件</summary>
        public event Action<string>? DebugLog;

        public bool IsOpen => _port.IsOpen;

        public ModbusClient(string portName, int baudRate = 115200)
        {
            _port = new SerialPort(portName, baudRate, Parity.None, 8, StopBits.One);
            _port.ReadTimeout = 500;
            _port.WriteTimeout = 500;
            _port.Open();
            Log($"Opened {portName} @ {baudRate}");
        }

        private void Log(string msg) => DebugLog?.Invoke(msg);

        /// <summary>
        /// 下发车体速度（功能码 0x10，写 0x0014~0x0019 共 6 个寄存器）。
        /// 坐标约定与固件 running/kinematics.c 完全一致：
        ///     x = 右+, y = 前+, wz = 逆时针+
        /// 上位机与固件用**同一套**约定，不需要做任何坐标交换。
        /// </summary>
        public void SendVelocity(float vx, float vy, float wz)
        {
            ushort[] regs = new ushort[6];
            WriteFloat(regs, 0, vy);   // → 0x0014 = VY_HI  前后轴（前+），驱动电机 2/4
            WriteFloat(regs, 2, vx);   // → 0x0016 = VX_HI  左右轴（右+），驱动电机 1/3
            WriteFloat(regs, 4, wz);   // → 0x0018 = WZ_HI  自转（逆时针+）
            WriteMultipleRegisters(RegBodyVelBase, regs);
        }

        public void SetMotorsEnabled(bool enabled)
        {
            ushort v = (ushort)(enabled ? 1 : 0);
            WriteMultipleRegisters(RegEnable, new ushort[] { v, v, v, v });
        }

        public void EmergencyStop()
        {
            try { SendVelocity(0f, 0f, 0f); } catch { }
            Thread.Sleep(30);
            try { SetMotorsEnabled(false); } catch { }
        }

        public float[] ReadMotorVelocities()
        {
            ushort[] regs = ReadHoldingRegisters(RegMotorVel, 8);
            float[] result = new float[4];
            for (int i = 0; i < 4; i++)
                result[i] = ReadFloat(regs[i * 2], regs[i * 2 + 1]);
            return result;
        }

        public MotorState[] ReadMotorStates()
        {
            ushort[] regs = ReadHoldingRegisters(RegMotorStat, 40);
            MotorState[] result = new MotorState[4];
            for (int i = 0; i < 4; i++)
            {
                int off = i * 10;
                result[i] = new MotorState
                {
                    Err       = regs[off + 0],
                    Pos       = ReadFloat(regs[off + 2], regs[off + 3]),
                    Vel       = ReadFloat(regs[off + 4], regs[off + 5]),
                    Torque    = ReadFloat(regs[off + 6], regs[off + 7]),
                    TempMos   = regs[off + 8],
                    TempRotor = regs[off + 9],
                };
            }
            return result;
        }

        // ========== 核心通信：对齐 Modbus Poll 的最简实现 ==========

        private ushort[] ReadHoldingRegisters(ushort startAddr, ushort count)
        {
            lock (_lock)
            {
                byte[] req = BuildReadRequest(startAddr, count);
                int expectedLen = 3 + count * 2 + 2;
                byte[] resp = SendAndReceive(req, expectedLen);
                if (resp == null)
                {
                    Log($"  READ FAIL: null response (addr={startAddr}, count={count})");
                    throw new TimeoutException($"读超时: addr={startAddr} count={count}");
                }

                Log($"  READ: addr={startAddr}, count={count}, respLen={resp.Length}");

                if (resp[0] != SlaveId || resp[1] != 0x03)
                {
                    Log($"  READ FAIL: slave={resp[0]} func={resp[1]} (expected {SlaveId}/0x03)");
                    throw new InvalidOperationException($"读响应错误: slave={resp[0]} func={resp[1]}");
                }

                // CRC 校验
                ushort rxCrc = (ushort)(resp[resp.Length - 1] << 8 | resp[resp.Length - 2]);
                ushort calcCrc = Crc16(resp, 0, resp.Length - 2);
                if (rxCrc != calcCrc)
                {
                    Log($"  READ FAIL: CRC mismatch (rx=0x{rxCrc:X4}, calc=0x{calcCrc:X4})");
                    // CRC 错误时仍然尝试解析（数据可能是对的，CRC 算错了）
                    // throw new InvalidOperationException("读CRC错误");
                }

                ushort[] result = new ushort[count];
                for (int i = 0; i < count; i++)
                    result[i] = (ushort)(resp[3 + 2 * i] << 8 | resp[3 + 2 * i + 1]);

                // 打印寄存器和 float 值用于调试
                if (count >= 4)
                {
                    Log($"  READ DATA: [{result[0]}, {result[1]}, {result[2]}, {result[3]}, ...]");
                    // 同时尝试解析 float
                    if (count >= 8 && startAddr == RegMotorVel)
                    {
                        // 这是命令速读取，打印 float
                        float f1 = ReadFloat(result[0], result[1]);
                        float f2 = ReadFloat(result[2], result[3]);
                        float f3 = ReadFloat(result[4], result[5]);
                        float f4 = ReadFloat(result[6], result[7]);
                        Log($"  CMD VEL: {f1:F3}, {f2:F3}, {f3:F3}, {f4:F3}");
                    }
                    else if (count >= 6 && startAddr == RegBodyVelBase)
                    {
                        // 顺序必须与寄存器一致：0x0014=VY、0x0016=VX、0x0018=WZ
                        // （原先这里把前两个标成了 vx/vy，与寄存器语义相反）
                        float vy = ReadFloat(result[0], result[1]);
                        float vx = ReadFloat(result[2], result[3]);
                        float wz = ReadFloat(result[4], result[5]);
                        Log($"  CMD: vx={vx:F3}(右+), vy={vy:F3}(前+), wz={wz:F3}");
                    }
                }

                return result;
            }
        }

        private void WriteMultipleRegisters(ushort startAddr, ushort[] regs)
        {
            lock (_lock)
            {
                byte[] req = BuildWriteRequest(startAddr, regs);
                byte[] resp = SendAndReceive(req, 8);
                if (resp == null)
                    throw new TimeoutException($"写超时: addr={startAddr} count={regs.Length}");

                if (resp[0] != SlaveId || resp[1] != 0x10)
                    throw new InvalidOperationException($"写响应错误: slave={resp[0]} func={resp[1]}");

                // CRC 校验
                ushort rxCrc = (ushort)(resp[7] << 8 | resp[6]);
                ushort calcCrc = Crc16(resp, 0, 6);
                if (rxCrc != calcCrc)
                    throw new InvalidOperationException($"写CRC错误");
            }
        }

        /// <summary>
        /// RS485 半双工通信：发 → 等 → 收 → 清残留。
        /// </summary>
        private byte[] SendAndReceive(byte[] req, int expectedLen)
        {
            // 1. 清空输入缓冲
            _port.DiscardInBuffer();

            // 2. 发送（不需要 Flush —— USB 串口驱动会自动发完）
            _port.Write(req, 0, req.Length);

            // 3. 等 RS485 方向切换 + STM32 响应
            //    小帧（≤8字节响应）：30ms 足够
            //    大帧（85字节响应）：需要 30ms 发 + ~75ms 传输 + STM32 处理
            int waitMs = Math.Max(30, expectedLen * 2 + 10);
            Thread.Sleep(waitMs);

            // 4. 读响应
            byte[] rx = ReadAvailable(expectedLen);

            // 5. 读完全部响应后，丢弃可能残留的回声/垃圾字节
            try { _port.DiscardInBuffer(); } catch { }

            if (rx == null)
                Log($"TIMEOUT: expected {expectedLen} bytes, got 0");
            else
                Log($"OK: {req.Length}→{rx.Length} bytes");

            return rx;
        }

        /// <summary>
        /// 读取串口数据。只靠"读到足够字节"或"总超时"结束。
        /// 不再用"10ms没数据就截断"——这是大帧丢失的根因。
        /// </summary>
        private byte[] ReadAvailable(int expectedLen)
        {
            var ms = new MemoryStream();
            int startTick = Environment.TickCount;

            while (ms.Length < expectedLen)
            {
                try
                {
                    int available = _port.BytesToRead;
                    if (available > 0)
                    {
                        byte[] buf = new byte[available];
                        _port.Read(buf, 0, available);
                        ms.Write(buf, 0, buf.Length);
                    }
                }
                catch { break; }

                if (ms.Length >= expectedLen) break;

                // 总超时 500ms
                if (Environment.TickCount - startTick > 500) break;

                Thread.Sleep(2);
            }

            // 日志：实际读到多少字节
            if (ms.Length < expectedLen)
                Log($"  ReadAvailable: got {ms.Length}/{expectedLen} bytes (TIMEOUT)");

            return ms.Length > 0 ? ms.ToArray() : null;
        }

        // ========== 帧构造 ==========

        private static byte[] BuildReadRequest(ushort addr, ushort count)
        {
            byte[] frame =
            {
                SlaveId, 0x03,
                (byte)(addr >> 8), (byte)(addr & 0xFF),
                (byte)(count >> 8), (byte)(count & 0xFF),
                0, 0
            };
            ushort crc = Crc16(frame, 0, 6);
            frame[6] = (byte)(crc & 0xFF);
            frame[7] = (byte)(crc >> 8);
            return frame;
        }

        private static byte[] BuildWriteRequest(ushort addr, ushort[] regs)
        {
            int n = regs.Length;
            byte[] frame = new byte[7 + n * 2 + 2];
            frame[0] = SlaveId;
            frame[1] = 0x10;
            frame[2] = (byte)(addr >> 8);
            frame[3] = (byte)(addr & 0xFF);
            frame[4] = (byte)(n >> 8);
            frame[5] = (byte)(n & 0xFF);
            frame[6] = (byte)(n * 2);
            for (int i = 0; i < n; i++)
            {
                frame[7 + 2 * i]     = (byte)(regs[i] >> 8);
                frame[7 + 2 * i + 1] = (byte)(regs[i] & 0xFF);
            }
            ushort crc = Crc16(frame, 0, 7 + n * 2);
            frame[7 + n * 2]     = (byte)(crc & 0xFF);
            frame[7 + n * 2 + 1] = (byte)(crc >> 8);
            return frame;
        }

        // ========== 工具方法 ==========

        private static ushort Crc16(byte[] data, int offset, int length)
        {
            ushort crc = 0xFFFF;
            for (int i = offset; i < offset + length; i++)
            {
                crc ^= data[i];
                for (int j = 0; j < 8; j++)
                {
                    if ((crc & 1) != 0)
                        crc = (ushort)((crc >> 1) ^ 0xA001);
                    else
                        crc >>= 1;
                }
            }
            return crc;
        }

        private static void WriteFloat(ushort[] buf, int idx, float f)
        {
            byte[] b = BitConverter.GetBytes(f);
            if (BitConverter.IsLittleEndian) Array.Reverse(b);
            buf[idx]     = (ushort)((b[0] << 8) | b[1]);
            buf[idx + 1] = (ushort)((b[2] << 8) | b[3]);
        }

        private static float ReadFloat(ushort hi, ushort lo)
        {
            // 组装大端 32 位值（与 STM32 寄存器存储顺序一致）
            uint u = ((uint)hi << 16) | lo;
            // 在 LE 系统上，GetBytes(u) 直接给出 float 的 LE 表示
            // 不需要 Reverse —— ToSingle 在 LE 系统上直接能正确解析
            byte[] b = BitConverter.GetBytes(u);
            return BitConverter.ToSingle(b, 0);
        }

        public void Dispose()
        {
            _port?.Dispose();
        }
    }

    public struct MotorState
    {
        public ushort Err;
        public float Pos;
        public float Vel;
        public float Torque;
        public ushort TempMos;
        public ushort TempRotor;
    }
}
