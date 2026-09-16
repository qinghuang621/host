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
         * 详见 接口文档.md §5.1「电机控制区」。 */
        private const ushort RegBodyVelBase = 0x0014;
        private const ushort RegMotorStat = 0x0064;

        // IMU 姿态输出区起始地址（0x0150 = 336）。共 16 只：roll/pitch/yaw/gyro_xyz/auto_duty/status/temp。
        // 详见 接口文档.md §6.6「IMU 姿态输出区」。
        private const ushort RegImu = 0x0150;

        // ========== 风机子系统寄存器（接口文档.md §6） ==========
        // 4 路手动占空比（自动模式下只保存不驱动）
        private const ushort RegFanDutyBase   = 0x0100; // 0x0100~0x0103 (4 只)
        // 4 路状态起：每路 10 只，间隔 10 (0x0110 / 0x011A / 0x0124 / 0x012E)
        private const ushort RegFanStatusBase = 0x0110; // 共 40 只 → 0x0110~0x0137
        // 风机自动模式参数区（0x0140~0x014F，16 只）
        private const ushort RegFanMode       = 0x0140; // 0=手动 1=线性 2=查表
        private const ushort RegFanAutoEn     = 0x0141; // 自动总开关
        private const ushort RegDutyMin       = 0x0143; // 必须 > 0，否则 θ=0 时掉壁
        private const ushort RegPitchOffset   = 0x0148; // 俯仰零位偏置（int16 整数度）
        private const ushort RegRollOffset    = 0x0149; // 横滚零位偏置
        // LUT 查表区（0x0160~0x016D，14 只 = 13 格 + 魔数）
        private const ushort RegLutBase       = 0x0160; // 0x0160~0x016C (13 格) + 0x016D (MAGIC)
        private const ushort RegLutMagic      = 0x016D; // 0xA5C3
        // 实时倾角 θ 与自动输出占空比（在 IMU 区里，但风机界面要用）
        private const ushort RegTiltTheta     = 0x016E; // float32, 0~180°
        private const ushort RegAutoDuty      = 0x015C; // float32, 自动模式当前输出占空比
        private const ushort RegImuStatus     = 0x015E; // uint16, 0=离线 1=加热 2=运行 3=错误

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

        /// <summary>
        /// 读 IMU 姿态块 + 实时 θ + 姿态四元数。
        /// 一次读 0x0150 起 44 只（0x0150~0x017B）：
        ///   0x0150~0x015F (16 只)  IMU 姿态 + 温度
        ///   0x0160~0x016D (14 只)  LUT 表 + 魔数（只读无害，直接跳过）
        ///   0x016E~0x016F ( 2 只)  实时总倾角 θ（float32）
        ///   0x0170~0x0173 ( 4 只)  空白（断电保持区尾部，恒 0）
        ///   0x0174~0x017B ( 8 只)  姿态四元数 (w, x, y, z)
        /// 字段口径与固件 components/algorithm/ins_task.h::ins_snapshot_t 及
        /// 接口文档.md §6.6、§6.7、§6.8 完全一致。
        /// </summary>
        public ImuData ReadImu()
        {
            ushort[] regs = ReadHoldingRegisters(RegImu, 44);
            return new ImuData
            {
                Roll     = ReadFloat(regs[0],  regs[1]),   // 0x0150
                Pitch    = ReadFloat(regs[2],  regs[3]),   // 0x0152
                Yaw      = ReadFloat(regs[4],  regs[5]),   // 0x0154
                GyroX    = ReadFloat(regs[6],  regs[7]),   // 0x0156
                GyroY    = ReadFloat(regs[8],  regs[9]),   // 0x0158
                GyroZ    = ReadFloat(regs[10], regs[11]),  // 0x015A
                AutoDuty = ReadFloat(regs[12], regs[13]), // 0x015C
                Status   = regs[14],                       // 0x015E
                TempX10  = (short)regs[15],                // 0x015F
                // 倾斜角：车体 z 轴与竖直向上的夹角，0~180°
                // 0x016E 起 2 只 → 偏移 0x1E (30)
                TiltTheta = ReadFloat(regs[30], regs[31]), // 0x016E
                // 四元数：0x0174 起 8 只 → 偏移 0x24 (36)，顺序 (w, x, y, z)
                Qw = ReadFloat(regs[36], regs[37]),        // 0x0174
                Qx = ReadFloat(regs[38], regs[39]),        // 0x0176
                Qy = ReadFloat(regs[40], regs[41]),        // 0x0178
                Qz = ReadFloat(regs[42], regs[43]),        // 0x017A
            };
        }

        // ========== 风机子系统：4 路 + LUT 模式前置条件 ==========

        /// <summary>
        /// 读 4 路风机状态（0x0110~0x0137 共 40 只，每路 10 只）。
        /// 字段口径与接口文档.md §6.2 一致：
        ///   +0 FAN_RUN(uint16) +2/+3 FAN_PULSE(float32 ABCD) +4/+5 FAN_RPM(float32)
        ///   +6/+7 FAN_DUTY_FB(float32, 取实际 CCR 值, 非 0x0100 寄存器值)
        /// 风机 3/4 共用 PWM7，二者反馈恒同值，正常现象。
        /// </summary>
        public FanState[] ReadFanStates()
        {
            ushort[] regs = ReadHoldingRegisters(RegFanStatusBase, 40);
            FanState[] result = new FanState[4];
            for (int i = 0; i < 4; i++)
            {
                int off = i * 10;
                result[i] = new FanState
                {
                    Run    = regs[off + 0],
                    Pulse  = ReadFloat(regs[off + 2], regs[off + 3]),
                    Rpm    = ReadFloat(regs[off + 4], regs[off + 5]),
                    DutyFb = ReadFloat(regs[off + 6], regs[off + 7]),
                };
            }
            return result;
        }

        /// <summary>
        /// 读风机自动模式参数区（0x0140~0x014F，16 只）。
        /// 仅取与 LUT 前置条件相关的字段；其余参数（如 KP_PITCH/MAG_*)固件预留或只读，不解析。
        /// </summary>
        public FanParams ReadFanParams()
        {
            ushort[] regs = ReadHoldingRegisters(RegFanMode, 16);
            return new FanParams
            {
                Mode        = regs[0],   // 0x0140
                AutoEn      = regs[1],   // 0x0141
                DutyFlat    = regs[2],   // 0x0142
                DutyMin     = regs[3],   // 0x0143
                DutyMax     = regs[4],   // 0x0144
                SlopeGain   = regs[5],   // 0x0145
                PitchOffset = (short)regs[8],  // 0x0148
                RollOffset  = (short)regs[9],  // 0x0149
                HeaterPwm   = regs[15],        // 0x014F
            };
        }

        /// <summary>
        /// 读 LUT 13 格占空比表 + 魔数（0x0160~0x016D，14 只，一条 FC03）。
        /// </summary>
        public (ushort[] lut, ushort magic) ReadLut()
        {
            ushort[] regs = ReadHoldingRegisters(RegLutBase, 14);
            ushort[] lut = new ushort[13];
            Array.Copy(regs, 0, lut, 0, 13);
            return (lut, regs[13]);
        }

        /// <summary>
        /// 一次性读取 LUT 模式前置条件所需全部寄存器。
        /// 一次 FC03 读 0x0140 起 48 只（0x0140~0x016F），覆盖：
        ///   0x0140~0x014F (16) 风机参数
        ///   0x0150~0x015F (16) IMU 姿态+状态（其中 0x015C AUTO_DUTY / 0x015E IMU_STATUS 要用）
        ///   0x0160~0x016D (14) LUT + 魔数
        ///   0x016E~0x016F ( 2) 实时倾角 θ
        /// CommLoop 每 200ms 调用一次，单帧 ≈ 100 字节响应，~10ms 传输。
        /// </summary>
        public FanPreCheck ReadFanPreCheck()
        {
            ushort[] regs = ReadHoldingRegisters(RegFanMode, 48);
            var lut = new ushort[13];
            Array.Copy(regs, 32, lut, 0, 13); // 0x0160~0x016C → 偏移 32..44

            return new FanPreCheck
            {
                Params = new FanParams
                {
                    Mode        = regs[0],             // 0x0140
                    AutoEn      = regs[1],             // 0x0141
                    DutyFlat    = regs[2],             // 0x0142
                    DutyMin     = regs[3],             // 0x0143
                    DutyMax     = regs[4],             // 0x0144
                    SlopeGain   = regs[5],             // 0x0145
                    PitchOffset = (short)regs[8],     // 0x0148
                    RollOffset  = (short)regs[9],      // 0x0149
                    HeaterPwm   = regs[15],            // 0x014F
                },
                // 0x015C~0x015D AUTO_DUTY float32 → 偏移 0x1C=28
                AutoDuty  = ReadFloat(regs[28], regs[29]),
                // 0x015E IMU_STATUS uint16 → 偏移 0x1E=30
                ImuStatus = regs[30],
                // 0x016D LUT_MAGIC → 偏移 0x2D=45
                LutMagic  = regs[45],
                Lut       = lut,
                // 0x016E~0x016F TILT_THETA float32 → 偏移 0x2E=46
                TiltTheta = ReadFloat(regs[46], regs[47]),
            };
        }

        /// <summary>
        /// 一次写 4 路手动占空比（0x0100~0x0103，一条 FC10）。
        /// 自动模式下只保存不驱动，输出被 fan_auto_update 每 5ms 覆盖。
        /// </summary>
        public void WriteFanDuties(ushort d1, ushort d2, ushort d3, ushort d4)
        {
            // 钳位 0~100
            d1 = (ushort)Math.Clamp(d1, (ushort)0, (ushort)100);
            d2 = (ushort)Math.Clamp(d2, (ushort)0, (ushort)100);
            d3 = (ushort)Math.Clamp(d3, (ushort)0, (ushort)100);
            d4 = (ushort)Math.Clamp(d4, (ushort)0, (ushort)100);
            WriteMultipleRegisters(RegFanDutyBase, new ushort[] { d1, d2, d3, d4 });
        }

        /// <summary>
        /// 切换风机模式（0x0140 FAN_MODE + 0x0141 FAN_AUTO_EN，一条 FC10）。
        /// ⚠️ 接口文档.md §6.5/§6.7 操作铁律：必须先写参数（LUT/OFFSET/DUTY_MIN）→ 最后切自动。
        /// 调用方应先用 ReadFanPreCheck() 确认前置条件满足再切。
        /// </summary>
        public void WriteFanMode(ushort mode, ushort autoEn)
        {
            WriteMultipleRegisters(RegFanMode, new ushort[] { mode, autoEn });
        }

        /// <summary>
        /// 一次写 LUT 13 格 + 魔数（0x0160~0x016D，一条 FC10）。
        /// ⚠️ 写 LUT 区会立刻触发一次 fan_auto_update 重算。
        /// </summary>
        public void WriteLut(ushort[] lut13, ushort magic = 0xA5C3)
        {
            if (lut13 == null || lut13.Length != 13)
                throw new ArgumentException("LUT 必须是 13 个值", nameof(lut13));
            ushort[] regs = new ushort[14];
            Array.Copy(lut13, 0, regs, 0, 13);
            regs[13] = magic;
            WriteMultipleRegisters(RegLutBase, regs);
        }

        /// <summary>
        /// 写俯仰/横滚零位偏置（0x0148/0x0149，int16 整数度，一条 FC10）。
        /// ⚠️ OFFSET 标错会让整张 LUT 整体偏移。
        /// </summary>
        public void WriteOffsets(short pitchOffset, short rollOffset)
        {
            WriteMultipleRegisters(RegPitchOffset, new ushort[]
            {
                (ushort)pitchOffset,
                (ushort)rollOffset,
            });
        }

        /// <summary>
        /// 风机紧急停止：写 0x0100~0x0103 = 0、再切回手动 0x0140=0/0x0141=0。
        /// 任何一步失败都不抛异常，仅记日志，确保紧急路径可靠。
        /// </summary>
        public void FanEStop()
        {
            try { WriteFanDuties(0, 0, 0, 0); } catch (Exception ex) { Log($"FanEStop duty: {ex.Message}"); }
            Thread.Sleep(20);
            try { WriteFanMode(0, 0); } catch (Exception ex) { Log($"FanEStop mode: {ex.Message}"); }
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

    /// <summary>
    /// IMU 姿态块 + 姿态四元数解析结果。
    /// 字段口径与固件一致：roll/pitch/yaw 单位 deg，gyro 单位 rad/s，temp_x10 单位 ℃×10。
    /// 四元数 (Qw, Qx, Qy, Qz) 为 body→earth，已归一化；当 ‖q‖≈0 时应回退用欧拉角。
    /// </summary>
    public struct ImuData
    {
        public float Roll;      // deg, 右倾为正
        public float Pitch;     // deg, 抬头为正
        public float Yaw;       // deg, 逆时针为正（六轴无磁力计，会持续漂移）
        public float GyroX;     // rad/s
        public float GyroY;     // rad/s
        public float GyroZ;     // rad/s
        public float AutoDuty;  // %, 自动模式当前输出占空比
        public ushort Status;   // 0=离线 1=加热中 2=运行 3=错误
        public short TempX10;   // ℃×10，如 343 = 34.3℃
        public float TiltTheta; // 车体 z 轴与竖直向上的夹角，0~180°
        public float Qw;        // 四元数实部
        public float Qx;        // 四元数 X 分量
        public float Qy;        // 四元数 Y 分量
        public float Qz;        // 四元数 Z 分量
    }

    /// <summary>
    /// 单路风机状态（接口文档.md §6.2）。
    /// 字段口径：+0 FAN_RUN(uint16) +2/+3 FAN_PULSE(float32 ABCD)
    /// +4/+5 FAN_RPM(float32) +6/+7 FAN_DUTY_FB(float32, 取实际 CCR 值)
    /// 注意：风机无温度通道，+8/+9 恒 0，不解析。
    /// </summary>
    public struct FanState
    {
        public ushort Run;      // 运行标志：占空比>0 为 1，否则 0
        public float  Pulse;     // FG 脉冲累计计数（float32, 长时间运行精度会下降）
        public float  Rpm;       // 实测转速 RPM（约 60 RPM 分辨率，PPR 硬编码为 2）
        public float  DutyFb;    // 当前输出占空比 0~100（取实际 CCR 值，非 0x0100）
    }

    /// <summary>
    /// 风机自动模式参数区（0x0140~0x014F）的子集。
    /// 只取与 LUT 模式前置条件相关的字段，预留位（如 KP_*/MAG_*）不解析。
    /// </summary>
    public struct FanParams
    {
        public ushort Mode;         // 0x0140: 0=手动 1=线性 2=查表
        public ushort AutoEn;       // 0x0141: 自动总开关，与 Mode 同时非 0 才接管输出
        public ushort DutyFlat;     // 0x0142: 水平基准占空比（仅线性模式）
        public ushort DutyMin;      // 0x0143: 输出下限，必须 > 0（两模式都生效）
        public ushort DutyMax;      // 0x0144: 输出上限（两模式都生效）
        public ushort SlopeGain;    // 0x0145: 坡度增益（仅线性模式）
        public short  PitchOffset;  // 0x0148: 俯仰零位偏置（整数度）
        public short  RollOffset;   // 0x0149: 横滚零位偏置（整数度）
        public ushort HeaterPwm;   // 0x014F: 加热 PWM 诊断值 0~4500
    }

    /// <summary>
    /// LUT 模式前置条件检查结果（一次 FC03 0x0140~0x016F 读出）。
    /// UI 应据此刷 7 个红绿灯：IMU_STATUS==2 / LUT_MAGIC==0xA5C3 / LUT 13 格非全 0
    /// / FAN_AUTO_EN==1 / FAN_MODE==2 / DUTY_MIN>0 / OFFSET 已校（水平台面时 roll/pitch≈0）。
    /// </summary>
    public struct FanPreCheck
    {
        public ushort   ImuStatus;  // 0x015E
        public ushort   LutMagic;    // 0x016D
        public ushort[] Lut;         // 0x0160~0x016C, 13 只
        public FanParams Params;     // 0x0140~0x014F
        public float    TiltTheta;   // 0x016E 实时倾角 0~180°
        public float    AutoDuty;    // 0x015C 自动模式当前输出占空比（手动模式恒 0）
    }
}
