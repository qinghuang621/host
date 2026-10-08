using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace GamepadSpeedController
{
    /// <summary>
    /// 姿态3D可视化窗口（白底简洁版）：
    ///   - 立方体 + 三根带箭头的本体坐标轴（红=X 右 / 绿=Y 前 / 蓝=Z 上）
    ///   - 坐标轴与立方体同组，随 IMU 欧拉角一起旋转
    ///
    /// 旋转约定（与固件 bsp_imu / 接口文档.md §6.6 一致）：
    ///   ROLL  → 绕 X 轴，右倾为正
    ///   PITCH → 绕 Y 轴，抬头为正
    ///   YAW   → 绕 Z 轴，逆时针为正
    /// 采用 ZYX intrinsic 顺序（yaw → pitch → roll）的航空/机器人常用约定：
    ///   q_total = q_yaw * q_pitch * q_roll
    /// </summary>
    public partial class AttitudeWindow : Window
    {
        private bool _sceneBuilt;

        /// <summary>
        /// 由 MainWindow 在创建本窗口时注入的 Modbus 客户端引用，
        /// 用于九轴/六轴切换按钮直接写 0x014A。
        /// 注入前为 null，按钮点击会被忽略（不会抛异常）。
        /// </summary>
        public ModbusClient? Modbus { get; set; }

        public AttitudeWindow()
        {
            InitializeComponent();
            Loaded += (_, _) => BuildScene();
        }

        // ============= 九轴切换按钮 =============
        // 写 0x014A：1=允许九轴，0=强制六轴。寄存器为断电保持，写入后无需重复写。
        //
        // ⚠️ 2026-09-29 两处修正：
        //  ① 按钮高亮不再在点击时手工染色，一律由 UpdateAngles() 从【设备回读】的
        //     MagEnable 派生（见 PaintMagButtons）。原来只靠点击染色有两个毛病：
        //     · 窗口初次打开时两个按钮颜色完全相同 → 无法判断当前是九轴还是六轴；
        //     · 会与徽标矛盾 —— 例如 MagInitErr!=0 时点"九轴"根本不生效，
        //       按钮变紫而徽标仍显示"六轴(降级?)"。
        //  ② 0x014A 落在断电保持区，写入后 2s 去抖 + 1~2s 扇区擦除（擦除时 CPU
        //     取指 stall、连中断都不响应），期间【所有】Modbus 请求都会超时。
        //     这里登记一个"落盘期"，供 MainWindow 轮询静默这些预期超时。

        /// <summary>落盘期长度：2s 去抖 + 1~2s 擦除 + 通信恢复余量。</summary>
        private const int MagCommitMs = 4500;

        private DateTime _magCommitUntil = DateTime.MinValue;

        /// <summary>
        /// true = 刚写过 0x014A，正处在"2s 去抖 + 1~2s 擦除"窗口内。
        /// 期间 Modbus 超时属预期行为，调用方应静默处理而不是报故障。
        /// </summary>
        public bool MagCommitBusy => DateTime.UtcNow < _magCommitUntil;

        // ===== 【临时诊断 2026-09-23】磁力计 I2C 链路灯状态（固件 0x0184~0x018C）=====
        // ⚠️ 与固件同名的"临时诊断区"配套：固件删除该段时，本组字段与 UpdateMagDiag()
        //    一并删除（清单见 ModbusClient.cs 常量区注释）。

        /// <summary>连续多少次刷新 MAG_OK 不变即判"卡死"。刷新约 100ms ⇒ 4 拍 ≈ 400ms。</summary>
        private const int FrozenTicksToConfirm = 4;

        /// <summary>毛刺闪烁窗长度(ms)。窗内按 160ms 节拍亮/暗交替，形成可见闪烁。</summary>
        private const int GlitchFlashMs = 900;

        private ushort _magPrevOkCnt;          // 上一拍 MAG_OK，用于判"是否还在涨"
        private ushort _magPrevErrCnt;         // 上一拍 MAG_ERR，用于算失败增量
        private ushort _magPrevRecover;        // 上一拍 MAG_RECOVER，用于算毛刺增量
        private int    _magOkFrozenTicks;      // MAG_OK 连续未变化的拍数
        private long   _magGlitchFlashUntil;   // 毛刺闪烁窗截止（TickCount64）
        private bool   _magDiagPrimed;         // 首拍只取基线，不把历史累计误判成新毛刺
        private short  _magPrevRawX;           // RAW 三个值的"变化哨兵"，只在变化时重建字符串
        // 融合频率推算窗（≥600ms 才结算一次，避免抖动）：
        //   固件 s_mag_read_div 数的是【循环次数】，满 10 次才读一次磁
        //   ⇒ 融合频率 = 磁读速率 × 10。读速率 = (ΔOK + ΔERR) / Δt。
        //   这是**免费**的实测仪表：不用改固件就能回答 ins_task 那边的
        //   "融合次数是否少于实际毫秒数"（机制 A：100kHz 磁读阻塞 1.5ms > 1ms 节拍）。
        private long   _magRateWinStartMs;
        private uint   _magRateWinOk;
        private uint   _magRateWinErr;

        private void BtnMagOn_Click(object sender, RoutedEventArgs e)  => ToggleMag(1, "开启九轴");
        private void BtnMagOff_Click(object sender, RoutedEventArgs e) => ToggleMag(0, "强制六轴");

        private void ToggleMag(ushort enable, string what)
        {
            if (Modbus == null) return;
            try
            {
                Modbus.WriteMagEnable(enable);

                // 进入落盘期：立刻给出可见反馈并禁用按钮防连点。
                // 真实状态等 UpdateAngles() 回读到 MagEnable 后再显示（不在这里假定成功）。
                _magCommitUntil = DateTime.UtcNow.AddMilliseconds(MagCommitMs);
                BtnMagOn.IsEnabled  = false;
                BtnMagOff.IsEnabled = false;
                TxtMagState.Text = "写入中…";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xFF, 0x8F, 0x00));
            }
            catch (Exception ex)
            {
                MessageBox.Show(this, $"{what}失败：{ex.Message}",
                    "九轴切换", MessageBoxButton.OK, MessageBoxImage.Warning);
            }
        }

        /// <summary>
        /// 按【设备回读】的使能值重绘两个切换按钮。
        /// 调用点在 UpdateAngles() ⇒ 打开窗口后 100ms 内即与实际状态同步。
        /// </summary>
        private void PaintMagButtons(bool enable)
        {
            BtnMagOn.Background  = enable ? new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A))
                                          : Brushes.LightGray;
            BtnMagOn.Foreground  = enable ? Brushes.White : Brushes.Black;
            BtnMagOff.Background = enable ? Brushes.LightGray
                                          : new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
            BtnMagOff.Foreground = enable ? Brushes.Black : Brushes.White;
        }

        // ============= 【临时诊断】磁力计 I2C 链路健康灯 =============
        // 数据源：固件 0x0184~0x018C（见 ModbusClient.cs 常量区）。
        // ⚠️ 该段在固件里标着"定位完删除"。**将来删固件那一段时，本方法连同上方的
        //    6 个 _magXxx 字段、XAML 里的 I2C 诊断块、以及 ImuData 的 9 个诊断字段一起删。**
        //
        // 灯色的物理含义（与固件 bsp_modbus.c 的判读表逐条对应）：
        //   灰 N/A   —— MAG_INIT_ERR≠0，链路压根没建立，后面的计数没有意义
        //   绿 正常  —— MAG_OK 在涨（I2C 真的读到了数据）
        //   黄 毛刺  —— MAG_RECOVER 涨：毛刺**确实发生过**，但被固件自愈（复位 I2C3）救回。
        //              这是 09-23 那批修复的验收信号：黄闪持续出现 ⇒ 100kHz 降速只是缓解，
        //              需按当时的结论加 4.7kΩ 外部上拉；不再出现 ⇒ 降速已够。
        //   红 卡死  —— MAG_OK 连续 4 拍（≈400ms）不涨：自愈也没救回来 / 读数彻底断了。
        //              这是 09-23 实测复现过的"永久卡死"，也是"六轴(降级)"的真因。
        //              2026-09-30 再细分两种病（靠 ERR 是否还在涨区分）：
        //                · 卡死·重试  —— ERR 仍在涨：固件还在发 I2C，只是全失败
        //                                ⇒ **自愈没生效**（固件版本旧 / 自愈分支有 bug）
        //                · 卡死·停摆  —— ERR 也冻结：连尝试都没有
        //                                ⇒ 读取循环整体停了（InsTask 卡住 / s_mag_present 掉 0）
        private void UpdateMagDiag(ImuData imu, bool magOk)
        {
            // 首拍只登记基线：固件计数是**上电以来**的累计值，
            // 若直接和历史比，会把开机前发生过的毛刺当成"刚发生"而误闪一次。
            if (!_magDiagPrimed)
            {
                _magPrevOkCnt       = imu.MagOkCnt;
                _magPrevErrCnt      = imu.MagErrCnt;
                _magPrevRecover     = imu.MagRecover;
                _magOkFrozenTicks   = 0;
                _magDiagPrimed      = true;
                _magRateWinStartMs  = Environment.TickCount64;
                _magRateWinOk       = 0;
                _magRateWinErr      = 0;
            }

            // ---- MAG_OK 是否还在涨（判"卡死"的唯一硬证据）----
            if (imu.MagOkCnt != _magPrevOkCnt)
            {
                _magOkFrozenTicks = 0;
            }
            else if (_magOkFrozenTicks < 1000)   // 封顶，防长时间挂机后溢出语义不清
            {
                _magOkFrozenTicks++;
            }

            // ---- 增量：uint16 无符号相减，天然处理 65535→0 回绕 ----
            ushort okDelta      = (ushort)(imu.MagOkCnt  - _magPrevOkCnt);
            ushort errDelta     = (ushort)(imu.MagErrCnt - _magPrevErrCnt);
            ushort recoverDelta = (ushort)(imu.MagRecover - _magPrevRecover);
            long   now          = Environment.TickCount64;
            if (recoverDelta > 0)
            {
                _magGlitchFlashUntil = now + GlitchFlashMs;
            }

            // ---- 融合频率推算（见字段区注释；只统计"读次数"，与成败无关）----
            _magRateWinOk  += okDelta;
            _magRateWinErr += errDelta;
            long winMs = now - _magRateWinStartMs;
            if (_magDiagPrimed && winMs >= 600)
            {
                if (!magOk)
                {
                    // 磁力计没在线（s_mag_present=0）⇒ 固件整个跳过读块，速率没有意义
                    TxtMagFusion.Text       = "--";
                    TxtMagFusion.Foreground = new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));
                }
                else
                {
                    double sec       = winMs / 1000.0;
                    double readsSec  = (_magRateWinOk + _magRateWinErr) / sec;
                    double fusionHz  = readsSec * 10.0;   // 每 10 次循环读一次磁
                    TxtMagFusion.Text = $"≈{fusionHz:F0} Hz";
                    // 额定 1000 Hz。明显偏低即说明节拍被挤掉（机制 A 的指纹），标橙。
                    TxtMagFusion.Foreground = fusionHz < 900.0
                        ? Brushes.OrangeRed
                        : new SolidColorBrush(Color.FromRgb(0x2E, 0x7D, 0x32));
                }
                _magRateWinStartMs = now;
                _magRateWinOk      = 0;
                _magRateWinErr     = 0;
            }

            bool frozen   = magOk && _magOkFrozenTicks >= FrozenTicksToConfirm;
            bool flashing = magOk && !frozen && now < _magGlitchFlashUntil;

            // ---- 灯 ----
            Color lamp;
            string glitchText;
            if (!magOk)
            {
                lamp       = Color.FromRgb(0xBD, 0xBD, 0xBD);
                glitchText = "N/A";
            }
            else if (frozen)
            {
                lamp       = Color.FromRgb(0xE5, 0x39, 0x35);
                glitchText = errDelta > 0 ? "卡死·重试" : "卡死·停摆";
            }
            else if (flashing)
            {
                // 亮/暗交替模拟闪烁（不用 DispatcherTimer：本方法本就每 ~100ms 被调一次）
                bool bright = (now / 160) % 2 == 0;
                lamp       = bright ? Color.FromRgb(0xF5, 0x7C, 0x00) : Color.FromRgb(0xFF, 0xE0, 0xB2);
                glitchText = "毛刺";
            }
            else
            {
                lamp       = Color.FromRgb(0x2E, 0x7D, 0x32);
                glitchText = "正常";
            }
            GlitchLamp.Background  = new SolidColorBrush(lamp);
            TxtGlitch.Text         = glitchText;
            TxtGlitch.Foreground   = (frozen || flashing) ? Brushes.OrangeRed
                                                          : new SolidColorBrush(Color.FromRgb(0x55, 0x55, 0x55));

            // 0x014B：卡死时徽标被红灯占位，这里单独显示，用于分辨固件版本
            //（=1 ⇒ 固件用旧判据 present&&enable；=0 ⇒ 含 09-29 的 fail_streak 判据）
            TxtMagActive2.Text = imu.MagActive.ToString();
            TxtMagActive2.Foreground = imu.MagActive == 1
                ? new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A))
                : new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33));

            // ---- 计数：RECOVER 带本窗口增量，一眼看出"这一拍新发生了几个毛刺" ----
            TxtMagRecover.Text = recoverDelta > 0
                ? $"{imu.MagRecover} (+{recoverDelta})"
                : imu.MagRecover.ToString();
            TxtMagRecover.Foreground = recoverDelta > 0
                ? Brushes.OrangeRed
                : new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33));

            TxtMagOk.Text = imu.MagOkCnt.ToString();
            TxtMagOk.Foreground = frozen
                ? Brushes.OrangeRed
                : new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33));

            // ERR 带增量：区分"还在重试"与"彻底停摆"靠的就是这个数涨不涨 ——
            // 单看一张静态截图也能判断，不必盯屏。
            TxtMagErr.Text = errDelta > 0
                ? $"{imu.MagErrCnt} (+{errDelta})"
                : imu.MagErrCnt.ToString();
            TxtMagErr.Foreground = imu.MagErrCnt > 0
                ? Brushes.OrangeRed
                : new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33));

            // ---- I2C 现场快照：HAL / State / ErrorCode 合看才能定位病因 ----
            TxtMagHal.Text = HalText(imu.MagHal);
            TxtMagHal.Foreground = imu.MagHal == 0
                ? new SolidColorBrush(Color.FromRgb(0x2E, 0x7D, 0x32))
                : Brushes.OrangeRed;

            TxtMagI2cState.Text = $"0x{imu.MagI2cState:X2}";
            TxtMagI2cState.Foreground = imu.MagI2cState == 0x20
                ? new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33))
                : Brushes.OrangeRed;

            TxtMagEcode.Text = $"0x{imu.MagI2cEcode:X2}";
            TxtMagEcode.Foreground = imu.MagI2cEcode == 0
                ? new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33))
                : Brushes.OrangeRed;

            // ---- 原始计数：只在变化时重建字符串（避免每 100ms 产生垃圾）----
            if (imu.MagRawX != _magPrevRawX)
            {
                TxtMagRaw.Text  = $"{imu.MagRawX} / {imu.MagRawY} / {imu.MagRawZ}";
                _magPrevRawX    = imu.MagRawX;
            }

            _magPrevOkCnt   = imu.MagOkCnt;
            _magPrevErrCnt  = imu.MagErrCnt;
            _magPrevRecover = imu.MagRecover;
        }

        /// <summary>HAL 返回码 -> 可读文字（0=OK 1=ERROR 2=BUSY 3=TIMEOUT）。</summary>
        private static string HalText(ushort hal) => hal switch
        {
            0 => "OK",
            1 => "ERROR",
            2 => "BUSY",
            3 => "TIMEOUT",
            _ => $"0x{hal:X2}",
        };

        // ============= 场景构建 =============

        private void BuildScene()
        {
            if (_sceneBuilt) return;
            _sceneBuilt = true;

            // 两个形状完全相同的立方体，分别放到左右两个 ModelVisual3D
            // 左（四元数）平移 -1.4，右（欧拉角）平移 +1.4
            CubeQuat.Content = BuildCubeGroup();
            var mq = Matrix3D.Identity;
            mq.Translate(new Vector3D(-1.4, 0, 0));
            CubeQuat.Transform = new MatrixTransform3D(mq);

            CubeEuler.Content = BuildCubeGroup();
            var me = Matrix3D.Identity;
            me.Translate(new Vector3D(1.4, 0, 0));
            CubeEuler.Transform = new MatrixTransform3D(me);
        }

        /// <summary>
        /// 构建一个立方体（长方体）+ 三根带箭头的本体坐标轴。
        /// 立方体边长按用户要求：sX=0.80, sY=0.55, sZ=0.37。
        /// 两个对比立方体共用此方法，保证形状完全一致。
        /// </summary>
        private static Model3DGroup BuildCubeGroup()
        {
            var cube = new Model3DGroup();

            // 立方体本体：6 个面按轴向着色（亮色=正方向，深色=负方向）
            double sX = 0.80, sY = 0.55, sZ = 0.37;
            cube.Children.Add(MakeFace( sX, 0, 0, new Vector3D( 1, 0, 0), sY, sZ, Color.FromRgb(0xEF, 0x53, 0x50))); // +X
            cube.Children.Add(MakeFace(-sX, 0, 0, new Vector3D(-1, 0, 0), sY, sZ, Color.FromRgb(0xC6, 0x28, 0x28))); // -X
            cube.Children.Add(MakeFace(0,  sY, 0, new Vector3D(0,  1, 0), sX, sZ, Color.FromRgb(0x66, 0xBB, 0x6A))); // +Y
            cube.Children.Add(MakeFace(0, -sY, 0, new Vector3D(0, -1, 0), sX, sZ, Color.FromRgb(0x2E, 0x7D, 0x32))); // -Y
            cube.Children.Add(MakeFace(0, 0,  sZ, new Vector3D(0, 0,  1), sX, sY, Color.FromRgb(0x42, 0xA5, 0xF5))); // +Z
            cube.Children.Add(MakeFace(0, 0, -sZ, new Vector3D(0, 0, -1), sX, sY, Color.FromRgb(0x15, 0x65, 0xC0))); // -Z

            // 本体坐标轴：从立方体中心沿 +X/+Y/+Z 穿出面画带箭头的轴（随立方体一起旋转）
            var xColor = Color.FromRgb(0xD3, 0x2F, 0x2F);
            var yColor = Color.FromRgb(0x2E, 0x7D, 0x32);
            var zColor = Color.FromRgb(0x15, 0x65, 0xC0);

            var arrowX = MakeArrow(1.05, 0.25, xColor);
            arrowX.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 90));  // +Z → +X
            var arrowY = MakeArrow(1.05, 0.25, yColor);
            arrowY.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), -90)); // +Z → +Y
            var arrowZ = MakeArrow(1.05, 0.25, zColor);                                                    // 沿 +Z

            cube.Children.Add(arrowX);
            cube.Children.Add(arrowY);
            cube.Children.Add(arrowZ);

            return cube;
        }

        /// <summary>
        /// 用 IMU 数据更新立方体旋转 + 数值显示。
        /// 优先使用固件输出的四元数 (Qw,Qx,Qy,Qz) 驱动旋转（无万向锁），
        /// 当四元数未输出（‖q‖≈0）时回退到欧拉角 ZYX intrinsic 合成。
        /// 调用方应已通过 Dispatcher.BeginInvoke 切到 UI 线程。
        /// </summary>
        public void UpdateAngles(ImuData imu)
        {
            if (!_sceneBuilt) return;

            // ========== 左：四元数驱动 ==========
            // 固件四元数约定：body→earth，顺序 (w,x,y,z)，已归一化。
            // WPF Quaternion(x,y,z,w) 直接对应。
            double norm2 = (double)imu.Qw * imu.Qw + (double)imu.Qx * imu.Qx
                         + (double)imu.Qy * imu.Qy + (double)imu.Qz * imu.Qz;

            Quaternion qQuat;
            if (norm2 > 0.001)
            {
                qQuat = new Quaternion(imu.Qx, imu.Qy, imu.Qz, imu.Qw);
            }
            else
            {
                // 四元数未输出：保持不动（identity），不回退到欧拉角，否则对比失真
                qQuat = Quaternion.Identity;
            }
            var mL = Matrix3D.Identity;
            mL.Translate(new Vector3D(-1.4, 0, 0));
            mL.Rotate(qQuat);
            CubeQuat.Transform = new MatrixTransform3D(mL);

            // ========== 右：欧拉角驱动（ZYX intrinsic: yaw→pitch→roll） ==========
            var qRoll  = new Quaternion(new Vector3D(1, 0, 0), imu.Roll);
            var qPitch = new Quaternion(new Vector3D(0, 1, 0), imu.Pitch);
            var qYaw   = new Quaternion(new Vector3D(0, 0, 1), imu.Yaw);
            var qEuler = Quaternion.Multiply(Quaternion.Multiply(qYaw, qPitch), qRoll);
            var mR = Matrix3D.Identity;
            mR.Translate(new Vector3D(1.4, 0, 0));
            mR.Rotate(qEuler);
            CubeEuler.Transform = new MatrixTransform3D(mR);

            // 数值显示
            TxtRoll.Text  = $"{imu.Roll:F1}°";
            TxtPitch.Text = $"{imu.Pitch:F1}°";
            TxtYaw.Text   = $"{imu.Yaw:F1}°";

            TxtStatus.Text = imu.Status switch
            {
                0 => "离线",
                1 => "加热中",
                2 => "运行",
                3 => "错误",
                _ => imu.Status.ToString(),
            };
            TxtStatus.Foreground = imu.Status == 2
                ? Brushes.Green
                : (imu.Status == 3 ? Brushes.OrangeRed : Brushes.Gray);
            TxtTemp.Text = $"{imu.TempX10 / 10.0:F1}℃";

            // 倾斜角：车体 z 轴与竖直向上的夹角，0~180°
            TxtTilt.Text = $"{imu.TiltTheta:F1}°";

            // 四元数数值（固件顺序 w,x,y,z）
            TxtQuat.Text = norm2 > 0.001
                ? $"(w={imu.Qw:F3}, x={imu.Qx:F3}, y={imu.Qy:F3}, z={imu.Qz:F3})"
                : "( -- )";

            // ========== 运动控制输出区（0x0190~0x01A7，接口文档.md §6.10）==========
            // 线加速度：机体系**比力**（含重力、固件不减）。静止水平时 ACCEL_Z ≈ +9.8，
            // 可当"加计通道是否正常"的快速判据（不在 ±9.8 附近就要查）。
            TxtAccelX.Text = $"{imu.AccelX:F2}";
            TxtAccelY.Text = $"{imu.AccelY:F2}";
            TxtAccelZ.Text = $"{imu.AccelZ:F2}";

            // 姿态协方差：固件给的是**方差**（rad²）⇒ 开方回标准差、再转度，
            // 才好和固件的 σ 设定值（0x00C0~0x00C5，默认 1.0/1.0/10.0 度）直接对照。
            double covRoll  = Math.Sqrt(Math.Max(0.0, imu.CovRoll))  * 180.0 / Math.PI;
            double covPitch = Math.Sqrt(Math.Max(0.0, imu.CovPitch)) * 180.0 / Math.PI;
            double covYaw   = Math.Sqrt(Math.Max(0.0, imu.CovYaw))   * 180.0 / Math.PI;
            TxtCovRoll.Text  = $"{covRoll:F2}";
            TxtCovPitch.Text = $"{covPitch:F2}";
            TxtCovYaw.Text   = $"{covYaw:F2}";

            // 两种异常都用橙红标出：
            //   ① 三项全 0 ⇒ 固件 0x00C0~0x00C5 被写成/加载成 0 ⇒ ROS2 会直接丢弃姿态
            //      （升级固件后旧 flash 镜像会把编译期默认的 1/1/10 覆盖成 0）
            //   ② 非对角非 0 ⇒ 违反契约，说明地址或字序错位（与传感器无关）
            bool covBad = (imu.CovRoll <= 0f && imu.CovPitch <= 0f && imu.CovYaw <= 0f)
                       || (imu.CovOffDiagMax > 1e-6f);
            var covBrush = covBad ? Brushes.OrangeRed : Brushes.Black;
            TxtCovRoll.Foreground  = covBrush;
            TxtCovPitch.Foreground = covBrush;
            TxtCovYaw.Foreground   = covBrush;

            // ========== IST8310 磁力计（接口文档.md §6.9）==========
            // 初始化错误码：0=成功，0x40=WHO_AM_I 失败，1~4=第 N 个配置寄存器回读校验失败
            bool magOk = imu.MagInitErr == 0;
            TxtMagInit.Text = imu.MagInitErr switch
            {
                0x00 => "OK",
                0x40 => "0x40 无传感器",
                _    => $"0x{imu.MagInitErr:X2} 配置失败",
            };
            TxtMagInit.Foreground = magOk ? Brushes.Green : Brushes.OrangeRed;

            // 按钮高亮由【设备回读】的 MagEnable 派生 —— 打开窗口即同步，不依赖"点过没点过"。
            // 落盘期内禁用按钮，防止连点写出第二次擦除。
            PaintMagButtons(imu.MagEnable == 1);
            BtnMagOn.IsEnabled = BtnMagOff.IsEnabled = !MagCommitBusy;

            // ===== I2C 链路诊断（必须在徽标之前跑：徽标要用它给出的"卡死"判据，
            //       才能把以前那个「六轴(降级?)」的问号变成确定结论）=====
            UpdateMagDiag(imu, magOk);

            // 融合状态徽标：九轴（紫）/ 六轴（灰）/ 离线·卡死（红）
            if (!magOk)
            {
                TxtMagState.Text = "MAG 离线";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xE5, 0x39, 0x35));
            }
            else if (_magOkFrozenTicks >= FrozenTicksToConfirm)
            {
                // MAG_OK 已连续多拍不涨 ⇒ I2C 读不到，固件按 fail_streak>=3 自动降级。
                // 这是"它挂了"，与用户有没有关 0x014A 无关，必须优先报出来。
                // 2026-09-30 起不再显示带问号的「六轴(降级?)」—— 有 0x0188 就是确定结论。
                TxtMagState.Text = "六轴(I2C 卡死)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xE5, 0x39, 0x35));
            }
            else if (imu.MagActive == 1)
            {
                // MagActive 由固件给出；2026-09-29 起其判据含"读数健康"
                //（连续读失败 ≥3 次会自动降级并报 0），所以这里 =1 就确实在跑九轴。
                TxtMagState.Text = "九轴融合";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A));
            }
            else if (imu.MagEnable == 0)
            {
                // 硬件在线、读数正常，但被用户显式关掉 ⇒ 这是"我关的"
                TxtMagState.Text = "六轴(已关闭)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
            }
            else
            {
                // enable=1、active=0，但 MAG_OK 仍在涨（未冻结）⇒ 只可能是启动最初几拍：
                // 固件要读满 3 次失败才降级，读数健康时下一周期就会切回九轴。
                TxtMagState.Text = "六轴(启动中)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
            }

            TxtMagX.Text = $"{imu.MagX:F1}";
            TxtMagY.Text = $"{imu.MagY:F1}";
            TxtMagZ.Text = $"{imu.MagZ:F1}";

            // 模值合理性：地磁总场约 25~65 μT；未做硬铁校准时偏置会让它偏大，故上界放宽到 150。
            // 作用：在"九轴在跑、但读数明显不对劲"时给出可见信号 —— 这类问题以前完全静默
            //（垃圾/冻结值被喂进 Mahony，yaw 是错的却毫无提示）。
            TxtMagNorm.Text = $"{imu.MagNorm:F1}";
            bool normSane = imu.MagNorm >= 15f && imu.MagNorm <= 150f;
            TxtMagNorm.Foreground = (magOk && !normSane)
                ? Brushes.OrangeRed
                : new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33));
            if (magOk && imu.MagActive == 1 && !normSane)
            {
                TxtMagState.Text = "九轴(模值异常)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xF5, 0x7C, 0x00));
            }
        }

        // ============= 几何辅助 =============

        /// <summary>
        /// 构造立方体的一块矩形面：中心在 (cx,cy,cz)，外法线 normal，
        /// 局部 u/v 方向半边长分别为 hu/hv（支持非正方体）。
        /// u/v 方向按法线轴向硬编码，避免依赖 refv 旋转后语义不清：
        ///   normal 沿 ±X：u=Y 轴、v=Z 轴
        ///   normal 沿 ±Y：u=X 轴、v=Z 轴
        ///   normal 沿 ±Z：u=X 轴、v=Y 轴
        /// </summary>
        private static GeometryModel3D MakeFace(double cx, double cy, double cz,
                                               Vector3D normal, double hu, double hv, Color color)
        {
            double ax = Math.Abs(normal.X), ay = Math.Abs(normal.Y), az = Math.Abs(normal.Z);
            Vector3D u, v;
            if (ax >= ay && ax >= az)        // normal 沿 X → u=Y, v=Z
            {
                u = new Vector3D(0, 1, 0);
                v = new Vector3D(0, 0, 1);
            }
            else if (ay >= az)               // normal 沿 Y → u=X, v=Z
            {
                u = new Vector3D(1, 0, 0);
                v = new Vector3D(0, 0, 1);
            }
            else                             // normal 沿 Z → u=X, v=Y
            {
                u = new Vector3D(1, 0, 0);
                v = new Vector3D(0, 1, 0);
            }

            Point3D c = new(cx, cy, cz);
            Point3D p0 = c + (-u * hu - v * hv);
            Point3D p1 = c + ( u * hu - v * hv);
            Point3D p2 = c + ( u * hu + v * hv);
            Point3D p3 = c + (-u * hu + v * hv);

            var mesh = new MeshGeometry3D
            {
                Positions = new Point3DCollection { p0, p1, p2, p3 },
                TriangleIndices = new Int32Collection { 0, 1, 2, 0, 2, 3 },
            };
            var mat = new DiffuseMaterial(new SolidColorBrush(color));
            return new GeometryModel3D
            {
                Geometry = mesh,
                Material = mat,
                BackMaterial = mat,
            };
        }

        /// <summary>
        /// 构造一根沿 +Z 的坐标轴箭头：圆柱杆 (0 → length-coneH) + 圆锥头 (→ length)。
        /// 基础几何沿 +Z，由调用方用 Transform 旋转到目标轴。
        /// </summary>
        private static Model3DGroup MakeArrow(double length, double coneH, Color color)
        {
            var group = new Model3DGroup();
            var mat = new DiffuseMaterial(new SolidColorBrush(color));

            const int Seg = 20;
            double shaftR = 0.035, headR = 0.09;
            double shaftLen = length - coneH;

            // --- 圆柱杆 ---
            var shaft = new MeshGeometry3D();
            var sPos = new Point3DCollection();
            var sIdx = new Int32Collection();
            for (int i = 0; i < Seg; i++)
            {
                double a0 = 2 * Math.PI * i / Seg;
                double a1 = 2 * Math.PI * (i + 1) / Seg;
                sPos.Add(new Point3D(shaftR * Math.Cos(a0), shaftR * Math.Sin(a0), 0));
                sPos.Add(new Point3D(shaftR * Math.Cos(a1), shaftR * Math.Sin(a1), 0));
                sPos.Add(new Point3D(shaftR * Math.Cos(a1), shaftR * Math.Sin(a1), shaftLen));
                sPos.Add(new Point3D(shaftR * Math.Cos(a0), shaftR * Math.Sin(a0), shaftLen));
                int b = i * 4;
                sIdx.Add(b);     sIdx.Add(b + 1); sIdx.Add(b + 2);
                sIdx.Add(b);     sIdx.Add(b + 2); sIdx.Add(b + 3);
                // 两端封口（中心点扇形）
                sPos.Add(new Point3D(0, 0, 0));
                sPos.Add(new Point3D(0, 0, shaftLen));
                int c0 = Seg * 4 + i * 2, c1 = c0 + 1;
                sIdx.Add(c0); sIdx.Add(b + 1); sIdx.Add(b);
                sIdx.Add(c1); sIdx.Add(b + 2); sIdx.Add(b + 3);
            }
            shaft.Positions = sPos;
            shaft.TriangleIndices = sIdx;
            group.Children.Add(new GeometryModel3D
            {
                Geometry = shaft,
                Material = mat,
                BackMaterial = mat,
            });

            // --- 圆锥头 ---
            var head = new MeshGeometry3D();
            var hPos = new Point3DCollection();
            var hIdx = new Int32Collection();
            var apex = new Point3D(0, 0, length);
            for (int i = 0; i < Seg; i++)
            {
                double a0 = 2 * Math.PI * i / Seg;
                double a1 = 2 * Math.PI * (i + 1) / Seg;
                hPos.Add(new Point3D(headR * Math.Cos(a0), headR * Math.Sin(a0), shaftLen));
                hPos.Add(new Point3D(headR * Math.Cos(a1), headR * Math.Sin(a1), shaftLen));
                hPos.Add(apex);
                int b = i * 3;
                hIdx.Add(b); hIdx.Add(b + 1); hIdx.Add(b + 2);
                // 底面封口
                hPos.Add(new Point3D(0, 0, shaftLen));
                int c = Seg * 3 + i;
                hIdx.Add(c); hIdx.Add(b + 1); hIdx.Add(b);
            }
            head.Positions = hPos;
            head.TriangleIndices = hIdx;
            group.Children.Add(new GeometryModel3D
            {
                Geometry = head,
                Material = mat,
                BackMaterial = mat,
            });

            return group;
        }
    }
}
