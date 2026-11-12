using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Windows.Threading;

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

            // 状态行的本机时钟：独立 1s 节拍，与 IMU 轮询解耦 —— 串口断开时时钟照常走。
            var clock = new DispatcherTimer { Interval = TimeSpan.FromSeconds(1) };
            clock.Tick += (_, _) => TxtClock.Text = DateTime.Now.ToString("yyyy-MM-dd HH:mm:ss");
            clock.Start();
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

        // ===== 磁力计 I2C 链路健康判据（2026-11-12 重构）=====
        // 旧实现依赖固件【临时诊断区】0x0184~0x018C 的 MAG_OK / MAG_ERR / MAG_RECOVER，
        // 但固件已在 cbc867a 按原计划整段删除（作者原注释即"定位完整段删除"）⇒ 那些地址
        // 现在是保留空洞、恒读 0 ⇒ "MAG_OK 不涨"的判据**永久成立**，徽标会一直报
        // 「六轴(I2C 卡死)」。这是 host 与固件版本不同步导致的假故障，不是真的 I2C 挂了。
        //
        // 现改用固件仍在维护的四个量：
        //   · 0x014B MAG_STATUS   —— 固件侧判据 present && enable && fail_streak<3
        //                            （连续读失败 ≥3 次自动降 0，这是最可信的"它挂了"）
        //   · 0x017C MAG_INIT_ERR —— 0=初始化成功，非 0 表示链路压根没建立
        //   · 0x014A MAG_ENABLE   —— 用户开关，用来区分"我关的"和"它挂了"
        //   · 0x017E~0x0183 MAG_X/Y/Z —— 补 MAG_STATUS 的盲区：s_mag 冻结在旧值时
        //     固件仍报 1（ins_task.c 注释明确点过这个静默故障），所以再盯"三轴变不变"。

        /// <summary>MAG_X/Y/Z 连续多少拍完全不变即判"读数冻结"。刷新约 100ms ⇒ 20 拍 ≈ 2s。</summary>
        private const int FrozenTicksToConfirm = 20;

        private float _magPrevX = float.NaN;   // 上一拍三轴，用于判"读数是否还在变"
        private float _magPrevY = float.NaN;
        private float _magPrevZ = float.NaN;
        private int   _magFrozenTicks;         // 三轴连续未变化的拍数
        private bool  _magFrozen;              // 本拍结论，供徽标复用（避免两处重复算）

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

        // ============= 磁力计 I2C 链路健康灯 =============
        // 判据树（顺序不可调换，每一档都靠前一档排除）：
        //   ① 预热期            → 灰 "预热中"       预热时 MagInitErr 读回 0（假通过），必须最先拦
        //   ② MAG_INIT_ERR≠0    → 灰 "N/A"          链路没建立，后面的判据都没意义
        //   ③ MAG_ENABLE=0      → 灰 "已关闭"       用户显式关掉，不是故障
        //                                            （**必须先于 ④**：关闭时 active 也是 0，否则误报卡死）
        //   ④ MAG_STATUS=0      → 红 "卡死·固件降级" 固件判 present && enable && fail_streak<3
        //                                            连续读失败≥3 次才降 0 —— 最硬的"它挂了"
        //   ⑤ 三轴 2s 不变      → 红 "读数冻结"      补 MAG_STATUS=1 仍报正常的盲区
        //   ⑥ 其余              → 绿 "正常"
        private void UpdateMagDiag(ImuData imu, bool magOk, bool warmup)
        {
            // ---- 预热期：一切归零 ----
            // 固件 ins_init() 的预热是阻塞循环，publish_snapshot() 尚未执行 ⇒ 除 0x015E 外
            // 姿态/磁/诊断寄存器全是初值 0 ⇒ MagInitErr=0 会让 magOk "假通过"。
            if (warmup)
            {
                _magFrozenTicks = 0;
                _magFrozen      = false;
                _magPrevX = _magPrevY = _magPrevZ = float.NaN;

                GlitchLamp.Background = new SolidColorBrush(Color.FromRgb(0xBD, 0xBD, 0xBD));
                TxtGlitch.Text        = "预热中";
                TxtGlitch.Foreground  = new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));

                TxtMagActive2.Text = "--";
                TxtMagEnable.Text  = "--";
                TxtMagMove.Text    = "--";
                TxtMagMove.Foreground = new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));
                return;
            }

            // ---- 三轴是否还在变（第⑤档的唯一依据）----
            // ⚠️ MAG_STATUS=0 时固件本来就不更新 s_mag，冻结是**结果**不是**原因**，
            //    这里必须清零，否则恢复九轴后还要再等 2s 才敢报正常。
            if (imu.MagActive == 0)
            {
                _magFrozenTicks = 0;
            }
            else if (imu.MagX == _magPrevX && imu.MagY == _magPrevY && imu.MagZ == _magPrevZ)
            {
                if (_magFrozenTicks < 6000) _magFrozenTicks++;   // 封顶（10 分钟），防长期挂机溢出
            }
            else
            {
                _magFrozenTicks = 0;
            }
            _magPrevX = imu.MagX;
            _magPrevY = imu.MagY;
            _magPrevZ = imu.MagZ;

            _magFrozen = magOk && imu.MagActive == 1
                      && _magFrozenTicks >= FrozenTicksToConfirm;

            // ---- 灯 ----
            Color lamp;
            string text;
            Color textColor = Color.FromRgb(0x55, 0x55, 0x55);

            if (!magOk)
            {
                // 链路压根没建立：MAG_STATUS 必为 0，但那是"从没起来"而不是"读崩了"
                lamp = Color.FromRgb(0xBD, 0xBD, 0xBD);
                text = "N/A";
            }
            else if (imu.MagEnable == 0)
            {
                lamp = Color.FromRgb(0xBD, 0xBD, 0xBD);
                text = "已关闭";
            }
            else if (imu.MagActive == 0)
            {
                lamp = Color.FromRgb(0xE5, 0x39, 0x35);
                text = "卡死·固件降级";
                textColor = Colors.OrangeRed;
            }
            else if (_magFrozen)
            {
                lamp = Color.FromRgb(0xE5, 0x39, 0x35);
                text = "读数冻结";
                textColor = Colors.OrangeRed;
            }
            else
            {
                lamp = Color.FromRgb(0x2E, 0x7D, 0x32);
                text = "正常";
            }

            GlitchLamp.Background = new SolidColorBrush(lamp);
            TxtGlitch.Text        = text;
            TxtGlitch.Foreground  = new SolidColorBrush(textColor);

            // ---- 数值 ----
            TxtMagActive2.Text = imu.MagActive.ToString();
            TxtMagActive2.Foreground = imu.MagActive == 1
                ? new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A))
                : new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));

            TxtMagEnable.Text = imu.MagEnable.ToString();
            TxtMagEnable.Foreground = imu.MagEnable == 1
                ? new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33))
                : new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));

            // 读数状态：只有"九轴真的在跑"才谈得上变不变
            if (!magOk || imu.MagActive == 0)
            {
                TxtMagMove.Text = "--";
                TxtMagMove.Foreground = new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88));
            }
            else if (_magFrozen)
            {
                TxtMagMove.Text = $"冻结 {_magFrozenTicks * 0.1:F1}s";
                TxtMagMove.Foreground = Brushes.OrangeRed;
            }
            else
            {
                TxtMagMove.Text = "变化中";
                TxtMagMove.Foreground = new SolidColorBrush(Color.FromRgb(0x2E, 0x7D, 0x32));
            }
        }

        // ============= 场景构建 =============

        private void BuildScene()
        {
            if (_sceneBuilt) return;
            _sceneBuilt = true;

            // 两个形状完全相同的立方体，分别放到左右两个 ModelVisual3D
            // 左（四元数）平移 -0.9，右（欧拉角）平移 +0.9
            CubeQuat.Content = BuildCubeGroup();
            var mq = Matrix3D.Identity;
            mq.Translate(new Vector3D(-0.9, 0, 0));
            CubeQuat.Transform = new MatrixTransform3D(mq);

            CubeEuler.Content = BuildCubeGroup();
            var me = Matrix3D.Identity;
            me.Translate(new Vector3D(0.9, 0, 0));
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
            double sX = 0.50, sY = 0.35, sZ = 0.23;
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

            var arrowX = MakeArrow(0.65, 0.16, xColor);
            arrowX.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(0, 1, 0), 90));  // +Z → +X
            var arrowY = MakeArrow(0.65, 0.16, yColor);
            arrowY.Transform = new RotateTransform3D(new AxisAngleRotation3D(new Vector3D(1, 0, 0), -90)); // +Z → +Y
            var arrowZ = MakeArrow(0.65, 0.16, zColor);                                                    // 沿 +Z

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

            // ===================== 预热期（IMU 状态 = 1）=====================
            // 固件预热期内主循环未跑、快照不发布 ⇒ 除 0x015E 状态外所有寄存器读回 0。
            // 🔴 那些 0 **不是真实读数**，绝不能再当数据展示（0.0° / 0.0 μT 会让人误以为
            //    "姿态真的就是零"）。预热期一律显示 "--"；I2C 卡死判据整体豁免，
            //    见 UpdateMagDiag 的预热期分支。
            // 前提：状态寄存器 0x015E 不在快照内（bsp_modbus.c 在 INS_get_snapshot 的
            //       提前 return **之前**就写了它）⇒ 预热期它照样能读到 1（加热中）。
            bool warmup = imu.Status == 1;

            // 预热期 ⇒ "--"；否则按各自精度输出。数值一律**强制带正负号**，
            // 配合 Consolas 等宽 + 右对齐 ⇒ 小数点逐列对齐、刷新时不抖动。
            string Num1(double v) => warmup ? "--" : $"{v:+0.0;-0.0;0.0}";
            string Num2(double v) => warmup ? "--" : $"{v:+0.00;-0.00;0.00}";
            string Num3(double v) => warmup ? "--" : $"{v:+0.000;-0.000;0.000}";
            string Pos2(double v) => warmup ? "--" : $"{v:F2}";   // 恒正量（协方差开方）

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
            mL.Translate(new Vector3D(-0.9, 0, 0));
            mL.Rotate(qQuat);
            CubeQuat.Transform = new MatrixTransform3D(mL);

            // ========== 右：欧拉角驱动（ZYX intrinsic: yaw→pitch→roll） ==========
            var qRoll  = new Quaternion(new Vector3D(1, 0, 0), imu.Roll);
            var qPitch = new Quaternion(new Vector3D(0, 1, 0), imu.Pitch);
            var qYaw   = new Quaternion(new Vector3D(0, 0, 1), imu.Yaw);
            var qEuler = Quaternion.Multiply(Quaternion.Multiply(qYaw, qPitch), qRoll);
            var mR = Matrix3D.Identity;
            mR.Translate(new Vector3D(0.9, 0, 0));
            mR.Rotate(qEuler);
            CubeEuler.Transform = new MatrixTransform3D(mR);

            // 数值显示
            // ⚠️ 单位已写进 XAML 的"单位列" ⇒ 这里只输出**纯数字**（不再拼 ° / ℃ / μT）。
            //    一律用 "+0.0;-0.0;0.0" 这类**强制带正负号**的格式：Consolas 等宽 + 右对齐时
            //    正负数宽度一致 ⇒ 小数点逐列对齐，数值刷新时不会左右抖动。
            TxtRoll.Text  = Num1(imu.Roll);
            TxtPitch.Text = Num1(imu.Pitch);
            TxtYaw.Text   = Num1(imu.Yaw);

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
            // 温度：预热期**照常显示** —— 固件 2026-10-08 起在预热循环里也会发布快照，
            // 而温度正是判断"为什么预热这么久"最需要的量（USB 供电到不了目标时会烧满 20s）。
            // 旧固件预热期读回 0 ⇒ 此时退回 "--"，不让 0.0℃ 冒充读数。
            TxtTemp.Text = (warmup && imu.TempX10 == 0) ? "--" : $"{imu.TempX10 / 10.0:F1}";

            // 倾斜角：车体 z 轴与竖直向上的夹角，0~180°（恒正，无需符号位）
            TxtTilt.Text = warmup ? "--" : $"{imu.TiltTheta:F1}";

            // 四元数数值（固件顺序 w,x,y,z）。逐个带符号 ⇒ 等宽下四段各自对齐。
            TxtQuat.Text = warmup
                ? "--"
                : (norm2 > 0.001
                    ? $"w {imu.Qw:+0.000;-0.000;0.000}   x {imu.Qx:+0.000;-0.000;0.000}   "
                    + $"y {imu.Qy:+0.000;-0.000;0.000}   z {imu.Qz:+0.000;-0.000;0.000}"
                    : "( -- )");

            // ===== 角速度（0x0156~0x015B，接口文档.md §6.6）=====
            // 陀螺**原始读数**：机体系、rad/s、未做任何滤波或坐标变换。
            // 固件侧 A1（实测 dt）只影响姿态积分，不改这三个数。
            // ⚠️ 静止时不归零是正常的 —— 它含陀螺零偏（0.00x rad/s 量级）；
            //    零偏随温度漂移，且 Mahony 的 Ki=0（无积分项）只能靠 P 项压成稳态误差。
            TxtGyroX.Text = Num3(imu.GyroX);
            TxtGyroY.Text = Num3(imu.GyroY);
            TxtGyroZ.Text = Num3(imu.GyroZ);

            // ========== 运动控制输出区（0x0190~0x01A7，接口文档.md §6.10）==========
            // 线加速度：机体系**比力**（含重力、固件不减）。静止水平时 ACCEL_Z ≈ +9.8，
            // 可当"加计通道是否正常"的快速判据（不在 ±9.8 附近就要查）。
            TxtAccelX.Text = Num2(imu.AccelX);
            TxtAccelY.Text = Num2(imu.AccelY);
            TxtAccelZ.Text = Num2(imu.AccelZ);

            // 姿态协方差：固件给的是**方差**（rad²）⇒ 开方回标准差、再转度，
            // 才好和固件的 σ 设定值（0x00C0~0x00C5，默认 1.0/1.0/10.0 度）直接对照。
            double covRoll  = Math.Sqrt(Math.Max(0.0, imu.CovRoll))  * 180.0 / Math.PI;
            double covPitch = Math.Sqrt(Math.Max(0.0, imu.CovPitch)) * 180.0 / Math.PI;
            double covYaw   = Math.Sqrt(Math.Max(0.0, imu.CovYaw))   * 180.0 / Math.PI;
            TxtCovRoll.Text  = Pos2(covRoll);
            TxtCovPitch.Text = Pos2(covPitch);
            TxtCovYaw.Text   = Pos2(covYaw);

            // 两种异常都用橙红标出。⚠️ 必须排除预热期 —— 那时三项必然是 0，
            // 但那是"快照还没发布"，不是"σ 没写"，标红属于误报。
            //   ① 三项全 0 ⇒ 固件 0x00C0~0x00C5 被写成/加载成 0 ⇒ 协方差无意义
            //      （升级固件后旧 flash 镜像会把编译期默认的 1/1/10 覆盖成 0）
            //   ② 非对角非 0 ⇒ 违反契约，说明地址或字序错位（与传感器无关）
            bool covBad = !warmup
                       && ((imu.CovRoll <= 0f && imu.CovPitch <= 0f && imu.CovYaw <= 0f)
                           || (imu.CovOffDiagMax > 1e-6f));
            var covBrush = covBad ? Brushes.OrangeRed : Brushes.Black;
            TxtCovRoll.Foreground  = covBrush;
            TxtCovPitch.Foreground = covBrush;
            TxtCovYaw.Foreground   = covBrush;

            // ========== IST8310 磁力计（接口文档.md §6.9）==========
            // 初始化错误码：0=成功，0x40=WHO_AM_I 失败，1~4=第 N 个配置寄存器回读校验失败
            bool magOk = imu.MagInitErr == 0;
            // ⚠️ 预热期这一组寄存器也是 0 ⇒ MagInitErr=0 ⇒ magOk "假通过"，显示上必须区分。
            TxtMagInit.Text = warmup ? "--" : imu.MagInitErr switch
            {
                0x00 => "OK",
                0x40 => "0x40 无传感器",
                _    => $"0x{imu.MagInitErr:X2} 配置失败",
            };
            TxtMagInit.Foreground = warmup
                ? new SolidColorBrush(Color.FromRgb(0x88, 0x88, 0x88))
                : (magOk ? Brushes.Green : Brushes.OrangeRed);

            // 按钮高亮由【设备回读】的 MagEnable 派生 —— 打开窗口即同步，不依赖"点过没点过"。
            // 落盘期内禁用按钮，防止连点写出第二次擦除。
            PaintMagButtons(imu.MagEnable == 1);
            BtnMagOn.IsEnabled = BtnMagOff.IsEnabled = !MagCommitBusy;

            // ===== I2C 链路诊断（必须在徽标之前跑：徽标要用它给出的"卡死"判据，
            //       才能把以前那个「六轴(降级?)」的问号变成确定结论）=====
            UpdateMagDiag(imu, magOk, warmup);

            // 融合状态徽标：九轴（紫）/ 六轴（灰）/ 离线·卡死·冻结（红）
            // ⚠️ 预热期必须**最先**判：那时 MagInitErr 读回 0 ⇒ magOk 假通过，
            //    不先拦 warmup 就会一路掉进"九轴融合"/"启动中"的错误分支。
            //
            // 🔴 2026-11-12 重排：原第 3 档是「MAG_OK 连续 4 拍不涨」，但那个寄存器
            //    （0x0188）已被固件 cbc867a 删除、恒读 0 ⇒ 该判据 100% 成立，徽标
            //    会**永久**显示「六轴(I2C 卡死)」。现改用 MAG_STATUS 本身（固件侧的
            //    fail_streak<3 判据），并把「已关闭」提到「卡死」之前 —— 关闭时
            //    MagActive 同样是 0，顺序错了就会把"我关的"误报成"它挂了"。
            if (warmup)
            {
                TxtMagState.Text = "预热中";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
            }
            else if (!magOk)
            {
                TxtMagState.Text = "MAG 离线";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xE5, 0x39, 0x35));
            }
            else if (imu.MagEnable == 0)
            {
                // 硬件在线、读数正常，但被用户显式关掉 ⇒ 这是"我关的"，不是故障
                TxtMagState.Text = "六轴(已关闭)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
            }
            else if (imu.MagActive == 0)
            {
                // enable=1 但固件把 MAG_STATUS 降为 0 ⇒ 连续读失败 ≥3 次（fail_streak），
                // 固件已自动从九轴降级。这是"它挂了"的确定结论，无需再看任何计数器。
                TxtMagState.Text = "六轴(I2C 卡死)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xE5, 0x39, 0x35));
            }
            else if (_magFrozen)
            {
                // MAG_STATUS 仍是 1，但三轴读数 2s 一个 bit 都没变 ⇒ 踩到 ins_task.c
                // 点明的盲区：s_mag 冻结在旧值，固件自己发现不了。
                TxtMagState.Text = "六轴(读数冻结)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0xE5, 0x39, 0x35));
            }
            else if (imu.MagActive == 1)
            {
                // MagActive 由固件给出；2026-09-29 起其判据含"读数健康"
                //（连续读失败 ≥3 次会自动降级并报 0），所以这里 =1 就确实在跑九轴。
                TxtMagState.Text = "九轴融合";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x6A, 0x1B, 0x9A));
            }
            else
            {
                // 兜底：MagActive 出现 0/1 之外的值（契约外）。正常情况下走不到这里。
                TxtMagState.Text = "六轴(状态异常)";
                MagStateBadge.Background = new SolidColorBrush(Color.FromRgb(0x9E, 0x9E, 0x9E));
            }

            TxtMagX.Text = Num1(imu.MagX);
            TxtMagY.Text = Num1(imu.MagY);
            TxtMagZ.Text = Num1(imu.MagZ);

            // 模值合理性：地磁总场约 25~65 μT；未做硬铁校准时偏置会让它偏大，故上界放宽到 150。
            // 作用：在"九轴在跑、但读数明显不对劲"时给出可见信号 —— 这类问题以前完全静默
            //（垃圾/冻结值被喂进 Mahony，yaw 是错的却毫无提示）。
            TxtMagNorm.Text = warmup ? "--" : $"{imu.MagNorm:F1}";
            bool normSane = imu.MagNorm >= 15f && imu.MagNorm <= 150f;
            // ⚠️ 预热期 MagNorm=0 ⇒ normSane=false，而 magOk 又假通过 ⇒ 会误标橙。
            TxtMagNorm.Foreground = (!warmup && magOk && !normSane)
                ? Brushes.OrangeRed
                : new SolidColorBrush(Color.FromRgb(0x33, 0x33, 0x33));
            if (!warmup && magOk && imu.MagActive == 1 && !normSane)
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
            double shaftR = 0.022, headR = 0.055;
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
