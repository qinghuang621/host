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

        public AttitudeWindow()
        {
            InitializeComponent();
            Loaded += (_, _) => BuildScene();
        }

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
