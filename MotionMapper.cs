namespace GamepadSpeedController
{
    /// <summary>
    /// 速度档位定义（vx前+/vy左+/wz逆时针+，单位 m/s 与 rad/s）
    /// </summary>
    public enum SpeedGear { Slow, Mid, Fast }

    /// <summary>
    /// 把摇杆值(-1..+1)映射成车体物理速度(vx, vy, wz)。
    /// 含死区处理、档位限速、指数曲线。
    /// </summary>
    public static class MotionMapper
    {
        // 死区：摇杆中心抖动阈值（0~1）
        public const float DeadZone = 0.10f;

        // 三档最大速度（实测后调整）
        // ⚠️ 元组顺序与下方 Map() 的赋值严格一致：
        //    vx = 横向(右+)限速，vy = 纵向(前+)限速 —— 不是"前后/左右"写反了。
        //    坐标约定与固件 running/kinematics.c 相同：x=右+, y=前+, wz=逆时针+。
        public static readonly (float vx, float vy, float wz)[] GearLimits =
        {
            (0.3f, 0.2f, 0.8f),   // Slow
            (1.0f, 0.6f, 2.0f),   // Mid
            (2.0f, 1.2f, 4.0f),   // Fast
        };

        /// <summary>
        /// 摇杆输入(各 -1..+1) → 车体速度
        /// 车体坐标系：x=右+, y=前+, wz=逆时针+
        /// 左摇杆 Y 上推 → vy(前进)，左摇杆 X 右推 → vx(右移)
        /// 右摇杆 X → wz(旋转)
        /// </summary>
        public static (float vx, float vy, float wz) Map(
            float leftY, float leftX, float rightX, SpeedGear gear)
        {
            var limit = GearLimits[(int)gear];

            // 死区 + 指数曲线（让小推杆更精细）
            float vy_raw = ApplyCurve(DeadZoneFilter(leftY));   // Y上推 → 前进
            float vx_raw = ApplyCurve(DeadZoneFilter(leftX));   // X右推 → 右移
            float wz_raw = ApplyCurve(DeadZoneFilter(rightX));  // 右X → 旋转

            // 符号约定：
            //   左Y上=+1 → vy前+（直接用）
            //   左X右=+1 → vx右+（直接用）
            //   右X右=+1 → wz顺时针，车体系逆时针+，所以取负
            float vx =  vx_raw * limit.vx;
            float vy =  vy_raw * limit.vy;
            float wz = -wz_raw * limit.wz;

            return (vx, vy, wz);
        }

        // 死区：|v| < deadZone 视为 0
        private static float DeadZoneFilter(float v)
        {
            float abs = Math.Abs(v);
            if (abs < DeadZone) return 0f;
            // 把死区外的范围重新映射到 0~1
            float sign = Math.Sign(v);
            return sign * (abs - DeadZone) / (1f - DeadZone);
        }

        // 指数曲线：让中心附近更精细，边缘更激进
        // f(x) = x * |x| （二次曲线，保留符号）
        private static float ApplyCurve(float x) => x * Math.Abs(x);
    }
}
