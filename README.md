# STM32 小车控制台（上位机）

基于 WPF (.NET 8) 的 STM32 小车上位机控制程序，通过 **Modbus RTU** 与车体主控通信，支持手柄速度控制、四路风机 Modbus 控制、IMU 姿态 3D 可视化等功能。

## 技术栈

- **运行时**：.NET 8 (`net8.0-windows`)
- **UI 框架**：WPF（`UseWPF`）
- **通信**：`System.IO.Ports`（串口 / Modbus RTU 主站）
- **手柄**：`SharpDX.XInput`（XInput 手柄）
- **3D 渲染**：WPF 内置 `Viewport3D` + `PerspectiveCamera`

## 主要功能

### 1. 电机手动控制
- 通过 `vx / vy / wz` 速度命令直接驱动车体（寄存器 `0x0014~0x0019`）
- 支持单次发送与 100ms 心跳持续发送
- XInput 手柄档位（慢/中/快）映射到速度命令
- 电机使能 / 紧急停止（寄存器 `0x0000`）

### 2. 风机 Modbus 四路控制
- **手动模式**：直接写 4 路占空比（`0x0100~0x0103`）
- **线性模式**：基于 `DutyMin / DutyMax / SlopeGain / PitchOffset / RollOffset` 自动分配
- **查表模式 (LUT)**：13 格查表 + 魔数校验（`0x0160~0x016D`，魔数 `0xA5C3`）
- **前置检查**：发送命令前轮询 `IMU_STATUS / LUT_MAGIC / 风机参数 / TILT_THETA / AUTO_DUTY`，校验通过才允许下发
- **状态反馈**：每路含 RUN 指示灯、目标占空比、实际占空比 ProgressBar、脉冲累计、转速 RPM
- **紧急停止**：`FanEStop` 一键切断四路输出

### 3. IMU 姿态 3D 可视化
独立窗口 `AttitudeWindow`，双立方体对比显示：
- **左立方体**：固件四元数 `(Qw, Qx, Qy, Qz)` 直接驱动
- **右立方体**：欧拉角（ZYX intrinsic）合成旋转 `qYaw * qPitch * qRoll`
- **TILT_THETA 倾斜角**：车体 Z 轴与重力反方向夹角（`0x016E`，float32，0~180°）
- 立方体表面绘制坐标轴，跟随立方体一起旋转，白底背景

## 项目结构

```
D:\stm32\host\
├── App.xaml / App.xaml.cs          # WPF 应用入口
├── MainWindow.xaml                # 主界面（串口 / 电机 / TabControl）
├── MainWindow.xaml.cs             # 主界面逻辑（通信、手柄、风机轮询）
├── ModbusClient.cs                # Modbus RTU 主站实现
├── MotionMapper.cs                # 手柄 → 速度命令映射
├── AttitudeWindow.xaml            # 姿态 3D 可视化界面
├── AttitudeWindow.xaml.cs         # 姿态窗口逻辑（四元数/欧拉角双立方体）
├── FanSerialClient.cs             # 旧版 USB CDC 风机协议（已废弃，保留参考）
├── GamepadSpeedController.csproj   # 工程文件
├── GamepadSpeedController.slnx    # 解决方案
└── app.manifest                   # 应用清单
```

## 依赖

| 包 | 版本 | 用途 |
|----|------|------|
| `SharpDX.XInput` | 4.2.0 | XInput 手柄输入 |
| `System.IO.Ports` | 8.0.0 | 串口 / Modbus RTU |

## 构建与运行

```powershell
# 还原依赖
dotnet restore

# 编译
dotnet build

# 运行（或在 Visual Studio 中按 F5）
dotnet run
```

## Modbus 通信协议

### 通用参数
- **从站 ID**：`1`
- **串口参数**：默认 `115200, 8N1`
- **支持功能码**：`0x03` 读保持寄存器 / `0x06` 写单个 / `0x10` 写多个
- **CRC16**：多项式 `0xA001`

### 关键寄存器映射

| 地址 | 名称 | 类型 | 说明 |
|------|------|------|------|
| `0x0000` | ENABLE | u16 | 电机使能（1=使能 0=失能）|
| `0x0014~0x0019` | 车体速度块 | int16 ×6 | VY_HI/LO, VX_HI/LO, WZ_HI/LO |
| `0x0064` | 电机状态 | u16 | 电机运行状态 |
| `0x0100~0x0103` | FAN_DUTY | u16 ×4 | 4 路手动占空比 |
| `0x0110~0x0137` | FAN_STATUS | 40 只 | 4 路状态（每路 10 只）|
| `0x0140` | FAN_MODE | u16 | 0=手动 1=线性 2=查表 |
| `0x0141` | FAN_AUTO_EN | u16 | 自动模式总开关 |
| `0x0143` | DUTY_MIN | u16 | 必须 > 0（否则 θ=0 掉壁）|
| `0x0148~0x0149` | PITCH/ROLL_OFFSET | int16 | 俯仰/横滚零位偏置 |
| `0x0150~0x015F` | IMU 姿态区 | float32 | roll/pitch/yaw + gyro + auto_duty + status |
| `0x015C` | AUTO_DUTY | float32 | 自动输出占空比 |
| `0x015E` | IMU_STATUS | u16 | IMU 状态字 |
| `0x0160~0x016C` | LUT 表 | u16 ×13 | 13 格查表 |
| `0x016D` | LUT_MAGIC | u16 | 魔数校验（`0xA5C3`）|
| `0x016E~0x016F` | TILT_THETA | float32 | 车体 Z 轴与重力反方向夹角（0~180°）|
| `0x0174~0x017B` | 四元数 | float32 ×4 | Qw, Qx, Qy, Qz |

### float32 字节序
**高字在前 (ABCD)**：高字 → 低字 → 组合为 float32。

## 风机使用要点

### 模式切换
1. **手动模式 (`FAN_MODE=0`)**：直接写 `0x0100~0x0103`，立即生效
2. **线性模式 (`FAN_MODE=1`)**：配置 `DUTY_MIN/MAX/SLOPE_GAIN/PITCH/ROLL_OFFSET` 后置 `FAN_AUTO_EN=1`
3. **查表模式 (`FAN_MODE=2`)**：写入 13 格 LUT 后写入魔数 `0xA5C3`，再置 `FAN_AUTO_EN=1`

### LUT 输出 PWM 的前置条件
- `IMU_STATUS` 必须正常（非 0）
- `LUT_MAGIC` 必须 = `0xA5C3`，否则 LUT 视为未初始化
- `DUTY_MIN` 必须 > 0，防止 θ=0 时风机掉壁
- 切换模式前建议先 `FanEStop` 停机

### 风机轮询周期
主通信循环 `CommLoop` 中每 `200ms`（tick % 2 == 0）读取一次：
- `ReadFanStates()` → 4 路 RUN/PULSE/RPM/DUTY_FB
- `ReadFanPreCheck()` → 模式 / 参数 / LUT / TILT_THETA / AUTO_DUTY

## 3D 姿态可视化说明

### 四元数 → WPF Quaternion 映射
固件输出顺序 `(Qw, Qx, Qy, Qz)`，WPF `Quaternion` 构造顺序为 `(x, y, z, w)`：

```csharp
new Quaternion(Qx, Qy, Qz, Qw)
```

### 欧拉角合成（ZYX intrinsic）
```
q_total = qYaw * qPitch * qRoll
```
顺序不可颠倒，否则姿态错误。

### Gimbal Lock 注意事项
当 pitch ≈ ±90° 时，`asin` 硬限会导致 roll/yaw 抖动，属正常现象。对比左右立方体可观察四元数与欧拉角的差异。

## 常见问题

| 现象 | 原因 | 解决 |
|------|------|------|
| `NullReferenceException` 于 `SlFanDuty_ValueChanged` | `InitializeComponent` 期间 Slider.Value=50 提前触发 ValueChanged，TextBlock 尚未创建 | 在 `ValueChanged` 开头加 `if (TxtFanDuty == null) return;` |
| `OK S0 -> STOP` 不刷新 UI | 旧版正则 `^OK S(\d+) TARGET=(\d+)` 要求 TARGET= 必填 | 改为 `^OK S(\d+)(?:\s+TARGET=(\d+))?` |
| 风机脉冲/转速反馈看不见 | 旧版窗口过小，反馈区被裁剪 | 调整 `Height=900 Width=960`，`ResizeMode=CanResize`，反馈区使用 `RowDefinition Height="*"` |
| `git push` 超时 | 国内访问 GitHub 443 端口被阻 | 配置代理或网络恢复后重试 `git push -u origin main` |
| 编译警告 `UpdateFanRow` 可能为 null | UI 元素未做空值检查 | 在 switch 表达式后加 `if (led == null \|\| pb == null ...) return;` |

## 提交规范

遵循 [Conventional Commits](https://www.conventionalcommits.org/) 风格：

```
<type>(<scope>): <subject>

<body>
```

类型示例：`feat / fix / docs / chore / refactor`。
作用域示例：`fan / imu / ui / protocol`。

## 相关文档

- 风机控制板固件接口文档：`D:\stm32\fan_control.worktrees\credit-limit-inquiry\接口文档.md`
