# 四旋翼飞控系统

基于英飞凌 CYT4BB7 双核微控制器的四旋翼无人机飞行控制系统，采用 Mahony 姿态解算算法和级联 PID 控制架构。

## 项目简介

本项目为全国大学生智能汽车竞赛飞越雷区赛道设计，实现了无人机的姿态控制、高度控制和视觉定位功能。采用双核异构架构，CM7_0 核心负责飞行控制，CM7_1 核心负责视觉处理。

### 主要特性

- ✅ **双核架构**：CM7_0 飞行控制 (1000Hz) + CM7_1 视觉处理 (异步)
- ✅ **姿态估计**：Mahony 互补滤波 + 四元数表示 (避免万向节锁)
- ✅ **级联 PID**：姿态环 → 角速度环 → 电机输出 (10 个 PID 控制器)
- ✅ **高度控制**：TOF 测距 + 速度环双环控制
- ✅ **视觉定位**：V 形标记检测 + PCA 方向估计
- ✅ **传感器融合**：IMU + TOF + 光流 + 摄像头

## 硬件平台

| 组件 | 型号 | 说明 |
|------|------|------|
| **主控** | Infineon CYT4BB7 | 双核 ARM Cortex-M7 @ 250MHz, 768KB SRAM |
| **IMU** | IMU963RA | 9 轴 (陀螺仪 + 加速度计 + 磁力计) |
| **测距** | DL1B TOF | 激光测距传感器，100Hz 更新频率 |
| **光流** | UP-FLOW-302 | UART 光流传感器，用于水平定位 |
| **摄像头** | MT9V03X | 188×120 灰度摄像头，用于视觉定位 |
| **电机** | MT2204 KV2300 | 无刷电机 × 4，配合 5 寸三叶桨 |
| **机架** | X 型四旋翼 | 对角电机布局，利于姿态控制 |

## 软件架构

### 控制流程

```
IMU (1000Hz) → Mahony 姿态解算 → 四元数 → 欧拉角
                       ↓
    目标姿态 → 姿态 PID → 目标角速度 → 角速度 PID → 油门修正
                                                    ↓
    TOF (50Hz) → 高度 PID → 目标速度 → 速度 PID → 油门修正
                                                    ↓
                                            电机混控 → PWM 输出
```

### 核心算法

#### 1. Mahony 互补滤波

- **输入**：陀螺仪 (角速度) + 加速度计 (重力方向)
- **输出**：四元数姿态 + 欧拉角 (Roll/Pitch/Yaw)
- **参数**：Kp=2.0, Ki=0.008, dt=1ms
- **滤波**：陀螺仪 150Hz 低通，加速度 80Hz 低通

#### 2. 级联 PID 控制

| 控制环 | 输入 | 输出 | 频率 |
|--------|------|------|------|
| 姿态环 | 角度误差 (°) | 目标角速度 (°/s) | 1000Hz |
| 角速度环 | 角速度误差 (°/s) | 电机油门 (0-7000) | 1000Hz |
| 高度环 | 高度误差 (m) | 目标速度 (m/s) | 50Hz |
| 速度环 | 速度误差 (m/s) | 油门修正 | 50Hz |

#### 3. 电机混控 (X 型布局)

```
     D (左上)       C (右上)
        ╲         ╱
          ╲     ╱
            ╳           ← 机体中心
          ╱     ╲
        ╱         ╲
     B (左下)       A (右下)
```

混控公式：
```
A = 基础油门 - Roll + Pitch + Yaw + 高度
B = 基础油门 + Roll + Pitch - Yaw + 高度
C = 基础油门 - Roll - Pitch - Yaw + 高度
D = 基础油门 + Roll - Pitch + Yaw + 高度
```

### 视觉算法 (CM7_1 核心)

1. **图像采集**：MT9V03X @ 188×120，二值化阈值 58.7
2. **框架检测**：连通域分析 + 最小外接旋转矩形 (0.2° 精度)
3. **V 形提取**：PCA 主轴 + 框架轴融合 (α=0.28)
4. **方向判别**：基于方差的尖端检测 (抗阈值变化)

## 项目结构

```
drone-flight-controller/
├── project/
│   ├── code/                  # 飞控算法
│   │   ├── Attitude_Est.c/h  # Mahony 姿态解算
│   │   ├── PID.c/h           # PID 控制器
│   │   ├── Motor.c/h         # 电机混控
│   │   ├── Filter.c/h        # 贝塞尔滤波器
│   │   └── cam_share.h       # 核间通信定义
│   ├── user/                  # 用户代码
│   │   ├── main_cm7_0.c      # CM7_0 主程序 (飞控)
│   │   ├── main_cm7_1.c      # CM7_1 主程序 (视觉)
│   │   ├── cm7_0_isr.c       # CM7_0 中断服务
│   │   └── cm7_1_isr.c       # CM7_1 中断服务
│   └── iar/                   # IAR 工程文件
│       ├── cyt4bb7.eww       # 工作区文件
│       └── project_config/   # 构建配置
└── libraries/                 # 驱动库
    ├── sdk/                  # 英飞凌 SDK
    ├── zf_driver/            # 逐飞驱动层 (14 个驱动)
    ├── zf_device/            # 逐飞设备层 (27 个设备)
    ├── zf_common/            # 公共工具 (8 个模块)
    └── zf_components/        # 调试组件
```

## 快速开始

### 环境要求

- **开发工具**：IAR Embedded Workbench 9.40.1 或更高版本
- **调试器**：CMSIS-DAP 或 J-Link
- **操作系统**：Windows 10/11 (IAR 仅支持 Windows)

### 编译步骤

1. 克隆仓库：
```bash
git clone https://github.com/xiangbei007/drone-flight-controller.git
cd drone-flight-controller
```

2. 打开 IAR 工程：
```
project/iar/cyt4bb7.eww
```

3. 编译两个核心：
   - 选择 `cyt4bb7_cm_7_0` 工程 → `Project → Rebuild All`
   - 选择 `cyt4bb7_cm_7_1` 工程 → `Project → Rebuild All`

4. 烧录：
   - `Project → Download → Download Active Application`
   - 或使用 `Download and Debug (Ctrl+D)`

### 硬件连接

#### 调试接口
- CMSIS-DAP → CYT4BB7 核心板 SWD 接口

#### 传感器接口
| 传感器 | 接口 | 引脚 |
|--------|------|------|
| IMU963RA | SPI | SCK/MOSI/MISO/CS |
| DL1B TOF | UART | TX/RX |
| UP-FLOW-302 | UART2 | P10_0 (RX) / P10_1 (TX) |
| MT9V03X | DVPCAM | 数据线 + PCLK + HREF + VSYNC |

#### 电机接口
| 电机位置 | PWM 引脚 |
|---------|---------|
| 左上 (D) | P08_1 (TCPWM_CH20) |
| 右上 (C) | P05_2 (TCPWM_CH11) |
| 左下 (B) | P17_3 (TCPWM_CH58) |
| 右下 (A) | P10_3 (TCPWM_CH31) |

### 调试输出

通过 UART 输出调试信息 (115200 波特率)：
```
UP-FLOW-302 optical flow sensor initialized.
[FLOW] State:1, Valid:245, X:2, Y:-15, Time:12345
Gyro Bias: X=0.12, Y=-0.08, Z=0.05 deg/s
Acc Earth Bias: X=0.0012 Y=-0.0008 Z=0.9998 g
```

## 分支说明

### main 分支
基础飞控系统，包含：
- ✅ 姿态估计与控制
- ✅ 高度控制
- ✅ 视觉 V 形标记检测
- ✅ 电机混控输出

### dev 分支
集成光流传感器，新增功能：
- ✅ UP-FLOW-302 光流初始化
- ✅ UART2 光流数据接收
- ✅ 光流数据打印验证
- 🚧 光流位置控制 (开发中)

## 开发路线

### 已完成 ✅
- [x] 姿态解算 (Mahony + 四元数)
- [x] 级联 PID 控制 (姿态 + 角速度 + 高度)
- [x] V 形视觉定位
- [x] 光流数据接收 (阶段 1)

### 进行中 🚧
- [ ] 光流速度控制 (阶段 2)
- [ ] 光流定点悬停 (阶段 3)
- [ ] 配平值自动校准
- [ ] 遥控器接入

### 计划中 📋
- [ ] 二维码识别定位
- [ ] 多点航线规划
- [ ] 失控保护
- [ ] 数据记录与回放

## 性能指标

| 指标 | 数值 |
|------|------|
| **姿态更新频率** | 1000 Hz |
| **高度更新频率** | 50 Hz |
| **视觉更新频率** | 异步 (约 30-50 fps) |
| **姿态角精度** | ±2° (静止) |
| **高度保持精度** | ±5 cm (0.5-1.5m) |
| **响应时间** | <50 ms (姿态调整) |

## PID 参数参考

### 姿态环 (Roll/Pitch)
```c
Kp = 5.0    // 角度 → 角速度
Ki = 0.001  // 消除静差
Kd = 0.3    // 抑制超调
```

### 角速度环 (Roll/Pitch Rate)
```c
Kp = 6.0    // 角速度 → 油门
Ki = 0.02   // 累积误差
Kd = 0.0    // 一般不用微分
```

### 高度环
```c
Kp = 6.927  // 高度 → 速度
Ki = 0.0013 // 消除静差
```

### 速度环
```c
Kp = 233.0  // 速度 → 油门
Ki = 0.0    // 不用积分 (防止积分饱和)
```

## 常见问题

### Q1: 编译报错 "file not found"
**A:** 确保 IAR 版本 ≥ 9.40.1，且所有库文件完整。

### Q2: 烧录失败
**A:** 检查 CMSIS-DAP 连接，确认核心板供电正常，尝试复位后重新烧录。

### Q3: IMU 数据异常
**A:** 检查 SPI 接线，确认 IMU 供电 3.3V，初始化时保持静止。

### Q4: 电机不转或转速异常
**A:** 检查 PWM 引脚连接，确认电调校准 (500-1200μs)，基础油门 ≥ 4200。

### Q5: 飞机一直漂移
**A:** 可能原因：
- 配平值不准 (修改 `main_cm7_0.c` 中的 +2.783° 和 +3.17°)
- 基础油门过低 (提升到 5200)
- 光流未集成 (切换到 `dev` 分支)

## 贡献指南

欢迎提交 Issue 和 Pull Request！

### 提交规范
- `feat`: 新功能
- `fix`: 修复 bug
- `docs`: 文档更新
- `refactor`: 代码重构
- `perf`: 性能优化

### 示例
```
feat: add optical flow position control

- Implement position PID controller
- Integrate optical flow data with height compensation
- Add position hold mode
```

## 开源协议

本项目基于 GPL 3.0 开源协议。

- 允许商业使用和修改
- 修改后的代码必须开源
- 保留原作者版权声明

详见 [LICENSE](libraries/doc/GPL3_permission_statement.txt)

## 致谢

- [逐飞科技](https://seekfree.taobao.com/) - CYT4BB7 开源库
- [Infineon](https://www.infineon.com/) - TRAVEO™ II SDK
- [全国大学生智能汽车竞赛](https://smartcar.cdstm.cn/)

## 联系方式

- **GitHub**: [@xiangbei007](https://github.com/xiangbei007)
- **仓库**: [drone-flight-controller](https://github.com/xiangbei007/drone-flight-controller)
- **Email**: 2561340938@qq.com

---

**祝飞行顺利！** 🚁
