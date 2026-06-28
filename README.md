# Hero_Robot_H723

> 基于 STM32H723 的 RoboMaster 英雄机器人电控系统

## 项目简介

本项目是 RoboMaster 机甲大师赛英雄机器人（Hero Robot）的嵌入式电控代码，基于 **STM32H723VGT6**（Cortex-M7，480MHz）设计。采用 **FreeRTOS** 实时操作系统，CMake 构建系统，实现多任务并发的机器人运动控制、通信和状态管理。

## 硬件平台

| 项目       | 规格                        |
| ---------- | --------------------------- |
| 主控芯片   | STM32H723VGT6 (Cortex-M7)   |
| 主频       | 480MHz                      |
| FPU        | 单精度 + 双精度硬件浮点加速 |
| RTOS       | FreeRTOS V10.3.1            |
| 构建系统   | CMake + STM32CubeMX         |

## 软件架构

```
Hero_Robot_H723/
├── Core/                   # STM32 HAL 层代码（CubeMX 生成）
│   ├── Inc/                # 头文件（FreeRTOSConfig, main, dma, fdcan...）
│   └── Src/                # 源文件（main, freertos, 中断服务...）
├── Drivers/                # STM32H7 HAL 驱动库
├── Middlewares/            # FreeRTOS 中间件
├── USB_DEVICE/             # USB CDC 虚拟串口
├── USER/
│   ├── Algorithm/          # 算法库
│   │   ├── PID/            # PID 控制器
│   │   ├── kalman_filter/  # 卡尔曼滤波器
│   │   ├── MahonyAHRS/     # Mahony 姿态解算
│   │   ├── QuaternionEKF/  # 四元数扩展卡尔曼滤波
│   │   ├── Filter/         # 滤波器（低通、均值等）
│   │   ├── Ramp/           # 斜坡发生器
│   │   ├── Crc/            # CRC 校验
│   │   ├── Fifo/           # FIFO 数据结构
│   │   ├── arm_math/       # ARM DSP 矩阵运算库
│   │   └── ...
│   ├── Module/             # 外设模块驱动
│   │   ├── Bmi088/         # BMI088 IMU 驱动
│   │   ├── DJ_Motor/       # 大疆电机驱动（CAN）
│   │   ├── DM_Motor_Fdcan/ # DM 电机 FDCAN 驱动
│   │   ├── Fdcan/          # FDCAN 通信封装
│   │   ├── Rc/             # 遥控器（DBUS/SBUS）
│   │   ├── Referee/        # 裁判系统
│   │   ├── Msg/            # 消息通信框架
│   │   ├── Dwt/            # DWT 延时
│   │   ├── Ui/             # UI 显示
│   │   └── Vofa+/          # Vofa+ 调试协议
│   └── Task/               # FreeRTOS 任务
│       ├── Chassis/        # 底盘控制任务
│       ├── Gimbal/         # 云台控制任务
│       ├── Shoot/          # 发射器控制任务
│       ├── Cmd/            # 控制指令融合任务
│       ├── Ins/            # 惯性导航/姿态解算任务
│       ├── Motor/          # 电机控制任务
│       ├── Transmission/   # 上位机通信任务
│       ├── Referee/        # 裁判系统通信任务
│       ├── Supercap/       # 超级电容任务
│       └── Robot_Config/   # 机器人配置文件
└── CtrBoard-H7_ALL.ioc     # STM32CubeMX 项目配置
```

## 功能模块

### 任务系统（FreeRTOS Tasks）

| 任务               | 功能说明                           |
| ------------------ | ---------------------------------- |
| **Chassis**        | 底盘运动控制，支持麦克纳姆轮模式   |
| **Gimbal**         | 云台 Yaw/Pitch 双轴控制            |
| **Shoot**          | 弹丸发射控制（摩擦轮 + 拨弹）      |
| **CMD**            | 多源控制指令融合与分发             |
| **INS**            | 惯性导航/IMU 姿态解算              |
| **Motor**          | 电机状态采集与反馈                 |
| **Transmission**   | 与上位机（视觉/PC）数据交互        |
| **Referee**        | RoboMaster 裁判系统通信            |
| **Supercap**       | 超级电容电量管理与功率缓冲         |

### 算法库

- **PID 控制器**：增量式 / 位置式 PID
- **卡尔曼滤波器**：传感器数据融合
- **Mahony AHRS**：IMU 姿态解算
- **四元数 EKF**：姿态估计
- **斜坡发生器**：平滑控制量过渡
- **CRC8/CRC16**：数据校验
- **ARM DSP 库**：矩阵运算、FFT

### 通信协议

- **FDCAN**：与 DJI 电机、DM 电机高速通信
- **DBUS/SBUS**：遥控器信号解析
- **USB CDC**：虚拟串口调试
- **UART**：裁判系统、超级电容、上位机通信
- **SPI**：BMI088 IMU 数据读取

## 底盘配置

- **麦克纳姆轮模式**（`BSP_CHASSIS_MECANUM_MODE`）
- 支持功率限制模式（`BSP_USING_POWER_LIMIT`）

## 开发环境

| 工具            | 用途                  |
| --------------- | --------------------- |
| STM32CubeMX     | 外设初始化代码生成    |
| CMake (≥3.16)   | 构建系统              |
| GCC ARM (arm-none-eabi-gcc) | 交叉编译工具链 |
| CLion / VSCode  | IDE 推荐              |

## 编译与烧录

```bash
# 1. 进入构建目录
cd cmake-build-debug

# 2. CMake 配置
cmake ..

# 3. 编译
make -j$(nproc)

# 4. 使用 ST-Link / J-Link / OpenOCD 烧录
# 生成的固件：cmake-build-debug/CtrBoard-H7_ALL.elf
```

## 关键配置

机器人功能通过 `USER/Task/Robot_Config/rm_config.h` 中的宏开关进行配置：

```c
#define BSP_USING_DJI_MOTOR      // 使用大疆电机
#define BSP_USING_RC_DBUS        // DBUS 遥控器
#define BSP_USING_SUPERCAP       // 超级电容
#define BSP_CHASSIS_MECANUM_MODE // 麦克纳姆轮底盘
```

## 数据流架构

```
遥控器/上位机 ──→ CMD任务（指令融合）──→ Chassis任务（底盘控制）
                                      ├──→ Gimbal任务（云台控制）
                                      └──→ Shoot任务（发射控制）

INS任务（姿态） ──→ 发布姿态消息 ──→ 各控制任务订阅

Motor任务（电机反馈）──→ 发布电机状态 ──→ 控制任务闭环
```

## 版本历史

| 时间 | 更新内容 |
| ---- | -------- |
| 2026 | 添加超级电容支持 |
| 2025 | 重构为英雄机器人代码，CMake 重构 |
| 2025 | 取消线性插值，提高电机速度，重构串口通信 |
| 2025 | 开启 USB CDC 任务，CMake 独立于 CubeMX |
| 2025 | 手动移植 DSP 矩阵运算库，开启 FPU 双精度加速 |

## 致谢

- 基于湖南大学（HNU）嵌入式框架 H723 重构
- FreeRTOS 实时操作系统
- STM32 HAL 库
- ARM CMSIS DSP 库

---

**Author**: Chenshuo (Gleam) | **License**: TBD
