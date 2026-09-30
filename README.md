# 步兵整车电控固件（单板）

> STM32H723 + 达妙 DM 电机 + DJI M3508 + FreeRTOS

## 授权说明

本项目为**全新编写**的交付级代码，不包含任何第三方 `all rights reserved` 源码。
算法（PID、麦轮运动学、云台角度环、视觉协议解析）均为按公开协议/标准算法独立实现，
可随整车交付。ST HAL / CMSIS / FreeRTOS 分别遵循 ST / Apache-2.0 / MIT 授权。

## 硬件拓扑

| 子系统 | 电机 | 数量 | 总线 | 控制方式 |
| ---- | ---- | ---- | ---- | ---- |
| 底盘 | DJI M3508 | 4 | FDCAN1 | 速度环 + 电流环 |
| 云台 yaw | DM6220 | 1 | FDCAN2 | MIT（MCU 角度环+速度环，IMU 反馈） |
| 云台 pitch | DM4310 | 1 | FDCAN2 | POSVEL 原生位置速度模式 |
| 摩擦轮 | DJI M3508 | 2 | FDCAN2 | 速度环 + 电流环 |
| 拨盘 | DM2325 | 1 | FDCAN2 | MIT 速度模式 |

CAN 地址规划（见 `app/config.h`）：

```
FDCAN1: M3508 x4   (DJI id 1~4, 反馈 0x201~0x204)
FDCAN2: yaw   DM id 1 (MIT,    tx 0x01,  rx 0x11)
        pitch DM id 2 (POSVEL, tx 0x102, rx 0x12)
        loader DM id 3 (MIT,   tx 0x03,  rx 0x13)
        friction M3508 x2 (DJI id 1~2, 反馈 0x201~0x202)
```

## 目录结构

```
infantry/
├── Core/ Drivers/ Middlewares/  # CubeMX 生成（ST HAL + FreeRTOS），用户生成
├── bsp/                         # 板级支持：CAN / UART / DWT / GPIO / PWM / SPI
├── device/                      # 设备驱动：dji_motor / dm_motor / bmi088
├── algo/                        # 算法：pid / 姿态解算 / 数学工具
├── proto/                       # 协议：seasky 视觉 / remote(SBUS)
└── app/                         # 业务：config / gimbal / chassis / shoot / cmd / robot
```

## 任务布局（FreeRTOS）

| 任务 | 频率 | 内容 |
| ---- | ---- | ---- |
| ins_task | 1kHz | BMI088 读取 + 姿态解算 |
| motor_task | 1kHz | DM/DJI 电机闭环发送 |
| robot_task | 100Hz | cmd 分发 → 云台 → 发射 → 底盘 |
| daemon_task | 100Hz | 模块在线检测（离线急停） |

## 首次搭建步骤

1. 用 STM32CubeMX 新建 STM32H723 工程，按硬件接线配好 FDCAN1/FDCAN2、SPI(BMI088)、
   串口(视觉 VCP/遥控 SBUS)、定时器(PWM 摩擦轮/舵机可选)、FreeRTOS。
2. 将 CubeMX 生成的 `Core/ Drivers/ Middlewares/` 放入本目录。
3. 在 `HAL_FDCAN_RxFifo0Callback` 里调用 `BspCanOnRx(hfdcan)` 完成 CAN 帧分发。
4. 按 `app/config.h` 校准机械参数（轴距、轮径、俯仰限位、视觉 yaw 零偏等）。
5. `make` 编译，Ozone/J-Link 烧录调试。

## 移植自哨兵、但独立重写的部分

- 云台 yaw：MIT 模式 + IMU 角度环（惯性空间绝对角），视觉 yaw 为**绝对角度**，不叠加 θ。
- 云台 pitch：POSVEL 原生位置速度模式，视觉/摇杆 → 位置指令。
- 底盘：麦轮运动学 + 3508 速度环。
- 发射：摩擦轮 3508 速度环 + 拨盘 DM 速度模式 + 单发状态机 + 卡弹反转。
- 视觉：Seasky V1 协议（0xA5 帧头 + CRC16）。
