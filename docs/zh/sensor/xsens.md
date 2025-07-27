# Xsens

## 概述

**Xsens** 是全球领先的运动追踪与惯性导航解决方案提供商。其高性能 IMU、VRU、AHRS 和 GNSS/INS 系统具备卓越的精度与可靠性，助力空中机器人、无人机及各类无人平台实现先进的自主能力。

Xsens 传感器与 PX4 深度集成，即便在最严苛的环境下，也能为您的系统带来工业级导航性能。

---

## 为什么选择 Xsens 作为 PX4 解决方案？

使用 Xsens，释放您自主系统的全部潜能：

* **精确导航**
  行业领先的航向、俯仰和横滚精度，保障飞控与状态估算的高可信度。

* **可靠的 GNSS 性能**
  即使在 GNSS 信号弱或多路径环境下，也能实现稳定、连续的定位。

* **即插即用，部署快速**
  PX4 驱动原生支持，轻松集成，几乎无需额外配置。

* **灵活的运行模式**
  可选择完整导航输出或原始传感器数据，根据系统架构灵活切换。

---

## 集成选项

PX4 提供两种灵活模式以集成 Xsens 设备：

### 1. **外部 INS 模式**

将 Xsens 用作**主导航源**，绕过 PX4 的内部估算器（EKF2）。适用于需要完整 GNSS/INS 融合、并带有板载航向与定位功能的系统。

### 2. **仅传感器模式**

将高质量的 IMU、磁力计和气压计数据从 Xsens 输入到 PX4 的估算器中。保留 EKF2，同时受益于高级别的传感器输入。

---

## 推荐 Xsens 设备

PX4 支持所有 Xsens MTi 系列 GNSS/INS 单元。以下是根据性能和应用推荐的型号：

### 🥇 **旗舰性能**

**MTi-G-710 GNSS/INS**
适用于恶劣环境的顶级导航设备，轻松应对振动、温度变化和快速动态响应。

### 📍 **厘米级 RTK 定位**

**MTi-680(G) GNSS/INS**
为基于 RTK 的系统优化，实现厘米级精度。

### 💡 **高性价比之选**

**MTi-7 / MTi-8 GNSS/INS**
适合对成本敏感、但仍需高质量导航的平台。

完整规格与比较请参见：[Xsens MTi 产品文档](https://mtidocs.movella.com/mti-documentation-overview)

---

## 硬件安装

### 电气连接

**UART 型号（MTi-7/8/670/680）**

* 连接至飞控的任意可用串口（GPS2、TELEM 等）
* 直接 UART 连接，无需信号转换

**RS232 型号（MTi-670G/680G/G-710）**

* 将 RS232 信号转换为 UART 电平
* **注意**：将 RS232 的 CTS 引脚拉高（+3V 至 +24V）以禁用硬件流控

### 安装方向

**方向要求**

* Xsens 使用 ENU（东-北-天）坐标系
* PX4 驱动自动转换为 NED（北-东-地）
* 设置 `SENS_XSENS_ORIEN` 参数：

  * `1`：传感器朝上安装（默认）
  * `0`：传感器朝下安装

**GNSS/INS 安装要求**

* GNSS 天线需刚性固定于惯性传感器上
* 天线需无遮挡，确保 GNSS 性能最佳

**附加资源**

* [Xsens 安装技巧](https://mtidocs.movella.com/installation-tips-and-tricks)
* [MTi 系统概览](https://mtidocs.movella.com/mti-system-overview)

---

## PX4 配置

### 步骤 1：启用驱动

在板卡配置中添加以下选项之一：

```
CONFIG_DRIVERS_INS_XSENS
```

或

```
CONFIG_COMMON_INS
```

详见 [PX4 Kconfig 板卡配置指南](../hardware/porting_guide_config.md#px4-board-configuration-kconfig)

### 步骤 2：配置串口

设置 [`SENS_XSENS_CFG`](../advanced_config/parameter_reference.md#SENS_XSENS_CFG) 参数指定所连接的串口。

参考 [串口配置指南](../peripherals/serial_configuration.md)

### 步骤 3：配置磁力计

由于 Xsens GNSS/INS 已包含磁力计，建议禁用 PX4 内部磁力计的起飞前检查：

参数设置：[`SYS_HAS_MAG`](../advanced_config/parameter_reference.md#SYS_HAS_MAG) = `0`

### 步骤 4：选择运行模式

配置 [`SENS_XSENS_MODE`](../advanced_config/parameter_reference.md#SENS_XSENS_MODE)

#### INS 模式（推荐）

* 设置值：`1`
* 行为：Xsens 成为主导航源，EKF2 自动禁用
* 适用场景：提供完整导航解决方案，精度最高

#### 仅传感器模式

* 设置值：`0`
* 行为：提供原始传感器数据至 PX4 估算器
* 额外设置：若保留内部 IMU，需配置传感器优先级：

  * [`CAL_GYROn_PRIO`](../advanced_config/parameter_reference.md#CAL_GYRO0_PRIO)
  * [`CAL_ACCn_PRIO`](../advanced_config/parameter_reference.md#CAL_ACC0_PRIO)
  * [`CAL_BAROn_PRIO`](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO)
  * [`CAL_MAGn_PRIO`](../advanced_config/parameter_reference.md#CAL_MAG0_PRIO)

> **注意**：外部 IMU 通常显示为编号最大的实例。可使用 `uorb top -1` 和 `listener` 命令确认设备 ID，查阅 `CAL_GYROn_ID`。优先级范围：0（禁用）至 255（最高）

### 步骤 5：配置数据输出频率

设置 [`SENS_XSENS_ODR`](../advanced_config/parameter_reference.md#SENS_XSENS_ODR)

**默认值**：100 Hz

**最大支持速率**：

* MTi-7/8：100 Hz
* MTi-670(G)、MTi-680(G)、MTi-G-710：400 Hz

> **重要**：更改参数后需重启 PX4 生效

### 步骤 6：配置通信速率

**默认波特率**：[`SENS_XSENS_BAUD`](../advanced_config/parameter_reference.md#SENS_XSENS_BAUD) = `115200`

**高频率应用场景**：若需 400Hz 输出，建议设为 `921600` 波特率以确保稳定通信

确保 Xsens 设备的波特率设置与 PX4 一致。可通过 Xsens MT Manager 修改设备波特率。

### 步骤 7：设置物理朝向

配置 [`SENS_XSENS_ORIEN`](../advanced_config/parameter_reference.md#SENS_XSENS_ORIEN)：

* `0`：朝下安装
* `1`：朝上安装（默认）

### 步骤 8：应用配置

**重启 PX4** 以使所有参数生效。系统启动时将自动检测并初始化 Xsens 驱动。

---

## Xsens 设备配置

### 自动配置

PX4 在初始化时自动设置以下 Xsens 输出项：

**输出内容**：

* 数据包计数器、精细采样时间、UTC 时间
* 姿态四元数、角速度、加速度
* 磁场、温度、状态字
* 纬度/经度、椭球高程、速度

**坐标系配置**：

* RotLocal：NED 坐标系
* RotSensor：根据 `SENS_XSENS_ORIEN` 设置

**通信配置**：

* 自动检测波特率参数并设置，默认值为 115200 bps

### 手动配置（如有必要）

使用 [Xsens MT Manager](https://www.movella.com/support/software-documentation) 进行手动设置：

#### 关键设置项：

**波特率**

* 若输出频率超过 200Hz，需提高波特率以避免数据溢出

**GNSS 杠杆臂（Lever Arm）**

* RTK 设备（如 MTi-8、680G）需配置
* 用于补偿天线与惯性传感器之间的位置偏差

**GNSS 配置**

* 可启用其他星座系统（如北斗）
* 针对特定应用优化 GNSS 行为

**u-blox GNSS 平台选择**

* 选择合适的平台（如“Airborne <2g”）以匹配动态特性

**滤波设置**

* 低速下启用“GeneralMag”提高磁力辅助航向稳定性
* 可根据实际使用环境优化

> **提示**：点击 MT Manager 中的“Apply”按钮，将配置保存至传感器非易失性存储中。

---

## 数据输出

### 固定发布话题

驱动会向以下 uORB 话题发布传感器数据：

* [`sensor_accel`](../msg_docs/SensorAccel.md)
* [`sensor_gyro`](../msg_docs/SensorGyro.md)
* [`sensor_mag`](../msg_docs/SensorMag.md)
* [`sensor_baro`](../msg_docs/SensorBaro.md)
* [`sensor_gps`](../msg_docs/SensorGps.md)

### INS 模式下附加话题

配置为外部 INS 时，还会发布：

* [`vehicle_local_position`](../msg_docs/VehicleLocalPosition.md)
* [`vehicle_global_position`](../msg_docs/VehicleGlobalPosition.md)
* [`vehicle_attitude`](../msg_docs/VehicleAttitude.md)

### 仅传感器模式下附加话题

配置为仅传感器模式时，还会发布：

* `external_ins_local_position`
* `external_ins_global_position`
* `external_ins_attitude`

> **验证方法**：使用 `listener` 命令监控话题数据，确保正常发布。

---

## 采购信息

* **官网**：[Movella 自动化与移动解决方案](https://www.movella.com/applications/automation-mobility)
* **邮箱**：[china@movella.com](mailto:china@movella.com)

---

## 技术支持

### 文档资料

* [MTi 系统文档](https://mtidocs.movella.com/mti-documentation-overview)
* **邮件支持**：[support@movella.com](mailto:support@movella.com)

### 软件工具

* [MT 软件套件](https://www.movella.com/support/software-documentation)，含 MT Manager 设备配置工具
