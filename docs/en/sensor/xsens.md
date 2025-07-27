# Xsens

## Overview

**Xsens** is a global leader in motion tracking and inertial navigation solutions. Their high-performance IMU, VRU, AHRS, and GNSS/INS systems deliver exceptional accuracy and reliability—enabling advanced autonomy across aerial robotics, drones, and unmanned platforms.

Seamlessly integrated with PX4, Xsens sensors bring industrial-grade navigation performance to your system, even in the most demanding environments.

---

## Why Choose Xsens for PX4?

Unlock the full potential of your autonomous system with Xsens:

* **Precision Navigation**
  Industry-leading heading, pitch, and roll accuracy for confident flight control and state estimation.

* **Reliable GNSS Performance**
  Consistent and stable positioning—even in GNSS-degraded or multipath environments.

* **Fully Integrated, Fast Deployment**
  Plug-and-play PX4 driver support enables quick integration with minimal configuration.

* **Flexible Operation Modes**
  Choose between full navigation output or raw sensor data—depending on your architecture needs.

---

## Integration Options

PX4 offers two flexible modes to incorporate Xsens devices:

### 1. **External INS Mode**

Use Xsens as the **primary navigation source**, bypassing PX4’s internal estimator (EKF2). Ideal for systems requiring full GNSS/INS fusion with onboard heading and position.

### 2. **Sensor-Only Mode**

Feed high-quality IMU, magnetometer, and barometer data from Xsens into PX4’s estimator pipeline. Retain EKF2, but benefit from premium-grade sensor input.

---

## Recommended Xsens Devices

All Xsens MTi-series GNSS/INS units are supported by PX4. Below are some recommended options based on performance and application requirements:

### 🥇 **Flagship Performance**

**MTi-G-710 GNSS/INS**
Top-tier navigation for harsh conditions. Handles vibration, temperature swings, and fast dynamics with ease.

### 📍 **Precision RTK Positioning**

**MTi-680(G) GNSS/INS**
Optimized for centimeter-level accuracy in RTK-based systems.

### 💡 **Affordable & Capable**

**MTi-7 / MTi-8 GNSS/INS**
Great value for cost-sensitive platforms that still demand quality navigation.

For full technical specs and comparisons, visit the [Xsens MTi Product Documentation](https://mtidocs.movella.com/mti-documentation-overview).

---

## Hardware Installation

### Electrical Connections

**UART Models (MTi-7/8/670/680)**
- Connect to any available flight controller serial port (GPS2, TELEM, etc.)
- Direct UART connection - no signal conversion required

**RS232 Models (MTi-670G/680G/G-710)**
- Convert RS232 to UART signal levels
- **Important**: Tie RS232 CTS to high (+3V to +24V) to disable hardware flow control

### Physical Mounting

**Orientation Requirements**
- Xsens uses East-North-Up (ENU) coordinate system
- Xsens PX4 driver automatically converts to North-East-Down (NED)
- Set `SENS_XSENS_ORIEN` parameter based on mounting:
  - `1`: Sensor mounted upwards (default)
  - `0`: Sensor mounted downwards

**GNSS/INS Specific Requirements**
- Mount GNSS antenna rigidly relative to the inertial sensor
- Ensure unobstructed sky view for optimal GNSS performance

**Additional Resources**
- [Xsens Installation Tips and Tricks](https://mtidocs.movella.com/installation-tips-and-tricks)
- [MTi System Overview](https://mtidocs.movella.com/mti-system-overview)

---

## PX4 Configuration

### Step 1: Enable Driver in Firmware

Add one of these options to your board configuration:
```
CONFIG_DRIVERS_INS_XSENS
```
or
```
CONFIG_COMMON_INS
```

Refer to the [PX4 Kconfig Board Configuration Guide](../hardware/porting_guide_config.md#px4-board-configuration-kconfig) for detailed instructions.

### Step 2: Configure Serial Port

Set [`SENS_XSENS_CFG`](../advanced_config/parameter_reference.md#SENS_XSENS_CFG) to specify the connected serial port.

See [Serial Port Configuration Guide](../peripherals/serial_configuration.md) for available ports.

### Step 3: Configure Magnetometer

Since the Xsens GNSS/INS includes a magnetometer, disable the PX4 internal magnetometer preflight check:

**Parameter**: [`SYS_HAS_MAG`](../advanced_config/parameter_reference.md#SYS_HAS_MAG) = `0`

### Step 4: Select Operating Mode

Configure [`SENS_XSENS_MODE`](../advanced_config/parameter_reference.md#SENS_XSENS_MODE):

#### INS Mode (Recommended)
- **Setting**: `1`
- **Behavior**: Xsens becomes primary navigation source; EKF2 automatically disabled
- **Use Case**: Full navigation solution with maximum accuracy

#### Sensors-Only Mode
- **Setting**: `0`
- **Behavior**: Provides raw sensor data to PX4 estimator
- **Additional Configuration**: Set sensor priorities if internal IMU remains active:
  - [`CAL_GYROn_PRIO`](../advanced_config/parameter_reference.md#CAL_GYRO0_PRIO)
  - [`CAL_ACCn_PRIO`](../advanced_config/parameter_reference.md#CAL_ACC0_PRIO)
  - [`CAL_BAROn_PRIO`](../advanced_config/parameter_reference.md#CAL_BARO0_PRIO)
  - [`CAL_MAGn_PRIO`](../advanced_config/parameter_reference.md#CAL_MAG0_PRIO)

> **Note**: External IMUs typically appear as the highest-numbered instance. Use `uorb top -1` and `listener` commands to identify the correct instance. Check `CAL_GYROn_ID` for device ID confirmation. Priority range: 0 (disabled) to 255 (highest).

### Step 5: Configure Data Rate

Set [`SENS_XSENS_ODR`](../advanced_config/parameter_reference.md#SENS_XSENS_ODR) for desired output frequency:

**Default**: 100 Hz

**Maximum Rates**:
- MTi-7/8: 100 Hz
- MTi-670(G), MTi-680(G), MTi-G-710: 400 Hz

> **Important**: Restart PX4 after changing this parameter.

### Step 6: Configure Communication Speed

**Default Baud Rate**: [`SENS_XSENS_BAUD`](../advanced_config/parameter_reference.md#SENS_XSENS_BAUD) = `115200`

**High-Rate Applications**: For 400Hz output, increase to `921600` baud for reliable communication.

Ensure the baud rate matches the Xsens device configuration, you could manually change the baud rate using Xsens MT Manager - Device Settings.

### Step 7: Set Physical Orientation

Configure [`SENS_XSENS_ORIEN`](../advanced_config/parameter_reference.md#SENS_XSENS_ORIEN):
- `0`: Downward-facing installation
- `1`: Upward-facing installation (default)

### Step 8: Apply Configuration

**Restart PX4** to activate all parameter changes. The system will automatically detect and initialize the Xsens driver on boot.

---

## Xsens Device Configuration

### Automatic Configuration

PX4 automatically configures these Xsens settings on initialization:

**Data Outputs**
- PacketCounter, SampleTimeFine, UTC Time
- Orientation Quaternion, RateOfTurn, Acceleration
- Magnetic Field, Temperature, Status Word
- Latitude/Longitude, Ellipsoid Altitude, Velocity

**Coordinate Systems**
- RotLocal: NED frame
- RotSensor: Based on `SENS_XSENS_ORIEN` setting

**Communication**
- detect baud rate parameter and sets to the corresponding value, default is 115200 bps

### Manual Configuration Requirements

Use [Xsens MT Manager](https://www.movella.com/support/software-documentation) to configure:

#### Essential Settings

**Baudrate**
- If you use data output rate more than 200Hz, change your baudrate to higher value to avoid data overflow.

**GNSS Lever Arm**
- Required for RTK GNSS/INS products (MTi-8, MTi-680G)
- Compensates for antenna/sensor offset

**GNSS Configuration**
- Enable additional constellations (e.g., BeiDou)
- Optimize for specific applications

**u-blox GNSS Platform**
- Select appropriate platform (e.g., "Airborne <2g")
- Matches expected dynamics

**Filter Settings**
- Change to "GeneralMag" for magnetic-aided yaw at low speeds
- Optimize for operational environment

> **Tip**: Click "Apply" in MT Manager to save settings to sensor's non-volatile memory.

---

## Data Output

### Always Published Topics

The driver publishes sensor data to these uORB topics:
- [`sensor_accel`](../msg_docs/SensorAccel.md)
- [`sensor_gyro`](../msg_docs/SensorGyro.md)
- [`sensor_mag`](../msg_docs/SensorMag.md)
- [`sensor_baro`](../msg_docs/SensorBaro.md)
- [`sensor_gps`](../msg_docs/SensorGps.md)

### INS Mode Additional Topics

When configured as external INS:
- [`vehicle_local_position`](../msg_docs/VehicleLocalPosition.md)
- [`vehicle_global_position`](../msg_docs/VehicleGlobalPosition.md)
- [`vehicle_attitude`](../msg_docs/VehicleAttitude.md)

### Sensors-Only Mode Additional Topics

When configured for sensors-only:
- `external_ins_local_position`
- `external_ins_global_position`
- `external_ins_attitude`

> **Verification**: Use the `listener` command to monitor published topics and verify proper operation.

---

## Procurement

- **Website**: [Movella Automation & Mobility](https://www.movella.com/applications/automation-mobility)
- **Email**: info@xsens.com

---

## Technical Support

### Documentation

- [MTi Documentation](https://mtidocs.movella.com)
- **Support via Email**: support@movella.com

### Software Tools
- [MT Software Suite](https://www.movella.com/support/software-documentation) with MT Manager for device configuration
