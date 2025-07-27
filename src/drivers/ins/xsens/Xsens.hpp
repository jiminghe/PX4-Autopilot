/**
 * @file Xsens.hpp
 * @brief Driver for Xsens INS devices (MTi-680G, MTi-670G, MTi-G-710, MTi-7, MTi-8)
 */

#pragma once

#include <stdio.h>
#include <inttypes.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

#include "xbus.h"
#include "xbus_parser.h"
#include "xbus_message_id.h"

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <lib/drivers/device/Device.hpp>
#include <lib/drivers/accelerometer/PX4Accelerometer.hpp>
#include <lib/drivers/gyroscope/PX4Gyroscope.hpp>
#include <lib/drivers/magnetometer/PX4Magnetometer.hpp>
#include <lib/geo/geo.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <uORB/Publication.hpp>
#include <uORB/SubscriptionInterval.hpp>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/sensor_baro.h>
#include <uORB/topics/sensor_gps.h>
#include <uORB/topics/sensor_selection.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_odometry.h>
#include <drivers/drv_sensor.h>
#include <px4_platform_common/Serial.hpp>

using namespace time_literals;

class Xsens : public ModuleBase<Xsens>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	Xsens(const char *port);
	~Xsens() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

private:
	enum class DeviceState {
		ENTERING_CONFIG_MODE,
		WAITING_FOR_CONFIG_MODE,
		CONFIGURING_OUTPUT,
		WAITING_FOR_OUTPUT_CONFIG,
		CONFIGURING_ROTLOCAL,
		WAITING_FOR_ROTLOCAL_CONFIG,
		CONFIGURING_ROTSENSOR,
		WAITING_FOR_ROTSENSOR_CONFIG,
		ENTERING_MEASUREMENT_MODE,
		WAITING_FOR_MEASUREMENT_MODE,
		RUNNING,
		ERROR
	};

	bool init();
	void Run() override;

	// Serial communication
	bool openSerialPort();
	void closeSerialPort();
	bool writeMessage(const uint8_t *message, size_t length);
	size_t readData(uint8_t *buffer, size_t max_length);

	// Message handling
	void processReceivedData();
	void handleMessage(const uint8_t *message);
	void handleMTData2Message(const uint8_t *message);
	void handleAckMessage(uint8_t messageId);
	void handleErrorMessage(const uint8_t *message);

	// Device configuration
	bool gotoConfigMode();
	bool configureOutputSettings();
	bool setNedFrame();
	bool setSensorFrame(bool isUp);
	bool gotoMeasurementMode();

	// State machine
	void updateStateMachine();
	void setState(DeviceState new_state);
	const char* getStateString(DeviceState state) const;

	// Utility functions
	void publishSensorData(const SensorData &data);
	void publishAttitude(const SensorData &data);
	void publishLocalPosition(const SensorData &data);
	void publishGlobalPosition(const SensorData &data);
	void publishGNSS(const SensorData &data);
	void publishBarometer(const SensorData &data);
	void publishAcceleration(const SensorData &data);
	void publishRateOfTurn(const SensorData &data);
	void publishMagneticField(const SensorData &data);

	// Temperature management
	void updateSensorTemperatures(const SensorData &data);

	// UTC time conversion utilities
	uint64_t convertUtcTimeToMicroseconds(const UtcTime &utcTime);
	uint64_t convertUtcTimeToUnixMicroseconds(const UtcTime &utcTime);

	// GPS fix type determination
	uint8_t determineFixType(uint32_t status);

	// return the square of two floating point numbers
	static constexpr float sq(float var) { return var * var; }

	// Member variables
	device::Serial *_serial{nullptr};
	char _port[20] {};
	DeviceState _state{DeviceState::ENTERING_CONFIG_MODE};

	bool _initialized{false};
	bool _configured{false};

	hrt_abstime _last_message_time{0};
	hrt_abstime _state_timeout{0};
	hrt_abstime _last_sensor_data_time{0};

	// Receive buffer
	static constexpr size_t BUFFER_SIZE = 1024;
	uint8_t _rx_buffer[BUFFER_SIZE];
	size_t _rx_buffer_pos{0};

	// Message buffer for parsing
	static constexpr size_t MAX_MESSAGE_SIZE = 512;
	uint8_t _message_buffer[MAX_MESSAGE_SIZE];

	// Current sensor temperature (in Celsius)
	float _current_temperature{NAN};

	// PX4 sensor objects
	PX4Accelerometer _px4_accel{0};
	PX4Gyroscope _px4_gyro{0};
	PX4Magnetometer _px4_mag{0};

	// Position reference
	MapProjection _pos_ref{};
	float _gps_alt_ref{NAN};

	// uORB publications
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};

	uORB::PublicationMulti<sensor_baro_s> _sensor_baro_pub{ORB_ID(sensor_baro)};
	uORB::PublicationMulti<sensor_gps_s> _sensor_gps_pub{ORB_ID(sensor_gps)};
	uORB::Publication<sensor_selection_s> _sensor_selection_pub{ORB_ID(sensor_selection)};

	uORB::PublicationMulti<vehicle_attitude_s> _attitude_pub;
	uORB::PublicationMulti<vehicle_local_position_s> _local_position_pub;
	uORB::PublicationMulti<vehicle_global_position_s> _global_position_pub;
	uORB::Publication<estimator_status_s> _estimator_status_pub{ORB_ID(estimator_status)};

	// Performance counters
	perf_counter_t _sample_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": read")};
	perf_counter_t _comms_errors{perf_alloc(PC_COUNT, MODULE_NAME": com_err")};
	perf_counter_t _timeout_errors{perf_alloc(PC_COUNT, MODULE_NAME": timeout")};

	perf_counter_t _accel_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": accel publish interval")};
	perf_counter_t _gyro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": gyro publish interval")};
	perf_counter_t _mag_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": mag publish interval")};
	perf_counter_t _gnss_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": GNSS publish interval")};
	perf_counter_t _baro_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": baro publish interval")};

	perf_counter_t _attitude_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": attitude publish interval")};
	perf_counter_t _local_position_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": local position publish interval")};
	perf_counter_t _global_position_pub_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": global position publish interval")};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_XSENS_MODE>) _param_xsens_mode,
		(ParamInt<px4::params::SENS_XSENS_BAUD>) _param_xsens_baud,
		(ParamInt<px4::params::SENS_XSENS_ODR>) _param_xsens_odr,
		(ParamInt<px4::params::SENS_XSENS_ORIEN>) _param_xsens_orient
	)
};
