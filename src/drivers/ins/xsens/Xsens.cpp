#include "Xsens.hpp"

#include <lib/drivers/device/Device.hpp>
#include <px4_platform_common/getopt.h>
#include <drivers/drv_hrt.h>
#include <matrix/math.hpp>
#include <cstring>
#include <lib/drivers/device/Device.hpp>
#include <inttypes.h>

using matrix::Vector2f;
using matrix::Quatf;
using matrix::Eulerf;

Xsens::Xsens(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port)),
	_attitude_pub((_param_xsens_mode.get() == 0) ? ORB_ID(external_ins_attitude) : ORB_ID(vehicle_attitude)),
	_local_position_pub((_param_xsens_mode.get() == 0) ? ORB_ID(external_ins_local_position) : ORB_ID(vehicle_local_position)),
	_global_position_pub((_param_xsens_mode.get() == 0) ? ORB_ID(external_ins_global_position) : ORB_ID(vehicle_global_position))
{
	// Store port name
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	// Configure for INS mode if selected
	if (_param_xsens_mode.get() == 1) {
		int32_t v = 0;

		// Disable EKF2
		v = 0;
		param_set(param_find("EKF2_EN"), &v);

		// Configure sensor selection
		v = 0;
		param_set(param_find("SENS_IMU_MODE"), &v);
		param_set(param_find("SENS_MAG_MODE"), &v);
	}

	// Setup device ID with generic Xsens type
	device::Device::DeviceId device_id{};
	device_id.devid_s.devtype = DRV_INS_DEVTYPE_XSENS_GENERIC;
	device_id.devid_s.bus_type = device::Device::DeviceBusType_SERIAL;

	_px4_accel.set_device_id(device_id.devid);
	_px4_gyro.set_device_id(device_id.devid);
	_px4_mag.set_device_id(device_id.devid);

	// Advertise publications
	_attitude_pub.advertise();
	_local_position_pub.advertise();
	_global_position_pub.advertise();
}

Xsens::~Xsens()
{
	closeSerialPort();

	// Free performance counters
	perf_free(_sample_perf);
	perf_free(_comms_errors);
	perf_free(_timeout_errors);
	perf_free(_accel_pub_interval_perf);
	perf_free(_gyro_pub_interval_perf);
	perf_free(_mag_pub_interval_perf);
	perf_free(_gnss_pub_interval_perf);
	perf_free(_baro_pub_interval_perf);
	perf_free(_attitude_pub_interval_perf);
	perf_free(_local_position_pub_interval_perf);
	perf_free(_global_position_pub_interval_perf);
}

int Xsens::openSerialPort()
{
	// Open serial port with NuttX-compatible flags
	_serial_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		PX4_ERR("Failed to open port %s (errno: %d)", _port, errno);
		return -1;
	}

	// Configure serial port
	struct termios portSettings;
	memset(&portSettings, 0, sizeof(portSettings));

	// Set baud rate using cfsetspeed (NuttX compatible)
	speed_t baud_rate = B115200;
	switch (_param_xsens_baud.get()) {
		case 9600:   baud_rate = B9600;   break;
		case 19200:  baud_rate = B19200;  break;
		case 38400:  baud_rate = B38400;  break;
		case 57600:  baud_rate = B57600;  break;
		case 115200: baud_rate = B115200; break;
		case 230400: baud_rate = B230400; break;
		case 460800: baud_rate = B460800; break;
		case 921600: baud_rate = B921600; break;
		default:     baud_rate = B115200; break;
	}

	// Use cfsetspeed for NuttX compatibility
	if (cfsetspeed(&portSettings, baud_rate) != 0) {
		PX4_ERR("Failed to set baud rate");
		closeSerialPort();
		return -1;
	}

	// Configure port settings
	// Set baudrate, 8n1, no modem control, enable receiving characters
	portSettings.c_cflag |= CS8 | CLOCAL | CREAD;

	portSettings.c_iflag = IGNPAR;		// Ignore bytes with parity errors
	portSettings.c_oflag = 0;			// Enable raw data output
	portSettings.c_lflag = 0;			// Raw input
	portSettings.c_cc[VTIME] = 0;		// Do not use inter-character timer
	portSettings.c_cc[VMIN] = 0;		// Non-blocking reads

	// Clear the COM port buffers
	if (tcflush(_serial_fd, TCIFLUSH) != 0) {
		PX4_WARN("Failed to flush serial port");
	}

	if (tcsetattr(_serial_fd, TCSANOW, &portSettings) != 0) {
		PX4_ERR("Failed to set port attributes (errno: %d)", errno);
		closeSerialPort();
		return -1;
	}

	return 0;
}

void Xsens::closeSerialPort()
{
	if (_serial_fd >= 0) {
		::close(_serial_fd);
		_serial_fd = -1;
	}
}

bool Xsens::writeMessage(const uint8_t *message, size_t length)
{
    if (_serial_fd < 0) {
        PX4_ERR("Serial port not open");
        return false;
    }

    ssize_t bytes_written = ::write(_serial_fd, message, length);
    if (bytes_written != (ssize_t)length) {
        PX4_ERR("Failed to write complete message: wrote %d of %zu bytes",
                (int)bytes_written, length);
        perf_count(_comms_errors);
        return false;
    }

    // Force flush the data (important for NuttX)
    fsync(_serial_fd);

    // Small delay to ensure message is sent
    px4_usleep(10000); // 10ms delay

    return true;
}

size_t Xsens::readData(uint8_t *buffer, size_t max_length)
{
    if (_serial_fd < 0) {
        return 0;
    }

    ssize_t bytes_read = ::read(_serial_fd, buffer, max_length);
    if (bytes_read < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            PX4_DEBUG("Read error: %d", errno);
        }
        return 0;
    }

    return static_cast<size_t>(bytes_read);
}

bool Xsens::init()
{
    if (openSerialPort() != 0) {
        return false;
    }

    // Initialize sensor data timestamp
    _last_sensor_data_time = hrt_absolute_time();

    setState(DeviceState::ENTERING_CONFIG_MODE);
    return true;
}

void Xsens::setState(DeviceState new_state)
{
    if (_state != new_state) {
        PX4_INFO("State change: %s -> %s", getStateString(_state), getStateString(new_state));
        _state = new_state;

        // Set different timeouts for different states
        switch (new_state) {
            case DeviceState::RUNNING:
                // Don't set a timeout for RUNNING state, let it run indefinitely
                // The data timeout check will handle reinitialization if needed
                _state_timeout = hrt_absolute_time() + 3600_s; // 1 hour timeout (effectively infinite)
                break;
            case DeviceState::WAITING_FOR_CONFIG_MODE:
            case DeviceState::WAITING_FOR_OUTPUT_CONFIG:
	    case DeviceState::CONFIGURING_ROTLOCAL:
	    case DeviceState::WAITING_FOR_ROTLOCAL_CONFIG:
	    case DeviceState::CONFIGURING_ROTSENSOR:
	    case DeviceState::WAITING_FOR_ROTSENSOR_CONFIG:
            case DeviceState::WAITING_FOR_MEASUREMENT_MODE:
                _state_timeout = hrt_absolute_time() + 2_s; // 2 second timeout for waiting states
                break;
            default:
                _state_timeout = hrt_absolute_time() + 5_s; // 5 second timeout for other states
                break;
        }
    }
}

const char* Xsens::getStateString(DeviceState state) const
{
	switch (state) {
		case DeviceState::ENTERING_CONFIG_MODE: return "ENTERING_CONFIG_MODE";
		case DeviceState::WAITING_FOR_CONFIG_MODE: return "WAITING_FOR_CONFIG_MODE";
		case DeviceState::CONFIGURING_OUTPUT: return "CONFIGURING_OUTPUT";
		case DeviceState::WAITING_FOR_OUTPUT_CONFIG: return "WAITING_FOR_OUTPUT_CONFIG";
		case DeviceState::CONFIGURING_ROTLOCAL: return "CONFIGURING_ROTLOCAL";
		case DeviceState::WAITING_FOR_ROTLOCAL_CONFIG: return "WAITING_FOR_ROTLOCAL_CONFIG";
		case DeviceState::CONFIGURING_ROTSENSOR: return "CONFIGURING_ROTSENSOR";
		case DeviceState::WAITING_FOR_ROTSENSOR_CONFIG: return "WAITING_FOR_ROTSENSOR_CONFIG";
		case DeviceState::ENTERING_MEASUREMENT_MODE: return "ENTERING_MEASUREMENT_MODE";
		case DeviceState::WAITING_FOR_MEASUREMENT_MODE: return "WAITING_FOR_MEASUREMENT_MODE";
		case DeviceState::RUNNING: return "RUNNING";
		case DeviceState::ERROR: return "ERROR";
		default: return "UNKNOWN";
	}
}

bool Xsens::gotoConfigMode()
{
	uint8_t config_msg[5];
	Xbus::createMessage(config_msg, 0xFF, XMID_GotoConfig, 0);
	Xbus::insertChecksum(config_msg);

	return writeMessage(config_msg, 5);
}

bool Xsens::configureOutputSettings()
{
	// Configure output settings to get the data we need
	uint8_t config_msg[64];
	Xbus::createMessage(config_msg, 0xFF, XMID_SetOutputConfig, 0);

	uint8_t *payload = Xbus::getPointerToPayload(config_msg);
	int payload_idx = 0;

	// Configure desired output rate
	uint16_t output_rate = _param_xsens_odr.get();

	// PacketCounter
	payload[payload_idx++] = (XDI::PACKET_COUNTER >> 8) & 0xFF;
	payload[payload_idx++] = XDI::PACKET_COUNTER & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Sample Time Fine
	payload[payload_idx++] = (XDI::SAMPLE_TIME_FINE >> 8) & 0xFF;
	payload[payload_idx++] = XDI::SAMPLE_TIME_FINE & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// UTC Time
	payload[payload_idx++] = (XDI::UTC_TIME >> 8) & 0xFF;
	payload[payload_idx++] = XDI::UTC_TIME & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Temperature
	payload[payload_idx++] = (XDI::TEMPERATURE >> 8) & 0xFF;
	payload[payload_idx++] = XDI::TEMPERATURE & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// QUATERNION
	payload[payload_idx++] = (XDI::QUATERNION >> 8) & 0xFF;
	payload[payload_idx++] = XDI::QUATERNION & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Acceleration
	payload[payload_idx++] = (XDI::ACCELERATION >> 8) & 0xFF;
	payload[payload_idx++] = XDI::ACCELERATION & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Rate of Turn
	payload[payload_idx++] = (XDI::RATE_OF_TURN >> 8) & 0xFF;
	payload[payload_idx++] = XDI::RATE_OF_TURN & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Magnetic Field
	payload[payload_idx++] = (XDI::MAGNETIC_FIELD >> 8) & 0xFF;
	payload[payload_idx++] = XDI::MAGNETIC_FIELD & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Status Word
	payload[payload_idx++] = (XDI::STATUS_WORD >> 8) & 0xFF;
	payload[payload_idx++] = XDI::STATUS_WORD & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Barometric Pressure
	payload[payload_idx++] = (XDI::BAROMETRIC_PRESSURE >> 8) & 0xFF;
	payload[payload_idx++] = XDI::BAROMETRIC_PRESSURE & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// For INS models, add position and velocity (generic configuration)
	// Assume INS capabilities for all devices
	// Latitude/Longitude
	payload[payload_idx++] = (XDI::LAT_LON >> 8) & 0xFF;
	payload[payload_idx++] = XDI::LAT_LON & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Altitude
	payload[payload_idx++] = (XDI::ALTITUDE_ELLIPSOID >> 8) & 0xFF;
	payload[payload_idx++] = XDI::ALTITUDE_ELLIPSOID & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	// Velocity
	payload[payload_idx++] = (XDI::VELOCITY_XYZ >> 8) & 0xFF;
	payload[payload_idx++] = XDI::VELOCITY_XYZ & 0xFF;
	payload[payload_idx++] = (output_rate >> 8) & 0xFF;
	payload[payload_idx++] = output_rate & 0xFF;

	PX4_INFO("Configured generic output for Xsens device");

	// Update payload length
	Xbus::setPayloadLength(config_msg, payload_idx);
	Xbus::insertChecksum(config_msg);

	return writeMessage(config_msg, Xbus::getRawLength(config_msg));
}

bool Xsens::configureAlignmentRotationSetting(int frame)
{
	// Send rotlocal: FA FF EC 11 01 00 00 00 00 3F 35 04 F4 3F 35 04 F4 00 00 00 00 2B
	//send rotsensor: FA FF EC 11 00 00 00 00 00 3F 35 04 F4 3F 35 04 F4 00 00 00 00 2C
	uint8_t config_msg[22];
	Xbus::createMessage(config_msg, 0xFF, XMID_SetAlignmentRotation, 0);

	uint8_t *payload = Xbus::getPointerToPayload(config_msg);
	int payload_idx = 0;

	// First byte: 01 means RotLocal frame.
	payload[payload_idx++] = frame;

	// quaternion w, 0.0
	payload[payload_idx++] = 0x00;
	payload[payload_idx++] = 0x00;
	payload[payload_idx++] = 0x00;
	payload[payload_idx++] = 0x00;
	// quaternion x, 0.7071068
	payload[payload_idx++] = 0x3F;
	payload[payload_idx++] = 0x35;
	payload[payload_idx++] = 0x04;
	payload[payload_idx++] = 0xF4;
	// quaternion y, 0.7071068
	payload[payload_idx++] = 0x3F;
	payload[payload_idx++] = 0x35;
	payload[payload_idx++] = 0x04;
	payload[payload_idx++] = 0xF4;
	// quaternion z, 0.0
	payload[payload_idx++] = 0x00;
	payload[payload_idx++] = 0x00;
	payload[payload_idx++] = 0x00;
	payload[payload_idx++] = 0x00;

	// Update payload length and add checksum
	Xbus::setPayloadLength(config_msg, payload_idx);
	Xbus::insertChecksum(config_msg);

	return writeMessage(config_msg, Xbus::getRawLength(config_msg));
}

bool Xsens::gotoMeasurementMode()
{
	uint8_t measurement_msg[5];
	Xbus::createMessage(measurement_msg, 0xFF, XMID_GotoMeasurement, 0);
	Xbus::insertChecksum(measurement_msg);

	return writeMessage(measurement_msg, 5);
}

void Xsens::updateStateMachine()
{
    hrt_abstime now = hrt_absolute_time();

    // Check for timeout
    if (now > _state_timeout) {
        PX4_WARN("State timeout in %s", getStateString(_state));
        perf_count(_timeout_errors);

        // Retry logic instead of going to error immediately
        switch (_state) {
            case DeviceState::WAITING_FOR_CONFIG_MODE:
                PX4_INFO("Retrying GotoConfig command");
                setState(DeviceState::ENTERING_CONFIG_MODE);
                break;
            case DeviceState::WAITING_FOR_OUTPUT_CONFIG:
                PX4_INFO("Retrying output configuration");
                setState(DeviceState::CONFIGURING_OUTPUT);
                break;
            case DeviceState::WAITING_FOR_ROTLOCAL_CONFIG:
                PX4_INFO("Retrying rotlocal configuration");
                setState(DeviceState::CONFIGURING_ROTLOCAL);
                break;
            case DeviceState::WAITING_FOR_ROTSENSOR_CONFIG:
                PX4_INFO("Retrying rotsensor configuration");
                setState(DeviceState::CONFIGURING_ROTSENSOR);
                break;
            case DeviceState::WAITING_FOR_MEASUREMENT_MODE:
                PX4_INFO("Retrying GotoMeasurement command");
                setState(DeviceState::ENTERING_MEASUREMENT_MODE);
                break;
            case DeviceState::RUNNING:
                // Check if we're actually receiving data before restarting
                if (now - _last_sensor_data_time > 5_s) {  // Increase timeout to 5 seconds
                    PX4_WARN("Sensor data timeout, reinitializing");
                    setState(DeviceState::ENTERING_CONFIG_MODE);
                } else {
                    PX4_DEBUG("State timeout in RUNNING but data is flowing, extending timeout");
                    _state_timeout = now + 5_s; // Extend timeout
                }
                break;
            default:
                setState(DeviceState::ERROR);
                break;
        }
        return;
    }

    switch (_state) {
        case DeviceState::ENTERING_CONFIG_MODE:
            PX4_INFO("Sending GotoConfig command");
            if (gotoConfigMode()) {
                setState(DeviceState::WAITING_FOR_CONFIG_MODE);
            } else {
                PX4_ERR("Failed to send GotoConfig command");
                setState(DeviceState::ERROR);
            }
            break;

        case DeviceState::WAITING_FOR_CONFIG_MODE:
            // Wait for config mode acknowledgment
            break;

        case DeviceState::CONFIGURING_OUTPUT:
            PX4_INFO("Sending output configuration");
            if (configureOutputSettings()) {
                setState(DeviceState::WAITING_FOR_OUTPUT_CONFIG);
            } else {
                PX4_ERR("Failed to send output configuration");
                setState(DeviceState::ERROR);
            }
            break;

        case DeviceState::WAITING_FOR_OUTPUT_CONFIG:
            // Wait for output configuration acknowledgment
            break;

        case DeviceState::CONFIGURING_ROTLOCAL:
            PX4_INFO("Sending rotlocal configuration");
            if (configureAlignmentRotationSetting(0x00)) {
                setState(DeviceState::WAITING_FOR_ROTLOCAL_CONFIG);
            } else {
                PX4_ERR("Failed to send rotlocal configuration");
                setState(DeviceState::ERROR);
            }
            break;

        case DeviceState::WAITING_FOR_ROTLOCAL_CONFIG:
            // Wait for rotlocal configuration acknowledgment
            break;

	case DeviceState::CONFIGURING_ROTSENSOR:
            PX4_INFO("Sending rotsensor configuration");
            if (configureAlignmentRotationSetting(0x01)) {
                setState(DeviceState::WAITING_FOR_ROTSENSOR_CONFIG);
            } else {
                PX4_ERR("Failed to send rotsensor configuration");
                setState(DeviceState::ERROR);
            }
            break;

        case DeviceState::WAITING_FOR_ROTSENSOR_CONFIG:
            // Wait for rotsensor configuration acknowledgment
            break;

        case DeviceState::ENTERING_MEASUREMENT_MODE:
            PX4_INFO("Sending GotoMeasurement command");
            if (gotoMeasurementMode()) {
                setState(DeviceState::WAITING_FOR_MEASUREMENT_MODE);
            } else {
                PX4_ERR("Failed to send GotoMeasurement command");
                setState(DeviceState::ERROR);
            }
            break;

        case DeviceState::WAITING_FOR_MEASUREMENT_MODE:
            // Wait for measurement mode acknowledgment
            break;

        case DeviceState::RUNNING:
            // Normal operation - only check for data timeout if we haven't received data recently
            if (now - _last_sensor_data_time > 10_s) {  // Much longer timeout for running state
                PX4_WARN("Sensor data timeout (10s), reinitializing");
                setState(DeviceState::ENTERING_CONFIG_MODE);
            }
            // Don't set a state timeout in RUNNING state, let it run indefinitely
            break;

        case DeviceState::ERROR:
            // Try to recover after some time
            if (now - _state_timeout > 5_s) {
                PX4_INFO("Attempting recovery from error state");
                setState(DeviceState::ENTERING_CONFIG_MODE);
            }
            break;

        default:
            setState(DeviceState::ERROR);
            break;
    }
}

void Xsens::processReceivedData()
{
    size_t bytes_available = readData(_rx_buffer + _rx_buffer_pos, BUFFER_SIZE - _rx_buffer_pos);

    if (bytes_available == 0) {
        return;
    }

    _rx_buffer_pos += bytes_available;

    // Look for complete messages in the buffer
    size_t search_start = 0;

    while (search_start < _rx_buffer_pos) {
        // Look for preamble (0xFA)
        size_t preamble_pos = search_start;
        while (preamble_pos < _rx_buffer_pos && _rx_buffer[preamble_pos] != 0xFA) {
            preamble_pos++;
        }

        if (preamble_pos >= _rx_buffer_pos) {
            // No preamble found, discard processed data
            _rx_buffer_pos = 0;
            break;
        }

        // Check if we have enough data for a complete message header
        if (preamble_pos + 4 > _rx_buffer_pos) {
            // Not enough data for header, move preamble to start and wait for more
            if (preamble_pos > 0) {
                memmove(_rx_buffer, _rx_buffer + preamble_pos, _rx_buffer_pos - preamble_pos);
                _rx_buffer_pos -= preamble_pos;
            }
            break;
        }

        // Validate message structure
        if (_rx_buffer[preamble_pos + 1] != 0xFF) {
            // Invalid BID, skip this byte
            search_start = preamble_pos + 1;
            continue;
        }

        uint8_t length = _rx_buffer[preamble_pos + 3];
        size_t total_msg_length = 4 + length + 1; // Header + payload + checksum

        if (preamble_pos + total_msg_length > _rx_buffer_pos) {
            // Not enough data for complete message, move to start and wait
            if (preamble_pos > 0) {
                memmove(_rx_buffer, _rx_buffer + preamble_pos, _rx_buffer_pos - preamble_pos);
                _rx_buffer_pos -= preamble_pos;
            }
            break;
        }

        // We have a complete message, verify checksum
        uint8_t calculated_checksum = 0;
        for (size_t i = 1; i < total_msg_length - 1; i++) {
            calculated_checksum += _rx_buffer[preamble_pos + i];
        }
        calculated_checksum = 0x100 - calculated_checksum;

        uint8_t received_checksum = _rx_buffer[preamble_pos + total_msg_length - 1];

        if (calculated_checksum != received_checksum) {
            PX4_DEBUG("Checksum mismatch: calc=0x%02X, recv=0x%02X", calculated_checksum, received_checksum);
            search_start = preamble_pos + 1;
            continue;
        }

        // Valid message found, process it
        handleMessage(_rx_buffer + preamble_pos);

        // Move to next potential message
        search_start = preamble_pos + total_msg_length;
    }

    // Remove processed data from buffer
    if (search_start > 0) {
        if (search_start < _rx_buffer_pos) {
            memmove(_rx_buffer, _rx_buffer + search_start, _rx_buffer_pos - search_start);
            _rx_buffer_pos -= search_start;
        } else {
            _rx_buffer_pos = 0;
        }
    }
}

void Xsens::handleMessage(const uint8_t *message)
{
    uint8_t message_id = message[2];  // Direct access instead of using Xbus::getMessageId
    uint8_t length = message[3];      // Direct access instead of using Xbus::getPayloadLength
    _last_message_time = hrt_absolute_time();

    // Always log received messages for debugging
    PX4_DEBUG("Received message ID: 0x%02X, Length: %d", message_id, length);

    // Normal message handling
    switch (message_id) {
        case XMID_GotoConfigAck:
            PX4_INFO("Received GotoConfigAck");
            if (_state == DeviceState::WAITING_FOR_CONFIG_MODE) {
                setState(DeviceState::CONFIGURING_OUTPUT);
            }
            break;

        case XMID_OutputConfig:
            PX4_INFO("Received SetOutputConfigAck");
            if (_state == DeviceState::WAITING_FOR_OUTPUT_CONFIG) {
                setState(DeviceState::CONFIGURING_ROTLOCAL);
            }
            break;

	case XMID_AlignmentRotationAck:
            PX4_INFO("Received AlignmentRotationAck");
            if (_state == DeviceState::WAITING_FOR_ROTLOCAL_CONFIG) {
                setState(DeviceState::CONFIGURING_ROTSENSOR);
            }
	    else if (_state == DeviceState::WAITING_FOR_ROTSENSOR_CONFIG) {
                setState(DeviceState::ENTERING_MEASUREMENT_MODE);
            }
            break;

        case XMID_GotoMeasurementAck:
            PX4_INFO("Received GotoMeasurementAck");
            if (_state == DeviceState::WAITING_FOR_MEASUREMENT_MODE) {
                setState(DeviceState::RUNNING);
                _configured = true;
                PX4_INFO("Successfully configured Xsens device, now in RUNNING state");
            }
            break;

        case XMID_MtData2:
            PX4_DEBUG("Processing MTData2 message");
            handleMTData2Message(message);
            break;

        case XMID_Error:
            PX4_ERR("Received error message from device");
            if (length > 0) {
                PX4_ERR("Error code: 0x%02X", message[4]);
            }
            handleErrorMessage(message);
            break;

        default:
            PX4_DEBUG("Unhandled message ID: 0x%02X", message_id);
            break;
    }
}

void Xsens::handleMTData2Message(const uint8_t *message)
{
    SensorData sensor_data;
    if (XbusParser::parseMTData2(message, sensor_data)) {
        _last_sensor_data_time = hrt_absolute_time();
        publishSensorData(sensor_data);
        perf_count(_sample_perf);

        // Add debug info
        PX4_DEBUG("MTData2 parsed successfully, updated sensor data timestamp");
    } else {
        PX4_WARN("Failed to parse MTData2 message");
    }
}

void Xsens::handleErrorMessage(const uint8_t *message)
{
	PX4_ERR("Received error message from device");
	perf_count(_comms_errors);
	setState(DeviceState::ERROR);
}

void Xsens::publishSensorData(const SensorData &data)
{
	// Update sensor temperatures first if available
	if (data.hasTemperature) {
		updateSensorTemperatures(data);
	}

	// Publish raw sensor data using PX4 sensor objects
	if (data.hasAcceleration) {
		publishAcceleration(data);
	}

	if (data.hasRateOfTurn) {
		publishRateOfTurn(data);
	}

	if (data.hasMagneticField) {
		publishMagneticField(data);
	}

	// Publish attitude using quaternion directly
	if (data.hasQuaternion) {
		publishAttitude(data);
	}

	// Publish barometric pressure
	if (data.hasBarometricPressure) {
		publishBarometer(data);
	}

	// Publish position data (for INS models)
	if (data.hasLatLon && data.hasAltitudeEllipsoid) {
		publishGlobalPosition(data);
		publishLocalPosition(data);
	}

	// Publish velocity data as GNSS with UTC time
	if (data.hasLatLon && data.hasVelocityXYZ) {
		publishGNSS(data);
	}
}

void Xsens::publishAttitude(const SensorData &data)
{
	// Use quaternion directly from Xsens data
	vehicle_attitude_s attitude{};
	attitude.timestamp_sample = hrt_absolute_time();

	// Xsens quaternion format: q0=w, q1=x, q2=y, q3=z
	attitude.q[0] = data.quaternion.q0;  // w
	attitude.q[1] = data.quaternion.q1;  // x
	attitude.q[2] = data.quaternion.q2;  // y
	attitude.q[3] = data.quaternion.q3;  // z

	attitude.timestamp = hrt_absolute_time();

	_attitude_pub.publish(attitude);
	perf_count(_attitude_pub_interval_perf);
}

void Xsens::publishLocalPosition(const SensorData &data)
{
	// Initialize position reference if needed
	if (!_pos_ref.isInitialized()) {
		_pos_ref.initReference(data.latLon.latitude, data.latLon.longitude, hrt_absolute_time());
		_gps_alt_ref = static_cast<float>(data.altitudeEllipsoid);
	}

	Vector2f pos_ned = _pos_ref.project(data.latLon.latitude, data.latLon.longitude);

	vehicle_local_position_s local_pos{};
	local_pos.timestamp_sample = hrt_absolute_time();
	local_pos.xy_valid = true;
	local_pos.z_valid = true;
	local_pos.v_xy_valid = data.hasVelocityXYZ;
	local_pos.v_z_valid = data.hasVelocityXYZ;

	local_pos.x = pos_ned(0);
	local_pos.y = pos_ned(1);
	local_pos.z = -(static_cast<float>(data.altitudeEllipsoid) - _gps_alt_ref);

	if (data.hasVelocityXYZ) {
		local_pos.vx = static_cast<float>(data.velocityXYZ.velX);
		local_pos.vy = static_cast<float>(data.velocityXYZ.velY);
		local_pos.vz = static_cast<float>(data.velocityXYZ.velZ);
	}

	if (data.hasQuaternion) {
		// Convert quaternion to yaw angle
		Quatf q(data.quaternion.q0, data.quaternion.q1, data.quaternion.q2, data.quaternion.q3);
		Eulerf euler(q);
		local_pos.heading = euler.psi();
		local_pos.heading_good_for_control = true;
	}

	if (_pos_ref.isInitialized()) {
		local_pos.xy_global = true;
		local_pos.z_global = true;
		local_pos.ref_timestamp = _pos_ref.getProjectionReferenceTimestamp();
		local_pos.ref_lat = _pos_ref.getProjectionReferenceLat();
		local_pos.ref_lon = _pos_ref.getProjectionReferenceLon();
		local_pos.ref_alt = _gps_alt_ref;
	}

	local_pos.timestamp = hrt_absolute_time();
	_local_position_pub.publish(local_pos);
	perf_count(_local_position_pub_interval_perf);
}

void Xsens::publishGlobalPosition(const SensorData &data)
{
	vehicle_global_position_s global_pos{};
	global_pos.timestamp_sample = hrt_absolute_time();
	global_pos.lat = data.latLon.latitude;
	global_pos.lon = data.latLon.longitude;
	global_pos.alt_ellipsoid = data.altitudeEllipsoid;
	global_pos.timestamp = hrt_absolute_time();

	_global_position_pub.publish(global_pos);
	perf_count(_global_position_pub_interval_perf);
}

void Xsens::publishGNSS(const SensorData &data)
{
	sensor_gps_s gps{};
	gps.timestamp_sample = hrt_absolute_time();
	gps.device_id = _px4_accel.get_device_id(); // Use accelerometer device ID as reference

	// Determine fix type from status word
	if (data.hasStatusWord) {
		gps.fix_type = determineFixType(data.statusWord);
	} else {
		gps.fix_type = 1; // Default to 1: GPS fix following the NMEA-0183 protocol.
	}


	gps.latitude_deg = data.latLon.latitude;
	gps.longitude_deg = data.latLon.longitude;
	gps.altitude_ellipsoid_m = static_cast<float>(data.altitudeEllipsoid);

	if (data.hasVelocityXYZ) {
		gps.vel_n_m_s = static_cast<float>(data.velocityXYZ.velX);
		gps.vel_e_m_s = static_cast<float>(data.velocityXYZ.velY);
		gps.vel_d_m_s = static_cast<float>(data.velocityXYZ.velZ);
		gps.vel_m_s = sqrtf(sq(gps.vel_n_m_s) + sq(gps.vel_e_m_s));
		gps.vel_ned_valid = true;
	}

	// Set UTC time from Xsens INS data
	if (data.hasUtcTime) {
		gps.time_utc_usec = convertUtcTimeToUnixMicroseconds(data.utcTime);
	} else {
		gps.time_utc_usec = 0;
	}

	// Default values for accuracy
	gps.eph = 1.0f;
	gps.epv = 1.5f;
	gps.hdop = 0.8f;
	gps.vdop = 1.0f;

	gps.timestamp = hrt_absolute_time();
	_sensor_gps_pub.publish(gps);
	perf_count(_gnss_pub_interval_perf);
}

void Xsens::publishBarometer(const SensorData &data)
{
	sensor_baro_s baro{};
	baro.timestamp_sample = hrt_absolute_time();
	baro.device_id = _px4_accel.get_device_id(); // Use accelerometer device ID as reference

	// Convert pressure from Pa to hPa (millibar)
	baro.pressure = static_cast<float>(data.barometricPressure.pressure) / 100.0f;

	// Use temperature from INS if available, otherwise set to NaN
	if (data.hasTemperature) {
		baro.temperature = data.temperature.temperature;
	} else {
		baro.temperature = NAN;
	}

	// Error estimate (default value)
	baro.error_count = 0;

	baro.timestamp = hrt_absolute_time();

	_sensor_baro_pub.publish(baro);
	perf_count(_baro_pub_interval_perf);
}



void Xsens::publishAcceleration(const SensorData &data)
{
    hrt_abstime timestamp = hrt_absolute_time();

    // Xsens acceleration data is already in m/s² (perfect for PX4)
    _px4_accel.update(timestamp,
                      data.acceleration.accX,
                      data.acceleration.accY,
                      data.acceleration.accZ);

    perf_count(_accel_pub_interval_perf);
}

void Xsens::publishRateOfTurn(const SensorData &data)
{
    hrt_abstime timestamp = hrt_absolute_time();

    // Xsens rate of turn data is already in rad/s (perfect for PX4)
    _px4_gyro.update(timestamp,
                     data.rateOfTurn.gyrX,
                     data.rateOfTurn.gyrY,
                     data.rateOfTurn.gyrZ);

    perf_count(_gyro_pub_interval_perf);
}

void Xsens::publishMagneticField(const SensorData &data)
{
    hrt_abstime timestamp = hrt_absolute_time();

    // Conversion factor from a.u. to Gauss (0.49 based on calibration location in the Netherlands)
    constexpr float A_U_TO_GAUSS = 0.49f;  // Adjust as needed

    // Convert magnetic field readings from arbitrary units to Gauss
    const float mag_x_gauss = data.magneticField.magX * A_U_TO_GAUSS;
    const float mag_y_gauss = data.magneticField.magY * A_U_TO_GAUSS;
    const float mag_z_gauss = data.magneticField.magZ * A_U_TO_GAUSS;

    //float32[3] magnetometer_ga  # Magnetic field in the FRD body frame XYZ-axis in Gauss
    _px4_mag.update(timestamp, mag_x_gauss, mag_y_gauss, mag_z_gauss);

    perf_count(_mag_pub_interval_perf);
}

void Xsens::updateSensorTemperatures(const SensorData &data) {
    _current_temperature = data.temperature.temperature;

    // Set temperature for all PX4 sensor objects
    _px4_accel.set_temperature(_current_temperature);
    _px4_gyro.set_temperature(_current_temperature);
    _px4_mag.set_temperature(_current_temperature);
}

uint64_t Xsens::convertUtcTimeToMicroseconds(const UtcTime &utcTime)
{
	// Convert nanoseconds to microseconds
	return static_cast<uint64_t>(utcTime.nanoseconds) / 1000;
}

uint64_t Xsens::convertUtcTimeToUnixMicroseconds(const UtcTime &utcTime)
{
#ifndef NO_MKTIME
	// Convert to unix timestamp using standard library (same approach as UBX GPS driver)
	tm timeinfo {};
	timeinfo.tm_year = utcTime.year - 1900;      // tm_year is years since 1900
	timeinfo.tm_mon = utcTime.month - 1;         // tm_mon is 0-11
	timeinfo.tm_mday = utcTime.day;              // tm_mday is 1-31
	timeinfo.tm_hour = utcTime.hour;             // tm_hour is 0-23
	timeinfo.tm_min = utcTime.minute;            // tm_min is 0-59
	timeinfo.tm_sec = utcTime.second;            // tm_sec is 0-59
	timeinfo.tm_isdst = 0;                       // Not daylight saving time

	time_t epoch = mktime(&timeinfo);

	// GPS epoch constant (from GPS drivers)
	constexpr time_t GPS_EPOCH_SECS = 315964800; // GPS epoch: 1980-01-06 00:00:00 UTC

	// Only set the time if it makes sense
	if (epoch > GPS_EPOCH_SECS) {
		uint64_t time_utc_usec = static_cast<uint64_t>(epoch) * 1000000ULL;
		time_utc_usec += utcTime.nanoseconds / 1000; // Convert nanoseconds to microseconds
		return time_utc_usec;
	} else {
		return 0;
	}
#else
	return 0;
#endif
}

uint8_t Xsens::determineFixType(uint32_t status)
{
	// Check RTK status first (from StatusWord bits 27-28)
	uint8_t rtk_status = (status >> 27) & 0x3;
	if (rtk_status == 2) {
		return 0x04; // 4 = RTK fixed
	} else if (rtk_status == 1) {
		return 0x05; // 5 = RTK float
	}

	// If no RTK, check GNSS fix bit from StatusWord
	bool gnssFix = status & (1 << 2);

	// If GNSS fix, return 1
	if (gnssFix) {
		return 0x01; // 1 = Autonomous GNSS fix
	}

	return 0x00; // Default to no fix
}

void Xsens::Run()
{
    if (should_exit()) {
        closeSerialPort();
        exit_and_cleanup();
        return;
    }

    // Parameter updates
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    if (!_initialized) {
        if (init()) {
            _initialized = true;
        } else {
            ScheduleDelayed(1_s);
            return;
        }
    }

    // Process received data
    processReceivedData();

    // Normal state machine
    updateStateMachine();

    // Publish sensor selection in INS mode
    if (_param_xsens_mode.get() == 1 && _configured) {
        if ((_px4_accel.get_device_id() != 0) && (_px4_gyro.get_device_id() != 0)) {
            sensor_selection_s sensor_selection{};
            sensor_selection.accel_device_id = _px4_accel.get_device_id();
            sensor_selection.gyro_device_id = _px4_gyro.get_device_id();
            sensor_selection.timestamp = hrt_absolute_time();
            _sensor_selection_pub.publish(sensor_selection);
        }
    }

    // Schedule next run
    ScheduleDelayed(10_ms);
}

int Xsens::print_status()
{
	printf("Using port '%s'\n", _port);
	printf("State: %s\n", getStateString(_state));
	printf("Initialized: %s\n", _initialized ? "yes" : "no");
	printf("Configured: %s\n", _configured ? "yes" : "no");

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);
	perf_print_counter(_timeout_errors);

	return 0;
}

int Xsens::task_spawn(int argc, char *argv[])
{
	bool error_flag = false;

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	const char *device_name = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
			case 'd':
				device_name = myoptarg;
				break;

			case '?':
				error_flag = true;
				break;

			default:
				PX4_WARN("unrecognized flag");
				error_flag = true;
				break;
		}
	}

	if (error_flag) {
		return -1;
	}

	if (device_name && (access(device_name, R_OK | W_OK) == 0)) {
		Xsens *instance = new Xsens(device_name);

		if (instance == nullptr) {
			PX4_ERR("alloc failed");
			return PX4_ERROR;
		}

		_object.store(instance);
		_task_id = task_id_is_work_queue;

		instance->ScheduleNow();

		return PX4_OK;

	} else {
		if (device_name) {
			PX4_ERR("invalid device (-d) %s", device_name);
		} else {
			PX4_INFO("valid device required");
		}
	}

	return PX4_ERROR;
}

int Xsens::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int Xsens::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description

Serial bus driver for Xsens INS devices (MTi-680G, MTi-670G, MTi-G-710, MTi-7, MTi-8).

Most boards are configured to enable/start the driver on a specified UART using the SENS_XSENS_CFG parameter.

Setup/usage information: https://docs.px4.io/main/en/sensor/xsens.html

### Examples

Attempt to start driver on a specified serial device.
$ xsens start -d /dev/ttyS1
Stop driver
$ xsens stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("xsens", "driver");
    PRINT_MODULE_USAGE_SUBCATEGORY("ins");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, nullptr, "Serial device", false);
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Driver status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop driver");

    return PX4_OK;
}

extern "C" __EXPORT int xsens_main(int argc, char *argv[])
{
	return Xsens::main(argc, argv);
}
