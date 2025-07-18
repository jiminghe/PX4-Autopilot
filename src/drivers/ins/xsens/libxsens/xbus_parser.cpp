#include "xbus_parser.h"
#include <cstring>
#include <cstdio>  // For snprintf

uint8_t XbusParser::readUint8(const uint8_t* data, int& index) {
    return data[index++];
}

uint16_t XbusParser::readUint16(const uint8_t* data, int& index) {
    uint16_t result = 0;
    result |= data[index++] << 8;
    result |= data[index++] << 0;
    return result;
}

uint32_t XbusParser::readUint32(const uint8_t* data, int& index) {
    uint32_t result = 0;
    result |= data[index++] << 24;
    result |= data[index++] << 16;
    result |= data[index++] << 8;
    result |= data[index++] << 0;
    return result;
}

float XbusParser::readFloat(const uint8_t* data, int& index) {
    uint32_t temp = readUint32(data, index);
    float result;
    memcpy(&result, &temp, 4);
    return result;
}

double XbusParser::readFP1632(const uint8_t* data, int& index) {
    // FP16.32 format: 32-bit fractional part followed by 16-bit integer part (6 bytes total)
    // Formula: i = round(realNumber * 2^32)
    // Range: [-32768.0 .. 32767.9999999998]
    // Transmission order: {fractional[31:24], fractional[23:16], fractional[15:8], fractional[7:0], integer[15:8], integer[7:0]}

    // Read 32-bit fractional part first (big-endian)
    uint32_t fractionalPart = readUint32(data, index);

    // Read 16-bit integer part (big-endian)
    uint16_t integerPartUnsigned = readUint16(data, index);
    // Convert to signed 16-bit value
    int16_t integerPart = static_cast<int16_t>(integerPartUnsigned);

    // Combine into 48-bit signed value and convert to double
    // The 48-bit value is: (integerPart << 32) | fractionalPart
    int64_t fixedPointValue = (static_cast<int64_t>(integerPart) << 32) | (static_cast<int64_t>(fractionalPart) & 0xffffffff);
    // Convert from fixed point to double: divide by 2^32
    double result = static_cast<double>(fixedPointValue) / 4294967296.0; // 2^32

    return result;
}

bool XbusParser::messageToString(const uint8_t* xbusData, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    output[0] = '\0'; // Initialize buffer

    if (!Xbus::checkPreamble(xbusData)) {
        strncpy(output, "Invalid xbus message", maxLen - 1);
        output[maxLen - 1] = '\0';
        return false;
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    int index = 4;
    char tempBuffer[64];

    switch (messageId) {
        case XMID_Wakeup:
            strncpy(output, "XMID_Wakeup", maxLen - 1);
            break;

        case XMID_DeviceId: {
            uint32_t deviceId = readUint32(xbusData, index);
            snprintf(output, maxLen, "XMID_DeviceId: 0x%08X", static_cast<unsigned int>(deviceId));
            break;
        }

        case XMID_GotoConfigAck:
            strncpy(output, "XMID_GotoConfigAck", maxLen - 1);
            break;

        case XMID_GotoMeasurementAck:
            strncpy(output, "XMID_GotoMeasurementAck", maxLen - 1);
            break;

        case XMID_MtData2: {
            // Use enhanced parsing for MTData2
            SensorData sensorData;
            if (parseMTData2(xbusData, sensorData)) {
                char sensorDataStr[MAX_SENSOR_DATA_STRING_LEN];
                if (formatSensorData(sensorData, sensorDataStr, sizeof(sensorDataStr))) {
                    snprintf(output, maxLen, "XMID_MtData2: %s", sensorDataStr);
                } else {
                    strncpy(output, "XMID_MtData2: Format error", maxLen - 1);
                }
            } else {
                strncpy(output, "XMID_MtData2: Failed to parse", maxLen - 1);
            }
            break;
        }

        case XMID_FirmwareRevision: {
            uint8_t major = readUint8(xbusData, index);
            uint8_t minor = readUint8(xbusData, index);
            uint8_t patch = readUint8(xbusData, index);
            snprintf(output, maxLen, "Firmware revision: %d.%d.%d",
                    static_cast<int>(major), static_cast<int>(minor), static_cast<int>(patch));
            break;
        }

        case XMID_GotoBootLoaderAck:
            strncpy(output, "XMID_GotoBootLoaderAck", maxLen - 1);
            break;

        case XMID_FirmwareUpdate:
            strncpy(output, "XMID_FirmwareUpdate", maxLen - 1);
            break;

        case XMID_ResetAck:
            strncpy(output, "XMID_ResetAck", maxLen - 1);
            break;

        default:
            snprintf(output, maxLen, "Unhandled xbus message: MessageId = 0x%02X", static_cast<int>(messageId));
            break;
    }

    output[maxLen - 1] = '\0'; // Ensure null termination
    return true;
}

bool XbusParser::parseMTData2(const uint8_t* xbusData, SensorData& sensorData) {
    if (!Xbus::checkPreamble(xbusData)) {
        return false;
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    if (messageId != XMID_MtData2) {
        return false;
    }

    // Reset the sensor data structure
    sensorData = SensorData();

    int payloadLength = Xbus::getPayloadLength(xbusData);
    const uint8_t* payload = Xbus::getConstPointerToPayload(xbusData);

    int index = 0;

    // Parse all data items in the payload
    while (index < payloadLength) {
        if (index + 3 > payloadLength) {
            break; // Not enough bytes for XDI and size
        }

        uint16_t xdi = readUint16(payload, index);
        uint8_t size = readUint8(payload, index);

        if (index + size > payloadLength) {
            break; // Not enough bytes for the data
        }

        switch (xdi) {
            case XDI::PACKET_COUNTER:
                if (size == 2) {
                    sensorData.packetCounter = readUint16(payload, index);
                    sensorData.hasPacketCounter = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::SAMPLE_TIME_FINE:
                if (size == 4) {
                    sensorData.sampleTimeFine = readUint32(payload, index);
                    sensorData.hasSampleTimeFine = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::EULER_ANGLES:
                if (size == 12) {
                    sensorData.eulerAngles.roll = readFloat(payload, index);
                    sensorData.eulerAngles.pitch = readFloat(payload, index);
                    sensorData.eulerAngles.yaw = readFloat(payload, index);
                    sensorData.hasEulerAngles = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::STATUS_WORD:
                if (size == 4) {
                    sensorData.statusWord = readUint32(payload, index);
                    sensorData.hasStatusWord = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::LAT_LON:
                if (size == 12) { // 6 bytes each for lat and lon
                    sensorData.latLon.latitude = readFP1632(payload, index);
                    sensorData.latLon.longitude = readFP1632(payload, index);
                    sensorData.hasLatLon = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::ALTITUDE_ELLIPSOID:
                if (size == 6) { // FP1632 format
                    sensorData.altitudeEllipsoid = readFP1632(payload, index);
                    sensorData.hasAltitudeEllipsoid = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::VELOCITY_XYZ:
                if (size == 18) { // 6 bytes each for X, Y, Z
                    sensorData.velocityXYZ.velX = readFP1632(payload, index);
                    sensorData.velocityXYZ.velY = readFP1632(payload, index);
                    sensorData.velocityXYZ.velZ = readFP1632(payload, index);
                    sensorData.hasVelocityXYZ = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::UTC_TIME:
                if (size == 12) { // 4 bytes ns + 2 bytes year + 1 byte each for month, day, hour, minute, second, flags
                    sensorData.utcTime.nanoseconds = readUint32(payload, index);
                    sensorData.utcTime.year = readUint16(payload, index);
                    sensorData.utcTime.month = readUint8(payload, index);
                    sensorData.utcTime.day = readUint8(payload, index);
                    sensorData.utcTime.hour = readUint8(payload, index);
                    sensorData.utcTime.minute = readUint8(payload, index);
                    sensorData.utcTime.second = readUint8(payload, index);
                    sensorData.utcTime.flags = readUint8(payload, index);
                    sensorData.hasUtcTime = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::QUATERNION:
                if (size == 16) { // 4 bytes each for q0, q1, q2, q3
                    sensorData.quaternion.q0 = readFloat(payload, index);
                    sensorData.quaternion.q1 = readFloat(payload, index);
                    sensorData.quaternion.q2 = readFloat(payload, index);
                    sensorData.quaternion.q3 = readFloat(payload, index);
                    sensorData.hasQuaternion = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::BAROMETRIC_PRESSURE:
                if (size == 4) { // 4 bytes for pressure
                    sensorData.barometricPressure.pressure = readUint32(payload, index);
                    sensorData.hasBarometricPressure = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::ACCELERATION:
                if (size == 12) { // 4 bytes each for X, Y, Z
                    sensorData.acceleration.accX = readFloat(payload, index);
                    sensorData.acceleration.accY = readFloat(payload, index);
                    sensorData.acceleration.accZ = readFloat(payload, index);
                    sensorData.hasAcceleration = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::RATE_OF_TURN:
                if (size == 12) { // 4 bytes each for X, Y, Z
                    sensorData.rateOfTurn.gyrX = readFloat(payload, index);
                    sensorData.rateOfTurn.gyrY = readFloat(payload, index);
                    sensorData.rateOfTurn.gyrZ = readFloat(payload, index);
                    sensorData.hasRateOfTurn = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::MAGNETIC_FIELD:
                if (size == 12) { // 4 bytes each for X, Y, Z
                    sensorData.magneticField.magX = readFloat(payload, index);
                    sensorData.magneticField.magY = readFloat(payload, index);
                    sensorData.magneticField.magZ = readFloat(payload, index);
                    sensorData.hasMagneticField = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            case XDI::TEMPERATURE:
                if (size == 4) { // 4 bytes for temperature
                    sensorData.temperature.temperature = readFloat(payload, index);
                    sensorData.hasTemperature = true;
                } else {
                    index += size; // Skip unknown size
                }
                break;

            default:
                // Unknown XDI, skip it
                index += size;
                break;
        }
    }

    return true;
}

bool XbusParser::formatSensorData(const SensorData& data, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    output[0] = '\0'; // Initialize buffer
    char tempBuffer[128];
    bool hasData = false;

    if (data.hasPacketCounter) {
        snprintf(tempBuffer, sizeof(tempBuffer), "PC=%u", static_cast<unsigned int>(data.packetCounter));
        if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        hasData = true;
    }

    if (data.hasSampleTimeFine) {
        snprintf(tempBuffer, sizeof(tempBuffer), "%sSTF=%u", hasData ? ", " : "", static_cast<unsigned int>(data.sampleTimeFine));
        if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        hasData = true;
    }

    if (data.hasUtcTime) {
        char timeStr[MAX_TIMESTAMP_STRING_LEN];
        if (formatUtcTime(data.utcTime, timeStr, sizeof(timeStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sUTC=%s", hasData ? ", " : "", timeStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasEulerAngles) {
        snprintf(tempBuffer, sizeof(tempBuffer), "%sEuler(R=%.2f°, P=%.2f°, Y=%.2f°)",
                hasData ? ", " : "",
                static_cast<double>(data.eulerAngles.roll),
                static_cast<double>(data.eulerAngles.pitch),
                static_cast<double>(data.eulerAngles.yaw));
        if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        hasData = true;
    }

    if (data.hasQuaternion) {
        char quatStr[64];
        if (formatQuaternion(data.quaternion, quatStr, sizeof(quatStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sQuat=%s", hasData ? ", " : "", quatStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasAcceleration) {
        char accStr[64];
        if (formatAcceleration(data.acceleration, accStr, sizeof(accStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sAcc=%s", hasData ? ", " : "", accStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasRateOfTurn) {
        char rotStr[64];
        if (formatRateOfTurn(data.rateOfTurn, rotStr, sizeof(rotStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sRoT=%s", hasData ? ", " : "", rotStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasMagneticField) {
        char magStr[64];
        if (formatMagneticField(data.magneticField, magStr, sizeof(magStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sMag=%s", hasData ? ", " : "", magStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasTemperature) {
        char tempStr[32];
        if (formatTemperature(data.temperature, tempStr, sizeof(tempStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sTemp=%s", hasData ? ", " : "", tempStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasLatLon) {
        snprintf(tempBuffer, sizeof(tempBuffer), "%sLatLon(%.8f, %.8f)",
                hasData ? ", " : "", data.latLon.latitude, data.latLon.longitude);
        if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        hasData = true;
    }

    if (data.hasAltitudeEllipsoid) {
        snprintf(tempBuffer, sizeof(tempBuffer), "%sAlt=%.3fm",
                hasData ? ", " : "", data.altitudeEllipsoid);
        if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        hasData = true;
    }

    if (data.hasVelocityXYZ) {
        snprintf(tempBuffer, sizeof(tempBuffer), "%sVel(%.4f, %.4f, %.4f)m/s",
                hasData ? ", " : "",
                data.velocityXYZ.velX, data.velocityXYZ.velY, data.velocityXYZ.velZ);
        if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        hasData = true;
    }

    if (data.hasBarometricPressure) {
        char baroStr[32];
        if (formatBarometricPressure(data.barometricPressure, baroStr, sizeof(baroStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sBaro=%s", hasData ? ", " : "", baroStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
            hasData = true;
        }
    }

    if (data.hasStatusWord) {
        char statusStr[64];
        if (formatStatusWord(data.statusWord, statusStr, sizeof(statusStr))) {
            snprintf(tempBuffer, sizeof(tempBuffer), "%sStatus=%s", hasData ? ", " : "", statusStr);
            if (!appendToBuffer(output, maxLen, tempBuffer)) return false;
        }
    }

    return true;
}

const char* XbusParser::getXDIName(uint16_t xdi) {
    switch (xdi) {
        case XDI::PACKET_COUNTER: return "PacketCounter";
        case XDI::SAMPLE_TIME_FINE: return "SampleTimeFine";
        case XDI::EULER_ANGLES: return "EulerAngles";
        case XDI::STATUS_WORD: return "StatusWord";
        case XDI::LAT_LON: return "LatLon";
        case XDI::ALTITUDE_ELLIPSOID: return "AltitudeEllipsoid";
        case XDI::VELOCITY_XYZ: return "VelocityXYZ";
        case XDI::QUATERNION: return "Quaternion";
        case XDI::ACCELERATION: return "Acceleration";
        case XDI::RATE_OF_TURN: return "RateOfTurn";
        case XDI::MAGNETIC_FIELD: return "MagneticField";
        case XDI::UTC_TIME: return "UtcTime";
        case XDI::BAROMETRIC_PRESSURE: return "BarometricPressure";
        case XDI::TEMPERATURE: return "Temperature";
        default: return "Unknown";
    }
}

bool XbusParser::formatStatusWord(uint32_t statusWord, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "0x%08X", static_cast<unsigned int>(statusWord));

    // Add some basic status interpretation
    char flags[64] = "";
    if (statusWord & 0x0001) strcat(flags, " [SelfTest]");
    if (statusWord & 0x0002) strcat(flags, " [FilterValid]");
    if (statusWord & 0x0004) strcat(flags, " [GNSSFix]");

    if (strlen(flags) > 0 && strlen(output) + strlen(flags) < maxLen - 1) {
        strcat(output, flags);
    }

    return true;
}

bool XbusParser::formatUtcTime(const UtcTime& utcTime, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "%04u-%02u-%02u %02u:%02u:%02u.%09u",
            static_cast<unsigned int>(utcTime.year),
            static_cast<unsigned int>(utcTime.month),
            static_cast<unsigned int>(utcTime.day),
            static_cast<unsigned int>(utcTime.hour),
            static_cast<unsigned int>(utcTime.minute),
            static_cast<unsigned int>(utcTime.second),
            static_cast<unsigned int>(utcTime.nanoseconds));

    if (utcTime.flags != 0 && strlen(output) < maxLen - 10) {
        char flagStr[10];
        snprintf(flagStr, sizeof(flagStr), " [F:%02X]", static_cast<unsigned int>(utcTime.flags));
        strcat(output, flagStr);
    }

    return true;
}

bool XbusParser::formatQuaternion(const Quaternion& quaternion, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "(%.6f, %.6f, %.6f, %.6f)",
            static_cast<double>(quaternion.q0), static_cast<double>(quaternion.q1),
            static_cast<double>(quaternion.q2), static_cast<double>(quaternion.q3));

    return true;
}

bool XbusParser::formatBarometricPressure(const BarometricPressure& pressure, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "%.2f hPa", static_cast<double>(pressure.pressure) / 100.0);
    return true;
}

bool XbusParser::formatAcceleration(const AccelerationXYZ& acceleration, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "(%.6f, %.6f, %.6f)m/s²",
            static_cast<double>(acceleration.accX),
            static_cast<double>(acceleration.accY),
            static_cast<double>(acceleration.accZ));

    return true;
}

bool XbusParser::formatRateOfTurn(const RateOfTurnXYZ& rateOfTurn, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "(%.6f, %.6f, %.6f)rad/s",
            static_cast<double>(rateOfTurn.gyrX),
            static_cast<double>(rateOfTurn.gyrY),
            static_cast<double>(rateOfTurn.gyrZ));

    return true;
}

bool XbusParser::formatMagneticField(const MagneticFieldXYZ& magneticField, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "(%.6f, %.6f, %.6f)a.u.",
            static_cast<double>(magneticField.magX),
            static_cast<double>(magneticField.magY),
            static_cast<double>(magneticField.magZ));

    return true;
}

bool XbusParser::formatTemperature(const Temperature& temperature, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    snprintf(output, maxLen, "%.6f°C", static_cast<double>(temperature.temperature));

    return true;
}

bool XbusParser::parseEulerAngles(const uint8_t* xbusData, EulerAngles& angles) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasEulerAngles) {
        angles = sensorData.eulerAngles;
        return true;
    }
    return false;
}

bool XbusParser::parseQuaternion(const uint8_t* xbusData, Quaternion& quaternion) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasQuaternion) {
        quaternion = sensorData.quaternion;
        return true;
    }
    return false;
}

bool XbusParser::parseUtcTime(const uint8_t* xbusData, UtcTime& utcTime) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasUtcTime) {
        utcTime = sensorData.utcTime;
        return true;
    }
    return false;
}

bool XbusParser::parseBarometricPressure(const uint8_t* xbusData, BarometricPressure& pressure) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasBarometricPressure) {
        pressure = sensorData.barometricPressure;
        return true;
    }
    return false;
}

bool XbusParser::parseAcceleration(const uint8_t* xbusData, AccelerationXYZ& acceleration) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasAcceleration) {
        acceleration = sensorData.acceleration;
        return true;
    }
    return false;
}

bool XbusParser::parseRateOfTurn(const uint8_t* xbusData, RateOfTurnXYZ& rateOfTurn) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasRateOfTurn) {
        rateOfTurn = sensorData.rateOfTurn;
        return true;
    }
    return false;
}

bool XbusParser::parseMagneticField(const uint8_t* xbusData, MagneticFieldXYZ& magneticField) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasMagneticField) {
        magneticField = sensorData.magneticField;
        return true;
    }
    return false;
}

bool XbusParser::parseTemperature(const uint8_t* xbusData, Temperature& temperature) {
    SensorData sensorData;
    if (parseMTData2(xbusData, sensorData) && sensorData.hasTemperature) {
        temperature = sensorData.temperature;
        return true;
    }
    return false;
}

uint32_t XbusParser::parseDeviceId(const uint8_t* xbusData) {
    if (!Xbus::checkPreamble(xbusData)) {
        return 0;
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    if (messageId != XMID_DeviceId) {
        return 0;
    }

    int index = 4;
    return readUint32(xbusData, index);
}

bool XbusParser::parseFirmwareRevision(const uint8_t* xbusData, char* output, size_t maxLen) {
    if (!output || maxLen == 0) {
        return false;
    }

    if (!Xbus::checkPreamble(xbusData)) {
        return false;
    }

    uint8_t messageId = Xbus::getMessageId(xbusData);
    if (messageId != XMID_FirmwareRevision) {
        return false;
    }

    int index = 4;
    uint8_t major = readUint8(xbusData, index);
    uint8_t minor = readUint8(xbusData, index);
    uint8_t patch = readUint8(xbusData, index);

    snprintf(output, maxLen, "%d.%d.%d",
            static_cast<int>(major), static_cast<int>(minor), static_cast<int>(patch));

    return true;
}

// Helper functions for safe string operations
bool XbusParser::appendToBuffer(char* buffer, size_t maxLen, const char* str) {
    if (!buffer || !str || maxLen == 0) {
        return false;
    }

    size_t currentLen = strlen(buffer);
    size_t strLen = strlen(str);

    if (currentLen + strLen >= maxLen) {
        return false; // Not enough space
    }

    strcat(buffer, str);
    return true;
}

bool XbusParser::appendFloatToBuffer(char* buffer, size_t maxLen, float value, int precision) {
    if (!buffer || maxLen == 0) {
        return false;
    }

    char tempStr[32];
    snprintf(tempStr, sizeof(tempStr), "%.*f", precision, static_cast<double>(value));

    return appendToBuffer(buffer, maxLen, tempStr);
}

bool XbusParser::appendDoubleToBuffer(char* buffer, size_t maxLen, double value, int precision) {
    if (!buffer || maxLen == 0) {
        return false;
    }

    char tempStr[32];
    snprintf(tempStr, sizeof(tempStr), "%.*f", precision, value);

    return appendToBuffer(buffer, maxLen, tempStr);
}

bool XbusParser::appendIntToBuffer(char* buffer, size_t maxLen, int value) {
    if (!buffer || maxLen == 0) {
        return false;
    }

    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "%d", value);

    return appendToBuffer(buffer, maxLen, tempStr);
}

bool XbusParser::appendHexToBuffer(char* buffer, size_t maxLen, uint32_t value, int width) {
    if (!buffer || maxLen == 0) {
        return false;
    }

    char tempStr[16];
    snprintf(tempStr, sizeof(tempStr), "0x%0*X", width, static_cast<unsigned int>(value));

    return appendToBuffer(buffer, maxLen, tempStr);
}
