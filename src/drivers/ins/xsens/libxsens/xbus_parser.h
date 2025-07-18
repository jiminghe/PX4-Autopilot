#ifndef XBUS_PARSER_H
#define XBUS_PARSER_H

#include "xbus.h"
#include "xbus_message_id.h"
#include <cstdint>
#include <cstring>  // For memcpy

// Maximum string lengths for output buffers
static constexpr size_t MAX_MESSAGE_STRING_LEN = 256;
static constexpr size_t MAX_SENSOR_DATA_STRING_LEN = 512;
static constexpr size_t MAX_TIMESTAMP_STRING_LEN = 32;
static constexpr size_t MAX_FIRMWARE_STRING_LEN = 16;

// Data structure definitions
struct EulerAngles {
    float roll;
    float pitch;
    float yaw;

    EulerAngles() : roll(0.0f), pitch(0.0f), yaw(0.0f) {}
    EulerAngles(float r, float p, float y) : roll(r), pitch(p), yaw(y) {}
};

struct LatLon {
    double latitude;
    double longitude;

    LatLon() : latitude(0.0), longitude(0.0) {}
    LatLon(double lat, double lon) : latitude(lat), longitude(lon) {}
};

struct VelocityXYZ {
    double velX;
    double velY;
    double velZ;

    VelocityXYZ() : velX(0.0), velY(0.0), velZ(0.0) {}
    VelocityXYZ(double x, double y, double z) : velX(x), velY(y), velZ(z) {}
};

struct Quaternion {
    float q0;  // w component
    float q1;  // x component
    float q2;  // y component
    float q3;  // z component

    Quaternion() : q0(1.0f), q1(0.0f), q2(0.0f), q3(0.0f) {}
    Quaternion(float w, float x, float y, float z) : q0(w), q1(x), q2(y), q3(z) {}
};

struct UtcTime {
    uint32_t nanoseconds;    // Fractional part (nanoseconds)
    uint16_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t flags;

    UtcTime() : nanoseconds(0), year(0), month(0), day(0), hour(0), minute(0), second(0), flags(0) {}
};

struct BarometricPressure {
    uint32_t pressure;  // Pressure in Pa (Pascal)

    BarometricPressure() : pressure(0) {}
    BarometricPressure(uint32_t p) : pressure(p) {}
};

// New data structures for the missing types
struct AccelerationXYZ {
    float accX;  // m/s²
    float accY;  // m/s²
    float accZ;  // m/s²

    AccelerationXYZ() : accX(0.0f), accY(0.0f), accZ(0.0f) {}
    AccelerationXYZ(float x, float y, float z) : accX(x), accY(y), accZ(z) {}
};

struct RateOfTurnXYZ {
    float gyrX;  // rad/s
    float gyrY;  // rad/s
    float gyrZ;  // rad/s

    RateOfTurnXYZ() : gyrX(0.0f), gyrY(0.0f), gyrZ(0.0f) {}
    RateOfTurnXYZ(float x, float y, float z) : gyrX(x), gyrY(y), gyrZ(z) {}
};

struct MagneticFieldXYZ {
    float magX;  // arbitrary unit
    float magY;  // arbitrary unit
    float magZ;  // arbitrary unit

    MagneticFieldXYZ() : magX(0.0f), magY(0.0f), magZ(0.0f) {}
    MagneticFieldXYZ(float x, float y, float z) : magX(x), magY(y), magZ(z) {}
};

struct Temperature {
    float temperature;  // °C

    Temperature() : temperature(0.0f) {}
    Temperature(float temp) : temperature(temp) {}
};

struct SensorData {
    bool hasPacketCounter = false;
    bool hasSampleTimeFine = false;
    bool hasEulerAngles = false;
    bool hasStatusWord = false;
    bool hasLatLon = false;
    bool hasAltitudeEllipsoid = false;
    bool hasVelocityXYZ = false;
    bool hasUtcTime = false;
    bool hasQuaternion = false;
    bool hasBarometricPressure = false;
    bool hasAcceleration = false;
    bool hasRateOfTurn = false;
    bool hasMagneticField = false;
    bool hasTemperature = false;

    uint16_t packetCounter = 0;
    uint32_t sampleTimeFine = 0;
    EulerAngles eulerAngles;
    uint32_t statusWord = 0;
    LatLon latLon;
    double altitudeEllipsoid = 0.0;
    VelocityXYZ velocityXYZ;
    UtcTime utcTime;
    Quaternion quaternion;
    BarometricPressure barometricPressure;
    AccelerationXYZ acceleration;
    RateOfTurnXYZ rateOfTurn;
    MagneticFieldXYZ magneticField;
    Temperature temperature;
};

// XDI (Xsens Data Identifier) constants
namespace XDI {
    constexpr uint16_t PACKET_COUNTER = 0x1020;
    constexpr uint16_t SAMPLE_TIME_FINE = 0x1060;
    constexpr uint16_t EULER_ANGLES = 0x2030;
    constexpr uint16_t STATUS_WORD = 0xE020;
    constexpr uint16_t LAT_LON = 0x5042;
    constexpr uint16_t ALTITUDE_ELLIPSOID = 0x5022;
    constexpr uint16_t VELOCITY_XYZ = 0xD012;
    constexpr uint16_t QUATERNION = 0x2010;
    constexpr uint16_t ACCELERATION = 0x4020;
    constexpr uint16_t RATE_OF_TURN = 0x8020;
    constexpr uint16_t MAGNETIC_FIELD = 0xC020;
    constexpr uint16_t UTC_TIME = 0x1010;
    constexpr uint16_t BAROMETRIC_PRESSURE = 0x3010;
    constexpr uint16_t TEMPERATURE = 0x0810;
}

class XbusParser {
public:
    // Helper functions for reading data from payload
    static uint8_t readUint8(const uint8_t* data, int& index);
    static uint16_t readUint16(const uint8_t* data, int& index);
    static uint32_t readUint32(const uint8_t* data, int& index);
    static float readFloat(const uint8_t* data, int& index);
    static double readFP1632(const uint8_t* data, int& index);  // For FP16.32 format

    // Parse specific message types - using fixed-size output buffers
    static bool messageToString(const uint8_t* xbusData, char* output, size_t maxLen);
    static bool parseEulerAngles(const uint8_t* xbusData, EulerAngles& angles);
    static bool parseQuaternion(const uint8_t* xbusData, Quaternion& quaternion);
    static bool parseUtcTime(const uint8_t* xbusData, UtcTime& utcTime);
    static bool parseBarometricPressure(const uint8_t* xbusData, BarometricPressure& pressure);
    static bool parseAcceleration(const uint8_t* xbusData, AccelerationXYZ& acceleration);
    static bool parseRateOfTurn(const uint8_t* xbusData, RateOfTurnXYZ& rateOfTurn);
    static bool parseMagneticField(const uint8_t* xbusData, MagneticFieldXYZ& magneticField);
    static bool parseTemperature(const uint8_t* xbusData, Temperature& temperature);
    static uint32_t parseDeviceId(const uint8_t* xbusData);
    static bool parseFirmwareRevision(const uint8_t* xbusData, char* output, size_t maxLen);

    // Enhanced MTData2 parsing
    static bool parseMTData2(const uint8_t* xbusData, SensorData& sensorData);
    static bool formatSensorData(const SensorData& data, char* output, size_t maxLen);

private:
    static const char* getXDIName(uint16_t xdi);
    static bool formatStatusWord(uint32_t statusWord, char* output, size_t maxLen);
    static bool formatUtcTime(const UtcTime& utcTime, char* output, size_t maxLen);
    static bool formatQuaternion(const Quaternion& quaternion, char* output, size_t maxLen);
    static bool formatBarometricPressure(const BarometricPressure& pressure, char* output, size_t maxLen);
    static bool formatAcceleration(const AccelerationXYZ& acceleration, char* output, size_t maxLen);
    static bool formatRateOfTurn(const RateOfTurnXYZ& rateOfTurn, char* output, size_t maxLen);
    static bool formatMagneticField(const MagneticFieldXYZ& magneticField, char* output, size_t maxLen);
    static bool formatTemperature(const Temperature& temperature, char* output, size_t maxLen);

    // Helper functions for safe string operations
    static bool appendToBuffer(char* buffer, size_t maxLen, const char* str);
    static bool appendFloatToBuffer(char* buffer, size_t maxLen, float value, int precision);
    static bool appendDoubleToBuffer(char* buffer, size_t maxLen, double value, int precision);
    static bool appendIntToBuffer(char* buffer, size_t maxLen, int value);
    static bool appendHexToBuffer(char* buffer, size_t maxLen, uint32_t value, int width);
};

#endif // XBUS_PARSER_H
