#ifndef XBUS_H
#define XBUS_H

#include <cstdint>
#include <cstddef>

class Xbus {
public:
    // Constants
    static constexpr uint8_t OFFSET_TO_PREAMBLE = 0;
    static constexpr uint8_t OFFSET_TO_BID = 1;
    static constexpr uint8_t OFFSET_TO_MID = 2;
    static constexpr uint8_t OFFSET_TO_LEN = 3;
    static constexpr uint8_t OFFSET_TO_LEN_EXT_HI = 4;
    static constexpr uint8_t OFFSET_TO_LEN_EXT_LO = 5;
    static constexpr uint8_t OFFSET_TO_PAYLOAD = 4;
    static constexpr uint8_t OFFSET_TO_PAYLOAD_EXT = 6;
    static constexpr uint8_t XBUS_CHECKSUM_SIZE = 1;
    static constexpr uint8_t LENGTH_EXTENDER_BYTE = 0xFF;
    static constexpr uint8_t XBUS_PREAMBLE = 0xFA;
    static constexpr uint8_t XBUS_MASTERDEVICE = 0xFF;
    static constexpr uint8_t XBUS_EXTENDED_LENGTH = 0xFF;

    // Static methods
    static bool checkPreamble(const uint8_t* xbusMessage);

    static int getBusId(const uint8_t* xbusMessage);
    static void setBusId(uint8_t* xbusMessage, uint8_t busId);

    static int getMessageId(const uint8_t* xbusMessage);
    static void setMessageId(uint8_t* xbusMessage, uint8_t messageId);

    static int getPayloadLength(const uint8_t* xbusMessage);
    static void setPayloadLength(uint8_t* xbusMessage, uint16_t payloadLength);

    static void createMessage(uint8_t* xbusMessage, uint8_t bid, uint8_t mid, uint16_t len);

    static int getRawLength(const uint8_t* xbusMessage);

    static uint8_t* getPointerToPayload(uint8_t* xbusMessage);
    static const uint8_t* getConstPointerToPayload(const uint8_t* xbusMessage);

    static void insertChecksum(uint8_t* xbusMessage);
    static bool verifyChecksum(const uint8_t* xbusMessage);

    // Helper method for creating raw messages for transmission
    static size_t createRawMessage(uint8_t* dest, const uint8_t* message);
};

#endif // XBUS_H
