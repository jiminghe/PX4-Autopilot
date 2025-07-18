#include "xbus.h"

bool Xbus::checkPreamble(const uint8_t* xbusMessage) {
    return xbusMessage[OFFSET_TO_PREAMBLE] == XBUS_PREAMBLE;
}

int Xbus::getBusId(const uint8_t* xbusMessage) {
    return (xbusMessage[OFFSET_TO_BID] & 0xff);
}

void Xbus::setBusId(uint8_t* xbusMessage, uint8_t busId) {
    xbusMessage[OFFSET_TO_BID] = busId & 0xff;
}

int Xbus::getMessageId(const uint8_t* xbusMessage) {
    return (xbusMessage[OFFSET_TO_MID] & 0xff);
}

void Xbus::setMessageId(uint8_t* xbusMessage, uint8_t messageId) {
    xbusMessage[OFFSET_TO_MID] = messageId & 0xff;
}

int Xbus::getPayloadLength(const uint8_t* xbusMessage) {
    int length = xbusMessage[OFFSET_TO_LEN] & 0xff;
    if (length != LENGTH_EXTENDER_BYTE) {
        return length;
    } else {
        int result = (xbusMessage[OFFSET_TO_LEN + 2] & 0xff);
        result += (xbusMessage[OFFSET_TO_LEN + 1] & 0xff) << 8;
        return result;
    }
}

void Xbus::setPayloadLength(uint8_t* xbusMessage, uint16_t payloadLength) {
    if (payloadLength < 255) {
        xbusMessage[OFFSET_TO_LEN] = payloadLength & 0xff;
    } else {
        xbusMessage[OFFSET_TO_LEN] = LENGTH_EXTENDER_BYTE;
        xbusMessage[OFFSET_TO_LEN + 1] = (payloadLength >> 8) & 0xff;
        xbusMessage[OFFSET_TO_LEN + 2] = payloadLength & 0xff;
    }
}

void Xbus::createMessage(uint8_t* xbusMessage, uint8_t bid, uint8_t mid, uint16_t len) {
    xbusMessage[0] = XBUS_PREAMBLE;
    setBusId(xbusMessage, bid);
    setMessageId(xbusMessage, mid);
    setPayloadLength(xbusMessage, len);
}

int Xbus::getRawLength(const uint8_t* xbusMessage) {
    int result = getPayloadLength(xbusMessage);

    if ((xbusMessage[OFFSET_TO_LEN] & 0xff) == LENGTH_EXTENDER_BYTE) {
        result += 7;
    } else {
        result += 5;
    }
    return result;
}

uint8_t* Xbus::getPointerToPayload(uint8_t* xbusMessage) {
    if ((xbusMessage[OFFSET_TO_LEN] & 0xff) == LENGTH_EXTENDER_BYTE) {
        return xbusMessage + OFFSET_TO_PAYLOAD_EXT;
    } else {
        return xbusMessage + OFFSET_TO_PAYLOAD;
    }
}

const uint8_t* Xbus::getConstPointerToPayload(const uint8_t* xbusMessage) {
    return getPointerToPayload(const_cast<uint8_t*>(xbusMessage));
}

void Xbus::insertChecksum(uint8_t* xbusMessage) {
    int nBytes = getRawLength(xbusMessage);

    uint8_t checksum = 0;
    for (int i = 0; i < nBytes - 2; i++) {
        checksum -= xbusMessage[1 + i];
    }

    xbusMessage[nBytes - 1] = checksum;
}

bool Xbus::verifyChecksum(const uint8_t* xbusMessage) {
    int nBytes = getRawLength(xbusMessage);
    uint8_t checksum = 0;
    for (int n = 1; n < nBytes; n++) {
        checksum += (xbusMessage[n] & 0xff);
    }
    checksum &= 0xff;
    return (checksum == 0);
}

size_t Xbus::createRawMessage(uint8_t* dest, const uint8_t* message) {
    int n;
    uint8_t checksum;
    uint16_t length;
    uint8_t* dptr = dest;

    length = getPayloadLength(message);

    if (dest == nullptr) {
        return (length < 255) ? length + 5 : length + 7;
    }

    *dptr++ = XBUS_PREAMBLE;
    *dptr++ = XBUS_MASTERDEVICE;

    checksum = 0;
    checksum -= XBUS_MASTERDEVICE;

    *dptr = getMessageId(message);
    checksum -= *dptr++;

    if (length < XBUS_EXTENDED_LENGTH) {
        *dptr = length & 0xFF;
        checksum -= *dptr++;
    } else {
        *dptr = XBUS_EXTENDED_LENGTH;
        checksum -= *dptr++;
        *dptr = length >> 8;
        checksum -= *dptr++;
        *dptr = length & 0xFF;
        checksum -= *dptr++;
    }

    for (n = 0; n < length; n++) {
        *dptr = getConstPointerToPayload(message)[n];
        checksum -= *dptr++;
    }

    *dptr++ = checksum;

    return dptr - dest;
}
