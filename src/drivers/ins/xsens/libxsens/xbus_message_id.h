#ifndef XBUS_MESSAGE_ID_H
#define XBUS_MESSAGE_ID_H

#include <cstdint>

enum class XsMessageId : uint8_t {
    Wakeup                = 0x3E,
    WakeupAck             = 0x3F,
    ReqDid                = 0x00,
    DeviceId              = 0x01,
    GotoConfig            = 0x30,
    GotoConfigAck         = 0x31,
    GotoMeasurement       = 0x10,
    GotoMeasurementAck    = 0x11,
    MtData2               = 0x36,
    ReqOutputConfig       = 0xC0,
    SetOutputConfig       = 0xC0,
    OutputConfig          = 0xC1,
    Reset                 = 0x40,
    ResetAck              = 0x41,
    Error                 = 0x42,
    SetAlignmentRotation  = 0xEC,
    AlignmentRotationAck  = 0xED,
    FirmwareUpdate        = 0xF2,
    GotoBootLoader        = 0xF0,
    GotoBootLoaderAck     = 0xF1,
    ReqFirmwareRevision   = 0x12,
    FirmwareRevision      = 0x13
};

// Legacy defines for backwards compatibility
#define XMID_Wakeup                  static_cast<uint8_t>(XsMessageId::Wakeup)
#define XMID_WakeupAck               static_cast<uint8_t>(XsMessageId::WakeupAck)
#define XMID_ReqDid                  static_cast<uint8_t>(XsMessageId::ReqDid)
#define XMID_DeviceId                static_cast<uint8_t>(XsMessageId::DeviceId)
#define XMID_GotoConfig              static_cast<uint8_t>(XsMessageId::GotoConfig)
#define XMID_GotoConfigAck           static_cast<uint8_t>(XsMessageId::GotoConfigAck)
#define XMID_GotoMeasurement         static_cast<uint8_t>(XsMessageId::GotoMeasurement)
#define XMID_GotoMeasurementAck      static_cast<uint8_t>(XsMessageId::GotoMeasurementAck)
#define XMID_MtData2                 static_cast<uint8_t>(XsMessageId::MtData2)
#define XMID_ReqOutputConfig         static_cast<uint8_t>(XsMessageId::ReqOutputConfig)
#define XMID_SetOutputConfig         static_cast<uint8_t>(XsMessageId::SetOutputConfig)
#define XMID_OutputConfig            static_cast<uint8_t>(XsMessageId::OutputConfig)
#define XMID_Reset                   static_cast<uint8_t>(XsMessageId::Reset)
#define XMID_ResetAck                static_cast<uint8_t>(XsMessageId::ResetAck)
#define XMID_Error                   static_cast<uint8_t>(XsMessageId::Error)
#define XMID_SetAlignmentRotation    static_cast<uint8_t>(XsMessageId::SetAlignmentRotation)
#define XMID_AlignmentRotationAck    static_cast<uint8_t>(XsMessageId::AlignmentRotationAck)
#define XMID_FirmwareUpdate          static_cast<uint8_t>(XsMessageId::FirmwareUpdate)
#define XMID_GotoBootLoader          static_cast<uint8_t>(XsMessageId::GotoBootLoader)
#define XMID_GotoBootLoaderAck       static_cast<uint8_t>(XsMessageId::GotoBootLoaderAck)
#define XMID_ReqFirmwareRevision     static_cast<uint8_t>(XsMessageId::ReqFirmwareRevision)
#define XMID_FirmwareRevision        static_cast<uint8_t>(XsMessageId::FirmwareRevision)

#endif // XBUS_MESSAGE_ID_H
