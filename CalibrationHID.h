#pragma once
#include "USB.h"
#include "USBHID.h"
#include "CalibrationManager.h"

enum CalibCmd : uint8_t {
    CMD_ENTER_CALIB  = 0x01,
    CMD_EXIT_CALIB   = 0x02,
    CMD_SET_MIN      = 0x03,
    CMD_SET_MAX      = 0x04,
    CMD_FREEZE_MIN   = 0x05,
    CMD_FREEZE_MAX   = 0x06,
};

struct __attribute__((packed)) CalibGetReport {
    uint8_t calibMode;
    uint8_t reserved;
    float   raw[4];
    float   min[4];
    float   max[4];
};  // 50 bytes

struct __attribute__((packed)) CalibSetReport {
    uint8_t command;
    uint8_t pedal;
    uint8_t reserved[2];
    float   value;
};  // 8 bytes

class CalibrationHID : public USBHIDDevice {
public:
    void begin();
    void setCalibrationManager(CalibrationManager* cm);
    void updateState(bool calibMode, const float raw[4],
                     const float min[4], const float max[4]);

    // USBHIDDevice overrides
    uint16_t _onGetDescriptor(uint8_t* buffer) override;
    uint16_t _onGetFeature(uint8_t report_id, uint8_t* buffer, uint16_t len) override;
    void     _onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len) override;

private:
    USBHID _hid;
    CalibGetReport _state = {};
    CalibrationManager* _calib = nullptr;
};
