#include "CalibrationHID.h"
#include <cstring>

static const uint8_t CALIB_HID_REPORT_DESC[] = {
    0x06, 0x00, 0xFF,  // Usage Page (Vendor Defined 0xFF00)
    0x09, 0x01,        // Usage (0x01)
    0xA1, 0x01,        // Collection (Application)

    0x85, 0x04,        //   Report ID (0x04) - GET_FEATURE
    0x09, 0x14,        //   Usage (0x14)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x32,        //   Report Count (50)
    0xB1, 0x02,        //   Feature (Data, Variable, Absolute)

    0x85, 0x05,        //   Report ID (0x05) - SET_FEATURE
    0x09, 0x15,        //   Usage (0x15)
    0x15, 0x00,        //   Logical Minimum (0)
    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x08,        //   Report Count (8)
    0xB1, 0x02,        //   Feature (Data, Variable, Absolute)

    0xC0               // End Collection
};

void CalibrationHID::begin() {
    _hid.addDevice(this, sizeof(CALIB_HID_REPORT_DESC));
}

void CalibrationHID::setCalibrationManager(CalibrationManager* cm) {
    _calib = cm;
}

void CalibrationHID::updateState(bool calibMode, const float raw[4],
                                  const float min[4], const float max[4]) {
    _state.calibMode = calibMode ? 1 : 0;
    _state.reserved = 0;
    memcpy(_state.raw, raw, sizeof(float) * 4);
    memcpy(_state.min, min, sizeof(float) * 4);
    memcpy(_state.max, max, sizeof(float) * 4);
}

uint16_t CalibrationHID::_onGetDescriptor(uint8_t* buffer) {
    memcpy(buffer, CALIB_HID_REPORT_DESC, sizeof(CALIB_HID_REPORT_DESC));
    return sizeof(CALIB_HID_REPORT_DESC);
}

uint16_t CalibrationHID::_onGetFeature(uint8_t report_id, uint8_t* buffer, uint16_t len) {
    if (report_id == 0x04) {
        uint16_t copyLen = sizeof(CalibGetReport);
        if (copyLen > len) copyLen = len;
        memcpy(buffer, &_state, copyLen);
        return copyLen;
    }
    return 0;
}

void CalibrationHID::_onSetFeature(uint8_t report_id, const uint8_t* buffer, uint16_t len) {
    if (report_id != 0x05 || len < sizeof(CalibSetReport) || !_calib)
        return;

    CalibSetReport cmd;
    memcpy(&cmd, buffer, sizeof(CalibSetReport));

    if (cmd.pedal >= PEDAL_COUNT)
        return;

    Pedal p = static_cast<Pedal>(cmd.pedal);

    switch (cmd.command) {
        case CMD_ENTER_CALIB:
            _calib->enterCalibration();
            break;
        case CMD_EXIT_CALIB:
            _calib->exitCalibration();
            break;
        case CMD_SET_MIN:
            _calib->setMin(p, cmd.value);
            break;
        case CMD_SET_MAX:
            _calib->setMax(p, cmd.value);
            break;
        case CMD_FREEZE_MIN:
            _calib->freezeMin(p);
            break;
        case CMD_FREEZE_MAX:
            _calib->freezeMax(p);
            break;
    }
}
