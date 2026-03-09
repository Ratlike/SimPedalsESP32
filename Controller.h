// Controller.h
#pragma once

#include "Arduino.h"
#include "Main.h"
#include "CalibrationManager.h"
#include "CalibrationHID.h"

extern CalibrationHID calibHid;

/*#ifdef CONFIG_IDF_TARGET_ESP32S2 ||ARDUINO_ESP32S3_DEV || CONFIG_IDF_TARGET_ESP32S3
  #define USB_JOYSTICK
#elif CONFIG_IDF_TARGET_ESP32
  #define BLUETOOTH_GAMEPAD
#endif
*/
static const int16_t JOYSTICK_MIN_VALUE = 0;
static const int16_t JOYSTICK_MAX_VALUE = 10000;
static const int16_t JOYSTICK_RANGE = JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE;

constexpr float BRAKE_DZ_LOW = 0.05f;
constexpr float BRAKE_DZ_HIGH = 0.00f;

constexpr float ACCEL_DZ_LOW = 0.05f;
constexpr float ACCEL_DZ_HIGH = 0.05f;

constexpr float CLUTCH_DZ_LOW = 0.05f;
constexpr float CLUTCH_DZ_HIGH = 0.05f;

constexpr float HANDBRAKE_DZ_LOW = 0.10f;
constexpr float HANDBRAKE_DZ_HIGH = 0.00f;

void SetupController();
bool IsControllerReady();

int32_t NormalizeControllerOutputValue(Pedal p, float value, float minVal, float maxVal, float maxGameOutput);

void SetBrake(int32_t value);
void SetAccelerator(int32_t value);
void SetClutch(int32_t value);
void SetHandbrake(int32_t value);

void SetButton(uint8_t index, bool pressed);

void sendState();