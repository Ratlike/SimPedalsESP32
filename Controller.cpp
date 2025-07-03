// Controller.cpp
#include "Controller.h"
#include <Joystick_ESP32S2.h>
#include <cmath>  // for fabsf
#include "CalibrationManager.h"

/*
 * Single-axis USB HID joystick.
 * Only the brake axis is used to report pedal force.
 */
static Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_GAMEPAD,
  CONTROLLER_BUTTON_COUNT, 0,  // buttons, hats
  true, true, false,           // X Y Z
  false, false, false,         // Rx Ry Rz
  false, true,                 // rudder, throttle
  false, true, false);         // accelerator, brake, steering

void SetupController() {

  USB.PID(0x8217);
  USB.VID(0x3035);
  USB.productName("Simracing Pedals");
  USB.manufacturerName("OpenSource");
  USB.begin();

  Joystick.setXAxisRange(JOYSTICK_MIN_VALUE, JOYSTICK_MAX_VALUE);
  Joystick.setYAxisRange(JOYSTICK_MIN_VALUE, JOYSTICK_MAX_VALUE);
  Joystick.setThrottleRange(JOYSTICK_MIN_VALUE, JOYSTICK_MAX_VALUE);
  Joystick.setBrakeRange(JOYSTICK_MIN_VALUE, JOYSTICK_MAX_VALUE);

  delay(100);        // allow USB stack to settle
  Joystick.begin(false);  // manual USB start, no auto-report
}

bool IsControllerReady() {
  return true;  // USB is ready immediately after begin()
}

void SetControllerOutputValue(int32_t value) {
  Joystick.setBrake(value);
  Joystick.sendState();
}

void SetBrake(int32_t value) {
  Joystick.setBrake(value);
}

void SetAccelerator(int32_t value) {
  Joystick.setXAxis(value);
}

void SetClutch(int32_t value) {
  Joystick.setYAxis(value);
}

void SetHandbrake(int32_t value) {
  Joystick.setThrottle(value);
}

void SetButton(uint8_t idx, bool pressed) {
  if (idx < CONTROLLER_BUTTON_COUNT) {
    Joystick.setButton(idx, pressed);
  }
}

void sendState() {
  Joystick.sendState();
}

int32_t NormalizeControllerOutputValue(Pedal p,
                                       float value,
                                       float minVal,
                                       float maxVal,
                                       float maxGameOutput) {
  float span = maxVal - minVal;
  if (fabsf(span) < 0.01f)
    return JOYSTICK_MIN_VALUE;

  // pick the hard-coded dead-zones
  float dzLow, dzHigh;
  switch (p) {
    case PEDAL_BRAKE:
      dzLow = BRAKE_DZ_LOW;
      dzHigh = BRAKE_DZ_HIGH;
      break;
    case PEDAL_ACCEL:
      dzLow = ACCEL_DZ_LOW;
      dzHigh = ACCEL_DZ_HIGH;
      break;
    case PEDAL_CLUTCH:
      dzLow = CLUTCH_DZ_LOW;
      dzHigh = CLUTCH_DZ_HIGH;
      break;
    case PEDAL_HANDBRAKE:
      dzLow = HANDBRAKE_DZ_LOW;
      dzHigh = HANDBRAKE_DZ_HIGH;
      break;
    default:
      dzLow = dzHigh = 0.0f;
  }

  // compute edges
  float loEdge = minVal + span * dzLow;
  float hiEdge = maxVal - span * dzHigh;

  // clamp into dead-zone
  if (value <= loEdge)
    return JOYSTICK_MIN_VALUE;
  if (value >= hiEdge)
    return static_cast<int32_t>((maxGameOutput / 100.0f) * JOYSTICK_MAX_VALUE);

  // scale the middle region
  float frac = (value - loEdge) / (hiEdge - loEdge);
  int32_t rawOut = JOYSTICK_MIN_VALUE + int32_t(frac * JOYSTICK_RANGE);

  // apply your existing upper-limit
  int32_t upperLimit =
    static_cast<int32_t>((maxGameOutput / 100.0f) * JOYSTICK_MAX_VALUE);
  return constrain(rawOut, JOYSTICK_MIN_VALUE, upperLimit);
}