// HandbrakeSensor.cpp
#include "HandbrakeSensor.h"

HandbrakeSensor::HandbrakeSensor(uint8_t doutPin, uint8_t sckPin) {
  _hx.begin(doutPin, sckPin);
  _hx.set_scale(10);  // scale factor for conversion
}

bool HandbrakeSensor::isReady() {
  return _hx.is_ready();
}

long HandbrakeSensor::readRaw() {
  // Direct raw read from HX711
  return _hx.read();
}
