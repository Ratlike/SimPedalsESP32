// HandbrakeSensor.h
#pragma once
#include <HX711.h>

// Provides direct access to the HX711 load-cell ADC.
class HandbrakeSensor
{
public:
  // Initializes HX711 on specified data and clock pins.
  HandbrakeSensor(uint8_t doutPin, uint8_t sckPin);

  // Returns true if the HX711 has data ready to read.
  bool isReady();

  // Returns a single raw reading from the HX711.
  long readRaw();

private:
  HX711 _hx;
};