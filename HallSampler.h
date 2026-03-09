// HallSampler.h
#pragma once
#include <Arduino.h>

namespace HallSampler
{
  void begin(uint32_t sampleHz = 2000, BaseType_t core = 1);
  float getAccelRaw();
  float getClutchRaw();
}
