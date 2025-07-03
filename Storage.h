// Storage.h
#pragma once
#include <Preferences.h>

class Storage {
public:
  // must be called before any load/save
  void begin();

  // load per-pedal raw min/max arrays (size PEDAL_COUNT)
  void loadCalibration(float minVals[], float maxVals[]);

  // save per-pedal raw min/max arrays
  void saveCalibration(const float minVals[], const float maxVals[]);

private:
  Preferences prefs;
};
