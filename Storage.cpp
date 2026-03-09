// Storage.cpp
#include "Storage.h"
#include "Main.h"

void Storage::begin()
{
  // no-op here; each load/save opens+closes
}

void Storage::loadCalibration(float minVals[], float maxVals[])
{
  prefs.begin("calib", false);
  minVals[0] = prefs.getFloat("brkMin", 0.0f);
  maxVals[0] = prefs.getFloat("brkMax", LOADCELL_WEIGHT_RATING_KG);
  minVals[1] = prefs.getFloat("accMin", 0.0f);
  maxVals[1] = prefs.getFloat("accMax", 4095.0f);
  minVals[2] = prefs.getFloat("cltMin", 0.0f);
  maxVals[2] = prefs.getFloat("cltMax", 4095.0f);
  minVals[3] = prefs.getFloat("hndMin", 0.0f);
  maxVals[3] = prefs.getFloat("hndMax", 4095.0f);
  prefs.end();
}

void Storage::saveCalibration(const float minVals[], const float maxVals[])
{
  prefs.begin("calib", false);
  prefs.putFloat("brkMin", minVals[0]);
  prefs.putFloat("brkMax", maxVals[0]);
  prefs.putFloat("accMin", minVals[1]);
  prefs.putFloat("accMax", maxVals[1]);
  prefs.putFloat("cltMin", minVals[2]);
  prefs.putFloat("cltMax", maxVals[2]);
  prefs.putFloat("hndMin", minVals[3]);
  prefs.putFloat("hndMax", maxVals[3]);
  prefs.end();
  Serial.println("Calibration saved to NVS");
}
