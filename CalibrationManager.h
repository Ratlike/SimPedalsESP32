// CalibrationManager.h
#pragma once

#include <Arduino.h>
#include "Storage.h"

// Enumeration for pedals
enum Pedal {
  PEDAL_BRAKE = 0,
  PEDAL_ACCEL = 1,
  PEDAL_CLUTCH = 2,
  PEDAL_HANDBRAKE = 3,
  PEDAL_COUNT
};

class CalibrationManager {
public:
  // Initialize storage and load any saved calibration
  void begin();

  // Process raw sensor values each loop
  void update(float rawBrake, float rawAccel, float rawClutch, float rawHandbrake);

  // Calibration mode status
  bool inCalibration() const;

  // Browser-driven calibration control
  void enterCalibration();
  void exitCalibration();
  void setMin(Pedal p, float value);
  void setMax(Pedal p, float value);
  void freezeMin(Pedal p);
  void freezeMax(Pedal p);

  // Access calibrated bounds
  float getMin(Pedal p) const;
  float getMax(Pedal p) const;

  // Auto-zero control
  void setAutoZeroEnabled(bool enabled);
  bool isAutoZeroEnabled() const;

private:
  Storage storage_;

  bool calibMode_ = false;

  float minVals_[PEDAL_COUNT];
  float maxVals_[PEDAL_COUNT];
  float rawBuffer_[PEDAL_COUNT];

  // Auto-zero: variance-based idle detection
  bool          autoZeroEnabled_ = true;
  void autoZeroUpdate(const float raws[]);
  void resetAutoZero();

  float         azMeanEma_[PEDAL_COUNT];
  float         azVarEma_[PEDAL_COUNT];
  unsigned long azStableStartMs_[PEDAL_COUNT];
  bool          azStable_[PEDAL_COUNT];

  // Auto-zero: accumulation window
  bool          azAccumulating_[PEDAL_COUNT];
  float         azAccumulator_[PEDAL_COUNT];
  unsigned long azSampleCount_[PEDAL_COUNT];
  unsigned long azWindowStartMs_[PEDAL_COUNT];

  // Auto-zero: cooldown
  unsigned long azLastRecalMs_[PEDAL_COUNT];
};
