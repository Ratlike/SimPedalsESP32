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

  // Process button states and raw sensor values each loop
  void update(bool shifter1Pressed,
              bool shifter2Pressed,
              float rawBrake,
              float rawAccel,
              float rawClutch,
              float rawHandbrake);

  // Calibration mode status
  bool inCalibration() const;

  // Mode toggle event
  bool hasModeChanged() const;
  void clearModeChanged();

  // Pedal cycle event
  bool hasPedalCycled() const;
  void clearPedalCycled();

  // Range change event (new rest-min or new max)
  bool hasRangeChanged() const;
  void clearRangeChanged();

  // Retrieve last event values
  float lastRaw() const;
  float lastMin() const;
  float lastMax() const;
  float getCurrentRaw() const;

  // Access calibrated bounds
  float getMin(Pedal p) const;
  float getMax(Pedal p) const;

  // Which pedal is currently selected in calibration
  Pedal getSelectedPedal() const;

  // Auto-zero control
  void setAutoZeroEnabled(bool enabled);
  bool isAutoZeroEnabled() const;

private:
  void toggleMode();
  void cyclePedal();

  Storage storage_;

  bool calibMode_ = false;
  Pedal selected_ = PEDAL_BRAKE;
  bool holdActive_ = false;
  unsigned long holdStartMs_ = 0;

  float minVals_[PEDAL_COUNT];
  float maxVals_[PEDAL_COUNT];
  float rawBuffer_[PEDAL_COUNT];

  // For rest-position averaging
  float restAccumulator_[PEDAL_COUNT];
  unsigned long restSampleCount_[PEDAL_COUNT];
  unsigned long restWindowStartMs_[PEDAL_COUNT];
  bool restCaptured_[PEDAL_COUNT];

  // Event flags and stored event data
  bool modeChanged_ = false;
  bool pedalCycled_ = false;
  bool rangeChanged_ = false;
  float lastRaw_ = 0.0f;
  float lastMin_ = 0.0f;
  float lastMax_ = 0.0f;

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