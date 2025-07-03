// CalibrationManager.cpp
#include "CalibrationManager.h"
#include <Arduino.h>

static const uint32_t HOLD_MS = 3000;         // ms to hold SHIFTER1 for toggle
static const uint32_t REST_WINDOW_MS = 2000;  // ms window for rest averaging
static const char* pedalNames[] = { "Brake", "Accelerator", "Clutch", "Handbrake" };

void CalibrationManager::begin() {
  storage_.begin();
  storage_.loadCalibration(minVals_, maxVals_);
  unsigned long now = millis();
  for (int i = 0; i < PEDAL_COUNT; ++i) {
    restAccumulator_[i] = 0.0f;
    restSampleCount_[i] = 0;
    restWindowStartMs_[i] = now;
    restCaptured_[i] = false;
  }
  // initial mode flag
  modeChanged_ = false;
  pedalCycled_ = false;
  rangeChanged_ = false;
}

void CalibrationManager::update(bool sh1,
                                bool sh2,
                                float rawBrake,
                                float rawAccel,
                                float rawClutch,
                                float rawHandbrake) {
  // Handle long-press to toggle calibration mode
  if (sh1) {
    if (!holdActive_) {
      holdActive_ = true;
      holdStartMs_ = millis();
    } else if (millis() - holdStartMs_ >= HOLD_MS) {
      holdActive_ = false;
      toggleMode();
    }
  } else {
    holdActive_ = false;
  }

  if (!calibMode_) return;

  // Pick the raw value for the selected pedal
  float raw = (selected_ == PEDAL_BRAKE)    ? rawBrake
              : (selected_ == PEDAL_ACCEL)  ? rawAccel
              : (selected_ == PEDAL_CLUTCH) ? rawClutch
                                            : rawHandbrake;

  rawBuffer_[selected_] = raw;
  unsigned long now = millis();

  // Accumulate for rest-based min
  if (!restCaptured_[selected_]) {
    restAccumulator_[selected_] += raw;
    restSampleCount_[selected_]++;
    if (now - restWindowStartMs_[selected_] >= REST_WINDOW_MS) {
      float avg = restAccumulator_[selected_] / restSampleCount_[selected_];
      minVals_[selected_] = avg;
      restCaptured_[selected_] = true;
      rangeChanged_ = true;
      lastRaw_ = raw;
      lastMin_ = avg;
      lastMax_ = maxVals_[selected_];
    }
  }

  // Detect new max
  if (raw > maxVals_[selected_]) {
    maxVals_[selected_] = raw;
    rangeChanged_ = true;
    lastRaw_ = raw;
    lastMin_ = minVals_[selected_];
    lastMax_ = raw;
  }

  // Cycle pedal on SHIFTER2 rising edge
  static bool prevSh2 = false;
  if (sh2 && !prevSh2) {
    cyclePedal();
  }
  prevSh2 = sh2;
}

bool CalibrationManager::inCalibration() const {
  return calibMode_;
}

bool CalibrationManager::hasModeChanged() const {
  return modeChanged_;
}

void CalibrationManager::clearModeChanged() {
  modeChanged_ = false;
}

bool CalibrationManager::hasPedalCycled() const {
  return pedalCycled_;
}

void CalibrationManager::clearPedalCycled() {
  pedalCycled_ = false;
}

bool CalibrationManager::hasRangeChanged() const {
  return rangeChanged_;
}

void CalibrationManager::clearRangeChanged() {
  rangeChanged_ = false;
}

float CalibrationManager::lastRaw() const {
  return lastRaw_;
}

float CalibrationManager::lastMin() const {
  return lastMin_;
}

float CalibrationManager::lastMax() const {
  return lastMax_;
}

float CalibrationManager::getCurrentRaw() const {
  return rawBuffer_[selected_];
}

float CalibrationManager::getMin(Pedal p) const {
  if (calibMode_) {
    // in calibration: don’t expose min until you’ve captured it
    return restCaptured_[p] ? minVals_[p] : 0.0f;
  } else {
    // in normal mode: always trust the stored value
    return minVals_[p];
  }
}

float CalibrationManager::getMax(Pedal p) const {
  return maxVals_[p] >= 0.0f ? maxVals_[p] : 4095.0f;
}

Pedal CalibrationManager::getSelectedPedal() const {
  return selected_;
}

void CalibrationManager::toggleMode() {
  calibMode_ = !calibMode_;
  modeChanged_ = true;
  if (calibMode_) {
    // entering: reset all data
    selected_ = PEDAL_BRAKE;
    unsigned long now = millis();
    for (int i = 0; i < PEDAL_COUNT; ++i) {
      minVals_[i] = 1e6f;
      maxVals_[i] = -1e6f;
      restAccumulator_[i] = 0.0f;
      restSampleCount_[i] = 0;
      restWindowStartMs_[i] = now;
      restCaptured_[i] = false;
    }
  } else {
    // exiting: save ranges
    storage_.saveCalibration(minVals_, maxVals_);
  }
}

void CalibrationManager::cyclePedal() {
  selected_ = Pedal((selected_ + 1) % PEDAL_COUNT);
  pedalCycled_ = true;
  // reset this pedal's rest-average
  restAccumulator_[selected_] = 0.0f;
  restSampleCount_[selected_] = 0;
  restWindowStartMs_[selected_] = millis();
  restCaptured_[selected_] = false;
}