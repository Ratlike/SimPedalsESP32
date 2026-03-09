// CalibrationManager.cpp
#include "CalibrationManager.h"
#include <Arduino.h>

// Auto-zero: stability-based idle recalibration
static const float    AUTOZERO_VAR_ALPHA   = 0.01f;
static const float    AUTOZERO_MEAN_ALPHA  = 0.01f;
static const float    AUTOZERO_VAR_THRESH  = 0.5f;
static const float    AUTOZERO_RANGE_FRAC  = 0.20f;
static const uint32_t AUTOZERO_IDLE_MS     = 5000;
static const uint32_t AUTOZERO_WINDOW_MS   = 2000;
static const uint32_t AUTOZERO_COOLDOWN_MS = 30000;

void CalibrationManager::begin() {
  storage_.begin();
  storage_.loadCalibration(minVals_, maxVals_);
  resetAutoZero();
}

void CalibrationManager::update(float rawBrake,
                                float rawAccel,
                                float rawClutch,
                                float rawHandbrake) {
  rawBuffer_[PEDAL_BRAKE]     = rawBrake;
  rawBuffer_[PEDAL_ACCEL]     = rawAccel;
  rawBuffer_[PEDAL_CLUTCH]    = rawClutch;
  rawBuffer_[PEDAL_HANDBRAKE] = rawHandbrake;

  if (!calibMode_ && autoZeroEnabled_) {
    autoZeroUpdate(rawBuffer_);
  }
}

bool CalibrationManager::inCalibration() const {
  return calibMode_;
}

void CalibrationManager::enterCalibration() {
  calibMode_ = true;
  resetAutoZero();
}

void CalibrationManager::exitCalibration() {
  storage_.saveCalibration(minVals_, maxVals_);
  calibMode_ = false;
}

void CalibrationManager::setMin(Pedal p, float value) {
  if (p < PEDAL_COUNT) minVals_[p] = value;
}

void CalibrationManager::setMax(Pedal p, float value) {
  if (p < PEDAL_COUNT) maxVals_[p] = value;
}

void CalibrationManager::freezeMin(Pedal p) {
  if (p < PEDAL_COUNT) minVals_[p] = rawBuffer_[p];
}

void CalibrationManager::freezeMax(Pedal p) {
  if (p < PEDAL_COUNT) maxVals_[p] = rawBuffer_[p];
}

float CalibrationManager::getMin(Pedal p) const {
  return minVals_[p];
}

float CalibrationManager::getMax(Pedal p) const {
  return maxVals_[p] >= 0.0f ? maxVals_[p] : 4095.0f;
}

void CalibrationManager::setAutoZeroEnabled(bool enabled) {
  autoZeroEnabled_ = enabled;
  if (!enabled) resetAutoZero();
}

bool CalibrationManager::isAutoZeroEnabled() const {
  return autoZeroEnabled_;
}

void CalibrationManager::resetAutoZero() {
  unsigned long now = millis();
  for (int i = 0; i < PEDAL_COUNT; ++i) {
    azMeanEma_[i]      = minVals_[i];
    azVarEma_[i]       = 0.0f;
    azStableStartMs_[i] = now;
    azStable_[i]       = false;
    azAccumulating_[i] = false;
    azAccumulator_[i]  = 0.0f;
    azSampleCount_[i]  = 0;
    azWindowStartMs_[i] = now;
    azLastRecalMs_[i]  = now;
  }
}

void CalibrationManager::autoZeroUpdate(const float raws[]) {
  unsigned long now = millis();

  for (int i = 0; i < PEDAL_COUNT; ++i) {
    float raw  = raws[i];
    float span = maxVals_[i] - minVals_[i];

    azMeanEma_[i] += AUTOZERO_MEAN_ALPHA * (raw - azMeanEma_[i]);
    float diff = raw - azMeanEma_[i];
    azVarEma_[i] += AUTOZERO_VAR_ALPHA * (diff * diff - azVarEma_[i]);

    bool isStable  = azVarEma_[i] < AUTOZERO_VAR_THRESH;
    bool isLowRange = (span > 0.01f) &&
                      (raw < minVals_[i] + AUTOZERO_RANGE_FRAC * span);
    bool cooldownOk = (now - azLastRecalMs_[i]) >= AUTOZERO_COOLDOWN_MS;

    if (!isStable || !isLowRange || !cooldownOk) {
      azStable_[i] = false;
      azAccumulating_[i] = false;
      continue;
    }

    if (!azStable_[i]) {
      azStable_[i] = true;
      azStableStartMs_[i] = now;
      continue;
    }

    if (!azAccumulating_[i]) {
      if (now - azStableStartMs_[i] >= AUTOZERO_IDLE_MS) {
        azAccumulating_[i] = true;
        azAccumulator_[i]  = 0.0f;
        azSampleCount_[i]  = 0;
        azWindowStartMs_[i] = now;
      }
      continue;
    }

    azAccumulator_[i] += raw;
    azSampleCount_[i]++;

    if (now - azWindowStartMs_[i] >= AUTOZERO_WINDOW_MS) {
      float newMin = azAccumulator_[i] / azSampleCount_[i];
      minVals_[i] = newMin;
      azLastRecalMs_[i]  = now;
      azStable_[i]       = false;
      azAccumulating_[i] = false;
    }
  }
}
