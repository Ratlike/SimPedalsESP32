/*
 *  PedalsESP32.ino – load-cell-only pedal firmware
 *  Hardware: ESP32-S3, ADS1256 load-cell amplifier, no servo
 *
 *  Fixes:
 *    • Throttles calibration prints so the USB-CDC transmit buffer never
 *      overflows and blocks further output
 *    • Uses a conservative baud rate that is known to be reliable
 */

#include <Arduino.h>
#include <Bounce2.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include "Main.h"
#include "LoadCell.h"
#include "SignalFilter.h"
#include "Controller.h"
#include "Storage.h"
#include "CalibrationManager.h"
#include "HandbrakeSensor.h"
#include "HallSampler.h"

// ---------------------------------------------------------------------------
// User-adjustable constants
// ---------------------------------------------------------------------------
constexpr float RATED_CAPACITY_KG = 200.0f; // load-cell full-scale rating
constexpr uint16_t MAX_GAME_OUTPUT = 10000; // matches original joystick scaling
constexpr uint8_t KF_PROCESS_NOISE_Q = 1;   // lower → more smoothing
constexpr uint32_t SERIAL_BAUD = 115200;    // stable for USB-CDC on ESP32-S3
constexpr uint32_t PRINT_INTERVAL_MS = 50;  // max one calibration line every 50 ms
constexpr float ACCEL_CLUTCH_ALPHA = 0.30f; // unchanged

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------
LoadCell_ADS1256 loadcell;
KalmanFilter *brakeKalman = nullptr;

Bounce debouncer1;
Bounce debouncer2;

CalibrationManager calib;

HandbrakeSensor handbrake(HANDBRAKE_DOUT, HANDBRAKE_SCK);

int lastButtonState[2] = {0, 0};

static const char *pedalNames[] = {"Brake", "Accelerator", "Clutch", "Handbrake"};

static uint32_t lastHidMs = 0;
static int32_t lastBrakeVal = -1;
static int32_t lastAccelVal = -1;
static int32_t lastClutchVal = -1;
static int32_t lastHandbrakeVal = -1;
static bool lastBtn0 = false;
static bool lastBtn1 = false;

static float accelEma = 0.0f;
static float clutchEma = 0.0f;

QueueHandle_t adcNotifyQ;           // 1-byte ping queue
volatile float g_lastBrakeKg = 0.0; // most-recent raw sample

void IRAM_ATTR adsDrdyIsr() // ISR
{
  uint8_t tok = 1;
  BaseType_t hpTaskWoken = pdFALSE;
  xQueueSendFromISR(adcNotifyQ, &tok, &hpTaskWoken);
  if (hpTaskWoken)
    portYIELD_FROM_ISR();
}

void adcSamplerTask(void *) // FreeRTOS task
{
  ADS1256 &adc = ADC(); // singleton from LoadCell.cpp
  uint8_t tok;
  for (;;)
  {
    if (xQueueReceive(adcNotifyQ, &tok, portMAX_DELAY) == pdTRUE)
      g_lastBrakeKg = adc.readCurrentChannel(); // one SPI read
  }
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(SERIAL_BAUD);

  unsigned long start = millis();
  while (!Serial && millis() - start < 3000)
  {
    delay(10);
  }
  Serial.println("Starting setup");

  // buttons
  pinMode(SHIFTER1_PIN, INPUT_PULLUP);
  debouncer1.attach(SHIFTER1_PIN);
  debouncer1.interval(10);

  pinMode(SHIFTER2_PIN, INPUT_PULLUP);
  debouncer2.attach(SHIFTER2_PIN);
  debouncer2.interval(10);

  HallSampler::begin();

  // --- Load-cell initialisation ---
  loadcell.setLoadcellRating(RATED_CAPACITY_KG);
  loadcell.setZeroPoint();
  loadcell.estimateVariance();
  brakeKalman = new KalmanFilter(loadcell.getVarianceEstimate());

  adcNotifyQ = xQueueCreate(4, sizeof(uint8_t));

  pinMode(PIN_DRDY, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_DRDY), adsDrdyIsr, FALLING);

  xTaskCreatePinnedToCore(adcSamplerTask, "adcTask",
                          4096, nullptr, 23, nullptr, 1);

  // HID
  SetupController();

  // calibration storage
  calib.begin();
  Serial.println("Setup done");
}

// ---------------------------------------------------------------------------
// Main loop
// ---------------------------------------------------------------------------
void loop()
{
  debouncer1.update();
  debouncer2.update();

  bool sh1 = !digitalRead(SHIFTER1_PIN);
  bool sh2 = !digitalRead(SHIFTER2_PIN);

  // read raw inputs
  float rawKg = loadcell.getReadingKg();
  float accelRaw = HallSampler::getAccelRaw();
  float clutchRaw = HallSampler::getClutchRaw();
  static long lastHandbrake = 0L;

  // then replace your current read with:
  if (handbrake.isReady())
  {
    lastHandbrake = handbrake.readRaw();
  }
  long rawHandbrake = lastHandbrake;

  // update calibration manager
  calib.update(sh1, sh2, rawKg, float(accelRaw), float(clutchRaw), float(rawHandbrake));

  // Print mode-toggle events
  if (calib.hasModeChanged())
  {
    if (calib.inCalibration())
    {
      Serial.println("Enter calibration mode");
    }
    else
    {
      Serial.println("Exit calibration mode — saved ranges:");
      for (int i = 0; i < PEDAL_COUNT; ++i)
      {
        Serial.print(pedalNames[i]);
        Serial.print("  min=");
        Serial.print(calib.getMin(Pedal(i)), 2);
        Serial.print("  max=");
        Serial.println(calib.getMax(Pedal(i)), 2);
      }
    }
    calib.clearModeChanged();
  }

  // Print pedal-cycle events
  if (calib.hasPedalCycled())
  {
    Pedal p = calib.getSelectedPedal();
    Serial.print("Now calibrating ");
    Serial.println(pedalNames[p]);
    calib.clearPedalCycled();
  }

  // Print rest-min / max events (throttled)
  if (calib.inCalibration())
  {
    // if (calib.inCalibration() && calib.hasRangeChanged()) {
    static uint32_t lastPrint = 0;
    uint32_t now = millis();
    if (now - lastPrint >= PRINT_INTERVAL_MS && Serial.availableForWrite() >= 32)
    {
      Pedal p = calib.getSelectedPedal();
      Serial.print(pedalNames[p]);
      Serial.print("  raw=");
      Serial.print(calib.getCurrentRaw(), 2);
      Serial.print("  min=");
      Serial.print(calib.lastMin(), 2);
      Serial.print("  max=");
      Serial.println(calib.lastMax(), 2);
      lastPrint = now;

      calib.clearRangeChanged();
    }
  }

  if (!calib.inCalibration())
  {
    // read and filter
    float brakeFiltered = brakeKalman->filteredValue(rawKg, 0, KF_PROCESS_NOISE_Q);
    int32_t brakeValue = NormalizeControllerOutputValue(
        PEDAL_BRAKE, brakeFiltered,
        calib.getMin(PEDAL_BRAKE),
        calib.getMax(PEDAL_BRAKE),
        MAX_GAME_OUTPUT);
    SetBrake(brakeValue);

    accelEma += ACCEL_CLUTCH_ALPHA * (accelRaw - accelEma);
    int32_t accelValue = NormalizeControllerOutputValue(
        PEDAL_ACCEL, accelEma,
        calib.getMin(PEDAL_ACCEL),
        calib.getMax(PEDAL_ACCEL),
        MAX_GAME_OUTPUT);
    SetAccelerator(accelValue);

    clutchEma += ACCEL_CLUTCH_ALPHA * (clutchRaw - clutchEma);
    int32_t clutchValue = NormalizeControllerOutputValue(
        PEDAL_CLUTCH, clutchEma,
        calib.getMin(PEDAL_CLUTCH),
        calib.getMax(PEDAL_CLUTCH),
        MAX_GAME_OUTPUT);
    SetClutch(clutchValue);

    int32_t handbrakeValue = NormalizeControllerOutputValue(
        PEDAL_HANDBRAKE, float(rawHandbrake),
        calib.getMin(PEDAL_HANDBRAKE),
        calib.getMax(PEDAL_HANDBRAKE),
        MAX_GAME_OUTPUT);
    SetHandbrake(handbrakeValue);

    // buttons
    bool b0 = sh1;
    bool b1 = sh2;
    if (b0 != lastButtonState[0])
    {
      SetButton(0, b0);
      lastButtonState[0] = b0;
    }
    if (b1 != lastButtonState[1])
    {
      SetButton(1, b1);
      lastButtonState[1] = b1;
    }

    // throttle USB reports and only on change of any control
    uint32_t now = millis();
    bool axisChanged = (brakeValue != lastBrakeVal ||
                        accelValue != lastAccelVal ||
                        clutchValue != lastClutchVal ||
                        handbrakeValue != lastHandbrakeVal);
    bool btnChanged = (b0 != lastBtn0) || (b1 != lastBtn1);

    if ((now - lastHidMs) >= 4 && (axisChanged || btnChanged))
    {
      sendState();

      lastHidMs = now;
      lastBrakeVal = brakeValue;
      lastAccelVal = accelValue;
      lastClutchVal = clutchValue;
      lastHandbrakeVal = handbrakeValue;
      lastBtn0 = b0;
      lastBtn1 = b1;
    }
  }

  yield();
  delay(1);
}
