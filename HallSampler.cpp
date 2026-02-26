// HallSampler.cpp
#include "HallSampler.h"
#include "Main.h"

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

namespace
{
  QueueHandle_t queueHandle = nullptr; // holds one packed sample
  volatile float lastAccel = 0;        // most-recent values
  volatile float lastClutch = 0;
}

// helper: dummy + real read to kill charge-sharing artefact
static inline uint16_t readAndFlush(uint8_t pin)
{
  (void)analogRead(pin);
  return analogRead(pin) & 0x0FFF;
}

// high-rate task pinned to core 1
static void hallSamplerTask(void *)
{
  // Tick period 1 ms → 1 kHz loop
  const TickType_t tick = pdMS_TO_TICKS(1);
  TickType_t next = xTaskGetTickCount();

  uint32_t packed;
  for (;;)
  {
    vTaskDelayUntil(&next, tick);

    packed = (readAndFlush(CLUTCH_PIN_2) << 16) | readAndFlush(ACCEL_PIN_2);
    xQueueOverwrite(queueHandle, &packed);

    lastAccel = float(packed & 0x0FFF);
    lastClutch = float(packed >> 16);
  }
}

void HallSampler::begin(uint32_t sampleHz, BaseType_t core)
{
  if (queueHandle)
    return; // already running

  pinMode(ACCEL_PIN_2, INPUT_PULLDOWN);
  pinMode(CLUTCH_PIN_2, INPUT_PULLDOWN);

  queueHandle = xQueueCreate(1, sizeof(uint32_t));

  // pin the task to core 1 (USB lives on core 0)
  xTaskCreatePinnedToCore(
      hallSamplerTask,
      "hallSampler",
      2048,
      nullptr,
      22,
      nullptr,
      1 // core 1
  );
}

float HallSampler::getAccelRaw()
{
  return lastAccel;
}
float HallSampler::getClutchRaw()
{
  return lastClutch;
}
