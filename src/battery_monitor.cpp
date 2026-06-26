#include "battery_monitor.h"

#include <math.h>

#include <esp_timer.h>

#include "system_diag.h"

namespace {

BatteryMonitorConfig g_config{};
BatterySnapshot g_snapshot{};
portMUX_TYPE g_batteryMux = portMUX_INITIALIZER_UNLOCKED;
bool g_initialized = false;

uint32_t clampRoundToU32(float value) {
  if (!isfinite(value) || value <= 0.0f) {
    return 0;
  }
  if (value >= static_cast<float>(UINT32_MAX)) {
    return UINT32_MAX;
  }
  return static_cast<uint32_t>(lroundf(value));
}

bool readAveragedPinMilliVolts(const BatteryMonitorConfig& config, uint32_t& pinMvOut) {
  const uint8_t sampleCount = (config.sampleCount < 3) ? 3 : config.sampleCount;
  uint32_t samples[16] = {};
  const uint8_t boundedCount = (sampleCount > 16) ? 16 : sampleCount;

  (void)analogReadMilliVolts(config.pin);

  for (uint8_t i = 0; i < boundedCount; ++i) {
    const uint32_t readingMv = analogReadMilliVolts(config.pin);
    samples[i] = readingMv;
  }

  for (uint8_t i = 0; i + 1 < boundedCount; ++i) {
    for (uint8_t j = i + 1; j < boundedCount; ++j) {
      if (samples[j] < samples[i]) {
        const uint32_t tmp = samples[i];
        samples[i] = samples[j];
        samples[j] = tmp;
      }
    }
  }

  uint8_t trimCount = 1;
  if (boundedCount >= 12) {
    trimCount = 3;
  } else if (boundedCount >= 8) {
    trimCount = 2;
  }

  if ((trimCount * 2) >= boundedCount) {
    trimCount = 1;
  }

  uint64_t sum = 0;
  uint8_t usedCount = 0;
  for (uint8_t i = trimCount; i < (boundedCount - trimCount); ++i) {
    sum += samples[i];
    ++usedCount;
  }

  if (usedCount > 0) {
    pinMvOut = static_cast<uint32_t>(sum / static_cast<uint64_t>(usedCount));
    return true;
  }

  pinMvOut = static_cast<uint32_t>(sum / static_cast<uint64_t>(boundedCount));
  return true;
}

}  // namespace

bool batteryMonitorInit(const BatteryMonitorConfig& config) {
  if (config.pin == 0 || config.dividerLowerOhm == 0 || config.period == 0 ||
      !isfinite(config.calibrationGain) || config.calibrationGain <= 0.0f) {
    return false;
  }

  analogReadResolution(12);
  analogSetPinAttenuation(config.pin, ADC_11db);
  pinMode(config.pin, INPUT);

  portENTER_CRITICAL(&g_batteryMux);
  g_config = config;
  g_snapshot = {};
  g_snapshot.driverReady = true;
  g_snapshot.pin = config.pin;
  g_initialized = true;
  portEXIT_CRITICAL(&g_batteryMux);
  return true;
}

bool batteryMonitorGetSnapshot(BatterySnapshot& snapshot) {
  portENTER_CRITICAL(&g_batteryMux);
  snapshot = g_snapshot;
  portEXIT_CRITICAL(&g_batteryMux);
  return snapshot.driverReady;
}

void taskBatteryMonitor(void* parameter) {
  const BatteryMonitorConfig* cfg = static_cast<const BatteryMonitorConfig*>(parameter);
  if (cfg == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(5000);
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastIterationStartUs = esp_timer_get_time();

  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;

    uint32_t adcPinMv = 0;
    readAveragedPinMilliVolts(*cfg, adcPinMv);

    const float scale =
        static_cast<float>(cfg->dividerUpperOhm + cfg->dividerLowerOhm) /
        static_cast<float>(cfg->dividerLowerOhm);
    const uint32_t batteryMv =
        clampRoundToU32(static_cast<float>(adcPinMv) * scale * cfg->calibrationGain);

    portENTER_CRITICAL(&g_batteryMux);
    g_snapshot.driverReady = g_initialized;
    g_snapshot.pin = cfg->pin;
    g_snapshot.adcPinMv = adcPinMv;
    g_snapshot.batteryMv = batteryMv;
    g_snapshot.sampleTick = xTaskGetTickCount();
    g_snapshot.sampleCount += 1;
    portEXIT_CRITICAL(&g_batteryMux);

    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kBattery, cycleUs, expectedPeriodUs, overrun, false);
    vTaskDelayUntil(&lastWake, period);
  }
}
