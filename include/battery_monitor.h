#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct BatteryMonitorConfig {
  uint8_t pin;
  uint8_t sampleCount;
  uint32_t dividerUpperOhm;
  uint32_t dividerLowerOhm;
  float calibrationGain;
  TickType_t period;
};

struct BatterySnapshot {
  bool driverReady;
  uint8_t pin;
  uint32_t adcPinMv;
  uint32_t batteryMv;
  TickType_t sampleTick;
  uint32_t sampleCount;
};

bool batteryMonitorInit(const BatteryMonitorConfig& config);
bool batteryMonitorGetSnapshot(BatterySnapshot& snapshot);
void taskBatteryMonitor(void* parameter);

#endif  // BATTERY_MONITOR_H
