#ifndef CAMERA_TELEMETRY_H
#define CAMERA_TELEMETRY_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct CameraTelemetryConfig {
  const char* url;
  TickType_t period;
  uint16_t httpTimeoutMs;
  bool log;
};

bool cameraTelemetryUrlConfigured(const char* url);
void taskCameraTelemetry(void* parameter);

#endif  // CAMERA_TELEMETRY_H
