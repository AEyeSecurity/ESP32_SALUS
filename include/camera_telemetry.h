#ifndef CAMERA_TELEMETRY_H
#define CAMERA_TELEMETRY_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct CameraTelemetryConfig {
  uint16_t port;
  TickType_t period;
  bool log;
};

void taskCameraTelemetryServer(void* parameter);

#endif  // CAMERA_TELEMETRY_H
