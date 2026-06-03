#include "camera_telemetry.h"

#include <WebServer.h>
#include <WiFi.h>
#include <esp_timer.h>

#include "ota_telnet.h"
#include "pid.h"
#include "quad_functions.h"
#include "system_diag.h"

namespace {

constexpr TickType_t kDefaultPeriod = pdMS_TO_TICKS(5);

int clampInt(int value, int minValue, int maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

int readTractionPwmSigned() {
  QuadDriveRuntimeSnapshot driveSnapshot{};
  if (!quadDriveGetRuntimeSnapshot(driveSnapshot) || !driveSnapshot.valid) {
    return 0;
  }
  return clampInt(driveSnapshot.tractionPwmSigned, -255, 255);
}

int readDirectionCommandDeg() {
  PidRuntimeSnapshot pidSnapshot{};
  if (!pidGetRuntimeSnapshot(pidSnapshot) || !pidSnapshot.valid) {
    return 90;
  }
  const int steeringCommand = clampInt(pidSnapshot.steeringCommand, -100, 100);
  return ((steeringCommand + 100) * 180 + 100) / 200;
}

int readBrakeBinary() {
  QuadDriveRuntimeSnapshot driveSnapshot{};
  if (!quadDriveGetRuntimeSnapshot(driveSnapshot) || !driveSnapshot.valid) {
    return 0;
  }
  return driveSnapshot.appliedBrakePercent > 0u ? 1 : 0;
}

String buildTelemetryPayload() {
  String payload;
  payload.reserve(24);
  payload += readTractionPwmSigned();
  payload += '_';
  payload += readDirectionCommandDeg();
  payload += '_';
  payload += readBrakeBinary();
  return payload;
}

void maybeLogStarted(const CameraTelemetryConfig& cfg) {
  if (!cfg.log) {
    return;
  }
  String msg;
  msg.reserve(64);
  msg += "[CAM][HTTP] Servidor iniciado en puerto ";
  msg += cfg.port;
  broadcastIf(true, msg);
}

}  // namespace

void taskCameraTelemetryServer(void* parameter) {
  const CameraTelemetryConfig* cfg = static_cast<const CameraTelemetryConfig*>(parameter);
  if (cfg == nullptr || cfg->port == 0) {
    broadcastIf(true, "[CAM][HTTP] Configuracion invalida, abortando tarea");
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->period > 0) ? cfg->period : kDefaultPeriod;
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  WebServer server(cfg->port);

  server.on("/telemetria", HTTP_GET, [&server]() {
    server.send(200, "text/plain", buildTelemetryPayload());
  });
  server.onNotFound([&server]() {
    server.send(404, "text/plain", "not found");
  });
  server.begin();
  maybeLogStarted(*cfg);

  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastIterationStartUs = esp_timer_get_time();

  while (true) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;

    server.handleClient();

    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kCameraTelemetry, cycleUs, expectedPeriodUs, overrun, false);
    vTaskDelayUntil(&lastWake, period);
  }
}
