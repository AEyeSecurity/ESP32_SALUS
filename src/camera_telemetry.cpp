#include "camera_telemetry.h"

#include <HTTPClient.h>
#include <WiFi.h>
#include <esp_timer.h>

#include <math.h>

#include "ota_telnet.h"
#include "pid.h"
#include "quad_functions.h"
#include "steering_calibration.h"
#include "system_diag.h"

namespace {

constexpr TickType_t kDefaultPeriod = pdMS_TO_TICKS(1000);
constexpr uint16_t kDefaultHttpTimeoutMs = 200;
constexpr int16_t kSteerOutNotAvailable = INT16_MIN;
constexpr int kHttpOkMin = 200;
constexpr int kHttpOkMax = 299;

int16_t clampCenteredCentiDeg(float centeredDeg) {
  if (!isfinite(centeredDeg)) {
    return kSteerOutNotAvailable;
  }
  if (centeredDeg < -180.0f) {
    centeredDeg = -180.0f;
  } else if (centeredDeg > 180.0f) {
    centeredDeg = 180.0f;
  }

  long centeredCenti = lroundf(centeredDeg * 100.0f);
  if (centeredCenti < -18000L) {
    centeredCenti = -18000L;
  } else if (centeredCenti > 18000L) {
    centeredCenti = 18000L;
  }
  return static_cast<int16_t>(centeredCenti);
}

int16_t readStemOutCentiDeg() {
  PidRuntimeSnapshot pidSnapshot{};
  if (!pidGetRuntimeSnapshot(pidSnapshot) || !pidSnapshot.valid || !pidSnapshot.sensorValid ||
      !isfinite(pidSnapshot.measuredDeg)) {
    return kSteerOutNotAvailable;
  }

  const SteeringCalibrationData calibration = steeringCalibrationSnapshot();
  const float centerDeg = calibration.initialized ? calibration.adjustedCenterDeg : 0.0f;
  return clampCenteredCentiDeg(wrapAngleDegrees(pidSnapshot.measuredDeg - centerDeg));
}

int readStemInCommand() {
  PidRuntimeSnapshot pidSnapshot{};
  if (!pidGetRuntimeSnapshot(pidSnapshot) || !pidSnapshot.valid) {
    return 0;
  }
  return pidSnapshot.steeringCommand;
}

uint8_t readOzhBrakePercent() {
  QuadDriveRuntimeSnapshot driveSnapshot{};
  if (!quadDriveGetRuntimeSnapshot(driveSnapshot) || !driveSnapshot.valid) {
    return 0;
  }
  if (driveSnapshot.appliedBrakePercent > 100u) {
    return 100u;
  }
  return driveSnapshot.appliedBrakePercent;
}

String buildPostBody() {
  String body;
  body.reserve(48);
  body += "data=";
  body += readStemInCommand();
  body += '_';
  body += readStemOutCentiDeg();
  body += '_';
  body += readOzhBrakePercent();
  return body;
}

bool postTelemetry(const CameraTelemetryConfig& cfg, int& httpCodeOut) {
  httpCodeOut = 0;
  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  WiFiClient client;
  client.setTimeout(cfg.httpTimeoutMs);

  HTTPClient http;
  http.setTimeout(cfg.httpTimeoutMs);
  http.setReuse(false);

  if (!http.begin(client, cfg.url)) {
    return false;
  }

  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  const String body = buildPostBody();
  httpCodeOut = http.POST(body);
  http.end();

  return httpCodeOut >= kHttpOkMin && httpCodeOut <= kHttpOkMax;
}

void maybeLogResult(const CameraTelemetryConfig& cfg, bool ok, int httpCode, TickType_t nowTick) {
  if (!cfg.log) {
    return;
  }

  static TickType_t s_lastLogTick = 0;
  const TickType_t minInterval = pdMS_TO_TICKS(2000);
  if (s_lastLogTick != 0 && (nowTick - s_lastLogTick) < minInterval) {
    return;
  }

  String msg;
  msg.reserve(96);
  msg += "[CAM][TEL] ";
  msg += ok ? "POST ok" : "POST fallo";
  msg += " code=";
  msg += httpCode;
  broadcastIf(true, msg);
  s_lastLogTick = nowTick;
}

}  // namespace

bool cameraTelemetryUrlConfigured(const char* url) {
  return url != nullptr && url[0] != '\0';
}

void taskCameraTelemetry(void* parameter) {
  CameraTelemetryConfig* cfg = static_cast<CameraTelemetryConfig*>(parameter);
  if (cfg == nullptr || !cameraTelemetryUrlConfigured(cfg->url)) {
    broadcastIf(true, "[CAM][TEL] Configuracion invalida, abortando tarea");
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->period > 0) ? cfg->period : kDefaultPeriod;
  if (cfg->httpTimeoutMs == 0) {
    cfg->httpTimeoutMs = kDefaultHttpTimeoutMs;
  }

  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastIterationStartUs = esp_timer_get_time();

  while (true) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;

    int httpCode = 0;
    const bool ok = postTelemetry(*cfg, httpCode);
    maybeLogResult(*cfg, ok, httpCode, xTaskGetTickCount());

    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kCameraTelemetry, cycleUs, expectedPeriodUs, overrun, false);
    vTaskDelayUntil(&lastWake, period);
  }
}
