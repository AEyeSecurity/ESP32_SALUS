#include "quad_functions.h"

#include <freertos/task.h>

#include "fs_ia6.h"
#include "ota_telnet.h"

namespace {

QuadThrottleConfig g_config{};
uint32_t g_maxDuty = 255;
bool g_initialized = false;
bool g_forwardState = true;
bool g_waitingDirection = false;
uint32_t g_directionReadyMs = 0;
int g_lastDuty = 0;

uint32_t computeMaxDuty(uint8_t resolutionBits) {
  if (resolutionBits == 0) {
    return 255u;
  }
  if (resolutionBits >= 16) {
    return 65535u;
  }
  return (1u << resolutionBits) - 1u;
}

int clampDuty(int duty) {
  if (duty < 0) {
    return 0;
  }
  if (duty > static_cast<int>(g_maxDuty)) {
    return static_cast<int>(g_maxDuty);
  }
  return duty;
}

bool hasDirectionPin() {
  return g_config.directionPin != kQuadNoGpio;
}

void applyDirectionLevel(bool forward) {
  if (!hasDirectionPin()) {
    return;
  }
  const bool levelHigh = forward ? g_config.forwardLevelHigh : !g_config.forwardLevelHigh;
  digitalWrite(g_config.directionPin, levelHigh ? HIGH : LOW);
}

bool directionDelayExpired(uint32_t nowMs) {
  if (!g_waitingDirection) {
    return true;
  }
  if (static_cast<int32_t>(nowMs - g_directionReadyMs) >= 0) {
    g_waitingDirection = false;
    return true;
  }
  return false;
}

void logDirectionChange(bool forward) {
  if (!g_config.logDirectionChanges) {
    return;
  }
  String msg = String("[THROTTLE] Direccion: ") + (forward ? "adelante" : "reversa");
  broadcastIf(true, msg);
}

}  // namespace

void initQuadThrottle(const QuadThrottleConfig& config) {
  g_config = config;
  g_maxDuty = computeMaxDuty(config.pwmResolutionBits);
  if (g_config.pwmMinDuty > g_config.pwmMaxDuty) {
    const int tmp = g_config.pwmMinDuty;
    g_config.pwmMinDuty = g_config.pwmMaxDuty;
    g_config.pwmMaxDuty = tmp;
  }
  g_config.pwmMinDuty = clampDuty(g_config.pwmMinDuty);
  g_config.pwmMaxDuty = clampDuty(g_config.pwmMaxDuty);
  g_lastDuty = g_config.pwmMinDuty;
  g_forwardState = true;
  g_waitingDirection = false;
  g_directionReadyMs = millis();

  if (hasDirectionPin()) {
    pinMode(g_config.directionPin, OUTPUT);
    applyDirectionLevel(true);
  }

  ledcSetup(g_config.ledcChannel, g_config.pwmFrequencyHz, g_config.pwmResolutionBits);
  ledcAttachPin(g_config.pwmPin, g_config.ledcChannel);
  ledcWrite(g_config.ledcChannel, g_lastDuty);

  g_initialized = true;
}

int quadThrottleUpdate(int rcValue) {
  if (!g_initialized) {
    return 0;
  }

  if (rcValue > 100) {
    rcValue = 100;
  } else if (rcValue < -100) {
    rcValue = -100;
  }

  int requestedDuty = g_config.pwmMinDuty;
  bool requestedForward = true;
  const int deadZone = g_config.deadZone;
  if (rcValue > deadZone) {
    requestedDuty = map(rcValue, deadZone, 100, g_config.pwmMinDuty, g_config.pwmMaxDuty);
    requestedForward = true;
  } else if (rcValue < -deadZone) {
    const int magnitude = (rcValue < 0) ? -rcValue : rcValue;
    requestedDuty = map(magnitude, deadZone, 100, g_config.pwmMinDuty, g_config.pwmMaxDuty);
    requestedForward = false;
  } else {
    requestedForward = true;
    requestedDuty = g_config.pwmMinDuty;
  }

  requestedDuty = clampDuty(requestedDuty);

  const uint32_t nowMs = millis();
  if (hasDirectionPin() && requestedForward != g_forwardState) {
    applyDirectionLevel(requestedForward);
    g_forwardState = requestedForward;
    g_directionReadyMs = nowMs + g_config.directionChangeDelayMs;
    g_waitingDirection = g_config.directionChangeDelayMs > 0;
    logDirectionChange(requestedForward);
  }

  if (!directionDelayExpired(nowMs)) {
    requestedDuty = g_config.pwmMinDuty;
  }

  requestedDuty = clampDuty(requestedDuty);
  ledcWrite(g_config.ledcChannel, requestedDuty);
  g_lastDuty = requestedDuty;

  return g_lastDuty;
}

void quadThrottleStop() {
  if (!g_initialized) {
    return;
  }
  const int duty = clampDuty(g_config.pwmMinDuty);
  ledcWrite(g_config.ledcChannel, duty);
  g_lastDuty = duty;
}

void taskQuadThrottleControl(void* parameter) {
  const QuadThrottleTaskConfig* cfg = static_cast<const QuadThrottleTaskConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[THROTTLE] Configuracion invalida, deteniendo tarea");
    vTaskDelete(nullptr);
    return;
  }

  if (cfg->autoInitHardware) {
    initQuadThrottle(cfg->throttle);
  }

  TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(50);
  TickType_t lastWake = xTaskGetTickCount();
  int lastRcValue = 9999;
  int lastDutyReported = -1;
  bool lastDirReported = true;

  for (;;) {
    const int rcValue = readChannel(cfg->rcInputPin, -100, 100, 0);
    const int duty = quadThrottleUpdate(rcValue);

    if (cfg->log && (rcValue != lastRcValue || duty != lastDutyReported || g_forwardState != lastDirReported)) {
      String msg;
      msg.reserve(96);
      msg += "[THROTTLE] rc=";
      msg += rcValue;
      msg += " duty=";
      msg += duty;
      msg += "/";
      msg += static_cast<int>(g_maxDuty);
      msg += " dir=";
      msg += g_forwardState ? "adelante" : "reversa";
      msg += g_waitingDirection ? " (esperando cambio)" : "";
      broadcastIf(true, msg);
      lastRcValue = rcValue;
      lastDutyReported = duty;
      lastDirReported = g_forwardState;
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
