#include "quad_functions.h"

#include <freertos/task.h>

#include "fs_ia6.h"
#include "ota_telnet.h"

namespace {

QuadThrottleConfig g_config{};
uint32_t g_maxDuty = 255;
bool g_initialized = false;
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
  if (g_config.activationThreshold < 0) {
    g_config.activationThreshold = 0;
  } else if (g_config.activationThreshold > 100) {
    g_config.activationThreshold = 100;
  }
  g_lastDuty = g_config.pwmMinDuty;

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
  } else if (rcValue < 0) {
    rcValue = 0;
  }

  const int threshold = g_config.activationThreshold;
  int duty = g_config.pwmMinDuty;

  if (rcValue > threshold) {
    const int span = g_config.pwmMaxDuty - g_config.pwmMinDuty;
    const int range = 100 - threshold;
    if (range <= 0) {
      duty = g_config.pwmMaxDuty;
    } else {
      duty = g_config.pwmMinDuty + ((rcValue - threshold) * span) / range;
    }
  }

  duty = clampDuty(duty);
  ledcWrite(g_config.ledcChannel, duty);
  g_lastDuty = duty;

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

  for (;;) {
    const int rcValue = readChannel(cfg->rcInputPin, 0, 100, 0);
    const int duty = quadThrottleUpdate(rcValue);

    if (cfg->log && (rcValue != lastRcValue || duty != lastDutyReported)) {
      String msg;
      msg.reserve(64);
      msg += "[THROTTLE] rc=";
      msg += rcValue;
      msg += " duty=";
      msg += duty;
      msg += "/";
      msg += static_cast<int>(g_maxDuty);
      broadcastIf(true, msg);
      lastRcValue = rcValue;
      lastDutyReported = duty;
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
