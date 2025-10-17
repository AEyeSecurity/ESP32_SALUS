#include "quad_functions.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "fs_ia6.h"
#include "ota_telnet.h"

namespace {

QuadThrottleConfig g_config{};
uint32_t g_maxDuty = 255;
bool g_initialized = false;
int g_lastDuty = 0;

QuadBrakeConfig g_brakeConfig{};
uint32_t g_brakeMaxDuty = 0;
uint32_t g_brakePeriodUs = 20000;
bool g_brakeInitialized = false;
int g_brakeCurrentAngle = 0;

// Filtro simple con offset auto-calibrado para el acelerador
volatile int g_filteredThrottleValue = 0;
volatile int g_normalizedThrottleValue = 0;
bool g_filterInitialized = false;
int g_filterOffset = 0;
bool g_filterOffsetReady = false;
volatile TickType_t g_lastThrottleUpdateTick = 0;

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

int clampAngle(int angleDeg) {
  if (angleDeg < 0) {
    return 0;
  }
  if (angleDeg > 180) {
    return 180;
  }
  return angleDeg;
}

uint32_t angleToPulseUs(int angleDeg) {
  const int clamped = clampAngle(angleDeg);
  return 500u + static_cast<uint32_t>((2000u * clamped) / 180u);
}

uint32_t pulseToDuty(uint32_t pulseUs) {
  if (g_brakePeriodUs == 0 || g_brakeMaxDuty == 0) {
    return 0;
  }
  return (pulseUs * g_brakeMaxDuty) / g_brakePeriodUs;
}

void writeServoAngle(uint8_t channel, int angleDeg) {
  const uint32_t pulseUs = angleToPulseUs(angleDeg);
  const uint32_t duty = pulseToDuty(pulseUs);
  ledcWrite(channel, duty);
}

void applyBrakeAngle(int angleDeg) {
  if (!g_brakeInitialized) {
    return;
  }
  const int clamped = clampAngle(angleDeg);
  writeServoAngle(g_brakeConfig.ledcChannelA, clamped);
  writeServoAngle(g_brakeConfig.ledcChannelB, clamped);
  g_brakeCurrentAngle = clamped;
}

int updateThrottleFilter(int rawValue) {
  if (rawValue > 100) {
    rawValue = 100;
  } else if (rawValue < -100) {
    rawValue = -100;
  }
  if (!g_filterInitialized) {
    g_filteredThrottleValue = rawValue;
    g_filterOffset = rawValue;
    g_filterOffsetReady = true;
    g_filterInitialized = true;
  } else {
    g_filteredThrottleValue = (g_filteredThrottleValue * 2 + rawValue) / 3;
    const int deviation = g_filteredThrottleValue - g_filterOffset;
    if (abs(deviation) < 8) {
      g_filterOffset = (g_filterOffset * 3 + g_filteredThrottleValue) / 4;
    }
  }

  int normalized = g_filteredThrottleValue - g_filterOffset;
  if (normalized > 100) {
    normalized = 100;
  } else if (normalized < -100) {
    normalized = -100;
  }
  g_normalizedThrottleValue = normalized;
  return g_normalizedThrottleValue;
}

int getFilteredThrottleValue() {
  return g_normalizedThrottleValue;
}

bool filterHasSamples() {
  return g_filterInitialized && g_filterOffsetReady;
}

bool throttleDataFresh(TickType_t maxAgeTicks) {
  if (!filterHasSamples()) {
    return false;
  }
  TickType_t now = xTaskGetTickCount();
  return (now - g_lastThrottleUpdateTick) <= maxAgeTicks;
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
  } else if (rcValue < -100) {
    rcValue = -100;
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

  if (rcValue <= threshold) {
    duty = g_config.pwmMinDuty;
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
  const int duty = g_config.pwmMinDuty;
  ledcWrite(g_config.ledcChannel, duty);
  g_lastDuty = duty;
}

void initQuadBrake(const QuadBrakeConfig& config) {
  g_brakeConfig = config;
  g_brakeConfig.releaseAngleDeg = clampAngle(g_brakeConfig.releaseAngleDeg);
  g_brakeConfig.brakeAngleDeg = clampAngle(g_brakeConfig.brakeAngleDeg);
  if (g_brakeConfig.activationThreshold > 0) {
    g_brakeConfig.activationThreshold = -abs(g_brakeConfig.activationThreshold);
  }

  g_brakeMaxDuty = computeMaxDuty(g_brakeConfig.pwmResolutionBits);
  g_brakePeriodUs = (g_brakeConfig.pwmFrequencyHz > 0) ? (1000000u / g_brakeConfig.pwmFrequencyHz) : 20000u;

  ledcSetup(g_brakeConfig.ledcChannelA, g_brakeConfig.pwmFrequencyHz, g_brakeConfig.pwmResolutionBits);
  ledcSetup(g_brakeConfig.ledcChannelB, g_brakeConfig.pwmFrequencyHz, g_brakeConfig.pwmResolutionBits);

  ledcAttachPin(g_brakeConfig.servoPinA, g_brakeConfig.ledcChannelA);
  ledcAttachPin(g_brakeConfig.servoPinB, g_brakeConfig.ledcChannelB);

  g_brakeInitialized = true;
  applyBrakeAngle(g_brakeConfig.releaseAngleDeg);
}

void quadBrakeUpdate(int rcValue) {
  if (!g_brakeInitialized) {
    return;
  }

  if (rcValue < g_brakeConfig.activationThreshold) {
    if (g_brakeCurrentAngle != g_brakeConfig.brakeAngleDeg) {
      applyBrakeAngle(g_brakeConfig.brakeAngleDeg);
    }
  } else {
    if (g_brakeCurrentAngle != g_brakeConfig.releaseAngleDeg) {
      applyBrakeAngle(g_brakeConfig.releaseAngleDeg);
    }
  }
}

void quadBrakeRelease() {
  if (!g_brakeInitialized) {
    return;
  }
  applyBrakeAngle(g_brakeConfig.releaseAngleDeg);
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

  TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  TickType_t lastWake = xTaskGetTickCount();
  int lastRcValue = 9999;
  int lastDutyReported = -1;

  for (;;) {
    RcSharedState rcSnapshot{};
    const bool rcValid = rcGetStateCopy(rcSnapshot);
    const TickType_t sampleTick = xTaskGetTickCount();
    int rawValue = 0;
    if (rcValid && rcSnapshot.valid && (sampleTick - rcSnapshot.lastUpdateTick) <= pdMS_TO_TICKS(50)) {
      rawValue = rcSnapshot.throttle;
    }

    const int rcValue = updateThrottleFilter(rawValue);
    g_lastThrottleUpdateTick = (rcValid && rcSnapshot.valid) ? rcSnapshot.lastUpdateTick : sampleTick;
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

void taskQuadBrakeControl(void* parameter) {
  const QuadBrakeTaskConfig* cfg = static_cast<const QuadBrakeTaskConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[BRAKE] Configuracion invalida, deteniendo tarea");
    vTaskDelete(nullptr);
    return;
  }

  if (cfg->autoInitHardware) {
    initQuadBrake(cfg->brake);
  }

  TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  TickType_t lastWake = xTaskGetTickCount();
  int lastRcValue = 9999;
  int lastAngleReported = -1;

  for (;;) {
    int rcValue = 0;
    if (throttleDataFresh(pdMS_TO_TICKS(60))) {
      rcValue = getFilteredThrottleValue();
    }
    quadBrakeUpdate(rcValue);

    if (cfg->log && (rcValue != lastRcValue || g_brakeCurrentAngle != lastAngleReported)) {
      String msg;
      msg.reserve(64);
      msg += "[BRAKE] rc=";
      msg += rcValue;
      msg += " angle=";
      msg += g_brakeCurrentAngle;
      msg += "deg";
      broadcastIf(true, msg);
      lastRcValue = rcValue;
      lastAngleReported = g_brakeCurrentAngle;
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
