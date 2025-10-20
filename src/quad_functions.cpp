#include "quad_functions.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

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

struct BrakeServoAngles {
  int servoA;
  int servoB;
};

BrakeServoAngles g_brakeCurrentAngles{0, 0};

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

void applyBrakeAngles(int angleServoADeg, int angleServoBDeg) {
  if (!g_brakeInitialized) {
    return;
  }
  const int clampedA = clampAngle(angleServoADeg);
  const int clampedB = clampAngle(angleServoBDeg);
  writeServoAngle(g_brakeConfig.ledcChannelA, clampedA);
  writeServoAngle(g_brakeConfig.ledcChannelB, clampedB);
  g_brakeCurrentAngles.servoA = clampedA;
  g_brakeCurrentAngles.servoB = clampedB;
}

int updateThrottleFilter(int rawValue) {
  if (rawValue > 100) {
    rawValue = 100;
  } else if (rawValue < -100) {
    rawValue = -100;
  }
  if (!g_filterInitialized) {
    g_filteredThrottleValue = rawValue;
    if (abs(rawValue) <= 15) {
      g_filterOffset = rawValue;
    } else {
      g_filterOffset = 0;
    }
    g_filterOffsetReady = true;
    g_filterInitialized = true;
  } else {
    g_filteredThrottleValue = (g_filteredThrottleValue * 2 + rawValue) / 3;
    if (abs(rawValue) < 15) {
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
  g_brakeConfig.releaseAngleServoADeg = clampAngle(g_brakeConfig.releaseAngleServoADeg);
  g_brakeConfig.brakeAngleServoADeg = clampAngle(g_brakeConfig.brakeAngleServoADeg);
  g_brakeConfig.releaseAngleServoBDeg = clampAngle(g_brakeConfig.releaseAngleServoBDeg);
  g_brakeConfig.brakeAngleServoBDeg = clampAngle(g_brakeConfig.brakeAngleServoBDeg);
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
  applyBrakeAngles(g_brakeConfig.releaseAngleServoADeg, g_brakeConfig.releaseAngleServoBDeg);
}

void quadBrakeUpdate(int rcValue) {
  if (!g_brakeInitialized) {
    return;
  }

  const bool brakeActive = rcValue < g_brakeConfig.activationThreshold;
  const int targetAngleA =
      brakeActive ? g_brakeConfig.brakeAngleServoADeg : g_brakeConfig.releaseAngleServoADeg;
  const int targetAngleB =
      brakeActive ? g_brakeConfig.brakeAngleServoBDeg : g_brakeConfig.releaseAngleServoBDeg;

  if (g_brakeCurrentAngles.servoA != targetAngleA || g_brakeCurrentAngles.servoB != targetAngleB) {
    applyBrakeAngles(targetAngleA, targetAngleB);
  }
}

void quadBrakeRelease() {
  if (!g_brakeInitialized) {
    return;
  }
  applyBrakeAngles(g_brakeConfig.releaseAngleServoADeg, g_brakeConfig.releaseAngleServoBDeg);
}

void taskQuadDriveControl(void* parameter) {
  const QuadDriveTaskConfig* cfg = static_cast<const QuadDriveTaskConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[DRIVE] Configuracion invalida, deteniendo tarea");
    vTaskDelete(nullptr);
    return;
  }

  if (cfg->autoInitHardware) {
    initQuadThrottle(cfg->throttle);
    initQuadBrake(cfg->brake);
  }

  const TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  TickType_t lastStaleLog = 0;
  TickType_t lastPerfLog = 0;
  int lastThrottleRcValue = 9999;
  int lastDutyReported = -1;
  int lastBrakeReportedA = -1;
  int lastBrakeReportedB = -1;

  TaskHandle_t self = xTaskGetCurrentTaskHandle();
  if (!rcRegisterConsumer(self)) {
    broadcastIf(true, "[DRIVE] No se pudo registrar la tarea para notificaciones RC");
  }

  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
    const uint32_t notificationCount = ulTaskNotifyTake(pdTRUE, period);
    (void)notificationCount;

    RcSharedState rcSnapshot{};
    const bool rcValid = rcGetStateCopy(rcSnapshot);
    const TickType_t sampleTick = xTaskGetTickCount();
    const TickType_t snapshotTick = rcSnapshot.lastUpdateTick;
    const bool snapshotFresh =
        rcValid && rcSnapshot.valid && snapshotTick != 0 && (sampleTick - snapshotTick) <= pdMS_TO_TICKS(50);
    const int rawThrottle = snapshotFresh ? rcSnapshot.throttle : 0;

    const int rcValue = updateThrottleFilter(rawThrottle);
    if (snapshotFresh) {
      g_lastThrottleUpdateTick = snapshotTick;
    }
    const int duty = quadThrottleUpdate(rcValue);

    const int brakeInput = throttleDataFresh(pdMS_TO_TICKS(60)) ? getFilteredThrottleValue() : 0;
    quadBrakeUpdate(brakeInput);

    if (cfg->log && (rcValue != lastThrottleRcValue || duty != lastDutyReported ||
                     g_brakeCurrentAngles.servoA != lastBrakeReportedA ||
                     g_brakeCurrentAngles.servoB != lastBrakeReportedB)) {
      String msg;
      msg.reserve(96);
      msg += "[DRIVE] rc=";
      msg += rcValue;
      msg += " duty=";
      msg += duty;
      msg += "/";
      msg += static_cast<int>(g_maxDuty);
      msg += " brakeA=";
      msg += g_brakeCurrentAngles.servoA;
      msg += "deg brakeB=";
      msg += g_brakeCurrentAngles.servoB;
      msg += "deg";
      if (snapshotFresh) {
        msg += " raw=";
        msg += rawThrottle;
      } else {
        msg += " raw=STALE";
      }
      msg += " ageMs=";
      msg += static_cast<int>((sampleTick - snapshotTick) * portTICK_PERIOD_MS);
      broadcastIf(true, msg);
      lastThrottleRcValue = rcValue;
      lastDutyReported = duty;
      lastBrakeReportedA = g_brakeCurrentAngles.servoA;
      lastBrakeReportedB = g_brakeCurrentAngles.servoB;
    }

    if (cfg->log && !snapshotFresh) {
      if ((sampleTick - lastStaleLog) >= pdMS_TO_TICKS(500)) {
        broadcastIf(true, "[DRIVE] sin datos frescos del RC (>50ms); usando 0");
        lastStaleLog = sampleTick;
      }
    } else {
      lastStaleLog = sampleTick;
    }

    const int64_t iterationDurationUs = esp_timer_get_time() - iterationStartUs;
    if (cfg->log && iterationDurationUs > 2000) {
      if ((sampleTick - lastPerfLog) >= pdMS_TO_TICKS(1000)) {
        String perfMsg = "[DRIVE] ciclo tardo ";
        perfMsg += iterationDurationUs / 1000.0f;
        perfMsg += "ms";
        broadcastIf(true, perfMsg);
        lastPerfLog = sampleTick;
      }
    }
  }
}
