#include "quad_functions.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include <math.h>

#include "fs_ia6.h"
#include "hall_speed.h"
#include "ota_telnet.h"
#include "pi_comms.h"

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
bool g_driveLogEnabled = false;
portMUX_TYPE g_driveLogMux = portMUX_INITIALIZER_UNLOCKED;

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

int interpolateAngle(int releaseAngleDeg, int brakeAngleDeg, uint8_t percent) {
  const int delta = brakeAngleDeg - releaseAngleDeg;
  const int interpolated =
      releaseAngleDeg + static_cast<int>((delta * static_cast<int>(percent)) / 100);
  return clampAngle(interpolated);
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

constexpr TickType_t kPiSnapshotFreshTicks = pdMS_TO_TICKS(120);
constexpr TickType_t kSpeedPidFeedbackGraceTicks = pdMS_TO_TICKS(1000);
constexpr TickType_t kDriveDtWarningCooldown = pdMS_TO_TICKS(1000);
constexpr TickType_t kDriveRuntimeWarningCooldown = pdMS_TO_TICKS(1000);

enum class SpeedControlSource : uint8_t {
  kNone = 0,
  kPiSpeedPid,
  kRcSpeedPid,
};

float mapAccelToSpeedTargetMps(int accelRaw) {
  if (accelRaw <= 0) {
    return 0.0f;
  }
  if (accelRaw > 100) {
    accelRaw = 100;
  }
  const float maxSpeed = speedPidGetMaxSpeedMps();
  return (static_cast<float>(accelRaw) * maxSpeed) / 100.0f;
}

float mapRcToSpeedTargetMps(int rcValue) {
  if (rcValue <= 0) {
    return 0.0f;
  }
  if (rcValue > 100) {
    rcValue = 100;
  }
  const float maxSpeed = speedPidGetMaxSpeedMps();
  return (static_cast<float>(rcValue) * maxSpeed) / 100.0f;
}

const char* speedControlSourceText(SpeedControlSource source) {
  switch (source) {
    case SpeedControlSource::kPiSpeedPid:
      return "PI";
    case SpeedControlSource::kRcSpeedPid:
      return "RC";
    case SpeedControlSource::kNone:
    default:
      return "NONE";
  }
}

uint8_t clampPercentFromFloat(float percent) {
  if (!isfinite(percent) || percent <= 0.0f) {
    return 0;
  }
  if (percent >= 100.0f) {
    return 100;
  }
  return static_cast<uint8_t>(percent + 0.5f);
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

  // Usar la magnitud del comando tanto para avance como reversa; el sentido se
  // resuelve fuera (relé) pero el PWM refleja la intensidad solicitada.
  const int magnitude = abs(rcValue);

  const int threshold = g_config.activationThreshold;
  int duty = g_config.pwmMinDuty;

  if (magnitude > threshold) {
    const int span = g_config.pwmMaxDuty - g_config.pwmMinDuty;
    const int range = 100 - threshold;
    if (range <= 0) {
      duty = g_config.pwmMaxDuty;
    } else {
      duty = g_config.pwmMinDuty + ((magnitude - threshold) * span) / range;
    }
  }

  if (magnitude <= threshold) {
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

void quadBrakeApplyPercent(uint8_t percent) {
  if (!g_brakeInitialized) {
    return;
  }
  if (percent > 100u) {
    percent = 100u;
  }
  const int targetAngleA =
      interpolateAngle(g_brakeConfig.releaseAngleServoADeg, g_brakeConfig.brakeAngleServoADeg, percent);
  const int targetAngleB =
      interpolateAngle(g_brakeConfig.releaseAngleServoBDeg, g_brakeConfig.brakeAngleServoBDeg, percent);
  if (g_brakeCurrentAngles.servoA == targetAngleA && g_brakeCurrentAngles.servoB == targetAngleB) {
    return;
  }
  applyBrakeAngles(targetAngleA, targetAngleB);
}

bool quadDriveSetLogEnabled(bool enabled) {
  portENTER_CRITICAL(&g_driveLogMux);
  g_driveLogEnabled = enabled;
  portEXIT_CRITICAL(&g_driveLogMux);
  return true;
}

bool quadDriveGetLogEnabled(bool& enabledOut) {
  portENTER_CRITICAL(&g_driveLogMux);
  enabledOut = g_driveLogEnabled;
  portEXIT_CRITICAL(&g_driveLogMux);
  return true;
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
  quadDriveSetLogEnabled(cfg->log);

  const TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  TickType_t lastStaleLog = 0;
  TickType_t lastPerfLog = 0;
  TickType_t lastDtWarningTick = 0;
  int lastThrottleCmdValue = 9999;
  int lastDutyReported = -1;
  int lastBrakeReportedA = -1;
  int lastBrakeReportedB = -1;
  TickType_t speedFeedbackMissingTick = 0;
  int64_t lastLoopUs = esp_timer_get_time();
  bool speedPidWasActive = false;
  float expectedPeriodSeconds = static_cast<float>(period * portTICK_PERIOD_MS) / 1000.0f;
  if (expectedPeriodSeconds <= 0.0f) {
    expectedPeriodSeconds = 0.001f;
  }
  const float dtOverrunThreshold = expectedPeriodSeconds + 0.010f;

  TaskHandle_t self = xTaskGetCurrentTaskHandle();
  if (!rcRegisterConsumer(self)) {
    broadcastIf(true, "[DRIVE] No se pudo registrar la tarea para notificaciones RC");
  }

  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
    const uint32_t notificationCount = ulTaskNotifyTake(pdTRUE, period);
    (void)notificationCount;
    const TickType_t sampleTick = xTaskGetTickCount();

    float dtSeconds = static_cast<float>(iterationStartUs - lastLoopUs) * 1e-6f;
    lastLoopUs = iterationStartUs;
    if (!isfinite(dtSeconds) || dtSeconds <= 0.0f || dtSeconds > 1.0f) {
      dtSeconds = expectedPeriodSeconds;
    }
    bool driveLogEnabled = false;
    quadDriveGetLogEnabled(driveLogEnabled);

    if (driveLogEnabled && dtSeconds > dtOverrunThreshold &&
        (sampleTick - lastDtWarningTick) >= kDriveDtWarningCooldown) {
      String warn;
      warn.reserve(80);
      warn += "[DRIVE] dt=";
      warn += dtSeconds * 1000.0f;
      warn += "ms (objetivo ";
      warn += expectedPeriodSeconds * 1000.0f;
      warn += "ms)";
      broadcastIf(true, warn);
      lastDtWarningTick = sampleTick;
    }

    RcSharedState rcSnapshot{};
    const bool rcValid = rcGetStateCopy(rcSnapshot);
    const TickType_t snapshotTick = rcSnapshot.lastUpdateTick;
    const bool snapshotFresh =
        rcValid && rcSnapshot.valid && snapshotTick != 0 && (sampleTick - snapshotTick) <= pdMS_TO_TICKS(50);
    const bool rcFresh = snapshotFresh;
    const int rawThrottle = snapshotFresh ? rcSnapshot.throttle : 0;

    const int rcValue = updateThrottleFilter(rawThrottle);
    if (snapshotFresh) {
      g_lastThrottleUpdateTick = snapshotTick;
    }

    PiCommsRxSnapshot piSnapshot{};
    const bool piDriverReady = piCommsGetRxSnapshot(piSnapshot);
    TickType_t piAgeTicks = 0;
    bool piAgeValid = false;
    if (piSnapshot.lastFrameTick != 0) {
      piAgeTicks = sampleTick - piSnapshot.lastFrameTick;
      piAgeValid = true;
    }
    const bool piFresh =
        piDriverReady && piSnapshot.hasFrame && piAgeValid && piAgeTicks <= kPiSnapshotFreshTicks;

    int commandValue = rcValue;
    bool commandFromPi = false;
    bool throttleInhibit = false;
    const bool piEstopActive = piFresh && piSnapshot.estop;
    const uint8_t piBrakePercent = piFresh ? (piEstopActive ? 100u : piSnapshot.brake) : 0u;
    const bool piBrakeActive = piFresh && !piEstopActive && (piBrakePercent > 0u);
    const bool piSpeedPidRequested =
        piFresh && piSnapshot.driveEnabled && !piEstopActive && !piBrakeActive;
    const bool rcSpeedPidRequested = !piFresh && rcFresh;
    SpeedControlSource speedControlSource = SpeedControlSource::kNone;
    if (piSpeedPidRequested) {
      speedControlSource = SpeedControlSource::kPiSpeedPid;
      commandFromPi = true;
    } else if (rcSpeedPidRequested) {
      speedControlSource = SpeedControlSource::kRcSpeedPid;
    }

    bool speedPidFeedbackOk = false;
    bool speedPidFailsafe = false;
    bool speedPidOverspeed = false;
    SpeedPidMode speedPidMode = SpeedPidMode::kNormal;
    float speedTargetRawMps = 0.0f;
    float speedTargetRampedMps = 0.0f;
    float speedMeasuredMps = 0.0f;
    float speedPidErrorMps = 0.0f;
    float speedPidOverspeedErrorMps = 0.0f;
    float speedPidThrottlePercent = 0.0f;
    float speedPidBrakePercent = 0.0f;

    if (speedControlSource != SpeedControlSource::kNone) {
      speedTargetRawMps = (speedControlSource == SpeedControlSource::kPiSpeedPid)
                              ? mapAccelToSpeedTargetMps(piSnapshot.accelRaw)
                              : mapRcToSpeedTargetMps(rcValue);

      HallSpeedSnapshot speedSnapshot{};
      const bool speedOk = hallSpeedGetSnapshot(speedSnapshot) && speedSnapshot.driverReady;
      HallSpeedConfig speedCfg{};
      hallSpeedGetConfig(speedCfg);
      const uint32_t rpmTimeoutUs = (speedCfg.rpmTimeoutUs > 0U) ? speedCfg.rpmTimeoutUs : 500000U;
      const bool transitionFresh =
          speedOk && speedSnapshot.hasTransition && speedSnapshot.transitionAgeUs <= rpmTimeoutUs;
      speedMeasuredMps = speedOk ? speedSnapshot.speedMps : 0.0f;

      if (!speedOk) {
        speedFeedbackMissingTick = sampleTick;
        speedPidFeedbackOk = false;
      } else if (speedTargetRawMps <= 0.05f) {
        speedFeedbackMissingTick = 0;
        speedPidFeedbackOk = true;
      } else if (transitionFresh) {
        speedFeedbackMissingTick = 0;
        speedPidFeedbackOk = true;
      } else {
        if (speedFeedbackMissingTick == 0) {
          speedFeedbackMissingTick = sampleTick;
        }
        speedPidFeedbackOk = (sampleTick - speedFeedbackMissingTick) <= kSpeedPidFeedbackGraceTicks;
      }

      SpeedPidControlOutput speedOutput{};
      const bool speedComputed =
          speedPidCompute(speedTargetRawMps, speedMeasuredMps, speedPidFeedbackOk, dtSeconds, speedOutput);

      if (!speedComputed) {
        speedPidFeedbackOk = false;
        speedPidFailsafe = true;
        speedPidThrottlePercent = 0.0f;
        speedPidBrakePercent = 0.0f;
      } else {
        speedTargetRampedMps = speedOutput.targetRampedMps;
        speedPidErrorMps = speedOutput.errorMps;
        speedPidOverspeedErrorMps = speedOutput.overspeedErrorMps;
        speedPidThrottlePercent = speedOutput.throttlePercent;
        speedPidBrakePercent = speedOutput.brakePercent;
        speedPidFeedbackOk = speedOutput.feedbackOk;
        speedPidFailsafe = speedOutput.failsafeActive;
        speedPidOverspeed = speedOutput.overspeedActive;
        speedPidMode = speedOutput.mode;
      }

      if (!speedPidFeedbackOk || speedPidFailsafe) {
        commandValue = 0;
        throttleInhibit = true;
      } else {
        commandValue = static_cast<int>(speedPidThrottlePercent + 0.5f);
      }
      speedPidWasActive = true;
    } else {
      if (speedPidWasActive) {
        speedPidReset();
        speedPidWasActive = false;
      }
      speedFeedbackMissingTick = 0;
      if (piEstopActive) {
        commandFromPi = true;
        commandValue = 0;
        throttleInhibit = true;
      } else if (piBrakeActive && piSnapshot.driveEnabled) {
        commandFromPi = true;
        commandValue = 0;
        throttleInhibit = true;
      }
    }

    uint8_t overspeedBrakePercent = 0;
    if (speedControlSource != SpeedControlSource::kNone && speedPidFeedbackOk && !speedPidFailsafe &&
        speedPidBrakePercent > 0.0f) {
      overspeedBrakePercent = clampPercentFromFloat(speedPidBrakePercent);
    }

    uint8_t appliedBrakePercent = 0;
    if (piFresh) {
      appliedBrakePercent = (piBrakePercent >= overspeedBrakePercent) ? piBrakePercent : overspeedBrakePercent;
      if (piEstopActive) {
        appliedBrakePercent = 100;
      }
    } else {
      uint8_t rcBrakePercent = 0;
      if (rcFresh) {
        if (rcValue < g_brakeConfig.activationThreshold) {
          rcBrakePercent = 100;
        }
      } else {
        const int brakeInput = throttleDataFresh(pdMS_TO_TICKS(60)) ? getFilteredThrottleValue() : 0;
        if (brakeInput < g_brakeConfig.activationThreshold) {
          rcBrakePercent = 100;
        }
      }
      appliedBrakePercent = (rcBrakePercent >= overspeedBrakePercent) ? rcBrakePercent : overspeedBrakePercent;
    }

    if (appliedBrakePercent > 0) {
      throttleInhibit = true;
    }
    quadBrakeApplyPercent(appliedBrakePercent);

    int duty = 0;
    if (throttleInhibit) {
      quadThrottleStop();
      duty = g_lastDuty;
    } else {
      duty = quadThrottleUpdate(commandValue);
    }

    if (driveLogEnabled && (commandValue != lastThrottleCmdValue || duty != lastDutyReported ||
                     g_brakeCurrentAngles.servoA != lastBrakeReportedA ||
                     g_brakeCurrentAngles.servoB != lastBrakeReportedB)) {
      String msg;
      msg.reserve(196);
      msg += "[DRIVE] cmd=";
      msg += commandValue;
      msg += commandFromPi ? " (PI)" : " (RC)";
      msg += " duty=";
      msg += duty;
      msg += "/";
      msg += static_cast<int>(g_maxDuty);
      msg += " brakeA=";
      msg += g_brakeCurrentAngles.servoA;
      msg += "deg brakeB=";
      msg += g_brakeCurrentAngles.servoB;
      msg += "deg";
      if (piFresh) {
        msg += " piAgeMs=";
        msg += static_cast<int>(piAgeTicks * portTICK_PERIOD_MS);
        msg += " piBrake=";
        msg += piBrakePercent;
      } else if (snapshotFresh) {
        msg += " raw=";
        msg += rawThrottle;
      } else {
        msg += " raw=STALE";
      }
      msg += " brake=";
      msg += appliedBrakePercent;
      if (speedControlSource != SpeedControlSource::kNone) {
        msg += " src=";
        msg += speedControlSourceText(speedControlSource);
        msg += " targetRaw=";
        msg += String(speedTargetRawMps, 2);
        msg += "m/s target=";
        msg += String(speedTargetRampedMps, 2);
        msg += "m/s speed=";
        msg += String(speedMeasuredMps, 2);
        msg += "m/s mode=";
        msg += speedPidModeText(speedPidMode);
        msg += " pidOut=";
        msg += String(speedPidThrottlePercent, 1);
        msg += "% autoBrake=";
        msg += String(speedPidBrakePercent, 1);
        msg += "% err=";
        msg += String(speedPidErrorMps, 2);
        msg += " over=";
        msg += String(speedPidOverspeedErrorMps, 2);
        msg += " fb=";
        msg += speedPidFeedbackOk ? "Y" : "N";
        msg += " fs=";
        msg += speedPidFailsafe ? "Y" : "N";
        msg += " ovs=";
        msg += speedPidOverspeed ? "Y" : "N";
      }
      msg += " ageMs=";
      msg += static_cast<int>((sampleTick - snapshotTick) * portTICK_PERIOD_MS);
      broadcastIf(true, msg);
      lastThrottleCmdValue = commandValue;
      lastDutyReported = duty;
      lastBrakeReportedA = g_brakeCurrentAngles.servoA;
      lastBrakeReportedB = g_brakeCurrentAngles.servoB;
    }

    if (driveLogEnabled && !snapshotFresh && !commandFromPi) {
      if ((sampleTick - lastStaleLog) >= pdMS_TO_TICKS(500)) {
        broadcastIf(true, "[DRIVE] sin datos frescos del RC (>50ms); usando 0");
        lastStaleLog = sampleTick;
      }
    } else {
      lastStaleLog = sampleTick;
    }

    const int64_t iterationDurationUs = esp_timer_get_time() - iterationStartUs;
    if (driveLogEnabled && iterationDurationUs > 4000) {
      if ((sampleTick - lastPerfLog) >= kDriveRuntimeWarningCooldown) {
        String perfMsg = "[DRIVE] ciclo tardo ";
        perfMsg += iterationDurationUs / 1000.0f;
        perfMsg += "ms";
        broadcastIf(true, perfMsg);
        lastPerfLog = sampleTick;
      }
    }
  }
}
