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
bool g_drivePidTraceEnabled = false;
TickType_t g_drivePidTracePeriodTicks = pdMS_TO_TICKS(100);
portMUX_TYPE g_driveLogMux = portMUX_INITIALIZER_UNLOCKED;
bool g_speedTargetOverrideEnabled = false;
float g_speedTargetOverrideMps = 0.0f;
portMUX_TYPE g_speedTargetOverrideMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool g_rcNeutralOffsetCalEnabled = true;
volatile bool g_rcNeutralOffsetCalAllowUpdate = true;
QuadDriveRcDebugSnapshot g_rcDebugSnapshot{};
portMUX_TYPE g_driveRcDebugMux = portMUX_INITIALIZER_UNLOCKED;

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
    const bool allowOffsetUpdate = g_rcNeutralOffsetCalEnabled && g_rcNeutralOffsetCalAllowUpdate;
    if (allowOffsetUpdate && abs(rawValue) < 15) {
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
constexpr TickType_t kSpeedPidFeedbackStartupGraceTicks = pdMS_TO_TICKS(2500);
constexpr TickType_t kSpeedPidFeedbackRunGraceTicks = pdMS_TO_TICKS(1000);
constexpr TickType_t kRcSourceDropoutGraceTicks = pdMS_TO_TICKS(150);
constexpr int kRcNeutralDeadbandPercent = 3;
constexpr float kRcTargetSlewMps2Up = 2.0f;
constexpr float kRcTargetSlewMps2Down = 3.0f;
constexpr float kRcMinValidTargetMps = 0.08f;
constexpr TickType_t kRcBrakeReleaseReentryHoldTicks = pdMS_TO_TICKS(200);
constexpr TickType_t kRcOffsetAutoCalNeutralHoldTicks = pdMS_TO_TICKS(250);
constexpr TickType_t kDriveDtWarningCooldown = pdMS_TO_TICKS(1000);
constexpr TickType_t kDriveRuntimeWarningCooldown = pdMS_TO_TICKS(1000);
constexpr TickType_t kDriveEventCooldown = pdMS_TO_TICKS(300);
constexpr TickType_t kDrivePidTraceDefaultPeriod = pdMS_TO_TICKS(100);

enum class DriveThrottleInhibitReason : uint8_t {
  kNone = 0,
  kEstop,
  kPiBrake,
  kRcBrake,
  kOverspeed,
  kFailsafe,
};

enum class SpeedControlSource : uint8_t {
  kNone = 0,
  kPiSpeedPid,
  kRcSpeedPid,
  kTelnetSpeedPid,
};

enum class RcDriveInputState : uint8_t {
  kStale = 0,
  kNeutral,
  kAccel,
  kManualBrake,
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
  if (rcValue <= kRcNeutralDeadbandPercent) {
    return 0.0f;
  }
  if (rcValue > 100) {
    rcValue = 100;
  }
  const float maxSpeed = speedPidGetMaxSpeedMps();
  float targetMps = (static_cast<float>(rcValue) * maxSpeed) / 100.0f;
  if (targetMps > 0.0f && targetMps < kRcMinValidTargetMps) {
    targetMps = 0.0f;
  }
  return targetMps;
}

const char* speedControlSourceText(SpeedControlSource source) {
  switch (source) {
    case SpeedControlSource::kPiSpeedPid:
      return "PI";
    case SpeedControlSource::kRcSpeedPid:
      return "RC";
    case SpeedControlSource::kTelnetSpeedPid:
      return "TEL";
    case SpeedControlSource::kNone:
    default:
      return "NONE";
  }
}

const char* rcDriveInputStateText(RcDriveInputState state) {
  switch (state) {
    case RcDriveInputState::kNeutral:
      return "NEUTRAL";
    case RcDriveInputState::kAccel:
      return "ACCEL";
    case RcDriveInputState::kManualBrake:
      return "BRAKE";
    case RcDriveInputState::kStale:
    default:
      return "STALE";
  }
}

float slewValue(float current, float target, float upRatePerSec, float downRatePerSec, float dtSeconds) {
  if (!isfinite(current)) {
    current = 0.0f;
  }
  if (!isfinite(target)) {
    target = 0.0f;
  }
  if (!isfinite(dtSeconds) || dtSeconds <= 0.0f) {
    return target;
  }
  const float upStep = fmaxf(0.0f, upRatePerSec) * dtSeconds;
  const float downStep = fmaxf(0.0f, downRatePerSec) * dtSeconds;
  if (target > current) {
    const float delta = target - current;
    return current + ((delta < upStep) ? delta : upStep);
  }
  if (target < current) {
    const float delta = current - target;
    return current - ((delta < downStep) ? delta : downStep);
  }
  return current;
}

void setRcDebugSnapshot(const QuadDriveRcDebugSnapshot& snapshot) {
  portENTER_CRITICAL(&g_driveRcDebugMux);
  g_rcDebugSnapshot = snapshot;
  portEXIT_CRITICAL(&g_driveRcDebugMux);
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

float clampFloat(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

const char* driveThrottleInhibitReasonText(DriveThrottleInhibitReason reason) {
  switch (reason) {
    case DriveThrottleInhibitReason::kEstop:
      return "ESTOP";
    case DriveThrottleInhibitReason::kPiBrake:
      return "PI_BRAKE";
    case DriveThrottleInhibitReason::kRcBrake:
      return "RC_BRAKE";
    case DriveThrottleInhibitReason::kOverspeed:
      return "OVERSPEED";
    case DriveThrottleInhibitReason::kFailsafe:
      return "FAILSAFE";
    case DriveThrottleInhibitReason::kNone:
    default:
      return "NONE";
  }
}

float computeServoBrakePercent(int currentAngleDeg, int releaseAngleDeg, int brakeAngleDeg, bool& validOut) {
  const int delta = brakeAngleDeg - releaseAngleDeg;
  if (delta == 0) {
    validOut = false;
    return 0.0f;
  }
  validOut = true;
  const float percent = (static_cast<float>(currentAngleDeg - releaseAngleDeg) * 100.0f) /
                        static_cast<float>(delta);
  return clampFloat(percent, 0.0f, 100.0f);
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

bool quadDriveSetPidTraceEnabled(bool enabled, TickType_t periodTicks) {
  portENTER_CRITICAL(&g_driveLogMux);
  g_drivePidTraceEnabled = enabled;
  if (enabled) {
    g_drivePidTracePeriodTicks = (periodTicks > 0) ? periodTicks : kDrivePidTraceDefaultPeriod;
  } else {
    g_drivePidTracePeriodTicks = (periodTicks > 0) ? periodTicks : g_drivePidTracePeriodTicks;
  }
  portEXIT_CRITICAL(&g_driveLogMux);
  return true;
}

bool quadDriveGetPidTraceConfig(bool& enabledOut, TickType_t& periodTicksOut) {
  portENTER_CRITICAL(&g_driveLogMux);
  enabledOut = g_drivePidTraceEnabled;
  periodTicksOut = (g_drivePidTracePeriodTicks > 0) ? g_drivePidTracePeriodTicks : kDrivePidTraceDefaultPeriod;
  portEXIT_CRITICAL(&g_driveLogMux);
  return true;
}

bool quadDriveSetSpeedTargetOverride(bool enabled, float targetMps) {
  if (!enabled) {
    portENTER_CRITICAL(&g_speedTargetOverrideMux);
    g_speedTargetOverrideEnabled = false;
    g_speedTargetOverrideMps = 0.0f;
    portEXIT_CRITICAL(&g_speedTargetOverrideMux);
    return true;
  }

  const float maxSpeed = speedPidGetMaxSpeedMps();
  if (!isfinite(targetMps) || targetMps < 0.0f || targetMps > maxSpeed) {
    return false;
  }

  portENTER_CRITICAL(&g_speedTargetOverrideMux);
  g_speedTargetOverrideEnabled = true;
  g_speedTargetOverrideMps = targetMps;
  portEXIT_CRITICAL(&g_speedTargetOverrideMux);
  return true;
}

bool quadDriveGetSpeedTargetOverride(bool& enabledOut, float& targetMpsOut) {
  portENTER_CRITICAL(&g_speedTargetOverrideMux);
  enabledOut = g_speedTargetOverrideEnabled;
  targetMpsOut = g_speedTargetOverrideMps;
  portEXIT_CRITICAL(&g_speedTargetOverrideMux);
  return true;
}

bool quadDriveGetRcDebugSnapshot(QuadDriveRcDebugSnapshot& out) {
  portENTER_CRITICAL(&g_driveRcDebugMux);
  out = g_rcDebugSnapshot;
  portEXIT_CRITICAL(&g_driveRcDebugMux);
  return true;
}

bool quadDriveSetRcNeutralCalEnabled(bool enabled) {
  g_rcNeutralOffsetCalEnabled = enabled;
  if (!enabled) {
    g_rcNeutralOffsetCalAllowUpdate = false;
  }
  return true;
}

bool quadDriveGetRcNeutralCalEnabled(bool& enabledOut) {
  enabledOut = g_rcNeutralOffsetCalEnabled;
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
  TickType_t lastPidTraceTick = 0;
  TickType_t lastFailsafeEventTick = 0;
  TickType_t lastOverspeedEventTick = 0;
  TickType_t lastInhibitEventTick = 0;
  TickType_t lastBrakeConfigEventTick = 0;
  TickType_t lastRcSourceEventTick = 0;
  TickType_t lastRcStateEventTick = 0;
  int lastThrottleCmdValue = 9999;
  int lastDutyReported = -1;
  int lastBrakeReportedA = -1;
  int lastBrakeReportedB = -1;
  TickType_t speedFeedbackMissingTick = 0;
  bool speedPidSeenTransition = false;
  bool speedTransitionCounterPrimed = false;
  uint32_t speedLastTransitionsOk = 0;
  SpeedControlSource lastSpeedControlSource = SpeedControlSource::kNone;
  int64_t lastLoopUs = esp_timer_get_time();
  bool speedPidWasActive = false;
  bool lastFailsafeState = false;
  bool lastOverspeedState = false;
  DriveThrottleInhibitReason lastInhibitReason = DriveThrottleInhibitReason::kNone;
  bool lastRcUsingLatchState = false;
  TickType_t rcLastFreshTick = 0;
  bool rcSourceLatched = false;
  float rcTargetShapedMps = 0.0f;
  float rcLastTargetRawMps = 0.0f;
  TickType_t rcBrakeReleaseReentryHoldUntilTick = 0;
  bool lastRcManualBrakeActive = false;
  RcDriveInputState lastRcInputState = RcDriveInputState::kStale;
  TickType_t rcNeutralStableSinceTick = 0;
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
    bool drivePidTraceEnabled = false;
    TickType_t drivePidTracePeriodTicks = kDrivePidTraceDefaultPeriod;
    quadDriveGetPidTraceConfig(drivePidTraceEnabled, drivePidTracePeriodTicks);
    const SpeedControlSource prevSpeedControlSource = lastSpeedControlSource;

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
      rcLastFreshTick = sampleTick;
    }

    const int rcFilteredThrottle = g_filteredThrottleValue;

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
    uint8_t rcBrakePercent = 0;
    if (!piFresh) {
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
    }
    const bool rcManualBrakeActive = (!piFresh && rcBrakePercent > 0);
    RcDriveInputState rcInputState = RcDriveInputState::kStale;
    if (!piFresh) {
      if (rcManualBrakeActive) {
        rcInputState = RcDriveInputState::kManualBrake;
      } else if (rcFresh && rcValue > kRcNeutralDeadbandPercent) {
        rcInputState = RcDriveInputState::kAccel;
      } else if (rcFresh) {
        rcInputState = RcDriveInputState::kNeutral;
      }
    }

    if (rcInputState == RcDriveInputState::kNeutral && rcFresh) {
      if (rcNeutralStableSinceTick == 0) {
        rcNeutralStableSinceTick = sampleTick;
      }
    } else {
      rcNeutralStableSinceTick = 0;
    }

    bool speedTargetOverrideEnabled = false;
    float speedTargetOverrideMps = 0.0f;
    quadDriveGetSpeedTargetOverride(speedTargetOverrideEnabled, speedTargetOverrideMps);

    const bool piSpeedPidRequested =
        piFresh && piSnapshot.driveEnabled && !piEstopActive && !piBrakeActive;
    const bool rcSpeedPidEligible =
        !piFresh && rcFresh && !speedTargetOverrideEnabled && !rcManualBrakeActive;
    const bool rcCanLatch =
        !piFresh && !speedTargetOverrideEnabled && !rcManualBrakeActive && !piEstopActive;
    const bool rcDropoutGraceActive =
        rcSourceLatched && rcCanLatch && rcLastFreshTick != 0 &&
        (sampleTick - rcLastFreshTick) <= kRcSourceDropoutGraceTicks;
    const bool rcSpeedPidRequested = rcSpeedPidEligible || rcDropoutGraceActive;
    const bool rcUsingLatchThisCycle = (!rcSpeedPidEligible && rcDropoutGraceActive);
    SpeedControlSource speedControlSource = SpeedControlSource::kNone;
    if (speedTargetOverrideEnabled && !piEstopActive) {
      speedControlSource = SpeedControlSource::kTelnetSpeedPid;
    } else if (piSpeedPidRequested) {
      speedControlSource = SpeedControlSource::kPiSpeedPid;
      commandFromPi = true;
    } else if (rcSpeedPidRequested) {
      speedControlSource = SpeedControlSource::kRcSpeedPid;
    }
    if (speedControlSource != SpeedControlSource::kRcSpeedPid &&
        lastSpeedControlSource == SpeedControlSource::kRcSpeedPid) {
      rcTargetShapedMps = 0.0f;
      rcLastTargetRawMps = 0.0f;
    }

    const bool rcManualBrakeReleased = lastRcManualBrakeActive && !rcManualBrakeActive;
    if (rcManualBrakeReleased) {
      rcBrakeReleaseReentryHoldUntilTick = sampleTick + kRcBrakeReleaseReentryHoldTicks;
      if (!speedTargetOverrideEnabled && !piFresh) {
        speedPidReset();
        speedPidWasActive = false;
      }
    }
    lastRcManualBrakeActive = rcManualBrakeActive;

    const bool rcReentryHoldActive =
        (rcBrakeReleaseReentryHoldUntilTick != 0) && (sampleTick < rcBrakeReleaseReentryHoldUntilTick);
    if (!rcReentryHoldActive && rcBrakeReleaseReentryHoldUntilTick != 0) {
      rcBrakeReleaseReentryHoldUntilTick = 0;
    }

    bool rcAllowOffsetCalThisCycle = g_rcNeutralOffsetCalEnabled;
    if (!(rcFresh && rcInputState == RcDriveInputState::kNeutral &&
          rcNeutralStableSinceTick != 0 &&
          (sampleTick - rcNeutralStableSinceTick) >= kRcOffsetAutoCalNeutralHoldTicks &&
          (speedControlSource == SpeedControlSource::kNone))) {
      rcAllowOffsetCalThisCycle = false;
    }
    g_rcNeutralOffsetCalAllowUpdate = rcAllowOffsetCalThisCycle;

    bool speedPidFeedbackOk = false;
    bool speedPidFailsafe = false;
    bool speedPidOverspeed = false;
    SpeedPidMode speedPidMode = SpeedPidMode::kNormal;
    float speedTargetRawMps = 0.0f;
    float speedTargetRampedMps = 0.0f;
    float speedMeasuredMps = 0.0f;
    float speedMeasuredFilteredMps = 0.0f;
    float speedPidErrorMps = 0.0f;
    float speedPidPTerm = 0.0f;
    float speedPidITerm = 0.0f;
    float speedPidDTerm = 0.0f;
    float speedPidUnsatOutput = 0.0f;
    float speedPidSatOutput = 0.0f;
    float speedPidThrottleBasePercent = 0.0f;
    float speedPidThrottleDeltaPercent = 0.0f;
    float speedPidThrottlePreSlewPercent = 0.0f;
    bool speedPidThrottleBaseActive = false;
    float speedPidOverspeedErrorMps = 0.0f;
    float speedPidThrottlePercent = 0.0f;
    float speedPidThrottleRawPercent = 0.0f;
    float speedPidThrottleFilteredPercent = 0.0f;
    float speedPidBrakePercent = 0.0f;
    float speedPidBrakeRawPercent = 0.0f;
    float speedPidBrakeFilteredPercent = 0.0f;
    bool speedPidThrottleSaturated = false;
    bool speedPidIntegratorClamped = false;
    bool speedPidLaunchAssistActive = false;
    uint16_t speedPidLaunchAssistRemainingMs = 0;
    bool speedPidOverspeedHoldActive = false;
    uint16_t speedPidOverspeedHoldRemainingMs = 0;
    float rcTargetRawMpsDebug = 0.0f;
    float rcTargetShapedMpsDebug = rcTargetShapedMps;
    bool rcSourceLatchedDebug = rcSourceLatched && speedControlSource == SpeedControlSource::kRcSpeedPid;
    bool rcReentryHoldBlocksThrottle = false;

    if (speedControlSource == SpeedControlSource::kRcSpeedPid) {
      rcSourceLatched = true;
      rcSourceLatchedDebug = rcUsingLatchThisCycle;
    } else if (speedControlSource != SpeedControlSource::kRcSpeedPid) {
      rcSourceLatched = false;
      rcSourceLatchedDebug = false;
    }

    if (speedControlSource != SpeedControlSource::kNone) {
      if (!speedPidWasActive || speedControlSource != lastSpeedControlSource) {
        speedFeedbackMissingTick = 0;
        speedPidSeenTransition = false;
        speedTransitionCounterPrimed = false;
        speedLastTransitionsOk = 0;
      }
      if (speedControlSource == SpeedControlSource::kPiSpeedPid) {
        speedTargetRawMps = mapAccelToSpeedTargetMps(piSnapshot.accelRaw);
      } else if (speedControlSource == SpeedControlSource::kRcSpeedPid) {
        const float rcTargetInstantMps =
            rcUsingLatchThisCycle ? rcLastTargetRawMps : mapRcToSpeedTargetMps(rcValue);
        rcLastTargetRawMps = rcTargetInstantMps;
        rcTargetRawMpsDebug = rcTargetInstantMps;
        if (!rcUsingLatchThisCycle) {
          rcTargetShapedMps = slewValue(rcTargetShapedMps,
                                        rcTargetInstantMps,
                                        kRcTargetSlewMps2Up,
                                        kRcTargetSlewMps2Down,
                                        dtSeconds);
          // No recortar la subida inicial: con dt~30ms y slew=2.0 m/s² el primer
          // paso (~0.06 m/s) quedaba por debajo del umbral y el target nunca despegaba.
          // El snap a cero se aplica solo cuando el objetivo instantáneo ya es cero.
          if (rcTargetInstantMps <= 0.0f && fabsf(rcTargetShapedMps) < kRcMinValidTargetMps) {
            rcTargetShapedMps = 0.0f;
          }
        }
        if (rcReentryHoldActive && rcInputState == RcDriveInputState::kNeutral) {
          rcReentryHoldBlocksThrottle = true;
          rcTargetShapedMps = 0.0f;
        }
        rcTargetShapedMpsDebug = rcTargetShapedMps;
        speedTargetRawMps = rcTargetShapedMps;
      } else {
        speedTargetRawMps = speedTargetOverrideMps;
      }

      HallSpeedSnapshot speedSnapshot{};
      const bool speedOk = hallSpeedGetSnapshot(speedSnapshot) && speedSnapshot.driverReady;
      HallSpeedConfig speedCfg{};
      hallSpeedGetConfig(speedCfg);
      const uint32_t rpmTimeoutUs = (speedCfg.rpmTimeoutUs > 0U) ? speedCfg.rpmTimeoutUs : 500000U;
      if (speedOk && !speedTransitionCounterPrimed) {
        speedLastTransitionsOk = speedSnapshot.transitionsOk;
        speedTransitionCounterPrimed = true;
      }
      bool validTransitionEvent = false;
      if (speedOk && speedTransitionCounterPrimed) {
        if (speedSnapshot.transitionsOk < speedLastTransitionsOk) {
          speedLastTransitionsOk = speedSnapshot.transitionsOk;
        } else if (speedSnapshot.transitionsOk > speedLastTransitionsOk) {
          validTransitionEvent = true;
          speedLastTransitionsOk = speedSnapshot.transitionsOk;
        }
      }
      const bool transitionFresh = speedOk && validTransitionEvent &&
                                   speedSnapshot.hasTransition &&
                                   speedSnapshot.transitionAgeUs <= rpmTimeoutUs;
      speedMeasuredMps = speedOk ? speedSnapshot.speedMps : 0.0f;

      if (!speedOk) {
        speedFeedbackMissingTick = sampleTick;
        speedPidFeedbackOk = false;
      } else if (speedTargetRawMps <= 0.05f) {
        speedFeedbackMissingTick = 0;
        speedPidFeedbackOk = true;
      } else if (transitionFresh) {
        speedFeedbackMissingTick = 0;
        speedPidSeenTransition = true;
        speedPidFeedbackOk = true;
      } else {
        if (speedFeedbackMissingTick == 0) {
          speedFeedbackMissingTick = sampleTick;
        }
        const TickType_t graceTicks =
            speedPidSeenTransition ? kSpeedPidFeedbackRunGraceTicks : kSpeedPidFeedbackStartupGraceTicks;
        speedPidFeedbackOk = (sampleTick - speedFeedbackMissingTick) <= graceTicks;
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
        speedMeasuredFilteredMps = speedOutput.measuredFilteredMps;
        speedPidErrorMps = speedOutput.errorMps;
        speedPidPTerm = speedOutput.pTerm;
        speedPidITerm = speedOutput.iTerm;
        speedPidDTerm = speedOutput.dTerm;
        speedPidUnsatOutput = speedOutput.pidUnsatOutput;
        speedPidSatOutput = speedOutput.pidSatOutput;
        speedPidThrottleBasePercent = speedOutput.throttleBasePercent;
        speedPidThrottleDeltaPercent = speedOutput.throttlePidDeltaPercent;
        speedPidThrottlePreSlewPercent = speedOutput.throttleCmdPreSlewPercent;
        speedPidThrottleBaseActive = speedOutput.throttleBaseActive;
        speedPidOverspeedErrorMps = speedOutput.overspeedErrorMps;
        speedPidThrottlePercent = speedOutput.throttlePercent;
        speedPidThrottleRawPercent = speedOutput.throttleCmdRawPercent;
        speedPidThrottleFilteredPercent = speedOutput.throttleCmdFilteredPercent;
        speedPidBrakePercent = speedOutput.brakePercent;
        speedPidBrakeRawPercent = speedOutput.overspeedBrakeRawPercent;
        speedPidBrakeFilteredPercent = speedOutput.overspeedBrakeFilteredPercent;
        speedPidThrottleSaturated = speedOutput.throttleSaturated;
        speedPidIntegratorClamped = speedOutput.integratorClamped;
        speedPidLaunchAssistActive = speedOutput.launchAssistActive;
        speedPidLaunchAssistRemainingMs = speedOutput.launchAssistRemainingMs;
        speedPidOverspeedHoldActive = speedOutput.overspeedHoldActive;
        speedPidOverspeedHoldRemainingMs = speedOutput.overspeedHoldRemainingMs;
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
      if (speedControlSource == SpeedControlSource::kRcSpeedPid && rcReentryHoldBlocksThrottle) {
        commandValue = 0;
      }
      speedPidWasActive = true;
      lastSpeedControlSource = speedControlSource;
    } else {
      if (speedPidWasActive) {
        speedPidReset();
        speedPidWasActive = false;
      }
      speedFeedbackMissingTick = 0;
      speedPidSeenTransition = false;
      speedTransitionCounterPrimed = false;
      speedLastTransitionsOk = 0;
      lastSpeedControlSource = SpeedControlSource::kNone;
      rcTargetShapedMps = 0.0f;
      rcLastTargetRawMps = 0.0f;
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
      appliedBrakePercent = (rcBrakePercent >= overspeedBrakePercent) ? rcBrakePercent : overspeedBrakePercent;
    }

    DriveThrottleInhibitReason inhibitReason = DriveThrottleInhibitReason::kNone;
    if (piEstopActive) {
      inhibitReason = DriveThrottleInhibitReason::kEstop;
    } else if (speedControlSource != SpeedControlSource::kNone && (!speedPidFeedbackOk || speedPidFailsafe)) {
      inhibitReason = DriveThrottleInhibitReason::kFailsafe;
    } else if (piFresh && piBrakePercent > 0) {
      inhibitReason = DriveThrottleInhibitReason::kPiBrake;
    } else if (!piFresh && rcBrakePercent > 0) {
      inhibitReason = DriveThrottleInhibitReason::kRcBrake;
    } else if (overspeedBrakePercent > 0) {
      inhibitReason = DriveThrottleInhibitReason::kOverspeed;
    }

    if (appliedBrakePercent > 0 || inhibitReason != DriveThrottleInhibitReason::kNone) {
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

    bool brakeServoAValid = false;
    bool brakeServoBValid = false;
    const float brakeServoAPercent =
        computeServoBrakePercent(g_brakeCurrentAngles.servoA,
                                 g_brakeConfig.releaseAngleServoADeg,
                                 g_brakeConfig.brakeAngleServoADeg,
                                 brakeServoAValid);
    const float brakeServoBPercent =
        computeServoBrakePercent(g_brakeCurrentAngles.servoB,
                                 g_brakeConfig.releaseAngleServoBDeg,
                                 g_brakeConfig.brakeAngleServoBDeg,
                                 brakeServoBValid);

    QuadDriveRcDebugSnapshot rcDebug{};
    rcDebug.rawThrottle = rawThrottle;
    rcDebug.filteredThrottle = rcFilteredThrottle;
    rcDebug.normalizedThrottle = rcValue;
    rcDebug.rcFresh = rcFresh;
    rcDebug.snapshotAgeMs = (snapshotTick != 0) ? static_cast<uint32_t>((sampleTick - snapshotTick) * portTICK_PERIOD_MS)
                                                : 0u;
    rcDebug.rcManualBrakeActive = rcManualBrakeActive;
    rcDebug.rcSpeedPidEligible = rcSpeedPidEligible;
    rcDebug.rcSourceLatched = rcUsingLatchThisCycle;
    rcDebug.rcNeutralOffsetCalEnabled = g_rcNeutralOffsetCalEnabled;
    rcDebug.rcNeutralOffsetCalAllowed = g_rcNeutralOffsetCalAllowUpdate;
    rcDebug.rcTargetRawMps = rcTargetRawMpsDebug;
    rcDebug.rcTargetShapedMps = rcTargetShapedMpsDebug;
    setRcDebugSnapshot(rcDebug);

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
        msg += "% autoBrakeRaw=";
        msg += String(speedPidBrakeRawPercent, 1);
        msg += "% autoBrakeFilt=";
        msg += String(speedPidBrakeFilteredPercent, 1);
        msg += "% autoBrake=";
        msg += String(speedPidBrakePercent, 1);
        msg += "% err=";
        msg += String(speedPidErrorMps, 2);
        msg += " over=";
        msg += String(speedPidOverspeedErrorMps, 2);
        msg += " hold=";
        msg += speedPidOverspeedHoldActive ? "Y" : "N";
        msg += "(";
        msg += speedPidOverspeedHoldRemainingMs;
        msg += "ms)";
        msg += " fb=";
        msg += speedPidFeedbackOk ? "Y" : "N";
        msg += " fs=";
        msg += speedPidFailsafe ? "Y" : "N";
        msg += " ovs=";
        msg += speedPidOverspeed ? "Y" : "N";
        msg += " launch=";
        msg += speedPidLaunchAssistActive ? "Y" : "N";
        msg += "(";
        msg += speedPidLaunchAssistRemainingMs;
        msg += "ms)";
        msg += " sat=";
        msg += speedPidThrottleSaturated ? "Y" : "N";
        msg += " iclamp=";
        msg += speedPidIntegratorClamped ? "Y" : "N";
      }
      msg += " ageMs=";
      msg += static_cast<int>((sampleTick - snapshotTick) * portTICK_PERIOD_MS);
      broadcastIf(true, msg);
      lastThrottleCmdValue = commandValue;
      lastDutyReported = duty;
      lastBrakeReportedA = g_brakeCurrentAngles.servoA;
      lastBrakeReportedB = g_brakeCurrentAngles.servoB;
    }

    if (drivePidTraceEnabled) {
      if ((!brakeServoAValid || !brakeServoBValid) &&
          (sampleTick - lastBrakeConfigEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(96);
        msg += "[DRIVE][EVENT] BRAKE_CFG_INVALID A=";
        msg += brakeServoAValid ? "Y" : "N";
        msg += " B=";
        msg += brakeServoBValid ? "Y" : "N";
        broadcastIf(true, msg);
        lastBrakeConfigEventTick = sampleTick;
      }

      if (speedPidFailsafe != lastFailsafeState &&
          (sampleTick - lastFailsafeEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(72);
        msg += "[DRIVE][EVENT] ";
        msg += speedPidFailsafe ? "FAILSAFE_ENTER" : "FAILSAFE_EXIT";
        msg += " src=";
        msg += speedControlSourceText(speedControlSource);
        broadcastIf(true, msg);
        lastFailsafeEventTick = sampleTick;
      }
      lastFailsafeState = speedPidFailsafe;

      if (speedPidOverspeed != lastOverspeedState &&
          (sampleTick - lastOverspeedEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(76);
        msg += "[DRIVE][EVENT] ";
        msg += speedPidOverspeed ? "OVERSPEED_ENTER" : "OVERSPEED_EXIT";
        msg += " src=";
        msg += speedControlSourceText(speedControlSource);
        broadcastIf(true, msg);
        lastOverspeedEventTick = sampleTick;
      }
      lastOverspeedState = speedPidOverspeed;

      if (inhibitReason != lastInhibitReason &&
          (sampleTick - lastInhibitEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(92);
        msg += "[DRIVE][EVENT] THROTTLE_INHIBIT reason=";
        msg += driveThrottleInhibitReasonText(inhibitReason);
        msg += " src=";
        msg += speedControlSourceText(speedControlSource);
        broadcastIf(true, msg);
        lastInhibitEventTick = sampleTick;
      }
      lastInhibitReason = inhibitReason;

      if (speedControlSource == SpeedControlSource::kRcSpeedPid &&
          prevSpeedControlSource != SpeedControlSource::kRcSpeedPid &&
          (sampleTick - lastRcSourceEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(80);
        msg += "[DRIVE][EVENT] RC_SRC_ENTER";
        msg += " latch=";
        msg += rcUsingLatchThisCycle ? "Y" : "N";
        broadcastIf(true, msg);
        lastRcSourceEventTick = sampleTick;
      }

      if (rcUsingLatchThisCycle != lastRcUsingLatchState &&
          (sampleTick - lastRcSourceEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(92);
        msg += "[DRIVE][EVENT] ";
        msg += rcUsingLatchThisCycle ? "RC_SRC_DROP_LATCH" : "RC_SRC_DROP_LATCH_EXIT";
        msg += " fresh=";
        msg += rcFresh ? "Y" : "N";
        msg += " src=";
        msg += speedControlSourceText(speedControlSource);
        broadcastIf(true, msg);
        lastRcSourceEventTick = sampleTick;
      }
      lastRcUsingLatchState = rcUsingLatchThisCycle;

      if (prevSpeedControlSource == SpeedControlSource::kRcSpeedPid &&
          speedControlSource != SpeedControlSource::kRcSpeedPid &&
          !piFresh && !speedTargetOverrideEnabled && !rcManualBrakeActive &&
          !rcFresh && !rcDropoutGraceActive &&
          (sampleTick - lastRcSourceEventTick) >= kDriveEventCooldown) {
        broadcastIf(true, "[DRIVE][EVENT] RC_SRC_DROP_TIMEOUT");
        lastRcSourceEventTick = sampleTick;
      }

      if (rcInputState != lastRcInputState &&
          (sampleTick - lastRcStateEventTick) >= kDriveEventCooldown) {
        String msg;
        msg.reserve(112);
        bool emitted = false;
        if (rcInputState == RcDriveInputState::kManualBrake || lastRcInputState == RcDriveInputState::kManualBrake) {
          msg += "[DRIVE][EVENT] ";
          msg += (rcInputState == RcDriveInputState::kManualBrake) ? "RC_BRAKE_ENTER" : "RC_BRAKE_EXIT";
          emitted = true;
        } else if (rcInputState == RcDriveInputState::kNeutral || lastRcInputState == RcDriveInputState::kNeutral) {
          msg += "[DRIVE][EVENT] ";
          msg += (rcInputState == RcDriveInputState::kNeutral) ? "RC_NEUTRAL_ENTER" : "RC_NEUTRAL_EXIT";
          emitted = true;
        }
        if (emitted) {
          msg += " raw=";
          msg += rawThrottle;
          msg += " norm=";
          msg += rcValue;
          broadcastIf(true, msg);
          lastRcStateEventTick = sampleTick;
        }
      }
      lastRcInputState = rcInputState;

      const bool traceDue =
          (lastPidTraceTick == 0) || ((sampleTick - lastPidTraceTick) >= drivePidTracePeriodTicks);
      if (traceDue && speedControlSource != SpeedControlSource::kNone) {
        const float pwmCmdPercent = throttleInhibit ? 0.0f : clampFloat(static_cast<float>(abs(commandValue)), 0.0f, 100.0f);
        const float pwmDutyPercent =
            (g_maxDuty > 0) ? (static_cast<float>(duty) * 100.0f) / static_cast<float>(g_maxDuty) : 0.0f;

        String msg;
        msg.reserve(520);
        msg += "[DRIVE][PIDTRACE] tMs=";
        msg += static_cast<uint32_t>(sampleTick * portTICK_PERIOD_MS);
        msg += " src=";
        msg += speedControlSourceText(speedControlSource);
        msg += " mode=";
        msg += speedPidModeText(speedPidMode);
        msg += " targetRawMps=";
        msg += String(speedTargetRawMps, 3);
        msg += " targetMps=";
        msg += String(speedTargetRampedMps, 3);
        msg += " speedMps=";
        msg += String(speedMeasuredMps, 3);
        msg += " speedFiltMps=";
        msg += String(speedMeasuredFilteredMps, 3);
        msg += " errMps=";
        msg += String(speedPidErrorMps, 3);
        msg += " p=";
        msg += String(speedPidPTerm, 3);
        msg += " i=";
        msg += String(speedPidITerm, 3);
        msg += " d=";
        msg += String(speedPidDTerm, 3);
        msg += " pidUnsat=";
        msg += String(speedPidUnsatOutput, 3);
        msg += " pidOutPct=";
        msg += String(speedPidThrottlePercent, 3);
        msg += " pidSatPct=";
        msg += String(speedPidSatOutput, 3);
        msg += " ffBasePct=";
        msg += String(speedPidThrottleBasePercent, 3);
        msg += " ffDeltaPct=";
        msg += String(speedPidThrottleDeltaPercent, 3);
        msg += " cmdPreSlewPct=";
        msg += String(speedPidThrottlePreSlewPercent, 3);
        msg += " ffActive=";
        msg += speedPidThrottleBaseActive ? "Y" : "N";
        msg += " throttleRawPct=";
        msg += String(speedPidThrottleRawPercent, 3);
        msg += " throttleFiltPct=";
        msg += String(speedPidThrottleFilteredPercent, 3);
        msg += " pwmCmdPct=";
        msg += String(pwmCmdPercent, 2);
        msg += " pwmDuty=";
        msg += duty;
        msg += "/";
        msg += static_cast<int>(g_maxDuty);
        msg += " pwmDutyPct=";
        msg += String(pwmDutyPercent, 2);
        msg += " autoBrakeRawPct=";
        msg += String(speedPidBrakeRawPercent, 2);
        msg += " autoBrakeFiltPct=";
        msg += String(speedPidBrakeFilteredPercent, 2);
        msg += " brakeAppliedPct=";
        msg += String(static_cast<float>(appliedBrakePercent), 2);
        msg += " brakeA_pct=";
        msg += String(brakeServoAPercent, 2);
        msg += " brakeB_pct=";
        msg += String(brakeServoBPercent, 2);
        msg += " launch=";
        msg += speedPidLaunchAssistActive ? "Y" : "N";
        msg += " launchAssistActive=";
        msg += speedPidLaunchAssistActive ? "Y" : "N";
        msg += " launchMs=";
        msg += speedPidLaunchAssistRemainingMs;
        msg += " sat=";
        msg += speedPidThrottleSaturated ? "Y" : "N";
        msg += " throttleSaturated=";
        msg += speedPidThrottleSaturated ? "Y" : "N";
        msg += " iclamp=";
        msg += speedPidIntegratorClamped ? "Y" : "N";
        msg += " integratorClamped=";
        msg += speedPidIntegratorClamped ? "Y" : "N";
        msg += " fb=";
        msg += speedPidFeedbackOk ? "Y" : "N";
        msg += " fs=";
        msg += speedPidFailsafe ? "Y" : "N";
        msg += " ovs=";
        msg += speedPidOverspeed ? "Y" : "N";
        msg += " estop=";
        msg += piEstopActive ? "Y" : "N";
        msg += " inhibit=";
        msg += driveThrottleInhibitReasonText(inhibitReason);
        if (speedControlSource == SpeedControlSource::kRcSpeedPid) {
          msg += " rcRaw=";
          msg += rawThrottle;
          msg += " rcFilt=";
          msg += rcFilteredThrottle;
          msg += " rcNorm=";
          msg += rcValue;
          msg += " rcFresh=";
          msg += rcFresh ? "Y" : "N";
          msg += " rcElig=";
          msg += rcSpeedPidEligible ? "Y" : "N";
          msg += " rcBrake=";
          msg += rcManualBrakeActive ? "Y" : "N";
          msg += " rcLatched=";
          msg += rcUsingLatchThisCycle ? "Y" : "N";
          msg += " rcState=";
          msg += rcDriveInputStateText(rcInputState);
          msg += " rcTargetRawMps=";
          msg += String(rcTargetRawMpsDebug, 3);
          msg += " rcTargetShapedMps=";
          msg += String(rcTargetShapedMpsDebug, 3);
        }
        broadcastIf(true, msg);
        lastPidTraceTick = sampleTick;
      }
    }
    lastRcInputState = rcInputState;
    lastRcUsingLatchState = rcUsingLatchThisCycle;

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
