#include "speed_pid.h"

#include <Preferences.h>
#include <freertos/FreeRTOS.h>

#include <math.h>

namespace {

constexpr float kOutputMinPercent = 0.0f;
constexpr float kOutputMaxPercent = 100.0f;

constexpr float kMinKp = 0.0f;
constexpr float kMaxKp = 500.0f;
constexpr float kMinKi = 0.0f;
constexpr float kMaxKi = 500.0f;
constexpr float kMinKd = 0.0f;
constexpr float kMaxKd = 500.0f;

constexpr float kMinMaxSpeedMps = 0.1f;
constexpr float kMaxMaxSpeedMps = 4.17f;
constexpr float kMinRampMps2 = 0.01f;
constexpr float kMaxRampMps2 = 50.0f;
constexpr float kMinIntegralLimit = 0.0f;
constexpr float kMaxIntegralLimit = 2000.0f;
constexpr float kMinDeadbandMps = 0.0f;
constexpr float kMaxDeadbandMps = 5.0f;
constexpr float kMinOverspeedBrakeCapPercent = 0.0f;
constexpr float kMaxOverspeedBrakeCapPercent = 100.0f;
constexpr float kMinOverspeedHysteresisMps = 0.0f;
constexpr float kMaxOverspeedHysteresisMps = 5.0f;

constexpr const char* kPrefsNamespace = "speed_pid";
constexpr uint32_t kPrefsVersion = 1;
constexpr const char* kPrefsKeyVersion = "ver";
constexpr const char* kPrefsKeyKp = "kp";
constexpr const char* kPrefsKeyKi = "ki";
constexpr const char* kPrefsKeyKd = "kd";
constexpr const char* kPrefsKeyRamp = "ramp";
constexpr const char* kPrefsKeyMaxMps = "maxmps";
constexpr const char* kPrefsKeyIntegralLimit = "ilim";
constexpr const char* kPrefsKeyDeadband = "db";
constexpr const char* kPrefsKeyBrakeCap = "brkcap";
constexpr const char* kPrefsKeyHysteresis = "hys";

struct SpeedPidState {
  bool initialized = false;
  bool enabled = true;

  SpeedPidTunings tunings{};
  SpeedPidConfig config{};

  SpeedPidTunings defaultTunings{};
  SpeedPidConfig defaultConfig{};

  float integralTerm = 0.0f;
  float prevError = 0.0f;
  bool firstRun = true;

  SpeedPidMode mode = SpeedPidMode::kNormal;
  bool feedbackOk = false;
  bool failsafeActive = false;
  bool overspeedActive = false;
  float targetRawMps = 0.0f;
  float targetRampedMps = 0.0f;
  float measuredMps = 0.0f;
  float errorMps = 0.0f;
  float overspeedErrorMps = 0.0f;
  float throttleCmdPercent = 0.0f;
  float brakeCmdPercent = 0.0f;
};

SpeedPidState g_state{};
Preferences g_prefs;
portMUX_TYPE g_speedPidMux = portMUX_INITIALIZER_UNLOCKED;

inline float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

bool isFiniteInRange(float value, float minValue, float maxValue) {
  if (!isfinite(value)) {
    return false;
  }
  return value >= minValue && value <= maxValue;
}

bool validateTunings(const SpeedPidTunings& tunings) {
  return isFiniteInRange(tunings.kp, kMinKp, kMaxKp) &&
         isFiniteInRange(tunings.ki, kMinKi, kMaxKi) &&
         isFiniteInRange(tunings.kd, kMinKd, kMaxKd);
}

bool validateConfig(const SpeedPidConfig& config) {
  if (!isFiniteInRange(config.maxSpeedMps, kMinMaxSpeedMps, kMaxMaxSpeedMps)) {
    return false;
  }
  if (!isFiniteInRange(config.maxSetpointRateMps2, kMinRampMps2, kMaxRampMps2)) {
    return false;
  }
  if (!isFiniteInRange(config.integralLimit, kMinIntegralLimit, kMaxIntegralLimit)) {
    return false;
  }
  if (!isFiniteInRange(config.deadbandMps, kMinDeadbandMps, kMaxDeadbandMps)) {
    return false;
  }
  if (!isFiniteInRange(config.overspeedBrakeMaxPercent,
                       kMinOverspeedBrakeCapPercent,
                       kMaxOverspeedBrakeCapPercent)) {
    return false;
  }
  if (!isFiniteInRange(config.overspeedReleaseHysteresisMps,
                       kMinOverspeedHysteresisMps,
                       kMaxOverspeedHysteresisMps)) {
    return false;
  }
  return true;
}

void resetControlTermsLocked() {
  g_state.integralTerm = 0.0f;
  g_state.prevError = 0.0f;
  g_state.firstRun = true;
}

void resetControllerLocked() {
  resetControlTermsLocked();
  g_state.mode = SpeedPidMode::kNormal;
  g_state.feedbackOk = false;
  g_state.failsafeActive = false;
  g_state.overspeedActive = false;
  g_state.targetRawMps = 0.0f;
  g_state.targetRampedMps = 0.0f;
  g_state.measuredMps = 0.0f;
  g_state.errorMps = 0.0f;
  g_state.overspeedErrorMps = 0.0f;
  g_state.throttleCmdPercent = 0.0f;
  g_state.brakeCmdPercent = 0.0f;
}

bool loadFromNvs(SpeedPidTunings& tuningsOut, SpeedPidConfig& configOut, bool* maxSpeedClampedOut) {
  bool maxSpeedClamped = false;
  if (!g_prefs.begin(kPrefsNamespace, true)) {
    return false;
  }

  const uint32_t version = g_prefs.getUInt(kPrefsKeyVersion, 0);
  if (version != kPrefsVersion) {
    g_prefs.end();
    return false;
  }

  SpeedPidTunings loadedTunings = tuningsOut;
  SpeedPidConfig loadedConfig = configOut;

  loadedTunings.kp = g_prefs.getFloat(kPrefsKeyKp, loadedTunings.kp);
  loadedTunings.ki = g_prefs.getFloat(kPrefsKeyKi, loadedTunings.ki);
  loadedTunings.kd = g_prefs.getFloat(kPrefsKeyKd, loadedTunings.kd);

  loadedConfig.maxSetpointRateMps2 = g_prefs.getFloat(kPrefsKeyRamp, loadedConfig.maxSetpointRateMps2);
  loadedConfig.maxSpeedMps = g_prefs.getFloat(kPrefsKeyMaxMps, loadedConfig.maxSpeedMps);
  loadedConfig.integralLimit = g_prefs.getFloat(kPrefsKeyIntegralLimit, loadedConfig.integralLimit);
  loadedConfig.deadbandMps = g_prefs.getFloat(kPrefsKeyDeadband, loadedConfig.deadbandMps);
  loadedConfig.overspeedBrakeMaxPercent =
      g_prefs.getFloat(kPrefsKeyBrakeCap, loadedConfig.overspeedBrakeMaxPercent);
  loadedConfig.overspeedReleaseHysteresisMps =
      g_prefs.getFloat(kPrefsKeyHysteresis, loadedConfig.overspeedReleaseHysteresisMps);

  g_prefs.end();

  if (loadedConfig.maxSpeedMps > kMaxMaxSpeedMps) {
    loadedConfig.maxSpeedMps = kMaxMaxSpeedMps;
    maxSpeedClamped = true;
  }

  if (!validateTunings(loadedTunings) || !validateConfig(loadedConfig)) {
    return false;
  }

  tuningsOut = loadedTunings;
  configOut = loadedConfig;
  if (maxSpeedClampedOut != nullptr) {
    *maxSpeedClampedOut = maxSpeedClamped;
  }
  return true;
}

bool persistToNvs(const SpeedPidTunings& tunings, const SpeedPidConfig& config) {
  if (!g_prefs.begin(kPrefsNamespace, false)) {
    return false;
  }

  g_prefs.putUInt(kPrefsKeyVersion, kPrefsVersion);
  g_prefs.putFloat(kPrefsKeyKp, tunings.kp);
  g_prefs.putFloat(kPrefsKeyKi, tunings.ki);
  g_prefs.putFloat(kPrefsKeyKd, tunings.kd);
  g_prefs.putFloat(kPrefsKeyRamp, config.maxSetpointRateMps2);
  g_prefs.putFloat(kPrefsKeyMaxMps, config.maxSpeedMps);
  g_prefs.putFloat(kPrefsKeyIntegralLimit, config.integralLimit);
  g_prefs.putFloat(kPrefsKeyDeadband, config.deadbandMps);
  g_prefs.putFloat(kPrefsKeyBrakeCap, config.overspeedBrakeMaxPercent);
  g_prefs.putFloat(kPrefsKeyHysteresis, config.overspeedReleaseHysteresisMps);
  g_prefs.end();
  return true;
}

}  // namespace

const char* speedPidModeText(SpeedPidMode mode) {
  switch (mode) {
    case SpeedPidMode::kNormal:
      return "NORMAL";
    case SpeedPidMode::kOverspeed:
      return "OVERSPEED";
    case SpeedPidMode::kFailsafe:
      return "FAILSAFE";
    default:
      return "UNKNOWN";
  }
}

bool speedPidInit(const SpeedPidTunings& defaultTunings, const SpeedPidConfig& defaultConfig) {
  SpeedPidTunings safeTunings = defaultTunings;
  SpeedPidConfig safeConfig = defaultConfig;
  if (!validateTunings(safeTunings) || !validateConfig(safeConfig)) {
    return false;
  }

  portENTER_CRITICAL(&g_speedPidMux);
  if (g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return true;
  }

  g_state.defaultTunings = safeTunings;
  g_state.defaultConfig = safeConfig;
  g_state.tunings = safeTunings;
  g_state.config = safeConfig;
  g_state.enabled = true;
  g_state.initialized = true;
  resetControllerLocked();
  portEXIT_CRITICAL(&g_speedPidMux);

  SpeedPidTunings persistedTunings = safeTunings;
  SpeedPidConfig persistedConfig = safeConfig;
  bool maxSpeedClamped = false;
  if (loadFromNvs(persistedTunings, persistedConfig, &maxSpeedClamped)) {
    portENTER_CRITICAL(&g_speedPidMux);
    g_state.tunings = persistedTunings;
    g_state.config = persistedConfig;
    resetControllerLocked();
    portEXIT_CRITICAL(&g_speedPidMux);

    if (maxSpeedClamped) {
      persistToNvs(persistedTunings, persistedConfig);
    }
  }

  return true;
}

bool speedPidCompute(float targetRawMps,
                     float measuredMps,
                     bool feedbackOk,
                     float dtSeconds,
                     SpeedPidControlOutput& output) {
  output = {};
  output.mode = SpeedPidMode::kFailsafe;

  SpeedPidTunings tunings{};
  SpeedPidConfig config{};
  float prevTargetRampedMps = 0.0f;
  float prevIntegral = 0.0f;
  float prevError = 0.0f;
  bool prevFirstRun = true;
  SpeedPidMode prevMode = SpeedPidMode::kNormal;

  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized || !g_state.enabled) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  tunings = g_state.tunings;
  config = g_state.config;
  prevTargetRampedMps = g_state.targetRampedMps;
  prevIntegral = g_state.integralTerm;
  prevError = g_state.prevError;
  prevFirstRun = g_state.firstRun;
  prevMode = g_state.mode;
  portEXIT_CRITICAL(&g_speedPidMux);

  if (!isfinite(dtSeconds) || dtSeconds <= 0.0f || dtSeconds > 1.0f) {
    dtSeconds = 0.001f;
  }

  const float maxSpeed = config.maxSpeedMps;
  targetRawMps = isfinite(targetRawMps) ? clampf(targetRawMps, 0.0f, maxSpeed) : 0.0f;
  measuredMps = isfinite(measuredMps) ? clampf(measuredMps, 0.0f, maxSpeed) : 0.0f;

  float targetRampedMps = prevTargetRampedMps;
  const float maxStep = config.maxSetpointRateMps2 * dtSeconds;
  const float targetDelta = targetRawMps - targetRampedMps;
  if (maxStep > 0.0f) {
    targetRampedMps += clampf(targetDelta, -maxStep, maxStep);
  } else {
    targetRampedMps = targetRawMps;
  }
  targetRampedMps = clampf(targetRampedMps, 0.0f, maxSpeed);

  float integral = prevIntegral;
  float pidPrevError = prevError;
  bool firstRun = prevFirstRun;

  float throttlePercent = 0.0f;
  float brakePercent = 0.0f;
  float errorMps = 0.0f;
  float overspeedErrorMps = 0.0f;
  SpeedPidMode mode = prevMode;

  if (!feedbackOk) {
    mode = SpeedPidMode::kFailsafe;
    targetRampedMps = 0.0f;
    integral = 0.0f;
    pidPrevError = 0.0f;
    firstRun = true;
  } else {
    const bool overspeedEntry = measuredMps > targetRampedMps;
    const bool overspeedExit = measuredMps <= (targetRampedMps + config.overspeedReleaseHysteresisMps);

    if (prevMode == SpeedPidMode::kOverspeed) {
      mode = overspeedExit ? SpeedPidMode::kNormal : SpeedPidMode::kOverspeed;
    } else if (overspeedEntry) {
      mode = SpeedPidMode::kOverspeed;
    } else {
      mode = SpeedPidMode::kNormal;
    }

    if (mode != prevMode) {
      integral = 0.0f;
      pidPrevError = 0.0f;
      firstRun = true;
    }

    if (mode == SpeedPidMode::kOverspeed) {
      overspeedErrorMps = measuredMps - targetRampedMps;
      if (overspeedErrorMps < 0.0f) {
        overspeedErrorMps = 0.0f;
      }
      const float brakeLinear =
          (overspeedErrorMps * 100.0f) / ((maxSpeed > 0.1f) ? maxSpeed : 0.1f);
      brakePercent = clampf(brakeLinear, 0.0f, config.overspeedBrakeMaxPercent);
      throttlePercent = 0.0f;
      errorMps = targetRampedMps - measuredMps;
    } else {
      errorMps = targetRampedMps - measuredMps;
      if (fabsf(errorMps) < config.deadbandMps) {
        errorMps = 0.0f;
      }

      integral += tunings.ki * errorMps * dtSeconds;
      integral = clampf(integral, -config.integralLimit, config.integralLimit);

      float derivative = 0.0f;
      if (!firstRun && dtSeconds > 0.0f) {
        derivative = (errorMps - pidPrevError) / dtSeconds;
      }

      const float pidOutput = (tunings.kp * errorMps) + integral + (tunings.kd * derivative);
      throttlePercent = clampf(pidOutput, kOutputMinPercent, kOutputMaxPercent);
      brakePercent = 0.0f;

      pidPrevError = errorMps;
      firstRun = false;
    }
  }

  output.throttlePercent = throttlePercent;
  output.brakePercent = brakePercent;
  output.targetRampedMps = targetRampedMps;
  output.errorMps = errorMps;
  output.overspeedErrorMps = overspeedErrorMps;
  output.feedbackOk = feedbackOk;
  output.failsafeActive = (mode == SpeedPidMode::kFailsafe);
  output.overspeedActive = (mode == SpeedPidMode::kOverspeed);
  output.mode = mode;

  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized || !g_state.enabled) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.mode = mode;
  g_state.feedbackOk = feedbackOk;
  g_state.failsafeActive = output.failsafeActive;
  g_state.overspeedActive = output.overspeedActive;
  g_state.targetRawMps = targetRawMps;
  g_state.targetRampedMps = targetRampedMps;
  g_state.measuredMps = measuredMps;
  g_state.errorMps = errorMps;
  g_state.overspeedErrorMps = overspeedErrorMps;
  g_state.throttleCmdPercent = throttlePercent;
  g_state.brakeCmdPercent = brakePercent;
  g_state.integralTerm = integral;
  g_state.prevError = pidPrevError;
  g_state.firstRun = firstRun;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

void speedPidReset() {
  portENTER_CRITICAL(&g_speedPidMux);
  if (g_state.initialized) {
    resetControllerLocked();
  }
  portEXIT_CRITICAL(&g_speedPidMux);
}

bool speedPidGetTunings(SpeedPidTunings& tunings) {
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  tunings = g_state.tunings;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetTunings(const SpeedPidTunings& tunings) {
  if (!validateTunings(tunings)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.tunings = tunings;
  resetControlTermsLocked();
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidGetConfig(SpeedPidConfig& config) {
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  config = g_state.config;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetMaxSpeedMps(float maxSpeedMps) {
  if (!isFiniteInRange(maxSpeedMps, kMinMaxSpeedMps, kMaxMaxSpeedMps)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.maxSpeedMps = maxSpeedMps;
  g_state.targetRawMps = clampf(g_state.targetRawMps, 0.0f, maxSpeedMps);
  g_state.targetRampedMps = clampf(g_state.targetRampedMps, 0.0f, maxSpeedMps);
  g_state.measuredMps = clampf(g_state.measuredMps, 0.0f, maxSpeedMps);
  resetControlTermsLocked();
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetRampRateMps2(float rampRateMps2) {
  if (!isFiniteInRange(rampRateMps2, kMinRampMps2, kMaxRampMps2)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.maxSetpointRateMps2 = rampRateMps2;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetOverspeedBrakeMaxPercent(float brakeCapPercent) {
  if (!isFiniteInRange(brakeCapPercent, kMinOverspeedBrakeCapPercent, kMaxOverspeedBrakeCapPercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.overspeedBrakeMaxPercent = brakeCapPercent;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetOverspeedReleaseHysteresisMps(float hysteresisMps) {
  if (!isFiniteInRange(hysteresisMps, kMinOverspeedHysteresisMps, kMaxOverspeedHysteresisMps)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.overspeedReleaseHysteresisMps = hysteresisMps;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSaveToNvs() {
  SpeedPidTunings tunings{};
  SpeedPidConfig config{};
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  tunings = g_state.tunings;
  config = g_state.config;
  portEXIT_CRITICAL(&g_speedPidMux);

  return persistToNvs(tunings, config);
}

bool speedPidResetToDefaults(bool persistDefaultsToNvs) {
  SpeedPidTunings defaultsTunings{};
  SpeedPidConfig defaultsConfig{};

  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }

  defaultsTunings = g_state.defaultTunings;
  defaultsConfig = g_state.defaultConfig;
  g_state.tunings = defaultsTunings;
  g_state.config = defaultsConfig;
  resetControllerLocked();
  portEXIT_CRITICAL(&g_speedPidMux);

  if (!persistDefaultsToNvs) {
    return true;
  }
  return persistToNvs(defaultsTunings, defaultsConfig);
}

bool speedPidGetSnapshot(SpeedPidRuntimeSnapshot& snapshot) {
  portENTER_CRITICAL(&g_speedPidMux);
  snapshot.initialized = g_state.initialized;
  snapshot.enabled = g_state.enabled;
  snapshot.feedbackOk = g_state.feedbackOk;
  snapshot.failsafeActive = g_state.failsafeActive;
  snapshot.overspeedActive = g_state.overspeedActive;
  snapshot.mode = g_state.mode;
  snapshot.targetRawMps = g_state.targetRawMps;
  snapshot.targetRampedMps = g_state.targetRampedMps;
  snapshot.measuredMps = g_state.measuredMps;
  snapshot.errorMps = g_state.errorMps;
  snapshot.overspeedErrorMps = g_state.overspeedErrorMps;
  snapshot.throttleCmdPercent = g_state.throttleCmdPercent;
  snapshot.brakeCmdPercent = g_state.brakeCmdPercent;
  snapshot.tunings = g_state.tunings;
  snapshot.config = g_state.config;
  portEXIT_CRITICAL(&g_speedPidMux);
  return snapshot.initialized;
}

float speedPidGetMaxSpeedMps() {
  portENTER_CRITICAL(&g_speedPidMux);
  const float maxSpeed = g_state.initialized ? g_state.config.maxSpeedMps : 4.17f;
  portEXIT_CRITICAL(&g_speedPidMux);
  return maxSpeed;
}
