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
constexpr float kMinMinThrottlePercent = 0.0f;
constexpr float kMaxMinThrottlePercent = 100.0f;
constexpr float kMinThrottleSlewPctPerSec = 0.1f;
constexpr float kMaxThrottleSlewPctPerSec = 1000.0f;
constexpr float kMinLaunchAssistMaxSpeedMps = 0.0f;
constexpr float kMaxLaunchAssistMaxSpeedMps = kMaxMaxSpeedMps;
constexpr uint16_t kMinLaunchAssistWindowMs = 0u;
constexpr uint16_t kMaxLaunchAssistWindowMs = 10000u;
constexpr float kMinThrottleBasePercent = 0.0f;
constexpr float kMaxThrottleBasePercent = 100.0f;
constexpr float kMinThrottleBaseActivationMps = 0.0f;
constexpr float kMaxThrottleBaseActivationMps = kMaxMaxSpeedMps;
constexpr float kMinThrottleBaseDeltaMaxPercent = 0.0f;
constexpr float kMaxThrottleBaseDeltaMaxPercent = 100.0f;
constexpr uint16_t kMinFeedbackLaunchGraceMs = 0u;
constexpr uint16_t kMaxFeedbackLaunchGraceMs = 5000u;
constexpr float kMinIntegratorUnwindGain = 0.0f;
constexpr float kMaxIntegratorUnwindGain = 10.0f;
constexpr float kMinDerivativeFilterHz = 0.1f;
constexpr float kMaxDerivativeFilterHz = 100.0f;
constexpr float kMinOverspeedBrakeCapPercent = 0.0f;
constexpr float kMaxOverspeedBrakeCapPercent = 100.0f;
constexpr float kMinOverspeedHysteresisMps = 0.0f;
constexpr float kMaxOverspeedHysteresisMps = 5.0f;
constexpr float kMinOverspeedBrakeSlewPctPerSec = 0.1f;
constexpr float kMaxOverspeedBrakeSlewPctPerSec = 1000.0f;
constexpr uint16_t kMinOverspeedBrakeHoldMs = 0u;
constexpr uint16_t kMaxOverspeedBrakeHoldMs = 5000u;
constexpr float kMinOverspeedBrakeDeadbandPercent = 0.0f;
constexpr float kMaxOverspeedBrakeDeadbandPercent = 100.0f;
constexpr float kBrakeThrottleInhibitThresholdPercent = 0.5f;
// Filtro liviano para velocidad usada por el control (no modifica Hall raw).
// Objetivo: amortiguar picos espurios sin introducir retardo fuerte.
constexpr float kMeasuredSpeedFilterTauSec = 0.045f;          // EMA corta (~45 ms)
constexpr float kMeasuredSpeedFilterMaxDeltaMpsPerSec = 20.0f;  // limitador de salto

constexpr const char* kPrefsNamespace = "speed_pid";
constexpr uint32_t kPrefsVersion = 4;
constexpr uint32_t kPrefsVersionV3 = 3;
constexpr uint32_t kPrefsVersionV2 = 2;
constexpr uint32_t kPrefsVersionLegacy = 1;
constexpr const char* kPrefsKeyVersion = "ver";
constexpr const char* kPrefsKeyKp = "kp";
constexpr const char* kPrefsKeyKi = "ki";
constexpr const char* kPrefsKeyKd = "kd";
constexpr const char* kPrefsKeyRamp = "ramp";
constexpr const char* kPrefsKeyMaxMps = "maxmps";
constexpr const char* kPrefsKeyIntegralLimit = "ilim";
constexpr const char* kPrefsKeyDeadband = "db";
constexpr const char* kPrefsKeyMinThrottle = "tmin";
constexpr const char* kPrefsKeyBrakeCap = "brkcap";
constexpr const char* kPrefsKeyHysteresis = "hys";
constexpr const char* kPrefsKeyBrakeSlewUp = "brsu";
constexpr const char* kPrefsKeyBrakeSlewDown = "brsd";
constexpr const char* kPrefsKeyBrakeHoldMs = "brhms";
constexpr const char* kPrefsKeyBrakeDeadband = "brdb";
constexpr const char* kPrefsKeyThrottleSlewUp = "thsup";
constexpr const char* kPrefsKeyThrottleSlewDown = "thsdown";
constexpr const char* kPrefsKeyMinThrottleAssistMaxSpeed = "minspd";
constexpr const char* kPrefsKeyLaunchAssistWindowMs = "lwin";
constexpr const char* kPrefsKeyThrottleBaseEnable = "ffen";
constexpr const char* kPrefsKeyThrottleBaseZero = "ffb0";
constexpr const char* kPrefsKeyThrottleBaseMax = "ffbmx";
constexpr const char* kPrefsKeyThrottleBaseDeltaUp = "ffdu";
constexpr const char* kPrefsKeyThrottleBaseDeltaDown = "ffdd";
constexpr const char* kPrefsKeyThrottleBaseMinSpeed = "ffmin";
constexpr const char* kPrefsKeyFeedbackLaunchGraceMs = "flgr";
constexpr const char* kPrefsKeyIntegratorUnwindGain = "iunw";
constexpr const char* kPrefsKeyDerivativeFilterHz = "dfhz";

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
  float measuredFilteredMps = 0.0f;
  float errorMps = 0.0f;
  float pTerm = 0.0f;
  float iTerm = 0.0f;
  float dTerm = 0.0f;
  float pidUnsatOutput = 0.0f;
  float pidSatOutput = 0.0f;
  float throttleBasePercent = 0.0f;
  float throttlePidDeltaPercent = 0.0f;
  float throttleCmdPreSlewPercent = 0.0f;
  bool throttleBaseActive = false;
  float throttleCmdRawPercent = 0.0f;
  float throttleCmdFilteredPercent = 0.0f;
  float overspeedErrorMps = 0.0f;
  float throttleCmdPercent = 0.0f;
  float brakeCmdPercent = 0.0f;
  float overspeedBrakeRawPercent = 0.0f;
  float overspeedBrakeFilteredPercent = 0.0f;
  bool throttleSaturated = false;
  bool integratorClamped = false;
  bool launchAssistActive = false;
  bool controlActive = false;
  float launchAssistRemainingSec = 0.0f;
  float feedbackLaunchGraceRemainingSec = 0.0f;
  float overspeedHoldRemainingSec = 0.0f;
  float prevMeasuredMpsForDerivative = 0.0f;
  float derivativeFiltered = 0.0f;
  bool derivativePrimed = false;
  bool measuredFilterPrimed = false;
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
  if (!isFiniteInRange(config.minThrottlePercent,
                       kMinMinThrottlePercent,
                       kMaxMinThrottlePercent)) {
    return false;
  }
  if (!isFiniteInRange(config.throttleSlewUpPctPerSec,
                       kMinThrottleSlewPctPerSec,
                       kMaxThrottleSlewPctPerSec)) {
    return false;
  }
  if (!isFiniteInRange(config.throttleSlewDownPctPerSec,
                       kMinThrottleSlewPctPerSec,
                       kMaxThrottleSlewPctPerSec)) {
    return false;
  }
  if (!isFiniteInRange(config.minThrottleAssistMaxSpeedMps,
                       kMinLaunchAssistMaxSpeedMps,
                       kMaxLaunchAssistMaxSpeedMps)) {
    return false;
  }
  if (config.launchAssistWindowMs < kMinLaunchAssistWindowMs ||
      config.launchAssistWindowMs > kMaxLaunchAssistWindowMs) {
    return false;
  }
  if (!isFiniteInRange(config.throttleBaseAtZeroMpsPercent,
                       kMinThrottleBasePercent,
                       kMaxThrottleBasePercent)) {
    return false;
  }
  if (!isFiniteInRange(config.throttleBaseAtMaxSpeedPercent,
                       kMinThrottleBasePercent,
                       kMaxThrottleBasePercent)) {
    return false;
  }
  if (!isFiniteInRange(config.throttleBasePidDeltaUpMaxPercent,
                       kMinThrottleBaseDeltaMaxPercent,
                       kMaxThrottleBaseDeltaMaxPercent)) {
    return false;
  }
  if (!isFiniteInRange(config.throttleBasePidDeltaDownMaxPercent,
                       kMinThrottleBaseDeltaMaxPercent,
                       kMaxThrottleBaseDeltaMaxPercent)) {
    return false;
  }
  if (!isFiniteInRange(config.throttleBaseActivationMinMps,
                       kMinThrottleBaseActivationMps,
                       kMaxThrottleBaseActivationMps)) {
    return false;
  }
  if (config.feedbackLaunchGraceMs < kMinFeedbackLaunchGraceMs ||
      config.feedbackLaunchGraceMs > kMaxFeedbackLaunchGraceMs) {
    return false;
  }
  if (!isFiniteInRange(config.integratorUnwindGain,
                       kMinIntegratorUnwindGain,
                       kMaxIntegratorUnwindGain)) {
    return false;
  }
  if (!isFiniteInRange(config.derivativeFilterHz, kMinDerivativeFilterHz, kMaxDerivativeFilterHz)) {
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
  if (!isFiniteInRange(config.overspeedBrakeSlewUpPctPerSec,
                       kMinOverspeedBrakeSlewPctPerSec,
                       kMaxOverspeedBrakeSlewPctPerSec)) {
    return false;
  }
  if (!isFiniteInRange(config.overspeedBrakeSlewDownPctPerSec,
                       kMinOverspeedBrakeSlewPctPerSec,
                       kMaxOverspeedBrakeSlewPctPerSec)) {
    return false;
  }
  if (config.overspeedBrakeHoldMs < kMinOverspeedBrakeHoldMs ||
      config.overspeedBrakeHoldMs > kMaxOverspeedBrakeHoldMs) {
    return false;
  }
  if (!isFiniteInRange(config.overspeedBrakeDeadbandPercent,
                       kMinOverspeedBrakeDeadbandPercent,
                       kMaxOverspeedBrakeDeadbandPercent)) {
    return false;
  }
  return true;
}

void resetControlTermsLocked() {
  g_state.integralTerm = 0.0f;
  g_state.prevError = 0.0f;
  g_state.firstRun = true;
  g_state.prevMeasuredMpsForDerivative = 0.0f;
  g_state.derivativeFiltered = 0.0f;
  g_state.derivativePrimed = false;
  g_state.measuredFilterPrimed = false;
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
  g_state.measuredFilteredMps = 0.0f;
  g_state.errorMps = 0.0f;
  g_state.pTerm = 0.0f;
  g_state.iTerm = 0.0f;
  g_state.dTerm = 0.0f;
  g_state.pidUnsatOutput = 0.0f;
  g_state.pidSatOutput = 0.0f;
  g_state.throttleBasePercent = 0.0f;
  g_state.throttlePidDeltaPercent = 0.0f;
  g_state.throttleCmdPreSlewPercent = 0.0f;
  g_state.throttleBaseActive = false;
  g_state.throttleCmdRawPercent = 0.0f;
  g_state.throttleCmdFilteredPercent = 0.0f;
  g_state.overspeedErrorMps = 0.0f;
  g_state.throttleCmdPercent = 0.0f;
  g_state.brakeCmdPercent = 0.0f;
  g_state.overspeedBrakeRawPercent = 0.0f;
  g_state.overspeedBrakeFilteredPercent = 0.0f;
  g_state.throttleSaturated = false;
  g_state.integratorClamped = false;
  g_state.launchAssistActive = false;
  g_state.controlActive = false;
  g_state.launchAssistRemainingSec = 0.0f;
  g_state.feedbackLaunchGraceRemainingSec = 0.0f;
  g_state.overspeedHoldRemainingSec = 0.0f;
}

bool loadFromNvs(SpeedPidTunings& tuningsOut, SpeedPidConfig& configOut, bool* needsPersistOut) {
  bool needsPersist = false;
  if (!g_prefs.begin(kPrefsNamespace, true)) {
    return false;
  }

  const uint32_t version = g_prefs.getUInt(kPrefsKeyVersion, 0);
  if (version != kPrefsVersion && version != kPrefsVersionV3 &&
      version != kPrefsVersionV2 && version != kPrefsVersionLegacy) {
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
  loadedConfig.minThrottlePercent = g_prefs.getFloat(kPrefsKeyMinThrottle, loadedConfig.minThrottlePercent);
  loadedConfig.overspeedBrakeMaxPercent =
      g_prefs.getFloat(kPrefsKeyBrakeCap, loadedConfig.overspeedBrakeMaxPercent);
  loadedConfig.overspeedReleaseHysteresisMps =
      g_prefs.getFloat(kPrefsKeyHysteresis, loadedConfig.overspeedReleaseHysteresisMps);
  if (version >= kPrefsVersionV2) {
    loadedConfig.overspeedBrakeSlewUpPctPerSec =
        g_prefs.getFloat(kPrefsKeyBrakeSlewUp, loadedConfig.overspeedBrakeSlewUpPctPerSec);
    loadedConfig.overspeedBrakeSlewDownPctPerSec =
        g_prefs.getFloat(kPrefsKeyBrakeSlewDown, loadedConfig.overspeedBrakeSlewDownPctPerSec);
    loadedConfig.overspeedBrakeHoldMs =
        static_cast<uint16_t>(g_prefs.getUShort(kPrefsKeyBrakeHoldMs, loadedConfig.overspeedBrakeHoldMs));
    loadedConfig.overspeedBrakeDeadbandPercent =
        g_prefs.getFloat(kPrefsKeyBrakeDeadband, loadedConfig.overspeedBrakeDeadbandPercent);
  } else {
    needsPersist = true;
  }
  if (version >= kPrefsVersionV3) {
    loadedConfig.throttleSlewUpPctPerSec =
        g_prefs.getFloat(kPrefsKeyThrottleSlewUp, loadedConfig.throttleSlewUpPctPerSec);
    loadedConfig.throttleSlewDownPctPerSec =
        g_prefs.getFloat(kPrefsKeyThrottleSlewDown, loadedConfig.throttleSlewDownPctPerSec);
    loadedConfig.minThrottleAssistMaxSpeedMps = g_prefs.getFloat(
        kPrefsKeyMinThrottleAssistMaxSpeed, loadedConfig.minThrottleAssistMaxSpeedMps);
    loadedConfig.launchAssistWindowMs = static_cast<uint16_t>(
        g_prefs.getUShort(kPrefsKeyLaunchAssistWindowMs, loadedConfig.launchAssistWindowMs));
    loadedConfig.integratorUnwindGain =
        g_prefs.getFloat(kPrefsKeyIntegratorUnwindGain, loadedConfig.integratorUnwindGain);
    loadedConfig.derivativeFilterHz =
        g_prefs.getFloat(kPrefsKeyDerivativeFilterHz, loadedConfig.derivativeFilterHz);
  } else {
    needsPersist = true;
  }
  if (version >= kPrefsVersion) {
    loadedConfig.throttleBaseEnable =
        g_prefs.getBool(kPrefsKeyThrottleBaseEnable, loadedConfig.throttleBaseEnable);
    loadedConfig.throttleBaseAtZeroMpsPercent =
        g_prefs.getFloat(kPrefsKeyThrottleBaseZero, loadedConfig.throttleBaseAtZeroMpsPercent);
    loadedConfig.throttleBaseAtMaxSpeedPercent =
        g_prefs.getFloat(kPrefsKeyThrottleBaseMax, loadedConfig.throttleBaseAtMaxSpeedPercent);
    loadedConfig.throttleBasePidDeltaUpMaxPercent =
        g_prefs.getFloat(kPrefsKeyThrottleBaseDeltaUp, loadedConfig.throttleBasePidDeltaUpMaxPercent);
    loadedConfig.throttleBasePidDeltaDownMaxPercent =
        g_prefs.getFloat(kPrefsKeyThrottleBaseDeltaDown, loadedConfig.throttleBasePidDeltaDownMaxPercent);
    loadedConfig.throttleBaseActivationMinMps =
        g_prefs.getFloat(kPrefsKeyThrottleBaseMinSpeed, loadedConfig.throttleBaseActivationMinMps);
    loadedConfig.feedbackLaunchGraceMs =
        static_cast<uint16_t>(g_prefs.getUShort(kPrefsKeyFeedbackLaunchGraceMs,
                                                loadedConfig.feedbackLaunchGraceMs));
  } else {
    needsPersist = true;
  }

  g_prefs.end();

  if (loadedConfig.maxSpeedMps > kMaxMaxSpeedMps) {
    loadedConfig.maxSpeedMps = kMaxMaxSpeedMps;
    needsPersist = true;
  }

  if (!validateTunings(loadedTunings) || !validateConfig(loadedConfig)) {
    return false;
  }

  tuningsOut = loadedTunings;
  configOut = loadedConfig;
  if (needsPersistOut != nullptr) {
    *needsPersistOut = needsPersist;
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
  g_prefs.putFloat(kPrefsKeyMinThrottle, config.minThrottlePercent);
  g_prefs.putFloat(kPrefsKeyThrottleSlewUp, config.throttleSlewUpPctPerSec);
  g_prefs.putFloat(kPrefsKeyThrottleSlewDown, config.throttleSlewDownPctPerSec);
  g_prefs.putFloat(kPrefsKeyMinThrottleAssistMaxSpeed, config.minThrottleAssistMaxSpeedMps);
  g_prefs.putUShort(kPrefsKeyLaunchAssistWindowMs, config.launchAssistWindowMs);
  g_prefs.putBool(kPrefsKeyThrottleBaseEnable, config.throttleBaseEnable);
  g_prefs.putFloat(kPrefsKeyThrottleBaseZero, config.throttleBaseAtZeroMpsPercent);
  g_prefs.putFloat(kPrefsKeyThrottleBaseMax, config.throttleBaseAtMaxSpeedPercent);
  g_prefs.putFloat(kPrefsKeyThrottleBaseDeltaUp, config.throttleBasePidDeltaUpMaxPercent);
  g_prefs.putFloat(kPrefsKeyThrottleBaseDeltaDown, config.throttleBasePidDeltaDownMaxPercent);
  g_prefs.putFloat(kPrefsKeyThrottleBaseMinSpeed, config.throttleBaseActivationMinMps);
  g_prefs.putUShort(kPrefsKeyFeedbackLaunchGraceMs, config.feedbackLaunchGraceMs);
  g_prefs.putFloat(kPrefsKeyIntegratorUnwindGain, config.integratorUnwindGain);
  g_prefs.putFloat(kPrefsKeyDerivativeFilterHz, config.derivativeFilterHz);
  g_prefs.putFloat(kPrefsKeyBrakeCap, config.overspeedBrakeMaxPercent);
  g_prefs.putFloat(kPrefsKeyHysteresis, config.overspeedReleaseHysteresisMps);
  g_prefs.putFloat(kPrefsKeyBrakeSlewUp, config.overspeedBrakeSlewUpPctPerSec);
  g_prefs.putFloat(kPrefsKeyBrakeSlewDown, config.overspeedBrakeSlewDownPctPerSec);
  g_prefs.putUShort(kPrefsKeyBrakeHoldMs, config.overspeedBrakeHoldMs);
  g_prefs.putFloat(kPrefsKeyBrakeDeadband, config.overspeedBrakeDeadbandPercent);
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
  bool nvsNeedsPersist = false;
  if (loadFromNvs(persistedTunings, persistedConfig, &nvsNeedsPersist)) {
    portENTER_CRITICAL(&g_speedPidMux);
    g_state.tunings = persistedTunings;
    g_state.config = persistedConfig;
    resetControllerLocked();
    portEXIT_CRITICAL(&g_speedPidMux);

    if (nvsNeedsPersist) {
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
  float prevBrakeFilteredPercent = 0.0f;
  float prevOverspeedHoldRemainingSec = 0.0f;
  float prevThrottleFilteredPercent = 0.0f;
  bool prevControlActive = false;
  float prevLaunchAssistRemainingSec = 0.0f;
  float prevFeedbackLaunchGraceRemainingSec = 0.0f;
  float prevMeasuredForDerivative = 0.0f;
  float prevDerivativeFiltered = 0.0f;
  bool prevDerivativePrimed = false;
  float prevMeasuredFilteredMps = 0.0f;
  bool prevMeasuredFilterPrimed = false;

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
  prevBrakeFilteredPercent = g_state.overspeedBrakeFilteredPercent;
  prevOverspeedHoldRemainingSec = g_state.overspeedHoldRemainingSec;
  prevThrottleFilteredPercent = g_state.throttleCmdFilteredPercent;
  prevControlActive = g_state.controlActive;
  prevLaunchAssistRemainingSec = g_state.launchAssistRemainingSec;
  prevFeedbackLaunchGraceRemainingSec = g_state.feedbackLaunchGraceRemainingSec;
  prevMeasuredForDerivative = g_state.prevMeasuredMpsForDerivative;
  prevDerivativeFiltered = g_state.derivativeFiltered;
  prevDerivativePrimed = g_state.derivativePrimed;
  prevMeasuredFilteredMps = g_state.measuredFilteredMps;
  prevMeasuredFilterPrimed = g_state.measuredFilterPrimed;
  portEXIT_CRITICAL(&g_speedPidMux);

  if (!isfinite(dtSeconds) || dtSeconds <= 0.0f || dtSeconds > 1.0f) {
    dtSeconds = 0.03f;
  }

  const float maxSpeed = config.maxSpeedMps;
  targetRawMps = isfinite(targetRawMps) ? clampf(targetRawMps, 0.0f, maxSpeed) : 0.0f;
  measuredMps = isfinite(measuredMps) ? clampf(measuredMps, 0.0f, maxSpeed) : 0.0f;
  const float measuredRawMps = measuredMps;

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
  float derivativeFiltered = prevDerivativeFiltered;
  float measuredForDerivative = prevMeasuredForDerivative;
  bool derivativePrimed = prevDerivativePrimed;
  float measuredFilteredMps = prevMeasuredFilteredMps;
  bool measuredFilterPrimed = prevMeasuredFilterPrimed;
  float launchAssistRemainingSec = prevLaunchAssistRemainingSec;
  float feedbackLaunchGraceRemainingSec = prevFeedbackLaunchGraceRemainingSec;
  bool controlActive = prevControlActive;

  float throttleRawPercent = 0.0f;
  float throttleFilteredPercent = prevThrottleFilteredPercent;
  float throttlePercent = 0.0f;
  float brakePercent = 0.0f;
  float overspeedBrakeRawPercent = 0.0f;
  float overspeedBrakeFilteredPercent = prevBrakeFilteredPercent;
  float errorMps = 0.0f;
  float pTerm = 0.0f;
  float iTerm = 0.0f;
  float dTerm = 0.0f;
  float pidUnsatOutput = 0.0f;
  float pidSatOutput = 0.0f;
  float throttleBasePercent = 0.0f;
  float throttlePidDeltaPercent = 0.0f;
  float throttleCmdPreSlewPercent = 0.0f;
  bool throttleBaseActive = false;
  float overspeedErrorMps = 0.0f;
  bool throttleSaturated = false;
  bool integratorClamped = false;
  bool launchAssistActive = false;
  SpeedPidMode mode = prevMode;
  float overspeedHoldRemainingSec =
      (prevOverspeedHoldRemainingSec > 0.0f) ? prevOverspeedHoldRemainingSec : 0.0f;
  const float overspeedHoldConfigSec = static_cast<float>(config.overspeedBrakeHoldMs) * 0.001f;
  const float launchAssistWindowSec = static_cast<float>(config.launchAssistWindowMs) * 0.001f;
  const float feedbackLaunchGraceSec =
      static_cast<float>(config.feedbackLaunchGraceMs) * 0.001f;
  const bool requestActiveNow = targetRawMps > 0.05f;

  // Filtro de medicion: limitador de salto + EMA corta.
  // Mantiene el dato raw para logs/diagnostico; el control usa el filtrado.
  if (!measuredFilterPrimed || !isfinite(measuredFilteredMps)) {
    measuredFilteredMps = measuredRawMps;
    measuredFilterPrimed = true;
  } else {
    const float maxStepMps = kMeasuredSpeedFilterMaxDeltaMpsPerSec * dtSeconds;
    const float limitedInput =
        measuredFilteredMps + clampf(measuredRawMps - measuredFilteredMps, -maxStepMps, maxStepMps);
    const float tauSec = (kMeasuredSpeedFilterTauSec > 0.0f) ? kMeasuredSpeedFilterTauSec : 0.001f;
    const float alpha = clampf(dtSeconds / (tauSec + dtSeconds), 0.02f, 1.0f);
    measuredFilteredMps += alpha * (limitedInput - measuredFilteredMps);
    measuredFilteredMps = clampf(measuredFilteredMps, 0.0f, maxSpeed);
  }

  if (requestActiveNow && !controlActive) {
    launchAssistRemainingSec = launchAssistWindowSec;
    feedbackLaunchGraceRemainingSec = feedbackLaunchGraceSec;
    controlActive = true;
  } else if (!requestActiveNow) {
    launchAssistRemainingSec = 0.0f;
    feedbackLaunchGraceRemainingSec = 0.0f;
    controlActive = false;
  }

  const bool feedbackGraceActive =
      (!feedbackOk) && requestActiveNow && (feedbackLaunchGraceRemainingSec > 0.0f);

  if (!feedbackOk && !feedbackGraceActive) {
    mode = SpeedPidMode::kFailsafe;
    targetRampedMps = 0.0f;
    integral = 0.0f;
    pidPrevError = 0.0f;
    firstRun = true;
    overspeedBrakeRawPercent = 0.0f;
    overspeedBrakeFilteredPercent = 0.0f;
    overspeedHoldRemainingSec = 0.0f;
    pTerm = 0.0f;
    iTerm = 0.0f;
    dTerm = 0.0f;
    pidUnsatOutput = 0.0f;
    pidSatOutput = 0.0f;
    throttleRawPercent = 0.0f;
    throttleFilteredPercent = 0.0f;
    launchAssistRemainingSec = 0.0f;
    feedbackLaunchGraceRemainingSec = 0.0f;
    controlActive = false;
    derivativeFiltered = 0.0f;
    measuredForDerivative = measuredFilteredMps;
    derivativePrimed = false;
  } else {
    const bool overspeedEntry = measuredFilteredMps > targetRampedMps;
    const bool overspeedExitByHysteresis =
        measuredFilteredMps <= (targetRampedMps + config.overspeedReleaseHysteresisMps);

    if (prevMode == SpeedPidMode::kOverspeed) {
      if (overspeedHoldRemainingSec > 0.0f) {
        overspeedHoldRemainingSec -= dtSeconds;
        if (overspeedHoldRemainingSec < 0.0f) {
          overspeedHoldRemainingSec = 0.0f;
        }
      }
      mode = (overspeedExitByHysteresis && overspeedHoldRemainingSec <= 0.0f)
                 ? SpeedPidMode::kNormal
                 : SpeedPidMode::kOverspeed;
    } else if (overspeedEntry) {
      mode = SpeedPidMode::kOverspeed;
      overspeedHoldRemainingSec = overspeedHoldConfigSec;
    } else {
      mode = SpeedPidMode::kNormal;
      overspeedHoldRemainingSec = 0.0f;
    }

    if (mode != prevMode) {
      integral = 0.0f;
      pidPrevError = 0.0f;
      firstRun = true;
      derivativeFiltered = 0.0f;
      measuredForDerivative = measuredFilteredMps;
      derivativePrimed = false;
      if (mode != SpeedPidMode::kNormal) {
        launchAssistRemainingSec = 0.0f;
        controlActive = false;
      }
    }

    if (mode == SpeedPidMode::kOverspeed) {
      if (prevMode != SpeedPidMode::kOverspeed) {
        overspeedHoldRemainingSec = overspeedHoldConfigSec;
      }
      overspeedErrorMps = measuredFilteredMps - targetRampedMps;
      if (overspeedErrorMps < 0.0f) {
        overspeedErrorMps = 0.0f;
      }
      const float brakeLinear =
          (overspeedErrorMps * 100.0f) / ((maxSpeed > 0.1f) ? maxSpeed : 0.1f);
      overspeedBrakeRawPercent = clampf(brakeLinear, 0.0f, config.overspeedBrakeMaxPercent);
      if (overspeedBrakeRawPercent < config.overspeedBrakeDeadbandPercent) {
        overspeedBrakeRawPercent = 0.0f;
      }
      throttleRawPercent = 0.0f;
      throttleBasePercent = 0.0f;
      throttlePidDeltaPercent = 0.0f;
      throttleCmdPreSlewPercent = 0.0f;
      throttleBaseActive = false;
      errorMps = targetRampedMps - measuredFilteredMps;
      pTerm = 0.0f;
      iTerm = 0.0f;
      dTerm = 0.0f;
      pidUnsatOutput = 0.0f;
      pidSatOutput = 0.0f;
      throttleSaturated = false;
      integratorClamped = true;
      launchAssistActive = false;
      launchAssistRemainingSec = 0.0f;
      controlActive = false;
    } else {
      overspeedBrakeRawPercent = 0.0f;
      overspeedHoldRemainingSec = 0.0f;
      errorMps = targetRampedMps - measuredFilteredMps;
      if (fabsf(errorMps) < config.deadbandMps) {
        errorMps = 0.0f;
      }

      float dMeas = 0.0f;
      if (derivativePrimed && dtSeconds > 0.0f) {
        dMeas = (measuredFilteredMps - measuredForDerivative) / dtSeconds;
      }
      measuredForDerivative = measuredFilteredMps;
      derivativePrimed = true;

      const float cutoffHz = (config.derivativeFilterHz > 0.0f) ? config.derivativeFilterHz : 0.0f;
      if (cutoffHz > 0.0f) {
        const float rc = 1.0f / (2.0f * 3.14159265359f * cutoffHz);
        const float alpha = dtSeconds / (rc + dtSeconds);
        derivativeFiltered += alpha * (dMeas - derivativeFiltered);
      } else {
        derivativeFiltered = dMeas;
      }

      pTerm = tunings.kp * errorMps;
      dTerm = -tunings.kd * derivativeFiltered;
      iTerm = integral;
      const bool baseEnabled = config.throttleBaseEnable && requestActiveNow;
      if (baseEnabled && targetRampedMps > config.throttleBaseActivationMinMps) {
        const float baseAt0 = clampf(config.throttleBaseAtZeroMpsPercent, 0.0f, 100.0f);
        const float baseAtMax = clampf(config.throttleBaseAtMaxSpeedPercent, 0.0f, 100.0f);
        const float denom = (maxSpeed > 0.001f) ? maxSpeed : 0.001f;
        const float ratio = clampf(targetRampedMps / denom, 0.0f, 1.0f);
        throttleBasePercent = baseAt0 + (baseAtMax - baseAt0) * ratio;
        throttleBasePercent = clampf(throttleBasePercent, 0.0f, 100.0f);
        throttleBaseActive = true;
      } else {
        throttleBasePercent = 0.0f;
        throttleBaseActive = false;
      }

      float pidPreOutput = pTerm + iTerm + dTerm;
      float pidSatMin = kOutputMinPercent;
      float pidSatMax = kOutputMaxPercent;
      if (throttleBaseActive) {
        const float upMax = clampf(config.throttleBasePidDeltaUpMaxPercent, 0.0f, 100.0f);
        const float downMax = clampf(config.throttleBasePidDeltaDownMaxPercent, 0.0f, 100.0f);
        pidSatMax = clampf(fminf(upMax, 100.0f - throttleBasePercent), 0.0f, 100.0f);
        pidSatMin = -clampf(fminf(downMax, throttleBasePercent), 0.0f, 100.0f);
      }
      const bool saturatedHigh = pidPreOutput > pidSatMax;
      const bool saturatedLow = pidPreOutput < pidSatMin;
      const bool pushingFurtherIntoSaturation =
          (saturatedHigh && errorMps > 0.0f) || (saturatedLow && errorMps < 0.0f);

      if (!(saturatedHigh || saturatedLow) || !pushingFurtherIntoSaturation) {
        integral += tunings.ki * errorMps * dtSeconds;
      } else {
        integral -= (config.integratorUnwindGain * integral * dtSeconds);
      }

      const float unclampedIntegral = integral;
      integral = clampf(integral, -config.integralLimit, config.integralLimit);
      if (fabsf(unclampedIntegral - integral) > 1e-6f) {
        integratorClamped = true;
      }
      if ((saturatedHigh || saturatedLow) && pushingFurtherIntoSaturation) {
        integratorClamped = true;
      }

      iTerm = integral;
      pidUnsatOutput = pTerm + iTerm + dTerm;
      pidSatOutput = clampf(pidUnsatOutput, pidSatMin, pidSatMax);
      throttleSaturated = fabsf(pidUnsatOutput - pidSatOutput) > 1e-4f;
      throttlePidDeltaPercent = pidSatOutput;
      throttleCmdPreSlewPercent = throttleBaseActive ? (throttleBasePercent + throttlePidDeltaPercent)
                                                     : pidSatOutput;
      throttleRawPercent = clampf(throttleCmdPreSlewPercent, kOutputMinPercent, kOutputMaxPercent);

      const bool canUseLaunchAssist =
          !throttleBaseActive &&
          controlActive && launchAssistRemainingSec > 0.0f &&
          measuredFilteredMps <= config.minThrottleAssistMaxSpeedMps &&
          errorMps > (config.deadbandMps + 0.05f) &&
          config.minThrottlePercent > 0.0f;
      if (canUseLaunchAssist) {
        const float minThrottle = clampf(config.minThrottlePercent, 0.0f, 100.0f);
        if (throttleRawPercent < minThrottle) {
          throttleRawPercent = minThrottle;
        }
        launchAssistActive = true;
        throttleCmdPreSlewPercent = throttleRawPercent;
      }
      if (requestActiveNow) {
        if (launchAssistRemainingSec > 0.0f) {
          launchAssistRemainingSec -= dtSeconds;
          if (launchAssistRemainingSec < 0.0f) {
            launchAssistRemainingSec = 0.0f;
          }
        }
        if (feedbackLaunchGraceRemainingSec > 0.0f) {
          feedbackLaunchGraceRemainingSec -= dtSeconds;
          if (feedbackLaunchGraceRemainingSec < 0.0f) {
            feedbackLaunchGraceRemainingSec = 0.0f;
          }
        }
      }
      if (feedbackGraceActive && !launchAssistActive) {
        // Reuse existing launch fields for observability of startup grace.
        launchAssistActive = true;
      }

      pidPrevError = errorMps;
      firstRun = false;
    }

    // Aplicar slew de throttle tanto en NORMAL como en OVERSPEED.
    // En FAILSAFE se conserva corte inmediato.
    const float throttleUpStep = config.throttleSlewUpPctPerSec * dtSeconds;
    const float throttleDownStep = config.throttleSlewDownPctPerSec * dtSeconds;
    if (throttleRawPercent > throttleFilteredPercent) {
      const float delta = throttleRawPercent - throttleFilteredPercent;
      throttleFilteredPercent += (delta < throttleUpStep) ? delta : throttleUpStep;
    } else if (throttleRawPercent < throttleFilteredPercent) {
      const float delta = throttleFilteredPercent - throttleRawPercent;
      throttleFilteredPercent -= (delta < throttleDownStep) ? delta : throttleDownStep;
    }
    throttleFilteredPercent = clampf(throttleFilteredPercent, kOutputMinPercent, kOutputMaxPercent);
    if (throttleFilteredPercent < 0.01f) {
      throttleFilteredPercent = 0.0f;
    }
    throttlePercent = throttleFilteredPercent;

    const float brakeSlewUpStep = config.overspeedBrakeSlewUpPctPerSec * dtSeconds;
    const float brakeSlewDownStep = config.overspeedBrakeSlewDownPctPerSec * dtSeconds;
    if (overspeedBrakeRawPercent > overspeedBrakeFilteredPercent) {
      const float delta = overspeedBrakeRawPercent - overspeedBrakeFilteredPercent;
      overspeedBrakeFilteredPercent += (delta < brakeSlewUpStep) ? delta : brakeSlewUpStep;
    } else if (overspeedBrakeRawPercent < overspeedBrakeFilteredPercent) {
      const float delta = overspeedBrakeFilteredPercent - overspeedBrakeRawPercent;
      overspeedBrakeFilteredPercent -= (delta < brakeSlewDownStep) ? delta : brakeSlewDownStep;
    }
    overspeedBrakeFilteredPercent =
        clampf(overspeedBrakeFilteredPercent, 0.0f, config.overspeedBrakeMaxPercent);
    if (overspeedBrakeFilteredPercent < 0.01f) {
      overspeedBrakeFilteredPercent = 0.0f;
    }
    brakePercent = overspeedBrakeFilteredPercent;
    if (brakePercent > kBrakeThrottleInhibitThresholdPercent) {
      throttleRawPercent = 0.0f;
      throttleFilteredPercent = 0.0f;
      throttlePercent = 0.0f;
      throttleCmdPreSlewPercent = 0.0f;
    }
  }

  output.throttlePercent = throttlePercent;
  output.brakePercent = brakePercent;
  output.targetRampedMps = targetRampedMps;
  output.measuredRawMps = measuredRawMps;
  output.measuredFilteredMps = measuredFilteredMps;
  output.errorMps = errorMps;
  output.pTerm = pTerm;
  output.iTerm = iTerm;
  output.dTerm = dTerm;
  output.pidUnsatOutput = pidUnsatOutput;
  output.pidSatOutput = pidSatOutput;
  output.throttleBasePercent = throttleBasePercent;
  output.throttlePidDeltaPercent = throttlePidDeltaPercent;
  output.throttleCmdPreSlewPercent = throttleCmdPreSlewPercent;
  output.throttleBaseActive = throttleBaseActive;
  output.throttleCmdRawPercent = throttleRawPercent;
  output.throttleCmdFilteredPercent = throttleFilteredPercent;
  output.throttleSaturated = throttleSaturated;
  output.integratorClamped = integratorClamped;
  output.launchAssistActive = launchAssistActive;
  output.launchAssistRemainingMs =
      static_cast<uint16_t>(
          clampf(fmaxf(launchAssistRemainingSec, feedbackLaunchGraceRemainingSec) * 1000.0f,
                 0.0f,
                 65535.0f));
  output.overspeedErrorMps = overspeedErrorMps;
  output.overspeedBrakeRawPercent = overspeedBrakeRawPercent;
  output.overspeedBrakeFilteredPercent = overspeedBrakeFilteredPercent;
  output.overspeedHoldActive =
      (mode == SpeedPidMode::kOverspeed) && (overspeedHoldRemainingSec > 0.0f);
  output.overspeedHoldRemainingMs =
      static_cast<uint16_t>(clampf(overspeedHoldRemainingSec * 1000.0f, 0.0f, 65535.0f));
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
  g_state.measuredMps = measuredRawMps;
  g_state.measuredFilteredMps = measuredFilteredMps;
  g_state.errorMps = errorMps;
  g_state.pTerm = pTerm;
  g_state.iTerm = iTerm;
  g_state.dTerm = dTerm;
  g_state.pidUnsatOutput = pidUnsatOutput;
  g_state.pidSatOutput = pidSatOutput;
  g_state.throttleBasePercent = throttleBasePercent;
  g_state.throttlePidDeltaPercent = throttlePidDeltaPercent;
  g_state.throttleCmdPreSlewPercent = throttleCmdPreSlewPercent;
  g_state.throttleBaseActive = throttleBaseActive;
  g_state.throttleCmdRawPercent = throttleRawPercent;
  g_state.throttleCmdFilteredPercent = throttleFilteredPercent;
  g_state.overspeedErrorMps = overspeedErrorMps;
  g_state.throttleCmdPercent = throttlePercent;
  g_state.brakeCmdPercent = brakePercent;
  g_state.overspeedBrakeRawPercent = overspeedBrakeRawPercent;
  g_state.overspeedBrakeFilteredPercent = overspeedBrakeFilteredPercent;
  g_state.throttleSaturated = throttleSaturated;
  g_state.integratorClamped = integratorClamped;
  g_state.launchAssistActive = launchAssistActive;
  g_state.controlActive = controlActive;
  g_state.launchAssistRemainingSec = launchAssistRemainingSec;
  g_state.feedbackLaunchGraceRemainingSec = feedbackLaunchGraceRemainingSec;
  g_state.overspeedHoldRemainingSec = overspeedHoldRemainingSec;
  g_state.integralTerm = integral;
  g_state.prevError = pidPrevError;
  g_state.firstRun = firstRun;
  g_state.prevMeasuredMpsForDerivative = measuredForDerivative;
  g_state.derivativeFiltered = derivativeFiltered;
  g_state.derivativePrimed = derivativePrimed;
  g_state.measuredFilterPrimed = measuredFilterPrimed;
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

bool speedPidSetMinThrottlePercent(float minThrottlePercent) {
  if (!isFiniteInRange(minThrottlePercent, kMinMinThrottlePercent, kMaxMinThrottlePercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.minThrottlePercent = minThrottlePercent;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleSlewUpPctPerSec(float slewUpPctPerSec) {
  if (!isFiniteInRange(slewUpPctPerSec, kMinThrottleSlewPctPerSec, kMaxThrottleSlewPctPerSec)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleSlewUpPctPerSec = slewUpPctPerSec;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleSlewDownPctPerSec(float slewDownPctPerSec) {
  if (!isFiniteInRange(slewDownPctPerSec, kMinThrottleSlewPctPerSec, kMaxThrottleSlewPctPerSec)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleSlewDownPctPerSec = slewDownPctPerSec;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetMinThrottleAssistMaxSpeedMps(float maxSpeedMps) {
  if (!isFiniteInRange(maxSpeedMps, kMinLaunchAssistMaxSpeedMps, kMaxLaunchAssistMaxSpeedMps)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.minThrottleAssistMaxSpeedMps = maxSpeedMps;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetLaunchAssistWindowMs(uint16_t windowMs) {
  if (windowMs < kMinLaunchAssistWindowMs || windowMs > kMaxLaunchAssistWindowMs) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.launchAssistWindowMs = windowMs;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleBaseEnable(bool enabled) {
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleBaseEnable = enabled;
  resetControlTermsLocked();
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleBaseAtZeroMpsPercent(float percent) {
  if (!isFiniteInRange(percent, kMinThrottleBasePercent, kMaxThrottleBasePercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleBaseAtZeroMpsPercent = percent;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleBaseAtMaxSpeedPercent(float percent) {
  if (!isFiniteInRange(percent, kMinThrottleBasePercent, kMaxThrottleBasePercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleBaseAtMaxSpeedPercent = percent;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleBasePidDeltaUpMaxPercent(float percent) {
  if (!isFiniteInRange(percent, kMinThrottleBaseDeltaMaxPercent, kMaxThrottleBaseDeltaMaxPercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleBasePidDeltaUpMaxPercent = percent;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleBasePidDeltaDownMaxPercent(float percent) {
  if (!isFiniteInRange(percent, kMinThrottleBaseDeltaMaxPercent, kMaxThrottleBaseDeltaMaxPercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleBasePidDeltaDownMaxPercent = percent;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetThrottleBaseActivationMinMps(float mps) {
  if (!isFiniteInRange(mps, kMinThrottleBaseActivationMps, kMaxThrottleBaseActivationMps)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.throttleBaseActivationMinMps = mps;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetFeedbackLaunchGraceMs(uint16_t graceMs) {
  if (graceMs < kMinFeedbackLaunchGraceMs || graceMs > kMaxFeedbackLaunchGraceMs) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.feedbackLaunchGraceMs = graceMs;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetIntegratorUnwindGain(float unwindGain) {
  if (!isFiniteInRange(unwindGain, kMinIntegratorUnwindGain, kMaxIntegratorUnwindGain)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.integratorUnwindGain = unwindGain;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetDerivativeFilterHz(float cutoffHz) {
  if (!isFiniteInRange(cutoffHz, kMinDerivativeFilterHz, kMaxDerivativeFilterHz)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.derivativeFilterHz = cutoffHz;
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

bool speedPidSetOverspeedBrakeSlewUpPctPerSec(float slewUpPctPerSec) {
  if (!isFiniteInRange(slewUpPctPerSec,
                       kMinOverspeedBrakeSlewPctPerSec,
                       kMaxOverspeedBrakeSlewPctPerSec)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.overspeedBrakeSlewUpPctPerSec = slewUpPctPerSec;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetOverspeedBrakeSlewDownPctPerSec(float slewDownPctPerSec) {
  if (!isFiniteInRange(slewDownPctPerSec,
                       kMinOverspeedBrakeSlewPctPerSec,
                       kMaxOverspeedBrakeSlewPctPerSec)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.overspeedBrakeSlewDownPctPerSec = slewDownPctPerSec;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetOverspeedBrakeHoldMs(uint16_t holdMs) {
  if (holdMs < kMinOverspeedBrakeHoldMs || holdMs > kMaxOverspeedBrakeHoldMs) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.overspeedBrakeHoldMs = holdMs;
  portEXIT_CRITICAL(&g_speedPidMux);
  return true;
}

bool speedPidSetOverspeedBrakeDeadbandPercent(float deadbandPercent) {
  if (!isFiniteInRange(deadbandPercent,
                       kMinOverspeedBrakeDeadbandPercent,
                       kMaxOverspeedBrakeDeadbandPercent)) {
    return false;
  }
  portENTER_CRITICAL(&g_speedPidMux);
  if (!g_state.initialized) {
    portEXIT_CRITICAL(&g_speedPidMux);
    return false;
  }
  g_state.config.overspeedBrakeDeadbandPercent = deadbandPercent;
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
  snapshot.measuredFilteredMps = g_state.measuredFilteredMps;
  snapshot.errorMps = g_state.errorMps;
  snapshot.pTerm = g_state.pTerm;
  snapshot.iTerm = g_state.iTerm;
  snapshot.dTerm = g_state.dTerm;
  snapshot.pidUnsatOutput = g_state.pidUnsatOutput;
  snapshot.pidSatOutput = g_state.pidSatOutput;
  snapshot.throttleBasePercent = g_state.throttleBasePercent;
  snapshot.throttlePidDeltaPercent = g_state.throttlePidDeltaPercent;
  snapshot.throttleCmdPreSlewPercent = g_state.throttleCmdPreSlewPercent;
  snapshot.throttleBaseActive = g_state.throttleBaseActive;
  snapshot.overspeedErrorMps = g_state.overspeedErrorMps;
  snapshot.throttleCmdPercent = g_state.throttleCmdPercent;
  snapshot.throttleCmdRawPercent = g_state.throttleCmdRawPercent;
  snapshot.throttleCmdFilteredPercent = g_state.throttleCmdFilteredPercent;
  snapshot.brakeCmdPercent = g_state.brakeCmdPercent;
  snapshot.overspeedBrakeRawPercent = g_state.overspeedBrakeRawPercent;
  snapshot.overspeedBrakeFilteredPercent = g_state.overspeedBrakeFilteredPercent;
  snapshot.throttleSaturated = g_state.throttleSaturated;
  snapshot.integratorClamped = g_state.integratorClamped;
  snapshot.launchAssistActive = g_state.launchAssistActive;
  snapshot.launchAssistRemainingMs =
      static_cast<uint16_t>(clampf(g_state.launchAssistRemainingSec * 1000.0f, 0.0f, 65535.0f));
  snapshot.overspeedHoldActive =
      (g_state.mode == SpeedPidMode::kOverspeed) && (g_state.overspeedHoldRemainingSec > 0.0f);
  snapshot.overspeedHoldRemainingMs =
      static_cast<uint16_t>(clampf(g_state.overspeedHoldRemainingSec * 1000.0f, 0.0f, 65535.0f));
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

float speedPidGetMinThrottlePercent() {
  portENTER_CRITICAL(&g_speedPidMux);
  const float minThrottle = g_state.initialized ? g_state.config.minThrottlePercent : 0.0f;
  portEXIT_CRITICAL(&g_speedPidMux);
  return minThrottle;
}
