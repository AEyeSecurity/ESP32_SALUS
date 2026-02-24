#ifndef SPEED_PID_H
#define SPEED_PID_H

#include <Arduino.h>

#include <stdint.h>

enum class SpeedPidMode : uint8_t {
  kNormal = 0,
  kOverspeed,
  kFailsafe,
};

struct SpeedPidTunings {
  float kp;
  float ki;
  float kd;
};

struct SpeedPidConfig {
  float maxSpeedMps;
  float maxSetpointRateMps2;
  float integralLimit;
  float deadbandMps;
  float minThrottlePercent;
  float throttleSlewUpPctPerSec;
  float throttleSlewDownPctPerSec;
  float minThrottleAssistMaxSpeedMps;
  uint16_t launchAssistWindowMs;
  bool throttleBaseEnable;
  float throttleBaseAtZeroMpsPercent;
  float throttleBaseAtMaxSpeedPercent;
  float throttleBasePidDeltaUpMaxPercent;
  float throttleBasePidDeltaDownMaxPercent;
  float throttleBaseActivationMinMps;
  uint16_t feedbackLaunchGraceMs;
  float integratorUnwindGain;
  float derivativeFilterHz;
  float overspeedBrakeMaxPercent;
  float overspeedReleaseHysteresisMps;
  float overspeedBrakeSlewUpPctPerSec;
  float overspeedBrakeSlewDownPctPerSec;
  uint16_t overspeedBrakeHoldMs;
  float overspeedBrakeDeadbandPercent;
};

struct SpeedPidControlOutput {
  float throttlePercent;
  float brakePercent;
  float targetRampedMps;
  float measuredRawMps;
  float measuredFilteredMps;
  float errorMps;
  float pTerm;
  float iTerm;
  float dTerm;
  float pidUnsatOutput;
  float pidSatOutput;
  float throttleBasePercent;
  float throttlePidDeltaPercent;
  float throttleCmdPreSlewPercent;
  bool throttleBaseActive;
  float throttleCmdRawPercent;
  float throttleCmdFilteredPercent;
  float overspeedErrorMps;
  float overspeedBrakeRawPercent;
  float overspeedBrakeFilteredPercent;
  bool throttleSaturated;
  bool integratorClamped;
  bool launchAssistActive;
  uint16_t launchAssistRemainingMs;
  bool overspeedHoldActive;
  uint16_t overspeedHoldRemainingMs;
  bool feedbackOk;
  bool failsafeActive;
  bool overspeedActive;
  SpeedPidMode mode;
};

struct SpeedPidRuntimeSnapshot {
  bool initialized;
  bool enabled;
  bool feedbackOk;
  bool failsafeActive;
  bool overspeedActive;
  SpeedPidMode mode;
  float targetRawMps;
  float targetRampedMps;
  float measuredMps;
  float measuredFilteredMps;
  float errorMps;
  float pTerm;
  float iTerm;
  float dTerm;
  float pidUnsatOutput;
  float pidSatOutput;
  float throttleBasePercent;
  float throttlePidDeltaPercent;
  float throttleCmdPreSlewPercent;
  bool throttleBaseActive;
  float overspeedErrorMps;
  float throttleCmdPercent;
  float throttleCmdRawPercent;
  float throttleCmdFilteredPercent;
  float brakeCmdPercent;
  float overspeedBrakeRawPercent;
  float overspeedBrakeFilteredPercent;
  bool throttleSaturated;
  bool integratorClamped;
  bool launchAssistActive;
  uint16_t launchAssistRemainingMs;
  bool overspeedHoldActive;
  uint16_t overspeedHoldRemainingMs;
  SpeedPidTunings tunings;
  SpeedPidConfig config;
};

bool speedPidInit(const SpeedPidTunings& defaultTunings, const SpeedPidConfig& defaultConfig);
bool speedPidCompute(float targetRawMps,
                     float measuredMps,
                     bool feedbackOk,
                     float dtSeconds,
                     SpeedPidControlOutput& output);
void speedPidReset();

bool speedPidGetTunings(SpeedPidTunings& tunings);
bool speedPidSetTunings(const SpeedPidTunings& tunings);

bool speedPidGetConfig(SpeedPidConfig& config);
bool speedPidSetMaxSpeedMps(float maxSpeedMps);
bool speedPidSetRampRateMps2(float rampRateMps2);
bool speedPidSetMinThrottlePercent(float minThrottlePercent);
bool speedPidSetThrottleSlewUpPctPerSec(float slewUpPctPerSec);
bool speedPidSetThrottleSlewDownPctPerSec(float slewDownPctPerSec);
bool speedPidSetMinThrottleAssistMaxSpeedMps(float maxSpeedMps);
bool speedPidSetLaunchAssistWindowMs(uint16_t windowMs);
bool speedPidSetThrottleBaseEnable(bool enabled);
bool speedPidSetThrottleBaseAtZeroMpsPercent(float percent);
bool speedPidSetThrottleBaseAtMaxSpeedPercent(float percent);
bool speedPidSetThrottleBasePidDeltaUpMaxPercent(float percent);
bool speedPidSetThrottleBasePidDeltaDownMaxPercent(float percent);
bool speedPidSetThrottleBaseActivationMinMps(float mps);
bool speedPidSetFeedbackLaunchGraceMs(uint16_t graceMs);
bool speedPidSetIntegratorUnwindGain(float unwindGain);
bool speedPidSetDerivativeFilterHz(float cutoffHz);
bool speedPidSetOverspeedBrakeMaxPercent(float brakeCapPercent);
bool speedPidSetOverspeedReleaseHysteresisMps(float hysteresisMps);
bool speedPidSetOverspeedBrakeSlewUpPctPerSec(float slewUpPctPerSec);
bool speedPidSetOverspeedBrakeSlewDownPctPerSec(float slewDownPctPerSec);
bool speedPidSetOverspeedBrakeHoldMs(uint16_t holdMs);
bool speedPidSetOverspeedBrakeDeadbandPercent(float deadbandPercent);

bool speedPidSaveToNvs();
bool speedPidResetToDefaults(bool persistToNvs);

bool speedPidGetSnapshot(SpeedPidRuntimeSnapshot& snapshot);
float speedPidGetMaxSpeedMps();
float speedPidGetMinThrottlePercent();
const char* speedPidModeText(SpeedPidMode mode);

#endif  // SPEED_PID_H
