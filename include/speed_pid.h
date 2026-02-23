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
  float overspeedBrakeMaxPercent;
  float overspeedReleaseHysteresisMps;
};

struct SpeedPidControlOutput {
  float throttlePercent;
  float brakePercent;
  float targetRampedMps;
  float errorMps;
  float overspeedErrorMps;
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
  float errorMps;
  float overspeedErrorMps;
  float throttleCmdPercent;
  float brakeCmdPercent;
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
bool speedPidSetOverspeedBrakeMaxPercent(float brakeCapPercent);
bool speedPidSetOverspeedReleaseHysteresisMps(float hysteresisMps);

bool speedPidSaveToNvs();
bool speedPidResetToDefaults(bool persistToNvs);

bool speedPidGetSnapshot(SpeedPidRuntimeSnapshot& snapshot);
float speedPidGetMaxSpeedMps();
const char* speedPidModeText(SpeedPidMode mode);

#endif  // SPEED_PID_H
