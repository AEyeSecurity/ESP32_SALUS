#ifndef QUAD_FUNCTIONS_H
#define QUAD_FUNCTIONS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include "speed_pid.h"

struct QuadThrottleConfig {
  uint8_t pwmPin;
  uint8_t ledcChannel;
  uint32_t pwmFrequencyHz;
  uint8_t pwmResolutionBits;
  int pwmMinDuty;
  int pwmMaxDuty;
  int activationThreshold;
};

void initQuadThrottle(const QuadThrottleConfig& config);
int quadThrottleUpdate(int rcValue);
void quadThrottleStop();

struct QuadBrakeConfig {
  uint8_t servoPinA;
  uint8_t servoPinB;
  uint8_t ledcChannelA;
  uint8_t ledcChannelB;
  uint32_t pwmFrequencyHz;
  uint8_t pwmResolutionBits;
  int releaseAngleServoADeg;
  int brakeAngleServoADeg;
  int releaseAngleServoBDeg;
  int brakeAngleServoBDeg;
  int activationThreshold;  // negative threshold (e.g. -15)
};

void initQuadBrake(const QuadBrakeConfig& config);
void quadBrakeUpdate(int rcValue);
void quadBrakeRelease();

struct QuadDriveTaskConfig {
  QuadThrottleConfig throttle;
  QuadBrakeConfig brake;
  SpeedPidTunings speedPidTuningsDefaults;
  SpeedPidConfig speedPidConfigDefaults;
  bool autoInitHardware;
  TickType_t period;
  bool log;
};

struct QuadDriveRcDebugSnapshot {
  int rawThrottle;
  int filteredThrottle;
  int normalizedThrottle;
  bool rcFresh;
  uint32_t snapshotAgeMs;
  bool rcManualBrakeActive;
  bool rcSpeedPidEligible;
  bool rcSourceLatched;
  bool rcNeutralOffsetCalEnabled;
  bool rcNeutralOffsetCalAllowed;
  float rcTargetRawMps;
  float rcTargetShapedMps;
};

enum class QuadDriveControlSource : uint8_t {
  kNone = 0,
  kPi = 1,
  kRc = 2,
  kTelnet = 3,
};

struct QuadDriveRuntimeSnapshot {
  bool valid;
  QuadDriveControlSource source;
  bool piFresh;
  bool piEstopActive;
  bool speedPidFailsafe;
  bool speedPidOverspeed;
  uint8_t appliedBrakePercent;
};

void taskQuadDriveControl(void* parameter);
bool quadDriveSetLogEnabled(bool enabled);
bool quadDriveGetLogEnabled(bool& enabledOut);
bool quadDriveSetPidTraceEnabled(bool enabled, TickType_t periodTicks);
bool quadDriveGetPidTraceConfig(bool& enabledOut, TickType_t& periodTicksOut);
bool quadDriveSetSpeedTargetOverride(bool enabled, float targetMps);
bool quadDriveGetSpeedTargetOverride(bool& enabledOut, float& targetMpsOut);
bool quadDriveGetRcDebugSnapshot(QuadDriveRcDebugSnapshot& out);
bool quadDriveGetRuntimeSnapshot(QuadDriveRuntimeSnapshot& out);
bool quadDriveSetRcNeutralCalEnabled(bool enabled);
bool quadDriveGetRcNeutralCalEnabled(bool& enabledOut);

#endif  // QUAD_FUNCTIONS_H
