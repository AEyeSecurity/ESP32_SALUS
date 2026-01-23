#ifndef STEERING_CALIBRATION_H
#define STEERING_CALIBRATION_H

#include <Arduino.h>

struct SteeringCalibrationData {
  bool initialized = false;
  bool hasCalibration = false;
  float leftLimitDeg = 0.0f;
  float rightLimitDeg = 0.0f;
  float rawCenterDeg = 0.0f;
  float adjustedCenterDeg = 0.0f;
  float userOffsetDeg = 0.0f;
  float maxOffsetLeftDeg = -0.0f;
  float maxOffsetRightDeg = 0.0f;
  uint32_t lastCalibrationMs = 0;
};

void steeringCalibrationInit(float defaultCenterDeg, float defaultSpanDeg);
SteeringCalibrationData steeringCalibrationSnapshot();

void steeringCalibrationRequest();
bool steeringCalibrationConsumeRequest();

void steeringCalibrationSetOffset(float offsetDeg);

void steeringCalibrationApply(float leftDeg, float rightDeg);
void steeringCalibrationReset(bool clearStorage);

#endif  // STEERING_CALIBRATION_H
