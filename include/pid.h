#pragma once

#include <stdint.h>

namespace steering_pid {

struct Config {
  int rcInputMin = -100;
  int rcInputMax = 100;
  float rcDeadband = 5.0f;
  float angleDeadbandDeg = 0.5f;
  float steeringCenterAbsDeg = 155.0f;
  float steeringMaxAngleDeg = 30.0f;
  float kp = 1.0f;
  float ki = 0.0f;
  float kd = 0.12f;
  float integralLimit = 200.0f;
  uint8_t minDutyPercent = 18;
  uint8_t maxDutyPercent = 85;
  float outputDeadbandPercent = 5.0f;
  bool enableSteeringActuator = true;
};

extern Config config;

void init();
void setRcValue(int value);
int rcValue();
float rcToSetpointDegrees(int rawValue);
float absoluteToRelative(float absoluteDegrees);
float update(float setpointDeg, float measuredDeg, float dtSeconds);
void reset();
void applyControl(float controlPercent);
void stopActuator();

}  // namespace steering_pid

