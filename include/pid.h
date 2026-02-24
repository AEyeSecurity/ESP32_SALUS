#ifndef PID_H
#define PID_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

#include <stdint.h>

#include "steering_calibration.h"

class AS5600;

class PidController {
 public:
  PidController();

  void setTunings(float kp, float ki, float kd);
  void setOutputLimits(float minOutput, float maxOutput);
  void setIntegralLimits(float minIntegral, float maxIntegral);
  void setEnabled(bool enabled);
  bool enabled() const;
  void reset();

  float update(float error, float dtSeconds);

  float kp() const { return kp_; }
  float ki() const { return ki_; }
  float kd() const { return kd_; }
  float outputMin() const { return outMin_; }
  float outputMax() const { return outMax_; }

 private:
  float kp_;
  float ki_;
  float kd_;
  float outMin_;
  float outMax_;
  float integralTerm_;
  float integralMin_;
  float integralMax_;
  float prevError_;
  bool firstRun_;
  bool enabled_;
};

float normalizeAngleDegrees(float angle);
float wrapAngleDegrees(float angle);
float computeAngleError(float setpointDeg, float measurementDeg);
float mapRcValueToAngle(int rcValue,
                        const SteeringCalibrationData& calibration,
                        float fallbackCenterDeg,
                        float fallbackSpanDeg);

struct PidTaskConfig {
  AS5600* sensor;
  uint8_t rcPin;
  float centerDeg;
  float spanDeg;
  float deadbandPercent;
  float minActivePercent;
  PidController* controller;
  TickType_t period;
  TickType_t logInterval;
  bool log;
  bool autoInitBridge;
};

struct PidRuntimeSnapshot {
  bool valid;
  bool calibrationActive;
  bool steeringFromPi;
  bool sensorValid;
  bool limitLeftActive;
  bool limitRightActive;
  int steeringCommand;
  float dtSeconds;
  float measuredDeg;
  float targetDeg;
  float errorDeg;
  float outputPercent;
};

void taskPidControl(void* parameter);

void pidRegisterController(PidController* controller);
void pidRegisterConfig(PidTaskConfig* config);
bool pidGetTunings(float& kp, float& ki, float& kd);
bool pidSetTunings(float kp, float ki, float kd);
bool pidGetOutputModifiers(float& deadbandPercent, float& minActivePercent);
bool pidSetDeadband(float deadbandPercent);
bool pidSetMinActive(float minActivePercent);
bool pidGetRuntimeSnapshot(PidRuntimeSnapshot& snapshot);

#endif  // PID_H
