#ifndef QUAD_FUNCTIONS_H
#define QUAD_FUNCTIONS_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

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
  int releaseAngleDeg;
  int brakeAngleDeg;
  int activationThreshold;  // negative threshold (e.g. -15)
};

void initQuadBrake(const QuadBrakeConfig& config);
void quadBrakeUpdate(int rcValue);
void quadBrakeRelease();

struct QuadThrottleTaskConfig {
  QuadThrottleConfig throttle;
  uint8_t rcInputPin;
  bool autoInitHardware;
  TickType_t period;
  bool log;
};

struct QuadBrakeTaskConfig {
  QuadBrakeConfig brake;
  uint8_t rcInputPin;
  bool autoInitHardware;
  TickType_t period;
  bool log;
};

void taskQuadThrottleControl(void* parameter);
void taskQuadBrakeControl(void* parameter);

#endif  // QUAD_FUNCTIONS_H
