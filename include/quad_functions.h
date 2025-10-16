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

struct QuadThrottleTaskConfig {
  QuadThrottleConfig throttle;
  uint8_t rcInputPin;
  bool autoInitHardware;
  TickType_t period;
  bool log;
};

void taskQuadThrottleControl(void* parameter);

#endif  // QUAD_FUNCTIONS_H
