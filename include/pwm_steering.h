#pragma once

#include <Arduino.h>

namespace pwm_steering {

struct SteeringState {
  float angleDegrees;
  float dutyCycle;
  uint32_t periodMicros;
  uint32_t highMicros;
  uint32_t updatedMicros;
  bool signalValid;
};

void init(uint8_t gpioPin = 15);
void task(void* parameter);

bool getState(SteeringState& out);
float angleDegrees();
bool signalValid();
float dutyCycle();
uint32_t lastUpdateMicros();

}  // namespace pwm_steering
