#ifndef HALL_SPEED_H
#define HALL_SPEED_H

#include <Arduino.h>

struct HallSpeedConfig {
  uint8_t pinA;
  uint8_t pinB;
  uint8_t pinC;
  bool activeLow;
  uint8_t motorPoles;
  float gearReduction;
  float wheelDiameterM;
  uint32_t rpmTimeoutUs;
};

struct HallSpeedSnapshot {
  bool driverReady;
  uint8_t hallMask;
  bool hasTransition;
  uint32_t transitionPeriodUs;
  uint32_t lastTransitionUs;
  uint32_t transitionAgeUs;
  float motorRpm;
  float speedKmh;
  float speedMps;
  uint32_t transitionsOk;
  uint32_t transitionsInvalidState;
  uint32_t transitionsInvalidJump;
  uint32_t isrCount;
};

bool hallSpeedInit(const HallSpeedConfig& config);
bool hallSpeedGetSnapshot(HallSpeedSnapshot& snapshot);
void hallSpeedGetConfig(HallSpeedConfig& config);
void hallSpeedResetStats();

#endif  // HALL_SPEED_H
