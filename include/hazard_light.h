#ifndef HAZARD_LIGHT_H
#define HAZARD_LIGHT_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>

enum class HazardLightSource : uint8_t {
  kFailsafe = 0,
  kUart = 1,
  kTelnet = 2,
};

struct HazardLightConfig {
  uint8_t relayPin;
  bool activeLow;
  TickType_t period;
  TickType_t piFreshTimeout;
  bool logTransitions;
};

struct HazardLightStatus {
  bool initialized;
  bool effectiveOn;
  bool uartRequestedOn;
  bool uartFresh;
  bool overrideEnabled;
  bool overrideOn;
  TickType_t lastUartFrameTick;
  uint8_t relayPin;
  bool activeLow;
  HazardLightSource source;
};

bool hazardLightInit(const HazardLightConfig& config);
bool hazardLightSetOverride(bool enabled, bool on);
bool hazardLightGetStatus(HazardLightStatus& out);
void taskHazardLightControl(void* parameter);

#endif  // HAZARD_LIGHT_H
