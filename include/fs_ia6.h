#ifndef ARDUINO_H_INCLUDE
#define ARDUINO_H_INCLUDE

#include <Arduino.h>

#endif  // ARDUINO_H_INCLUDE

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Include iBusBM Library
#include <IBusBM.h>

// Set all pins as inputs for COMM
void initFS_IA6(uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4, uint8_t ch5, uint8_t ch6);

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue);

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue);

// IBUS Function

// Attach iBus object to serial port
void initIBus(IBusBM& ibus, HardwareSerial& serial);

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(IBusBM& ibus, byte channelInput, int minLimit, int maxLimit, int defaultValue);

struct FsIa6TaskConfig {
  bool log;
  TickType_t period;
};

struct FsIa6SamplerConfig {
  bool log;
  TickType_t period;
};

struct RcSharedState {
  int ch0;
  int ch2;
  int throttle;
  int steering;
  TickType_t lastUpdateTick;
  bool valid;
};

constexpr uint8_t kRcSteeringPin = 16;
constexpr uint8_t kRcThrottlePin = 4;

void taskRcSampler(void* parameter);
void taskRcMonitor(void* parameter);
bool rcGetStateCopy(RcSharedState& out);
