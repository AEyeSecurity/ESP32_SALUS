#ifndef FS_IA6_H
#define FS_IA6_H

#include <Arduino.h>

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
  TickType_t staleThreshold;
  TickType_t rmtReceiveTimeout;
};

struct RcSharedState {
  int ch0;
  int ch2;
  int throttle;
  int steering;
  TickType_t lastUpdateTick;
  bool valid;
};

struct RcRawDebugSnapshot {
  bool rmtInitialized;
  bool ppmPolarityLocked;
  uint8_t ppmMarkerLevel;
  uint8_t trustedFrameCount;
  uint32_t rawBursts;
  uint32_t decodedFrames;
  uint32_t decodeFailures;
  uint32_t rmtRestarts;
  uint32_t rmtRestartErrors;
  int lastRmtRestartErr;
  uint32_t gpioEdges;
  uint32_t gpioPpmFrames;
  uint32_t gpioPpmErrors;
  uint32_t lastGpioEdgeUs;
  uint8_t gpioLevel;
  TickType_t lastGpioFrameTick;
  TickType_t lastRawTick;
  TickType_t lastDecodedTick;
  TickType_t lastRmtRestartTick;
  size_t lastRawItemCount;
  uint8_t lastChannelCount;
  uint8_t sampleItemCount;
  uint16_t sampleDuration0[4];
  uint16_t sampleDuration1[4];
  uint8_t sampleLevel0[4];
  uint8_t sampleLevel1[4];
};

constexpr uint8_t kRcPpmPin = 16;

void taskRcSampler(void* parameter);
void taskRcMonitor(void* parameter);
bool rcGetStateCopy(RcSharedState& out);
bool rcGetRawDebugSnapshot(RcRawDebugSnapshot& out);
bool rcRegisterConsumer(TaskHandle_t handle);
void rcUnregisterConsumer(TaskHandle_t handle);

#endif  // FS_IA6_H
