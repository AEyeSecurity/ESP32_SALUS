#ifndef PI_COMMS_H
#define PI_COMMS_H

#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct PiCommsConfig {
  uart_port_t uartNum;
  int txPin;
  int rxPin;
  int baudRate;
  size_t rxBufferSize;
  size_t txBufferSize;
  TickType_t rxTaskPeriod;
  TickType_t txTaskPeriod;
  TickType_t rxReadTimeout;
  bool logRx;
  bool logTx;
};

struct PiCommsRxSnapshot {
  bool driverReady;
  bool hasFrame;
  TickType_t lastFrameTick;
  uint8_t verFlags;
  int8_t steer;
  uint16_t speedCmdCentiMps;
  uint8_t brake;
  bool driveEnabled;
  bool estop;
  uint32_t framesOk;
  uint32_t framesCrcError;
  uint32_t framesMalformed;
  uint32_t framesVersionError;
};

bool piCommsInit(const PiCommsConfig& config);
bool piCommsGetRxSnapshot(PiCommsRxSnapshot& snapshot);
void piCommsResetStats();

void taskPiCommsRx(void* parameter);
void taskPiCommsTx(void* parameter);

#endif
