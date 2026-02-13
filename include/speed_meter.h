#ifndef SPEED_METER_H
#define SPEED_METER_H

#include <Arduino.h>
#include <driver/uart.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

struct SpeedMeterConfig {
  uart_port_t uartNum;
  int rxPin;
  int baudRate;
  size_t rxBufferSize;
  size_t eventQueueSize;
  uint32_t frameGapUs;
  uint8_t minFrameLen;
  uint8_t maxFrameLen;
  bool logRx;
  bool invertRx;
};

struct SpeedMeterSnapshot {
  bool driverReady;
  bool hasFrame;
  TickType_t lastFrameTick;
  int speedKmh;
  int confidence;
  bool throttleOn;
  uint32_t framesOk;
  uint32_t framesShort;
  uint32_t framesLong;
  uint32_t frameOverflows;
  uint32_t framesUnknown;
  uint32_t uartBytesRx;
  uint32_t uartEventsData;
  uint32_t uartEventsFifoOvf;
  uint32_t uartEventsBufferFull;
  uint32_t uartEventsFrameErr;
  uint32_t uartEventsParityErr;
  uint32_t uartEventsBreak;
  uint8_t lastFrameLen;
  uint8_t lastB8;
  uint8_t lastB16;
  uint8_t lastB17;
};

bool speedMeterInit(const SpeedMeterConfig& config);
bool speedMeterGetSnapshot(SpeedMeterSnapshot& snapshot);
void speedMeterGetConfig(SpeedMeterConfig& config);
bool speedMeterSetUartConfig(int baudRate, bool invertRx);
void speedMeterResetStats();
void taskSpeedMeterRx(void* parameter);

#endif  // SPEED_METER_H
