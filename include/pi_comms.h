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
  TickType_t batteryTxPeriod;
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
  int16_t speedCmdSignedCentiMps;
  bool speedReverseRequest;
  bool hazardLightRequested;
  uint8_t brake;
  bool driveEnabled;
  bool estop;
  uint32_t framesOk;
  uint32_t framesCrcError;
  uint32_t framesMalformed;
  uint32_t framesVersionError;
};

struct PiCommsBatteryTxSnapshot {
  bool driverReady;
  bool hasFrame;
  TickType_t lastFrameTick;
  uint8_t flags;
  uint16_t batteryCentiVolts;
  uint16_t adcPinMv;
  uint8_t sampleAgeDs;
  uint32_t framesSent;
};

bool piCommsInit(const PiCommsConfig& config);
bool piCommsGetRxSnapshot(PiCommsRxSnapshot& snapshot);
bool piCommsGetBatteryTxSnapshot(PiCommsBatteryTxSnapshot& snapshot);
void piCommsResetStats();

// Diagnostic-only sideband. It never changes the binary UART protocol or drive control.
void piCommsSetHallTelemetryTraceEnabled(bool enabled);
bool piCommsGetHallTelemetryTraceEnabled();

// Binary RAM capture for post-run Hall diagnostics. This is sideband-only: the
// trigger preserves evidence but never rejects or changes speed telemetry.
struct PiHallTelemetryCaptureRecord {
  uint32_t sequence;
  uint32_t txTimestampUs;
  uint16_t speedCentiMps;
  uint32_t transitionPeriodUs;
  uint32_t lastTransitionUs;
  uint32_t eventAgeUs;
  uint32_t transitionsOk;
  uint32_t transitionsInvalidState;
  uint32_t transitionsInvalidJump;
  uint32_t isrCount;
  uint8_t hallMask;
  uint8_t flags;
};

struct PiHallTelemetryCaptureStatus {
  bool armed;
  bool triggered;
  bool frozen;
  uint16_t count;
  uint16_t capacity;
  uint16_t postTriggerRemaining;
  uint32_t triggerSequence;
  uint16_t triggerSpeedCentiMps;
};

void piCommsArmHallTelemetryCapture();
void piCommsClearHallTelemetryCapture();
bool piCommsGetHallTelemetryCaptureStatus(PiHallTelemetryCaptureStatus& status);
bool piCommsReadHallTelemetryCaptureRecord(size_t oldestIndex,
                                           PiHallTelemetryCaptureRecord& record);

void taskPiCommsRx(void* parameter);
void taskPiCommsTx(void* parameter);

#endif
