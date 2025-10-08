#ifndef QUAD_LOGIC_H
#define QUAD_LOGIC_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>

struct RcInputSnapshot {
  int throttle;   // -100..100
  int steering;   // -100..100
  int aux1;       // -100..100
  int aux2;       // -100..100
  TickType_t timestamp;
  bool valid;
};

enum class GearCommand {
  None = 0,
  Up,
  Down
};

struct QuadLogicConfig {
  int throttlePwmPin;
  uint8_t throttleLedcChannel;
  double throttlePwmFrequency;
  uint8_t throttlePwmResolutionBits;
  int throttleDeadzone;           // +/- range treated as neutro (0..100)
  uint8_t throttleMinPercent;     // Duty (%) applied once se supera el deadzone
  uint8_t throttleMaxPercent;     // Duty (%) maximo permitido
  TickType_t taskPeriod;
  TickType_t rcTimeout;
  bool log;
  HardwareSerial* gearboxSerial;
  uint32_t gearboxBaud;
  int gearboxRxPin;
  int gearboxTxPin;
  const char* reverseEnableCommand;
  const char* reverseDisableCommand;
  QueueHandle_t rcQueue;
};

QueueHandle_t quadLogicCreateRcQueue();
bool quadLogicQueueRcInput(const RcInputSnapshot& snapshot);
bool initQuadLogic(const QuadLogicConfig& config);
void taskQuadLogic(void* parameter);

#endif  // QUAD_LOGIC_H
