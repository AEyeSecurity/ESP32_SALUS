#ifndef SYSTEM_DIAG_H
#define SYSTEM_DIAG_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

enum class SystemDiagTaskId : uint8_t {
  kOtaTelnet = 0,
  kRcSampler,
  kPid,
  kDrive,
  kPiUartRx,
  kPiUartTx,
  kCount,
};

constexpr size_t kSystemDiagTaskCount = static_cast<size_t>(SystemDiagTaskId::kCount);

struct SystemDiagTaskSample {
  bool registered;
  char name[16];
  BaseType_t core;
  UBaseType_t priority;
  uint32_t stackHighWaterWords;
  uint32_t lastLoopUs;
  uint32_t maxLoopUs;
  uint32_t jitterUs;
  uint32_t maxJitterUs;
  uint32_t overrunCount;
  uint32_t notifyTimeoutCount;
};

struct SystemDiagSnapshot {
  uint32_t timestampMs;
  bool cpuStatsEnabled;
  uint32_t worstJitterUs;
  uint32_t taskCount;
  SystemDiagTaskSample tasks[kSystemDiagTaskCount];
};

void systemDiagInit();
bool systemDiagRegisterTask(SystemDiagTaskId id, const char* name, TaskHandle_t handle);
void systemDiagReportLoop(SystemDiagTaskId id,
                          uint32_t loopUs,
                          uint32_t expectedPeriodUs,
                          bool overrun,
                          bool notifyTimeout);
bool systemDiagGetSnapshot(SystemDiagSnapshot& out);
bool systemDiagResetStats(bool keepRegistration = true);

bool systemDiagSetJitterStream(bool enabled, TickType_t periodTicks);
void systemDiagGetJitterStreamConfig(bool& enabledOut, TickType_t& periodTicksOut);
bool systemDiagShouldEmitJitter(TickType_t nowTick);

#endif  // SYSTEM_DIAG_H
