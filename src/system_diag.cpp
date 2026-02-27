#include "system_diag.h"

#include <string.h>

namespace {

struct SystemDiagSlot {
  TaskHandle_t handle;
  SystemDiagTaskSample sample;
};

constexpr TickType_t kDefaultJitterStreamPeriod = pdMS_TO_TICKS(1000);

SystemDiagSlot g_slots[kSystemDiagTaskCount];
portMUX_TYPE g_diagMux = portMUX_INITIALIZER_UNLOCKED;

bool g_jitterStreamEnabled = false;
TickType_t g_jitterStreamPeriod = kDefaultJitterStreamPeriod;
TickType_t g_lastJitterEmitTick = 0;

bool taskIdToIndex(SystemDiagTaskId id, size_t& indexOut) {
  const size_t index = static_cast<size_t>(id);
  if (index >= kSystemDiagTaskCount) {
    return false;
  }
  indexOut = index;
  return true;
}

void resetSample(SystemDiagTaskSample& sample) {
  sample = {};
  sample.core = -1;
}

void refreshTaskStateLocked(SystemDiagSlot& slot) {
  if (!slot.sample.registered || slot.handle == nullptr) {
    return;
  }
  slot.sample.stackHighWaterWords = uxTaskGetStackHighWaterMark(slot.handle);
  slot.sample.priority = uxTaskPriorityGet(slot.handle);
  const BaseType_t affinity = xTaskGetAffinity(slot.handle);
  slot.sample.core = (affinity == tskNO_AFFINITY) ? -1 : affinity;
}

uint32_t absDiffU32(uint32_t a, uint32_t b) {
  return (a >= b) ? (a - b) : (b - a);
}

}  // namespace

void systemDiagInit() {
  portENTER_CRITICAL(&g_diagMux);
  for (size_t i = 0; i < kSystemDiagTaskCount; ++i) {
    g_slots[i].handle = nullptr;
    resetSample(g_slots[i].sample);
  }
  g_jitterStreamEnabled = false;
  g_jitterStreamPeriod = kDefaultJitterStreamPeriod;
  g_lastJitterEmitTick = 0;
  portEXIT_CRITICAL(&g_diagMux);
}

bool systemDiagRegisterTask(SystemDiagTaskId id, const char* name, TaskHandle_t handle) {
  if (name == nullptr || handle == nullptr) {
    return false;
  }
  size_t index = 0;
  if (!taskIdToIndex(id, index)) {
    return false;
  }

  portENTER_CRITICAL(&g_diagMux);
  SystemDiagSlot& slot = g_slots[index];
  slot.handle = handle;
  resetSample(slot.sample);
  slot.sample.registered = true;
  strncpy(slot.sample.name, name, sizeof(slot.sample.name) - 1);
  slot.sample.name[sizeof(slot.sample.name) - 1] = '\0';
  refreshTaskStateLocked(slot);
  portEXIT_CRITICAL(&g_diagMux);
  return true;
}

void systemDiagReportLoop(SystemDiagTaskId id,
                          uint32_t loopUs,
                          uint32_t expectedPeriodUs,
                          bool overrun,
                          bool notifyTimeout) {
  size_t index = 0;
  if (!taskIdToIndex(id, index)) {
    return;
  }

  portENTER_CRITICAL(&g_diagMux);
  SystemDiagSlot& slot = g_slots[index];
  if (!slot.sample.registered) {
    portEXIT_CRITICAL(&g_diagMux);
    return;
  }

  slot.sample.lastLoopUs = loopUs;
  if (loopUs > slot.sample.maxLoopUs) {
    slot.sample.maxLoopUs = loopUs;
  }

  uint32_t jitterUs = 0;
  if (expectedPeriodUs > 0U) {
    jitterUs = absDiffU32(loopUs, expectedPeriodUs);
  }
  slot.sample.jitterUs = jitterUs;
  if (jitterUs > slot.sample.maxJitterUs) {
    slot.sample.maxJitterUs = jitterUs;
  }

  if (overrun) {
    slot.sample.overrunCount++;
  }
  if (notifyTimeout) {
    slot.sample.notifyTimeoutCount++;
  }

  refreshTaskStateLocked(slot);
  portEXIT_CRITICAL(&g_diagMux);
}

bool systemDiagGetSnapshot(SystemDiagSnapshot& out) {
  out = {};
  out.timestampMs = millis();
  out.cpuStatsEnabled = false;

  portENTER_CRITICAL(&g_diagMux);
  uint32_t worstJitter = 0;
  uint32_t taskCount = 0;
  for (size_t i = 0; i < kSystemDiagTaskCount; ++i) {
    out.tasks[i] = g_slots[i].sample;
    if (out.tasks[i].registered) {
      taskCount++;
      if (out.tasks[i].maxJitterUs > worstJitter) {
        worstJitter = out.tasks[i].maxJitterUs;
      }
    }
  }
  out.taskCount = taskCount;
  out.worstJitterUs = worstJitter;
  portEXIT_CRITICAL(&g_diagMux);
  return true;
}

bool systemDiagResetStats(bool keepRegistration) {
  portENTER_CRITICAL(&g_diagMux);
  for (size_t i = 0; i < kSystemDiagTaskCount; ++i) {
    SystemDiagSlot& slot = g_slots[i];
    if (!keepRegistration) {
      slot.handle = nullptr;
      resetSample(slot.sample);
      continue;
    }

    if (!slot.sample.registered) {
      continue;
    }

    slot.sample.lastLoopUs = 0;
    slot.sample.maxLoopUs = 0;
    slot.sample.jitterUs = 0;
    slot.sample.maxJitterUs = 0;
    slot.sample.overrunCount = 0;
    slot.sample.notifyTimeoutCount = 0;
    refreshTaskStateLocked(slot);
  }
  g_lastJitterEmitTick = 0;
  portEXIT_CRITICAL(&g_diagMux);
  return true;
}

bool systemDiagSetJitterStream(bool enabled, TickType_t periodTicks) {
  if (enabled && periodTicks == 0) {
    return false;
  }
  portENTER_CRITICAL(&g_diagMux);
  g_jitterStreamEnabled = enabled;
  if (enabled) {
    g_jitterStreamPeriod = periodTicks;
    g_lastJitterEmitTick = 0;
  }
  portEXIT_CRITICAL(&g_diagMux);
  return true;
}

void systemDiagGetJitterStreamConfig(bool& enabledOut, TickType_t& periodTicksOut) {
  portENTER_CRITICAL(&g_diagMux);
  enabledOut = g_jitterStreamEnabled;
  periodTicksOut = g_jitterStreamPeriod;
  portEXIT_CRITICAL(&g_diagMux);
}

bool systemDiagShouldEmitJitter(TickType_t nowTick) {
  bool shouldEmit = false;
  portENTER_CRITICAL(&g_diagMux);
  if (g_jitterStreamEnabled && g_jitterStreamPeriod > 0) {
    if (g_lastJitterEmitTick == 0 || (nowTick - g_lastJitterEmitTick) >= g_jitterStreamPeriod) {
      g_lastJitterEmitTick = nowTick;
      shouldEmit = true;
    }
  }
  portEXIT_CRITICAL(&g_diagMux);
  return shouldEmit;
}
