#include "quad_logic.h"

#include <algorithm>
#include <cstdint>
#include <cstdio>

#include "h_bridge.h"
#include "ota_telnet.h"

namespace {

QuadLogicConfig g_config{};
QueueHandle_t g_rcQueue = nullptr;
bool g_initialized = false;
bool g_serialReady = false;
bool g_bridgeEnabled = false;
bool g_reverseActive = false;
uint8_t g_currentGear = 1;
bool g_shiftAuxActive = false;
bool g_shiftHoldSatisfied = false;
TickType_t g_shiftAuxSince = 0;

RcInputSnapshot g_lastSnapshot{};
TickType_t g_lastSnapshotTick = 0;

int clampPercent(int value, int minValue, int maxValue) {
  return std::min(std::max(value, minValue), maxValue);
}

int mapRange(int value, int inMin, int inMax, int outMin, int outMax) {
  if (inMax == inMin) {
    return outMin;
  }
  return outMin + (static_cast<int64_t>(value - inMin) * (outMax - outMin)) / (inMax - inMin);
}

uint32_t dutyFromPercent(uint8_t resolutionBits, int percent) {
  percent = clampPercent(percent, 0, 100);
  const uint32_t maxDuty = (1u << resolutionBits) - 1u;
  return (percent >= 100) ? maxDuty : static_cast<uint32_t>((percent * maxDuty) / 100);
}

void ensureBridgeEnabled() {
  if (!g_bridgeEnabled) {
    enable_bridge_h();
    g_bridgeEnabled = true;
  }
}

void ensureBridgeDisabled() {
  if (g_bridgeEnabled) {
    disable_bridge_h();
    g_bridgeEnabled = false;
  }
}

void applySteering(int steeringPercent) {
  steeringPercent = clampPercent(steeringPercent, -100, 100);
  if (steeringPercent > 0) {
    ensureBridgeEnabled();
    bridge_turn_right(static_cast<uint8_t>(steeringPercent));
  } else if (steeringPercent < 0) {
    ensureBridgeEnabled();
    bridge_turn_left(static_cast<uint8_t>(-steeringPercent));
  } else {
    bridge_stop();
    ensureBridgeDisabled();
  }
}

void applyThrottleDutyPercent(int dutyPercent) {
  dutyPercent = clampPercent(dutyPercent, 0, 100);
  const uint32_t duty = dutyFromPercent(g_config.throttlePwmResolutionBits, dutyPercent);
  ledcWrite(g_config.throttleLedcChannel, duty);
}

bool sendReverseCommand(bool reverseEnabled, bool log) {
  if (g_config.gearboxSerial == nullptr || !g_serialReady) {
    return false;
  }
  const char* payload = reverseEnabled ? g_config.reverseEnableCommand : g_config.reverseDisableCommand;
  if (payload == nullptr) {
    return false;
  }
  g_config.gearboxSerial->print(payload);
  if (log) {
    broadcastIf(true, String("[QUAD] Reverse command sent: ") + payload);
  }
  return true;
}

void ensureReverseState(bool reverseEnabled, bool log) {
  if (reverseEnabled == g_reverseActive) {
    return;
  }
  if (sendReverseCommand(reverseEnabled, log)) {
    g_reverseActive = reverseEnabled;
  }
}

bool sendGearShift(uint8_t newGear, bool log) {
  if (g_config.gearboxSerial == nullptr || !g_serialReady) {
    return false;
  }
  char buffer[24];
  const int written = snprintf(buffer, sizeof(buffer), ">GEAR:SHIFT:%u\n", static_cast<unsigned>(newGear));
  if (written <= 0) {
    return false;
  }
  g_config.gearboxSerial->print(buffer);
  if (log) {
    broadcastIf(true, String("[QUAD] Gear shift -> marcha ") + String(newGear));
  }
  return true;
}

void handleGear(const RcInputSnapshot& snapshot, bool log) {
  const TickType_t now = snapshot.timestamp;
  const bool auxAbove = snapshot.aux1 >= g_config.gearShiftThreshold;

  if (auxAbove) {
    if (!g_shiftAuxActive) {
      g_shiftAuxActive = true;
      g_shiftAuxSince = now;
      g_shiftHoldSatisfied = false;
    } else if (!g_shiftHoldSatisfied) {
      if ((now - g_shiftAuxSince) >= g_config.gearShiftHoldTicks) {
        g_shiftHoldSatisfied = true;
        if (log) {
          broadcastIf(true, "[QUAD] Cambio armado (aux1 sobre threshold)");
        }
      }
    }
    return;
  }

  if (g_shiftAuxActive) {
    if (g_shiftHoldSatisfied) {
      uint8_t nextGear = (g_currentGear >= g_config.gearMaxNumber) ? 1 : static_cast<uint8_t>(g_currentGear + 1);
      g_currentGear = nextGear;
      sendGearShift(g_currentGear, log);
    }
  }

  g_shiftAuxActive = false;
  g_shiftHoldSatisfied = false;
}

void logSnapshot(const RcInputSnapshot& snapshot) {
  String msg = "[QUAD] RC snapshot -> throttle=" + String(snapshot.throttle) +
               " steering=" + String(snapshot.steering) +
               " aux1=" + String(snapshot.aux1) +
               " aux2=" + String(snapshot.aux2);
  broadcastIf(true, msg);
}

}  // namespace

QueueHandle_t quadLogicCreateRcQueue() {
  if (g_rcQueue == nullptr) {
    g_rcQueue = xQueueCreate(1, sizeof(RcInputSnapshot));
  }
  return g_rcQueue;
}

bool quadLogicQueueRcInput(const RcInputSnapshot& snapshot) {
  if (g_rcQueue == nullptr) {
    return false;
  }
  const BaseType_t ok = xQueueOverwrite(g_rcQueue, &snapshot);
  return ok == pdPASS;
}

bool initQuadLogic(const QuadLogicConfig& config) {
  if (config.rcQueue == nullptr) {
    return false;
  }
  g_config = config;
  g_rcQueue = config.rcQueue;

  g_config.throttleDeadzone = clampPercent(g_config.throttleDeadzone, 0, 99);
  g_config.throttleMinPercent = static_cast<uint8_t>(clampPercent(g_config.throttleMinPercent, 0, 100));
  g_config.throttleMaxPercent = static_cast<uint8_t>(clampPercent(g_config.throttleMaxPercent, g_config.throttleMinPercent, 100));
  if (g_config.gearShiftThreshold < -100) {
    g_config.gearShiftThreshold = -100;
  } else if (g_config.gearShiftThreshold > 100) {
    g_config.gearShiftThreshold = 100;
  }
  if (g_config.gearShiftHoldTicks == 0) {
    g_config.gearShiftHoldTicks = pdMS_TO_TICKS(500);
  }
  if (g_config.gearMaxNumber == 0) {
    g_config.gearMaxNumber = 1;
  }
  if (g_config.gearInitialNumber == 0 || g_config.gearInitialNumber > g_config.gearMaxNumber) {
    g_config.gearInitialNumber = 1;
  }

  // Configura el canal LEDC reservado para el acelerador (documentado en README).
  ledcSetup(g_config.throttleLedcChannel, g_config.throttlePwmFrequency, g_config.throttlePwmResolutionBits);
  ledcAttachPin(g_config.throttlePwmPin, g_config.throttleLedcChannel);
  ledcWrite(g_config.throttleLedcChannel, 0);

  init_h_bridge();
  ensureBridgeDisabled();

  g_serialReady = false;
  if (g_config.gearboxSerial != nullptr) {
    if (g_config.gearboxRxPin >= 0 && g_config.gearboxTxPin >= 0) {
      g_config.gearboxSerial->begin(g_config.gearboxBaud, SERIAL_8N1, g_config.gearboxRxPin, g_config.gearboxTxPin);
    } else {
      g_config.gearboxSerial->begin(g_config.gearboxBaud);
    }
    g_serialReady = true;
  }

  g_lastSnapshot = {};
  g_lastSnapshot.valid = false;
  g_lastSnapshotTick = 0;
  g_reverseActive = false;
  g_currentGear = g_config.gearInitialNumber;
  g_shiftAuxActive = false;
  g_shiftHoldSatisfied = false;
  g_shiftAuxSince = 0;
  g_initialized = true;
  return true;
}

void taskQuadLogic(void* parameter) {
  const QuadLogicConfig* cfg = static_cast<const QuadLogicConfig*>(parameter);
  if (cfg == nullptr || !g_initialized) {
    vTaskDelete(nullptr);
    return;
  }

  TickType_t lastWake = xTaskGetTickCount();
  const TickType_t period = (cfg->taskPeriod > 0) ? cfg->taskPeriod : pdMS_TO_TICKS(20);
  const TickType_t timeout = (cfg->rcTimeout > 0) ? cfg->rcTimeout : pdMS_TO_TICKS(200);

  bool lastRcFresh = false;
  int lastAppliedThrottleDuty = -1;
  int lastAppliedSteering = -101;

  for (;;) {
    RcInputSnapshot snapshot{};
    bool received = false;
    while (g_rcQueue != nullptr && xQueueReceive(g_rcQueue, &snapshot, 0) == pdPASS) {
      g_lastSnapshot = snapshot;
      g_lastSnapshotTick = snapshot.timestamp;
      received = true;
    }

    const TickType_t now = xTaskGetTickCount();
    const bool rcFresh = g_lastSnapshot.valid && (now - g_lastSnapshotTick <= timeout);

    if (!rcFresh) {
      if (lastRcFresh && cfg->log) {
        broadcastIf(true, "[QUAD] RC timeout -> entering failsafe");
      }
      lastRcFresh = false;
      ensureReverseState(false, cfg->log);
      if (lastAppliedThrottleDuty != 0) {
        applyThrottleDutyPercent(0);
        lastAppliedThrottleDuty = 0;
      }
      if (lastAppliedSteering != 0) {
        applySteering(0);
        lastAppliedSteering = 0;
      }
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    lastRcFresh = true;

    if (cfg->log && received) {
      logSnapshot(g_lastSnapshot);
    }

    const int throttleCommandRaw = clampPercent(g_lastSnapshot.throttle, -100, 100);
    const int steeringCommand = clampPercent(g_lastSnapshot.steering, -100, 100);

    const int deadzone = g_config.throttleDeadzone;
    const int absThrottle = (throttleCommandRaw >= 0) ? throttleCommandRaw : -throttleCommandRaw;
    bool reverseRequested = false;
    int dutyPercent = 0;

    if (absThrottle > deadzone) {
      reverseRequested = (throttleCommandRaw < 0);
      const int scaled = mapRange(absThrottle, deadzone, 100, 0, 100);
      const int minPct = g_config.throttleMinPercent;
      const int maxPct = g_config.throttleMaxPercent;
      if (scaled <= 0) {
        dutyPercent = minPct;
      } else {
        dutyPercent = minPct + ((maxPct - minPct) * scaled) / 100;
      }
    } else {
      reverseRequested = false;
      dutyPercent = 0;
    }

    ensureReverseState(reverseRequested, cfg->log);

    if (dutyPercent != lastAppliedThrottleDuty) {
      applyThrottleDutyPercent(dutyPercent);
      lastAppliedThrottleDuty = dutyPercent;
    }

    if (steeringCommand != lastAppliedSteering) {
      applySteering(steeringCommand);
      lastAppliedSteering = steeringCommand;
    }

    handleGear(g_lastSnapshot, cfg->log);

    vTaskDelayUntil(&lastWake, period);
  }
}
