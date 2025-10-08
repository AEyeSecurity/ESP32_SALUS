#include "quad_logic.h"

#include <algorithm>

#include "h_bridge.h"
#include "ota_telnet.h"

namespace {

QuadLogicConfig g_config{};
QueueHandle_t g_rcQueue = nullptr;
bool g_initialized = false;
bool g_serialReady = false;
bool g_bridgeEnabled = false;

RcInputSnapshot g_lastSnapshot{};
TickType_t g_lastSnapshotTick = 0;

int clampPercent(int value, int minValue, int maxValue) {
  return std::min(std::max(value, minValue), maxValue);
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

void applyThrottle(int throttlePercent) {
  // Only positive throttle drives the PWM; negatives are treated as zero for now.
  throttlePercent = clampPercent(throttlePercent, 0, 100);
  const uint32_t duty = dutyFromPercent(g_config.throttlePwmResolutionBits, throttlePercent);
  ledcWrite(g_config.throttleLedcChannel, duty);
}

bool sendGearCommand(GearCommand cmd, bool log) {
  if (cmd == GearCommand::None || g_config.gearboxSerial == nullptr) {
    return false;
  }
  if (!g_serialReady) {
    return false;
  }
  const char* payload = nullptr;
  switch (cmd) {
    case GearCommand::Up:
      payload = ">GEAR:UP\n";
      break;
    case GearCommand::Down:
      payload = ">GEAR:DOWN\n";
      break;
    default:
      payload = nullptr;
      break;
  }
  if (payload == nullptr) {
    return false;
  }
  g_config.gearboxSerial->print(payload);
  if (log) {
    broadcastIf(true, String("[QUAD] Gear command sent: ") + payload);
  }
  return true;
}

void handleGear(const RcInputSnapshot& snapshot, bool log) {
  static bool aux1Latched = false;
  static bool aux2Latched = false;

  const bool aux1Active = snapshot.aux1 > 50;
  const bool aux2Active = snapshot.aux2 > 50;

  GearCommand cmd = GearCommand::None;
  if (aux1Active && !aux1Latched) {
    cmd = GearCommand::Up;
  } else if (aux2Active && !aux2Latched) {
    cmd = GearCommand::Down;
  }

  if (cmd != GearCommand::None) {
    sendGearCommand(cmd, log);
  }

  aux1Latched = aux1Active;
  aux2Latched = aux2Active;
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
  int lastAppliedThrottle = -1;
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
      if (lastRcFresh) {
        if (cfg->log) {
          broadcastIf(true, "[QUAD] RC timeout -> entering failsafe");
        }
      }
      lastRcFresh = false;
      if (lastAppliedThrottle != 0) {
        applyThrottle(0);
        lastAppliedThrottle = 0;
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

    const int throttleCommand = clampPercent(g_lastSnapshot.throttle, -100, 100);
    const int steeringCommand = clampPercent(g_lastSnapshot.steering, -100, 100);

    const int positiveThrottle = (throttleCommand > 0) ? throttleCommand : 0;
    if (positiveThrottle != lastAppliedThrottle) {
      applyThrottle(positiveThrottle);
      lastAppliedThrottle = positiveThrottle;
    }

    if (steeringCommand != lastAppliedSteering) {
      applySteering(steeringCommand);
      lastAppliedSteering = steeringCommand;
    }

    handleGear(g_lastSnapshot, cfg->log);

    vTaskDelayUntil(&lastWake, period);
  }
}
