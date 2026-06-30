#include "hazard_light.h"

#include "ota_telnet.h"
#include "pi_comms.h"
#include "system_diag.h"

namespace {

HazardLightConfig g_config{};
bool g_initialized = false;
bool g_effectiveOn = false;
bool g_uartRequestedOn = false;
bool g_uartFresh = false;
bool g_overrideEnabled = false;
bool g_overrideOn = false;
TickType_t g_lastUartFrameTick = 0;
HazardLightSource g_source = HazardLightSource::kFailsafe;
portMUX_TYPE g_hazardMux = portMUX_INITIALIZER_UNLOCKED;

const char* sourceText(HazardLightSource source) {
  switch (source) {
    case HazardLightSource::kUart:
      return "UART";
    case HazardLightSource::kTelnet:
      return "TELNET";
    case HazardLightSource::kFailsafe:
    default:
      return "FAILSAFE";
  }
}

void writeRelayOutput(bool on) {
  if (!g_initialized) {
    return;
  }

  const uint8_t level = (g_config.activeLow ? (on ? LOW : HIGH) : (on ? HIGH : LOW));
  digitalWrite(g_config.relayPin, level);
}

void logTransitionIfNeeded(bool previousOn, HazardLightSource previousSource, bool newOn, HazardLightSource newSource) {
  if (!g_config.logTransitions) {
    return;
  }
  if (previousOn == newOn && previousSource == newSource) {
    return;
  }

  String msg;
  msg.reserve(96);
  msg += "[HAZARD] ";
  msg += newOn ? "ON" : "OFF";
  msg += " src=";
  msg += sourceText(newSource);
  msg += " prev=";
  msg += sourceText(previousSource);
  broadcastIf(true, msg);
}

void resolveAndApplyLocked(bool& effectiveOnOut, HazardLightSource& sourceOut) {
  if (g_overrideEnabled) {
    effectiveOnOut = g_overrideOn;
    sourceOut = HazardLightSource::kTelnet;
    return;
  }
  if (g_uartFresh) {
    effectiveOnOut = g_uartRequestedOn;
    sourceOut = HazardLightSource::kUart;
    return;
  }
  effectiveOnOut = false;
  sourceOut = HazardLightSource::kFailsafe;
}

void applyResolvedState() {
  bool previousOn = false;
  HazardLightSource previousSource = HazardLightSource::kFailsafe;
  bool newOn = false;
  HazardLightSource newSource = HazardLightSource::kFailsafe;

  portENTER_CRITICAL(&g_hazardMux);
  previousOn = g_effectiveOn;
  previousSource = g_source;
  resolveAndApplyLocked(newOn, newSource);
  g_effectiveOn = newOn;
  g_source = newSource;
  portEXIT_CRITICAL(&g_hazardMux);

  writeRelayOutput(newOn);
  logTransitionIfNeeded(previousOn, previousSource, newOn, newSource);
}

void updateFromPiSnapshot(const PiCommsRxSnapshot& snapshot, bool piFresh) {
  portENTER_CRITICAL(&g_hazardMux);
  g_uartRequestedOn = piFresh && snapshot.hazardLightRequested;
  g_uartFresh = piFresh;
  g_lastUartFrameTick = snapshot.hasFrame ? snapshot.lastFrameTick : 0;
  portEXIT_CRITICAL(&g_hazardMux);
  applyResolvedState();
}

}  // namespace

bool hazardLightInit(const HazardLightConfig& config) {
  if (g_initialized) {
    return true;
  }

  g_config = config;
  pinMode(g_config.relayPin, OUTPUT);

  portENTER_CRITICAL(&g_hazardMux);
  g_effectiveOn = false;
  g_uartRequestedOn = false;
  g_uartFresh = false;
  g_overrideEnabled = false;
  g_overrideOn = false;
  g_lastUartFrameTick = 0;
  g_source = HazardLightSource::kFailsafe;
  g_initialized = true;
  portEXIT_CRITICAL(&g_hazardMux);

  writeRelayOutput(false);

  return true;
}

bool hazardLightSetOverride(bool enabled, bool on) {
  if (!g_initialized) {
    return false;
  }

  portENTER_CRITICAL(&g_hazardMux);
  g_overrideEnabled = enabled;
  g_overrideOn = on;
  portEXIT_CRITICAL(&g_hazardMux);
  applyResolvedState();
  return true;
}

bool hazardLightGetStatus(HazardLightStatus& out) {
  portENTER_CRITICAL(&g_hazardMux);
  out.initialized = g_initialized;
  out.effectiveOn = g_effectiveOn;
  out.uartRequestedOn = g_uartRequestedOn;
  out.uartFresh = g_uartFresh;
  out.overrideEnabled = g_overrideEnabled;
  out.overrideOn = g_overrideOn;
  out.lastUartFrameTick = g_lastUartFrameTick;
  out.relayPin = g_config.relayPin;
  out.activeLow = g_config.activeLow;
  out.source = g_source;
  portEXIT_CRITICAL(&g_hazardMux);
  return out.initialized;
}

void taskHazardLightControl(void* parameter) {
  HazardLightConfig* cfg = static_cast<HazardLightConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[HAZARD] Configuracion invalida, abortando tarea");
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  TickType_t lastWakeTick = xTaskGetTickCount();
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);

  for (;;) {
    const int64_t loopStartUs = esp_timer_get_time();
    const TickType_t nowTick = xTaskGetTickCount();

    PiCommsRxSnapshot snapshot{};
    const bool driverReady = piCommsGetRxSnapshot(snapshot);
    TickType_t ageTicks = 0;
    bool ageValid = false;
    if (snapshot.lastFrameTick != 0) {
      ageTicks = nowTick - snapshot.lastFrameTick;
      ageValid = true;
    }
    const bool piFresh =
        driverReady && snapshot.hasFrame && ageValid && ageTicks <= cfg->piFreshTimeout;

    updateFromPiSnapshot(snapshot, piFresh);

    vTaskDelayUntil(&lastWakeTick, period);

    const int64_t loopEndUs = esp_timer_get_time();
    uint32_t loopUs = 0;
    if (loopEndUs > loopStartUs) {
      loopUs = static_cast<uint32_t>(loopEndUs - loopStartUs);
    }
    const bool overrun = loopUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kHazardLight, loopUs, expectedPeriodUs, overrun, false);
  }
}
