#include "../include/fs_ia6.h"

#include <algorithm>

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_err.h>
#include <esp_timer.h>

#include "ota_telnet.h"
#include "system_diag.h"

namespace {
constexpr uint32_t kMinPulseUs = 950;
constexpr uint32_t kMaxPulseUs = 2050;
constexpr size_t kMaxRcConsumers = 6;
constexpr size_t kRcChannelCount = 4;

struct RcConsumerRegistry {
  TaskHandle_t handle = nullptr;
};

struct RcChannelRuntime {
  uint8_t pin;
  rmt_channel_t channel;
  RingbufHandle_t ring;
  uint32_t lastPulseUs;
  TickType_t lastUpdateTick;
  const char* label;

  RcChannelRuntime()
      : pin(0), channel(RMT_CHANNEL_0), ring(nullptr), lastPulseUs(1500), lastUpdateTick(0), label(nullptr) {}

  RcChannelRuntime(uint8_t pin_, rmt_channel_t channel_, const char* label_, uint32_t initialPulse = 1500)
      : pin(pin_),
        channel(channel_),
        ring(nullptr),
        lastPulseUs(initialPulse),
        lastUpdateTick(0),
        label(label_) {}
};

portMUX_TYPE g_rcStateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_consumerMux = portMUX_INITIALIZER_UNLOCKED;

RcSharedState g_rcState = {0, 0, 0, 0, 0, false};
TickType_t g_lastRcLogTick = 0;
bool g_rmtInitialized = false;

RcConsumerRegistry g_consumerRegistry[kMaxRcConsumers];

RcChannelRuntime g_rcChannels[kRcChannelCount] = {
    RcChannelRuntime{kRcAux1Pin, RMT_CHANNEL_0, "AUX1"},
    RcChannelRuntime{kRcAux2Pin, RMT_CHANNEL_1, "AUX2"},
    RcChannelRuntime{kRcThrottlePin, RMT_CHANNEL_2, "THROTTLE"},
    RcChannelRuntime{kRcSteeringPin, RMT_CHANNEL_3, "STEERING"},
};

int clampToRange(int value, int minValue, int maxValue) {
  if (minValue < maxValue) {
    return std::min(std::max(value, minValue), maxValue);
  }
  return std::max(std::min(value, minValue), maxValue);
}

int pulseWidthToRange(uint32_t pulseUs, int minLimit, int maxLimit) {
  uint32_t clamped = std::min(std::max(pulseUs, kMinPulseUs), kMaxPulseUs);
  long mapped = map(static_cast<long>(clamped), 1000, 2000, minLimit, maxLimit);
  return clampToRange(static_cast<int>(mapped), minLimit, maxLimit);
}

bool configureRmtChannel(RcChannelRuntime& runtime, bool logErrors) {
  rmt_config_t config{};
  config.rmt_mode = RMT_MODE_RX;
  config.channel = runtime.channel;
  config.clk_div = 80;  // 1 tick = 1 microsecond
  config.gpio_num = static_cast<gpio_num_t>(runtime.pin);
  config.mem_block_num = 1;
  config.rx_config.filter_en = true;
  config.rx_config.filter_ticks_thresh = 100;     // Ignore glitches <100us
  config.rx_config.idle_threshold = 12000;        // 12ms -> end of frame

  esp_err_t err = rmt_config(&config);
  if (err != ESP_OK) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error configurando canal ") + runtime.label + " err=" + err);
    }
    return false;
  }

  err = rmt_driver_install(runtime.channel, 2048, 0);
  if (err != ESP_OK) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error instalando driver canal ") + runtime.label + " err=" + err);
    }
    return false;
  }

  err = rmt_get_ringbuf_handle(runtime.channel, &runtime.ring);
  if (err != ESP_OK || runtime.ring == nullptr) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error obteniendo ringbuffer canal ") + runtime.label);
    }
    return false;
  }

  err = rmt_rx_start(runtime.channel, true);
  if (err != ESP_OK) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error iniciando recepcion canal ") + runtime.label + " err=" + err);
    }
    return false;
  }

  runtime.lastPulseUs = 1500;
  runtime.lastUpdateTick = xTaskGetTickCount();
  return true;
}

bool ensureRmtInitialized(bool logErrors) {
  if (g_rmtInitialized) {
    return true;
  }

  bool ok = true;
  for (RcChannelRuntime& runtime : g_rcChannels) {
    if (!configureRmtChannel(runtime, logErrors)) {
      ok = false;
    }
  }

  g_rmtInitialized = ok;
  return ok;
}

bool readRmtPulseUs(RcChannelRuntime& runtime, TickType_t timeoutTicks, uint32_t& outPulseUs) {
  if (runtime.ring == nullptr) {
    return false;
  }

  size_t rxSize = 0;
  rmt_item32_t* items =
      static_cast<rmt_item32_t*>(xRingbufferReceive(runtime.ring, &rxSize, timeoutTicks));
  if (items == nullptr) {
    return false;
  }

  const size_t itemCount = rxSize / sizeof(rmt_item32_t);
  bool found = false;
  for (size_t i = 0; i < itemCount; ++i) {
    const rmt_item32_t& item = items[i];
    if (item.level0 == 1) {
      outPulseUs = item.duration0;
      found = true;
      break;
    }
    if (item.level1 == 1) {
      outPulseUs = item.duration1;
      found = true;
      break;
    }
  }

  vRingbufferReturnItem(runtime.ring, items);
  rmt_rx_start(runtime.channel, false);
  return found;
}

void notifyConsumers() {
  TaskHandle_t handles[kMaxRcConsumers];
  size_t handleCount = 0;

  portENTER_CRITICAL(&g_consumerMux);
  for (const RcConsumerRegistry& entry : g_consumerRegistry) {
    if (entry.handle != nullptr && handleCount < kMaxRcConsumers) {
      handles[handleCount++] = entry.handle;
    }
  }
  portEXIT_CRITICAL(&g_consumerMux);

  for (size_t i = 0; i < handleCount; ++i) {
    xTaskNotifyGive(handles[i]);
  }
}

}  // namespace

void initFS_IA6(uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4, uint8_t ch5, uint8_t ch6) {
  pinMode(ch1, INPUT_PULLUP);
  pinMode(ch2, INPUT_PULLUP);
  pinMode(ch3, INPUT_PULLUP);
  pinMode(ch4, INPUT_PULLUP);
  pinMode(ch5, INPUT_PULLUP);
  pinMode(ch6, INPUT_PULLUP);
}

int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  (void)channelInput;
  (void)minLimit;
  (void)maxLimit;
  return defaultValue;
}

bool readSwitch(byte channelInput, bool defaultValue) {
  (void)channelInput;
  return defaultValue;
}

void initIBus(IBusBM& ibus, HardwareSerial& serial) { ibus.begin(serial); }

int readChannel(IBusBM& ibus, byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) {
    return defaultValue;
  }
  int value = map(ch, 1000, 2000, minLimit, maxLimit);
  return clampToRange(value, minLimit, maxLimit);
}

bool rcGetStateCopy(RcSharedState& out) {
  portENTER_CRITICAL(&g_rcStateMux);
  out = g_rcState;
  portEXIT_CRITICAL(&g_rcStateMux);
  return out.valid;
}

bool rcRegisterConsumer(TaskHandle_t handle) {
  if (handle == nullptr) {
    return false;
  }

  bool inserted = false;
  portENTER_CRITICAL(&g_consumerMux);
  for (RcConsumerRegistry& entry : g_consumerRegistry) {
    if (entry.handle == handle) {
      inserted = true;
      break;
    }
    if (!inserted && entry.handle == nullptr) {
      entry.handle = handle;
      inserted = true;
      break;
    }
  }
  portEXIT_CRITICAL(&g_consumerMux);
  return inserted;
}

void rcUnregisterConsumer(TaskHandle_t handle) {
  if (handle == nullptr) {
    return;
  }
  portENTER_CRITICAL(&g_consumerMux);
  for (RcConsumerRegistry& entry : g_consumerRegistry) {
    if (entry.handle == handle) {
      entry.handle = nullptr;
      break;
    }
  }
  portEXIT_CRITICAL(&g_consumerMux);
}

void taskRcSampler(void* parameter) {
  const FsIa6SamplerConfig* cfg = static_cast<const FsIa6SamplerConfig*>(parameter);
  const bool log = (cfg != nullptr) ? cfg->log : false;
  const TickType_t period = (cfg != nullptr && cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(10);
  const TickType_t staleThreshold =
      (cfg != nullptr && cfg->staleThreshold > 0) ? cfg->staleThreshold : pdMS_TO_TICKS(60);
  // A timeout of 0 is valid: non-blocking read for each channel.
  const TickType_t receiveTimeout = (cfg != nullptr) ? cfg->rmtReceiveTimeout : pdMS_TO_TICKS(5);

  if (!ensureRmtInitialized(log)) {
    broadcastIf(true, "[RC][RMT] No se pudo inicializar RMT para FS-iA6, finalizando tarea");
    vTaskDelete(nullptr);
    return;
  }

  TickType_t lastWake = xTaskGetTickCount();
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  int64_t lastIterationStartUs = esp_timer_get_time();

  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;
    const TickType_t now = xTaskGetTickCount();
    bool anyUpdated = false;

    for (RcChannelRuntime& runtime : g_rcChannels) {
      uint32_t pulseUs = 0;
      if (readRmtPulseUs(runtime, receiveTimeout, pulseUs)) {
        runtime.lastPulseUs = pulseUs;
        runtime.lastUpdateTick = now;
        anyUpdated = true;
      }
    }

    RcSharedState snapshot{};
    snapshot.ch0 = pulseWidthToRange(g_rcChannels[0].lastPulseUs, -100, 100);
    snapshot.ch2 = pulseWidthToRange(g_rcChannels[1].lastPulseUs, -100, 100);

    const bool throttleFresh = (now - g_rcChannels[2].lastUpdateTick) <= staleThreshold;
    const bool steeringFresh = (now - g_rcChannels[3].lastUpdateTick) <= staleThreshold;

    snapshot.throttle = throttleFresh ? pulseWidthToRange(g_rcChannels[2].lastPulseUs, -100, 100) : 0;
    snapshot.steering = steeringFresh ? pulseWidthToRange(g_rcChannels[3].lastPulseUs, -100, 100) : 0;
    snapshot.lastUpdateTick = std::max(g_rcChannels[2].lastUpdateTick, g_rcChannels[3].lastUpdateTick);
    snapshot.valid = throttleFresh || steeringFresh;

    portENTER_CRITICAL(&g_rcStateMux);
    g_rcState = snapshot;
    portEXIT_CRITICAL(&g_rcStateMux);

    if (log && anyUpdated) {
      const TickType_t lastLogDelta = now - g_lastRcLogTick;
      if (lastLogDelta >= pdMS_TO_TICKS(500)) {
        String rcMsg = "RC sampler -> AUX1 GPIO" + String(kRcAux1Pin) + ": " + String(snapshot.ch0) +
                       " | AUX2 GPIO" + String(kRcAux2Pin) + ": " + String(snapshot.ch2) +
                       " | acelerador GPIO" + String(kRcThrottlePin) + ": " + String(snapshot.throttle) +
                       " | direccion GPIO" + String(kRcSteeringPin) + ": " + String(snapshot.steering);
        broadcastIf(true, rcMsg);
        g_lastRcLogTick = now;
      }
    }

    if (anyUpdated) {
      notifyConsumers();
    }

    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kRcSampler, cycleUs, expectedPeriodUs, overrun, false);

    vTaskDelayUntil(&lastWake, period);
  }
}

void taskRcMonitor(void* parameter) {
  const FsIa6TaskConfig* cfg = static_cast<const FsIa6TaskConfig*>(parameter);
  const bool log = (cfg != nullptr) ? cfg->log : false;
  const TickType_t period = (cfg != nullptr) ? cfg->period : pdMS_TO_TICKS(100);

  int lastCh0 = 9999;
  int lastCh2 = 9999;
  int lastThrottle = 9999;
  int lastSteering = 9999;

  for (;;) {
    RcSharedState snapshot{};
    if (!rcGetStateCopy(snapshot)) {
      vTaskDelay(period);
      continue;
    }

    const int ch0 = snapshot.ch0;
    const int ch2 = snapshot.ch2;
    const int throttle = snapshot.throttle;
    const int steering = snapshot.steering;

    if (ch0 != lastCh0 || ch2 != lastCh2 || throttle != lastThrottle || steering != lastSteering) {
      if (log) {
        String rcMsg = "FS-iA6 -> AUX1 GPIO" + String(kRcAux1Pin) + ": " + String(ch0) +
                       " | AUX2 GPIO" + String(kRcAux2Pin) + ": " + String(ch2) +
                       " | acelerador GPIO" + String(kRcThrottlePin) + ": " + String(throttle) +
                       " | direccion GPIO" + String(kRcSteeringPin) + ": " + String(steering);
        broadcastIf(true, rcMsg);
      }
      lastCh0 = ch0;
      lastCh2 = ch2;
      lastThrottle = throttle;
      lastSteering = steering;
    }
    vTaskDelay(period);
  }
}
