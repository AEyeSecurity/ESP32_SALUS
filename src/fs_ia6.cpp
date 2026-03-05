#include "fs_ia6.h"

#include <algorithm>
#include <array>

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

constexpr rmt_channel_t kRcRmtChannel = RMT_CHANNEL_0;
constexpr uint8_t kPpmMaxChannels = 12;
constexpr uint8_t kPpmSteeringIdx = 0;   // CH1
constexpr uint8_t kPpmThrottleIdx = 1;   // CH2
constexpr uint8_t kPpmAux1Idx = 4;       // CH5
constexpr uint8_t kPpmAux2Idx = 5;       // CH6
constexpr uint8_t kPpmDebugChannels = 8; // Mostrar siempre CH1..CH8 en logs
constexpr uint16_t kPpmDefaultPulseUs = 1500;
constexpr uint16_t kPpmMarkerMinUs = 100;
constexpr uint16_t kPpmMarkerMaxUs = 500;
constexpr uint16_t kPpmChannelMinUs = 900;
constexpr uint16_t kPpmChannelMaxUs = 2100;
constexpr uint16_t kPpmSyncGapUs = 2700;
constexpr uint8_t kPpmMinValidChannels = 4;
constexpr uint8_t kPpmDefaultMarkerLevel = 0;

struct RcConsumerRegistry {
  TaskHandle_t handle = nullptr;
};

struct ParsedPpmFrame {
  bool valid = false;
  uint8_t channelCount = 0;
  std::array<uint16_t, kPpmMaxChannels> channels{};
};

struct PpmRuntime {
  uint8_t pin = kRcPpmPin;
  rmt_channel_t channel = kRcRmtChannel;
  RingbufHandle_t ring = nullptr;
  std::array<uint16_t, kPpmMaxChannels> lastChannels{};
  uint8_t lastChannelCount = 0;
  TickType_t lastFrameTick = 0;
  bool polarityLocked = false;
  uint8_t markerLevel = kPpmDefaultMarkerLevel;
};

portMUX_TYPE g_rcStateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_consumerMux = portMUX_INITIALIZER_UNLOCKED;

RcSharedState g_rcState = {0, 0, 0, 0, 0, false};
TickType_t g_lastRcLogTick = 0;
bool g_rmtInitialized = false;
PpmRuntime g_ppmRuntime;

RcConsumerRegistry g_consumerRegistry[kMaxRcConsumers];

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

bool isMarkerSegment(uint8_t level, uint16_t durationUs, uint8_t markerLevel) {
  return level == markerLevel && durationUs >= kPpmMarkerMinUs && durationUs <= kPpmMarkerMaxUs;
}

ParsedPpmFrame parsePpmWithMarkerLevel(const rmt_item32_t* items, size_t itemCount, uint8_t markerLevel) {
  ParsedPpmFrame bestFrame;
  std::array<uint16_t, kPpmMaxChannels> currentChannels{};
  uint8_t currentCount = 0;
  bool haveMarker = false;
  uint32_t markerToMarkerUs = 0;

  for (size_t i = 0; i < itemCount; ++i) {
    const rmt_item32_t& item = items[i];
    const uint8_t levels[2] = {
        static_cast<uint8_t>(item.level0),
        static_cast<uint8_t>(item.level1),
    };
    const uint16_t durations[2] = {
        static_cast<uint16_t>(item.duration0),
        static_cast<uint16_t>(item.duration1),
    };

    for (uint8_t part = 0; part < 2; ++part) {
      const uint16_t durationUs = durations[part];
      if (durationUs == 0) {
        continue;
      }

      const uint8_t level = levels[part];
      if (isMarkerSegment(level, durationUs, markerLevel)) {
        if (haveMarker) {
          const uint32_t intervalUs = markerToMarkerUs;
          if (intervalUs >= kPpmSyncGapUs) {
            if (currentCount >= kPpmMinValidChannels && currentCount > bestFrame.channelCount) {
              bestFrame.valid = true;
              bestFrame.channelCount = currentCount;
              bestFrame.channels = currentChannels;
            }
            currentCount = 0;
          } else if (intervalUs >= kPpmChannelMinUs && intervalUs <= kPpmChannelMaxUs) {
            if (currentCount < kPpmMaxChannels) {
              currentChannels[currentCount++] = static_cast<uint16_t>(intervalUs);
            }
          } else {
            currentCount = 0;
          }
        }

        markerToMarkerUs = durationUs;
        haveMarker = true;
      } else if (haveMarker) {
        markerToMarkerUs += durationUs;
      }
    }
  }

  if (currentCount >= kPpmMinValidChannels && currentCount > bestFrame.channelCount) {
    bestFrame.valid = true;
    bestFrame.channelCount = currentCount;
    bestFrame.channels = currentChannels;
  }

  return bestFrame;
}

bool decodePpmFrame(const rmt_item32_t* items,
                    size_t itemCount,
                    bool log,
                    uint8_t& inOutMarkerLevel,
                    bool& inOutPolarityLocked,
                    ParsedPpmFrame& outFrame) {
  if (itemCount == 0) {
    return false;
  }

  const ParsedPpmFrame frameLevel0 = parsePpmWithMarkerLevel(items, itemCount, 0);
  const ParsedPpmFrame frameLevel1 = parsePpmWithMarkerLevel(items, itemCount, 1);

  const bool valid0 = frameLevel0.valid;
  const bool valid1 = frameLevel1.valid;
  if (!valid0 && !valid1) {
    return false;
  }

  uint8_t chosenLevel = inOutMarkerLevel;
  ParsedPpmFrame chosenFrame{};

  if (inOutPolarityLocked) {
    const bool lockLevelIs0 = (inOutMarkerLevel == 0);
    const ParsedPpmFrame& lockedFrame = lockLevelIs0 ? frameLevel0 : frameLevel1;
    const ParsedPpmFrame& otherFrame = lockLevelIs0 ? frameLevel1 : frameLevel0;
    if (lockedFrame.valid) {
      outFrame = lockedFrame;
      return true;
    }
    if (otherFrame.valid) {
      chosenLevel = lockLevelIs0 ? 1 : 0;
      chosenFrame = otherFrame;
    } else {
      return false;
    }
  } else if (valid0 && valid1) {
    if (frameLevel0.channelCount >= frameLevel1.channelCount) {
      chosenLevel = 0;
      chosenFrame = frameLevel0;
    } else {
      chosenLevel = 1;
      chosenFrame = frameLevel1;
    }
  } else if (valid0) {
    chosenLevel = 0;
    chosenFrame = frameLevel0;
  } else {
    chosenLevel = 1;
    chosenFrame = frameLevel1;
  }

  if (!inOutPolarityLocked || inOutMarkerLevel != chosenLevel) {
    inOutPolarityLocked = true;
    inOutMarkerLevel = chosenLevel;
    if (log) {
      String msg = "[RC][PPM] Polaridad bloqueada markerLevel=";
      msg += chosenLevel;
      msg += " canales=";
      msg += chosenFrame.channelCount;
      broadcastIf(true, msg);
    }
  }

  outFrame = chosenFrame;
  return true;
}

bool configurePpmRmt(PpmRuntime& runtime, bool logErrors) {
  rmt_config_t config{};
  config.rmt_mode = RMT_MODE_RX;
  config.channel = runtime.channel;
  config.clk_div = 80;  // 1 tick = 1 microsecond
  config.gpio_num = static_cast<gpio_num_t>(runtime.pin);
  config.mem_block_num = 1;
  config.rx_config.filter_en = true;
  config.rx_config.filter_ticks_thresh = 80;
  config.rx_config.idle_threshold = 3000;

  esp_err_t err = rmt_config(&config);
  if (err != ESP_OK) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error configurando RX PPM err=") + err);
    }
    return false;
  }

  err = rmt_driver_install(runtime.channel, 2048, 0);
  if (err != ESP_OK) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error instalando driver PPM err=") + err);
    }
    return false;
  }

  err = rmt_get_ringbuf_handle(runtime.channel, &runtime.ring);
  if (err != ESP_OK || runtime.ring == nullptr) {
    if (logErrors) {
      broadcastIf(true, "[RC][RMT] Error obteniendo ringbuffer PPM");
    }
    return false;
  }

  err = rmt_rx_start(runtime.channel, true);
  if (err != ESP_OK) {
    if (logErrors) {
      broadcastIf(true, String("[RC][RMT] Error iniciando RX PPM err=") + err);
    }
    return false;
  }

  runtime.lastChannels.fill(kPpmDefaultPulseUs);
  runtime.lastChannelCount = 0;
  runtime.lastFrameTick = 0;
  runtime.markerLevel = kPpmDefaultMarkerLevel;
  runtime.polarityLocked = false;
  return true;
}

bool ensureRmtInitialized(bool logErrors) {
  if (g_rmtInitialized) {
    return true;
  }

  g_rmtInitialized = configurePpmRmt(g_ppmRuntime, logErrors);
  return g_rmtInitialized;
}

bool readRmtItems(PpmRuntime& runtime, TickType_t timeoutTicks, rmt_item32_t*& outItems, size_t& outItemCount) {
  outItems = nullptr;
  outItemCount = 0;
  if (runtime.ring == nullptr) {
    return false;
  }

  size_t rxSize = 0;
  rmt_item32_t* items = static_cast<rmt_item32_t*>(xRingbufferReceive(runtime.ring, &rxSize, timeoutTicks));
  if (items == nullptr || rxSize == 0) {
    return false;
  }

  outItems = items;
  outItemCount = rxSize / sizeof(rmt_item32_t);
  return outItemCount > 0;
}

int ppmChannelToRange(const PpmRuntime& runtime, uint8_t channelIndex, int minLimit, int maxLimit, int defaultValue) {
  if (channelIndex >= runtime.lastChannelCount || channelIndex >= runtime.lastChannels.size()) {
    return defaultValue;
  }
  return pulseWidthToRange(runtime.lastChannels[channelIndex], minLimit, maxLimit);
}

String buildPpmChannelsSummary(const PpmRuntime& runtime) {
  String msg;
  msg.reserve(120);
  for (uint8_t idx = 0; idx < kPpmDebugChannels; ++idx) {
    if (idx > 0) {
      msg += " ";
    }
    msg += "CH";
    msg += String(static_cast<int>(idx) + 1);
    msg += "=";
    if (idx < runtime.lastChannelCount && idx < runtime.lastChannels.size()) {
      msg += String(pulseWidthToRange(runtime.lastChannels[idx], -100, 100));
    } else {
      msg += "0";
    }
  }
  return msg;
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
  const TickType_t receiveTimeout = (cfg != nullptr) ? cfg->rmtReceiveTimeout : pdMS_TO_TICKS(5);

  if (!ensureRmtInitialized(log)) {
    broadcastIf(true, "[RC][RMT] No se pudo inicializar RMT para PPM, finalizando tarea");
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

    rmt_item32_t* items = nullptr;
    size_t itemCount = 0;
    if (readRmtItems(g_ppmRuntime, receiveTimeout, items, itemCount)) {
      ParsedPpmFrame decodedFrame;
      if (decodePpmFrame(items,
                         itemCount,
                         log,
                         g_ppmRuntime.markerLevel,
                         g_ppmRuntime.polarityLocked,
                         decodedFrame)) {
        g_ppmRuntime.lastChannels = decodedFrame.channels;
        g_ppmRuntime.lastChannelCount = decodedFrame.channelCount;
        g_ppmRuntime.lastFrameTick = now;
        anyUpdated = true;
      }

      vRingbufferReturnItem(g_ppmRuntime.ring, items);
      rmt_rx_start(g_ppmRuntime.channel, false);
    }

    RcSharedState snapshot{};
    const bool fresh = g_ppmRuntime.lastFrameTick != 0 && (now - g_ppmRuntime.lastFrameTick) <= staleThreshold;
    if (fresh) {
      snapshot.steering = ppmChannelToRange(g_ppmRuntime, kPpmSteeringIdx, -100, 100, 0);
      snapshot.throttle = ppmChannelToRange(g_ppmRuntime, kPpmThrottleIdx, -100, 100, 0);
      snapshot.ch0 = ppmChannelToRange(g_ppmRuntime, kPpmAux1Idx, -100, 100, 0);
      snapshot.ch2 = ppmChannelToRange(g_ppmRuntime, kPpmAux2Idx, -100, 100, 0);
      snapshot.lastUpdateTick = g_ppmRuntime.lastFrameTick;
      snapshot.valid = true;
    } else {
      snapshot.ch0 = 0;
      snapshot.ch2 = 0;
      snapshot.throttle = 0;
      snapshot.steering = 0;
      snapshot.lastUpdateTick = g_ppmRuntime.lastFrameTick;
      snapshot.valid = false;
    }

    portENTER_CRITICAL(&g_rcStateMux);
    g_rcState = snapshot;
    portEXIT_CRITICAL(&g_rcStateMux);

    if (log && anyUpdated) {
      const TickType_t lastLogDelta = now - g_lastRcLogTick;
      if (lastLogDelta >= pdMS_TO_TICKS(500)) {
        String rcMsg = "[RC] " + buildPpmChannelsSummary(g_ppmRuntime);
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
        String rcMsg = "[RC][PPM] monitor GPIO" + String(kRcPpmPin) + " -> CH1 steer: " + String(steering) +
                       " | CH2 throttle: " + String(throttle) + " | CH5 aux1: " + String(ch0) +
                       " | CH6 aux2: " + String(ch2);
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
