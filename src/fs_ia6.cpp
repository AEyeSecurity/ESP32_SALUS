#include "fs_ia6.h"

#include <algorithm>
#include <array>

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <esp_err.h>
#include <esp_timer.h>
#include <soc/gpio_struct.h>

#include "ota_telnet.h"
#include "system_diag.h"

#ifndef RC_GPIO_EDGE_DEBUG
#define RC_GPIO_EDGE_DEBUG 0
#endif

#ifndef RC_GPIO_PPM_FALLBACK
#define RC_GPIO_PPM_FALLBACK 0
#endif

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
constexpr uint8_t kPpmRequiredChannels = kPpmDebugChannels;
constexpr uint8_t kPpmFramesToTrust = 3;
constexpr uint8_t kPpmDefaultMarkerLevel = 0;
constexpr TickType_t kRmtStallRestartTicks = pdMS_TO_TICKS(200);
constexpr TickType_t kRmtRestartCooldownTicks = pdMS_TO_TICKS(500);
constexpr bool kEnableGpioPpmFallback = RC_GPIO_PPM_FALLBACK != 0;
constexpr bool kEnableGpioEdgeDebug = (RC_GPIO_EDGE_DEBUG != 0) || kEnableGpioPpmFallback;

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
  uint8_t trustedFrameCount = 0;
  bool polarityLocked = false;
  uint8_t markerLevel = kPpmDefaultMarkerLevel;
};

portMUX_TYPE g_rcStateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_rcRawDebugMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_rcGpioPpmMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_consumerMux = portMUX_INITIALIZER_UNLOCKED;

RcSharedState g_rcState = {0, 0, 0, 0, 0, false};
RcRawDebugSnapshot g_rcRawDebug{};
TickType_t g_lastRcLogTick = 0;
bool g_rmtInitialized = false;
PpmRuntime g_ppmRuntime;

RcConsumerRegistry g_consumerRegistry[kMaxRcConsumers];
volatile uint32_t g_rcGpioEdgeCount = 0;
volatile uint32_t g_rcLastGpioEdgeUs = 0;
volatile uint8_t g_rcLastGpioLevel = 0;
bool g_rcGpioEdgeMonitorInitialized = false;
volatile uint32_t g_gpioPpmLastMarkerUs = 0;
volatile uint8_t g_gpioPpmWorkingCount = 0;
volatile uint16_t g_gpioPpmWorkingChannels[kPpmMaxChannels] = {};
volatile uint16_t g_gpioPpmFrameChannels[kPpmMaxChannels] = {};
volatile uint8_t g_gpioPpmFrameChannelCount = 0;
volatile bool g_gpioPpmFrameAvailable = false;
volatile uint32_t g_gpioPpmFrameCount = 0;
volatile uint32_t g_gpioPpmErrorCount = 0;
volatile uint32_t g_gpioPpmLastFrameUs = 0;

void IRAM_ATTR onRcPpmGpioChange() {
  const uint32_t nowUs = micros();
  const uint8_t level = static_cast<uint8_t>((GPIO.in >> kRcPpmPin) & 0x01U);
  g_rcGpioEdgeCount++;
  g_rcLastGpioEdgeUs = nowUs;
  g_rcLastGpioLevel = level;

  if (level != kPpmDefaultMarkerLevel) {
    return;
  }

  portENTER_CRITICAL_ISR(&g_rcGpioPpmMux);
  if (g_gpioPpmLastMarkerUs != 0) {
    const uint32_t intervalUs = nowUs - g_gpioPpmLastMarkerUs;
    if (intervalUs >= kPpmSyncGapUs) {
      if (g_gpioPpmWorkingCount >= kPpmMinValidChannels) {
        for (uint8_t i = 0; i < kPpmMaxChannels; ++i) {
          g_gpioPpmFrameChannels[i] = (i < g_gpioPpmWorkingCount) ? g_gpioPpmWorkingChannels[i] : 0;
        }
        g_gpioPpmFrameChannelCount = g_gpioPpmWorkingCount;
        g_gpioPpmFrameAvailable = true;
        g_gpioPpmFrameCount++;
        g_gpioPpmLastFrameUs = nowUs;
      }
      g_gpioPpmWorkingCount = 0;
    } else if (intervalUs >= kPpmChannelMinUs && intervalUs <= kPpmChannelMaxUs) {
      if (g_gpioPpmWorkingCount < kPpmMaxChannels) {
        g_gpioPpmWorkingChannels[g_gpioPpmWorkingCount++] = static_cast<uint16_t>(intervalUs);
      } else {
        g_gpioPpmWorkingCount = 0;
        g_gpioPpmErrorCount++;
      }
    } else if (intervalUs > kPpmMarkerMaxUs) {
      g_gpioPpmWorkingCount = 0;
      g_gpioPpmErrorCount++;
    }
  }
  g_gpioPpmLastMarkerUs = nowUs;
  portEXIT_CRITICAL_ISR(&g_rcGpioPpmMux);
}

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
  runtime.trustedFrameCount = 0;
  runtime.markerLevel = kPpmDefaultMarkerLevel;
  runtime.polarityLocked = false;
  return true;
}

void ensureGpioEdgeMonitorInitialized() {
  if (!kEnableGpioEdgeDebug) {
    return;
  }
  if (g_rcGpioEdgeMonitorInitialized) {
    return;
  }
  g_rcLastGpioLevel = static_cast<uint8_t>(digitalRead(kRcPpmPin));
  attachInterrupt(digitalPinToInterrupt(kRcPpmPin), onRcPpmGpioChange, CHANGE);
  g_rcGpioEdgeMonitorInitialized = true;
}

bool ensureRmtInitialized(bool logErrors) {
  if (g_rmtInitialized) {
    ensureGpioEdgeMonitorInitialized();
    return true;
  }

  g_rmtInitialized = configurePpmRmt(g_ppmRuntime, logErrors);
  if (g_rmtInitialized) {
    ensureGpioEdgeMonitorInitialized();
  }
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

bool readGpioPpmFrame(ParsedPpmFrame& outFrame) {
  outFrame = {};
  if (!kEnableGpioPpmFallback) {
    return false;
  }
  bool available = false;
  uint8_t channelCount = 0;
  std::array<uint16_t, kPpmMaxChannels> channels{};

  portENTER_CRITICAL(&g_rcGpioPpmMux);
  if (g_gpioPpmFrameAvailable) {
    available = true;
    channelCount = g_gpioPpmFrameChannelCount;
    for (uint8_t i = 0; i < kPpmMaxChannels; ++i) {
      channels[i] = g_gpioPpmFrameChannels[i];
    }
    g_gpioPpmFrameAvailable = false;
  }
  portEXIT_CRITICAL(&g_rcGpioPpmMux);

  if (!available || channelCount < kPpmMinValidChannels) {
    return false;
  }

  outFrame.valid = true;
  outFrame.channelCount = channelCount;
  outFrame.channels = channels;
  return true;
}

bool hasRequiredPpmChannels(const ParsedPpmFrame& frame) {
  return frame.valid && frame.channelCount >= kPpmRequiredChannels;
}

void recordRmtRestartResult(PpmRuntime& runtime, TickType_t now, bool ok, esp_err_t err, bool resetTrust) {
  if (resetTrust) {
    runtime.trustedFrameCount = 0;
  }
  portENTER_CRITICAL(&g_rcRawDebugMux);
  g_rcRawDebug.rmtRestarts++;
  g_rcRawDebug.lastRmtRestartTick = now;
  g_rcRawDebug.trustedFrameCount = runtime.trustedFrameCount;
  g_rcRawDebug.rmtInitialized = g_rmtInitialized;
  g_rcRawDebug.gpioEdges = g_rcGpioEdgeCount;
  g_rcRawDebug.gpioPpmFrames = g_gpioPpmFrameCount;
  g_rcRawDebug.gpioPpmErrors = g_gpioPpmErrorCount;
  g_rcRawDebug.lastGpioEdgeUs = g_rcLastGpioEdgeUs;
  g_rcRawDebug.gpioLevel = g_rcLastGpioLevel;
  if (!ok) {
    g_rcRawDebug.rmtRestartErrors++;
    g_rcRawDebug.lastRmtRestartErr = static_cast<int>(err);
  } else {
    g_rcRawDebug.lastRmtRestartErr = 0;
  }
  portEXIT_CRITICAL(&g_rcRawDebugMux);
}

bool rearmPpmRmt(PpmRuntime& runtime, TickType_t now, bool clearMemory, bool logErrors) {
  rmt_rx_stop(runtime.channel);
  esp_err_t startErr = rmt_rx_start(runtime.channel, clearMemory);
  const bool ok = (startErr == ESP_OK);
  if (clearMemory || !ok) {
    recordRmtRestartResult(runtime, now, ok, startErr, clearMemory);
  }

  if (!ok && logErrors) {
    broadcastIf(true, String("[RC][RMT] Error reiniciando RX PPM err=") + startErr);
  }
  return ok;
}

bool reinstallPpmRmt(PpmRuntime& runtime, TickType_t now, bool logErrors) {
  rmt_rx_stop(runtime.channel);
  rmt_driver_uninstall(runtime.channel);
  runtime.ring = nullptr;
  g_rmtInitialized = false;

  const bool ok = configurePpmRmt(runtime, logErrors);
  g_rmtInitialized = ok;
  recordRmtRestartResult(runtime, now, ok, ok ? ESP_OK : ESP_FAIL, true);
  return ok;
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

void recordRawRmtBurst(const rmt_item32_t* items, size_t itemCount, TickType_t now) {
  portENTER_CRITICAL(&g_rcRawDebugMux);
  g_rcRawDebug.rmtInitialized = g_rmtInitialized;
  g_rcRawDebug.gpioEdges = g_rcGpioEdgeCount;
  g_rcRawDebug.gpioPpmFrames = g_gpioPpmFrameCount;
  g_rcRawDebug.gpioPpmErrors = g_gpioPpmErrorCount;
  g_rcRawDebug.lastGpioEdgeUs = g_rcLastGpioEdgeUs;
  g_rcRawDebug.gpioLevel = g_rcLastGpioLevel;
  g_rcRawDebug.ppmPolarityLocked = g_ppmRuntime.polarityLocked;
  g_rcRawDebug.ppmMarkerLevel = g_ppmRuntime.markerLevel;
  g_rcRawDebug.trustedFrameCount = g_ppmRuntime.trustedFrameCount;
  g_rcRawDebug.rawBursts++;
  g_rcRawDebug.lastRawTick = now;
  g_rcRawDebug.lastRawItemCount = itemCount;
  g_rcRawDebug.sampleItemCount = static_cast<uint8_t>(std::min<size_t>(itemCount, 4));
  for (uint8_t i = 0; i < 4; ++i) {
    if (i < g_rcRawDebug.sampleItemCount) {
      g_rcRawDebug.sampleDuration0[i] = static_cast<uint16_t>(items[i].duration0);
      g_rcRawDebug.sampleDuration1[i] = static_cast<uint16_t>(items[i].duration1);
      g_rcRawDebug.sampleLevel0[i] = static_cast<uint8_t>(items[i].level0);
      g_rcRawDebug.sampleLevel1[i] = static_cast<uint8_t>(items[i].level1);
    } else {
      g_rcRawDebug.sampleDuration0[i] = 0;
      g_rcRawDebug.sampleDuration1[i] = 0;
      g_rcRawDebug.sampleLevel0[i] = 0;
      g_rcRawDebug.sampleLevel1[i] = 0;
    }
  }
  portEXIT_CRITICAL(&g_rcRawDebugMux);
}

void recordPpmDecodeResult(bool decoded, const ParsedPpmFrame& frame, TickType_t now) {
  portENTER_CRITICAL(&g_rcRawDebugMux);
  g_rcRawDebug.rmtInitialized = g_rmtInitialized;
  g_rcRawDebug.gpioEdges = g_rcGpioEdgeCount;
  g_rcRawDebug.lastGpioEdgeUs = g_rcLastGpioEdgeUs;
  g_rcRawDebug.gpioLevel = g_rcLastGpioLevel;
  g_rcRawDebug.ppmPolarityLocked = g_ppmRuntime.polarityLocked;
  g_rcRawDebug.ppmMarkerLevel = g_ppmRuntime.markerLevel;
  g_rcRawDebug.trustedFrameCount = g_ppmRuntime.trustedFrameCount;
  if (decoded) {
    g_rcRawDebug.decodedFrames++;
    g_rcRawDebug.lastDecodedTick = now;
    g_rcRawDebug.lastChannelCount = frame.channelCount;
  } else {
    g_rcRawDebug.decodeFailures++;
    g_rcRawDebug.lastChannelCount = 0;
  }
  portEXIT_CRITICAL(&g_rcRawDebugMux);
}

void recordGpioPpmFrame(const ParsedPpmFrame& frame, TickType_t now) {
  portENTER_CRITICAL(&g_rcRawDebugMux);
  g_rcRawDebug.rmtInitialized = g_rmtInitialized;
  g_rcRawDebug.gpioEdges = g_rcGpioEdgeCount;
  g_rcRawDebug.gpioPpmFrames = g_gpioPpmFrameCount;
  g_rcRawDebug.gpioPpmErrors = g_gpioPpmErrorCount;
  g_rcRawDebug.lastGpioEdgeUs = g_rcLastGpioEdgeUs;
  g_rcRawDebug.gpioLevel = g_rcLastGpioLevel;
  g_rcRawDebug.lastGpioFrameTick = now;
  g_rcRawDebug.lastChannelCount = frame.channelCount;
  g_rcRawDebug.trustedFrameCount = g_ppmRuntime.trustedFrameCount;
  portEXIT_CRITICAL(&g_rcRawDebugMux);
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

bool rcGetRawDebugSnapshot(RcRawDebugSnapshot& out) {
  portENTER_CRITICAL(&g_rcRawDebugMux);
  out = g_rcRawDebug;
  out.gpioEdges = g_rcGpioEdgeCount;
  out.gpioPpmFrames = g_gpioPpmFrameCount;
  out.gpioPpmErrors = g_gpioPpmErrorCount;
  out.lastGpioEdgeUs = g_rcLastGpioEdgeUs;
  out.gpioLevel = g_rcLastGpioLevel;
  portEXIT_CRITICAL(&g_rcRawDebugMux);
  return out.rmtInitialized;
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
  TickType_t lastRmtRestartAttemptTick = 0;

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
      recordRawRmtBurst(items, itemCount, now);
      ParsedPpmFrame decodedFrame;
      const bool decoded = decodePpmFrame(items,
                                          itemCount,
                                          log,
                                          g_ppmRuntime.markerLevel,
                                          g_ppmRuntime.polarityLocked,
                                          decodedFrame);
      const bool accepted = decoded && hasRequiredPpmChannels(decodedFrame);
      if (accepted) {
        g_ppmRuntime.lastChannels = decodedFrame.channels;
        g_ppmRuntime.lastChannelCount = decodedFrame.channelCount;
        g_ppmRuntime.lastFrameTick = now;
        if (g_ppmRuntime.trustedFrameCount < kPpmFramesToTrust) {
          g_ppmRuntime.trustedFrameCount++;
        }
        anyUpdated = true;
      } else {
        g_ppmRuntime.trustedFrameCount = 0;
      }
      recordPpmDecodeResult(accepted, decodedFrame, now);

      vRingbufferReturnItem(g_ppmRuntime.ring, items);
      rearmPpmRmt(g_ppmRuntime, now, false, log);
    } else {
      ParsedPpmFrame gpioFrame;
      if (kEnableGpioPpmFallback && readGpioPpmFrame(gpioFrame) && hasRequiredPpmChannels(gpioFrame)) {
        g_ppmRuntime.lastChannels = gpioFrame.channels;
        g_ppmRuntime.lastChannelCount = gpioFrame.channelCount;
        g_ppmRuntime.lastFrameTick = now;
        if (g_ppmRuntime.trustedFrameCount < kPpmFramesToTrust) {
          g_ppmRuntime.trustedFrameCount++;
        }
        anyUpdated = true;
        recordGpioPpmFrame(gpioFrame, now);
      } else if (g_ppmRuntime.lastFrameTick != 0 &&
	                 (now - g_ppmRuntime.lastFrameTick) >= kRmtStallRestartTicks &&
	                 (lastRmtRestartAttemptTick == 0 ||
	                  (now - lastRmtRestartAttemptTick) >= kRmtRestartCooldownTicks)) {
        reinstallPpmRmt(g_ppmRuntime, now, log);
        lastRmtRestartAttemptTick = now;
      }
    }

    RcSharedState snapshot{};
    const bool fresh = g_ppmRuntime.lastFrameTick != 0 &&
                       g_ppmRuntime.trustedFrameCount >= kPpmFramesToTrust &&
                       (now - g_ppmRuntime.lastFrameTick) <= staleThreshold;
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
      if (g_ppmRuntime.lastFrameTick == 0 || (now - g_ppmRuntime.lastFrameTick) > staleThreshold) {
        g_ppmRuntime.trustedFrameCount = 0;
      }
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
