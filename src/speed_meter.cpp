#include "speed_meter.h"

#include <freertos/queue.h>

#include "ota_telnet.h"

namespace {

constexpr size_t kFrameBufferSize = 256;
constexpr int kVoteWindowSize = 3;
constexpr int kVoteMinVotes = 2;
constexpr int kConfirmRepeats = 2;
constexpr uint32_t kOffDecayStepMs = 300;
constexpr uint32_t kOffForceZeroMs = 1000;
constexpr TickType_t kLogMinInterval = pdMS_TO_TICKS(500);

SpeedMeterConfig g_config{};
QueueHandle_t g_uartQueue = nullptr;
bool g_initialized = false;

portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
SpeedMeterSnapshot g_snapshot = {false, false, 0, -1, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

uint8_t g_frame[kFrameBufferSize]{};
size_t g_frameLen = 0;
uint32_t g_lastByteUs = 0;

int g_voteWindow[kVoteWindowSize] = {-1, -1, -1};
int g_voteCount = 0;
int g_voteHead = 0;
int g_pendingSpeedChange = -1;
int g_pendingSpeedChangeCount = 0;

bool g_haveThrottle = false;
uint32_t g_throttleOffSinceMs = 0;
uint32_t g_lastOffDecayStepMs = 0;

TickType_t g_lastLogTick = 0;

int clamp100(int value) {
  if (value < 0) {
    return 0;
  }
  if (value > 100) {
    return 100;
  }
  return value;
}

int absDiff(int a, int b) {
  return (a > b) ? (a - b) : (b - a);
}

int continuityScore(int candidate, int currentSpeed) {
  if (currentSpeed < 0) {
    return 5;
  }
  const int delta = absDiff(candidate, currentSpeed);
  if (delta <= 1) {
    return 15;
  }
  if (delta <= 3) {
    return 11;
  }
  if (delta <= 5) {
    return 6;
  }
  if (delta <= 7) {
    return 1;
  }
  return -8;
}

int candidateConfidence(int candidate, bool throttle, int frameLen, int currentSpeed) {
  if (candidate < 0) {
    return 0;
  }
  int conf = 40;
  conf += continuityScore(candidate, currentSpeed);
  conf += (frameLen == 25 || frameLen == 26) ? 5 : -10;
  if (!throttle && currentSpeed >= 0 && candidate > currentSpeed) {
    conf -= 30;
  }
  return clamp100(conf);
}

int speedFromB16B17(uint8_t b16, uint8_t b17) {
  switch (b16) {
    case 0xDB:
      if (b17 == 0xDB) {
        return 0;
      }
      if (b17 == 0xB2 || b17 == 0x92) {
        return 2;
      }
      break;
    case 0x5B:
      if (b17 == 0xD9 || b17 == 0x59) {
        return 3;
      }
      if (b17 == 0xC9 || b17 == 0x49 || b17 == 0x96) {
        return 4;
      }
      if (b17 == 0xB2 || b17 == 0x92) {
        return 5;
      }
      break;
    case 0xCB:
      if (b17 == 0x59 || b17 == 0x5B || b17 == 0x4B) {
        return 6;
      }
      if (b17 == 0xB6 || b17 == 0x49 || b17 == 0xB2) {
        return 7;
      }
      if (b17 == 0x92 || b17 == 0xDB) {
        return 8;
      }
      break;
    case 0x4B:
      if (b17 == 0xB6 || b17 == 0xC9 || b17 == 0x59 || b17 == 0x49 || b17 == 0xD9) {
        return 9;
      }
      if (b17 == 0x92 || b17 == 0x96 || b17 == 0xB2) {
        return 10;
      }
      break;
    case 0xD9:
      if (b17 == 0x9B || b17 == 0x5B || b17 == 0xD9) {
        return 11;
      }
      if (b17 == 0xB6 || b17 == 0xC9 || b17 == 0x59) {
        return 12;
      }
      if (b17 == 0xB2 || b17 == 0x96 || b17 == 0x92) {
        return 13;
      }
      break;
    case 0x59:
      if (b17 == 0xD9) {
        return 14;
      }
      break;
    default:
      break;
  }
  return -1;
}

void resetSpeedVotes() {
  for (int i = 0; i < kVoteWindowSize; ++i) {
    g_voteWindow[i] = -1;
  }
  g_voteCount = 0;
  g_voteHead = 0;
  g_pendingSpeedChange = -1;
  g_pendingSpeedChangeCount = 0;
}

void pushSpeedVote(int speed) {
  g_voteWindow[g_voteHead] = speed;
  g_voteHead = (g_voteHead + 1) % kVoteWindowSize;
  if (g_voteCount < kVoteWindowSize) {
    g_voteCount++;
  }
}

int filteredSpeedFromVotes() {
  if (g_voteCount < kVoteMinVotes) {
    return -1;
  }

  constexpr int kMaxSpeed = 64;
  int counts[kMaxSpeed + 1] = {0};
  for (int i = 0; i < g_voteCount; ++i) {
    const int speed = g_voteWindow[i];
    if (speed >= 0 && speed <= kMaxSpeed) {
      counts[speed]++;
    }
  }

  int currentSpeed = -1;
  portENTER_CRITICAL(&g_stateMux);
  currentSpeed = g_snapshot.speedKmh;
  portEXIT_CRITICAL(&g_stateMux);

  int bestSpeed = -1;
  int bestCount = 0;
  for (int speed = 0; speed <= kMaxSpeed; ++speed) {
    const int count = counts[speed];
    if (count > bestCount) {
      bestCount = count;
      bestSpeed = speed;
    } else if (count == bestCount && count > 0 && currentSpeed >= 0 && bestSpeed >= 0) {
      if (absDiff(speed, currentSpeed) < absDiff(bestSpeed, currentSpeed)) {
        bestSpeed = speed;
      }
    }
  }

  return (bestCount >= kVoteMinVotes) ? bestSpeed : -1;
}

bool shouldCommitSpeedChange(int candidateSpeed, int currentSpeed) {
  if (candidateSpeed < 0 || candidateSpeed == currentSpeed) {
    g_pendingSpeedChange = -1;
    g_pendingSpeedChangeCount = 0;
    return false;
  }

  if (g_pendingSpeedChange == candidateSpeed) {
    g_pendingSpeedChangeCount++;
  } else {
    g_pendingSpeedChange = candidateSpeed;
    g_pendingSpeedChangeCount = 1;
  }

  if (g_pendingSpeedChangeCount >= kConfirmRepeats) {
    g_pendingSpeedChange = -1;
    g_pendingSpeedChangeCount = 0;
    return true;
  }
  return false;
}

void emitLogIfNeeded() {
  if (!g_config.logRx) {
    return;
  }
  const TickType_t now = xTaskGetTickCount();
  if (g_lastLogTick != 0 && (now - g_lastLogTick) < kLogMinInterval) {
    return;
  }

  SpeedMeterSnapshot snapshot{};
  speedMeterGetSnapshot(snapshot);

  String msg;
  msg.reserve(160);
  msg += "[SPD][RX] speed=";
  msg += snapshot.speedKmh;
  msg += " conf=";
  msg += snapshot.confidence;
  msg += " thr=";
  msg += snapshot.throttleOn ? "ON" : "OFF";
  msg += " ok=";
  msg += snapshot.framesOk;
  msg += " short=";
  msg += snapshot.framesShort;
  msg += " long=";
  msg += snapshot.framesLong;
  msg += " overflow=";
  msg += snapshot.frameOverflows;
  msg += " unknown=";
  msg += snapshot.framesUnknown;
  broadcastIf(true, msg);
  g_lastLogTick = now;
}

void handleFrame(const uint8_t* buf, size_t len) {
  if (buf == nullptr || len == 0) {
    return;
  }

  const TickType_t nowTick = xTaskGetTickCount();
  const uint32_t nowMs = millis();
  const uint8_t b8 = (len > 8) ? buf[8] : 0x00;
  const uint8_t b16 = (len > 16) ? buf[16] : 0x00;
  const uint8_t b17 = (len > 17) ? buf[17] : 0x00;

  portENTER_CRITICAL(&g_stateMux);
  g_snapshot.hasFrame = true;
  g_snapshot.lastFrameTick = nowTick;
  g_snapshot.lastFrameLen = static_cast<uint8_t>(len > 255 ? 255 : len);
  g_snapshot.lastB8 = b8;
  g_snapshot.lastB16 = b16;
  g_snapshot.lastB17 = b17;
  if (len < g_config.minFrameLen) {
    g_snapshot.framesShort++;
    portEXIT_CRITICAL(&g_stateMux);
    return;
  }
  if (len > g_config.maxFrameLen) {
    g_snapshot.framesLong++;
    portEXIT_CRITICAL(&g_stateMux);
    return;
  }
  portEXIT_CRITICAL(&g_stateMux);

  const bool zeroSignature = (len > 17 && b16 == 0xDB && b17 == 0xDB);
  const bool frameThrottle = (len > 8 && b8 == 0x5B && !zeroSignature);
  const bool throttle = frameThrottle;
  const bool throttleChanged = (!g_haveThrottle || throttle != g_snapshot.throttleOn);

  if (throttleChanged) {
    g_haveThrottle = true;
    resetSpeedVotes();
    if (throttle) {
      g_throttleOffSinceMs = 0;
      g_lastOffDecayStepMs = 0;
    } else {
      g_throttleOffSinceMs = nowMs;
      g_lastOffDecayStepMs = nowMs;
    }
  }

  int currentSpeed = -1;
  portENTER_CRITICAL(&g_stateMux);
  currentSpeed = g_snapshot.speedKmh;
  portEXIT_CRITICAL(&g_stateMux);

  int candidate = speedFromB16B17(b16, b17);
  int confidence = candidateConfidence(candidate, throttle, static_cast<int>(len), currentSpeed);

  if (candidate < 0) {
    portENTER_CRITICAL(&g_stateMux);
    g_snapshot.framesUnknown++;
    portEXIT_CRITICAL(&g_stateMux);
  }

  if (!throttle && currentSpeed >= 0 && candidate > currentSpeed) {
    candidate = -1;
    confidence = 0;
  }

  if (candidate >= 0) {
    pushSpeedVote(candidate);
    const int speedFiltered = filteredSpeedFromVotes();
    if (speedFiltered >= 0 && shouldCommitSpeedChange(speedFiltered, currentSpeed)) {
      currentSpeed = speedFiltered;
      confidence = clamp100(confidence + 20);
      resetSpeedVotes();
    }
  }

  if (!throttle && currentSpeed > 0) {
    if (g_throttleOffSinceMs != 0 && (nowMs - g_throttleOffSinceMs) >= kOffForceZeroMs) {
      currentSpeed = 0;
      confidence = 100;
      resetSpeedVotes();
    } else if (throttleChanged || g_lastOffDecayStepMs == 0 || (nowMs - g_lastOffDecayStepMs) >= kOffDecayStepMs) {
      g_lastOffDecayStepMs = nowMs;
      if (currentSpeed > 0) {
        currentSpeed -= 1;
      }
      if (currentSpeed < 0) {
        currentSpeed = 0;
      }
      confidence = 80;
    }
  }

  portENTER_CRITICAL(&g_stateMux);
  g_snapshot.throttleOn = throttle;
  g_snapshot.speedKmh = currentSpeed;
  g_snapshot.confidence = confidence;
  g_snapshot.framesOk++;
  portEXIT_CRITICAL(&g_stateMux);
}

void flushFrame() {
  if (g_frameLen == 0) {
    return;
  }
  handleFrame(g_frame, g_frameLen);
  g_frameLen = 0;
  g_lastByteUs = 0;
}

void pushFrameByte(uint8_t value) {
  const uint32_t nowUs = micros();
  if (g_frameLen > 0 && g_lastByteUs != 0 && (nowUs - g_lastByteUs) >= g_config.frameGapUs) {
    flushFrame();
  }

  if (g_frameLen < kFrameBufferSize) {
    g_frame[g_frameLen++] = value;
  } else {
    portENTER_CRITICAL(&g_stateMux);
    g_snapshot.frameOverflows++;
    portEXIT_CRITICAL(&g_stateMux);
    flushFrame();
    g_frame[g_frameLen++] = value;
  }

  g_lastByteUs = nowUs;
}

void consumeUartData() {
  uint8_t chunk[128];
  while (true) {
    const int readCount = uart_read_bytes(g_config.uartNum, chunk, sizeof(chunk), 0);
    if (readCount <= 0) {
      break;
    }
    portENTER_CRITICAL(&g_stateMux);
    g_snapshot.uartBytesRx += static_cast<uint32_t>(readCount);
    portEXIT_CRITICAL(&g_stateMux);
    for (int i = 0; i < readCount; ++i) {
      pushFrameByte(chunk[i]);
    }
  }
}

uint8_t computeRxTimeoutSymbols(const SpeedMeterConfig& cfg) {
  if (cfg.baudRate <= 0 || cfg.frameGapUs == 0) {
    return 0;
  }

  const uint64_t symbolUs = (11ULL * 1000000ULL) / static_cast<uint64_t>(cfg.baudRate);
  if (symbolUs == 0) {
    return 1;
  }

  uint64_t threshold = cfg.frameGapUs / symbolUs;
  if (threshold == 0) {
    threshold = 1;
  }
  if (threshold > 126) {
    threshold = 126;
  }
  return static_cast<uint8_t>(threshold);
}

bool applyUartRuntimeConfig(int baudRate, bool invertRx) {
  if (uart_set_baudrate(g_config.uartNum, static_cast<uint32_t>(baudRate)) != ESP_OK) {
    return false;
  }

  const uart_signal_inv_t invMask = invertRx ? UART_SIGNAL_RXD_INV : UART_SIGNAL_INV_DISABLE;
  if (uart_set_line_inverse(g_config.uartNum, invMask) != ESP_OK) {
    return false;
  }

  g_config.baudRate = baudRate;
  g_config.invertRx = invertRx;
  return true;
}

}  // namespace

bool speedMeterInit(const SpeedMeterConfig& config) {
  if (g_initialized) {
    return true;
  }

  g_config = config;
  if (g_config.baudRate <= 0) {
    g_config.baudRate = 2000;
  }
  if (g_config.rxBufferSize <= 128) {
    g_config.rxBufferSize = 1024;
  }
  if (g_config.eventQueueSize == 0) {
    g_config.eventQueueSize = 16;
  }
  if (g_config.frameGapUs == 0) {
    g_config.frameGapUs = 12000;
  }
  if (g_config.minFrameLen == 0) {
    g_config.minFrameLen = 19;
  }
  if (g_config.maxFrameLen < g_config.minFrameLen) {
    g_config.maxFrameLen = 30;
  }

  uart_driver_delete(g_config.uartNum);
  g_uartQueue = nullptr;

  uart_config_t uartCfg{};
  uartCfg.baud_rate = g_config.baudRate;
  uartCfg.data_bits = UART_DATA_8_BITS;
  uartCfg.parity = UART_PARITY_DISABLE;
  uartCfg.stop_bits = UART_STOP_BITS_1;
  uartCfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uartCfg.source_clk = UART_SCLK_APB;

  if (uart_param_config(g_config.uartNum, &uartCfg) != ESP_OK) {
    return false;
  }
  if (uart_set_pin(g_config.uartNum, UART_PIN_NO_CHANGE, g_config.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) !=
      ESP_OK) {
    return false;
  }
  if (uart_driver_install(
          g_config.uartNum, static_cast<int>(g_config.rxBufferSize), 0, static_cast<int>(g_config.eventQueueSize), &g_uartQueue, 0) !=
      ESP_OK) {
    return false;
  }
  if (!applyUartRuntimeConfig(g_config.baudRate, g_config.invertRx)) {
    return false;
  }

  const uint8_t rxTout = computeRxTimeoutSymbols(g_config);
  if (rxTout > 0) {
    uart_set_rx_timeout(g_config.uartNum, rxTout);
  }

  uart_flush_input(g_config.uartNum);

  resetSpeedVotes();
  g_frameLen = 0;
  g_lastByteUs = 0;
  g_haveThrottle = false;
  g_throttleOffSinceMs = 0;
  g_lastOffDecayStepMs = 0;
  g_lastLogTick = 0;

  portENTER_CRITICAL(&g_stateMux);
  g_snapshot = {true, false, 0, -1, 0, false, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  portEXIT_CRITICAL(&g_stateMux);

  g_initialized = true;
  return true;
}

bool speedMeterGetSnapshot(SpeedMeterSnapshot& snapshot) {
  portENTER_CRITICAL(&g_stateMux);
  snapshot = g_snapshot;
  portEXIT_CRITICAL(&g_stateMux);
  return snapshot.driverReady;
}

void speedMeterGetConfig(SpeedMeterConfig& config) {
  config = g_config;
}

bool speedMeterSetUartConfig(int baudRate, bool invertRx) {
  if (!g_initialized || baudRate <= 0) {
    return false;
  }

  if (!applyUartRuntimeConfig(baudRate, invertRx)) {
    return false;
  }

  uart_flush_input(g_config.uartNum);
  return true;
}

void speedMeterResetStats() {
  portENTER_CRITICAL(&g_stateMux);
  g_snapshot.framesOk = 0;
  g_snapshot.framesShort = 0;
  g_snapshot.framesLong = 0;
  g_snapshot.frameOverflows = 0;
  g_snapshot.framesUnknown = 0;
  g_snapshot.uartBytesRx = 0;
  g_snapshot.uartEventsData = 0;
  g_snapshot.uartEventsFifoOvf = 0;
  g_snapshot.uartEventsBufferFull = 0;
  g_snapshot.uartEventsFrameErr = 0;
  g_snapshot.uartEventsParityErr = 0;
  g_snapshot.uartEventsBreak = 0;
  portEXIT_CRITICAL(&g_stateMux);
}

void taskSpeedMeterRx(void* parameter) {
  SpeedMeterConfig* cfg = static_cast<SpeedMeterConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[SPD][RX] Configuracion invalida, abortando tarea");
    vTaskDelete(nullptr);
    return;
  }

  if (!g_initialized) {
    if (!speedMeterInit(*cfg)) {
      broadcastIf(true, "[SPD][RX] UART2 no inicializada, abortando tarea");
      vTaskDelete(nullptr);
      return;
    }
  }

  uart_event_t event{};
  for (;;) {
    if (xQueueReceive(g_uartQueue, &event, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    switch (event.type) {
      case UART_DATA:
        portENTER_CRITICAL(&g_stateMux);
        g_snapshot.uartEventsData++;
        portEXIT_CRITICAL(&g_stateMux);
        consumeUartData();
        if (event.timeout_flag) {
          flushFrame();
        }
        break;
      case UART_FIFO_OVF:
        portENTER_CRITICAL(&g_stateMux);
        g_snapshot.uartEventsFifoOvf++;
        g_snapshot.frameOverflows++;
        portEXIT_CRITICAL(&g_stateMux);
        uart_flush_input(g_config.uartNum);
        xQueueReset(g_uartQueue);
        flushFrame();
        break;
      case UART_BUFFER_FULL:
        portENTER_CRITICAL(&g_stateMux);
        g_snapshot.uartEventsBufferFull++;
        g_snapshot.frameOverflows++;
        portEXIT_CRITICAL(&g_stateMux);
        uart_flush_input(g_config.uartNum);
        xQueueReset(g_uartQueue);
        flushFrame();
        break;
      case UART_FRAME_ERR:
        portENTER_CRITICAL(&g_stateMux);
        g_snapshot.uartEventsFrameErr++;
        portEXIT_CRITICAL(&g_stateMux);
        break;
      case UART_PARITY_ERR:
        portENTER_CRITICAL(&g_stateMux);
        g_snapshot.uartEventsParityErr++;
        portEXIT_CRITICAL(&g_stateMux);
        break;
      case UART_BREAK:
      case UART_DATA_BREAK:
        portENTER_CRITICAL(&g_stateMux);
        g_snapshot.uartEventsBreak++;
        portEXIT_CRITICAL(&g_stateMux);
        break;
      case UART_PATTERN_DET:
      default:
        break;
    }

    emitLogIfNeeded();
  }
}
