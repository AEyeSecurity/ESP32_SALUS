#include "pi_comms.h"

#include <array>
#include <cstring>

#include "esp_err.h"

#include "ota_telnet.h"

namespace {

constexpr uint8_t kHeaderRx = 0xAA;
constexpr uint8_t kHeaderTx = 0x55;
constexpr size_t kRxFrameSize = 6;
constexpr size_t kTxFrameSize = 4;

PiCommsConfig g_config{};
bool g_initialized = false;

struct PiRxState {
  bool hasFrame = false;
  TickType_t lastFrameTick = 0;
  uint8_t verFlags = 0;
  int8_t steer = 0;
  int8_t accel = 0;
  uint8_t brake = 0;
  uint32_t framesOk = 0;
  uint32_t framesCrcError = 0;
  uint32_t framesMalformed = 0;
};

struct PiTxState {
  uint8_t statusFlags = 0x01;  // READY asserted por defecto
  uint8_t telemetry = 255;     // Valor “no disponible”
};

PiRxState g_rxState{};
PiTxState g_txState{};

portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

uint8_t crc8_maxim(const uint8_t* data, size_t len) {
  uint8_t c = 0x00;
  for (size_t i = 0; i < len; ++i) {
    c ^= data[i];
    for (int b = 0; b < 8; ++b) {
      if (c & 0x80) {
        c = static_cast<uint8_t>((c << 1) ^ 0x31);
      } else {
        c <<= 1;
      }
    }
  }
  return c;
}

struct FrameParser {
  size_t index = 0;
  uint8_t buffer[kRxFrameSize]{};

  void reset() { index = 0; }

  bool feed(uint8_t byte, uint8_t (&frame)[kRxFrameSize], bool& malformed) {
    malformed = false;
    if (index == 0) {
      if (byte == kHeaderRx) {
        buffer[0] = byte;
        index = 1;
      }
      return false;
    }

    if (byte == kHeaderRx && index > 0) {
      // Nuevo header inesperado -> descartar trama parcial
      malformed = true;
      buffer[0] = byte;
      index = 1;
      return false;
    }

    buffer[index++] = byte;
    if (index >= kRxFrameSize) {
      std::memcpy(frame, buffer, kRxFrameSize);
      index = 0;
      return true;
    }
    return false;
  }
};

void logRxFrame(const PiCommsConfig& cfg, const PiRxState& state, TickType_t nowTick) {
  if (!cfg.logRx) {
    return;
  }
  static TickType_t s_lastLogTick = 0;
  const TickType_t minInterval = pdMS_TO_TICKS(200);
  if (s_lastLogTick != 0 && (nowTick - s_lastLogTick) < minInterval) {
    return;
  }
  String msg;
  msg.reserve(96);
  msg += "[PI][RX] verFlags=0x";
  msg += String(state.verFlags, HEX);
  msg += " steer=";
  msg += state.steer;
  msg += " accel=";
  msg += state.accel;
  msg += " brake=";
  msg += state.brake;
  msg += " ok=";
  msg += state.framesOk;
  msg += " crcErr=";
  msg += state.framesCrcError;
  msg += " malformed=";
  msg += state.framesMalformed;
  broadcastIf(true, msg);
  s_lastLogTick = nowTick;
}

void logRxError(const PiCommsConfig& cfg, const char* reason, TickType_t nowTick) {
  if (!cfg.logRx) {
    return;
  }
  static TickType_t s_lastErrorLogTick = 0;
  const TickType_t minInterval = pdMS_TO_TICKS(500);
  if (s_lastErrorLogTick != 0 && (nowTick - s_lastErrorLogTick) < minInterval) {
    return;
  }
  String msg;
  msg.reserve(64);
  msg += "[PI][RX][WARN] ";
  msg += reason;
  broadcastIf(true, msg);
  s_lastErrorLogTick = nowTick;
}

void logTxFrame(const PiCommsConfig& cfg, uint8_t status, uint8_t telemetry, TickType_t nowTick) {
  if (!cfg.logTx) {
    return;
  }
  static TickType_t s_lastLogTick = 0;
  const TickType_t minInterval = pdMS_TO_TICKS(1000);
  if (s_lastLogTick != 0 && (nowTick - s_lastLogTick) < minInterval) {
    return;
  }
  String msg;
  msg.reserve(64);
  msg += "[PI][TX] status=0x";
  msg += String(status, HEX);
  msg += " telemetry=";
  msg += telemetry;
  broadcastIf(true, msg);
  s_lastLogTick = nowTick;
}

}  // namespace

bool piCommsInit(const PiCommsConfig& config) {
  if (g_initialized) {
    return true;
  }

  g_config = config;

  uart_config_t uartCfg{};
  uartCfg.baud_rate = config.baudRate;
  uartCfg.data_bits = UART_DATA_8_BITS;
  uartCfg.parity = UART_PARITY_DISABLE;
  uartCfg.stop_bits = UART_STOP_BITS_1;
  uartCfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
  uartCfg.source_clk = UART_SCLK_APB;

  const uart_port_t port = config.uartNum;

  if (uart_param_config(port, &uartCfg) != ESP_OK) {
    return false;
  }
  if (uart_set_pin(port, config.txPin, config.rxPin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE) != ESP_OK) {
    return false;
  }
  if (uart_driver_install(port, config.rxBufferSize, config.txBufferSize, 0, nullptr, 0) != ESP_OK) {
    return false;
  }

  uart_flush_input(port);

  portENTER_CRITICAL(&g_stateMux);
  g_rxState = PiRxState{};
  g_txState = PiTxState{};
  g_txState.statusFlags = 0x01;
  g_txState.telemetry = 255;
  portEXIT_CRITICAL(&g_stateMux);

  g_initialized = true;
  return true;
}

bool piCommsGetRxSnapshot(PiCommsRxSnapshot& snapshot) {
  portENTER_CRITICAL(&g_stateMux);
  snapshot.driverReady = g_initialized;
  snapshot.hasFrame = g_rxState.hasFrame;
  snapshot.lastFrameTick = g_rxState.lastFrameTick;
  snapshot.verFlags = g_rxState.verFlags;
  snapshot.steer = g_rxState.steer;
  snapshot.accel = g_rxState.accel;
  snapshot.brake = g_rxState.brake;
  snapshot.framesOk = g_rxState.framesOk;
  snapshot.framesCrcError = g_rxState.framesCrcError;
  snapshot.framesMalformed = g_rxState.framesMalformed;
  portEXIT_CRITICAL(&g_stateMux);
  return snapshot.driverReady;
}

void piCommsResetStats() {
  portENTER_CRITICAL(&g_stateMux);
  g_rxState.framesOk = 0;
  g_rxState.framesCrcError = 0;
  g_rxState.framesMalformed = 0;
  portEXIT_CRITICAL(&g_stateMux);
}

void piCommsSetStatusFlags(uint8_t flags) {
  portENTER_CRITICAL(&g_stateMux);
  g_txState.statusFlags = flags | 0x01;  // READY siempre activo
  portEXIT_CRITICAL(&g_stateMux);
}

void piCommsSetTelemetry(uint8_t telemetry) {
  portENTER_CRITICAL(&g_stateMux);
  g_txState.telemetry = telemetry;
  portEXIT_CRITICAL(&g_stateMux);
}

void taskPiCommsRx(void* parameter) {
  PiCommsConfig* cfg = static_cast<PiCommsConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[PI][RX] Configuracion invalida, abortando tarea");
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->rxTaskPeriod > 0) ? cfg->rxTaskPeriod : pdMS_TO_TICKS(1);
  FrameParser parser;
  TickType_t lastWake = xTaskGetTickCount();

  std::array<uint8_t, 64> chunk{};

  while (true) {
    const int len = g_initialized
                        ? uart_read_bytes(cfg->uartNum, chunk.data(), chunk.size(), cfg->rxReadTimeout)
                        : 0;

    if (len > 0) {
      for (int i = 0; i < len; ++i) {
        bool malformed = false;
        uint8_t frame[kRxFrameSize]{};
        const bool complete = parser.feed(chunk[i], frame, malformed);

        if (malformed) {
          portENTER_CRITICAL(&g_stateMux);
          g_rxState.framesMalformed++;
          portEXIT_CRITICAL(&g_stateMux);
          logRxError(*cfg, "Trama mal formada (re-sync)", xTaskGetTickCount());
          continue;
        }

        if (!complete) {
          continue;
        }

        const uint8_t crc = crc8_maxim(frame, kRxFrameSize - 1);
        const uint8_t receivedCrc = frame[kRxFrameSize - 1];
        const TickType_t nowTick = xTaskGetTickCount();

        if (crc != receivedCrc) {
          portENTER_CRITICAL(&g_stateMux);
          g_rxState.framesCrcError++;
          portEXIT_CRITICAL(&g_stateMux);
          logRxError(*cfg, "CRC invalido", nowTick);
          continue;
        }

        PiRxState newState{};
        newState.hasFrame = true;
        newState.lastFrameTick = nowTick;
        newState.verFlags = frame[1];
        newState.steer = static_cast<int8_t>(frame[2]);
        newState.accel = static_cast<int8_t>(frame[3]);
        newState.brake = frame[4];

        portENTER_CRITICAL(&g_stateMux);
        g_rxState.hasFrame = newState.hasFrame;
        g_rxState.lastFrameTick = newState.lastFrameTick;
        g_rxState.verFlags = newState.verFlags;
        g_rxState.steer = newState.steer;
        g_rxState.accel = newState.accel;
        g_rxState.brake = newState.brake;
        g_rxState.framesOk++;
        newState.framesOk = g_rxState.framesOk;
        newState.framesCrcError = g_rxState.framesCrcError;
        newState.framesMalformed = g_rxState.framesMalformed;
        portEXIT_CRITICAL(&g_stateMux);

        logRxFrame(*cfg, newState, nowTick);
      }
    }

    vTaskDelayUntil(&lastWake, period);
  }
}

void taskPiCommsTx(void* parameter) {
  PiCommsConfig* cfg = static_cast<PiCommsConfig*>(parameter);
  if (cfg == nullptr) {
    broadcastIf(true, "[PI][TX] Configuracion invalida, abortando tarea");
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->txTaskPeriod > 0) ? cfg->txTaskPeriod : pdMS_TO_TICKS(10);
  TickType_t lastWake = xTaskGetTickCount();

  while (true) {
    if (g_initialized) {
      uint8_t status = 0;
      uint8_t telemetry = 255;
      portENTER_CRITICAL(&g_stateMux);
      status = g_txState.statusFlags;
      telemetry = g_txState.telemetry;
      portEXIT_CRITICAL(&g_stateMux);

      uint8_t frame[kTxFrameSize];
      frame[0] = kHeaderTx;
      frame[1] = status;
      frame[2] = telemetry;
      frame[3] = crc8_maxim(frame, kTxFrameSize - 1);

      uart_write_bytes(cfg->uartNum, reinterpret_cast<const char*>(frame), sizeof(frame));
      logTxFrame(*cfg, status, telemetry, xTaskGetTickCount());
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
