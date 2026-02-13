#include "pi_comms.h"

#include <array>
#include <cstring>

#include "esp_err.h"

#include "ota_telnet.h"
#include "speed_meter.h"

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
  int8_t accelRaw = 0;
  int8_t accelEffective = 0;
  bool driveEnabled = false;
  bool estop = false;
  bool allowReverse = false;
  bool wantsReverse = false;
  bool reverseRequestActive = false;
  bool reverseAwaitingGrant = false;
  bool reverseGranted = false;
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
uint8_t g_manualStatusFlags = 0;
bool g_reverseRequestActive = false;

portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;

constexpr uint8_t kCmdFlagEstop = 1 << 0;
constexpr uint8_t kCmdFlagDriveEnable = 1 << 1;
constexpr uint8_t kCmdFlagAllowReverse = 1 << 2;

constexpr uint8_t kStatusReady = 1 << 0;
constexpr uint8_t kStatusFault = 1 << 1;
constexpr uint8_t kStatusOvercurrent = 1 << 2;
constexpr uint8_t kStatusReverseReq = 1 << 3;
constexpr uint8_t kTelemetryAuto = 255;
constexpr TickType_t kSpeedTelemetryMaxAge = pdMS_TO_TICKS(500);

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

void appendFlag(String& out, bool enabled, const char* label) {
  if (!enabled) {
    return;
  }
  if (out.length() > 0) {
    out += '|';
  }
  out += label;
}

String describeCommandFlags(uint8_t verFlags) {
  String msg;
  msg.reserve(48);
  msg += "ver=";
  msg += (verFlags >> 4) & 0x0F;
  msg += " flags=";
  String flags;
  appendFlag(flags, (verFlags & 0x01) != 0, "ESTOP");
  appendFlag(flags, (verFlags & 0x02) != 0, "DRIVE_EN");
  appendFlag(flags, (verFlags & 0x04) != 0, "ALLOW_REV");
  appendFlag(flags, (verFlags & 0x08) != 0, "RESV");
  msg += (flags.length() > 0) ? flags : "none";
  return msg;
}

String describeStatusFlags(uint8_t status) {
  String flags;
  flags.reserve(48);
  appendFlag(flags, (status & 0x01) != 0, "READY");
  appendFlag(flags, (status & 0x02) != 0, "FAULT");
  appendFlag(flags, (status & 0x04) != 0, "OVERCURRENT");
  appendFlag(flags, (status & 0x08) != 0, "REV_REQ");
  String msg;
  msg.reserve(64);
  msg += "status=0x";
  msg += String(status, HEX);
  msg += " (";
  msg += (flags.length() > 0) ? flags : "none";
  msg += ")";
  return msg;
}

String describeTelemetry(uint8_t telemetry) {
  if (telemetry == kTelemetryAuto) {
    return "N/A";
  }
  if (telemetry <= 254) {
    String kmh = String(telemetry);
    kmh += "km/h";
    return kmh;
  }
  String hex = "0x";
  hex += String(telemetry, HEX);
  return hex;
}

uint8_t encodeTelemetryFromSpeed() {
  SpeedMeterSnapshot speed{};
  speedMeterGetSnapshot(speed);
  if (!speed.driverReady || !speed.hasFrame || speed.speedKmh < 0 || speed.lastFrameTick == 0) {
    return kTelemetryAuto;
  }

  const TickType_t nowTick = xTaskGetTickCount();
  const TickType_t ageTicks = nowTick - speed.lastFrameTick;
  if (ageTicks > kSpeedTelemetryMaxAge) {
    return kTelemetryAuto;
  }

  if (speed.speedKmh >= 254) {
    return 254;
  }

  return static_cast<uint8_t>(speed.speedKmh);
}

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
  msg.reserve(160);
  msg += "[PI][RX] ";
  msg += describeCommandFlags(state.verFlags);
  msg += " steer=";
  msg += state.steer;
  msg += "%";
  msg += " accelRaw=";
  msg += state.accelRaw;
  msg += "%";
  msg += " accelEff=";
  msg += state.accelEffective;
  msg += "%";
  msg += " brake=";
  msg += state.brake;
  msg += "%";
  msg += " estop=";
  msg += state.estop ? "Y" : "N";
  msg += " drive=";
  msg += state.driveEnabled ? "Y" : "N";
  msg += " reverse{allow=";
  msg += state.allowReverse ? "Y" : "N";
  msg += " wants=";
  msg += state.wantsReverse ? "Y" : "N";
  msg += " req=";
  msg += state.reverseRequestActive ? "Y" : "N";
  msg += " wait=";
  msg += state.reverseAwaitingGrant ? "Y" : "N";
  msg += " granted=";
  msg += state.reverseGranted ? "Y" : "N";
  msg += "}";
  msg += " stats{ok=";
  msg += state.framesOk;
  msg += " crcErr=";
  msg += state.framesCrcError;
  msg += " malformed=";
  msg += state.framesMalformed;
  msg += "}";
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
  PiRxState snapshot{};
  portENTER_CRITICAL(&g_stateMux);
  snapshot = g_rxState;
  portEXIT_CRITICAL(&g_stateMux);
  String msg;
  msg.reserve(128);
  msg += "[PI][RX][WARN] ";
  msg += reason;
  msg += " stats{ok=";
  msg += snapshot.framesOk;
  msg += " crcErr=";
  msg += snapshot.framesCrcError;
  msg += " malformed=";
  msg += snapshot.framesMalformed;
  msg += "}";
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
  msg.reserve(112);
  msg += "[PI][TX] ";
  msg += describeStatusFlags(status);
  msg += " telemetry=";
  msg += describeTelemetry(telemetry);
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
  g_txState.statusFlags = kStatusReady;
  g_txState.telemetry = kTelemetryAuto;
  g_manualStatusFlags = 0;
  g_reverseRequestActive = false;
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
  snapshot.accelRaw = g_rxState.accelRaw;
  snapshot.accelEffective = g_rxState.accelEffective;
  snapshot.brake = g_rxState.brake;
  snapshot.driveEnabled = g_rxState.driveEnabled;
  snapshot.estop = g_rxState.estop;
  snapshot.allowReverse = g_rxState.allowReverse;
  snapshot.wantsReverse = g_rxState.wantsReverse;
  snapshot.reverseRequestActive = g_rxState.reverseRequestActive;
  snapshot.reverseAwaitingGrant = g_rxState.reverseAwaitingGrant;
  snapshot.reverseGranted = g_rxState.reverseGranted;
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
  const uint8_t sanitized = (flags | kStatusReady) & ~kStatusReverseReq;
  g_txState.statusFlags = sanitized;
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
        newState.accelRaw = static_cast<int8_t>(frame[3]);
        newState.brake = frame[4];
        newState.estop = (newState.verFlags & kCmdFlagEstop) != 0;
        newState.driveEnabled = (newState.verFlags & kCmdFlagDriveEnable) != 0;
        newState.allowReverse = (newState.verFlags & kCmdFlagAllowReverse) != 0;
        newState.wantsReverse = (newState.accelRaw < 0);
        newState.reverseRequestActive = newState.wantsReverse && !newState.allowReverse;
        newState.reverseAwaitingGrant = newState.reverseRequestActive;
        const bool reverseGranted = newState.wantsReverse && newState.allowReverse;
        newState.reverseGranted = reverseGranted;
        newState.accelEffective =
            reverseGranted ? newState.accelRaw : ((newState.accelRaw > 0) ? newState.accelRaw : 0);

        portENTER_CRITICAL(&g_stateMux);
        g_rxState.hasFrame = newState.hasFrame;
        g_rxState.lastFrameTick = newState.lastFrameTick;
        g_rxState.verFlags = newState.verFlags;
        g_rxState.steer = newState.steer;
        g_rxState.accelRaw = newState.accelRaw;
        g_rxState.accelEffective = newState.accelEffective;
        g_rxState.estop = newState.estop;
        g_rxState.driveEnabled = newState.driveEnabled;
        g_rxState.allowReverse = newState.allowReverse;
        g_rxState.wantsReverse = newState.wantsReverse;
        g_rxState.reverseRequestActive = newState.reverseRequestActive;
        g_rxState.reverseAwaitingGrant = newState.reverseAwaitingGrant;
        g_rxState.reverseGranted = newState.reverseGranted;
        g_rxState.brake = newState.brake;
        g_reverseRequestActive = newState.reverseRequestActive;
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
      uint8_t telemetryOverride = kTelemetryAuto;
      bool reverseRequest = false;
      portENTER_CRITICAL(&g_stateMux);
      status = g_txState.statusFlags;
      telemetryOverride = g_txState.telemetry;
      reverseRequest = g_reverseRequestActive;
      portEXIT_CRITICAL(&g_stateMux);
      if (reverseRequest) {
        status |= kStatusReverseReq;
      } else {
        status &= ~kStatusReverseReq;
      }
      const uint8_t telemetry =
          (telemetryOverride == kTelemetryAuto) ? encodeTelemetryFromSpeed() : telemetryOverride;

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
