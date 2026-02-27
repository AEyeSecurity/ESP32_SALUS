#include "pi_comms.h"

#include <array>
#include <cstring>
#include <limits>

#include <math.h>

#include "esp_err.h"
#include "esp_timer.h"
#include "esp32-hal-timer.h"

#include "hall_speed.h"
#include "ota_telnet.h"
#include "pid.h"
#include "quad_functions.h"
#include "steering_calibration.h"
#include "system_diag.h"

namespace {

constexpr uint8_t kHeaderRx = 0xAA;
constexpr uint8_t kHeaderTx = 0x55;
constexpr size_t kRxFrameSize = 7;
constexpr size_t kTxFrameSize = 8;
constexpr uint8_t kProtocolVersion = 2;
constexpr uint32_t kPiRxWakeTimeoutFactor = 4;
constexpr uint8_t kPiRxWakeTimerIndex = 0;
constexpr uint16_t kPiRxWakeTimerDivider = 80;  // 1 tick = 1 us (80 MHz / 80)
constexpr size_t kPiRxReadChunkSize = 24;       // Bound RX parse work per 2ms cycle

PiCommsConfig g_config{};
bool g_initialized = false;

struct PiRxState {
  bool hasFrame = false;
  TickType_t lastFrameTick = 0;
  uint8_t verFlags = 0;
  int8_t steer = 0;
  uint16_t speedCmdCentiMps = 0;
  bool driveEnabled = false;
  bool estop = false;
  uint8_t brake = 0;
  uint32_t framesOk = 0;
  uint32_t framesCrcError = 0;
  uint32_t framesMalformed = 0;
  uint32_t framesVersionError = 0;
};

PiRxState g_rxState{};

portMUX_TYPE g_stateMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_piRxWakeMux = portMUX_INITIALIZER_UNLOCKED;
TaskHandle_t g_piRxWakeTaskHandle = nullptr;
hw_timer_t* g_piRxWakeTimer = nullptr;

constexpr uint8_t kCmdFlagEstop = 1 << 0;
constexpr uint8_t kCmdFlagDriveEnable = 1 << 1;
constexpr uint8_t kCmdFlagReserved2 = 1 << 2;

constexpr uint8_t kStatusReady = 1 << 0;
constexpr uint8_t kStatusEstopActive = 1 << 1;
constexpr uint8_t kStatusFailsafeActive = 1 << 2;
constexpr uint8_t kStatusPiFresh = 1 << 3;
constexpr uint8_t kStatusControlSourceShift = 4;
constexpr uint8_t kStatusControlSourceMask = static_cast<uint8_t>(0x03u << kStatusControlSourceShift);
constexpr uint8_t kStatusOverspeedActive = 1 << 6;

constexpr uint16_t kSpeedTelemetryNotAvailable = 0xFFFFu;
constexpr int16_t kSteerTelemetryNotAvailable = std::numeric_limits<int16_t>::min();

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
  appendFlag(flags, (verFlags & kCmdFlagEstop) != 0, "ESTOP");
  appendFlag(flags, (verFlags & kCmdFlagDriveEnable) != 0, "DRIVE_EN");
  appendFlag(flags, (verFlags & kCmdFlagReserved2) != 0, "RESV2");
  appendFlag(flags, (verFlags & 0x08) != 0, "RESV3");
  msg += (flags.length() > 0) ? flags : "none";
  return msg;
}

const char* statusSourceText(uint8_t sourceCode) {
  switch (sourceCode) {
    case 1:
      return "PI";
    case 2:
      return "RC";
    case 3:
      return "TEL";
    case 0:
    default:
      return "NONE";
  }
}

String describeStatusFlags(uint8_t status) {
  String flags;
  flags.reserve(64);
  appendFlag(flags, (status & kStatusReady) != 0, "READY");
  appendFlag(flags, (status & kStatusEstopActive) != 0, "ESTOP");
  appendFlag(flags, (status & kStatusFailsafeActive) != 0, "FAILSAFE");
  appendFlag(flags, (status & kStatusPiFresh) != 0, "PI_FRESH");
  appendFlag(flags, (status & kStatusOverspeedActive) != 0, "OVERSPEED");

  const uint8_t sourceCode = static_cast<uint8_t>((status & kStatusControlSourceMask) >> kStatusControlSourceShift);

  String msg;
  msg.reserve(96);
  msg += "status=0x";
  msg += String(status, HEX);
  msg += " (";
  msg += (flags.length() > 0) ? flags : "none";
  msg += " src=";
  msg += statusSourceText(sourceCode);
  msg += ")";
  return msg;
}

String describeSpeedTelemetry(uint16_t speedCentiMps) {
  if (speedCentiMps == kSpeedTelemetryNotAvailable) {
    return "N/A";
  }
  String msg = String(static_cast<float>(speedCentiMps) / 100.0f, 2);
  msg += "m/s";
  return msg;
}

String describeSteerTelemetry(int16_t steerCentiDeg) {
  if (steerCentiDeg == kSteerTelemetryNotAvailable) {
    return "N/A";
  }
  String msg = String(static_cast<float>(steerCentiDeg) / 100.0f, 2);
  msg += "deg";
  return msg;
}

uint16_t encodeSpeedTelemetryFromHall() {
  HallSpeedSnapshot speed{};
  if (!hallSpeedGetSnapshot(speed) || !speed.driverReady || !isfinite(speed.speedMps)) {
    return kSpeedTelemetryNotAvailable;
  }

  float speedMps = speed.speedMps;
  if (speedMps < 0.0f) {
    speedMps = 0.0f;
  }

  long speedCenti = lroundf(speedMps * 100.0f);
  if (speedCenti < 0) {
    speedCenti = 0;
  } else if (speedCenti > 65534L) {
    speedCenti = 65534L;
  }
  return static_cast<uint16_t>(speedCenti);
}

int16_t encodeSteerTelemetryCentered() {
  PidRuntimeSnapshot pidSnapshot{};
  if (!pidGetRuntimeSnapshot(pidSnapshot) || !pidSnapshot.valid || !pidSnapshot.sensorValid ||
      !isfinite(pidSnapshot.measuredDeg)) {
    return kSteerTelemetryNotAvailable;
  }

  const SteeringCalibrationData calibration = steeringCalibrationSnapshot();
  const float centerDeg = calibration.initialized ? calibration.adjustedCenterDeg : 0.0f;
  float centeredDeg = wrapAngleDegrees(pidSnapshot.measuredDeg - centerDeg);
  if (!isfinite(centeredDeg)) {
    return kSteerTelemetryNotAvailable;
  }

  if (centeredDeg < -180.0f) {
    centeredDeg = -180.0f;
  } else if (centeredDeg > 180.0f) {
    centeredDeg = 180.0f;
  }

  long centeredCenti = lroundf(centeredDeg * 100.0f);
  if (centeredCenti < -18000L) {
    centeredCenti = -18000L;
  } else if (centeredCenti > 18000L) {
    centeredCenti = 18000L;
  }
  return static_cast<int16_t>(centeredCenti);
}

uint8_t encodeDriveSource(QuadDriveControlSource source) {
  switch (source) {
    case QuadDriveControlSource::kPi:
      return 1;
    case QuadDriveControlSource::kRc:
      return 2;
    case QuadDriveControlSource::kTelnet:
      return 3;
    case QuadDriveControlSource::kNone:
    default:
      return 0;
  }
}

uint8_t encodeStatusFlags() {
  uint8_t status = kStatusReady;

  QuadDriveRuntimeSnapshot driveSnapshot{};
  if (!quadDriveGetRuntimeSnapshot(driveSnapshot) || !driveSnapshot.valid) {
    return status;
  }

  if (driveSnapshot.piEstopActive) {
    status |= kStatusEstopActive;
  }
  if (driveSnapshot.speedPidFailsafe) {
    status |= kStatusFailsafeActive;
  }
  if (driveSnapshot.piFresh) {
    status |= kStatusPiFresh;
  }

  const uint8_t source = encodeDriveSource(driveSnapshot.source);
  status |= static_cast<uint8_t>((source << kStatusControlSourceShift) & kStatusControlSourceMask);

  if (driveSnapshot.speedPidOverspeed) {
    status |= kStatusOverspeedActive;
  }

  return status;
}

uint8_t encodeAppliedBrakePercent() {
  QuadDriveRuntimeSnapshot driveSnapshot{};
  if (!quadDriveGetRuntimeSnapshot(driveSnapshot) || !driveSnapshot.valid) {
    return 0;
  }
  if (driveSnapshot.appliedBrakePercent > 100u) {
    return 100u;
  }
  return driveSnapshot.appliedBrakePercent;
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
  msg.reserve(176);
  msg += "[PI][RX] ";
  msg += describeCommandFlags(state.verFlags);
  msg += " steer=";
  msg += state.steer;
  msg += "%";
  msg += " speedCmd=";
  msg += String(static_cast<float>(state.speedCmdCentiMps) / 100.0f, 2);
  msg += "m/s";
  msg += " brake=";
  msg += state.brake;
  msg += "%";
  msg += " estop=";
  msg += state.estop ? "Y" : "N";
  msg += " drive=";
  msg += state.driveEnabled ? "Y" : "N";
  msg += " stats{ok=";
  msg += state.framesOk;
  msg += " crcErr=";
  msg += state.framesCrcError;
  msg += " malformed=";
  msg += state.framesMalformed;
  msg += " verErr=";
  msg += state.framesVersionError;
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
  msg.reserve(144);
  msg += "[PI][RX][WARN] ";
  msg += reason;
  msg += " stats{ok=";
  msg += snapshot.framesOk;
  msg += " crcErr=";
  msg += snapshot.framesCrcError;
  msg += " malformed=";
  msg += snapshot.framesMalformed;
  msg += " verErr=";
  msg += snapshot.framesVersionError;
  msg += "}";
  broadcastIf(true, msg);
  s_lastErrorLogTick = nowTick;
}

void logTxFrame(const PiCommsConfig& cfg,
                uint8_t status,
                uint16_t speedCentiMps,
                int16_t steerCentiDeg,
                uint8_t brakePercent,
                TickType_t nowTick) {
  if (!cfg.logTx) {
    return;
  }
  static TickType_t s_lastLogTick = 0;
  const TickType_t minInterval = pdMS_TO_TICKS(1000);
  if (s_lastLogTick != 0 && (nowTick - s_lastLogTick) < minInterval) {
    return;
  }
  String msg;
  msg.reserve(200);
  msg += "[PI][TX] ";
  msg += describeStatusFlags(status);
  msg += " speed=";
  msg += describeSpeedTelemetry(speedCentiMps);
  msg += " steer=";
  msg += describeSteerTelemetry(steerCentiDeg);
  msg += " brake=";
  msg += brakePercent;
  msg += "%";
  broadcastIf(true, msg);
  s_lastLogTick = nowTick;
}

void IRAM_ATTR onPiRxWakeTimer() {
  BaseType_t taskWoken = pdFALSE;
  TaskHandle_t task = nullptr;
  portENTER_CRITICAL_ISR(&g_piRxWakeMux);
  task = g_piRxWakeTaskHandle;
  portEXIT_CRITICAL_ISR(&g_piRxWakeMux);
  if (task != nullptr) {
    vTaskNotifyGiveFromISR(task, &taskWoken);
    if (taskWoken == pdTRUE) {
      portYIELD_FROM_ISR();
    }
  }
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
  snapshot.speedCmdCentiMps = g_rxState.speedCmdCentiMps;
  snapshot.brake = g_rxState.brake;
  snapshot.driveEnabled = g_rxState.driveEnabled;
  snapshot.estop = g_rxState.estop;
  snapshot.framesOk = g_rxState.framesOk;
  snapshot.framesCrcError = g_rxState.framesCrcError;
  snapshot.framesMalformed = g_rxState.framesMalformed;
  snapshot.framesVersionError = g_rxState.framesVersionError;
  portEXIT_CRITICAL(&g_stateMux);
  return snapshot.driverReady;
}

void piCommsResetStats() {
  portENTER_CRITICAL(&g_stateMux);
  g_rxState.framesOk = 0;
  g_rxState.framesCrcError = 0;
  g_rxState.framesMalformed = 0;
  g_rxState.framesVersionError = 0;
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
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  const uint32_t wakeTimeoutMs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * kPiRxWakeTimeoutFactor);
  TickType_t wakeTimeoutTicks = pdMS_TO_TICKS(wakeTimeoutMs);
  if (wakeTimeoutTicks == 0) {
    wakeTimeoutTicks = 1;
  }
  FrameParser parser;
  const TaskHandle_t selfHandle = xTaskGetCurrentTaskHandle();
  int64_t lastIterationStartUs = esp_timer_get_time();

  bool usePeriodicWakeTimer = false;
  portENTER_CRITICAL(&g_piRxWakeMux);
  if (g_piRxWakeTimer == nullptr) {
    g_piRxWakeTimer = timerBegin(kPiRxWakeTimerIndex, kPiRxWakeTimerDivider, true);
    if (g_piRxWakeTimer != nullptr) {
      timerAttachInterrupt(g_piRxWakeTimer, &onPiRxWakeTimer, true);
    }
  }
  if (g_piRxWakeTimer != nullptr) {
    g_piRxWakeTaskHandle = selfHandle;
    timerAlarmDisable(g_piRxWakeTimer);
    timerAlarmWrite(g_piRxWakeTimer, expectedPeriodUs, true);
    timerAlarmEnable(g_piRxWakeTimer);
    usePeriodicWakeTimer = true;
  }
  portEXIT_CRITICAL(&g_piRxWakeMux);

  std::array<uint8_t, kPiRxReadChunkSize> chunk{};

  while (true) {
    bool notifyTimeout = false;
    if (usePeriodicWakeTimer) {
      const uint32_t notifyCount = ulTaskNotifyTake(pdTRUE, wakeTimeoutTicks);
      notifyTimeout = (notifyCount == 0);
    }

    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;
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

        const uint8_t version = static_cast<uint8_t>((frame[1] >> 4) & 0x0F);
        if (version != kProtocolVersion) {
          portENTER_CRITICAL(&g_stateMux);
          g_rxState.framesVersionError++;
          portEXIT_CRITICAL(&g_stateMux);
          logRxError(*cfg, "Version de protocolo no soportada", nowTick);
          continue;
        }

        PiRxState newState{};
        newState.hasFrame = true;
        newState.lastFrameTick = nowTick;
        newState.verFlags = frame[1];
        newState.steer = static_cast<int8_t>(frame[2]);
        newState.speedCmdCentiMps =
            static_cast<uint16_t>(frame[3]) | static_cast<uint16_t>(frame[4]) << 8;
        newState.brake = frame[5];
        newState.estop = (newState.verFlags & kCmdFlagEstop) != 0;
        newState.driveEnabled = (newState.verFlags & kCmdFlagDriveEnable) != 0;

        portENTER_CRITICAL(&g_stateMux);
        g_rxState.hasFrame = newState.hasFrame;
        g_rxState.lastFrameTick = newState.lastFrameTick;
        g_rxState.verFlags = newState.verFlags;
        g_rxState.steer = newState.steer;
        g_rxState.speedCmdCentiMps = newState.speedCmdCentiMps;
        g_rxState.estop = newState.estop;
        g_rxState.driveEnabled = newState.driveEnabled;
        g_rxState.brake = newState.brake;
        g_rxState.framesOk++;
        newState.framesOk = g_rxState.framesOk;
        newState.framesCrcError = g_rxState.framesCrcError;
        newState.framesMalformed = g_rxState.framesMalformed;
        newState.framesVersionError = g_rxState.framesVersionError;
        portEXIT_CRITICAL(&g_stateMux);

        logRxFrame(*cfg, newState, nowTick);
      }
    }

    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kPiUartRx, cycleUs, expectedPeriodUs, overrun, notifyTimeout);
    if (!usePeriodicWakeTimer) {
      vTaskDelay(period);
    }
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
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  TickType_t lastWake = xTaskGetTickCount();
  int64_t lastIterationStartUs = esp_timer_get_time();

  while (true) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;
    if (g_initialized) {
      const uint8_t status = encodeStatusFlags();
      const uint16_t speedTelemetry = encodeSpeedTelemetryFromHall();
      const int16_t steerTelemetry = encodeSteerTelemetryCentered();
      const uint8_t brakePercent = encodeAppliedBrakePercent();

      uint8_t frame[kTxFrameSize];
      frame[0] = kHeaderTx;
      frame[1] = status;
      frame[2] = static_cast<uint8_t>(speedTelemetry & 0xFFu);
      frame[3] = static_cast<uint8_t>((speedTelemetry >> 8) & 0xFFu);
      const uint16_t steerBits = static_cast<uint16_t>(steerTelemetry);
      frame[4] = static_cast<uint8_t>(steerBits & 0xFFu);
      frame[5] = static_cast<uint8_t>((steerBits >> 8) & 0xFFu);
      frame[6] = brakePercent;
      frame[7] = crc8_maxim(frame, kTxFrameSize - 1);

      uart_write_bytes(cfg->uartNum, reinterpret_cast<const char*>(frame), sizeof(frame));
      logTxFrame(*cfg, status, speedTelemetry, steerTelemetry, brakePercent, xTaskGetTickCount());
    }

    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kPiUartTx, cycleUs, expectedPeriodUs, overrun, false);
    vTaskDelayUntil(&lastWake, period);
  }
}
