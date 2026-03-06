#include "ota_telnet.h"

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <esp_timer.h>

#include <ctype.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#include <freertos/queue.h>

#include "steering_calibration.h"
#include "pi_comms.h"
#include "pid.h"
#include "hall_speed.h"
#include "speed_pid.h"
#include "quad_functions.h"
#include "system_diag.h"

#ifndef WIFI_STA_SSID
#define WIFI_STA_SSID "TU_SSID"
#endif

#ifndef WIFI_STA_PASS
#define WIFI_STA_PASS "TU_PASSWORD"
#endif

#ifndef WIFI_AP_SSID
#define WIFI_AP_SSID "esp32-salus"
#endif

#ifndef WIFI_AP_PASS
#define WIFI_AP_PASS "teamcit2024"
#endif

#ifndef OTA_HOSTNAME
#define OTA_HOSTNAME "esp32-salus"
#endif

#ifndef OTA_PASSWORD
#define OTA_PASSWORD "ota1234"
#endif

#ifndef WIFI_STA_CONNECT_TIMEOUT_MS
#define WIFI_STA_CONNECT_TIMEOUT_MS 10000
#endif

namespace {
constexpr uint16_t kTelnetPort = 23;
// STREAM config por dominio (periodos en ms).
constexpr TickType_t kSpeedStreamDefaultPeriod = pdMS_TO_TICKS(200);
constexpr int kSpeedStreamMinPeriodMs = 20;
constexpr int kSpeedStreamMaxPeriodMs = 5000;
constexpr TickType_t kPidStreamDefaultPeriod = pdMS_TO_TICKS(200);
constexpr int kPidStreamMinPeriodMs = 50;
constexpr int kPidStreamMaxPeriodMs = 5000;
constexpr TickType_t kSpeedPidStreamDefaultPeriod = pdMS_TO_TICKS(200);
constexpr int kSpeedPidStreamMinPeriodMs = 50;
constexpr int kSpeedPidStreamMaxPeriodMs = 5000;
constexpr TickType_t kDrivePidTraceDefaultPeriod = pdMS_TO_TICKS(100);
constexpr int kDrivePidTraceMinPeriodMs = 50;
constexpr int kDrivePidTraceMaxPeriodMs = 1000;
constexpr TickType_t kDriveRcStreamDefaultPeriod = pdMS_TO_TICKS(100);
constexpr int kDriveRcStreamMinPeriodMs = 50;
constexpr int kDriveRcStreamMaxPeriodMs = 1000;
constexpr TickType_t kSystemJitterDefaultPeriod = pdMS_TO_TICKS(500);
constexpr int kSystemJitterMinPeriodMs = 50;
constexpr int kSystemJitterMaxPeriodMs = 5000;
constexpr TickType_t kTelnetIdleTimeout = pdMS_TO_TICKS(300000);
constexpr bool kFocusedControlLogsOnly = true;
constexpr size_t kTelnetLogQueueLength = 256;
constexpr size_t kTelnetLogMessageMaxLen = 256;
constexpr size_t kTelnetDrainMaxMessagesPerCycle = 32;
constexpr uint32_t kTelnetDrainTimeBudgetUs = 5000;

enum class NetworkMode {
  kUnknown,
  kSta,
  kAp,
};

struct TelnetLogMessage {
  char text[kTelnetLogMessageMaxLen];
};

WiFiServer g_telnetServer(kTelnetPort);
WiFiClient g_telnetClient;
QueueHandle_t g_telnetLogQueue = nullptr;
TaskHandle_t g_otaTaskHandleRuntime = nullptr;
portMUX_TYPE g_telnetStatsMux = portMUX_INITIALIZER_UNLOCKED;
uint32_t g_telnetLogDropCount = 0;
String g_telnetCommandBuffer;
NetworkMode g_networkMode = NetworkMode::kUnknown;
String g_networkSsid;
IPAddress g_networkIp;
// Estado runtime de streams Telnet.
bool g_speedStreamEnabled = false;
TickType_t g_speedStreamPeriod = kSpeedStreamDefaultPeriod;
TickType_t g_lastSpeedStreamTick = 0;
bool g_pidStreamEnabled = false;
TickType_t g_pidStreamPeriod = kPidStreamDefaultPeriod;
TickType_t g_lastPidStreamTick = 0;
bool g_speedPidStreamEnabled = false;
TickType_t g_speedPidStreamPeriod = kSpeedPidStreamDefaultPeriod;
TickType_t g_lastSpeedPidStreamTick = 0;
bool g_driveRcStreamEnabled = false;
TickType_t g_driveRcStreamPeriod = kDriveRcStreamDefaultPeriod;
TickType_t g_lastDriveRcStreamTick = 0;
TickType_t g_telnetLastActivityTick = 0;

void reportNetworkStatus();

const char* networkModeText(NetworkMode mode) {
  switch (mode) {
    case NetworkMode::kSta:
      return "STA";
    case NetworkMode::kAp:
      return "AP";
    case NetworkMode::kUnknown:
    default:
      return "UNKNOWN";
  }
}

void updateNetworkState(NetworkMode mode, const String& ssid, const IPAddress& ip) {
  g_networkMode = mode;
  g_networkSsid = ssid;
  g_networkIp = ip;
}

bool isCurrentTaskOtaTelnet() {
  return g_otaTaskHandleRuntime != nullptr && xTaskGetCurrentTaskHandle() == g_otaTaskHandleRuntime;
}

uint32_t telnetLogDropCount() {
  uint32_t out = 0;
  portENTER_CRITICAL(&g_telnetStatsMux);
  out = g_telnetLogDropCount;
  portEXIT_CRITICAL(&g_telnetStatsMux);
  return out;
}

void incrementTelnetLogDropCount() {
  portENTER_CRITICAL(&g_telnetStatsMux);
  g_telnetLogDropCount++;
  portEXIT_CRITICAL(&g_telnetStatsMux);
}

size_t telnetQueueDepth() {
  if (g_telnetLogQueue == nullptr) {
    return 0;
  }
  return static_cast<size_t>(uxQueueMessagesWaiting(g_telnetLogQueue));
}

void clearTelnetLogQueue() {
  if (g_telnetLogQueue == nullptr) {
    return;
  }
  TelnetLogMessage dropped{};
  while (xQueueReceive(g_telnetLogQueue, &dropped, 0) == pdTRUE) {
  }
}

bool sendTelnetDirect(const String& message) {
  if (!(g_telnetClient && g_telnetClient.connected())) {
    return false;
  }
  g_telnetClient.println(message);
  return true;
}

bool enqueueTelnetLog(const String& message) {
  if (g_telnetLogQueue == nullptr) {
    return sendTelnetDirect(message);
  }
  TelnetLogMessage msg{};
  message.toCharArray(msg.text, sizeof(msg.text));
  if (xQueueSend(g_telnetLogQueue, &msg, 0) != pdTRUE) {
    incrementTelnetLogDropCount();
    return false;
  }
  return true;
}

void drainTelnetLogQueue(size_t maxMessages) {
  if (g_telnetLogQueue == nullptr) {
    return;
  }
  if (!(g_telnetClient && g_telnetClient.connected())) {
    clearTelnetLogQueue();
    return;
  }
  TelnetLogMessage msg{};
  size_t drained = 0;
  const int64_t startUs = esp_timer_get_time();
  while (drained < maxMessages) {
    if ((esp_timer_get_time() - startUs) >= kTelnetDrainTimeBudgetUs) {
      break;
    }
    if (xQueueReceive(g_telnetLogQueue, &msg, 0) != pdTRUE) {
      break;
    }
    g_telnetClient.println(msg.text);
    drained++;
  }
}

void sendTelnet(const String& message) {
  EnviarMensajeTelnet(message);
}

bool shouldForwardLog(const String& message) {
  if (!kFocusedControlLogsOnly) {
    return true;
  }
  return message.startsWith("[PI]") || message.startsWith("[DRIVE]") || message.startsWith("[SPID]") ||
         message.startsWith("[SPD]") || message.startsWith("[RC]");
}

void markTelnetActivity() {
  g_telnetLastActivityTick = xTaskGetTickCount();
}

void resetTelnetSession() {
  if (g_telnetClient) {
    g_telnetClient.stop();
  }
  clearTelnetLogQueue();
  // Return to low-noise operation when a Telnet session ends.
  quadDriveSetLogEnabled(false);
  quadDriveSetPidTraceEnabled(false, kDrivePidTraceDefaultPeriod);
  quadDriveSetPwmOverride(false, 0);
  quadDriveSetDirectionOverride(QuadDriveDirection::kForward);
  g_telnetCommandBuffer = "";
  g_speedStreamEnabled = false;
  g_lastSpeedStreamTick = 0;
  g_pidStreamEnabled = false;
  g_lastPidStreamTick = 0;
  g_speedPidStreamEnabled = false;
  g_lastSpeedPidStreamTick = 0;
  g_driveRcStreamEnabled = false;
  g_lastDriveRcStreamTick = 0;
  systemDiagSetJitterStream(false, 1);
  g_telnetLastActivityTick = 0;
}

void openTelnetSession(WiFiClient& incoming) {
  g_telnetClient = incoming;
  g_telnetClient.setNoDelay(true);
  g_telnetCommandBuffer = "";
  markTelnetActivity();
  g_telnetClient.println("=== Servidor Telnet ESP32 ===");
  g_telnetClient.println("Conexion establecida correctamente");
  g_telnetClient.println(
      "Comandos: steer.help | pid.help | spid.help | comms.status | speed.status | speed.reset | speed.stream | speed.uart | pid.stream | spid.stream | spid.target | drive.log | drive.pwm | drive.dir | drive.rc.status | drive.rc.stream | sys.rt | sys.stack | sys.jitter | sys.reset | net.status | exit");
  reportNetworkStatus();
}

bool parseFloatArg(const String& text, float& valueOut) {
  if (text.length() == 0) {
    return false;
  }
  const char* raw = text.c_str();
  char* endPtr = nullptr;
  valueOut = static_cast<float>(strtod(raw, &endPtr));
  if (endPtr == raw) {
    return false;
  }
  while (endPtr != nullptr && *endPtr != '\0') {
    if (!isspace(static_cast<unsigned char>(*endPtr))) {
      return false;
    }
    ++endPtr;
  }
  return true;
}

bool parseIntArg(const String& text, int& valueOut) {
  if (text.length() == 0) {
    return false;
  }
  const char* raw = text.c_str();
  char* endPtr = nullptr;
  const long parsed = strtol(raw, &endPtr, 10);
  if (endPtr == raw) {
    return false;
  }
  while (endPtr != nullptr && *endPtr != '\0') {
    if (!isspace(static_cast<unsigned char>(*endPtr))) {
      return false;
    }
    ++endPtr;
  }
  valueOut = static_cast<int>(parsed);
  return true;
}

bool parseNextFloat(const char*& cursor, float& valueOut) {
  if (cursor == nullptr) {
    return false;
  }
  while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  if (*cursor == '\0') {
    return false;
  }
  char* endPtr = nullptr;
  valueOut = static_cast<float>(strtod(cursor, &endPtr));
  if (endPtr == cursor) {
    return false;
  }
  cursor = endPtr;
  return true;
}

bool parseFloatTriplet(const String& text, float& a, float& b, float& c) {
  const char* cursor = text.c_str();
  if (!parseNextFloat(cursor, a)) {
    return false;
  }
  if (!parseNextFloat(cursor, b)) {
    return false;
  }
  if (!parseNextFloat(cursor, c)) {
    return false;
  }
  while (*cursor != '\0' && isspace(static_cast<unsigned char>(*cursor))) {
    ++cursor;
  }
  return *cursor == '\0';
}

struct ParsedTelnetCommand {
  String line;
  String command;
  String args;
};

bool parseCommandLine(String line, ParsedTelnetCommand& parsedOut) {
  line.trim();
  if (line.isEmpty()) {
    return false;
  }
  const int spaceIndex = line.indexOf(' ');
  parsedOut.line = line;
  parsedOut.command = (spaceIndex < 0) ? line : line.substring(0, spaceIndex);
  parsedOut.args = (spaceIndex < 0) ? "" : line.substring(spaceIndex + 1);
  parsedOut.args.trim();
  return true;
}

bool parseStreamToggleArgs(const String& args,
                           int defaultPeriodMs,
                           int minPeriodMs,
                           int maxPeriodMs,
                           const char* usageMessage,
                           bool& enabledOut,
                           TickType_t& periodTicksOut,
                           int& periodMsOut) {
  String normalized = args;
  normalized.toLowerCase();
  if (normalized == "off" || normalized == "0" || normalized == "stop") {
    enabledOut = false;
    periodMsOut = defaultPeriodMs;
    periodTicksOut = pdMS_TO_TICKS(periodMsOut);
    if (periodTicksOut == 0) {
      periodTicksOut = 1;
    }
    return true;
  }

  int periodMs = defaultPeriodMs;
  bool hasPeriodArg = false;
  String periodText;

  if (normalized.startsWith("on")) {
    periodText = args.substring(2);
    periodText.trim();
    hasPeriodArg = !periodText.isEmpty();
  } else {
    periodText = args;
    hasPeriodArg = true;
  }

  if (hasPeriodArg) {
    int parsedPeriod = 0;
    if (!parseIntArg(periodText, parsedPeriod)) {
      sendTelnet(usageMessage);
      return false;
    }
    if (parsedPeriod < minPeriodMs) {
      parsedPeriod = minPeriodMs;
    } else if (parsedPeriod > maxPeriodMs) {
      parsedPeriod = maxPeriodMs;
    }
    periodMs = parsedPeriod;
  }

  TickType_t periodTicks = pdMS_TO_TICKS(periodMs);
  if (periodTicks == 0) {
    periodTicks = 1;
  }

  enabledOut = true;
  periodMsOut = periodMs;
  periodTicksOut = periodTicks;
  return true;
}

void reportSteeringStatus() {
  const SteeringCalibrationData data = steeringCalibrationSnapshot();
  String msg = "[STEER] estado=";
  msg += data.hasCalibration ? "calibrado" : "sin-calibrar";
  msg += " centro=";
  msg += String(data.adjustedCenterDeg, 2);
  msg += "deg (raw ";
  msg += String(data.rawCenterDeg, 2);
  msg += "deg)";
  msg += " offset=";
  msg += String(data.userOffsetDeg, 2);
  msg += "deg limites=[";
  msg += String(data.leftLimitDeg, 2);
  msg += ", ";
  msg += String(data.rightLimitDeg, 2);
  msg += "]";
  msg += " offsetRange=[";
  msg += String(data.maxOffsetLeftDeg, 2);
  msg += ", ";
  msg += String(data.maxOffsetRightDeg, 2);
  msg += "]";
  if (data.lastCalibrationMs != 0) {
    msg += " tCal=";
    msg += String(data.lastCalibrationMs);
    msg += "ms";
  }
  sendTelnet(msg);
}

void reportPidStatus() {
  float kp = 0.0f;
  float ki = 0.0f;
  float kd = 0.0f;
  float deadband = 0.0f;
  float minActive = 0.0f;
  if (!pidGetTunings(kp, ki, kd)) {
    sendTelnet("[PID] Controlador no registrado");
    return;
  }
  String msg = "[PID] kp=";
  msg += String(kp, 3);
  msg += " ki=";
  msg += String(ki, 3);
  msg += " kd=";
  msg += String(kd, 3);
  if (pidGetOutputModifiers(deadband, minActive)) {
    msg += " deadband=";
    msg += String(deadband, 3);
    msg += "% minActive=";
    msg += String(minActive, 3);
    msg += "%";
  }
  sendTelnet(msg);
}

void reportPidRuntimeStatus() {
  PidRuntimeSnapshot snapshot{};
  if (!pidGetRuntimeSnapshot(snapshot)) {
    sendTelnet("[PID][RT] Sin datos runtime");
    return;
  }

  String msg = "[PID][RT] src=";
  msg += snapshot.steeringFromPi ? "PI" : "RC";
  msg += " calib=";
  msg += snapshot.calibrationActive ? "Y" : "N";
  msg += " sensor=";
  msg += snapshot.sensorValid ? "OK" : "ERR";
  msg += " limL=";
  msg += snapshot.limitLeftActive ? "1" : "0";
  msg += " limR=";
  msg += snapshot.limitRightActive ? "1" : "0";
  msg += " cmd=";
  msg += snapshot.steeringCommand;
  msg += " dt=";
  msg += String(snapshot.dtSeconds * 1000.0f, 1);
  msg += "ms";
  if (snapshot.sensorValid) {
    msg += " sensorDeg=";
    msg += String(snapshot.measuredDeg, 2);
    msg += " targetDeg=";
    msg += String(snapshot.targetDeg, 2);
    msg += " errDeg=";
    msg += String(snapshot.errorDeg, 2);
    msg += " out=";
    msg += String(snapshot.outputPercent, 1);
    msg += "%";
  }
  sendTelnet(msg);
}

void reportSpeedPidStatus() {
  SpeedPidRuntimeSnapshot snapshot{};
  if (!speedPidGetSnapshot(snapshot)) {
    sendTelnet("[SPID] Controlador no inicializado");
    return;
  }
  float reverseMaxMps = snapshot.config.maxSpeedMps;
  float reverseAwx = 1.0f;
  float reverseErrThMps = 0.10f;
  quadDriveGetReverseSpeedLimitMps(reverseMaxMps);
  quadDriveGetReverseAntiWindupScale(reverseAwx);
  quadDriveGetReverseAntiWindupErrorThresholdMps(reverseErrThMps);

  String msg = "[SPID] state{init=";
  msg += snapshot.initialized ? "Y" : "N";
  msg += " en=";
  msg += snapshot.enabled ? "Y" : "N";
  msg += " fb=";
  msg += snapshot.feedbackOk ? "Y" : "N";
  msg += " failsafe=";
  msg += snapshot.failsafeActive ? "Y" : "N";
  msg += " overspeed=";
  msg += snapshot.overspeedActive ? "Y" : "N";
  msg += " mode=";
  msg += speedPidModeText(snapshot.mode);
  msg += "} targetRaw=";
  msg += String(snapshot.targetRawMps, 2);
  msg += "m/s target=";
  msg += String(snapshot.targetRampedMps, 2);
  msg += "m/s speedRaw=";
  msg += String(snapshot.measuredMps, 2);
  msg += "m/s speedFilt=";
  msg += String(snapshot.measuredFilteredMps, 2);
  msg += "m/s err=";
  msg += String(snapshot.errorMps, 2);
  msg += "m/s p=";
  msg += String(snapshot.pTerm, 2);
  msg += " i=";
  msg += String(snapshot.iTerm, 2);
  msg += " d=";
  msg += String(snapshot.dTerm, 2);
  msg += " unsat=";
  msg += String(snapshot.pidUnsatOutput, 2);
  msg += " sat=";
  msg += String(snapshot.pidSatOutput, 2);
  msg += " ffBase=";
  msg += String(snapshot.throttleBasePercent, 1);
  msg += "% ffDelta=";
  msg += String(snapshot.throttlePidDeltaPercent, 1);
  msg += "% ffPre=";
  msg += String(snapshot.throttleCmdPreSlewPercent, 1);
  msg += "% ffAct=";
  msg += snapshot.throttleBaseActive ? "Y" : "N";
  msg += " overErr=";
  msg += String(snapshot.overspeedErrorMps, 2);
  msg += " throttle=";
  msg += String(snapshot.throttleCmdPercent, 1);
  msg += "% throttleRaw=";
  msg += String(snapshot.throttleCmdRawPercent, 1);
  msg += "% throttleFilt=";
  msg += String(snapshot.throttleCmdFilteredPercent, 1);
  msg += "% brake=";
  msg += String(snapshot.brakeCmdPercent, 1);
  msg += "% brakeRaw=";
  msg += String(snapshot.overspeedBrakeRawPercent, 1);
  msg += "% brakeFilt=";
  msg += String(snapshot.overspeedBrakeFilteredPercent, 1);
  msg += "% sat=";
  msg += snapshot.throttleSaturated ? "Y" : "N";
  msg += " iclamp=";
  msg += snapshot.integratorClamped ? "Y" : "N";
  msg += " launch=";
  msg += snapshot.launchAssistActive ? "Y" : "N";
  msg += " launchMs=";
  msg += snapshot.launchAssistRemainingMs;
  msg += " hold=";
  msg += snapshot.overspeedHoldActive ? "Y" : "N";
  msg += " holdMs=";
  msg += snapshot.overspeedHoldRemainingMs;
  msg += " tune{kp=";
  msg += String(snapshot.tunings.kp, 3);
  msg += " ki=";
  msg += String(snapshot.tunings.ki, 3);
  msg += " kd=";
  msg += String(snapshot.tunings.kd, 3);
  msg += "} cfg{max=";
  msg += String(snapshot.config.maxSpeedMps, 2);
  msg += "m/s ramp=";
  msg += String(snapshot.config.maxSetpointRateMps2, 2);
  msg += "m/s2 ilim=";
  msg += String(snapshot.config.integralLimit, 2);
  msg += " db=";
  msg += String(snapshot.config.deadbandMps, 3);
  msg += "m/s tmin=";
  msg += String(snapshot.config.minThrottlePercent, 1);
  msg += "% thsup=";
  msg += String(snapshot.config.throttleSlewUpPctPerSec, 1);
  msg += " thsdown=";
  msg += String(snapshot.config.throttleSlewDownPctPerSec, 1);
  msg += " minspd=";
  msg += String(snapshot.config.minThrottleAssistMaxSpeedMps, 2);
  msg += "m/s lwin=";
  msg += snapshot.config.launchAssistWindowMs;
  msg += "ms ff{en=";
  msg += snapshot.config.throttleBaseEnable ? "Y" : "N";
  msg += " b0=";
  msg += String(snapshot.config.throttleBaseAtZeroMpsPercent, 1);
  msg += " bmax=";
  msg += String(snapshot.config.throttleBaseAtMaxSpeedPercent, 1);
  msg += " du=";
  msg += String(snapshot.config.throttleBasePidDeltaUpMaxPercent, 1);
  msg += " dd=";
  msg += String(snapshot.config.throttleBasePidDeltaDownMaxPercent, 1);
  msg += " min=";
  msg += String(snapshot.config.throttleBaseActivationMinMps, 2);
  msg += "m/s} flgr=";
  msg += snapshot.config.feedbackLaunchGraceMs;
  msg += "ms iunw=";
  msg += String(snapshot.config.integratorUnwindGain, 2);
  msg += " dfhz=";
  msg += String(snapshot.config.derivativeFilterHz, 2);
  msg += " cap=";
  msg += String(snapshot.config.overspeedBrakeMaxPercent, 1);
  msg += "% hys=";
  msg += String(snapshot.config.overspeedReleaseHysteresisMps, 2);
  msg += "m/s brsu=";
  msg += String(snapshot.config.overspeedBrakeSlewUpPctPerSec, 1);
  msg += " brsd=";
  msg += String(snapshot.config.overspeedBrakeSlewDownPctPerSec, 1);
  msg += " brh=";
  msg += snapshot.config.overspeedBrakeHoldMs;
  msg += "ms brdb=";
  msg += String(snapshot.config.overspeedBrakeDeadbandPercent, 1);
  msg += "%} rev{max=";
  msg += String(reverseMaxMps, 2);
  msg += "m/s awx=";
  msg += String(reverseAwx, 2);
  msg += " eth=";
  msg += String(reverseErrThMps, 2);
  msg += "m/s}";
  sendTelnet(msg);
}

void reportSpeedPidTargetStatus() {
  bool enabled = false;
  float targetMps = 0.0f;
  if (!quadDriveGetSpeedTargetOverride(enabled, targetMps)) {
    sendTelnet("[SPID][TARGET] N/A");
    return;
  }

  String msg = "[SPID][TARGET] ";
  msg += enabled ? "ON" : "OFF";
  msg += " target=";
  msg += String(targetMps, 2);
  msg += "m/s";
  sendTelnet(msg);
}

void reportSpeedPidReverseStatus() {
  float reverseMaxMps = speedPidGetMaxSpeedMps();
  float reverseAwx = 1.0f;
  float reverseErrThMps = 0.10f;
  quadDriveGetReverseSpeedLimitMps(reverseMaxMps);
  quadDriveGetReverseAntiWindupScale(reverseAwx);
  quadDriveGetReverseAntiWindupErrorThresholdMps(reverseErrThMps);

  String msg = "[SPID][REV] max=";
  msg += String(reverseMaxMps, 2);
  msg += "m/s awx=";
  msg += String(reverseAwx, 2);
  msg += " errTh=";
  msg += String(reverseErrThMps, 2);
  msg += "m/s";
  sendTelnet(msg);
}

void reportDrivePwmOverrideStatus() {
  bool enabled = false;
  int percent = 0;
  if (!quadDriveGetPwmOverride(enabled, percent)) {
    sendTelnet("[DRIVE][PWM] N/A");
    return;
  }
  String msg = "[DRIVE][PWM] ";
  msg += enabled ? "ON" : "OFF";
  msg += " duty=";
  msg += percent;
  msg += "%";
  sendTelnet(msg);
}

const char* driveDirectionText(QuadDriveDirection dir) {
  return (dir == QuadDriveDirection::kReverse) ? "REV" : "FWD";
}

const char* hallDirectionText(HallDirection direction) {
  switch (direction) {
    case HallDirection::kForward:
      return "FWD";
    case HallDirection::kReverse:
      return "REV";
    case HallDirection::kUnknown:
    default:
      return "UNK";
  }
}

void reportDriveDirectionStatus() {
  QuadDriveDirection direction = QuadDriveDirection::kForward;
  bool switching = false;
  bool relayEnergized = false;
  if (!quadDriveGetDirectionStatus(direction, switching, relayEnergized)) {
    sendTelnet("[DRIVE][DIR] N/A");
    return;
  }

  uint8_t relayPin = 0;
  bool activeLow = true;
  quadDriveGetDirectionConfig(relayPin, activeLow);

  bool pwmEnabled = false;
  int pwmPercent = 0;
  quadDriveGetPwmOverride(pwmEnabled, pwmPercent);

  String msg = "[DRIVE][DIR] ";
  msg += driveDirectionText(direction);
  msg += " switching=";
  msg += switching ? "Y" : "N";
  msg += " relay=";
  msg += relayEnergized ? "ON" : "OFF";
  msg += " pin=";
  msg += relayPin;
  msg += " activeLow=";
  msg += activeLow ? "Y" : "N";
  msg += " pwm=";
  msg += pwmEnabled ? "ON" : "OFF";
  msg += "(";
  msg += pwmPercent;
  msg += "%)";
  sendTelnet(msg);
}

void reportNetworkStatus() {
  String msg = "[NET] mode=";
  msg += networkModeText(g_networkMode);
  msg += " ssid=";
  msg += g_networkSsid.length() > 0 ? g_networkSsid : "-";
  msg += " ip=";
  msg += g_networkIp.toString();
  msg += " hostname=";
  msg += OTA_HOSTNAME;
  msg += " telnetPort=";
  msg += String(kTelnetPort);
  msg += " logQ=";
  msg += String(static_cast<uint32_t>(telnetQueueDepth()));
  msg += "/";
  msg += String(static_cast<uint32_t>(kTelnetLogQueueLength));
  msg += " logDrop=";
  msg += String(telnetLogDropCount());
  sendTelnet(msg);
}

String buildDriveRcStatusMessage() {
  QuadDriveRcDebugSnapshot snapshot{};
  if (!quadDriveGetRcDebugSnapshot(snapshot)) {
    return "[DRIVE][RC] N/A";
  }

  String msg = "[DRIVE][RC] raw=";
  msg += snapshot.rawThrottle;
  msg += " filt=";
  msg += snapshot.filteredThrottle;
  msg += " norm=";
  msg += snapshot.normalizedThrottle;
  msg += " aux5=";
  msg += snapshot.rcAuxReverse;
  msg += " fresh=";
  msg += snapshot.rcFresh ? "Y" : "N";
  msg += " ageMs=";
  msg += snapshot.snapshotAgeMs;
  msg += " brake=";
  msg += snapshot.rcManualBrakeActive ? "Y" : "N";
  msg += " elig=";
  msg += snapshot.rcSpeedPidEligible ? "Y" : "N";
  msg += " revReq=";
  msg += snapshot.rcReverseRequest ? "Y" : "N";
  msg += " latched=";
  msg += snapshot.rcSourceLatched ? "Y" : "N";
  msg += " cal{en=";
  msg += snapshot.rcNeutralOffsetCalEnabled ? "Y" : "N";
  msg += " allow=";
  msg += snapshot.rcNeutralOffsetCalAllowed ? "Y" : "N";
  msg += "} targetRaw=";
  msg += String(snapshot.rcTargetRawMps, 3);
  msg += "m/s targetShaped=";
  msg += String(snapshot.rcTargetShapedMps, 3);
  msg += "m/s";
  return msg;
}

String buildSpeedStatusMessage() {
  HallSpeedSnapshot snapshot{};
  const bool ok = hallSpeedGetSnapshot(snapshot);
  HallSpeedConfig cfg{};
  hallSpeedGetConfig(cfg);

  String msg = "[SPD][STATUS] source=hall driver=";
  msg += (ok && snapshot.driverReady) ? "READY" : "NOT_READY";
  msg += " dir=";
  msg += hallDirectionText(snapshot.direction);
  msg += " speed=";
  msg += String(snapshot.speedKmh, 2);
  msg += "km/h ";
  msg += String(snapshot.speedMps, 2);
  msg += "m/s rpm=";
  msg += String(snapshot.motorRpm, 1);
  msg += " speedAbs=";
  msg += String(fabsf(snapshot.speedMps), 2);
  msg += "m/s";
  msg += " hall=0b";
  msg += (snapshot.hallMask & (1U << 2)) ? '1' : '0';
  msg += (snapshot.hallMask & (1U << 1)) ? '1' : '0';
  msg += (snapshot.hallMask & (1U << 0)) ? '1' : '0';
  msg += " ageUs=";
  msg += snapshot.transitionAgeUs;
  msg += " periodUs=";
  msg += snapshot.transitionPeriodUs;
  msg += " ok=";
  msg += snapshot.transitionsOk;
  msg += " invState=";
  msg += snapshot.transitionsInvalidState;
  msg += " invJump=";
  msg += snapshot.transitionsInvalidJump;
  msg += " isr=";
  msg += snapshot.isrCount;
  msg += " cfg{pins=";
  msg += cfg.pinA;
  msg += "/";
  msg += cfg.pinB;
  msg += "/";
  msg += cfg.pinC;
  msg += " low=";
  msg += cfg.activeLow ? "Y" : "N";
  msg += " dirInv=";
  msg += cfg.directionInverted ? "Y" : "N";
  msg += " poles=";
  msg += cfg.motorPoles;
  msg += " red=";
  msg += String(cfg.gearReduction, 2);
  msg += " wheelM=";
  msg += String(cfg.wheelDiameterM, 3);
  msg += "}";
  return msg;
}

void reportSystemRtStatus() {
  SystemDiagSnapshot snapshot{};
  if (!systemDiagGetSnapshot(snapshot)) {
    sendTelnet("[SYS][RT] N/A");
    return;
  }

  String header = "[SYS][RT] tsMs=";
  header += snapshot.timestampMs;
  header += " tasks=";
  header += snapshot.taskCount;
  header += " worstJitterUs=";
  header += snapshot.worstJitterUs;
  header += " cpuStats=";
  header += snapshot.cpuStatsEnabled ? "ON" : "OFF";
  sendTelnet(header);

  for (size_t i = 0; i < kSystemDiagTaskCount; ++i) {
    const SystemDiagTaskSample& task = snapshot.tasks[i];
    if (!task.registered) {
      continue;
    }
    String line = "[SYS][RT] ";
    line += task.name;
    line += " core=";
    line += task.core;
    line += " prio=";
    line += task.priority;
    line += " loopUs=";
    line += task.lastLoopUs;
    line += " maxLoopUs=";
    line += task.maxLoopUs;
    line += " jitterUs=";
    line += task.jitterUs;
    line += " maxJitterUs=";
    line += task.maxJitterUs;
    line += " overrun=";
    line += task.overrunCount;
    line += " notifyTimeout=";
    line += task.notifyTimeoutCount;
    sendTelnet(line);
  }
}

void reportSystemStackStatus() {
  SystemDiagSnapshot snapshot{};
  if (!systemDiagGetSnapshot(snapshot)) {
    sendTelnet("[SYS][STACK] N/A");
    return;
  }

  String header = "[SYS][STACK] tsMs=";
  header += snapshot.timestampMs;
  header += " tasks=";
  header += snapshot.taskCount;
  sendTelnet(header);

  for (size_t i = 0; i < kSystemDiagTaskCount; ++i) {
    const SystemDiagTaskSample& task = snapshot.tasks[i];
    if (!task.registered) {
      continue;
    }
    const uint32_t stackBytes = task.stackHighWaterWords * sizeof(StackType_t);
    String line = "[SYS][STACK] ";
    line += task.name;
    line += " highWaterWords=";
    line += task.stackHighWaterWords;
    line += " highWaterBytes=";
    line += stackBytes;
    line += " core=";
    line += task.core;
    sendTelnet(line);
  }
}

String buildSystemJitterSummary() {
  SystemDiagSnapshot snapshot{};
  if (!systemDiagGetSnapshot(snapshot)) {
    return "[SYS][JITTER] N/A";
  }

  String msg = "[SYS][JITTER] tsMs=";
  msg += snapshot.timestampMs;
  msg += " worstMaxUs=";
  msg += snapshot.worstJitterUs;
  msg += " tasks=";
  msg += snapshot.taskCount;

  for (size_t i = 0; i < kSystemDiagTaskCount; ++i) {
    const SystemDiagTaskSample& task = snapshot.tasks[i];
    if (!task.registered) {
      continue;
    }
    msg += " ";
    msg += task.name;
    msg += "=";
    msg += task.jitterUs;
    msg += "/";
    msg += task.maxJitterUs;
  }
  return msg;
}

bool handleSteerCommand(const String& command, const String& args) {
  if (command.equalsIgnoreCase("steer.calibrate")) {
    steeringCalibrationRequest();
    sendTelnet("[STEER] Calibracion solicitada");
    return true;
  }

  if (command.equalsIgnoreCase("steer.offset")) {
    if (args.isEmpty()) {
      reportSteeringStatus();
      return true;
    }
    float offset = 0.0f;
    if (!parseFloatArg(args, offset)) {
      sendTelnet("[STEER] Offset invalido, usa grados (ej: steer.offset -2.5)");
      return true;
    }
    steeringCalibrationSetOffset(offset);
    const SteeringCalibrationData data = steeringCalibrationSnapshot();
    String msg = "[STEER] Offset aplicado=";
    msg += String(data.userOffsetDeg, 2);
    msg += "deg (rango ";
    msg += String(data.maxOffsetLeftDeg, 2);
    msg += " .. ";
    msg += String(data.maxOffsetRightDeg, 2);
    msg += ")";
    sendTelnet(msg);
    return true;
  }

  if (command.equalsIgnoreCase("steer.reset") || command.equalsIgnoreCase("steer.clear")) {
    steeringCalibrationReset(true);
    sendTelnet("[STEER] Calibracion reseteada a valores por defecto (NVS borrado)");
    reportSteeringStatus();
    return true;
  }

  if (command.equalsIgnoreCase("steer.status")) {
    reportSteeringStatus();
    return true;
  }

  if (command.equalsIgnoreCase("steer.help")) {
    sendTelnet("Comandos: steer.calibrate | steer.offset <deg> | steer.reset | steer.status");
    return true;
  }

  return false;
}

bool handleSessionCommand(const String& command, const String& args) {
  (void)args;

  if (command.equalsIgnoreCase("exit") || command.equalsIgnoreCase("quit") || command.equalsIgnoreCase("logout")) {
    sendTelnet("[TELNET] Cerrando sesion");
    resetTelnetSession();
    return true;
  }

  return false;
}

bool handleCommsCommand(const String& command, const String& args) {
  (void)args;
  if (command.equalsIgnoreCase("comms.status")) {
    PiCommsRxSnapshot snapshot{};
    piCommsGetRxSnapshot(snapshot);
    String msg = "[PI][STATUS] driver=";
    msg += snapshot.driverReady ? "READY" : "NOT_READY";
    msg += " lastFrame=";
    if (snapshot.hasFrame) {
      const TickType_t ageTicks = xTaskGetTickCount() - snapshot.lastFrameTick;
      const uint32_t ageMs = ageTicks * portTICK_PERIOD_MS;
      msg += String(ageMs);
      msg += "ms";
    } else {
      msg += "NONE";
    }
    msg += " verFlags=0x";
    msg += String(snapshot.verFlags, HEX);
    msg += " steer=";
    msg += snapshot.steer;
    msg += " speedCmd=";
    msg += String(static_cast<float>(snapshot.speedCmdCentiMps) / 100.0f, 2);
    msg += "m/s";
    msg += " speedCmdSigned=";
    msg += String(static_cast<float>(snapshot.speedCmdSignedCentiMps) / 100.0f, 2);
    msg += "m/s";
    msg += " revReq=";
    msg += snapshot.speedReverseRequest ? "Y" : "N";
    msg += " brake=";
    msg += snapshot.brake;
    msg += " estop=";
    msg += snapshot.estop ? "Y" : "N";
    msg += " drive=";
    msg += snapshot.driveEnabled ? "Y" : "N";
    msg += " ok=";
    msg += snapshot.framesOk;
    msg += " crcErr=";
    msg += snapshot.framesCrcError;
    msg += " malformed=";
    msg += snapshot.framesMalformed;
    msg += " verErr=";
    msg += snapshot.framesVersionError;
    sendTelnet(msg);
    return true;
  }

  if (command.equalsIgnoreCase("comms.reset")) {
    piCommsResetStats();
    sendTelnet("[PI][STATUS] Contadores reseteados");
    return true;
  }

  return false;
}

bool handleSpeedCommand(const String& command, const String& args) {
  if (command.equalsIgnoreCase("speed.status")) {
    sendTelnet(buildSpeedStatusMessage());
    return true;
  }

  if (command.equalsIgnoreCase("speed.reset")) {
    hallSpeedResetStats();
    sendTelnet("[SPD][STATUS] Contadores Hall reseteados");
    sendTelnet(buildSpeedStatusMessage());
    return true;
  }

  if (command.equalsIgnoreCase("speed.stream")) {
    if (args.isEmpty()) {
      const uint32_t periodMs = static_cast<uint32_t>(g_speedStreamPeriod * portTICK_PERIOD_MS);
      String msg = "[SPD][STREAM] ";
      msg += g_speedStreamEnabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += periodMs;
      msg += "ms";
      sendTelnet(msg);
      return true;
    }

    bool enabled = false;
    TickType_t periodTicks = g_speedStreamPeriod;
    int periodMs = static_cast<int>(g_speedStreamPeriod * portTICK_PERIOD_MS);
    if (!parseStreamToggleArgs(args,
                               periodMs,
                               kSpeedStreamMinPeriodMs,
                               kSpeedStreamMaxPeriodMs,
                               "[SPD][STREAM] Uso: speed.stream on [ms] | speed.stream off",
                               enabled,
                               periodTicks,
                               periodMs)) {
      return true;
    }

    if (!enabled) {
      g_speedStreamEnabled = false;
      sendTelnet("[SPD][STREAM] OFF");
      return true;
    }

    g_speedStreamPeriod = periodTicks;
    g_speedStreamEnabled = true;
    g_lastSpeedStreamTick = 0;

    String msg = "[SPD][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    sendTelnet(buildSpeedStatusMessage());
    return true;
  }

  if (command.equalsIgnoreCase("speed.uart")) {
    sendTelnet("[SPD][UART] N/A source=hall");
    return true;
  }

  return false;
}

bool handleSysCommand(const String& command, const String& args) {
  if (command.equalsIgnoreCase("sys.rt")) {
    reportSystemRtStatus();
    return true;
  }

  if (command.equalsIgnoreCase("sys.stack")) {
    reportSystemStackStatus();
    return true;
  }

  if (command.equalsIgnoreCase("sys.reset")) {
    bool keepRegistration = true;
    if (!args.isEmpty()) {
      String normalized = args;
      normalized.toLowerCase();
      if (normalized == "full" || normalized == "all" || normalized == "0" || normalized == "false") {
        keepRegistration = false;
      } else if (normalized == "keep" || normalized == "1" || normalized == "true") {
        keepRegistration = true;
      } else {
        sendTelnet("[SYS][RESET] Uso: sys.reset [keep|full]");
        return true;
      }
    }
    if (!systemDiagResetStats(keepRegistration)) {
      sendTelnet("[SYS][RESET] ERROR");
      return true;
    }
    String msg = "[SYS][RESET] OK keepRegistration=";
    msg += keepRegistration ? "1" : "0";
    sendTelnet(msg);
    return true;
  }

  if (command.equalsIgnoreCase("sys.jitter")) {
    if (args.isEmpty()) {
      bool enabled = false;
      TickType_t periodTicks = kSystemJitterDefaultPeriod;
      systemDiagGetJitterStreamConfig(enabled, periodTicks);
      String msg = "[SYS][JITTER] ";
      msg += enabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += static_cast<uint32_t>(periodTicks * portTICK_PERIOD_MS);
      msg += "ms";
      sendTelnet(msg);
      sendTelnet(buildSystemJitterSummary());
      return true;
    }

    bool enabled = false;
    TickType_t periodTicks = kSystemJitterDefaultPeriod;
    int periodMs = static_cast<int>(kSystemJitterDefaultPeriod * portTICK_PERIOD_MS);
    if (!parseStreamToggleArgs(args,
                               periodMs,
                               kSystemJitterMinPeriodMs,
                               kSystemJitterMaxPeriodMs,
                               "[SYS][JITTER] Uso: sys.jitter on [ms] | sys.jitter off",
                               enabled,
                               periodTicks,
                               periodMs)) {
      return true;
    }

    if (!enabled) {
      if (!systemDiagSetJitterStream(false, 1)) {
        sendTelnet("[SYS][JITTER] Error desactivando stream");
        return true;
      }
      sendTelnet("[SYS][JITTER] OFF");
      return true;
    }

    if (!systemDiagSetJitterStream(true, periodTicks)) {
      sendTelnet("[SYS][JITTER] Error activando stream");
      return true;
    }
    String msg = "[SYS][JITTER] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    sendTelnet(buildSystemJitterSummary());
    return true;
  }

  return false;
}

bool handleNetCommand(const String& command, const String& args) {
  (void)args;
  if (command.equalsIgnoreCase("net.status")) {
    reportNetworkStatus();
    return true;
  }
  return false;
}

bool handlePidCommand(const String& command, const String& args) {
  if (command.equalsIgnoreCase("pid.set")) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!parseFloatTriplet(args, kp, ki, kd)) {
      sendTelnet("[PID] Uso: pid.set <kp> <ki> <kd>");
      return true;
    }
    if (!pidSetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.kp")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Kp invalido (ej: pid.kp 2.5)");
      return true;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!pidGetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    if (!pidSetTunings(value, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.ki")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Ki invalido (ej: pid.ki 0.5)");
      return true;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!pidGetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    if (!pidSetTunings(kp, value, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.kd")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Kd invalido (ej: pid.kd 0.1)");
      return true;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!pidGetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    if (!pidSetTunings(kp, ki, value)) {
      sendTelnet("[PID] Controlador no registrado");
      return true;
    }
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.status")) {
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.help")) {
    sendTelnet(
        "Comandos: pid.set <kp> <ki> <kd> | pid.kp <v> | pid.ki <v> | pid.kd <v> | pid.deadband <pct> | pid.minactive <pct> | pid.status | pid.stream on [ms] | pid.stream off");
    return true;
  }

  if (command.equalsIgnoreCase("pid.deadband")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Deadband invalido (ej: pid.deadband 0.3)");
      return true;
    }
    if (!pidSetDeadband(value)) {
      sendTelnet("[PID] Configuracion PID no registrada");
      return true;
    }
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.minactive")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] MinActive invalido (ej: pid.minactive 10)");
      return true;
    }
    if (!pidSetMinActive(value)) {
      sendTelnet("[PID] Configuracion PID no registrada");
      return true;
    }
    reportPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("pid.stream")) {
    if (args.isEmpty()) {
      const uint32_t periodMs = static_cast<uint32_t>(g_pidStreamPeriod * portTICK_PERIOD_MS);
      String msg = "[PID][STREAM] ";
      msg += g_pidStreamEnabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += periodMs;
      msg += "ms";
      sendTelnet(msg);
      return true;
    }

    bool enabled = false;
    TickType_t periodTicks = g_pidStreamPeriod;
    int periodMs = static_cast<int>(g_pidStreamPeriod * portTICK_PERIOD_MS);
    if (!parseStreamToggleArgs(args,
                               periodMs,
                               kPidStreamMinPeriodMs,
                               kPidStreamMaxPeriodMs,
                               "[PID][STREAM] Uso: pid.stream on [ms] | pid.stream off",
                               enabled,
                               periodTicks,
                               periodMs)) {
      return true;
    }

    if (!enabled) {
      g_pidStreamEnabled = false;
      sendTelnet("[PID][STREAM] OFF");
      return true;
    }

    g_pidStreamPeriod = periodTicks;
    g_pidStreamEnabled = true;
    g_lastPidStreamTick = 0;

    String msg = "[PID][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    reportPidStatus();
    reportPidRuntimeStatus();
    return true;
  }

  return false;
}

bool handleSpidCommand(const String& command, const String& args) {
  if (command.equalsIgnoreCase("spid.set")) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!parseFloatTriplet(args, kp, ki, kd)) {
      sendTelnet("[SPID] Uso: spid.set <kp> <ki> <kd>");
      return true;
    }
    SpeedPidTunings tunings{kp, ki, kd};
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Tunings invalidos o controlador no inicializado");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.kp")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Kp invalido (ej: spid.kp 10.0)");
      return true;
    }
    SpeedPidTunings tunings{};
    if (!speedPidGetTunings(tunings)) {
      sendTelnet("[SPID] Controlador no inicializado");
      return true;
    }
    tunings.kp = value;
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Kp fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ki")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Ki invalido (ej: spid.ki 2.0)");
      return true;
    }
    SpeedPidTunings tunings{};
    if (!speedPidGetTunings(tunings)) {
      sendTelnet("[SPID] Controlador no inicializado");
      return true;
    }
    tunings.ki = value;
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Ki fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.kd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Kd invalido (ej: spid.kd 0.1)");
      return true;
    }
    SpeedPidTunings tunings{};
    if (!speedPidGetTunings(tunings)) {
      sendTelnet("[SPID] Controlador no inicializado");
      return true;
    }
    tunings.kd = value;
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Kd fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ramp")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Ramp invalido (ej: spid.ramp 2.0)");
      return true;
    }
    if (!speedPidSetRampRateMps2(value)) {
      sendTelnet("[SPID] Ramp fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.minthrottle")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] MinThrottle invalido (ej: spid.minthrottle 90)");
      return true;
    }
    if (!speedPidSetMinThrottlePercent(value)) {
      sendTelnet("[SPID] MinThrottle fuera de rango (0..100)");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.thslewup")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] ThrottleSlewUp invalido (ej: spid.thslewup 30)");
      return true;
    }
    if (!speedPidSetThrottleSlewUpPctPerSec(value)) {
      sendTelnet("[SPID] ThrottleSlewUp fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.thslewdown")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] ThrottleSlewDown invalido (ej: spid.thslewdown 45)");
      return true;
    }
    if (!speedPidSetThrottleSlewDownPctPerSec(value)) {
      sendTelnet("[SPID] ThrottleSlewDown fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.minth.spd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] MinThrottleAssist speed invalido (ej: spid.minth.spd 0.35)");
      return true;
    }
    if (!speedPidSetMinThrottleAssistMaxSpeedMps(value)) {
      sendTelnet("[SPID] MinThrottleAssist speed fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.launchwin")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    int value = 0;
    if (!parseIntArg(args, value)) {
      sendTelnet("[SPID] LaunchWindow invalido (ej: spid.launchwin 1200)");
      return true;
    }
    if (value < 0 || value > 65535) {
      sendTelnet("[SPID] LaunchWindow fuera de rango");
      return true;
    }
    if (!speedPidSetLaunchAssistWindowMs(static_cast<uint16_t>(value))) {
      sendTelnet("[SPID] LaunchWindow fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff") || command.equalsIgnoreCase("spid.ff.status")) {
    if (command.equalsIgnoreCase("spid.ff.status") || args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    String normalized = args;
    normalized.toLowerCase();
    normalized.trim();
    bool enable = false;
    if (normalized == "on" || normalized == "1" || normalized == "true") {
      enable = true;
    } else if (normalized == "off" || normalized == "0" || normalized == "false") {
      enable = false;
    } else {
      sendTelnet("[SPID] Uso: spid.ff on|off");
      return true;
    }
    if (!speedPidSetThrottleBaseEnable(enable)) {
      sendTelnet("[SPID] Error aplicando ff on/off");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff.base0")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF base0 invalido (ej: spid.ff.base0 0)");
      return true;
    }
    if (!speedPidSetThrottleBaseAtZeroMpsPercent(value)) {
      sendTelnet("[SPID] FF base0 fuera de rango (0..100)");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff.basemax")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF basemax invalido (ej: spid.ff.basemax 55)");
      return true;
    }
    if (!speedPidSetThrottleBaseAtMaxSpeedPercent(value)) {
      sendTelnet("[SPID] FF basemax fuera de rango (0..100)");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff.du")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF delta up invalido (ej: spid.ff.du 35)");
      return true;
    }
    if (!speedPidSetThrottleBasePidDeltaUpMaxPercent(value)) {
      sendTelnet("[SPID] FF delta up fuera de rango (0..100)");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff.dd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF delta down invalido (ej: spid.ff.dd 45)");
      return true;
    }
    if (!speedPidSetThrottleBasePidDeltaDownMaxPercent(value)) {
      sendTelnet("[SPID] FF delta down fuera de rango (0..100)");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff.minspd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF minspd invalido (ej: spid.ff.minspd 0.10)");
      return true;
    }
    if (!speedPidSetThrottleBaseActivationMinMps(value)) {
      sendTelnet("[SPID] FF minspd fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.ff.grace")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    int value = 0;
    if (!parseIntArg(args, value)) {
      sendTelnet("[SPID] FF grace invalido (ej: spid.ff.grace 1200)");
      return true;
    }
    if (value < 0 || value > 65535) {
      sendTelnet("[SPID] FF grace fuera de rango");
      return true;
    }
    if (!speedPidSetFeedbackLaunchGraceMs(static_cast<uint16_t>(value))) {
      sendTelnet("[SPID] FF grace fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.iunwind")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] IntegratorUnwind invalido (ej: spid.iunwind 0.35)");
      return true;
    }
    if (!speedPidSetIntegratorUnwindGain(value)) {
      sendTelnet("[SPID] IntegratorUnwind fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.dfilter")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] DerivativeFilter invalido (ej: spid.dfilter 3.0)");
      return true;
    }
    if (!speedPidSetDerivativeFilterHz(value)) {
      sendTelnet("[SPID] DerivativeFilter fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.max")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Max invalido (ej: spid.max 4.17)");
      return true;
    }
    if (!speedPidSetMaxSpeedMps(value)) {
      sendTelnet("[SPID] Max fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.maxrev")) {
    if (args.isEmpty()) {
      reportSpeedPidReverseStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] MaxRev invalido (ej: spid.maxrev 1.35)");
      return true;
    }
    if (!quadDriveSetReverseSpeedLimitMps(value)) {
      String msg = "[SPID] MaxRev fuera de rango (0.05..";
      msg += String(speedPidGetMaxSpeedMps(), 2);
      msg += " m/s)";
      sendTelnet(msg);
      return true;
    }
    reportSpeedPidReverseStatus();
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.awx")) {
    if (args.isEmpty()) {
      reportSpeedPidReverseStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Awx invalido (ej: spid.awx 3.0)");
      return true;
    }
    if (!quadDriveSetReverseAntiWindupScale(value)) {
      sendTelnet("[SPID] Awx fuera de rango (1.0..10.0)");
      return true;
    }
    reportSpeedPidReverseStatus();
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.brakecap")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeCap invalido (ej: spid.brakecap 30)");
      return true;
    }
    if (!speedPidSetOverspeedBrakeMaxPercent(value)) {
      sendTelnet("[SPID] BrakeCap fuera de rango (0..100)");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.hys")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Hysteresis invalido (ej: spid.hys 0.3)");
      return true;
    }
    if (!speedPidSetOverspeedReleaseHysteresisMps(value)) {
      sendTelnet("[SPID] Hysteresis fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.brakeslewup")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeSlewUp invalido (ej: spid.brakeslewup 35)");
      return true;
    }
    if (!speedPidSetOverspeedBrakeSlewUpPctPerSec(value)) {
      sendTelnet("[SPID] BrakeSlewUp fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.brakeslewdown")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeSlewDown invalido (ej: spid.brakeslewdown 55)");
      return true;
    }
    if (!speedPidSetOverspeedBrakeSlewDownPctPerSec(value)) {
      sendTelnet("[SPID] BrakeSlewDown fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.brakehold")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    int value = 0;
    if (!parseIntArg(args, value)) {
      sendTelnet("[SPID] BrakeHold invalido (ej: spid.brakehold 200)");
      return true;
    }
    if (value < 0 || value > 65535) {
      sendTelnet("[SPID] BrakeHold fuera de rango");
      return true;
    }
    if (!speedPidSetOverspeedBrakeHoldMs(static_cast<uint16_t>(value))) {
      sendTelnet("[SPID] BrakeHold fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.brakedb")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return true;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeDeadband invalido (ej: spid.brakedb 3)");
      return true;
    }
    if (!speedPidSetOverspeedBrakeDeadbandPercent(value)) {
      sendTelnet("[SPID] BrakeDeadband fuera de rango");
      return true;
    }
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.target")) {
    if (args.isEmpty()) {
      reportSpeedPidTargetStatus();
      return true;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      if (!quadDriveSetSpeedTargetOverride(false, 0.0f)) {
        sendTelnet("[SPID][TARGET] No se pudo desactivar override");
        return true;
      }
      reportSpeedPidTargetStatus();
      return true;
    }

    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID][TARGET] Valor invalido (ej: spid.target -1.5 | spid.target off)");
      return true;
    }
    if (!quadDriveSetSpeedTargetOverride(true, value)) {
      float reverseMaxMps = speedPidGetMaxSpeedMps();
      quadDriveGetReverseSpeedLimitMps(reverseMaxMps);
      String msg = "[SPID][TARGET] Fuera de rango (-";
      msg += String(reverseMaxMps, 2);
      msg += "..";
      msg += String(speedPidGetMaxSpeedMps(), 2);
      msg += " m/s)";
      sendTelnet(msg);
      return true;
    }
    reportSpeedPidTargetStatus();
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.save")) {
    if (!speedPidSaveToNvs()) {
      sendTelnet("[SPID] Error guardando en NVS");
      return true;
    }
    sendTelnet("[SPID] Configuracion guardada en NVS");
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.reset")) {
    if (!speedPidResetToDefaults(true)) {
      sendTelnet("[SPID] Error restaurando defaults");
      return true;
    }
    sendTelnet("[SPID] Defaults restaurados y persistidos");
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.status")) {
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.stream")) {
    if (args.isEmpty()) {
      const uint32_t periodMs = static_cast<uint32_t>(g_speedPidStreamPeriod * portTICK_PERIOD_MS);
      String msg = "[SPID][STREAM] ";
      msg += g_speedPidStreamEnabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += periodMs;
      msg += "ms";
      sendTelnet(msg);
      return true;
    }

    bool enabled = false;
    TickType_t periodTicks = g_speedPidStreamPeriod;
    int periodMs = static_cast<int>(g_speedPidStreamPeriod * portTICK_PERIOD_MS);
    if (!parseStreamToggleArgs(args,
                               periodMs,
                               kSpeedPidStreamMinPeriodMs,
                               kSpeedPidStreamMaxPeriodMs,
                               "[SPID][STREAM] Uso: spid.stream on [ms] | spid.stream off",
                               enabled,
                               periodTicks,
                               periodMs)) {
      return true;
    }

    if (!enabled) {
      g_speedPidStreamEnabled = false;
      sendTelnet("[SPID][STREAM] OFF");
      return true;
    }

    g_speedPidStreamPeriod = periodTicks;
    g_speedPidStreamEnabled = true;
    g_lastSpeedPidStreamTick = 0;

    String msg = "[SPID][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    reportSpeedPidStatus();
    return true;
  }

  if (command.equalsIgnoreCase("spid.help")) {
    sendTelnet(
        "Comandos: spid.set <kp> <ki> <kd> | spid.kp <v> | spid.ki <v> | spid.kd <v> | spid.ramp <mps2> | spid.minthrottle <pct> | spid.thslewup <pctps> | spid.thslewdown <pctps> | spid.minth.spd <mps> | spid.launchwin <ms> | spid.ff on|off | spid.ff.base0 <pct> | spid.ff.basemax <pct> | spid.ff.du <pct> | spid.ff.dd <pct> | spid.ff.minspd <mps> | spid.ff.grace <ms> | spid.iunwind <gain> | spid.dfilter <hz> | spid.max <mps> | spid.maxrev <mps> | spid.awx <scale> | spid.brakecap <pct> | spid.hys <mps> | spid.brakeslewup <pctps> | spid.brakeslewdown <pctps> | spid.brakehold <ms> | spid.brakedb <pct> | spid.target <signed_mps|off> | spid.save | spid.reset | spid.status | spid.stream on [ms] | spid.stream off");
    return true;
  }

  return false;
}

bool handleDriveCommand(const String& command, const String& args) {
  if (command.equalsIgnoreCase("drive.log")) {
    if (args.isEmpty()) {
      bool baseEnabled = false;
      bool pidTraceEnabled = false;
      TickType_t pidTracePeriodTicks = kDrivePidTraceDefaultPeriod;
      quadDriveGetLogEnabled(baseEnabled);
      quadDriveGetPidTraceConfig(pidTraceEnabled, pidTracePeriodTicks);
      String msg = "[DRIVE][LOG] base=";
      msg += baseEnabled ? "ON" : "OFF";
      msg += " pid=";
      msg += pidTraceEnabled ? "ON" : "OFF";
      msg += " period=";
      msg += static_cast<uint32_t>(pidTracePeriodTicks * portTICK_PERIOD_MS);
      msg += "ms";
      sendTelnet(msg);
      return true;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "on" || normalized == "1") {
      quadDriveSetLogEnabled(true);
      sendTelnet("[DRIVE][LOG] ON");
      return true;
    }
    if (normalized == "off" || normalized == "0") {
      quadDriveSetLogEnabled(false);
      sendTelnet("[DRIVE][LOG] OFF");
      return true;
    }

    if (normalized.startsWith("pid")) {
      String subArgs = args.substring(3);
      subArgs.trim();
      if (subArgs.isEmpty()) {
        bool pidTraceEnabled = false;
        TickType_t pidTracePeriodTicks = kDrivePidTraceDefaultPeriod;
        quadDriveGetPidTraceConfig(pidTraceEnabled, pidTracePeriodTicks);
        String msg = "[DRIVE][LOG][PID] ";
        msg += pidTraceEnabled ? "ON" : "OFF";
        msg += " periodo=";
        msg += static_cast<uint32_t>(pidTracePeriodTicks * portTICK_PERIOD_MS);
        msg += "ms";
        sendTelnet(msg);
        return true;
      }

      bool enabled = false;
      TickType_t periodTicks = kDrivePidTraceDefaultPeriod;
      int periodMs = static_cast<int>(kDrivePidTraceDefaultPeriod * portTICK_PERIOD_MS);
      if (!parseStreamToggleArgs(subArgs,
                                 periodMs,
                                 kDrivePidTraceMinPeriodMs,
                                 kDrivePidTraceMaxPeriodMs,
                                 "[DRIVE][LOG][PID] Uso: drive.log pid on [ms] | drive.log pid off",
                                 enabled,
                                 periodTicks,
                                 periodMs)) {
        return true;
      }

      if (!enabled) {
        quadDriveSetPidTraceEnabled(false, kDrivePidTraceDefaultPeriod);
        sendTelnet("[DRIVE][LOG][PID] OFF");
        return true;
      }

      quadDriveSetPidTraceEnabled(true, periodTicks);
      String msg = "[DRIVE][LOG][PID] ON periodo=";
      msg += periodMs;
      msg += "ms";
      sendTelnet(msg);
      return true;
    }

    sendTelnet(
        "[DRIVE][LOG] Uso: drive.log | drive.log on | drive.log off | drive.log pid on [ms] | drive.log pid off");
    return true;
  }

  if (command.equalsIgnoreCase("drive.pwm")) {
    if (args.isEmpty()) {
      reportDrivePwmOverrideStatus();
      return true;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      quadDriveSetPwmOverride(false, 0);
      sendTelnet("[DRIVE][PWM] OFF");
      return true;
    }

    int dutyPercent = -1;
    String dutyText = args;
    if (normalized.startsWith("on")) {
      dutyText = args.substring(2);
      dutyText.trim();
    }
    if (!parseIntArg(dutyText, dutyPercent)) {
      sendTelnet("[DRIVE][PWM] Uso: drive.pwm <0-100> | drive.pwm off");
      return true;
    }
    if (dutyPercent < 0 || dutyPercent > 100) {
      sendTelnet("[DRIVE][PWM] Valor fuera de rango (0..100)");
      return true;
    }

    QuadDriveDirection currentDirection = QuadDriveDirection::kForward;
    bool directionSwitching = false;
    bool relayEnergized = false;
    if (quadDriveGetDirectionStatus(currentDirection, directionSwitching, relayEnergized) &&
        directionSwitching && dutyPercent > 0) {
      sendTelnet("[DRIVE][PWM] Cambio de direccion en progreso; espera y reintenta");
      return true;
    }

    // Evitar un objetivo SPID latente al salir del modo PWM directo.
    quadDriveSetSpeedTargetOverride(false, 0.0f);
    if (!quadDriveSetPwmOverride(true, dutyPercent)) {
      sendTelnet("[DRIVE][PWM] No se pudo activar");
      return true;
    }

    String msg = "[DRIVE][PWM] ON duty=";
    msg += dutyPercent;
    msg += "%";
    sendTelnet(msg);
    return true;
  }

  if (command.equalsIgnoreCase("drive.dir")) {
    if (args.isEmpty()) {
      reportDriveDirectionStatus();
      return true;
    }

    String normalized = args;
    normalized.toLowerCase();
    normalized.trim();

    QuadDriveDirection requested = QuadDriveDirection::kForward;
    if (normalized == "fwd" || normalized == "forward") {
      requested = QuadDriveDirection::kForward;
    } else if (normalized == "rev" || normalized == "reverse") {
      requested = QuadDriveDirection::kReverse;
    } else {
      sendTelnet("[DRIVE][DIR] Uso: drive.dir <fwd|rev>");
      return true;
    }

    bool pwmEnabled = false;
    int pwmPercent = 0;
    quadDriveGetPwmOverride(pwmEnabled, pwmPercent);
    if (!pwmEnabled) {
      sendTelnet("[DRIVE][DIR] Reversa disponible solo en modo drive.pwm");
      return true;
    }

    if (!quadDriveSetDirectionOverride(requested)) {
      sendTelnet("[DRIVE][DIR] No se pudo aplicar solicitud");
      return true;
    }

    String msg = "[DRIVE][DIR] Solicitud ";
    msg += driveDirectionText(requested);
    msg += " aceptada";
    sendTelnet(msg);
    reportDriveDirectionStatus();
    return true;
  }

  if (command.equalsIgnoreCase("drive.rc.status")) {
    sendTelnet(buildDriveRcStatusMessage());
    return true;
  }

  if (command.equalsIgnoreCase("drive.rc.cal")) {
    if (args.isEmpty()) {
      bool enabled = true;
      quadDriveGetRcNeutralCalEnabled(enabled);
      String msg = "[DRIVE][RC][CAL] ";
      msg += enabled ? "ON" : "OFF";
      sendTelnet(msg);
      return true;
    }
    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "on" || normalized == "1") {
      quadDriveSetRcNeutralCalEnabled(true);
      sendTelnet("[DRIVE][RC][CAL] ON");
      return true;
    }
    if (normalized == "off" || normalized == "0") {
      quadDriveSetRcNeutralCalEnabled(false);
      sendTelnet("[DRIVE][RC][CAL] OFF");
      return true;
    }
    sendTelnet("[DRIVE][RC][CAL] Uso: drive.rc.cal on|off");
    return true;
  }

  if (command.equalsIgnoreCase("drive.rc.stream")) {
    if (args.isEmpty()) {
      String msg = "[DRIVE][RC][STREAM] ";
      msg += g_driveRcStreamEnabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += static_cast<uint32_t>(g_driveRcStreamPeriod * portTICK_PERIOD_MS);
      msg += "ms";
      sendTelnet(msg);
      return true;
    }

    bool enabled = false;
    TickType_t periodTicks = g_driveRcStreamPeriod;
    int periodMs = static_cast<int>(kDriveRcStreamDefaultPeriod * portTICK_PERIOD_MS);
    if (!parseStreamToggleArgs(args,
                               periodMs,
                               kDriveRcStreamMinPeriodMs,
                               kDriveRcStreamMaxPeriodMs,
                               "[DRIVE][RC][STREAM] Uso: drive.rc.stream on [ms] | drive.rc.stream off",
                               enabled,
                               periodTicks,
                               periodMs)) {
      return true;
    }

    if (!enabled) {
      g_driveRcStreamEnabled = false;
      sendTelnet("[DRIVE][RC][STREAM] OFF");
      return true;
    }

    g_driveRcStreamPeriod = periodTicks;
    g_driveRcStreamEnabled = true;
    String msg = "[DRIVE][RC][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    sendTelnet(buildDriveRcStatusMessage());
    return true;
  }

  return false;
}

void handleTelnetCommand(String line) {
  ParsedTelnetCommand parsed{};
  if (!parseCommandLine(line, parsed)) {
    return;
  }

  // Orden fijo de dispatch para preservar compatibilidad de comandos.
  if (handleSessionCommand(parsed.command, parsed.args) || handleSteerCommand(parsed.command, parsed.args) ||
      handlePidCommand(parsed.command, parsed.args) ||
      handleSpidCommand(parsed.command, parsed.args) || handleDriveCommand(parsed.command, parsed.args) ||
      handleCommsCommand(parsed.command, parsed.args) || handleSpeedCommand(parsed.command, parsed.args) ||
      handleSysCommand(parsed.command, parsed.args) || handleNetCommand(parsed.command, parsed.args)) {
    return;
  }

  sendTelnet("[STEER] Comando desconocido: " + parsed.line);
}

void handleTelnetClient() {
  if (g_telnetClient && g_telnetClient.connected() && g_telnetLastActivityTick != 0) {
    const TickType_t now = xTaskGetTickCount();
    if ((now - g_telnetLastActivityTick) >= kTelnetIdleTimeout) {
      g_telnetClient.println("Sesion cerrada por inactividad");
      resetTelnetSession();
    }
  }

  if (g_telnetServer.hasClient()) {
    WiFiClient incoming = g_telnetServer.available();
    if (incoming) {
      incoming.setNoDelay(true);
      if (g_telnetClient && g_telnetClient.connected()) {
        g_telnetClient.println("Sesion cerrada: nuevo cliente conectado");
      }
      resetTelnetSession();
      openTelnetSession(incoming);
    }
  }

  if (g_telnetClient && !g_telnetClient.connected()) {
    resetTelnetSession();
  }
}

void processTelnetInput() {
  while (g_telnetClient && g_telnetClient.connected() && g_telnetClient.available() > 0) {
    const char c = static_cast<char>(g_telnetClient.read());
    markTelnetActivity();
    if (c == '\r') {
      continue;
    }
    if (c == '\b' || c == 0x7f) {  // Backspace/Delete
      if (!g_telnetCommandBuffer.isEmpty()) {
        g_telnetCommandBuffer.remove(g_telnetCommandBuffer.length() - 1);
      }
      continue;
    }
    if (c == '\n') {
      handleTelnetCommand(g_telnetCommandBuffer);
      g_telnetCommandBuffer = "";
      markTelnetActivity();
      continue;
    }
    const unsigned char uc = static_cast<unsigned char>(c);
    if (uc < 0x20 || uc >= 0x7F) {
      continue;
    }
    if (g_telnetCommandBuffer.length() >= 120) {
      continue;
    }
    g_telnetCommandBuffer += c;
  }
}
}  // namespace


/* ========= Wi-Fi ========= */
void InicializaWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(OTA_HOSTNAME);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASS);

  const uint32_t timeoutMs = static_cast<uint32_t>(WIFI_STA_CONNECT_TIMEOUT_MS);
  const uint32_t startMs = millis();
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    delay(250);
  }

  if (WiFi.status() == WL_CONNECTED) {
    updateNetworkState(NetworkMode::kSta, WIFI_STA_SSID, WiFi.localIP());
    return;
  }

  WiFi.disconnect(true, true);
  WiFi.mode(WIFI_AP);
  if (strlen(WIFI_AP_PASS) == 0) {
    WiFi.softAP(WIFI_AP_SSID);
  } else {
    WiFi.softAP(WIFI_AP_SSID, WIFI_AP_PASS);
  }
  updateNetworkState(NetworkMode::kAp, WIFI_AP_SSID, WiFi.softAPIP());
}

/* ========= OTA ========= */
void InicializaOTA() {
  ArduinoOTA.setHostname(OTA_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() { EnviarMensajeTelnet("OTA iniciado"); });
  ArduinoOTA.onEnd([]() { EnviarMensajeTelnet("OTA finalizado"); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    const unsigned int percent = (total > 0U) ? ((progress * 100U) / total) : 0U;
    EnviarMensajeTelnet("Progreso OTA: " + String(percent) + "%");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    String msg = "Error OTA [" + String(error) + "]: ";
    if (error == OTA_AUTH_ERROR) {
      msg += "Auth Failed";
    } else if (error == OTA_BEGIN_ERROR) {
      msg += "Begin Failed";
    } else if (error == OTA_CONNECT_ERROR) {
      msg += "Connect Failed";
    } else if (error == OTA_RECEIVE_ERROR) {
      msg += "Receive Failed";
    } else if (error == OTA_END_ERROR) {
      msg += "End Failed";
    } else {
      msg += "Unknown";
    }
    EnviarMensajeTelnet(msg);
  });
  ArduinoOTA.begin();
}

void InicializaTelnet() {  // Inicia Telnet en puerto 23
  if (g_telnetLogQueue == nullptr) {
    g_telnetLogQueue = xQueueCreate(kTelnetLogQueueLength, sizeof(TelnetLogMessage));
  }
  portENTER_CRITICAL(&g_telnetStatsMux);
  g_telnetLogDropCount = 0;
  portEXIT_CRITICAL(&g_telnetStatsMux);
  g_telnetServer.begin();
  g_telnetServer.setNoDelay(true);
  delay(1000);  // Esperar a que se inicie el servidor
}

void EnviarMensajeTelnet(const String& txt) {  // Envia mensaje por Telnet
  if (isCurrentTaskOtaTelnet()) {
    sendTelnetDirect(txt);
    return;
  }
  (void)enqueueTelnetLog(txt);
}

void serialIf(bool enabled, const String& message) {
  (void)enabled;
  (void)message;
}

void telnetIf(bool enabled, const String& message) {
  if (!enabled) {
    return;
  }
  if (!shouldForwardLog(message)) {
    return;
  }
  EnviarMensajeTelnet(message);
}

void broadcastIf(bool enabled, const String& message) {
  serialIf(enabled, message);
  telnetIf(enabled, message);
}

void taskOtaTelnet(void* parameter) {
  OtaTelnetTaskConfig* config = static_cast<OtaTelnetTaskConfig*>(parameter);
  g_otaTaskHandleRuntime = xTaskGetCurrentTaskHandle();
  const TickType_t period = (config != nullptr) ? config->taskPeriod : pdMS_TO_TICKS(20);
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(period * portTICK_PERIOD_MS * 1000U);
  const TickType_t heartbeat = (config != nullptr) ? config->heartbeatInterval : pdMS_TO_TICKS(5000);
  const bool logHeartbeat = (config != nullptr) ? config->logHeartbeat : false;
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastHeartbeat = lastWake;
  int64_t lastIterationStartUs = esp_timer_get_time();
  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;
    ArduinoOTA.handle();
    handleTelnetClient();
    processTelnetInput();
    drainTelnetLogQueue(kTelnetDrainMaxMessagesPerCycle);
    if (g_speedStreamEnabled && g_telnetClient && g_telnetClient.connected()) {
      const TickType_t now = xTaskGetTickCount();
      if (g_lastSpeedStreamTick == 0 || (now - g_lastSpeedStreamTick) >= g_speedStreamPeriod) {
        sendTelnet(buildSpeedStatusMessage());
        g_lastSpeedStreamTick = now;
      }
    }
    if (g_speedPidStreamEnabled && g_telnetClient && g_telnetClient.connected()) {
      const TickType_t now = xTaskGetTickCount();
      if (g_lastSpeedPidStreamTick == 0 || (now - g_lastSpeedPidStreamTick) >= g_speedPidStreamPeriod) {
        reportSpeedPidStatus();
        g_lastSpeedPidStreamTick = now;
      }
    }
    if (g_driveRcStreamEnabled && g_telnetClient && g_telnetClient.connected()) {
      const TickType_t now = xTaskGetTickCount();
      if (g_lastDriveRcStreamTick == 0 || (now - g_lastDriveRcStreamTick) >= g_driveRcStreamPeriod) {
        sendTelnet(buildDriveRcStatusMessage());
        g_lastDriveRcStreamTick = now;
      }
    }
    if (g_pidStreamEnabled && g_telnetClient && g_telnetClient.connected()) {
      const TickType_t now = xTaskGetTickCount();
      if (g_lastPidStreamTick == 0 || (now - g_lastPidStreamTick) >= g_pidStreamPeriod) {
        reportPidRuntimeStatus();
        g_lastPidStreamTick = now;
      }
    }
    if (g_telnetClient && g_telnetClient.connected()) {
      const TickType_t now = xTaskGetTickCount();
      if (systemDiagShouldEmitJitter(now)) {
        sendTelnet(buildSystemJitterSummary());
      }
    }
    if (logHeartbeat && (xTaskGetTickCount() - lastHeartbeat >= heartbeat)) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kOtaTelnet, cycleUs, expectedPeriodUs, overrun, false);
    vTaskDelayUntil(&lastWake, period);
  }
}
