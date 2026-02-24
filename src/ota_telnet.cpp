#include "ota_telnet.h"

#include <WiFi.h>
#include <ArduinoOTA.h>

#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "steering_calibration.h"
#include "pi_comms.h"
#include "pid.h"
#include "hall_speed.h"
#include "speed_pid.h"
#include "quad_functions.h"

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
constexpr TickType_t kTelnetIdleTimeout = pdMS_TO_TICKS(300000);
constexpr bool kFocusedControlLogsOnly = true;

enum class NetworkMode {
  kUnknown,
  kSta,
  kAp,
};

WiFiServer g_telnetServer(kTelnetPort);
WiFiClient g_telnetClient;
String g_telnetCommandBuffer;
NetworkMode g_networkMode = NetworkMode::kUnknown;
String g_networkSsid;
IPAddress g_networkIp;
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
  g_telnetCommandBuffer = "";
  g_speedStreamEnabled = false;
  g_lastSpeedStreamTick = 0;
  g_pidStreamEnabled = false;
  g_lastPidStreamTick = 0;
  g_speedPidStreamEnabled = false;
  g_lastSpeedPidStreamTick = 0;
  g_driveRcStreamEnabled = false;
  g_lastDriveRcStreamTick = 0;
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
      "Comandos: steer.help | pid.help | spid.help | comms.status | speed.status | speed.reset | speed.stream | speed.uart | pid.stream | spid.stream | spid.target | drive.log | drive.rc.status | drive.rc.stream | net.status");
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
  msg += "%}";
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
  msg += " fresh=";
  msg += snapshot.rcFresh ? "Y" : "N";
  msg += " ageMs=";
  msg += snapshot.snapshotAgeMs;
  msg += " brake=";
  msg += snapshot.rcManualBrakeActive ? "Y" : "N";
  msg += " elig=";
  msg += snapshot.rcSpeedPidEligible ? "Y" : "N";
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
  msg += " speed=";
  msg += String(snapshot.speedKmh, 2);
  msg += "km/h ";
  msg += String(snapshot.speedMps, 2);
  msg += "m/s rpm=";
  msg += String(snapshot.motorRpm, 1);
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
  msg += " poles=";
  msg += cfg.motorPoles;
  msg += " red=";
  msg += String(cfg.gearReduction, 2);
  msg += " wheelM=";
  msg += String(cfg.wheelDiameterM, 3);
  msg += "}";
  return msg;
}

void handleTelnetCommand(String line) {
  line.trim();
  if (line.isEmpty()) {
    return;
  }

  int spaceIndex = line.indexOf(' ');
  String command = (spaceIndex < 0) ? line : line.substring(0, spaceIndex);
  String args = (spaceIndex < 0) ? "" : line.substring(spaceIndex + 1);
  args.trim();

  if (command.equalsIgnoreCase("steer.calibrate")) {
    steeringCalibrationRequest();
    sendTelnet("[STEER] Calibracion solicitada");
    return;
  }

  if (command.equalsIgnoreCase("steer.offset")) {
    if (args.isEmpty()) {
      reportSteeringStatus();
      return;
    }
    float offset = 0.0f;
    if (!parseFloatArg(args, offset)) {
      sendTelnet("[STEER] Offset invalido, usa grados (ej: steer.offset -2.5)");
      return;
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
    return;
  }

  if (command.equalsIgnoreCase("steer.reset") || command.equalsIgnoreCase("steer.clear")) {
    steeringCalibrationReset(true);
    sendTelnet("[STEER] Calibracion reseteada a valores por defecto (NVS borrado)");
    reportSteeringStatus();
    return;
  }

  if (command.equalsIgnoreCase("steer.status")) {
    reportSteeringStatus();
    return;
  }

  if (command.equalsIgnoreCase("steer.help")) {
    sendTelnet("Comandos: steer.calibrate | steer.offset <deg> | steer.reset | steer.status");
    return;
  }

  if (command.equalsIgnoreCase("pid.set")) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!parseFloatTriplet(args, kp, ki, kd)) {
      sendTelnet("[PID] Uso: pid.set <kp> <ki> <kd>");
      return;
    }
    if (!pidSetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    reportPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("pid.kp")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Kp invalido (ej: pid.kp 2.5)");
      return;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!pidGetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    if (!pidSetTunings(value, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    reportPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("pid.ki")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Ki invalido (ej: pid.ki 0.5)");
      return;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!pidGetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    if (!pidSetTunings(kp, value, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    reportPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("pid.kd")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Kd invalido (ej: pid.kd 0.1)");
      return;
    }
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!pidGetTunings(kp, ki, kd)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    if (!pidSetTunings(kp, ki, value)) {
      sendTelnet("[PID] Controlador no registrado");
      return;
    }
    reportPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("pid.status")) {
    reportPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("pid.help")) {
    sendTelnet(
        "Comandos: pid.set <kp> <ki> <kd> | pid.kp <v> | pid.ki <v> | pid.kd <v> | pid.deadband <pct> | pid.minactive <pct> | pid.status | pid.stream on [ms] | pid.stream off");
    return;
  }

  if (command.equalsIgnoreCase("pid.deadband")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] Deadband invalido (ej: pid.deadband 0.3)");
      return;
    }
    if (!pidSetDeadband(value)) {
      sendTelnet("[PID] Configuracion PID no registrada");
      return;
    }
    reportPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("pid.minactive")) {
    if (args.isEmpty()) {
      reportPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[PID] MinActive invalido (ej: pid.minactive 10)");
      return;
    }
    if (!pidSetMinActive(value)) {
      sendTelnet("[PID] Configuracion PID no registrada");
      return;
    }
    reportPidStatus();
    return;
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
      return;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      g_pidStreamEnabled = false;
      sendTelnet("[PID][STREAM] OFF");
      return;
    }

    int periodMs = static_cast<int>(g_pidStreamPeriod * portTICK_PERIOD_MS);
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
        sendTelnet("[PID][STREAM] Uso: pid.stream on [ms] | pid.stream off");
        return;
      }
      if (parsedPeriod < kPidStreamMinPeriodMs) {
        parsedPeriod = kPidStreamMinPeriodMs;
      } else if (parsedPeriod > kPidStreamMaxPeriodMs) {
        parsedPeriod = kPidStreamMaxPeriodMs;
      }
      periodMs = parsedPeriod;
    }

    TickType_t periodTicks = pdMS_TO_TICKS(periodMs);
    if (periodTicks == 0) {
      periodTicks = 1;
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
    return;
  }

  if (command.equalsIgnoreCase("spid.set")) {
    float kp = 0.0f;
    float ki = 0.0f;
    float kd = 0.0f;
    if (!parseFloatTriplet(args, kp, ki, kd)) {
      sendTelnet("[SPID] Uso: spid.set <kp> <ki> <kd>");
      return;
    }
    SpeedPidTunings tunings{kp, ki, kd};
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Tunings invalidos o controlador no inicializado");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.kp")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Kp invalido (ej: spid.kp 10.0)");
      return;
    }
    SpeedPidTunings tunings{};
    if (!speedPidGetTunings(tunings)) {
      sendTelnet("[SPID] Controlador no inicializado");
      return;
    }
    tunings.kp = value;
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Kp fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ki")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Ki invalido (ej: spid.ki 2.0)");
      return;
    }
    SpeedPidTunings tunings{};
    if (!speedPidGetTunings(tunings)) {
      sendTelnet("[SPID] Controlador no inicializado");
      return;
    }
    tunings.ki = value;
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Ki fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.kd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Kd invalido (ej: spid.kd 0.1)");
      return;
    }
    SpeedPidTunings tunings{};
    if (!speedPidGetTunings(tunings)) {
      sendTelnet("[SPID] Controlador no inicializado");
      return;
    }
    tunings.kd = value;
    if (!speedPidSetTunings(tunings)) {
      sendTelnet("[SPID] Kd fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ramp")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Ramp invalido (ej: spid.ramp 2.0)");
      return;
    }
    if (!speedPidSetRampRateMps2(value)) {
      sendTelnet("[SPID] Ramp fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.minthrottle")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] MinThrottle invalido (ej: spid.minthrottle 90)");
      return;
    }
    if (!speedPidSetMinThrottlePercent(value)) {
      sendTelnet("[SPID] MinThrottle fuera de rango (0..100)");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.thslewup")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] ThrottleSlewUp invalido (ej: spid.thslewup 30)");
      return;
    }
    if (!speedPidSetThrottleSlewUpPctPerSec(value)) {
      sendTelnet("[SPID] ThrottleSlewUp fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.thslewdown")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] ThrottleSlewDown invalido (ej: spid.thslewdown 45)");
      return;
    }
    if (!speedPidSetThrottleSlewDownPctPerSec(value)) {
      sendTelnet("[SPID] ThrottleSlewDown fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.minth.spd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] MinThrottleAssist speed invalido (ej: spid.minth.spd 0.35)");
      return;
    }
    if (!speedPidSetMinThrottleAssistMaxSpeedMps(value)) {
      sendTelnet("[SPID] MinThrottleAssist speed fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.launchwin")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    int value = 0;
    if (!parseIntArg(args, value)) {
      sendTelnet("[SPID] LaunchWindow invalido (ej: spid.launchwin 1200)");
      return;
    }
    if (value < 0 || value > 65535) {
      sendTelnet("[SPID] LaunchWindow fuera de rango");
      return;
    }
    if (!speedPidSetLaunchAssistWindowMs(static_cast<uint16_t>(value))) {
      sendTelnet("[SPID] LaunchWindow fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff") || command.equalsIgnoreCase("spid.ff.status")) {
    if (command.equalsIgnoreCase("spid.ff.status") || args.isEmpty()) {
      reportSpeedPidStatus();
      return;
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
      return;
    }
    if (!speedPidSetThrottleBaseEnable(enable)) {
      sendTelnet("[SPID] Error aplicando ff on/off");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff.base0")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF base0 invalido (ej: spid.ff.base0 0)");
      return;
    }
    if (!speedPidSetThrottleBaseAtZeroMpsPercent(value)) {
      sendTelnet("[SPID] FF base0 fuera de rango (0..100)");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff.basemax")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF basemax invalido (ej: spid.ff.basemax 55)");
      return;
    }
    if (!speedPidSetThrottleBaseAtMaxSpeedPercent(value)) {
      sendTelnet("[SPID] FF basemax fuera de rango (0..100)");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff.du")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF delta up invalido (ej: spid.ff.du 35)");
      return;
    }
    if (!speedPidSetThrottleBasePidDeltaUpMaxPercent(value)) {
      sendTelnet("[SPID] FF delta up fuera de rango (0..100)");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff.dd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF delta down invalido (ej: spid.ff.dd 45)");
      return;
    }
    if (!speedPidSetThrottleBasePidDeltaDownMaxPercent(value)) {
      sendTelnet("[SPID] FF delta down fuera de rango (0..100)");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff.minspd")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] FF minspd invalido (ej: spid.ff.minspd 0.10)");
      return;
    }
    if (!speedPidSetThrottleBaseActivationMinMps(value)) {
      sendTelnet("[SPID] FF minspd fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.ff.grace")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    int value = 0;
    if (!parseIntArg(args, value)) {
      sendTelnet("[SPID] FF grace invalido (ej: spid.ff.grace 1200)");
      return;
    }
    if (value < 0 || value > 65535) {
      sendTelnet("[SPID] FF grace fuera de rango");
      return;
    }
    if (!speedPidSetFeedbackLaunchGraceMs(static_cast<uint16_t>(value))) {
      sendTelnet("[SPID] FF grace fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.iunwind")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] IntegratorUnwind invalido (ej: spid.iunwind 0.35)");
      return;
    }
    if (!speedPidSetIntegratorUnwindGain(value)) {
      sendTelnet("[SPID] IntegratorUnwind fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.dfilter")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] DerivativeFilter invalido (ej: spid.dfilter 3.0)");
      return;
    }
    if (!speedPidSetDerivativeFilterHz(value)) {
      sendTelnet("[SPID] DerivativeFilter fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.max")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Max invalido (ej: spid.max 4.17)");
      return;
    }
    if (!speedPidSetMaxSpeedMps(value)) {
      sendTelnet("[SPID] Max fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.brakecap")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeCap invalido (ej: spid.brakecap 30)");
      return;
    }
    if (!speedPidSetOverspeedBrakeMaxPercent(value)) {
      sendTelnet("[SPID] BrakeCap fuera de rango (0..100)");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.hys")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] Hysteresis invalido (ej: spid.hys 0.3)");
      return;
    }
    if (!speedPidSetOverspeedReleaseHysteresisMps(value)) {
      sendTelnet("[SPID] Hysteresis fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.brakeslewup")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeSlewUp invalido (ej: spid.brakeslewup 35)");
      return;
    }
    if (!speedPidSetOverspeedBrakeSlewUpPctPerSec(value)) {
      sendTelnet("[SPID] BrakeSlewUp fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.brakeslewdown")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeSlewDown invalido (ej: spid.brakeslewdown 55)");
      return;
    }
    if (!speedPidSetOverspeedBrakeSlewDownPctPerSec(value)) {
      sendTelnet("[SPID] BrakeSlewDown fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.brakehold")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    int value = 0;
    if (!parseIntArg(args, value)) {
      sendTelnet("[SPID] BrakeHold invalido (ej: spid.brakehold 200)");
      return;
    }
    if (value < 0 || value > 65535) {
      sendTelnet("[SPID] BrakeHold fuera de rango");
      return;
    }
    if (!speedPidSetOverspeedBrakeHoldMs(static_cast<uint16_t>(value))) {
      sendTelnet("[SPID] BrakeHold fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.brakedb")) {
    if (args.isEmpty()) {
      reportSpeedPidStatus();
      return;
    }
    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID] BrakeDeadband invalido (ej: spid.brakedb 3)");
      return;
    }
    if (!speedPidSetOverspeedBrakeDeadbandPercent(value)) {
      sendTelnet("[SPID] BrakeDeadband fuera de rango");
      return;
    }
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.target")) {
    if (args.isEmpty()) {
      reportSpeedPidTargetStatus();
      return;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      if (!quadDriveSetSpeedTargetOverride(false, 0.0f)) {
        sendTelnet("[SPID][TARGET] No se pudo desactivar override");
        return;
      }
      reportSpeedPidTargetStatus();
      return;
    }

    float value = 0.0f;
    if (!parseFloatArg(args, value)) {
      sendTelnet("[SPID][TARGET] Valor invalido (ej: spid.target 1.5 | spid.target off)");
      return;
    }
    if (!quadDriveSetSpeedTargetOverride(true, value)) {
      String msg = "[SPID][TARGET] Fuera de rango (0..";
      msg += String(speedPidGetMaxSpeedMps(), 2);
      msg += " m/s)";
      sendTelnet(msg);
      return;
    }
    reportSpeedPidTargetStatus();
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.save")) {
    if (!speedPidSaveToNvs()) {
      sendTelnet("[SPID] Error guardando en NVS");
      return;
    }
    sendTelnet("[SPID] Configuracion guardada en NVS");
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.reset")) {
    if (!speedPidResetToDefaults(true)) {
      sendTelnet("[SPID] Error restaurando defaults");
      return;
    }
    sendTelnet("[SPID] Defaults restaurados y persistidos");
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.status")) {
    reportSpeedPidStatus();
    return;
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
      return;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      g_speedPidStreamEnabled = false;
      sendTelnet("[SPID][STREAM] OFF");
      return;
    }

    int periodMs = static_cast<int>(g_speedPidStreamPeriod * portTICK_PERIOD_MS);
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
        sendTelnet("[SPID][STREAM] Uso: spid.stream on [ms] | spid.stream off");
        return;
      }
      if (parsedPeriod < kSpeedPidStreamMinPeriodMs) {
        parsedPeriod = kSpeedPidStreamMinPeriodMs;
      } else if (parsedPeriod > kSpeedPidStreamMaxPeriodMs) {
        parsedPeriod = kSpeedPidStreamMaxPeriodMs;
      }
      periodMs = parsedPeriod;
    }

    TickType_t periodTicks = pdMS_TO_TICKS(periodMs);
    if (periodTicks == 0) {
      periodTicks = 1;
    }

    g_speedPidStreamPeriod = periodTicks;
    g_speedPidStreamEnabled = true;
    g_lastSpeedPidStreamTick = 0;

    String msg = "[SPID][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    reportSpeedPidStatus();
    return;
  }

  if (command.equalsIgnoreCase("spid.help")) {
    sendTelnet(
        "Comandos: spid.set <kp> <ki> <kd> | spid.kp <v> | spid.ki <v> | spid.kd <v> | spid.ramp <mps2> | spid.minthrottle <pct> | spid.thslewup <pctps> | spid.thslewdown <pctps> | spid.minth.spd <mps> | spid.launchwin <ms> | spid.ff on|off | spid.ff.base0 <pct> | spid.ff.basemax <pct> | spid.ff.du <pct> | spid.ff.dd <pct> | spid.ff.minspd <mps> | spid.ff.grace <ms> | spid.iunwind <gain> | spid.dfilter <hz> | spid.max <mps> | spid.brakecap <pct> | spid.hys <mps> | spid.brakeslewup <pctps> | spid.brakeslewdown <pctps> | spid.brakehold <ms> | spid.brakedb <pct> | spid.target <mps|off> | spid.save | spid.reset | spid.status | spid.stream on [ms] | spid.stream off");
    return;
  }

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
      return;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "on" || normalized == "1") {
      quadDriveSetLogEnabled(true);
      sendTelnet("[DRIVE][LOG] ON");
      return;
    }
    if (normalized == "off" || normalized == "0") {
      quadDriveSetLogEnabled(false);
      sendTelnet("[DRIVE][LOG] OFF");
      return;
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
        return;
      }

      String subNormalized = subArgs;
      subNormalized.toLowerCase();
      if (subNormalized == "off" || subNormalized == "0" || subNormalized == "stop") {
        quadDriveSetPidTraceEnabled(false, kDrivePidTraceDefaultPeriod);
        sendTelnet("[DRIVE][LOG][PID] OFF");
        return;
      }

      int periodMs = static_cast<int>(kDrivePidTraceDefaultPeriod * portTICK_PERIOD_MS);
      bool hasPeriodArg = false;
      String periodText;

      if (subNormalized.startsWith("on")) {
        periodText = subArgs.substring(2);
        periodText.trim();
        hasPeriodArg = !periodText.isEmpty();
      } else {
        periodText = subArgs;
        hasPeriodArg = true;
      }

      if (hasPeriodArg) {
        int parsedPeriod = 0;
        if (!parseIntArg(periodText, parsedPeriod)) {
          sendTelnet("[DRIVE][LOG][PID] Uso: drive.log pid on [ms] | drive.log pid off");
          return;
        }
        if (parsedPeriod < kDrivePidTraceMinPeriodMs) {
          parsedPeriod = kDrivePidTraceMinPeriodMs;
        } else if (parsedPeriod > kDrivePidTraceMaxPeriodMs) {
          parsedPeriod = kDrivePidTraceMaxPeriodMs;
        }
        periodMs = parsedPeriod;
      }

      TickType_t periodTicks = pdMS_TO_TICKS(periodMs);
      if (periodTicks == 0) {
        periodTicks = 1;
      }
      quadDriveSetPidTraceEnabled(true, periodTicks);
      String msg = "[DRIVE][LOG][PID] ON periodo=";
      msg += periodMs;
      msg += "ms";
      sendTelnet(msg);
      return;
    }

    sendTelnet(
        "[DRIVE][LOG] Uso: drive.log | drive.log on | drive.log off | drive.log pid on [ms] | drive.log pid off");
    return;
  }

  if (command.equalsIgnoreCase("drive.rc.status")) {
    sendTelnet(buildDriveRcStatusMessage());
    return;
  }

  if (command.equalsIgnoreCase("drive.rc.cal")) {
    if (args.isEmpty()) {
      bool enabled = true;
      quadDriveGetRcNeutralCalEnabled(enabled);
      String msg = "[DRIVE][RC][CAL] ";
      msg += enabled ? "ON" : "OFF";
      sendTelnet(msg);
      return;
    }
    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "on" || normalized == "1") {
      quadDriveSetRcNeutralCalEnabled(true);
      sendTelnet("[DRIVE][RC][CAL] ON");
      return;
    }
    if (normalized == "off" || normalized == "0") {
      quadDriveSetRcNeutralCalEnabled(false);
      sendTelnet("[DRIVE][RC][CAL] OFF");
      return;
    }
    sendTelnet("[DRIVE][RC][CAL] Uso: drive.rc.cal on|off");
    return;
  }

  if (command.equalsIgnoreCase("drive.rc.stream")) {
    if (args.isEmpty()) {
      String msg = "[DRIVE][RC][STREAM] ";
      msg += g_driveRcStreamEnabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += static_cast<uint32_t>(g_driveRcStreamPeriod * portTICK_PERIOD_MS);
      msg += "ms";
      sendTelnet(msg);
      return;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      g_driveRcStreamEnabled = false;
      sendTelnet("[DRIVE][RC][STREAM] OFF");
      return;
    }

    int periodMs = static_cast<int>(kDriveRcStreamDefaultPeriod * portTICK_PERIOD_MS);
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
        sendTelnet("[DRIVE][RC][STREAM] Uso: drive.rc.stream on [ms] | drive.rc.stream off");
        return;
      }
      if (parsedPeriod < kDriveRcStreamMinPeriodMs) {
        parsedPeriod = kDriveRcStreamMinPeriodMs;
      } else if (parsedPeriod > kDriveRcStreamMaxPeriodMs) {
        parsedPeriod = kDriveRcStreamMaxPeriodMs;
      }
      periodMs = parsedPeriod;
    }

    g_driveRcStreamPeriod = pdMS_TO_TICKS(periodMs);
    if (g_driveRcStreamPeriod == 0) {
      g_driveRcStreamPeriod = 1;
    }
    g_driveRcStreamEnabled = true;
    String msg = "[DRIVE][RC][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    sendTelnet(buildDriveRcStatusMessage());
    return;
  }

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
    msg += " accelRaw=";
    msg += snapshot.accelRaw;
    msg += " accelEff=";
    msg += snapshot.accelEffective;
    msg += " brake=";
    msg += snapshot.brake;
    msg += " estop=";
    msg += snapshot.estop ? "Y" : "N";
    msg += " drive=";
    msg += snapshot.driveEnabled ? "Y" : "N";
    msg += " reverse{allow=";
    msg += snapshot.allowReverse ? "Y" : "N";
    msg += " wants=";
    msg += snapshot.wantsReverse ? "Y" : "N";
    msg += " req=";
    msg += snapshot.reverseRequestActive ? "Y" : "N";
    msg += " wait=";
    msg += snapshot.reverseAwaitingGrant ? "Y" : "N";
    msg += " granted=";
    msg += snapshot.reverseGranted ? "Y" : "N";
    msg += "}";
    msg += " ok=";
    msg += snapshot.framesOk;
    msg += " crcErr=";
    msg += snapshot.framesCrcError;
    msg += " malformed=";
    msg += snapshot.framesMalformed;
    sendTelnet(msg);
    return;
  }

  if (command.equalsIgnoreCase("comms.reset")) {
    piCommsResetStats();
    sendTelnet("[PI][STATUS] Contadores reseteados");
    return;
  }

  if (command.equalsIgnoreCase("speed.status")) {
    sendTelnet(buildSpeedStatusMessage());
    return;
  }

  if (command.equalsIgnoreCase("speed.reset")) {
    hallSpeedResetStats();
    sendTelnet("[SPD][STATUS] Contadores Hall reseteados");
    sendTelnet(buildSpeedStatusMessage());
    return;
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
      return;
    }

    String normalized = args;
    normalized.toLowerCase();
    if (normalized == "off" || normalized == "0" || normalized == "stop") {
      g_speedStreamEnabled = false;
      sendTelnet("[SPD][STREAM] OFF");
      return;
    }

    int periodMs = static_cast<int>(g_speedStreamPeriod * portTICK_PERIOD_MS);
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
        sendTelnet("[SPD][STREAM] Uso: speed.stream on [ms] | speed.stream off");
        return;
      }
      if (parsedPeriod < kSpeedStreamMinPeriodMs) {
        parsedPeriod = kSpeedStreamMinPeriodMs;
      } else if (parsedPeriod > kSpeedStreamMaxPeriodMs) {
        parsedPeriod = kSpeedStreamMaxPeriodMs;
      }
      periodMs = parsedPeriod;
    }

    TickType_t periodTicks = pdMS_TO_TICKS(periodMs);
    if (periodTicks == 0) {
      periodTicks = 1;
    }

    g_speedStreamPeriod = periodTicks;
    g_speedStreamEnabled = true;
    g_lastSpeedStreamTick = 0;

    String msg = "[SPD][STREAM] ON periodo=";
    msg += periodMs;
    msg += "ms";
    sendTelnet(msg);
    sendTelnet(buildSpeedStatusMessage());
    return;
  }

  if (command.equalsIgnoreCase("speed.uart")) {
    (void)args;
    sendTelnet("[SPD][UART] N/A source=hall");
    return;
  }

  if (command.equalsIgnoreCase("net.status")) {
    reportNetworkStatus();
    return;
  }

  sendTelnet("[STEER] Comando desconocido: " + line);
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
  g_telnetServer.begin();
  g_telnetServer.setNoDelay(true);
  delay(1000);  // Esperar a que se inicie el servidor
}

void EnviarMensajeTelnet(const String& txt) {  // Envia mensaje por Telnet
  if (g_telnetClient && g_telnetClient.connected()) {
    g_telnetClient.println(txt);
  }
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
  const TickType_t period = (config != nullptr) ? config->taskPeriod : pdMS_TO_TICKS(20);
  const TickType_t heartbeat = (config != nullptr) ? config->heartbeatInterval : pdMS_TO_TICKS(5000);
  const bool logHeartbeat = (config != nullptr) ? config->logHeartbeat : false;
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastHeartbeat = lastWake;
  for (;;) {
    ArduinoOTA.handle();
    handleTelnetClient();
    processTelnetInput();
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
    if (logHeartbeat && (xTaskGetTickCount() - lastHeartbeat >= heartbeat)) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
