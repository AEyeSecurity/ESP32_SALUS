#include "ota_telnet.h"

#include <WiFi.h>
#include <ArduinoOTA.h>

#include <ctype.h>
#include <string.h>
#include <stdlib.h>

#include "steering_calibration.h"
#include "pi_comms.h"
#include "pid.h"
#include "speed_meter.h"

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
constexpr TickType_t kTelnetIdleTimeout = pdMS_TO_TICKS(300000);

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
      "Comandos: steer.help | pid.help | comms.status | speed.status | speed.reset | speed.stream | speed.uart | net.status");
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
  long parsed = strtol(raw, &endPtr, 10);
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

bool parseBoolArg(const String& text, bool& valueOut) {
  String normalized = text;
  normalized.trim();
  normalized.toLowerCase();
  if (normalized == "1" || normalized == "on" || normalized == "true" || normalized == "yes") {
    valueOut = true;
    return true;
  }
  if (normalized == "0" || normalized == "off" || normalized == "false" || normalized == "no") {
    valueOut = false;
    return true;
  }
  return false;
}

void appendHexByte(String& msg, uint8_t value) {
  msg += "0x";
  if (value < 0x10) {
    msg += "0";
  }
  msg += String(value, HEX);
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

String buildSpeedStatusMessage() {
  SpeedMeterSnapshot snapshot{};
  speedMeterGetSnapshot(snapshot);
  SpeedMeterConfig cfg{};
  speedMeterGetConfig(cfg);
  String msg = "[SPD][STATUS] driver=";
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
  msg += " speed=";
  msg += snapshot.speedKmh;
  msg += "km/h";
  msg += " conf=";
  msg += snapshot.confidence;
  msg += " throttle=";
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
  msg += " uart{baud=";
  msg += cfg.baudRate;
  msg += " inv=";
  msg += cfg.invertRx ? "ON" : "OFF";
  msg += " bytes=";
  msg += snapshot.uartBytesRx;
  msg += " dataEvt=";
  msg += snapshot.uartEventsData;
  msg += " fe=";
  msg += snapshot.uartEventsFrameErr;
  msg += " pe=";
  msg += snapshot.uartEventsParityErr;
  msg += " brk=";
  msg += snapshot.uartEventsBreak;
  msg += " fifo=";
  msg += snapshot.uartEventsFifoOvf;
  msg += " full=";
  msg += snapshot.uartEventsBufferFull;
  msg += "}";
  msg += " raw{len=";
  msg += snapshot.lastFrameLen;
  msg += " b8=";
  appendHexByte(msg, snapshot.lastB8);
  msg += " b16=";
  appendHexByte(msg, snapshot.lastB16);
  msg += " b17=";
  appendHexByte(msg, snapshot.lastB17);
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
    sendTelnet("Comandos: pid.set <kp> <ki> <kd> | pid.kp <v> | pid.ki <v> | pid.kd <v> | pid.deadband <pct> | pid.minactive <pct> | pid.status");
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
    speedMeterResetStats();
    sendTelnet("[SPD][STATUS] Contadores reseteados");
    return;
  }

  if (command.equalsIgnoreCase("speed.stream")) {
    if (args.isEmpty()) {
      const uint32_t periodMs = static_cast<uint32_t>(g_speedStreamPeriod * portTICK_PERIOD_MS);
      String msg = "[SPD][STREAM] ";
      msg += g_speedStreamEnabled ? "ON" : "OFF";
      msg += " periodo=";
      msg += String(periodMs);
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
    msg += String(periodMs);
    msg += "ms";
    sendTelnet(msg);
    sendTelnet(buildSpeedStatusMessage());
    return;
  }

  if (command.equalsIgnoreCase("speed.uart")) {
    SpeedMeterConfig cfg{};
    speedMeterGetConfig(cfg);

    if (args.isEmpty()) {
      String msg = "[SPD][UART] baud=";
      msg += cfg.baudRate;
      msg += " invert=";
      msg += cfg.invertRx ? "ON" : "OFF";
      msg += " rxPin=";
      msg += cfg.rxPin;
      sendTelnet(msg);
      return;
    }

    String arg1 = args;
    String arg2;
    const int spaceIndexArgs = args.indexOf(' ');
    if (spaceIndexArgs >= 0) {
      arg1 = args.substring(0, spaceIndexArgs);
      arg2 = args.substring(spaceIndexArgs + 1);
      arg1.trim();
      arg2.trim();
    }

    int newBaud = cfg.baudRate;
    bool newInvert = cfg.invertRx;
    bool parsedAny = false;

    int parsedBaud = 0;
    bool parsedInvert = false;

    if (parseIntArg(arg1, parsedBaud)) {
      if (parsedBaud <= 0) {
        sendTelnet("[SPD][UART] Baud invalido");
        return;
      }
      newBaud = parsedBaud;
      parsedAny = true;
    } else if (parseBoolArg(arg1, parsedInvert)) {
      newInvert = parsedInvert;
      parsedAny = true;
    } else {
      sendTelnet("[SPD][UART] Uso: speed.uart | speed.uart <baud> [on|off] | speed.uart [on|off] [baud]");
      return;
    }

    if (!arg2.isEmpty()) {
      if (parseIntArg(arg2, parsedBaud)) {
        if (parsedBaud <= 0) {
          sendTelnet("[SPD][UART] Baud invalido");
          return;
        }
        newBaud = parsedBaud;
      } else if (parseBoolArg(arg2, parsedInvert)) {
        newInvert = parsedInvert;
      } else {
        sendTelnet("[SPD][UART] Uso: speed.uart | speed.uart <baud> [on|off] | speed.uart [on|off] [baud]");
        return;
      }
      parsedAny = true;
    }

    if (!parsedAny) {
      sendTelnet("[SPD][UART] Uso: speed.uart | speed.uart <baud> [on|off] | speed.uart [on|off] [baud]");
      return;
    }

    if (!speedMeterSetUartConfig(newBaud, newInvert)) {
      sendTelnet("[SPD][UART] Error aplicando configuracion");
      return;
    }

    speedMeterGetConfig(cfg);
    String msg = "[SPD][UART] aplicado baud=";
    msg += cfg.baudRate;
    msg += " invert=";
    msg += cfg.invertRx ? "ON" : "OFF";
    msg += " rxPin=";
    msg += cfg.rxPin;
    sendTelnet(msg);
    sendTelnet(buildSpeedStatusMessage());
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
    if (logHeartbeat && (xTaskGetTickCount() - lastHeartbeat >= heartbeat)) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
