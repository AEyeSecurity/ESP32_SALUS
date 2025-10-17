#include "ota_telnet.h"

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>

#include <ctype.h>
#include <stdlib.h>

#include "steering_calibration.h"

namespace {
String g_telnetCommandBuffer;

void sendTelnet(const String& message) {
  EnviarMensajeTelnet(message);
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

  if (command.equalsIgnoreCase("steer.status")) {
    reportSteeringStatus();
    return;
  }

  if (command.equalsIgnoreCase("steer.help")) {
    sendTelnet("Comandos: steer.calibrate | steer.offset <deg> | steer.status");
    return;
  }

  sendTelnet("[STEER] Comando desconocido: " + line);
}

void processTelnetInput() {
  while (TelnetStream.available() > 0) {
    const char c = static_cast<char>(TelnetStream.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      handleTelnetCommand(g_telnetCommandBuffer);
      g_telnetCommandBuffer = "";
      continue;
    }
    if (g_telnetCommandBuffer.length() >= 120) {
      continue;
    }
    g_telnetCommandBuffer += c;
  }
}
}  // namespace


/* ========= UART ========= */
void InicializaUart(long baud) { Serial.begin(baud); }

void EnviarMensaje(const String& txt) { Serial.println(txt); }

bool RecibirMensaje(String& txt) {
  if (Serial.available()) {
    txt = Serial.readStringUntil('\n');
    return true;
  }
  return false;
}

/* ========= Wi-Fi ========= */
void InicializaWiFi(const char* ssid, const char* pass) {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, pass);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

/* ========= OTA ========= */
void InicializaOTA() {
  ArduinoOTA.onStart([]() { EnviarMensaje("*** OTA INICIO ***"); });
  ArduinoOTA.onEnd([]() { EnviarMensaje("*** OTA FIN ***"); });
  ArduinoOTA.onError([](ota_error_t e) { EnviarMensaje("Error OTA: " + String(e)); });
  ArduinoOTA.begin();
}

void InicializaTelnet() {  // Inicia Telnet en puerto 23
  TelnetStream.begin(23);
  delay(1000);  // Esperar a que se inicie el servidor
  EnviarMensaje("Servidor Telnet iniciado en puerto 23");
  TelnetStream.println("=== Servidor Telnet ESP32 ===");
  TelnetStream.println("Conexion establecida correctamente");
}

void EnviarMensajeTelnet(const String& txt) {  // Envia mensaje por Telnet
  TelnetStream.println(txt);
}

void serialIf(bool enabled, const String& message) {
  if (!enabled) {
    return;
  }
  EnviarMensaje(message);
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
    processTelnetInput();
    if (logHeartbeat && (xTaskGetTickCount() - lastHeartbeat >= heartbeat)) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
