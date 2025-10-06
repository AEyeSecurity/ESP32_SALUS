#include "ota_telnet.h"

#include <WiFi.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>

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
    if (logHeartbeat && (xTaskGetTickCount() - lastHeartbeat >= heartbeat)) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, period);
  }
}
