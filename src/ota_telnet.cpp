#include "ota_telnet.h"
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <TelnetStream.h>

/* ========= UART ========= */
void InicializaUart(long baud) { Serial.begin(baud); }

void EnviarMensaje(const String& txt) { Serial.println(txt); }

bool RecibirMensaje(String& txt) {
    if (Serial.available()) { txt = Serial.readStringUntil('\n'); return true; }
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
    ArduinoOTA.onStart([]() {
        EnviarMensaje("*** OTA INICIO ***");
    });
    ArduinoOTA.onEnd([]() {
        EnviarMensaje("*** OTA FIN ***");
    });
    ArduinoOTA.onError([](ota_error_t e) {
        EnviarMensaje("Error OTA: " + String(e));
    });
    ArduinoOTA.begin();
}

void InicializaTelnet() {// Inicia Telnet en puerto 23
  TelnetStream.begin(23);
  delay(1000); // Esperar a que se inicie el servidor
  EnviarMensaje("Servidor Telnet iniciado en puerto 23");
  TelnetStream.println("=== Servidor Telnet ESP32 ===");
  TelnetStream.println("Conexi\u00f3n establecida correctamente");
}
void EnviarMensajeTelnet(const String& txt) { // Env\u00eda mensaje por Telnet
  TelnetStream.println(txt);
}

