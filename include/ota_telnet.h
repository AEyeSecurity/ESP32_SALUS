#ifndef OTA_TELNET_H
#define OTA_TELNET_H

#include <TelnetStream.h>
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void InicializaWiFi(const char* ssid, const char* contrasena); //Funcion que inicia la conexion con la ESP por WiFi
void InicializaOTA(); //Funcion que inicializa OTA
void InicializaTelnet();
void EnviarMensajeTelnet(const String& txt); // Envia mensaje por Telnet

struct OtaTelnetTaskConfig {
  bool logHeartbeat;
  TickType_t heartbeatInterval;
  TickType_t taskPeriod;
};

void serialIf(bool enabled, const String& message);
void telnetIf(bool enabled, const String& message);
void broadcastIf(bool enabled, const String& message);
void taskOtaTelnet(void* parameter);

#endif
