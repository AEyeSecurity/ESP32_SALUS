#ifndef OTA_TELNET_H
#define OTA_TELNET_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

void InicializaWiFi(); // Funcion que inicializa WiFi en modo STA con fallback AP
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
