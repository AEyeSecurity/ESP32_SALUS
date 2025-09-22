#include <Arduino.h>
#include "../include/LibreriaCuatri.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <TelnetStream.h>

// Defino la conexión a WiFi
const char* ssid = ("ESPcuatri");
const char* contrasena = ("teamcit2024");

// PWM Pins
#define ACCEL_PWM 17
#define PWM_CHANNEL 0       // Canal 0 de PWM
#define PWM_FREQ 5000       // Frecuencia 5kHz
#define PWM_RESOLUTION 8    // Resolución de 8 bits (0-255)

void setup() {
  InicializaUart();  
  InicializaWiFi(ssid, contrasena);
  InicializaOTA();
  delay(1000);
  InicializaTelnet();
  EnviarMensaje("ESP32 conectado y listo para comunicación");

  // Configuro PWM en ESP32
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(ACCEL_PWM, PWM_CHANNEL);

  // Duty inicial en 0
  ledcWrite(PWM_CHANNEL, 0);
}

void loop() {
  String mensaje;
  if (RecibirMensaje(mensaje)) {
    EnviarMensaje("Mensaje recibido: " + mensaje);
    EnviarMensajeTelnet("UART: " + mensaje);
  }

  static unsigned long lastTelnetMsg = 0;
  if (millis() - lastTelnetMsg > 5000) {
    EnviarMensajeTelnet("ESP32 activo - " + String(millis()/1000) + "s");
    lastTelnetMsg = millis();
  }

  // Siempre revisar OTA
  ArduinoOTA.handle();

  // PWM inicial
  float pwm = 61;
  ledcWrite(PWM_CHANNEL, pwm);
  EnviarMensaje("offset");

  // Espera de 2 segundos pero sin bloquear OTA
  unsigned long start = millis();
  while (millis() - start < 2000) {
    ArduinoOTA.handle();
    delay(10);
  }

  // Barrido de PWM con OTA activo
  for (int i = 61; i < 227; i++) {
    ledcWrite(PWM_CHANNEL, i);
    EnviarMensaje("a");

    unsigned long stepStart = millis();
    while (millis() - stepStart < 250) {
      ArduinoOTA.handle();
      delay(10);
    }
  }
}
