#include <Arduino.h>
#include "../include/LibreriaCuatri.h"
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <TelnetStream.h>
//defino la conexion a WiFi
const char* ssid = ("ESPcuatri");
const char* contrasena = ("teamcit2024");
void setup() {
  InicializaUart();  // Inicializa UART0 con 115200 baudios
  InicializaWiFi(ssid,contrasena); //hago el llamado desde el main para que se conecte al WiFi
  InicializaOTA();
  delay(1000);
  InicializaTelnet(); // Inicia Telnet
  EnviarMensaje("ESP32 conectado y listo para comunicación");
}

void loop() {
  String mensaje;
  if (RecibirMensaje(mensaje)) {
    // Procesar el mensaje recibido
    EnviarMensaje("Mensaje recibido: " + mensaje);
    EnviarMensajeTelnet("UART: " + mensaje);
  }
  
  // Enviar mensaje periódico por Telnet para verificar conexión
  static unsigned long lastTelnetMsg = 0;
  if (millis() - lastTelnetMsg > 5000) { // Cada 5 segundos
    EnviarMensajeTelnet("ESP32 activo - " + String(millis()/1000) + "s");
    lastTelnetMsg = millis();
  }
  
  ArduinoOTA.handle(); //Importante para el funcionamiento del OTA llama a la ESP a revisar el estado
  delay(100);
}

