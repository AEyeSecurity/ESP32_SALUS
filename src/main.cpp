#include <Arduino.h>
#include "ota_telnet.h"
#include <ArduinoOTA.h>
#include "fs_ia6.h"
#include "h_bridge.h"

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

  // Inicializar pines usados por el control remoto (asumimos canales en GPIO 14 y 16)
  // Nota: la librerda proporciona initFS_IA6 que puede inicializar 6 pines;
  // para evitar elegir pines adicionales arbitrarios solo configuramos los dos que usamos.
  pinMode(14, INPUT);
  pinMode(16, INPUT);
  // Inicializar H-bridge (configura pines y PWM)
  init_h_bridge();
  EnviarMensaje("Inicializado H-bridge (pins 21 enable, 19 left PWM, 18 right PWM)");
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
  
  // --- Prueba infinita del puente H: se ejecuta cada iteración del loop ---
  enable_bridge_h();
  EnviarMensaje("Iniciando prueba H-bridge: izquierda subir/bajar, derecha subir/bajar");
  EnviarMensajeTelnet("Iniciando prueba H-bridge");

  // Ramp: izquierda subir a 100% y bajar
  for (int d = 0; d <= 100; d += 5) {
    bridge_turn_left(d);
    // Print status cada 20% para no saturar el canal
    if (d % 20 == 0 || d == 0 || d == 100) {
      String s = "H-bridge LEFT duty: " + String(d) + "%";
      EnviarMensaje(s);
      EnviarMensajeTelnet(s);
    }
    ArduinoOTA.handle();
    delay(80);
  }
  for (int d = 100; d >= 0; d -= 5) {
    bridge_turn_left(d);
    if (d % 20 == 0 || d == 0 || d == 100) {
      String s = "H-bridge LEFT duty: " + String(d) + "%";
      EnviarMensaje(s);
      EnviarMensajeTelnet(s);
    }
    ArduinoOTA.handle();
    delay(60);
  }

  // Pause corto
  bridge_stop();
  for (int i = 0; i < 30; ++i) { // 30 * 10ms = 300ms
    ArduinoOTA.handle();
    delay(10);
  }

  // Ramp: derecha subir a 100% y bajar
  for (int d = 0; d <= 100; d += 5) {
    bridge_turn_right(d);
    if (d % 20 == 0 || d == 0 || d == 100) {
      String s = "H-bridge RIGHT duty: " + String(d) + "%";
      EnviarMensaje(s);
      EnviarMensajeTelnet(s);
    }
    ArduinoOTA.handle();
    delay(80);
  }
  for (int d = 100; d >= 0; d -= 5) {
    bridge_turn_right(d);
    if (d % 20 == 0 || d == 0 || d == 100) {
      String s = "H-bridge RIGHT duty: " + String(d) + "%";
      EnviarMensaje(s);
      EnviarMensajeTelnet(s);
    }
    ArduinoOTA.handle();
    delay(60);
  }

  bridge_stop();
  disable_bridge_h();
  EnviarMensaje("Prueba H-bridge finalizada");

  // Espera entre intentos (pausa antes del siguiente ciclo de prueba)
  // Llamamos ArduinoOTA.handle() periódicamente durante la espera para
  // mantener la disponibilidad de OTA.
  for (int i = 0; i < 200; ++i) { // 200 * 10ms = 2000ms
    ArduinoOTA.handle();
    delay(10);
  }
  
  ArduinoOTA.handle(); //Importante para el funcionamiento del OTA llama a la ESP a revisar el estado
  // Leer canales del control remoto conectado a los pines 14 y 16
  static int lastCh14 = 9999;
  static int lastCh16 = 9999;
  int ch14 = readChannel(14, -100, 100, 0); // mapea a rango -100..100
  int ch16 = readChannel(16, -100, 100, 0);
  if (ch14 != lastCh14 || ch16 != lastCh16) {
    String rcMsg = "RC -> Pin14: " + String(ch14) + " | Pin16: " + String(ch16);
    EnviarMensaje(rcMsg);
    EnviarMensajeTelnet(rcMsg);
    lastCh14 = ch14;
    lastCh16 = ch16;
  }

  delay(100);
}

