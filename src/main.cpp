#include <Arduino.h>
#include "ota_telnet.h"
#include <ArduinoOTA.h>
#include "fs_ia6.h"
#include "hall_sensor.h"

//defino la conexion a WiFi
const char* ssid = ("ESPcuatri");
const char* contrasena = ("teamcit2024");

// Create hall sensor instance for pin 15
HallSensor hallSensor(15);

void setup() {
  InicializaUart();  // Inicializa UART0 con 115200 baudios
  InicializaWiFi(ssid,contrasena); //hago el llamado desde el main para que se conecte al WiFi
  InicializaOTA();
  delay(1000);
  InicializaTelnet(); // Inicia Telnet
  EnviarMensaje("ESP32 conectado y listo para comunicación");

  // Initialize hall sensor
  hallSensor.begin();

  // Inicializar pines usados por el control remoto (asumimos canales en GPIO 14 y 16)
  // Nota: la librerda proporciona initFS_IA6 que puede inicializar 6 pines;
  // para evitar elegir pines adicionales arbitrarios solo configuramos los dos que usamos.
  pinMode(14, INPUT);
  pinMode(16, INPUT);
}

void loop() {
  String mensaje;
  if (RecibirMensaje(mensaje)) {
    // Procesar el mensaje recibido
    EnviarMensaje("Mensaje recibido: " + mensaje);
    EnviarMensajeTelnet("UART: " + mensaje);
  }
  
  // Read and send hall sensor angle
  if (hallSensor.isAngleReady()) {
    float angle = hallSensor.getAngle();
    String angleMsg = "Hall Sensor Angle: " + String(angle, 2) + " degrees";
    EnviarMensaje(angleMsg);
    EnviarMensajeTelnet(angleMsg);
  }
  
  // Enviar mensaje periódico por Telnet para verificar conexión
  static unsigned long lastTelnetMsg = 0;
  if (millis() - lastTelnetMsg > 5000) { // Cada 5 segundos
    EnviarMensajeTelnet("ESP32 activo - " + String(millis()/1000) + "s");
    lastTelnetMsg = millis();
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

