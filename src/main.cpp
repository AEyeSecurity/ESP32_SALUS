#include <Arduino.h>
#include "ota_telnet.h"
#include <ArduinoOTA.h>
#include "fs_ia6.h"
#include "h_bridge.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const char* ssid = "ESPcuatri";
const char* contrasena = "teamcit2024";

constexpr uint16_t STACK_OTA = 4096;
constexpr uint16_t STACK_BRIDGE = 4096;
constexpr uint16_t STACK_RC = 2048;
constexpr uint16_t STACK_PID = 2048;

constexpr TickType_t OTA_PERIOD = pdMS_TO_TICKS(20);
constexpr TickType_t PID_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t RC_PERIOD = pdMS_TO_TICKS(100);

static void taskOtaTelnet(void* parameter);
static void taskBridgeTest(void* parameter);
static void taskRcMonitor(void* parameter);
static void taskPid(void* parameter);

static bool startTaskPinned(TaskFunction_t task,
                            const char* name,
                            uint16_t stackSize,
                            void* parameter,
                            UBaseType_t priority,
                            TaskHandle_t* handle,
                            const BaseType_t coreId) {
  BaseType_t result = xTaskCreatePinnedToCore(task, name, stackSize, parameter, priority, handle, coreId);
  if (result != pdPASS) {
    String msg = String("No se pudo crear la tarea ") + name;
    EnviarMensaje(msg);
    EnviarMensajeTelnet(msg);
    return false;
  }
  return true;
}

void setup() {
  InicializaUart();
  InicializaWiFi(ssid, contrasena);
  InicializaOTA();
  delay(1000);
  InicializaTelnet();
  EnviarMensaje("ESP32 conectado y listo para comunicacion");

  pinMode(14, INPUT);
  pinMode(16, INPUT);

  init_h_bridge();
  EnviarMensaje("Inicializado H-bridge (pins 21 enable, 19 left PWM, 18 right PWM)");

  startTaskPinned(taskOtaTelnet, "OTA", STACK_OTA, nullptr, 3, nullptr, 0);
  startTaskPinned(taskBridgeTest, "BridgeTest", STACK_BRIDGE, nullptr, 2, nullptr, 1);
  startTaskPinned(taskRcMonitor, "RCMonitor", STACK_RC, nullptr, 1, nullptr, 1);
  startTaskPinned(taskPid, "PID", STACK_PID, nullptr, 2, nullptr, 1);
}

void loop() {
  String mensaje;
  if (RecibirMensaje(mensaje)) {
    EnviarMensaje("Mensaje recibido: " + mensaje);
    EnviarMensajeTelnet("UART: " + mensaje);
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}

static void taskOtaTelnet(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastHeartbeat = lastWake;
  for (;;) {
    ArduinoOTA.handle();
    if (xTaskGetTickCount() - lastHeartbeat >= pdMS_TO_TICKS(5000)) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, OTA_PERIOD);
  }
}

static void reportDuty(const char* label, int duty) {
  String msg = String("H-bridge ") + label + " duty: " + duty + "%";
  EnviarMensaje(msg);
  EnviarMensajeTelnet(msg);
}

static void taskBridgeTest(void* parameter) {
  const TickType_t rampUpStepDelay = pdMS_TO_TICKS(80);
  const TickType_t rampDownStepDelay = pdMS_TO_TICKS(60);
  const TickType_t pauseBetweenDirections = pdMS_TO_TICKS(300);
  const TickType_t cyclePause = pdMS_TO_TICKS(2000);

  for (;;) {
    enable_bridge_h();
    EnviarMensaje("Iniciando prueba H-bridge: izquierda subir/bajar, derecha subir/bajar");
    EnviarMensajeTelnet("Iniciando prueba H-bridge");

    for (int d = 0; d <= 100; d += 5) {
      bridge_turn_left(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("LEFT", d);
      }
      vTaskDelay(rampUpStepDelay);
    }

    for (int d = 100; d >= 0; d -= 5) {
      bridge_turn_left(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("LEFT", d);
      }
      vTaskDelay(rampDownStepDelay);
    }

    bridge_stop();
    vTaskDelay(pauseBetweenDirections);

    for (int d = 0; d <= 100; d += 5) {
      bridge_turn_right(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("RIGHT", d);
      }
      vTaskDelay(rampUpStepDelay);
    }

    for (int d = 100; d >= 0; d -= 5) {
      bridge_turn_right(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("RIGHT", d);
      }
      vTaskDelay(rampDownStepDelay);
    }

    bridge_stop();
    disable_bridge_h();
    EnviarMensaje("Prueba H-bridge finalizada");

    vTaskDelay(cyclePause);
  }
}

static void taskRcMonitor(void* parameter) {
  int lastCh14 = 9999;
  int lastCh16 = 9999;
  for (;;) {
    int ch14 = readChannel(14, -100, 100, 0);
    int ch16 = readChannel(16, -100, 100, 0);
    if (ch14 != lastCh14 || ch16 != lastCh16) {
      String rcMsg = "RC -> Pin14: " + String(ch14) + " | Pin16: " + String(ch16);
      EnviarMensaje(rcMsg);
      EnviarMensajeTelnet(rcMsg);
      lastCh14 = ch14;
      lastCh16 = ch16;
    }
    vTaskDelay(RC_PERIOD);
  }
}

static void taskPid(void* parameter) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    // Pendiente: implementar logica del PID (ejecuta cada 30 ms)
    vTaskDelayUntil(&last, PID_PERIOD);
  }
}