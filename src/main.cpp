#include <Arduino.h>
#include "ota_telnet.h"
#include <ArduinoOTA.h>
#include "fs_ia6.h"
#include "h_bridge.h"
#include "AS5600.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const char* ssid = "ESPcuatri";
const char* contrasena = "teamcit2024";

constexpr uint16_t STACK_OTA = 4096;
constexpr uint16_t STACK_BRIDGE = 4096;
constexpr uint16_t STACK_RC = 2048;
constexpr uint16_t STACK_AS5600 = 3072;
constexpr uint16_t STACK_PID = 2048;

constexpr int AS5600_SDA_PIN = 25;
constexpr int AS5600_SCL_PIN = 26;

constexpr TickType_t OTA_PERIOD = pdMS_TO_TICKS(20);
constexpr TickType_t PID_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t RC_PERIOD = pdMS_TO_TICKS(100);
constexpr TickType_t AS5600_PERIOD = pdMS_TO_TICKS(200);

namespace debug {
constexpr bool kLogSystem = false;
constexpr bool kLogOta = false;
constexpr bool kLogBridge = false;
constexpr bool kLogLoop = false;
constexpr bool kLogRc = true;
constexpr bool kLogAs5600 = true;
constexpr bool kEnableBridgeTask = false;
}  // namespace debug

static void serialIf(bool enabled, const String& message) {
  if (!enabled) {
    return;
  }
  EnviarMensaje(message);
}

static void telnetIf(bool enabled, const String& message) {
  if (!enabled) {
    return;
  }
  EnviarMensajeTelnet(message);
}

static void broadcastIf(bool enabled, const String& message) {
  serialIf(enabled, message);
  telnetIf(enabled, message);
}

static void taskOtaTelnet(void* parameter);
static void taskBridgeTest(void* parameter);
static void taskRcMonitor(void* parameter);
static void taskPid(void* parameter);
static void taskAs5600Monitor(void* parameter);

static AS5600 g_as5600;

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

  broadcastIf(debug::kLogRc, "Iniciando pruebas FS-iA6 (GPIO0, GPIO2, GPIO4 y GPIO16)");

  pinMode(0, INPUT);
  pinMode(2, INPUT);
  pinMode(4, INPUT);
  pinMode(16, INPUT);

  if (debug::kEnableBridgeTask) {
    init_h_bridge();
    broadcastIf(debug::kLogBridge, "Inicializado H-bridge (pins 21 enable, 19 left PWM, 18 right PWM)");
  }

  startTaskPinned(taskOtaTelnet, "OTA", STACK_OTA, nullptr, 3, nullptr, 0);
  if (debug::kEnableBridgeTask) {
    startTaskPinned(taskBridgeTest, "BridgeTest", STACK_BRIDGE, nullptr, 2, nullptr, 1);
  }
  startTaskPinned(taskRcMonitor, "RCMonitor", STACK_RC, nullptr, 1, nullptr, 1);
  startTaskPinned(taskPid, "PID", STACK_PID, nullptr, 2, nullptr, 1);

  g_as5600.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  if (!g_as5600.isConnected()) {
    Serial.println("[AS5600] No se detecta el sensor en el bus I2C");
  } else {
    Serial.println("[AS5600] Sensor inicializado correctamente");
  }
  startTaskPinned(taskAs5600Monitor, "AS5600", STACK_AS5600, nullptr, 1, nullptr, 1);
}

void loop() {
  String mensaje;
  if (RecibirMensaje(mensaje) && debug::kLogLoop) {
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
    if (debug::kLogOta && (xTaskGetTickCount() - lastHeartbeat >= pdMS_TO_TICKS(5000))) {
      EnviarMensajeTelnet("ESP32 activo - " + String(millis() / 1000) + "s");
      lastHeartbeat = xTaskGetTickCount();
    }
    vTaskDelayUntil(&lastWake, OTA_PERIOD);
  }
}

static void reportDuty(const char* label, int duty) {
  String msg = String("H-bridge ") + label + " duty: " + duty + "%";
  broadcastIf(debug::kLogBridge, msg);
}

static void taskBridgeTest(void* parameter) {
  const TickType_t rampUpStepDelay = pdMS_TO_TICKS(80);
  const TickType_t rampDownStepDelay = pdMS_TO_TICKS(60);
  const TickType_t pauseBetweenDirections = pdMS_TO_TICKS(300);
  const TickType_t cyclePause = pdMS_TO_TICKS(2000);

  for (;;) {
    enable_bridge_h();
    broadcastIf(debug::kLogBridge, "Iniciando prueba H-bridge: izquierda subir/bajar, derecha subir/bajar");

    for (int d = 0; d <= 100; d += 5) {
      bridge_turn_left(static_cast<uint8_t>(d));
      if (debug::kLogBridge && (d % 20 == 0 || d == 0 || d == 100)) {
        reportDuty("LEFT", d);
      }
      vTaskDelay(rampUpStepDelay);
    }

    for (int d = 100; d >= 0; d -= 5) {
      bridge_turn_left(static_cast<uint8_t>(d));
      if (debug::kLogBridge && (d % 20 == 0 || d == 0 || d == 100)) {
        reportDuty("LEFT", d);
      }
      vTaskDelay(rampDownStepDelay);
    }

    bridge_stop();
    vTaskDelay(pauseBetweenDirections);

    for (int d = 0; d <= 100; d += 5) {
      bridge_turn_right(static_cast<uint8_t>(d));
      if (debug::kLogBridge && (d % 20 == 0 || d == 0 || d == 100)) {
        reportDuty("RIGHT", d);
      }
      vTaskDelay(rampUpStepDelay);
    }

    for (int d = 100; d >= 0; d -= 5) {
      bridge_turn_right(static_cast<uint8_t>(d));
      if (debug::kLogBridge && (d % 20 == 0 || d == 0 || d == 100)) {
        reportDuty("RIGHT", d);
      }
      vTaskDelay(rampDownStepDelay);
    }

    bridge_stop();
    disable_bridge_h();
    broadcastIf(debug::kLogBridge, "Prueba H-bridge finalizada");

    vTaskDelay(cyclePause);
  }
}

static void taskRcMonitor(void* parameter) {
  int lastCh0 = 9999;
  int lastCh2 = 9999;
  int lastCh4 = 9999;
  int lastCh16 = 9999;
  for (;;) {
    int ch0 = readChannel(0, -100, 100, 0);
    int ch2 = readChannel(2, -100, 100, 0);
    int ch4 = readChannel(4, -100, 100, 0);
    int ch16 = readChannel(16, -100, 100, 0);
    if (ch0 != lastCh0 || ch2 != lastCh2 || ch4 != lastCh4 || ch16 != lastCh16) {
      String rcMsg = "FS-iA6 -> GPIO0: " + String(ch0) + " | GPIO2: " + String(ch2) +
                     " | GPIO4: " + String(ch4) + " | GPIO16: " + String(ch16);
      broadcastIf(debug::kLogRc, rcMsg);
      lastCh0 = ch0;
      lastCh2 = ch2;
      lastCh4 = ch4;
      lastCh16 = ch16;
    }
    vTaskDelay(RC_PERIOD);
  }
}

static void taskAs5600Monitor(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();
  for (;;) {
    if (!g_as5600.isConnected()) {
      Serial.println("[AS5600] Error: sensor desconectado");
      vTaskDelayUntil(&lastWake, AS5600_PERIOD);
      continue;
    }

    const uint8_t status = g_as5600.getStatus();
    const float angle_deg = g_as5600.getAngleDegrees();

    String statusMsg;
    if (status == 0xFF) {
      statusMsg = "estado no disponible";
    } else if ((status & 0x20) == 0) {  // STATUS_MD bit
      statusMsg = "sin iman";
    } else if (status & 0x08) {  // STATUS_MH bit
      statusMsg = "iman muy cerca";
    } else if (status & 0x10) {  // STATUS_ML bit
      statusMsg = "iman muy lejos";
    } else {
      statusMsg = "iman OK";
    }

    const bool angleOk = angle_deg >= 0.0f;
    const String angleMsg = angleOk ? String(angle_deg, 2) + "°" : String("error lectura");

    String logMsg = "[AS5600] angulo=" + angleMsg + " | " + statusMsg;
    broadcastIf(debug::kLogAs5600, logMsg);

    vTaskDelayUntil(&lastWake, AS5600_PERIOD);
  }
}

static void taskPid(void* parameter) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, PID_PERIOD);
  }
}


