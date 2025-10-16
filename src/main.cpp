#include <Arduino.h>
#include "ota_telnet.h"
#include "fs_ia6.h"
#include "h_bridge.h"
#include "AS5600.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos_utils.h"
#include "pid.h"
#include "quad_functions.h"

const char* ssid = "ESPcuatri";
const char* contrasena = "teamcit2024";

constexpr uint16_t STACK_OTA = 4096;
constexpr uint16_t STACK_BRIDGE = 4096;
constexpr uint16_t STACK_RC = 2048;
constexpr uint16_t STACK_AS5600 = 3072;
constexpr uint16_t STACK_PID = 4096;
constexpr uint16_t STACK_THROTTLE = 2048;

constexpr int AS5600_SDA_PIN = 25;
constexpr int AS5600_SCL_PIN = 26;

constexpr float PID_CENTER_DEG = 115.0f;
constexpr float PID_SPAN_DEG = 30.0f;
constexpr float PID_DEADBAND_PERCENT = 5.0f;
constexpr float PID_MIN_ACTIVE_PERCENT = 15.0f;
constexpr float PID_KP = 2.0f;
constexpr float PID_KI = 0.0f;
constexpr float PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 50.0f;

constexpr uint8_t THROTTLE_PWM_PIN = 17;
constexpr uint8_t THROTTLE_LEDC_CHANNEL = 2;
constexpr uint32_t THROTTLE_PWM_FREQ = 20000;
constexpr uint8_t THROTTLE_PWM_RESOLUTION = 8;
constexpr int THROTTLE_PWM_MIN_DUTY = 0;
constexpr int THROTTLE_PWM_MAX_DUTY = 200;
constexpr int THROTTLE_THRESHOLD = 15;

constexpr TickType_t OTA_PERIOD = pdMS_TO_TICKS(20);
constexpr TickType_t RC_PERIOD = pdMS_TO_TICKS(100);
constexpr TickType_t AS5600_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t AS5600_LOG_INTERVAL = pdMS_TO_TICKS(500);
constexpr TickType_t PID_PERIOD = pdMS_TO_TICKS(20);
constexpr TickType_t PID_LOG_INTERVAL = pdMS_TO_TICKS(200);
constexpr TickType_t THROTTLE_PERIOD = pdMS_TO_TICKS(40);

namespace debug {
constexpr bool kLogSystem = false;
constexpr bool kLogOta = false;
constexpr bool kLogBridge = false;
constexpr bool kLogLoop = false;
constexpr bool kLogRc = false;
constexpr bool kLogAs5600 = false;
constexpr bool kLogPid = true;
constexpr bool kLogThrottle = false;
constexpr bool kEnableBridgeTask = false;
constexpr bool kEnableRcTask = false;
constexpr bool kEnablePidTask = true;
constexpr bool kEnableThrottleTask = true;
}  // namespace debug

static AS5600 g_as5600;
static PidController g_pidController;

static OtaTelnetTaskConfig g_otaConfig = {debug::kLogOta, pdMS_TO_TICKS(5000), OTA_PERIOD};
static HBridgeTaskConfig g_bridgeConfig = {debug::kLogBridge};
static FsIa6TaskConfig g_rcConfig = {debug::kLogRc, RC_PERIOD};
static AS5600MonitorConfig g_as5600TaskConfig = {&g_as5600, debug::kLogAs5600, AS5600_PERIOD, AS5600_LOG_INTERVAL};
static PidTaskConfig g_pidTaskConfig = {
    &g_as5600,
    kRcSteeringPin,
    PID_CENTER_DEG,
    PID_SPAN_DEG,
    PID_DEADBAND_PERCENT,
    PID_MIN_ACTIVE_PERCENT,
    &g_pidController,
    PID_PERIOD,
    PID_LOG_INTERVAL,
    debug::kLogPid,
    true};
static QuadThrottleTaskConfig g_throttleTaskConfig = {
    {
        THROTTLE_PWM_PIN,
        THROTTLE_LEDC_CHANNEL,
        THROTTLE_PWM_FREQ,
        THROTTLE_PWM_RESOLUTION,
        THROTTLE_PWM_MIN_DUTY,
        THROTTLE_PWM_MAX_DUTY,
        THROTTLE_THRESHOLD,
    },
    kRcThrottlePin,
    true,
    THROTTLE_PERIOD,
    debug::kLogThrottle};

void setup() {
  InicializaUart();
  InicializaWiFi(ssid, contrasena);
  InicializaOTA();
  delay(1000);
  InicializaTelnet();

  g_pidController.setTunings(PID_KP, PID_KI, PID_KD);
  g_pidController.setOutputLimits(-100.0f, 100.0f);
  g_pidController.setIntegralLimits(-PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  g_pidController.reset();

  broadcastIf(
      debug::kLogRc,
      "Iniciando pruebas FS-iA6 (GPIO0, GPIO2, acelerador GPIO4, direccion izquierda/derecha GPIO16)");

  pinMode(0, INPUT);
  pinMode(2, INPUT);
  pinMode(kRcThrottlePin, INPUT);
  pinMode(kRcSteeringPin, INPUT);

  if (debug::kEnableBridgeTask) {
    init_h_bridge();
    broadcastIf(debug::kLogBridge, "Inicializado H-bridge (pins 21 enable, 19 left PWM, 18 right PWM)");
  }

  startTaskPinned(taskOtaTelnet, "OTA", STACK_OTA, &g_otaConfig, 3, nullptr, 0);
  if (debug::kEnableBridgeTask) {
    startTaskPinned(taskBridgeTest, "BridgeTest", STACK_BRIDGE, &g_bridgeConfig, 2, nullptr, 1);
  }
  if (debug::kEnableRcTask) {
    startTaskPinned(taskRcMonitor, "RCMonitor", STACK_RC, &g_rcConfig, 1, nullptr, 1);
  }
  if (debug::kEnableThrottleTask) {
    startTaskPinned(taskQuadThrottleControl, "Throttle", STACK_THROTTLE, &g_throttleTaskConfig, 1, nullptr, 1);
  }

  g_as5600.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  runAs5600SelfTest(g_as5600, debug::kLogAs5600);
  startTaskPinned(taskAs5600Monitor, "AS5600", STACK_AS5600, &g_as5600TaskConfig, 1, nullptr, 1);

  if (debug::kEnablePidTask) {
    if (debug::kEnableBridgeTask) {
      broadcastIf(true, "[PID] BridgeTest habilitado; omitiendo tarea PID para evitar conflictos");
    } else {
      broadcastIf(debug::kLogPid, "[PID] Tarea PID iniciada");
      startTaskPinned(taskPidControl, "PID", STACK_PID, &g_pidTaskConfig, 2, nullptr, 1);
    }
  }
}

void loop() {
  String mensaje;
  if (RecibirMensaje(mensaje) && debug::kLogLoop) {
    EnviarMensaje("Mensaje recibido: " + mensaje);
    EnviarMensajeTelnet("UART: " + mensaje);
  }
  vTaskDelay(pdMS_TO_TICKS(50));
}

