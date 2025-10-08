#include <Arduino.h>
#include "ota_telnet.h"
#include "fs_ia6.h"
#include "AS5600.h"
#include "h_bridge.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos_utils.h"
#include "quad_logic.h"

const char* ssid = "ESPcuatri";
const char* contrasena = "teamcit2024";

constexpr uint16_t STACK_OTA = 4096;
constexpr uint16_t STACK_BRIDGE = 4096;
constexpr uint16_t STACK_RC = 2048;
constexpr uint16_t STACK_AS5600 = 3072;
constexpr uint16_t STACK_QUAD = 4096;

constexpr int AS5600_SDA_PIN = 25;
constexpr int AS5600_SCL_PIN = 26;

constexpr TickType_t OTA_PERIOD = pdMS_TO_TICKS(20);
constexpr TickType_t RC_PERIOD = pdMS_TO_TICKS(100);
constexpr TickType_t AS5600_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t AS5600_LOG_INTERVAL = pdMS_TO_TICKS(500);
constexpr TickType_t QUAD_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t QUAD_RC_TIMEOUT = pdMS_TO_TICKS(250);

constexpr int QUAD_THROTTLE_PWM_PIN = 17;
constexpr uint8_t QUAD_THROTTLE_LEDC_CHANNEL = 4;  // Reservado para acelerador (no se solapa con el puente H)
constexpr double QUAD_THROTTLE_PWM_FREQUENCY = 20000.0;
constexpr uint8_t QUAD_THROTTLE_PWM_RES_BITS = 10;
constexpr int QUAD_THROTTLE_DEADZONE = 10;
constexpr uint8_t QUAD_THROTTLE_MIN_PERCENT = 25;
constexpr uint8_t QUAD_THROTTLE_MAX_PERCENT = 90;

constexpr uint32_t GEARBOX_UART_BAUD = 115200;
constexpr int GEARBOX_UART_RX_PIN = -1;  // Configurar cuando se definan los pines
constexpr int GEARBOX_UART_TX_PIN = -1;
constexpr const char* QUAD_REVERSE_ENABLE_CMD = ">DRIVE:REV:ON\n";
constexpr const char* QUAD_REVERSE_DISABLE_CMD = ">DRIVE:REV:OFF\n";

namespace debug {
constexpr bool kLogSystem = false;
constexpr bool kLogOta = false;
constexpr bool kLogBridge = false;
constexpr bool kLogLoop = false;
constexpr bool kLogRc = true;
constexpr bool kLogAs5600 = false;
constexpr bool kEnableBridgeTask = false;
constexpr bool kEnableRcTask = true;
constexpr bool kLogQuad = false;
constexpr bool kEnableQuadTask = true;
}  // namespace debug

static AS5600 g_as5600;

static OtaTelnetTaskConfig g_otaConfig = {debug::kLogOta, pdMS_TO_TICKS(5000), OTA_PERIOD};
static HBridgeTaskConfig g_bridgeConfig = {debug::kLogBridge};
static FsIa6TaskConfig g_rcConfig = {debug::kLogRc, RC_PERIOD};
static AS5600MonitorConfig g_as5600TaskConfig = {&g_as5600, debug::kLogAs5600, AS5600_PERIOD, AS5600_LOG_INTERVAL};
static QuadLogicConfig g_quadConfig{};
static QueueHandle_t g_rcInputQueue = nullptr;

void setup() {
  InicializaUart();
  InicializaWiFi(ssid, contrasena);
  InicializaOTA();
  delay(1000);
  InicializaTelnet();

  broadcastIf(debug::kLogRc,
              "Iniciando pruebas FS-iA6 (GPIO4 acelerador/reversa, GPIO6 direccion, GPIO0 auxiliar1, GPIO2 auxiliar2)");

  pinMode(kRcThrottlePin, INPUT);
  pinMode(kRcSteeringPin, INPUT);
  pinMode(kRcAux1Pin, INPUT);
  pinMode(kRcAux2Pin, INPUT);

  if (debug::kEnableBridgeTask) {
    init_h_bridge();
    broadcastIf(debug::kLogBridge, "Inicializado H-bridge (pins 21 enable, 19 left PWM, 18 right PWM)");
  }

  g_rcInputQueue = quadLogicCreateRcQueue();
  bool quadLogicReady = false;
  if (g_rcInputQueue == nullptr) {
    broadcastIf(true, "[QUAD] ERROR: no se pudo crear la cola de entradas RC");
  } else {
    g_quadConfig.throttlePwmPin = QUAD_THROTTLE_PWM_PIN;
    g_quadConfig.throttleLedcChannel = QUAD_THROTTLE_LEDC_CHANNEL;
    g_quadConfig.throttlePwmFrequency = QUAD_THROTTLE_PWM_FREQUENCY;
    g_quadConfig.throttlePwmResolutionBits = QUAD_THROTTLE_PWM_RES_BITS;
    g_quadConfig.throttleDeadzone = QUAD_THROTTLE_DEADZONE;
    g_quadConfig.throttleMinPercent = QUAD_THROTTLE_MIN_PERCENT;
    g_quadConfig.throttleMaxPercent = QUAD_THROTTLE_MAX_PERCENT;
    g_quadConfig.taskPeriod = QUAD_PERIOD;
    g_quadConfig.rcTimeout = QUAD_RC_TIMEOUT;
    g_quadConfig.log = debug::kLogQuad;
    g_quadConfig.gearboxSerial = &Serial1;
    g_quadConfig.gearboxBaud = GEARBOX_UART_BAUD;
    g_quadConfig.gearboxRxPin = GEARBOX_UART_RX_PIN;
    g_quadConfig.gearboxTxPin = GEARBOX_UART_TX_PIN;
    g_quadConfig.reverseEnableCommand = QUAD_REVERSE_ENABLE_CMD;
    g_quadConfig.reverseDisableCommand = QUAD_REVERSE_DISABLE_CMD;
    g_quadConfig.rcQueue = g_rcInputQueue;

    if (debug::kEnableQuadTask) {
      quadLogicReady = initQuadLogic(g_quadConfig);
      if (!quadLogicReady) {
        broadcastIf(true, "[QUAD] ERROR: initQuadLogic fallo");
      }
    }
  }

  startTaskPinned(taskOtaTelnet, "OTA", STACK_OTA, &g_otaConfig, 3, nullptr, 0);
  if (debug::kEnableBridgeTask) {
    startTaskPinned(taskBridgeTest, "BridgeTest", STACK_BRIDGE, &g_bridgeConfig, 2, nullptr, 1);
  }
  if (debug::kEnableRcTask) {
    startTaskPinned(taskRcMonitor, "RCMonitor", STACK_RC, &g_rcConfig, 1, nullptr, 1);
  }

  g_as5600.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  runAs5600SelfTest(g_as5600, debug::kLogAs5600);
  startTaskPinned(taskAs5600Monitor, "AS5600", STACK_AS5600, &g_as5600TaskConfig, 1, nullptr, 1);

  if (debug::kEnableQuadTask && quadLogicReady) {
    startTaskPinned(taskQuadLogic, "QuadLogic", STACK_QUAD, &g_quadConfig, 2, nullptr, 1);
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
