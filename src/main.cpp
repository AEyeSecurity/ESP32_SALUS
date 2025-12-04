#include <Arduino.h>
#include "ota_telnet.h"
#include "fs_ia6.h"
#include "h_bridge.h"
#include "AS5600.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "freertos_utils.h"
#include "pid.h"
#include "steering_calibration.h"
#include "quad_functions.h"
#include "pi_comms.h"

const char* ssid = "ESPcuatri";
const char* contrasena = "teamcit2024";

constexpr uint16_t STACK_OTA = 4096;
constexpr uint16_t STACK_BRIDGE = 4096;
constexpr uint16_t STACK_RC = 2048;
constexpr uint16_t STACK_AS5600 = 3072;
constexpr uint16_t STACK_PID = 4096;
constexpr uint16_t STACK_DRIVE = 4096;
constexpr uint16_t STACK_PI_RX = 3072;
constexpr uint16_t STACK_PI_TX = 2048;

constexpr int AS5600_SDA_PIN = 25;
constexpr int AS5600_SCL_PIN = 26;

constexpr float PID_CENTER_DEG = 240.0f;
constexpr float PID_SPAN_DEG = 40.0f;
constexpr float PID_DEADBAND_PERCENT = 0.25f;
constexpr float PID_MIN_ACTIVE_PERCENT = 15.0f;
constexpr float PID_KP = 3.0f;
constexpr float PID_KI = 0.5f;
constexpr float PID_KD = 0.0f;
constexpr float PID_INTEGRAL_LIMIT = 50.0f;

constexpr uint8_t THROTTLE_PWM_PIN = 17;
constexpr uint8_t THROTTLE_LEDC_CHANNEL = 4;
constexpr uint32_t THROTTLE_PWM_FREQ = 20000;
constexpr uint8_t THROTTLE_PWM_RESOLUTION = 8;

constexpr double kArduinoPwmReferenceVolts = 5.0;
constexpr double kEsp32PwmReferenceVolts = 3.3;
constexpr int kArduinoThrottlePwmMin = 60;
constexpr int kArduinoThrottlePwmMax = 150;

constexpr int scaleAnalogWriteValue(int arduinoValue) {
  return static_cast<int>(((arduinoValue * kArduinoPwmReferenceVolts) / kEsp32PwmReferenceVolts) + 0.5);
}

constexpr int THROTTLE_PWM_MIN_DUTY = scaleAnalogWriteValue(kArduinoThrottlePwmMin);
constexpr int THROTTLE_PWM_MAX_DUTY = scaleAnalogWriteValue(kArduinoThrottlePwmMax);
constexpr int THROTTLE_PWM_MAX_VALUE = (1 << THROTTLE_PWM_RESOLUTION) - 1;
static_assert(THROTTLE_PWM_MAX_DUTY <= THROTTLE_PWM_MAX_VALUE, "Throttle max duty exceeds LEDC resolution");
constexpr int THROTTLE_THRESHOLD = 15;

constexpr uint8_t BRAKE_SERVO_PIN_A = 23;
constexpr uint8_t BRAKE_SERVO_PIN_B = 22;
constexpr uint8_t BRAKE_SERVO_CHANNEL_A = 8;
constexpr uint8_t BRAKE_SERVO_CHANNEL_B = 9;
constexpr uint32_t BRAKE_PWM_FREQ = 50;
constexpr uint8_t BRAKE_PWM_RESOLUTION = 16;
constexpr int BRAKE_RELEASE_ANGLE_SERVO_A = 30;
constexpr int BRAKE_APPLY_ANGLE_SERVO_A = 100;
constexpr int BRAKE_RELEASE_ANGLE_SERVO_B = 120;
constexpr int BRAKE_APPLY_ANGLE_SERVO_B = 90;
constexpr int BRAKE_THRESHOLD = -15;

constexpr TickType_t OTA_PERIOD = pdMS_TO_TICKS(20);
constexpr TickType_t RC_SAMPLER_PERIOD = pdMS_TO_TICKS(10);
constexpr TickType_t RC_MONITOR_PERIOD = pdMS_TO_TICKS(100);
constexpr TickType_t AS5600_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t AS5600_LOG_INTERVAL = pdMS_TO_TICKS(500);
constexpr TickType_t PID_PERIOD = pdMS_TO_TICKS(30);
constexpr TickType_t PID_LOG_INTERVAL = pdMS_TO_TICKS(200);
constexpr TickType_t THROTTLE_PERIOD = pdMS_TO_TICKS(30);

namespace debug {
constexpr bool kLogSystem = false;
constexpr bool kLogOta = false;
constexpr bool kLogBridge = false;
constexpr bool kLogLoop = false;
constexpr bool kLogRc = false;
constexpr bool kLogAs5600 = false;
constexpr bool kLogPid = true;
constexpr bool kLogDrive = false;
constexpr bool kLogPiComms = false;
constexpr bool kEnableBridgeTask = false;
constexpr bool kEnableRcTask = false;
constexpr bool kEnablePidTask = true;
constexpr bool kEnableDriveTask = true;
}  // namespace debug

static AS5600 g_as5600;
static PidController g_pidController;

static OtaTelnetTaskConfig g_otaConfig = {debug::kLogOta, pdMS_TO_TICKS(5000), OTA_PERIOD};
static HBridgeTaskConfig g_bridgeConfig = {debug::kLogBridge};
static FsIa6SamplerConfig g_rcSamplerConfig = {
    debug::kLogRc,
    RC_SAMPLER_PERIOD,
    pdMS_TO_TICKS(75),
    pdMS_TO_TICKS(5)};
static FsIa6TaskConfig g_rcConfig = {debug::kLogRc, RC_MONITOR_PERIOD};
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
static QuadDriveTaskConfig g_driveTaskConfig = {
    {
        THROTTLE_PWM_PIN,
        THROTTLE_LEDC_CHANNEL,
        THROTTLE_PWM_FREQ,
        THROTTLE_PWM_RESOLUTION,
        THROTTLE_PWM_MIN_DUTY,
        THROTTLE_PWM_MAX_DUTY,
        THROTTLE_THRESHOLD,
    },
    {
        BRAKE_SERVO_PIN_A,
        BRAKE_SERVO_PIN_B,
        BRAKE_SERVO_CHANNEL_A,
        BRAKE_SERVO_CHANNEL_B,
        BRAKE_PWM_FREQ,
        BRAKE_PWM_RESOLUTION,
        BRAKE_RELEASE_ANGLE_SERVO_A,
        BRAKE_APPLY_ANGLE_SERVO_A,
        BRAKE_RELEASE_ANGLE_SERVO_B,
        BRAKE_APPLY_ANGLE_SERVO_B,
        BRAKE_THRESHOLD,
    },
    true,
    THROTTLE_PERIOD,
    debug::kLogDrive};
static PiCommsConfig g_piCommsConfig = {
    UART_NUM_0,
    1,
    3,
    460800,
    512,
    256,
    pdMS_TO_TICKS(1),
    pdMS_TO_TICKS(10),
    pdMS_TO_TICKS(2),
    debug::kLogPiComms,
    debug::kLogPiComms};

void setup() {
  InicializaWiFi(ssid, contrasena);
  InicializaOTA();
  delay(1000);
  InicializaTelnet();

  g_pidController.setTunings(PID_KP, PID_KI, PID_KD);
  g_pidController.setOutputLimits(-100.0f, 100.0f);
  g_pidController.setIntegralLimits(-PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT);
  g_pidController.reset();

  steeringCalibrationInit(PID_CENTER_DEG, PID_SPAN_DEG);

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
  startTaskPinned(taskRcSampler, "RCSampler", STACK_RC, &g_rcSamplerConfig, 4, nullptr, 1);
  if (debug::kEnableRcTask) {
    startTaskPinned(taskRcMonitor, "RCMonitor", STACK_RC, &g_rcConfig, 1, nullptr, 1);
  }
  if (debug::kEnableDriveTask) {
    startTaskPinned(taskQuadDriveControl, "Drive", STACK_DRIVE, &g_driveTaskConfig, 3, nullptr, 1);
  }

  g_as5600.begin(AS5600_SDA_PIN, AS5600_SCL_PIN);
  runAs5600SelfTest(g_as5600, debug::kLogAs5600);
  startTaskPinned(taskAs5600Monitor, "AS5600", STACK_AS5600, &g_as5600TaskConfig, 1, nullptr, 1);

  if (debug::kEnablePidTask) {
    if (debug::kEnableBridgeTask) {
      broadcastIf(true, "[PID] BridgeTest habilitado; omitiendo tarea PID para evitar conflictos");
    } else {
      broadcastIf(debug::kLogPid, "[PID] Tarea PID iniciada");
      startTaskPinned(taskPidControl, "PID", STACK_PID, &g_pidTaskConfig, 4, nullptr, 0);
    }
  }

  if (piCommsInit(g_piCommsConfig)) {
    startTaskPinned(taskPiCommsRx, "PiUartRx", STACK_PI_RX, &g_piCommsConfig, 3, nullptr, 0);
    startTaskPinned(taskPiCommsTx, "PiUartTx", STACK_PI_TX, &g_piCommsConfig, 3, nullptr, 0);
  } else {
    broadcastIf(true, "[PI][UART] Error inicializando UART0 para Raspberry Pi");
  }
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(50));
}

