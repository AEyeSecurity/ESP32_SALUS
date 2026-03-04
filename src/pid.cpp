#include "pid.h"

#include <math.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <esp_timer.h>

#include "AS5600.h"
#include "fs_ia6.h"
#include "h_bridge.h"
#include "ota_telnet.h"
#include "steering_calibration.h"
#include "pi_comms.h"
#include "system_diag.h"

constexpr float kFullRotationDegrees = 360.0f;  // Degrees in one full turn for normalization

namespace {
float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

float applyDeadband(float value, float deadband) {
  if (deadband <= 0.0f) {
    return value;
  }
  if (fabsf(value) < deadband) {
    return 0.0f;
  }
  return value;
}

float applyMinActive(float value, float minActive) {
  if (minActive <= 0.0f) {
    return value;
  }
  const float magnitude = fabsf(value);
  if (magnitude > 0.0f && magnitude < minActive) {
    return (value > 0.0f) ? minActive : -minActive;
  }
  return value;
}

int applySteeringCommandDeadband(int commandPercent, int deadbandPercent) {
  int clamped = commandPercent;
  if (clamped < -100) {
    clamped = -100;
  } else if (clamped > 100) {
    clamped = 100;
  }
  if (deadbandPercent <= 0) {
    return clamped;
  }
  if (deadbandPercent > 100) {
    deadbandPercent = 100;
  }
  if (clamped >= -deadbandPercent && clamped <= deadbandPercent) {
    return 0;
  }
  return clamped;
}

float computeDtSeconds(uint32_t prevMicros, uint32_t currentMicros) {
  if (currentMicros >= prevMicros) {
    return static_cast<float>(currentMicros - prevMicros) * 1e-6f;
  }
  return static_cast<float>((0xFFFFFFFFu - prevMicros) + currentMicros + 1u) * 1e-6f;
}

float ticksToSeconds(TickType_t ticks) {
  const float tickPeriodMs = static_cast<float>(portTICK_PERIOD_MS);
  return (static_cast<float>(ticks) * tickPeriodMs) / 1000.0f;
}

constexpr TickType_t kPiSnapshotFreshTicks = pdMS_TO_TICKS(120);
portMUX_TYPE g_pidMux = portMUX_INITIALIZER_UNLOCKED;
PidController* g_pidController = nullptr;
PidTaskConfig* g_pidConfig = nullptr;
PidRuntimeSnapshot g_pidRuntimeSnapshot{};
}  // namespace

namespace {
constexpr uint8_t kCalibrationDutyPercent = 65;
constexpr uint8_t kCalibrationReleaseDutyPercent = 30;
constexpr float kCalibrationStallThresholdDeg = 0.5f;
constexpr int kRcSteeringCommandDeadbandPercent = 4;
constexpr int kPiSteeringCommandDeadbandPercent = 1;
const TickType_t kRcFreshThreshold = pdMS_TO_TICKS(50);
const TickType_t kRcNeutralCaptureWindow = pdMS_TO_TICKS(800);
constexpr int kRcNeutralCaptureMaxAbsCommand = 30;
constexpr int kRcNeutralCaptureMinSamples = 8;
constexpr float kNeutralHoldErrorDeg = 0.6f;
const TickType_t kCalibrationTimeout = pdMS_TO_TICKS(10000);
const TickType_t kCalibrationReleaseDuration = pdMS_TO_TICKS(250);
const TickType_t kCalibrationStallDuration = pdMS_TO_TICKS(1200);
const TickType_t kCalibrationDebugInterval = pdMS_TO_TICKS(250);
}  // namespace

PidController::PidController()
    : kp_(0.0f),
      ki_(0.0f),
      kd_(0.0f),
      outMin_(-100.0f),
      outMax_(100.0f),
      integralTerm_(0.0f),
      integralMin_(-100.0f),
      integralMax_(100.0f),
      prevError_(0.0f),
      firstRun_(true),
      enabled_(true) {}

void PidController::setTunings(float kp, float ki, float kd) {
  kp_ = kp;
  ki_ = ki;
  kd_ = kd;
}

void PidController::setOutputLimits(float minOutput, float maxOutput) {
  if (minOutput > maxOutput) {
    const float tmp = minOutput;
    minOutput = maxOutput;
    maxOutput = tmp;
  }
  outMin_ = minOutput;
  outMax_ = maxOutput;
  integralTerm_ = clampf(integralTerm_, integralMin_, integralMax_);
}

void PidController::setIntegralLimits(float minIntegral, float maxIntegral) {
  if (minIntegral > maxIntegral) {
    const float tmp = minIntegral;
    minIntegral = maxIntegral;
    maxIntegral = tmp;
  }
  integralMin_ = minIntegral;
  integralMax_ = maxIntegral;
  integralTerm_ = clampf(integralTerm_, integralMin_, integralMax_);
}

void PidController::setEnabled(bool enabled) {
  enabled_ = enabled;
  if (!enabled_) {
    reset();
  }
}

bool PidController::enabled() const {
  return enabled_;
}

void PidController::reset() {
  integralTerm_ = 0.0f;
  prevError_ = 0.0f;
  firstRun_ = true;
}

float PidController::update(float error, float dtSeconds) {
  if (!enabled_) {
    return 0.0f;
  }

  if (dtSeconds <= 0.0f) {
    dtSeconds = 1e-3f;
  }

  integralTerm_ += ki_ * error * dtSeconds;
  integralTerm_ = clampf(integralTerm_, integralMin_, integralMax_);

  float derivative = 0.0f;
  if (!firstRun_ && dtSeconds > 0.0f) {
    derivative = (error - prevError_) / dtSeconds;
  }

  float output = (kp_ * error) + integralTerm_ + (kd_ * derivative);
  output = clampf(output, outMin_, outMax_);

  prevError_ = error;
  firstRun_ = false;

  return output;
}

float normalizeAngleDegrees(float angle) {
  angle = fmodf(angle, kFullRotationDegrees);
  if (angle < 0.0f) {
    angle += kFullRotationDegrees;
  }
  return angle;
}

float wrapAngleDegrees(float angle) {
  angle = fmodf(angle, kFullRotationDegrees);
  if (angle > 180.0f) {
    angle -= kFullRotationDegrees;
  } else if (angle < -180.0f) {
    angle += kFullRotationDegrees;
  }
  return angle;
}

float computeAngleError(float setpointDeg, float measurementDeg) {
  const float normalizedSetpoint = normalizeAngleDegrees(setpointDeg);
  const float normalizedMeasurement = normalizeAngleDegrees(measurementDeg);
  return wrapAngleDegrees(normalizedSetpoint - normalizedMeasurement);
}

float mapRcValueToAngle(int rcValue,
                        const SteeringCalibrationData& calibration,
                        float fallbackCenterDeg,
                        float fallbackSpanDeg) {
  const float normalizedFallbackCenter = normalizeAngleDegrees(fallbackCenterDeg);
  float center = normalizedFallbackCenter;
  float leftLimit = normalizeAngleDegrees(fallbackCenterDeg - fallbackSpanDeg);
  float rightLimit = normalizeAngleDegrees(fallbackCenterDeg + fallbackSpanDeg);
  bool useCalibration = calibration.initialized;

  if (useCalibration) {
    center = normalizeAngleDegrees(calibration.adjustedCenterDeg);
    leftLimit = normalizeAngleDegrees(calibration.leftLimitDeg);
    rightLimit = normalizeAngleDegrees(calibration.rightLimitDeg);
  }

  float rcNorm = clampf(static_cast<float>(rcValue), -100.0f, 100.0f) / 100.0f;
  float leftSpan = useCalibration ? fabsf(wrapAngleDegrees(center - leftLimit)) : fabsf(fallbackSpanDeg);
  float rightSpan =
      useCalibration ? fabsf(wrapAngleDegrees(rightLimit - center)) : fabsf(fallbackSpanDeg);

  if (leftSpan < 0.001f) {
    leftSpan = fabsf(fallbackSpanDeg);
  }
  if (rightSpan < 0.001f) {
    rightSpan = fabsf(fallbackSpanDeg);
  }

  float target = center;
  if (rcNorm > 0.0f) {
    target = center + rightSpan * rcNorm;
  } else if (rcNorm < 0.0f) {
    target = center + leftSpan * rcNorm;
  }
  return normalizeAngleDegrees(target);
}

void pidRegisterController(PidController* controller) {
  portENTER_CRITICAL(&g_pidMux);
  g_pidController = controller;
  portEXIT_CRITICAL(&g_pidMux);
}

void pidRegisterConfig(PidTaskConfig* config) {
  portENTER_CRITICAL(&g_pidMux);
  g_pidConfig = config;
  portEXIT_CRITICAL(&g_pidMux);
}

bool pidGetTunings(float& kp, float& ki, float& kd) {
  portENTER_CRITICAL(&g_pidMux);
  PidController* controller = g_pidController;
  if (controller == nullptr) {
    portEXIT_CRITICAL(&g_pidMux);
    return false;
  }
  kp = controller->kp();
  ki = controller->ki();
  kd = controller->kd();
  portEXIT_CRITICAL(&g_pidMux);
  return true;
}

bool pidSetTunings(float kp, float ki, float kd) {
  portENTER_CRITICAL(&g_pidMux);
  PidController* controller = g_pidController;
  if (controller == nullptr) {
    portEXIT_CRITICAL(&g_pidMux);
    return false;
  }
  controller->setTunings(kp, ki, kd);
  portEXIT_CRITICAL(&g_pidMux);
  return true;
}

bool pidGetOutputModifiers(float& deadbandPercent, float& minActivePercent) {
  portENTER_CRITICAL(&g_pidMux);
  PidTaskConfig* config = g_pidConfig;
  if (config == nullptr) {
    portEXIT_CRITICAL(&g_pidMux);
    return false;
  }
  deadbandPercent = config->deadbandPercent;
  minActivePercent = config->minActivePercent;
  portEXIT_CRITICAL(&g_pidMux);
  return true;
}

bool pidSetDeadband(float deadbandPercent) {
  portENTER_CRITICAL(&g_pidMux);
  PidTaskConfig* config = g_pidConfig;
  if (config == nullptr) {
    portEXIT_CRITICAL(&g_pidMux);
    return false;
  }
  config->deadbandPercent = deadbandPercent;
  portEXIT_CRITICAL(&g_pidMux);
  return true;
}

bool pidSetMinActive(float minActivePercent) {
  portENTER_CRITICAL(&g_pidMux);
  PidTaskConfig* config = g_pidConfig;
  if (config == nullptr) {
    portEXIT_CRITICAL(&g_pidMux);
    return false;
  }
  config->minActivePercent = minActivePercent;
  portEXIT_CRITICAL(&g_pidMux);
  return true;
}

bool pidGetRuntimeSnapshot(PidRuntimeSnapshot& snapshot) {
  portENTER_CRITICAL(&g_pidMux);
  snapshot = g_pidRuntimeSnapshot;
  portEXIT_CRITICAL(&g_pidMux);
  return snapshot.valid;
}

namespace {

enum class CalibrationState : uint8_t { Idle, MoveLeft, ReleaseLeft, MoveRight, ReleaseRight };

struct PidCalibrationContext {
  CalibrationState state = CalibrationState::Idle;
  CalibrationState lastState = CalibrationState::Idle;
  TickType_t stateStartTick = 0;
  float leftAngleDeg = 0.0f;
  float rightAngleDeg = 0.0f;
  bool active = false;
  float lastAngleDeg = 0.0f;
  TickType_t lastMoveTick = 0;
  bool motionValid = false;
  TickType_t lastDebugTick = 0;
};

const char* calibrationStateName(CalibrationState state) {
  switch (state) {
    case CalibrationState::MoveLeft:
      return "MoveLeft";
    case CalibrationState::ReleaseLeft:
      return "ReleaseLeft";
    case CalibrationState::MoveRight:
      return "MoveRight";
    case CalibrationState::ReleaseRight:
      return "ReleaseRight";
    case CalibrationState::Idle:
    default:
      return "Idle";
  }
}

void publishPidRuntimeSnapshot(const PidRuntimeSnapshot& snapshot) {
  portENTER_CRITICAL(&g_pidMux);
  g_pidRuntimeSnapshot = snapshot;
  portEXIT_CRITICAL(&g_pidMux);
}

void finishCalibrationAndReport(float leftAngleDeg, float rightAngleDeg, float measuredDeg) {
  if (measuredDeg >= 0.0f) {
    steeringCalibrationApply(leftAngleDeg, rightAngleDeg);
    String msg = "[PID] Calibracion completa. Izq=";
    msg += String(leftAngleDeg, 2);
    msg += "deg, Der=";
    msg += String(rightAngleDeg, 2);
    msg += "deg";
    broadcastIf(true, msg);
  } else {
    broadcastIf(true, "[PID] Calibracion completa pero sin lectura de angulo valida");
  }
}

bool calibrationStallDetected(PidCalibrationContext& calibration, TickType_t nowTicks, float measuredDeg) {
  if (measuredDeg < 0.0f) {
    return false;
  }
  if (!calibration.motionValid) {
    calibration.motionValid = true;
    calibration.lastAngleDeg = measuredDeg;
    calibration.lastMoveTick = nowTicks;
    return false;
  }
  if (fabsf(measuredDeg - calibration.lastAngleDeg) > kCalibrationStallThresholdDeg) {
    calibration.lastAngleDeg = measuredDeg;
    calibration.lastMoveTick = nowTicks;
    return false;
  }
  return (nowTicks - calibration.lastMoveTick) >= kCalibrationStallDuration;
}

void stopCalibrationSession(PidCalibrationContext& calibration, PidController* controller) {
  bridge_stop();
  calibration.active = false;
  calibration.state = CalibrationState::Idle;
  calibration.lastState = CalibrationState::Idle;
  calibration.stateStartTick = 0;
  calibration.motionValid = false;
  calibration.lastDebugTick = 0;
  controller->reset();
}

void startCalibrationSession(PidCalibrationContext& calibration,
                             TickType_t nowTicks,
                             PidController* controller,
                             bool& bridgeEnabled) {
  calibration.active = true;
  calibration.state = CalibrationState::MoveRight;
  calibration.lastState = CalibrationState::Idle;
  calibration.stateStartTick = nowTicks;
  calibration.leftAngleDeg = 0.0f;
  calibration.rightAngleDeg = 0.0f;
  calibration.motionValid = false;
  calibration.lastMoveTick = nowTicks;
  calibration.lastDebugTick = 0;
  controller->reset();
  if (!bridgeEnabled) {
    enable_bridge_h();
    bridgeEnabled = true;
  }
  broadcastIf(true, "[PID] Iniciando calibracion de limites de direccion");
}

void emitCalibrationDebug(PidCalibrationContext& calibration,
                          TickType_t nowTicks,
                          float measuredDeg,
                          bool limitLeftActive,
                          bool limitRightActive) {
  if ((nowTicks - calibration.lastDebugTick) < kCalibrationDebugInterval) {
    return;
  }
  String dbg = "[PID][CAL] estado=";
  dbg += calibrationStateName(calibration.state);
  dbg += " limL=";
  dbg += limitLeftActive ? "1" : "0";
  dbg += " limR=";
  dbg += limitRightActive ? "1" : "0";
  dbg += " ang=";
  if (measuredDeg >= 0.0f) {
    dbg += String(measuredDeg, 2);
    dbg += "deg";
  } else {
    dbg += "ERR";
  }
  dbg += " t=";
  dbg += static_cast<int>((nowTicks - calibration.stateStartTick) * portTICK_PERIOD_MS);
  dbg += "ms";
  broadcastIf(true, dbg);
  calibration.lastDebugTick = nowTicks;
}

bool processCalibrationStep(PidCalibrationContext& calibration,
                            PidTaskConfig* cfg,
                            TickType_t nowTicks,
                            float measuredDeg,
                            bool limitLeftActive,
                            bool limitRightActive) {
  if (calibration.state != calibration.lastState) {
    calibration.motionValid = false;
    calibration.lastMoveTick = nowTicks;
    calibration.lastState = calibration.state;
  }

  bool keepCalibrating = true;
  switch (calibration.state) {
    case CalibrationState::MoveLeft: {
      if (measuredDeg < 0.0f) {
        broadcastIf(true, "[PID] Calibracion abortada: lectura AS5600 invalida");
        keepCalibrating = false;
        break;
      }
      bridge_turn_left(kCalibrationDutyPercent);
      if (calibrationStallDetected(calibration, nowTicks, measuredDeg)) {
        calibration.leftAngleDeg = (measuredDeg >= 0.0f) ? measuredDeg : calibration.lastAngleDeg;
        String msg = "[PID] Calibracion: limite izquierdo inferido por estancamiento (sin FC)";
        msg += " ang=";
        msg += String(calibration.leftAngleDeg, 2);
        msg += "deg";
        broadcastIf(true, msg);
        bridge_stop();
        calibration.state = CalibrationState::ReleaseLeft;
        calibration.stateStartTick = nowTicks;
        break;
      }
      if (limitLeftActive) {
        calibration.leftAngleDeg = measuredDeg;
        bridge_stop();
        calibration.state = CalibrationState::ReleaseLeft;
        calibration.stateStartTick = nowTicks;
        String msg = "[PID] Calibracion: limite izquierdo registrado en ";
        msg += String(calibration.leftAngleDeg, 2);
        msg += "deg";
        broadcastIf(true, msg);
      } else if ((nowTicks - calibration.stateStartTick) >= kCalibrationTimeout) {
        broadcastIf(true, "[PID] Calibracion abortada: timeout alcanzando limite izquierdo");
        keepCalibrating = false;
      }
      break;
    }
    case CalibrationState::ReleaseLeft: {
      if (!limitLeftActive && (nowTicks - calibration.stateStartTick) >= pdMS_TO_TICKS(50)) {
        bridge_stop();
        finishCalibrationAndReport(calibration.leftAngleDeg, calibration.rightAngleDeg, measuredDeg);
        keepCalibrating = false;
      } else if ((nowTicks - calibration.stateStartTick) >= kCalibrationReleaseDuration) {
        broadcastIf(
            true,
            "[PID] Calibracion: forzando finalizacion tras liberar el limite izquierdo (timeout)");
        bridge_stop();
        finishCalibrationAndReport(calibration.leftAngleDeg, calibration.rightAngleDeg, measuredDeg);
        keepCalibrating = false;
      } else {
        bridge_turn_right(kCalibrationReleaseDutyPercent);
      }
      break;
    }
    case CalibrationState::MoveRight: {
      if (measuredDeg < 0.0f) {
        broadcastIf(true, "[PID] Calibracion abortada: lectura AS5600 invalida");
        keepCalibrating = false;
        break;
      }
      bridge_turn_right(kCalibrationDutyPercent);
      if (calibrationStallDetected(calibration, nowTicks, measuredDeg)) {
        calibration.rightAngleDeg = (measuredDeg >= 0.0f) ? measuredDeg : calibration.lastAngleDeg;
        bridge_stop();
        calibration.state = CalibrationState::ReleaseRight;
        calibration.stateStartTick = nowTicks;
        String msg = "[PID] Calibracion: limite derecho inferido por estancamiento (sin FC)";
        msg += " ang=";
        msg += String(calibration.rightAngleDeg, 2);
        msg += "deg";
        broadcastIf(true, msg);
        break;
      }
      if (limitRightActive) {
        calibration.rightAngleDeg = measuredDeg;
        bridge_stop();
        calibration.state = CalibrationState::ReleaseRight;
        calibration.stateStartTick = nowTicks;
        String msg = "[PID] Calibracion: limite derecho registrado en ";
        msg += String(calibration.rightAngleDeg, 2);
        msg += "deg";
        broadcastIf(true, msg);
      } else if ((nowTicks - calibration.stateStartTick) >= kCalibrationTimeout) {
        String msg = "[PID] Calibracion abortada: timeout alcanzando limite derecho (limL=";
        msg += limitLeftActive ? "1" : "0";
        msg += ", limR=";
        msg += limitRightActive ? "1" : "0";
        msg += ", ang=";
        if (measuredDeg >= 0.0f) {
          msg += String(measuredDeg, 2);
          msg += "deg";
        } else {
          msg += "ERR";
        }
        msg += ")";
        broadcastIf(true, msg);
        keepCalibrating = false;
      }
      break;
    }
    case CalibrationState::ReleaseRight: {
      if (!limitRightActive && (nowTicks - calibration.stateStartTick) >= pdMS_TO_TICKS(50)) {
        bridge_stop();
        calibration.state = CalibrationState::MoveLeft;
        calibration.stateStartTick = nowTicks;
        broadcastIf(true, "[PID] Calibracion: iniciando busqueda de limite izquierdo");
      } else if ((nowTicks - calibration.stateStartTick) >= kCalibrationReleaseDuration) {
        bridge_stop();
        calibration.state = CalibrationState::MoveLeft;
        calibration.stateStartTick = nowTicks;
        broadcastIf(
            true,
            "[PID] Calibracion: forzando busqueda de limite izquierdo (timeout liberando derecho)");
      } else {
        bridge_turn_left(kCalibrationReleaseDutyPercent);
      }
      break;
    }
    case CalibrationState::Idle:
    default:
      keepCalibrating = false;
      break;
  }

  if (!keepCalibrating) {
    stopCalibrationSession(calibration, cfg->controller);
  }
  return calibration.active;
}

}  // namespace

void taskPidControl(void* parameter) {
  PidTaskConfig* cfg = static_cast<PidTaskConfig*>(parameter);
  if (cfg == nullptr || cfg->sensor == nullptr || cfg->controller == nullptr) {
    broadcastIf(true, "[PID] Configuracion invalida, finalizando tarea");
    vTaskDelete(nullptr);
    return;
  }

  const TickType_t period = (cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  const TickType_t logInterval = (cfg->logInterval > 0) ? cfg->logInterval : pdMS_TO_TICKS(200);
  TickType_t lastLog = xTaskGetTickCount();

  if (!rcRegisterConsumer(xTaskGetCurrentTaskHandle())) {
    broadcastIf(true, "[PID] No se pudo registrar la tarea para notificaciones RC");
  }

  if (cfg->autoInitBridge) {
    init_h_bridge();
    enable_bridge_h();
  }

  bool bridgeEnabled = cfg->autoInitBridge;
  uint32_t lastMicros = micros();
  bool leftLimitLatched = false;
  bool rightLimitLatched = false;
  float expectedPeriodSeconds = ticksToSeconds(period);
  if (expectedPeriodSeconds <= 0.0f) {
    expectedPeriodSeconds = 0.001f;
  }
  const uint32_t expectedPeriodUs = static_cast<uint32_t>(expectedPeriodSeconds * 1000000.0f + 0.5f);
  const float dtOverrunThreshold = expectedPeriodSeconds + 0.010f;
  const TickType_t dtWarningCooldown = pdMS_TO_TICKS(500);
  const TickType_t runtimeWarningCooldown = pdMS_TO_TICKS(1000);
  TickType_t lastDtWarningTick = 0;
  TickType_t lastRuntimeWarningTick = 0;
  int64_t lastIterationStartUs = esp_timer_get_time();

  portENTER_CRITICAL(&g_pidMux);
  g_pidRuntimeSnapshot = PidRuntimeSnapshot{};
  g_pidRuntimeSnapshot.valid = true;
  portEXIT_CRITICAL(&g_pidMux);

  PidCalibrationContext calibration{};
  bool rcWasFresh = false;
  bool rcNeutralCaptureActive = false;
  TickType_t rcNeutralCaptureStartTick = 0;
  int rcNeutralSampleSum = 0;
  int rcNeutralSampleCount = 0;
  int rcSteeringNeutralOffset = 0;

  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
    uint32_t cycleUs = 0;
    if (iterationStartUs > lastIterationStartUs) {
      cycleUs = static_cast<uint32_t>(iterationStartUs - lastIterationStartUs);
    }
    lastIterationStartUs = iterationStartUs;
    const uint32_t notificationCount = ulTaskNotifyTake(pdTRUE, period);
    (void)notificationCount;

    const uint32_t nowMicros = micros();
    float dtSeconds = computeDtSeconds(lastMicros, nowMicros);
    lastMicros = nowMicros;

    TickType_t nowTicks = xTaskGetTickCount();
    if (dtSeconds <= 0.0f || dtSeconds > 1.0f) {
      dtSeconds = expectedPeriodSeconds;
    }

    if (cfg->log && dtSeconds > dtOverrunThreshold) {
      if ((nowTicks - lastDtWarningTick) >= dtWarningCooldown) {
        String warn;
        warn.reserve(80);
        warn += "[PID] dt=";
        warn += dtSeconds * 1000.0f;
        warn += "ms (objetivo ";
        warn += expectedPeriodSeconds * 1000.0f;
        warn += "ms)";
        broadcastIf(true, warn);
        lastDtWarningTick = nowTicks;
      }
    }

    if (!calibration.active && steeringCalibrationConsumeRequest()) {
      startCalibrationSession(calibration, nowTicks, cfg->controller, bridgeEnabled);
    }

    RcSharedState rcSnapshot{};
    const bool rcValid = rcGetStateCopy(rcSnapshot);
    const bool rcFresh = rcValid && rcSnapshot.valid && (nowTicks - rcSnapshot.lastUpdateTick) <= kRcFreshThreshold;
    if (!rcFresh) {
      rcSnapshot.steering = 0;
    }
    const int rcValue = rcSnapshot.steering;
    int rcSteeringCorrected = rcValue;

    if (!rcWasFresh && rcFresh) {
      rcNeutralCaptureActive = true;
      rcNeutralCaptureStartTick = nowTicks;
      rcNeutralSampleSum = 0;
      rcNeutralSampleCount = 0;
    }

    if (rcNeutralCaptureActive) {
      if (!rcFresh) {
        rcNeutralCaptureActive = false;
      } else if ((nowTicks - rcNeutralCaptureStartTick) <= kRcNeutralCaptureWindow) {
        const int absRcValue = (rcValue >= 0) ? rcValue : -rcValue;
        if (absRcValue <= kRcNeutralCaptureMaxAbsCommand) {
          rcNeutralSampleSum += rcValue;
          rcNeutralSampleCount++;
        }
      } else {
        if (rcNeutralSampleCount >= kRcNeutralCaptureMinSamples) {
          rcSteeringNeutralOffset = static_cast<int>(
              lroundf(static_cast<float>(rcNeutralSampleSum) / static_cast<float>(rcNeutralSampleCount)));
          String msg = "[PID] RC neutral capturado offset=";
          msg += rcSteeringNeutralOffset;
          msg += " (muestras=";
          msg += rcNeutralSampleCount;
          msg += ")";
          broadcastIf(true, msg);
        }
        rcNeutralCaptureActive = false;
      }
    }
    rcWasFresh = rcFresh;

    if (rcFresh) {
      rcSteeringCorrected = rcValue - rcSteeringNeutralOffset;
      if (rcSteeringCorrected < -100) {
        rcSteeringCorrected = -100;
      } else if (rcSteeringCorrected > 100) {
        rcSteeringCorrected = 100;
      }
    }

    PiCommsRxSnapshot piSnapshot{};
    const bool piDriverReady = piCommsGetRxSnapshot(piSnapshot);
    TickType_t piAgeTicks = 0;
    bool piAgeValid = false;
    if (piSnapshot.lastFrameTick != 0) {
      piAgeTicks = nowTicks - piSnapshot.lastFrameTick;
      piAgeValid = true;
    }
    const bool piFresh =
        piDriverReady && piSnapshot.hasFrame && piAgeValid && piAgeTicks <= kPiSnapshotFreshTicks;

    int steeringCommand = rcSteeringCorrected;
    bool steeringFromPi = false;
    if (piFresh) {
      steeringCommand = piSnapshot.steer;
      steeringFromPi = true;
    }
    const int steeringCommandRaw = steeringCommand;
    steeringCommand = applySteeringCommandDeadband(
        steeringCommand,
        steeringFromPi ? kPiSteeringCommandDeadbandPercent : kRcSteeringCommandDeadbandPercent);

    const float measuredDeg = cfg->sensor->getAngleDegrees();

    const bool limitLeftActive = bridge_limit_left_active();
    const bool limitRightActive = bridge_limit_right_active();

    PidRuntimeSnapshot runtimeSnapshot{};
    runtimeSnapshot.valid = true;
    runtimeSnapshot.calibrationActive = calibration.active;
    runtimeSnapshot.steeringFromPi = steeringFromPi;
    runtimeSnapshot.sensorValid = measuredDeg >= 0.0f;
    runtimeSnapshot.limitLeftActive = limitLeftActive;
    runtimeSnapshot.limitRightActive = limitRightActive;
    runtimeSnapshot.steeringCommand = steeringCommand;
    runtimeSnapshot.dtSeconds = dtSeconds;
    runtimeSnapshot.measuredDeg = measuredDeg;
    runtimeSnapshot.targetDeg = 0.0f;
    runtimeSnapshot.errorDeg = 0.0f;
    runtimeSnapshot.outputPercent = 0.0f;

    const bool shouldLog = cfg->log && (logInterval == 0 || (nowTicks - lastLog) >= logInterval);

    if (calibration.active) {
      emitCalibrationDebug(calibration, nowTicks, measuredDeg, limitLeftActive, limitRightActive);
      runtimeSnapshot.calibrationActive =
          processCalibrationStep(calibration, cfg, nowTicks, measuredDeg, limitLeftActive, limitRightActive);
      publishPidRuntimeSnapshot(runtimeSnapshot);
      const bool overrun = cycleUs > expectedPeriodUs;
      systemDiagReportLoop(SystemDiagTaskId::kPid, cycleUs, expectedPeriodUs, overrun, notificationCount == 0);
      continue;
    }

    const SteeringCalibrationData calibrationData = steeringCalibrationSnapshot();
    const float targetDeg =
        mapRcValueToAngle(steeringCommand, calibrationData, cfg->centerDeg, cfg->spanDeg);
    runtimeSnapshot.targetDeg = targetDeg;

    bool skipControl = false;
    if (measuredDeg < 0.0f) {
      cfg->controller->reset();
      bridge_stop();
      skipControl = true;
      if (shouldLog) {
        broadcastIf(true, "[PID] Lectura AS5600 invalida, deteniendo puente H");
        lastLog = nowTicks;
      }
    }

    float outputPercent = 0.0f;
    float errorDeg = 0.0f;

    if (!skipControl) {
      errorDeg = computeAngleError(targetDeg, measuredDeg);
      outputPercent = cfg->controller->update(errorDeg, dtSeconds);
      outputPercent = applyDeadband(outputPercent, cfg->deadbandPercent);
      outputPercent = applyMinActive(outputPercent, cfg->minActivePercent);
      outputPercent = clampf(outputPercent, -100.0f, 100.0f);

      const bool neutralCommand = steeringCommand == 0;
      if (neutralCommand && fabsf(errorDeg) <= kNeutralHoldErrorDeg) {
        cfg->controller->reset();
        outputPercent = 0.0f;
      }

      bool blockedByLimit = false;
      if (limitLeftActive && outputPercent < -0.001f) {
        outputPercent = 0.0f;
        blockedByLimit = true;
        if (!leftLimitLatched) {
          broadcastIf(true, "[PID] Final de carrera izquierda activo; deteniendo giro hacia la izquierda");
          leftLimitLatched = true;
        }
      } else if (!limitLeftActive) {
        leftLimitLatched = false;
      }

      if (limitRightActive && outputPercent > 0.001f) {
        outputPercent = 0.0f;
        blockedByLimit = true;
        if (!rightLimitLatched) {
          broadcastIf(true, "[PID] Final de carrera derecha activo; deteniendo giro hacia la derecha");
          rightLimitLatched = true;
        }
      } else if (!limitRightActive) {
        rightLimitLatched = false;
      }

      if (blockedByLimit) {
        cfg->controller->reset();
      }

      if (fabsf(outputPercent) < 0.001f) {
        bridge_stop();
      } else if (outputPercent > 0.0f) {
        if (!bridgeEnabled) {
          enable_bridge_h();
          bridgeEnabled = true;
        }
        bridge_turn_right(static_cast<uint8_t>(outputPercent));
      } else {  // outputPercent < 0
        if (!bridgeEnabled) {
          enable_bridge_h();
          bridgeEnabled = true;
        }
        bridge_turn_left(static_cast<uint8_t>(-outputPercent));
      }

      if (shouldLog) {
        String msg;
        msg.reserve(160);
        msg += "[PID] sensor=";
        msg += String(measuredDeg, 2);
        msg += "deg";
        msg += " center=";
        msg += String(calibrationData.adjustedCenterDeg, 2);
        msg += "deg";
        msg += " cmd=";
        msg += steeringCommand;
        if (steeringCommandRaw != steeringCommand) {
          msg += " raw=";
          msg += steeringCommandRaw;
        }
        msg += steeringFromPi ? " (PI" : " (RC";
        if (steeringFromPi && piAgeValid) {
          msg += " ageMs=";
          msg += static_cast<int>(piAgeTicks * portTICK_PERIOD_MS);
        }
        msg += ")";
        msg += " rcGPIO16=";
        msg += rcValue;
        msg += " target=";
        msg += String(targetDeg, 2);
        msg += "deg";
        msg += " error=";
        msg += String(errorDeg, 2);
        msg += "deg";
        msg += " salida=";
        msg += String(outputPercent, 1);
        msg += "%";
        broadcastIf(true, msg);
        lastLog = nowTicks;
      }
    }

    runtimeSnapshot.errorDeg = errorDeg;
    runtimeSnapshot.outputPercent = outputPercent;
    publishPidRuntimeSnapshot(runtimeSnapshot);

    const int64_t iterationDurationUs = esp_timer_get_time() - iterationStartUs;
    if (cfg->log && iterationDurationUs > 4000) {
      if ((nowTicks - lastRuntimeWarningTick) >= runtimeWarningCooldown) {
        String perf;
        perf.reserve(64);
        perf += "[PID] ciclo tardo ";
        perf += iterationDurationUs / 1000.0f;
        perf += "ms";
        broadcastIf(true, perf);
        lastRuntimeWarningTick = nowTicks;
      }
    }
    const bool overrun = cycleUs > expectedPeriodUs;
    systemDiagReportLoop(SystemDiagTaskId::kPid, cycleUs, expectedPeriodUs, overrun, notificationCount == 0);
  }
}
