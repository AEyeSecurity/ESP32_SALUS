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
}  // namespace

namespace {
constexpr uint8_t kCalibrationDutyPercent = 65;
constexpr uint8_t kCalibrationReleaseDutyPercent = 30;
constexpr float kCalibrationStallThresholdDeg = 0.5f;
const TickType_t kCalibrationTimeout = pdMS_TO_TICKS(10000);
const TickType_t kCalibrationReleaseDuration = pdMS_TO_TICKS(250);
const TickType_t kCalibrationStallDuration = pdMS_TO_TICKS(1200);
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
  const float dtOverrunThreshold = expectedPeriodSeconds + 0.010f;
  const TickType_t dtWarningCooldown = pdMS_TO_TICKS(500);
  const TickType_t runtimeWarningCooldown = pdMS_TO_TICKS(1000);
  const TickType_t calibrationDebugInterval = pdMS_TO_TICKS(250);
  TickType_t lastDtWarningTick = 0;
  TickType_t lastRuntimeWarningTick = 0;
  TickType_t lastCalibrationDebugTick = 0;

  enum class CalibrationState { Idle, MoveLeft, ReleaseLeft, MoveRight, ReleaseRight };
  CalibrationState calibrationState = CalibrationState::Idle;
  CalibrationState lastCalibrationState = CalibrationState::Idle;
  TickType_t calibrationStateStart = 0;
  float calibrationLeftAngleDeg = 0.0f;
  float calibrationRightAngleDeg = 0.0f;
  bool calibrationActive = false;
  float calibrationLastAngleDeg = 0.0f;
  TickType_t calibrationLastMoveTick = 0;
  bool calibrationMotionValid = false;

  auto calibrationStateName = [](CalibrationState state) -> const char* {
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
  };

  for (;;) {
    const int64_t iterationStartUs = esp_timer_get_time();
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

    if (!calibrationActive && steeringCalibrationConsumeRequest()) {
      calibrationActive = true;
      calibrationState = CalibrationState::MoveRight;
      lastCalibrationState = CalibrationState::Idle;
      calibrationStateStart = nowTicks;
      calibrationLeftAngleDeg = 0.0f;
      calibrationRightAngleDeg = 0.0f;
      calibrationMotionValid = false;
      calibrationLastMoveTick = nowTicks;
      cfg->controller->reset();
      if (!bridgeEnabled) {
        enable_bridge_h();
        bridgeEnabled = true;
      }
      broadcastIf(true, "[PID] Iniciando calibracion de limites de direccion");
    }

    RcSharedState rcSnapshot{};
    const bool rcValid = rcGetStateCopy(rcSnapshot);
    if (!rcValid || !rcSnapshot.valid || (nowTicks - rcSnapshot.lastUpdateTick) > pdMS_TO_TICKS(50)) {
      rcSnapshot.steering = 0;
    }
    const int rcValue = rcSnapshot.steering;

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

    int steeringCommand = rcValue;
    bool steeringFromPi = false;
    if (piFresh) {
      steeringCommand = piSnapshot.steer;
      steeringFromPi = true;
    }

    const float measuredDeg = cfg->sensor->getAngleDegrees();

    const bool limitLeftActive = bridge_limit_left_active();
    const bool limitRightActive = bridge_limit_right_active();

    const bool shouldLog = cfg->log && (logInterval == 0 || (nowTicks - lastLog) >= logInterval);

    if (calibrationActive && calibrationState != lastCalibrationState) {
      calibrationMotionValid = false;
      calibrationLastMoveTick = nowTicks;
      lastCalibrationState = calibrationState;
    }

    if (calibrationActive) {
      if ((nowTicks - lastCalibrationDebugTick) >= calibrationDebugInterval) {
        String dbg = "[PID][CAL] estado=";
        dbg += calibrationStateName(calibrationState);
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
        dbg += static_cast<int>((nowTicks - calibrationStateStart) * portTICK_PERIOD_MS);
        dbg += "ms";
        broadcastIf(true, dbg);
        lastCalibrationDebugTick = nowTicks;
      }
      bool keepCalibrating = true;
      switch (calibrationState) {
        case CalibrationState::MoveLeft: {
          if (measuredDeg < 0.0f) {
            broadcastIf(true, "[PID] Calibracion abortada: lectura AS5600 invalida");
            keepCalibrating = false;
            break;
          }
          bridge_turn_left(kCalibrationDutyPercent);
          bool stallDetected = false;
          if (measuredDeg >= 0.0f) {
            if (!calibrationMotionValid) {
              calibrationMotionValid = true;
              calibrationLastAngleDeg = measuredDeg;
              calibrationLastMoveTick = nowTicks;
            } else if (fabsf(measuredDeg - calibrationLastAngleDeg) > kCalibrationStallThresholdDeg) {
              calibrationLastAngleDeg = measuredDeg;
              calibrationLastMoveTick = nowTicks;
            } else if ((nowTicks - calibrationLastMoveTick) >= kCalibrationStallDuration) {
              stallDetected = true;
            }
          }
          if (stallDetected) {
            calibrationLeftAngleDeg = (measuredDeg >= 0.0f) ? measuredDeg : calibrationLastAngleDeg;
            String msg = "[PID] Calibracion: limite izquierdo inferido por estancamiento (sin FC)";
            msg += " ang=";
            msg += String(calibrationLeftAngleDeg, 2);
            msg += "deg";
            broadcastIf(true, msg);
            bridge_stop();
            calibrationState = CalibrationState::ReleaseLeft;
            calibrationStateStart = nowTicks;
            break;
          }
          if (limitLeftActive) {
            calibrationLeftAngleDeg = measuredDeg;
            bridge_stop();
            calibrationState = CalibrationState::ReleaseLeft;
            calibrationStateStart = nowTicks;
            String msg = "[PID] Calibracion: limite izquierdo registrado en ";
            msg += String(calibrationLeftAngleDeg, 2);
            msg += "deg";
            broadcastIf(true, msg);
          } else if ((nowTicks - calibrationStateStart) >= kCalibrationTimeout) {
            broadcastIf(true, "[PID] Calibracion abortada: timeout alcanzando limite izquierdo");
            keepCalibrating = false;
          }
          break;
        }
        case CalibrationState::ReleaseLeft: {
          if (!limitLeftActive && (nowTicks - calibrationStateStart) >= pdMS_TO_TICKS(50)) {
            bridge_stop();
            if (measuredDeg >= 0.0f) {
              steeringCalibrationApply(calibrationLeftAngleDeg, calibrationRightAngleDeg);
              String msg = "[PID] Calibracion completa. Izq=";
              msg += String(calibrationLeftAngleDeg, 2);
              msg += "deg, Der=";
              msg += String(calibrationRightAngleDeg, 2);
              msg += "deg";
              broadcastIf(true, msg);
            } else {
              broadcastIf(true, "[PID] Calibracion completa pero sin lectura de angulo valida");
            }
            keepCalibrating = false;
          } else if ((nowTicks - calibrationStateStart) >= kCalibrationReleaseDuration) {
            broadcastIf(true,
                        "[PID] Calibracion: forzando finalizacion tras liberar el limite izquierdo (timeout)");
            bridge_stop();
            if (measuredDeg >= 0.0f) {
              steeringCalibrationApply(calibrationLeftAngleDeg, calibrationRightAngleDeg);
              String msg = "[PID] Calibracion completa. Izq=";
              msg += String(calibrationLeftAngleDeg, 2);
              msg += "deg, Der=";
              msg += String(calibrationRightAngleDeg, 2);
              msg += "deg";
              broadcastIf(true, msg);
            } else {
              broadcastIf(true, "[PID] Calibracion completa pero sin lectura de angulo valida");
            }
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
          bool stallDetected = false;
          if (measuredDeg >= 0.0f) {
            if (!calibrationMotionValid) {
              calibrationMotionValid = true;
              calibrationLastAngleDeg = measuredDeg;
              calibrationLastMoveTick = nowTicks;
            } else if (fabsf(measuredDeg - calibrationLastAngleDeg) > kCalibrationStallThresholdDeg) {
              calibrationLastAngleDeg = measuredDeg;
              calibrationLastMoveTick = nowTicks;
            } else if ((nowTicks - calibrationLastMoveTick) >= kCalibrationStallDuration) {
              stallDetected = true;
            }
          }
          if (stallDetected) {
            calibrationRightAngleDeg = (measuredDeg >= 0.0f) ? measuredDeg : calibrationLastAngleDeg;
            bridge_stop();
            calibrationState = CalibrationState::ReleaseRight;
            calibrationStateStart = nowTicks;
            String msg = "[PID] Calibracion: limite derecho inferido por estancamiento (sin FC)";
            msg += " ang=";
            msg += String(calibrationRightAngleDeg, 2);
            msg += "deg";
            broadcastIf(true, msg);
            break;
          }
          if (limitRightActive) {
            calibrationRightAngleDeg = measuredDeg;
            bridge_stop();
            calibrationState = CalibrationState::ReleaseRight;
            calibrationStateStart = nowTicks;
            String msg = "[PID] Calibracion: limite derecho registrado en ";
            msg += String(calibrationRightAngleDeg, 2);
            msg += "deg";
            broadcastIf(true, msg);
          } else if ((nowTicks - calibrationStateStart) >= kCalibrationTimeout) {
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
          if (!limitRightActive && (nowTicks - calibrationStateStart) >= pdMS_TO_TICKS(50)) {
            bridge_stop();
            calibrationState = CalibrationState::MoveLeft;
            calibrationStateStart = nowTicks;
            String msg = "[PID] Calibracion: iniciando busqueda de limite izquierdo";
            broadcastIf(true, msg);
          } else if ((nowTicks - calibrationStateStart) >= kCalibrationReleaseDuration) {
            bridge_stop();
            calibrationState = CalibrationState::MoveLeft;
            calibrationStateStart = nowTicks;
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
        bridge_stop();
        calibrationActive = false;
        calibrationState = CalibrationState::Idle;
        calibrationStateStart = 0;
        calibrationMotionValid = false;
        lastCalibrationState = CalibrationState::Idle;
        lastCalibrationDebugTick = 0;
        cfg->controller->reset();
      }
      continue;
    }

    const SteeringCalibrationData calibrationData = steeringCalibrationSnapshot();
    const float targetDeg =
        mapRcValueToAngle(steeringCommand, calibrationData, cfg->centerDeg, cfg->spanDeg);

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
  }
}
