#include "pid.h"

#include <math.h>

#include <freertos/task.h>

#include <esp_timer.h>

#include "AS5600.h"
#include "fs_ia6.h"
#include "h_bridge.h"
#include "ota_telnet.h"

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

float mapRcValueToAngle(int rcValue, float centerDeg, float spanDeg) {
  const float rawTarget = centerDeg + (spanDeg * static_cast<float>(rcValue) / 100.0f);
  return normalizeAngleDegrees(rawTarget);
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
  TickType_t lastDtWarningTick = 0;
  TickType_t lastRuntimeWarningTick = 0;

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

    if (dtSeconds > dtOverrunThreshold) {
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

    RcSharedState rcSnapshot{};
    const bool rcValid = rcGetStateCopy(rcSnapshot);
    if (!rcValid || !rcSnapshot.valid || (nowTicks - rcSnapshot.lastUpdateTick) > pdMS_TO_TICKS(50)) {
      rcSnapshot.steering = 0;
    }
    const int rcValue = rcSnapshot.steering;
    const float targetDeg = mapRcValueToAngle(rcValue, cfg->centerDeg, cfg->spanDeg);
    const float measuredDeg = cfg->sensor->getAngleDegrees();

    const bool limitLeftActive = bridge_limit_left_active();
    const bool limitRightActive = bridge_limit_right_active();

    const bool shouldLog = cfg->log && (logInterval == 0 || (nowTicks - lastLog) >= logInterval);

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
    if (iterationDurationUs > 4000) {
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
