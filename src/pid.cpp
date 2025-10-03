#include "pid.h"

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/portmacro.h>
#include <math.h>

#include "h_bridge.h"

namespace steering_pid {
namespace {
portMUX_TYPE s_rcMux = portMUX_INITIALIZER_UNLOCKED;
int s_rcChannel4Value = 0;

struct PidState {
  float integral;
  float prevError;
  bool havePrevError;
};

PidState s_pidState = {0.0f, 0.0f, false};
bool s_bridgeInitialized = false;

float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void ensureBridgeReady() {
  if (!config.enableSteeringActuator) {
    return;
  }
  if (!s_bridgeInitialized) {
    init_h_bridge();
    enable_bridge_h();
    bridge_stop();
    s_bridgeInitialized = true;
  }
}

}  // namespace

Config config;

void init() {
  portENTER_CRITICAL(&s_rcMux);
  s_rcChannel4Value = 0;
  portEXIT_CRITICAL(&s_rcMux);
  reset();
  s_bridgeInitialized = false;
}

void setRcValue(int value) {
  portENTER_CRITICAL(&s_rcMux);
  s_rcChannel4Value = value;
  portEXIT_CRITICAL(&s_rcMux);
}

int rcValue() {
  portENTER_CRITICAL(&s_rcMux);
  const int value = s_rcChannel4Value;
  portEXIT_CRITICAL(&s_rcMux);
  return value;
}

float rcToSetpointDegrees(int rawValue) {
  const int minInput = config.rcInputMin;
  const int maxInput = config.rcInputMax;
  if (minInput >= maxInput) {
    return 0.0f;
  }

  int clamped = rawValue;
  if (clamped < minInput) {
    clamped = minInput;
  } else if (clamped > maxInput) {
    clamped = maxInput;
  }

  const float mid = 0.5f * (static_cast<float>(minInput) + static_cast<float>(maxInput));
  const float span = 0.5f * static_cast<float>(maxInput - minInput);
  if (span <= 0.0f) {
    return 0.0f;
  }

  float offset = static_cast<float>(clamped) - mid;
  if (fabsf(offset) <= config.rcDeadband) {
    return 0.0f;
  }

  float normalized = offset / span;
  if (normalized < -1.0f) {
    normalized = -1.0f;
  } else if (normalized > 1.0f) {
    normalized = 1.0f;
  }

  return normalized * config.steeringMaxAngleDeg;
}

float absoluteToRelative(float absoluteDegrees) {
  if (isnan(absoluteDegrees)) {
    return NAN;
  }

  float delta = absoluteDegrees - config.steeringCenterAbsDeg;
  while (delta > 180.0f) {
    delta -= 360.0f;
  }
  while (delta < -180.0f) {
    delta += 360.0f;
  }

  return clampf(delta, -config.steeringMaxAngleDeg, config.steeringMaxAngleDeg);
}

void reset() {
  s_pidState.integral = 0.0f;
  s_pidState.prevError = 0.0f;
  s_pidState.havePrevError = false;
}

float update(float setpointDeg, float measuredDeg, float dtSeconds) {
  const float error = setpointDeg - measuredDeg;

  if (fabsf(setpointDeg) <= config.angleDeadbandDeg &&
      fabsf(error) <= config.angleDeadbandDeg) {
    reset();
    return 0.0f;
  }

  float integral = s_pidState.integral + error * dtSeconds;
  integral = clampf(integral, -config.integralLimit, config.integralLimit);

  float derivative = 0.0f;
  if (s_pidState.havePrevError) {
    derivative = (error - s_pidState.prevError) / dtSeconds;
  }

  s_pidState.integral = integral;
  s_pidState.prevError = error;
  s_pidState.havePrevError = true;

  return config.kp * error + config.ki * integral + config.kd * derivative;
}

void applyControl(float controlPercent) {
  if (!config.enableSteeringActuator) {
    return;
  }

  const float limited = clampf(controlPercent,
                               -static_cast<float>(config.maxDutyPercent),
                               static_cast<float>(config.maxDutyPercent));
  if (fabsf(limited) < config.outputDeadbandPercent) {
    if (s_bridgeInitialized) {
      bridge_stop();
    }
    return;
  }

  ensureBridgeReady();

  uint8_t duty = static_cast<uint8_t>(fabsf(limited));
  if (duty < config.minDutyPercent) {
    duty = config.minDutyPercent;
  }
  if (duty > config.maxDutyPercent) {
    duty = config.maxDutyPercent;
  }

  if (limited > 0.0f) {
    bridge_turn_right(duty);
  } else {
    bridge_turn_left(duty);
  }
}

void stopActuator() {
  if (!config.enableSteeringActuator) {
    return;
  }
  if (!s_bridgeInitialized) {
    return;
  }
  bridge_stop();
}

}  // namespace steering_pid
