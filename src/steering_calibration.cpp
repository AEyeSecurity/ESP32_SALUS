#include "steering_calibration.h"

#include <freertos/FreeRTOS.h>

#include <math.h>

namespace {
portMUX_TYPE g_calibrationMux = portMUX_INITIALIZER_UNLOCKED;
SteeringCalibrationData g_state{};
volatile bool g_calibrationRequested = false;

float normalizeDeg(float angle) {
  angle = fmodf(angle, 360.0f);
  if (angle < 0.0f) {
    angle += 360.0f;
  }
  return angle;
}

float wrapDeg(float angle) {
  angle = fmodf(angle, 360.0f);
  if (angle > 180.0f) {
    angle -= 360.0f;
  } else if (angle < -180.0f) {
    angle += 360.0f;
  }
  return angle;
}

float clampf(float value, float minValue, float maxValue) {
  if (value < minValue) {
    return minValue;
  }
  if (value > maxValue) {
    return maxValue;
  }
  return value;
}

void recalcAdjustedCenterLocked() {
  const float leftNormalized = normalizeDeg(g_state.leftLimitDeg);
  const float rightNormalized = normalizeDeg(g_state.rightLimitDeg);

  g_state.leftLimitDeg = leftNormalized;
  g_state.rightLimitDeg = rightNormalized;
  g_state.rawCenterDeg = normalizeDeg(g_state.rawCenterDeg);

  float delta = wrapDeg(rightNormalized - leftNormalized);
  if (fabsf(delta) < 0.0001f) {
    delta = 0.0f;
  }
  const float halfDelta = delta * 0.5f;
  g_state.rawCenterDeg = normalizeDeg(leftNormalized + halfDelta);

  const float maxOffsetRight = wrapDeg(rightNormalized - g_state.rawCenterDeg);
  const float maxOffsetLeft = wrapDeg(leftNormalized - g_state.rawCenterDeg);

  float clampedOffset = g_state.userOffsetDeg;
  if (maxOffsetRight >= 0.0f && maxOffsetLeft <= 0.0f) {
    clampedOffset = clampf(g_state.userOffsetDeg, maxOffsetLeft, maxOffsetRight);
  } else {
    clampedOffset = 0.0f;
  }

  g_state.maxOffsetLeftDeg = maxOffsetLeft;
  g_state.maxOffsetRightDeg = maxOffsetRight;
  g_state.userOffsetDeg = clampedOffset;
  g_state.adjustedCenterDeg = normalizeDeg(g_state.rawCenterDeg + clampedOffset);
}
}  // namespace

void steeringCalibrationInit(float defaultCenterDeg, float defaultSpanDeg) {
  portENTER_CRITICAL(&g_calibrationMux);
  g_state.initialized = true;
  g_state.hasCalibration = false;
  g_state.rawCenterDeg = normalizeDeg(defaultCenterDeg);
  g_state.adjustedCenterDeg = g_state.rawCenterDeg;
  g_state.userOffsetDeg = 0.0f;
  g_state.leftLimitDeg = normalizeDeg(defaultCenterDeg - defaultSpanDeg);
  g_state.rightLimitDeg = normalizeDeg(defaultCenterDeg + defaultSpanDeg);
  g_state.maxOffsetLeftDeg = -defaultSpanDeg;
  g_state.maxOffsetRightDeg = defaultSpanDeg;
  g_state.lastCalibrationMs = 0;
  recalcAdjustedCenterLocked();
  portEXIT_CRITICAL(&g_calibrationMux);
}

SteeringCalibrationData steeringCalibrationSnapshot() {
  portENTER_CRITICAL(&g_calibrationMux);
  SteeringCalibrationData snapshot = g_state;
  portEXIT_CRITICAL(&g_calibrationMux);
  return snapshot;
}

void steeringCalibrationRequest() {
  g_calibrationRequested = true;
}

bool steeringCalibrationConsumeRequest() {
  bool requested = false;
  portENTER_CRITICAL(&g_calibrationMux);
  if (g_calibrationRequested) {
    requested = true;
    g_calibrationRequested = false;
  }
  portEXIT_CRITICAL(&g_calibrationMux);
  return requested;
}

void steeringCalibrationSetOffset(float offsetDeg) {
  portENTER_CRITICAL(&g_calibrationMux);
  g_state.userOffsetDeg = offsetDeg;
  recalcAdjustedCenterLocked();
  portEXIT_CRITICAL(&g_calibrationMux);
}

void steeringCalibrationApply(float leftDeg, float rightDeg) {
  portENTER_CRITICAL(&g_calibrationMux);
  g_state.initialized = true;
  g_state.hasCalibration = true;
  g_state.leftLimitDeg = normalizeDeg(leftDeg);
  g_state.rightLimitDeg = normalizeDeg(rightDeg);
  g_state.rawCenterDeg = normalizeDeg((leftDeg + rightDeg) * 0.5f);
  g_state.lastCalibrationMs = millis();
  recalcAdjustedCenterLocked();
  portEXIT_CRITICAL(&g_calibrationMux);
}
