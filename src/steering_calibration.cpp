#include "steering_calibration.h"

#include <Preferences.h>
#include <freertos/FreeRTOS.h>

#include <math.h>

namespace {
portMUX_TYPE g_calibrationMux = portMUX_INITIALIZER_UNLOCKED;
SteeringCalibrationData g_state{};
volatile bool g_calibrationRequested = false;
Preferences g_calibrationPrefs;
float g_defaultCenterDeg = 0.0f;
float g_defaultSpanDeg = 0.0f;

constexpr const char* kPrefsNamespace = "steer_cal";
constexpr uint32_t kPrefsVersion = 1;
constexpr const char* kPrefsKeyVersion = "ver";
constexpr const char* kPrefsKeyHasCalibration = "has";
constexpr const char* kPrefsKeyLeft = "left";
constexpr const char* kPrefsKeyRight = "right";
constexpr const char* kPrefsKeyCenter = "center";
constexpr const char* kPrefsKeyOffset = "offset";

struct PersistedCalibration {
  bool hasCalibration = false;
  float leftLimitDeg = NAN;
  float rightLimitDeg = NAN;
  float centerDeg = NAN;
  float userOffsetDeg = 0.0f;
};

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

void recalcAdjustedCenterLocked();

void applyDefaultsLocked() {
  g_state.initialized = true;
  g_state.hasCalibration = false;
  g_state.rawCenterDeg = normalizeDeg(g_defaultCenterDeg);
  g_state.userOffsetDeg = 0.0f;
  g_state.leftLimitDeg = normalizeDeg(g_defaultCenterDeg - g_defaultSpanDeg);
  g_state.rightLimitDeg = normalizeDeg(g_defaultCenterDeg + g_defaultSpanDeg);
  g_state.lastCalibrationMs = 0;
  recalcAdjustedCenterLocked();
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

bool loadPersistedCalibration(PersistedCalibration* out) {
  if (!g_calibrationPrefs.begin(kPrefsNamespace, true)) {
    return false;
  }

  const uint32_t storedVersion = g_calibrationPrefs.getUInt(kPrefsKeyVersion, 0);
  if (storedVersion != kPrefsVersion) {
    g_calibrationPrefs.end();
    return false;
  }

  out->hasCalibration = g_calibrationPrefs.getBool(kPrefsKeyHasCalibration, false);
  out->leftLimitDeg = g_calibrationPrefs.getFloat(kPrefsKeyLeft, NAN);
  out->rightLimitDeg = g_calibrationPrefs.getFloat(kPrefsKeyRight, NAN);
  out->centerDeg = g_calibrationPrefs.getFloat(kPrefsKeyCenter, NAN);
  out->userOffsetDeg = g_calibrationPrefs.getFloat(kPrefsKeyOffset, 0.0f);
  g_calibrationPrefs.end();

  if (!out->hasCalibration) {
    return false;
  }

  if (isnan(out->leftLimitDeg) || isnan(out->rightLimitDeg) || isnan(out->centerDeg)) {
    return false;
  }

  const float span = fabsf(wrapDeg(out->rightLimitDeg - out->leftLimitDeg));
  if (span < 1.0f) {
    return false;
  }

  return true;
}

void persistCalibrationSnapshot(const SteeringCalibrationData& data) {
  if (!g_calibrationPrefs.begin(kPrefsNamespace, false)) {
    return;
  }
  g_calibrationPrefs.putUInt(kPrefsKeyVersion, kPrefsVersion);
  g_calibrationPrefs.putBool(kPrefsKeyHasCalibration, data.hasCalibration);
  g_calibrationPrefs.putFloat(kPrefsKeyLeft, data.leftLimitDeg);
  g_calibrationPrefs.putFloat(kPrefsKeyRight, data.rightLimitDeg);
  g_calibrationPrefs.putFloat(kPrefsKeyCenter, data.rawCenterDeg);
  g_calibrationPrefs.putFloat(kPrefsKeyOffset, data.userOffsetDeg);
  g_calibrationPrefs.end();
}

void loadCalibrationFromStorage() {
  PersistedCalibration persisted;
  if (!loadPersistedCalibration(&persisted)) {
    return;
  }

  portENTER_CRITICAL(&g_calibrationMux);
  g_state.initialized = true;
  g_state.hasCalibration = persisted.hasCalibration;
  g_state.leftLimitDeg = normalizeDeg(persisted.leftLimitDeg);
  g_state.rightLimitDeg = normalizeDeg(persisted.rightLimitDeg);
  g_state.rawCenterDeg = normalizeDeg(persisted.centerDeg);
  g_state.userOffsetDeg = persisted.userOffsetDeg;
  g_state.lastCalibrationMs = persisted.hasCalibration ? millis() : 0;
  recalcAdjustedCenterLocked();
  portEXIT_CRITICAL(&g_calibrationMux);
}
}  // namespace

void steeringCalibrationInit(float defaultCenterDeg, float defaultSpanDeg) {
  portENTER_CRITICAL(&g_calibrationMux);
  g_defaultCenterDeg = defaultCenterDeg;
  g_defaultSpanDeg = defaultSpanDeg;
  applyDefaultsLocked();
  portEXIT_CRITICAL(&g_calibrationMux);

  loadCalibrationFromStorage();
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
  SteeringCalibrationData snapshot;
  portENTER_CRITICAL(&g_calibrationMux);
  g_state.userOffsetDeg = offsetDeg;
  recalcAdjustedCenterLocked();
  snapshot = g_state;
  portEXIT_CRITICAL(&g_calibrationMux);
  persistCalibrationSnapshot(snapshot);
}

void steeringCalibrationApply(float leftDeg, float rightDeg) {
  SteeringCalibrationData snapshot;
  portENTER_CRITICAL(&g_calibrationMux);
  g_state.initialized = true;
  g_state.hasCalibration = true;
  g_state.leftLimitDeg = normalizeDeg(leftDeg);
  g_state.rightLimitDeg = normalizeDeg(rightDeg);
  g_state.rawCenterDeg = normalizeDeg((leftDeg + rightDeg) * 0.5f);
  g_state.lastCalibrationMs = millis();
  recalcAdjustedCenterLocked();
  snapshot = g_state;
  portEXIT_CRITICAL(&g_calibrationMux);
  persistCalibrationSnapshot(snapshot);
}

void steeringCalibrationReset(bool clearStorage) {
  SteeringCalibrationData snapshot;
  portENTER_CRITICAL(&g_calibrationMux);
  applyDefaultsLocked();
  snapshot = g_state;
  portEXIT_CRITICAL(&g_calibrationMux);

  if (clearStorage) {
    if (g_calibrationPrefs.begin(kPrefsNamespace, false)) {
      g_calibrationPrefs.clear();
      g_calibrationPrefs.end();
    }
  } else {
    persistCalibrationSnapshot(snapshot);
  }
}
