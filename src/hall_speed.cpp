#include "hall_speed.h"

#include <soc/gpio_struct.h>

namespace {

HallSpeedConfig g_config{};
bool g_initialized = false;

volatile uint8_t g_hallMask = 0;
volatile bool g_hasTransition = false;
volatile uint32_t g_transitionPeriodUs = 0;
volatile uint32_t g_lastTransitionUs = 0;
volatile uint32_t g_transitionsOk = 0;
volatile uint32_t g_transitionsInvalidState = 0;
volatile uint32_t g_transitionsInvalidJump = 0;
volatile uint32_t g_isrCount = 0;

portMUX_TYPE g_mux = portMUX_INITIALIZER_UNLOCKED;

inline uint8_t IRAM_ATTR readPinLevelFast(uint8_t pin) {
  return static_cast<uint8_t>((GPIO.in >> pin) & 0x01U);
}

uint8_t IRAM_ATTR readHallMaskFromPins() {
  const uint8_t levelA = readPinLevelFast(g_config.pinA);
  const uint8_t levelB = readPinLevelFast(g_config.pinB);
  const uint8_t levelC = readPinLevelFast(g_config.pinC);

  uint8_t mask = 0;
  const uint8_t activeA = g_config.activeLow ? (levelA == 0U) : (levelA != 0U);
  const uint8_t activeB = g_config.activeLow ? (levelB == 0U) : (levelB != 0U);
  const uint8_t activeC = g_config.activeLow ? (levelC == 0U) : (levelC != 0U);
  if (activeA) {
    mask |= (1U << 0);
  }
  if (activeB) {
    mask |= (1U << 1);
  }
  if (activeC) {
    mask |= (1U << 2);
  }
  return mask;
}

void IRAM_ATTR onHallSensorChange() {
  const uint32_t nowUs = micros();
  const uint8_t currentMask = readHallMaskFromPins();

  portENTER_CRITICAL_ISR(&g_mux);
  g_isrCount++;

  const uint8_t previousMask = g_hallMask;
  g_hallMask = currentMask;

  if ((currentMask == previousMask) || (currentMask == 0U) || (currentMask == 0x07U)) {
    g_transitionsInvalidState++;
    portEXIT_CRITICAL_ISR(&g_mux);
    return;
  }

  const uint8_t changedBits = currentMask ^ previousMask;
  if (__builtin_popcount(static_cast<unsigned int>(changedBits)) != 1) {
    g_transitionsInvalidJump++;
    portEXIT_CRITICAL_ISR(&g_mux);
    return;
  }

  if (g_lastTransitionUs != 0U) {
    g_transitionPeriodUs = nowUs - g_lastTransitionUs;
    g_hasTransition = true;
    g_transitionsOk++;
  }
  g_lastTransitionUs = nowUs;
  portEXIT_CRITICAL_ISR(&g_mux);
}

inline float rpmFromTransitionUs(uint32_t transitionUs, uint8_t motorPoles) {
  if (transitionUs == 0U || motorPoles < 2U) {
    return 0.0f;
  }
  const uint8_t polePairs = motorPoles / 2U;
  if (polePairs == 0U) {
    return 0.0f;
  }
  const uint32_t transitionsPerRev = 6U * static_cast<uint32_t>(polePairs);
  if (transitionsPerRev == 0U) {
    return 0.0f;
  }
  return (60.0f * 1000000.0f) / (static_cast<float>(transitionUs) * static_cast<float>(transitionsPerRev));
}

inline float kmhFromRpm(float motorRpm, float gearReduction, float wheelDiameterM) {
  if (motorRpm <= 0.0f || gearReduction <= 0.0f || wheelDiameterM <= 0.0f) {
    return 0.0f;
  }
  const float wheelRpm = motorRpm / gearReduction;
  const float wheelCircumferenceM = PI * wheelDiameterM;
  return wheelRpm * wheelCircumferenceM * 0.06f;
}

inline float mpsFromKmh(float speedKmh) {
  return speedKmh > 0.0f ? (speedKmh / 3.6f) : 0.0f;
}

}  // namespace

bool hallSpeedInit(const HallSpeedConfig& config) {
  if (g_initialized) {
    return true;
  }

  if (config.motorPoles < 2U || config.gearReduction <= 0.0f || config.wheelDiameterM <= 0.0f || config.rpmTimeoutUs == 0U) {
    return false;
  }

  g_config = config;
  pinMode(g_config.pinA, INPUT);
  pinMode(g_config.pinB, INPUT);
  pinMode(g_config.pinC, INPUT);

  const uint8_t initialMask = readHallMaskFromPins();
  portENTER_CRITICAL(&g_mux);
  g_hallMask = initialMask;
  g_hasTransition = false;
  g_transitionPeriodUs = 0;
  g_lastTransitionUs = 0;
  g_transitionsOk = 0;
  g_transitionsInvalidState = 0;
  g_transitionsInvalidJump = 0;
  g_isrCount = 0;
  portEXIT_CRITICAL(&g_mux);

  attachInterrupt(digitalPinToInterrupt(g_config.pinA), onHallSensorChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(g_config.pinB), onHallSensorChange, CHANGE);
  attachInterrupt(digitalPinToInterrupt(g_config.pinC), onHallSensorChange, CHANGE);

  g_initialized = true;
  return true;
}

bool hallSpeedGetSnapshot(HallSpeedSnapshot& snapshot) {
  snapshot = {};
  snapshot.driverReady = g_initialized;
  if (!g_initialized) {
    return false;
  }

  uint8_t hallMask = 0;
  bool hasTransition = false;
  uint32_t transitionPeriodUs = 0;
  uint32_t lastTransitionUs = 0;
  uint32_t transitionsOk = 0;
  uint32_t transitionsInvalidState = 0;
  uint32_t transitionsInvalidJump = 0;
  uint32_t isrCount = 0;

  portENTER_CRITICAL(&g_mux);
  hallMask = g_hallMask;
  hasTransition = g_hasTransition;
  transitionPeriodUs = g_transitionPeriodUs;
  lastTransitionUs = g_lastTransitionUs;
  transitionsOk = g_transitionsOk;
  transitionsInvalidState = g_transitionsInvalidState;
  transitionsInvalidJump = g_transitionsInvalidJump;
  isrCount = g_isrCount;
  portEXIT_CRITICAL(&g_mux);

  const uint32_t nowUs = micros();
  uint32_t ageUs = 0;
  if (lastTransitionUs != 0U) {
    ageUs = nowUs - lastTransitionUs;
  }

  float rpm = 0.0f;
  if (hasTransition && ageUs <= g_config.rpmTimeoutUs) {
    rpm = rpmFromTransitionUs(transitionPeriodUs, g_config.motorPoles);
  }
  const float speedKmh = kmhFromRpm(rpm, g_config.gearReduction, g_config.wheelDiameterM);
  const float speedMps = mpsFromKmh(speedKmh);

  snapshot.driverReady = true;
  snapshot.hallMask = hallMask;
  snapshot.hasTransition = hasTransition;
  snapshot.transitionPeriodUs = transitionPeriodUs;
  snapshot.lastTransitionUs = lastTransitionUs;
  snapshot.transitionAgeUs = ageUs;
  snapshot.motorRpm = rpm;
  snapshot.speedKmh = speedKmh;
  snapshot.speedMps = speedMps;
  snapshot.transitionsOk = transitionsOk;
  snapshot.transitionsInvalidState = transitionsInvalidState;
  snapshot.transitionsInvalidJump = transitionsInvalidJump;
  snapshot.isrCount = isrCount;
  return true;
}

void hallSpeedGetConfig(HallSpeedConfig& config) {
  config = g_config;
}

void hallSpeedResetStats() {
  portENTER_CRITICAL(&g_mux);
  g_hasTransition = false;
  g_transitionPeriodUs = 0;
  g_lastTransitionUs = 0;
  g_transitionsOk = 0;
  g_transitionsInvalidState = 0;
  g_transitionsInvalidJump = 0;
  g_isrCount = 0;
  portEXIT_CRITICAL(&g_mux);
}
