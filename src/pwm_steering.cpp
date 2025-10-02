#include "pwm_steering.h"

#include <driver/gpio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

namespace pwm_steering {
namespace {
constexpr float kMinDuty = 0.05f;
constexpr float kDutyTolerance = 0.01f;
constexpr uint32_t kMinPeriodUs = 1000;      // Reject very high frequency noise
constexpr uint32_t kMaxPeriodUs = 80000;     // Reject abnormally low frequency pulses
constexpr uint32_t kSignalTimeoutUs = 250000;  // Consider the signal lost after 250 ms
constexpr TickType_t kTaskPeriod = pdMS_TO_TICKS(20);

portMUX_TYPE s_measureMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE s_stateMux = portMUX_INITIALIZER_UNLOCKED;

volatile uint32_t s_lastRiseMicros = 0;
volatile uint32_t s_highCandidateMicros = 0;
volatile bool s_highPending = false;
volatile uint32_t s_pendingPeriod = 0;
volatile uint32_t s_pendingHigh = 0;
volatile bool s_hasNewMeasurement = false;

SteeringState s_state = {NAN, 0.0f, 0u, 0u, 0u, false};

uint8_t s_pin = 15;
gpio_num_t s_gpioNum = GPIO_NUM_15;

void IRAM_ATTR pwmIsr() {
  const uint32_t now = micros();
  const int level = gpio_get_level(s_gpioNum);

  if (level) {
    const uint32_t previousRise = s_lastRiseMicros;
    s_lastRiseMicros = now;

    if (previousRise != 0u) {
      const uint32_t period = now - previousRise;
      portENTER_CRITICAL_ISR(&s_measureMux);
      if (s_highPending) {
        s_pendingPeriod = period;
        s_pendingHigh = s_highCandidateMicros;
        s_hasNewMeasurement = true;
        s_highPending = false;
      }
      portEXIT_CRITICAL_ISR(&s_measureMux);
    }
  } else {
    const uint32_t rise = s_lastRiseMicros;
    if (rise != 0u) {
      const uint32_t high = now - rise;
      portENTER_CRITICAL_ISR(&s_measureMux);
      s_highCandidateMicros = high;
      s_highPending = true;
      portEXIT_CRITICAL_ISR(&s_measureMux);
    }
  }
}

}  // namespace

void init(uint8_t gpioPin) {
  s_pin = gpioPin;
  s_gpioNum = static_cast<gpio_num_t>(gpioPin);
  pinMode(s_pin, INPUT);

  const uint32_t now = micros();

  portENTER_CRITICAL(&s_measureMux);
  s_lastRiseMicros = 0;
  s_highCandidateMicros = 0;
  s_highPending = false;
  s_pendingPeriod = 0;
  s_pendingHigh = 0;
  s_hasNewMeasurement = false;
  portEXIT_CRITICAL(&s_measureMux);

  SteeringState initial{};
  initial.angleDegrees = NAN;
  initial.dutyCycle = 0.0f;
  initial.periodMicros = 0u;
  initial.highMicros = 0u;
  initial.updatedMicros = now;
  initial.signalValid = false;

  portENTER_CRITICAL(&s_stateMux);
  s_state = initial;
  portEXIT_CRITICAL(&s_stateMux);

  detachInterrupt(s_pin);
  attachInterrupt(s_pin, pwmIsr, CHANGE);
}

void task(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    uint32_t period = 0;
    uint32_t high = 0;
    bool newMeasurement = false;

    portENTER_CRITICAL(&s_measureMux);
    if (s_hasNewMeasurement) {
      period = s_pendingPeriod;
      high = s_pendingHigh;
      s_hasNewMeasurement = false;
      newMeasurement = true;
    }
    portEXIT_CRITICAL(&s_measureMux);

    const uint32_t now = micros();

    if (newMeasurement) {
      const bool periodWithinBounds = period >= kMinPeriodUs && period <= kMaxPeriodUs;
      float duty = 0.0f;
      if (period != 0u) {
        duty = static_cast<float>(high) / static_cast<float>(period);
      }

      const bool dutyWithinBounds = duty >= 0.0f && duty <= 1.05f;
      const bool dutyAboveMin = duty >= (kMinDuty - kDutyTolerance);
      const bool signalValidNow = periodWithinBounds && dutyWithinBounds && dutyAboveMin;

      float angle = NAN;
      if (signalValidNow) {
        float normalized = (duty - kMinDuty) / (1.0f - kMinDuty);
        if (normalized < 0.0f) {
          normalized = 0.0f;
        } else if (normalized > 1.0f) {
          normalized = 1.0f;
        }
        angle = normalized * 360.0f;
      }

      portENTER_CRITICAL(&s_stateMux);
      s_state.periodMicros = period;
      s_state.highMicros = high;
      s_state.dutyCycle = duty;
      s_state.angleDegrees = angle;
      s_state.signalValid = signalValidNow;
      s_state.updatedMicros = now;
      portEXIT_CRITICAL(&s_stateMux);
    } else {
      portENTER_CRITICAL(&s_stateMux);
      if (s_state.signalValid) {
        const uint32_t elapsed = now - s_state.updatedMicros;
        if (elapsed > kSignalTimeoutUs) {
          s_state.signalValid = false;
          s_state.angleDegrees = NAN;
        }
      }
      portEXIT_CRITICAL(&s_stateMux);
    }

    vTaskDelayUntil(&lastWake, kTaskPeriod);
  }
}

bool getState(SteeringState& out) {
  portENTER_CRITICAL(&s_stateMux);
  out = s_state;
  portEXIT_CRITICAL(&s_stateMux);
  return out.signalValid;
}

float angleDegrees() {
  portENTER_CRITICAL(&s_stateMux);
  const float angle = s_state.angleDegrees;
  portEXIT_CRITICAL(&s_stateMux);
  return angle;
}

bool signalValid() {
  portENTER_CRITICAL(&s_stateMux);
  const bool valid = s_state.signalValid;
  portEXIT_CRITICAL(&s_stateMux);
  return valid;
}

float dutyCycle() {
  portENTER_CRITICAL(&s_stateMux);
  const float duty = s_state.dutyCycle;
  portEXIT_CRITICAL(&s_stateMux);
  return duty;
}

uint32_t lastUpdateMicros() {
  portENTER_CRITICAL(&s_stateMux);
  const uint32_t updated = s_state.updatedMicros;
  portEXIT_CRITICAL(&s_stateMux);
  return updated;
}

}  // namespace pwm_steering
