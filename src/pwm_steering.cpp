#include "pwm_steering.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <math.h>

namespace pwm_steering {
namespace {
constexpr float kMinDuty = 0.05f;
constexpr float kDutyTolerance = 0.01f;
constexpr uint32_t kMinPeriodUs = 1000;        // Rechaza ruido de muy alta frecuencia
constexpr uint32_t kMaxPeriodUs = 80000;       // Rechaza pulsos anormalmente lentos
constexpr uint32_t kSignalTimeoutUs = 250000;  // Marca perdida de senal tras 250 ms
constexpr uint32_t kPulseTimeoutUs = 30000;    // Tiempo maximo esperando alto/bajo

portMUX_TYPE s_stateMux = portMUX_INITIALIZER_UNLOCKED;

SteeringState s_state = {NAN, 0.0f, 0u, 0u, 0u, false};

uint8_t s_pin = 15;

void markNoSignal(uint32_t nowUs, bool forceUpdate) {
  portENTER_CRITICAL(&s_stateMux);
  const bool changed = forceUpdate || s_state.signalValid || s_state.periodMicros != 0u ||
                       s_state.highMicros != 0u;
  s_state.periodMicros = 0u;
  s_state.highMicros = 0u;
  s_state.dutyCycle = 0.0f;
  s_state.angleDegrees = NAN;
  s_state.signalValid = false;
  if (changed) {
    s_state.updatedMicros = nowUs;
  }
  portEXIT_CRITICAL(&s_stateMux);
}

void processMeasurement(uint32_t periodUs, uint32_t highUs, uint32_t nowUs) {
  const bool periodWithinBounds = periodUs >= kMinPeriodUs && periodUs <= kMaxPeriodUs;
  float duty = 0.0f;
  if (periodUs != 0u) {
    duty = static_cast<float>(highUs) / static_cast<float>(periodUs);
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
  s_state.periodMicros = periodUs;
  s_state.highMicros = highUs;
  s_state.dutyCycle = duty;
  s_state.angleDegrees = angle;
  s_state.signalValid = signalValidNow;
  s_state.updatedMicros = nowUs;
  portEXIT_CRITICAL(&s_stateMux);
}

bool samplePulse(uint32_t& periodUs, uint32_t& highUs) {
  const uint32_t high = static_cast<uint32_t>(pulseInLong(s_pin, HIGH, kPulseTimeoutUs));
  if (high == 0u) {
    return false;
  }
  const uint32_t low = static_cast<uint32_t>(pulseInLong(s_pin, LOW, kPulseTimeoutUs));
  if (low == 0u) {
    return false;
  }
  highUs = high;
  periodUs = high + low;
  return true;
}

}  // namespace

void init(uint8_t gpioPin) {
  s_pin = gpioPin;
  pinMode(s_pin, INPUT);

  markNoSignal(micros(), true);
}

void task(void* parameter) {
  for (;;) {
    uint32_t periodUs = 0;
    uint32_t highUs = 0;
    if (samplePulse(periodUs, highUs)) {
      processMeasurement(periodUs, highUs, micros());
    } else {
      markNoSignal(micros(), false);
      vTaskDelay(pdMS_TO_TICKS(5));
    }
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
