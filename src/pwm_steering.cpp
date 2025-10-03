#include "pwm_steering.h"

#include <driver/gpio.h>
#include <driver/rmt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/ringbuf.h>
#include <freertos/task.h>
#include <math.h>

namespace pwm_steering {
namespace {
constexpr float kMinDuty = 0.05f;
constexpr float kDutyTolerance = 0.01f;
constexpr uint32_t kMinPeriodUs = 1000;        // Reject very high frequency noise
constexpr uint32_t kMaxPeriodUs = 80000;       // Reject abnormally low frequency pulses
constexpr uint32_t kSignalTimeoutUs = 250000;  // Consider the signal lost after 250 ms
constexpr TickType_t kTaskPeriod = pdMS_TO_TICKS(20);

constexpr rmt_channel_t kChannel = RMT_CHANNEL_0;
constexpr uint32_t kClockMHz = 80;
constexpr uint32_t kClkDiv = 100;  // 1.25 us resolution
constexpr uint32_t kFilterUs = 3;
constexpr uint32_t kIdleMarginUs = 5000;

constexpr uint32_t ticksFromUs(uint32_t us) {
  return (us * kClockMHz + (kClkDiv - 1)) / kClkDiv;
}

constexpr uint32_t usFromTicks(uint32_t ticks) {
  return (ticks * kClkDiv + (kClockMHz / 2)) / kClockMHz;
}

constexpr uint32_t kFilterTicks = ticksFromUs(kFilterUs);
constexpr uint32_t kIdleThresholdRaw = ticksFromUs(kMaxPeriodUs + kIdleMarginUs);
constexpr uint32_t kIdleThresholdTicks =
    (kIdleThresholdRaw > 0xFFFFu) ? 0xFFFFu : kIdleThresholdRaw;

portMUX_TYPE s_stateMux = portMUX_INITIALIZER_UNLOCKED;

SteeringState s_state = {NAN, 0.0f, 0u, 0u, 0u, false};

uint8_t s_pin = 15;
gpio_num_t s_gpioNum = GPIO_NUM_15;
RingbufHandle_t s_ring = nullptr;
bool s_driverReady = false;

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

void handleTimeout(uint32_t nowUs) {
  portENTER_CRITICAL(&s_stateMux);
  if (s_state.signalValid) {
    const uint32_t elapsed = nowUs - s_state.updatedMicros;
    if (elapsed > kSignalTimeoutUs) {
      s_state.signalValid = false;
      s_state.angleDegrees = NAN;
    }
  }
  portEXIT_CRITICAL(&s_stateMux);
}

}  // namespace

void init(uint8_t gpioPin) {
  s_pin = gpioPin;
  s_gpioNum = static_cast<gpio_num_t>(gpioPin);

  if (s_driverReady) {
    rmt_rx_stop(kChannel);
    rmt_driver_uninstall(kChannel);
    s_driverReady = false;
    s_ring = nullptr;
  }

  pinMode(s_pin, INPUT);

  const uint32_t now = micros();
  portENTER_CRITICAL(&s_stateMux);
  s_state.angleDegrees = NAN;
  s_state.dutyCycle = 0.0f;
  s_state.periodMicros = 0u;
  s_state.highMicros = 0u;
  s_state.updatedMicros = now;
  s_state.signalValid = false;
  portEXIT_CRITICAL(&s_stateMux);

  rmt_config_t config = {};
  config.rmt_mode = RMT_MODE_RX;
  config.channel = kChannel;
  config.gpio_num = s_gpioNum;
  config.clk_div = kClkDiv;
  config.mem_block_num = 2;
  config.rx_config.filter_en = true;
  config.rx_config.filter_ticks_thresh = static_cast<uint8_t>(kFilterTicks);
  config.rx_config.idle_threshold = static_cast<uint16_t>(kIdleThresholdTicks);

  if (rmt_config(&config) != ESP_OK) {
    return;
  }

  if (rmt_driver_install(kChannel, 1024, 0) != ESP_OK) {
    return;
  }

  if (rmt_get_ringbuf_handle(kChannel, &s_ring) != ESP_OK || s_ring == nullptr) {
    rmt_driver_uninstall(kChannel);
    s_ring = nullptr;
    return;
  }

  if (rmt_rx_start(kChannel, true) != ESP_OK) {
    rmt_driver_uninstall(kChannel);
    s_ring = nullptr;
    return;
  }

  s_driverReady = true;
}

void task(void* parameter) {
  TickType_t lastWake = xTaskGetTickCount();

  for (;;) {
    bool processedMeasurement = false;

    if (s_driverReady && s_ring != nullptr) {
      size_t length = 0;
      rmt_item32_t* items =
          reinterpret_cast<rmt_item32_t*>(xRingbufferReceive(s_ring, &length, pdMS_TO_TICKS(5)));
      if (items != nullptr) {
        const size_t itemCount = length / sizeof(rmt_item32_t);
        uint32_t highAccum = 0;
        uint32_t lowAccum = 0;
        bool haveHigh = false;
        bool haveLow = false;

        auto finalizeMeasurement = [&]() {
          if (haveHigh && haveLow) {
            const uint32_t period = highAccum + lowAccum;
            processMeasurement(period, highAccum, micros());
            processedMeasurement = true;
          }
          highAccum = 0;
          lowAccum = 0;
          haveHigh = false;
          haveLow = false;
        };

        for (size_t i = 0; i < itemCount; ++i) {
          const rmt_item32_t& item = items[i];
          const uint32_t durations[2] = {item.duration0, item.duration1};
          const bool levels[2] = {static_cast<bool>(item.level0), static_cast<bool>(item.level1)};

          for (int j = 0; j < 2; ++j) {
            const uint32_t ticks = durations[j];
            if (ticks == 0u) {
              continue;
            }
            const uint32_t segmentUs = usFromTicks(ticks);
            if (levels[j]) {
              if (haveLow) {
                finalizeMeasurement();
              }
              highAccum += segmentUs;
              haveHigh = true;
            } else {
              if (!haveHigh) {
                continue;
              }
              lowAccum += segmentUs;
              haveLow = true;
            }
          }
        }

        finalizeMeasurement();
        vRingbufferReturnItem(s_ring, reinterpret_cast<void*>(items));
        rmt_rx_start(kChannel, true);
      }
    }

    if (!processedMeasurement) {
      handleTimeout(micros());
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
