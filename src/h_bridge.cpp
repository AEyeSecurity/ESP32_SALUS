// h_bridge.cpp
// Implementacion simple del control de un puente H en ESP32 usando LEDC

#include "../include/h_bridge.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ota_telnet.h"

// GPIO assignments
static const int ENABLE_PIN = 21;
static const int LEFT_PWM_PIN = 19;  // girar a la izquierda
static const int RIGHT_PWM_PIN = 18; // girar a la derecha
static const int LIMIT_LEFT_PIN = 27;  // final de carrera izquierda
static const int LIMIT_RIGHT_PIN = 14; // final de carrera derecha
static const int LIMIT_ACTIVE_STATE = LOW; // usando pull-ups internos

struct LimitDebugState {
  bool initialized = false;
  bool lastState = false;
};

static LimitDebugState leftLimitDebug;
static LimitDebugState rightLimitDebug;
static uint32_t lastLeftBlockLogMs = 0;
static uint32_t lastRightBlockLogMs = 0;

static void logLimitStatus(const char* label, LimitDebugState& debug, bool isActive) {
  if (!debug.initialized) {
    debug.initialized = true;
    debug.lastState = isActive;
    if (!isActive) {
      return;
    }
    broadcastIf(true, String("[HBRIDGE] FC ") + label + " ACTIVO");
    return;
  }

  if (isActive == debug.lastState) {
    return;
  }

  debug.lastState = isActive;
  const char* stateText = isActive ? "ACTIVO" : "INACTIVO";
  broadcastIf(true, String("[HBRIDGE] FC ") + label + " " + stateText);
}

// LEDC settings
static const int LEDC_FREQ = 20000;      // 20 kHz
static const int LEDC_RESOLUTION = 8;    // 8 bits -> duty 0..255
static const int LEFT_LEDC_CHANNEL = 0;
static const int RIGHT_LEDC_CHANNEL = 1;
static const int LEDC_TIMER = 0; // timer index

static bool limit_active(int pin) {
  return digitalRead(pin) == LIMIT_ACTIVE_STATE;
}

bool bridge_limit_left_active() {
  const bool active = limit_active(LIMIT_LEFT_PIN);
  logLimitStatus("IZQUIERDO", leftLimitDebug, active);
  return active;
}

bool bridge_limit_right_active() {
  const bool active = limit_active(LIMIT_RIGHT_PIN);
  logLimitStatus("DERECHO", rightLimitDebug, active);
  return active;
}

void init_h_bridge() {
  // Configure enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // disabled by default

  // Configure limit switches with pull-ups
  pinMode(LIMIT_LEFT_PIN, INPUT_PULLUP);
  pinMode(LIMIT_RIGHT_PIN, INPUT_PULLUP);

  // Configure LEDC timer
  ledcSetup(LEDC_TIMER, LEDC_FREQ, LEDC_RESOLUTION);

  // Attach channels to timer
  ledcAttachPin(LEFT_PWM_PIN, LEFT_LEDC_CHANNEL);
  ledcAttachPin(RIGHT_PWM_PIN, RIGHT_LEDC_CHANNEL);

  // Ensure outputs are zero
  ledcWrite(LEFT_LEDC_CHANNEL, 0);
  ledcWrite(RIGHT_LEDC_CHANNEL, 0);
}

void enable_bridge_h() {
  digitalWrite(ENABLE_PIN, HIGH);
}

void disable_bridge_h() {
  // Stop PWM and disable
  bridge_stop();
  digitalWrite(ENABLE_PIN, LOW);
}

// dutyPercent: 0..100
static uint32_t duty_from_percent(uint8_t dutyPercent) {
  if (dutyPercent >= 100) {
    return (uint32_t)((1 << LEDC_RESOLUTION) - 1);
  }
  return (uint32_t)((dutyPercent * ((1 << LEDC_RESOLUTION) - 1)) / 100);
}

void bridge_turn_left(uint8_t dutyPercent) {
  if (bridge_limit_left_active()) {
    bridge_stop();
    const uint32_t nowMs = millis();
    if (nowMs - lastLeftBlockLogMs >= 500) {
      String msg = String("[HBRIDGE] Giro izquierda bloqueado por FC IZQUIERDO (orden=") + String(dutyPercent) + "%, t=" + String(nowMs) + "ms)";
      broadcastIf(true, msg);
      lastLeftBlockLogMs = nowMs;
    }
    return;
  }
  // Left: apply PWM to LEFT channel, ensure RIGHT channel is 0
  uint32_t duty = duty_from_percent(dutyPercent);
  ledcWrite(RIGHT_LEDC_CHANNEL, 0);
  ledcWrite(LEFT_LEDC_CHANNEL, duty);
}

void bridge_turn_right(uint8_t dutyPercent) {
  if (bridge_limit_right_active()) {
    bridge_stop();
    const uint32_t nowMs = millis();
    if (nowMs - lastRightBlockLogMs >= 500) {
      String msg = String("[HBRIDGE] Giro derecha bloqueado por FC DERECHO (orden=") + String(dutyPercent) + "%, t=" + String(nowMs) + "ms)";
      broadcastIf(true, msg);
      lastRightBlockLogMs = nowMs;
    }
    return;
  }
  // Right: apply PWM to RIGHT channel, ensure LEFT channel is 0
  uint32_t duty = duty_from_percent(dutyPercent);
  ledcWrite(LEFT_LEDC_CHANNEL, 0);
  ledcWrite(RIGHT_LEDC_CHANNEL, duty);
}

void bridge_stop() {
  ledcWrite(LEFT_LEDC_CHANNEL, 0);
  ledcWrite(RIGHT_LEDC_CHANNEL, 0);
}

static void reportDuty(const char* label, int duty, bool log) {
  if (!log) {
    return;
  }
  String msg = String("H-bridge ") + label + " duty: " + duty + "%";
  broadcastIf(true, msg);
}

void taskBridgeTest(void* parameter) {
  const HBridgeTaskConfig* cfg = static_cast<const HBridgeTaskConfig*>(parameter);
  const bool log = (cfg != nullptr) ? cfg->log : false;

  const TickType_t rampUpStepDelay = pdMS_TO_TICKS(80);
  const TickType_t rampDownStepDelay = pdMS_TO_TICKS(60);
  const TickType_t pauseBetweenDirections = pdMS_TO_TICKS(300);
  const TickType_t cyclePause = pdMS_TO_TICKS(2000);

  for (;;) {
    enable_bridge_h();
    broadcastIf(log, "Iniciando prueba H-bridge: izquierda subir/bajar, derecha subir/bajar");

    for (int d = 0; d <= 100; d += 5) {
      bridge_turn_left(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("LEFT", d, log);
      }
      vTaskDelay(rampUpStepDelay);
    }

    for (int d = 100; d >= 0; d -= 5) {
      bridge_turn_left(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("LEFT", d, log);
      }
      vTaskDelay(rampDownStepDelay);
    }

    bridge_stop();
    vTaskDelay(pauseBetweenDirections);

    for (int d = 0; d <= 100; d += 5) {
      bridge_turn_right(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("RIGHT", d, log);
      }
      vTaskDelay(rampUpStepDelay);
    }

    for (int d = 100; d >= 0; d -= 5) {
      bridge_turn_right(static_cast<uint8_t>(d));
      if (d % 20 == 0 || d == 0 || d == 100) {
        reportDuty("RIGHT", d, log);
      }
      vTaskDelay(rampDownStepDelay);
    }

    bridge_stop();
    disable_bridge_h();
    broadcastIf(log, "Prueba H-bridge finalizada");

    vTaskDelay(cyclePause);
  }
}
