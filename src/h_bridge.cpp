// h_bridge.cpp
// Implementación simple del control de un puente H en ESP32 usando LEDC

#include "../include/h_bridge.h"

// GPIO assignments
static const int ENABLE_PIN = 21;
static const int LEFT_PWM_PIN = 19;  // girar a la izquierda
static const int RIGHT_PWM_PIN = 18; // girar a la derecha

// LEDC settings
static const int LEDC_FREQ = 20000;      // 20 kHz
static const int LEDC_RESOLUTION = 8;    // 8 bits -> duty 0..255
static const int LEFT_LEDC_CHANNEL = 0;
static const int RIGHT_LEDC_CHANNEL = 1;
static const int LEDC_TIMER = 0; // timer index

void init_h_bridge() {
  // Configure enable pin
  pinMode(ENABLE_PIN, OUTPUT);
  digitalWrite(ENABLE_PIN, LOW); // disabled by default

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
  if (dutyPercent >= 100) return (uint32_t)((1 << LEDC_RESOLUTION) - 1);
  return (uint32_t)((dutyPercent * ((1 << LEDC_RESOLUTION) - 1)) / 100);
}

void bridge_turn_left(uint8_t dutyPercent) {
  // Left: apply PWM to LEFT channel, ensure RIGHT channel is 0
  uint32_t duty = duty_from_percent(dutyPercent);
  ledcWrite(RIGHT_LEDC_CHANNEL, 0);
  ledcWrite(LEFT_LEDC_CHANNEL, duty);
}

void bridge_turn_right(uint8_t dutyPercent) {
  // Right: apply PWM to RIGHT channel, ensure LEFT channel is 0
  uint32_t duty = duty_from_percent(dutyPercent);
  ledcWrite(LEFT_LEDC_CHANNEL, 0);
  ledcWrite(RIGHT_LEDC_CHANNEL, duty);
}

void bridge_stop() {
  ledcWrite(LEFT_LEDC_CHANNEL, 0);
  ledcWrite(RIGHT_LEDC_CHANNEL, 0);
}
