// h_bridge.h
// Simple H-bridge control API for ESP32 using LEDC PWM
#ifndef H_BRIDGE_H
#define H_BRIDGE_H

#include <Arduino.h>

// Inicializa el driver del puente H: configura pines y PWM (LEDC)
void init_h_bridge();

// Habilita / deshabilita el puente H mediante el pin de enable (GPIO21)
void enable_bridge_h();
void disable_bridge_h();

// Giro: dutyPercent en 0..100 (%). No habilita automaticamente el enable_pin;
// el usuario debe llamar a enable_bridge_h() antes de usar estos.
void bridge_turn_left(uint8_t dutyPercent);
void bridge_turn_right(uint8_t dutyPercent);

// Detiene cualquier salida PWM (poniendo duty a 0)
void bridge_stop();

struct HBridgeTaskConfig {
  bool log;
};

void taskBridgeTest(void* parameter);

#endif // H_BRIDGE_H
