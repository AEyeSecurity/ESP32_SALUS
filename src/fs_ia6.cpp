#include "../include/fs_ia6.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ota_telnet.h"
#include "quad_logic.h"

//////////////// NO IBUS /////////////////////////////////////////////////////////////

// Set all pins as inputs for COMM
void initFS_IA6(uint8_t ch1, uint8_t ch2, uint8_t ch3, uint8_t ch4, uint8_t ch5, uint8_t ch6) {
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
}

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue) {
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100) {
    return defaultValue;
  }
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue) {
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

//////////////// IBUS  /////////////////////////////////////////////////////////////////

// Attach iBus object to serial port
void initIBus(IBusBM& ibus, HardwareSerial& serial) {
  ibus.begin(serial);
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(IBusBM& ibus, byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) {
    return defaultValue;
  }
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

void taskRcMonitor(void* parameter) {
  const FsIa6TaskConfig* cfg = static_cast<const FsIa6TaskConfig*>(parameter);
  const bool log = (cfg != nullptr) ? cfg->log : false;
  const TickType_t period = (cfg != nullptr) ? cfg->period : pdMS_TO_TICKS(100);

  int lastThrottle = 9999;
  int lastSteering = 9999;
  int lastAux1 = 9999;
  int lastAux2 = 9999;

  for (;;) {
    int throttle = readChannel(kRcThrottlePin, -100, 100, 0);
    int steering = readChannel(kRcSteeringPin, -100, 100, 0);
    int aux1 = readChannel(kRcAux1Pin, -100, 100, 0);
    int aux2 = readChannel(kRcAux2Pin, -100, 100, 0);

    RcInputSnapshot snapshot{};
    snapshot.throttle = throttle;
    snapshot.steering = steering;
    snapshot.aux1 = aux1;
    snapshot.aux2 = aux2;
    snapshot.timestamp = xTaskGetTickCount();
    snapshot.valid = true;

    if (!quadLogicQueueRcInput(snapshot) && log) {
      broadcastIf(true, "[FS-iA6] WARNING: no se pudo publicar RC snapshot a quad_logic");
    }

    if (throttle != lastThrottle || steering != lastSteering || aux1 != lastAux1 || aux2 != lastAux2) {
      if (log) {
        String rcMsg = "FS-iA6 -> Acelerador/Reversa(GPIO4): " + String(throttle) +
                       " | Direccion(GPIO6): " + String(steering) +
                       " | Aux1(GPIO0): " + String(aux1) +
                       " | Aux2(GPIO2): " + String(aux2);
        broadcastIf(true, rcMsg);
      }
      lastThrottle = throttle;
      lastSteering = steering;
      lastAux1 = aux1;
      lastAux2 = aux2;
    }
    vTaskDelay(period);
  }
}

