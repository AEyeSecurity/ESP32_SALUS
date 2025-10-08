#include "../include/fs_ia6.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ota_telnet.h"

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

  int lastAccel = 9999;
  int lastSteering = 9999;
  int lastBrakeGear = 9999;
  int lastAux = 9999;

  for (;;) {
    int accel = readChannel(kRcAccelPin, -100, 100, 0);
    int steering = readChannel(kRcSteeringPin, -100, 100, 0);
    int brakeGear = readChannel(kRcBrakeGearPin, -100, 100, 0);
    int aux = readChannel(kRcAuxPin, -100, 100, 0);

    if (accel != lastAccel || steering != lastSteering || brakeGear != lastBrakeGear || aux != lastAux) {
      if (log) {
        String rcMsg = "FS-iA6 -> Aceleracion(GPIO0): " + String(accel) +
                       " | Direccion(GPIO6): " + String(steering) +
                       " | Freno/Marchas(GPIO4): " + String(brakeGear) +
                       " | Aux(GPIO16): " + String(aux);
        broadcastIf(true, rcMsg);
      }
      lastAccel = accel;
      lastSteering = steering;
      lastBrakeGear = brakeGear;
      lastAux = aux;
    }
    vTaskDelay(period);
  }
}

