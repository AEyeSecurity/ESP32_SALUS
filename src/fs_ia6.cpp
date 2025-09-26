#include "../include/fs_ia6.h"



//////////////// NO IBUS /////////////////////////////////////////////////////////////

// Set all pins as inputs for COMM
void initFS_IA6(uint8_t ch1,uint8_t ch2,uint8_t ch3,uint8_t ch4,uint8_t ch5,uint8_t ch6){
  pinMode(ch1, INPUT);
  pinMode(ch2, INPUT);
  pinMode(ch3, INPUT);
  pinMode(ch4, INPUT);
  pinMode(ch5, INPUT);
  pinMode(ch6, INPUT);
}

// Read the number of a specified channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(int channelInput, int minLimit, int maxLimit, int defaultValue){
  int ch = pulseIn(channelInput, HIGH, 30000);
  if (ch < 100)  return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Read the switch channel and return a boolean value
bool readSwitch(byte channelInput, bool defaultValue){
  int intDefaultValue = (defaultValue)? 100: 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}


//////////////// IBUS  /////////////////////////////////////////////////////////////////

// Attach iBus object to serial port
void initIBus(IBusBM& ibus, HardwareSerial& serial){
    ibus.begin(serial);
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value

int readChannel(IBusBM& ibus, byte channelInput, int minLimit, int maxLimit, int defaultValue) {
  uint16_t ch = ibus.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}



/// IBUS LOOP
  // Cycle through first 5 channels and determine values
  // Print values to serial monitor
  // Note IBusBM library labels channels starting with "0"

  // for (byte i = 0; i < 4; i++) {
  //   int value = readChannel(ibus, i, -100, 100, 0);
  //   Serial.print("Ch");
  //   Serial.print(i + 1);
  //   Serial.print(": ");
  //   Serial.print(value);
  //   Serial.print(" | ");
  // }

  // // Print channel 5 (switch) boolean value
  // Serial.print("Ch5: ");
  // Serial.print(readSwitch(4, false));
  // Serial.println();
  // // Print channel 6 (switch) boolean value
  // Serial.print("Ch5: ");
  // Serial.print(readSwitch(5, false));
  // Serial.println();