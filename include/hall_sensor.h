#ifndef HALL_SENSOR_H
#define HALL_SENSOR_H

#include <Arduino.h>

class HallSensor {
private:
    const int pwmPin;
    volatile unsigned long pulseStartTime;
    volatile unsigned long pulseWidth;
    volatile bool measurementComplete;
    static void IRAM_ATTR isrPulseTimer(void* arg);

public:
    HallSensor(int pin);
    void begin();
    float getAngle();  // Returns angle in degrees (0-360)
    bool isAngleReady() const;
};

#endif // HALL_SENSOR_H
