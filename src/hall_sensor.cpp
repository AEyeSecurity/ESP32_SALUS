#include "hall_sensor.h"

HallSensor::HallSensor(int pin) : pwmPin(pin), pulseStartTime(0), pulseWidth(0), measurementComplete(false) {}

void IRAM_ATTR HallSensor::isrPulseTimer(void* arg) {
    HallSensor* sensor = static_cast<HallSensor*>(arg);
    bool currentLevel = digitalRead(sensor->pwmPin);
    unsigned long currentTime = micros();
    
    static bool inDataSection = false;
    static unsigned long dataStartTime = 0;
    
    if (currentLevel == HIGH && !inDataSection) {
        // Start of data section after initial 128 high periods
        dataStartTime = currentTime;
        inDataSection = true;
    }
    else if (currentLevel == LOW && inDataSection) {
        // End of data section
        sensor->pulseWidth = currentTime - dataStartTime;
        sensor->measurementComplete = true;
        inDataSection = false;
    }
}

void HallSensor::begin() {
    pinMode(pwmPin, INPUT);
    
    // Attach interrupt to the PWM pin
    attachInterruptArg(digitalPinToInterrupt(pwmPin), isrPulseTimer, this, CHANGE);
}

float HallSensor::getAngle() {
    if (!measurementComplete) {
        return -1.0f; // Invalid angle
    }
    
    // The data part is 4095 periods, and each period represents 1/4096th of 360 degrees
    // pulseWidth represents the high time in the data section
    float ratio = static_cast<float>(pulseWidth) / 4095.0f;
    float angle = ratio * 360.0f;
    
    measurementComplete = false; // Reset for next measurement
    return angle;
}

bool HallSensor::isAngleReady() const {
    return measurementComplete;
}
