#ifndef AS5600_H
#define AS5600_H

#include <Arduino.h>
#include <Wire.h>

class AS5600 {
public:
    AS5600(uint8_t addr = AS5600_DEFAULT_ADDR);

    void begin(int sda_pin = -1, int scl_pin = -1, uint32_t freq = 400000);
    bool isConnected();

    // Funciones de diagnóstico y lectura
    void printStatus();
    void printAgcMagnitude();
    void printAngles();
    void printConf();
    
    uint8_t getStatus();
    uint8_t getAgc();
    uint16_t getMagnitude();
    uint16_t getRawAngle();
    uint16_t getAngle();
    float getAngleDegrees();

    // Funciones de configuración
    bool setOutsToPwm();
    uint16_t getConf();
    bool setConf(uint16_t conf);

    // Medición de PWM (requiere pin externo)
    bool measurePwm(int pwm_pin, float &duty_percent, float &freq_hz);

private:
    static const uint8_t AS5600_DEFAULT_ADDR = 0x36;

    // Registros AS5600
    static const uint8_t REG_ZMCO        = 0x00;
    static const uint8_t REG_ZPOS_MSB    = 0x01;
    static const uint8_t REG_ZPOS_LSB    = 0x02;
    static const uint8_t REG_MPOS_MSB    = 0x03;
    static const uint8_t REG_MPOS_LSB    = 0x04;
    static const uint8_t REG_MANG_MSB    = 0x05;
    static const uint8_t REG_MANG_LSB    = 0x06;
    static const uint8_t REG_CONF_MSB    = 0x07;
    static const uint8_t REG_CONF_LSB    = 0x08;
    static const uint8_t REG_RAW_ANG_MSB = 0x0C;
    static const uint8_t REG_RAW_ANG_LSB = 0x0D;
    static const uint8_t REG_ANG_MSB     = 0x0E;
    static const uint8_t REG_ANG_LSB     = 0x0F;
    static const uint8_t REG_STATUS      = 0x0B;
    static const uint8_t REG_AGC         = 0x1A;
    static const uint8_t REG_MAG_MSB     = 0x1B;
    static const uint8_t REG_MAG_LSB     = 0x1C;

    // Bits de STATUS
    static const uint8_t STATUS_MD = 1 << 5;
    static const uint8_t STATUS_ML = 1 << 4;
    static const uint8_t STATUS_MH = 1 << 3;
    
    // Bits de CONF
    static const uint16_t CONF_OUTS_MASK = (0b11 << 10);
    static const uint16_t CONF_OUTS_PWM  = (0b10 << 10);

    uint8_t _addr;

    // Utilitarios I2C
    bool i2cWrite8(uint8_t reg, uint8_t val);
    bool i2cRead8(uint8_t reg, uint8_t &val);
    bool i2cRead16(uint8_t reg, uint16_t &val);
    bool i2cWrite16(uint8_t reg, uint16_t val);
};

#endif // AS5600_H