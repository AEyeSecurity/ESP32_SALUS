#include "AS5600.h"

AS5600::AS5600(uint8_t addr) : _addr(addr) {}

void AS5600::begin(int sda_pin, int scl_pin, uint32_t freq) {
    if (sda_pin >= 0 && scl_pin >= 0) {
        Wire.begin(sda_pin, scl_pin, freq);
    } else {
        Wire.begin();
    }
    delay(10);
}

bool AS5600::isConnected() {
    Wire.beginTransmission(_addr);
    return (Wire.endTransmission() == 0);
}

void AS5600::printStatus() {
    uint8_t status = getStatus();
    if (status == 0xFF) {
        Serial.println(F("[ERROR] No se pudo leer STATUS (0x0B)."));
        return;
    }

    bool md = status & STATUS_MD;
    bool ml = status & STATUS_ML;
    bool mh = status & STATUS_MH;

    Serial.print(F("STATUS 0x0B = 0b"));
    for (int i = 7; i >= 0; --i) Serial.print((status >> i) & 1);
    Serial.println();

    Serial.print(F("  MD (magnet detected): ")); Serial.println(md ? "SI" : "NO");
    Serial.print(F("  ML (muy débil): "));      Serial.println(ml ? "SI" : "NO");
    Serial.print(F("  MH (muy fuerte): "));     Serial.println(mh ? "SI" : "NO");

    if (!md) {
        Serial.println(F("  -> Sin imán o campo insuficiente: la salida OUT queda LOW (sin PWM)."));
    }
}

void AS5600::printAgcMagnitude() {
    uint8_t agc = getAgc();
    uint16_t mag = getMagnitude();

    if (agc == 0xFF) Serial.println(F("[ERROR] No se pudo leer AGC (0x1A)."));
    if (mag == 0xFFFF) Serial.println(F("[ERROR] No se pudo leer MAGNITUDE (0x1B–0x1C)."));
    
    Serial.print(F("AGC: ")); Serial.println(agc);
    Serial.print(F("MAGNITUDE: ")); Serial.println(mag);
}

void AS5600::printAngles() {
    uint16_t ang = getAngle();
    uint16_t raw = getRawAngle();

    if (ang == 0xFFFF) Serial.println(F("[ERROR] No se pudo leer ANGLE (0x0E–0x0F)."));
    if (raw == 0xFFFF) Serial.println(F("[ERROR] No se pudo leer RAW_ANGLE (0x0C–0x0D)."));
    
    float ang_deg = (ang * 360.0f) / 4096.0f;
    float raw_deg = (raw * 360.0f) / 4096.0f;

    Serial.print(F("ANGLE: ")); Serial.print(ang);
    Serial.print(F(" (")); Serial.print(ang_deg, 2); Serial.println(F("°)"));
    Serial.print(F("RAW_ANGLE: ")); Serial.print(raw);
    Serial.print(F(" (")); Serial.print(raw_deg, 2); Serial.println(F("°)"));
}

void AS5600::printConf() {
    uint16_t conf = getConf();
    if (conf == 0xFFFF) {
        Serial.println(F("[ERROR] No se pudo leer CONF (0x07–0x08)."));
        return;
    }
    Serial.print(F("CONF = 0x"));
    Serial.println(conf, HEX);

    uint16_t outs = (conf & CONF_OUTS_MASK) >> 10;
    Serial.print(F("  OUTS (bits 11:10) = 0b"));
    Serial.println(outs, BIN);
    if (outs == 0b10) {
        Serial.println(F("  -> Salida configurada en PWM."));
    } else {
        Serial.println(F("  -> Salida NO está en PWM (prob. analógica)."));
    }
}

uint8_t AS5600::getStatus() {
    uint8_t val;
    return i2cRead8(REG_STATUS, val) ? val : 0xFF;
}

uint8_t AS5600::getAgc() {
    uint8_t val;
    return i2cRead8(REG_AGC, val) ? val : 0xFF;
}

uint16_t AS5600::getMagnitude() {
    uint16_t val;
    return i2cRead16(REG_MAG_MSB, val) ? val : 0xFFFF;
}

uint16_t AS5600::getRawAngle() {
    uint16_t val;
    return i2cRead16(REG_RAW_ANG_MSB, val) ? val : 0xFFFF;
}

uint16_t AS5600::getAngle() {
    uint16_t val;
    return i2cRead16(REG_ANG_MSB, val) ? val : 0xFFFF;
}

float AS5600::getAngleDegrees() {
    uint16_t ang = getAngle();
    if (ang == 0xFFFF) return -1.0f;
    return (ang * 360.0f) / 4096.0f;
}

bool AS5600::setOutsToPwm() {
    uint16_t conf = getConf();
    if (conf == 0xFFFF) return false;
    conf &= ~CONF_OUTS_MASK;
    conf |= CONF_OUTS_PWM;
    return setConf(conf);
}

uint16_t AS5600::getConf() {
    uint16_t val;
    return i2cRead16(REG_CONF_MSB, val) ? val : 0xFFFF;
}

bool AS5600::setConf(uint16_t conf) {
    return i2cWrite16(REG_CONF_MSB, conf);
}

bool AS5600::measurePwm(int pwm_pin, float &duty_percent, float &freq_hz) {
    if (pwm_pin < 0) return false;

    if (pulseIn((uint8_t)pwm_pin, HIGH, 200000) == 0 &&
        pulseIn((uint8_t)pwm_pin, LOW,  200000) == 0) {
        return false;
    }

    unsigned long tHigh = pulseIn((uint8_t)pwm_pin, HIGH, 200000);
    unsigned long tLow  = pulseIn((uint8_t)pwm_pin, LOW,  200000);
    if (tHigh == 0 || tLow == 0) return false;

    float period_us = (float)tHigh + (float)tLow;
    duty_percent = (100.0f * (float)tHigh) / period_us;
    freq_hz = 1e6f / period_us;
    return true;
}

// I2C Helpers
bool AS5600::i2cWrite8(uint8_t reg, uint8_t val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(val);
    return (Wire.endTransmission() == 0);
}

bool AS5600::i2cRead8(uint8_t reg, uint8_t &val) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;
    if (Wire.requestFrom((int)_addr, 1) != 1) return false;
    val = Wire.read();
    return true;
}

bool AS5600::i2cRead16(uint8_t reg, uint16_t &val) {
    uint8_t msb, lsb;
    if (!i2cRead8(reg, msb)) return false;
    if (!i2cRead8(reg + 1, lsb)) return false;
    val = ((uint16_t)msb << 8) | lsb;
    return true;
}

bool AS5600::i2cWrite16(uint8_t reg, uint16_t val) {
    uint8_t msb = (val >> 8) & 0xFF;
    uint8_t lsb = val & 0xFF;
    if (!i2cWrite8(reg, msb)) return false;
    if (!i2cWrite8(reg + 1, lsb)) return false;
    return true;
}