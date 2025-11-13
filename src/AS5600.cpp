#include "AS5600.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "ota_telnet.h"


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
        broadcastIf(true, "[AS5600] ERROR leyendo STATUS (0x0B)");
        return;
    }

    bool md = status & STATUS_MD;
    bool ml = status & STATUS_ML;
    bool mh = status & STATUS_MH;

    String msg;
    msg.reserve(128);
    msg += "[AS5600] STATUS=0x";
    msg += String(status, HEX);
    msg += " md=";
    msg += md ? "1" : "0";
    msg += " ml=";
    msg += ml ? "1" : "0";
    msg += " mh=";
    msg += mh ? "1" : "0";
    if (!md) {
        msg += " (sin iman o campo insuficiente)";
    }
    broadcastIf(true, msg);
}

void AS5600::printAgcMagnitude() {
    uint8_t agc = getAgc();
    uint16_t mag = getMagnitude();

    String msg;
    msg.reserve(96);
    if (agc == 0xFF) {
        msg += "[AS5600] ERROR leyendo AGC";
    } else {
        msg += "[AS5600] AGC=";
        msg += String(agc);
    }
    if (mag == 0xFFFF) {
        msg += " MAG=ERR";
    } else {
        msg += " MAG=";
        msg += String(mag);
    }
    broadcastIf(true, msg);
}

void AS5600::printAngles() {
    uint16_t ang = getAngle();
    uint16_t raw = getRawAngle();
    
    float ang_deg = (ang * 360.0f) / 4096.0f;
    float raw_deg = (raw * 360.0f) / 4096.0f;

    String msg;
    msg.reserve(120);
    msg += "[AS5600] ANGLE=";
    if (ang == 0xFFFF) {
        msg += "ERR";
    } else {
        msg += String(ang);
        msg += " (";
        msg += String(ang_deg, 2);
        msg += "deg)";
    }
    msg += " RAW=";
    if (raw == 0xFFFF) {
        msg += "ERR";
    } else {
        msg += String(raw);
        msg += " (";
        msg += String(raw_deg, 2);
        msg += "deg)";
    }
    broadcastIf(true, msg);
}

void AS5600::printConf() {
    uint16_t conf = getConf();
    if (conf == 0xFFFF) {
        broadcastIf(true, "[AS5600] ERROR leyendo CONF (0x07-0x08)");
        return;
    }
    uint16_t outs = (conf & CONF_OUTS_MASK) >> 10;
    String msg;
    msg.reserve(96);
    msg += "[AS5600] CONF=0x";
    msg += String(conf, HEX);
    msg += " outs=0b";
    msg += String(outs, BIN);
    msg += (outs == 0b10) ? " (PWM)" : " (no PWM)";
    broadcastIf(true, msg);
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
void runAs5600SelfTest(AS5600& sensor, bool log) {
  broadcastIf(log, "[AS5600][TEST] Iniciando diagnostico basico");

  const bool connected = sensor.isConnected();
  broadcastIf(log, String("[AS5600][TEST] Conectado al bus I2C: ") + (connected ? "SI" : "NO"));
  if (!connected) {
    broadcastIf(log, "[AS5600][TEST] Abortado: sin respuesta del dispositivo");
    return;
  }

  const uint8_t status = sensor.getStatus();
  if (status == 0xFF) {
    broadcastIf(log, "[AS5600][TEST][ERROR] No se pudo leer STATUS (0x0B)");
  } else {
    String msg = "[AS5600][TEST] STATUS (0x0B) = 0x";
    msg += String(status, HEX);
    msg += " md=";
    msg += (status & 0x20) ? "1" : "0";
    msg += " ml=";
    msg += (status & 0x10) ? "1" : "0";
    msg += " mh=";
    msg += (status & 0x08) ? "1" : "0";
    broadcastIf(log, msg);
  }

  const uint16_t conf = sensor.getConf();
  if (conf == 0xFFFF) {
    broadcastIf(log, "[AS5600][TEST][ERROR] No se pudo leer CONF (0x07-0x08)");
  } else {
    String msg = "[AS5600][TEST] CONF (0x07-0x08) = 0x";
    msg += String(conf, HEX);
    broadcastIf(log, msg);
  }

  const uint8_t agc = sensor.getAgc();
  if (agc == 0xFF) {
    broadcastIf(log, "[AS5600][TEST][ERROR] No se pudo leer AGC (0x1A)");
  } else {
    String msg = "[AS5600][TEST] AGC (0x1A) = ";
    msg += String(agc);
    broadcastIf(log, msg);
  }

  const uint16_t magnitude = sensor.getMagnitude();
  if (magnitude == 0xFFFF) {
    broadcastIf(log, "[AS5600][TEST][ERROR] No se pudo leer MAGNITUDE (0x1B-0x1C)");
  } else {
    String msg = "[AS5600][TEST] MAG (0x1B-0x1C) = ";
    msg += String(magnitude);
    broadcastIf(log, msg);
  }

  const uint16_t rawAngle = sensor.getRawAngle();
  if (rawAngle == 0xFFFF) {
    broadcastIf(log, "[AS5600][TEST][ERROR] No se pudo leer RAW_ANGLE (0x0C-0x0D)");
  } else {
    String msg = "[AS5600][TEST] RAW_ANGLE (0x0C-0x0D) = ";
    msg += String(rawAngle);
    broadcastIf(log, msg);
  }

  const uint16_t angle = sensor.getAngle();
  if (angle == 0xFFFF) {
    broadcastIf(log, "[AS5600][TEST][ERROR] No se pudo leer ANGLE (0x0E-0x0F)");
  } else {
    const float angleDeg = (angle * 360.0f) / 4096.0f;
    String msg = "[AS5600][TEST] ANGLE (0x0E-0x0F) = ";
    msg += String(angle);
    msg += " -> ";
    msg += String(angleDeg, 2);
    msg += "deg";
    broadcastIf(log, msg);
  }

  broadcastIf(log, "[AS5600][TEST] Diagnostico completado");
}

void taskAs5600Monitor(void* parameter) {
  AS5600MonitorConfig* cfg = static_cast<AS5600MonitorConfig*>(parameter);
  AS5600* sensor = (cfg != nullptr) ? cfg->sensor : nullptr;
  const bool log = (cfg != nullptr) ? cfg->log : false;
  const TickType_t period = (cfg != nullptr && cfg->period > 0) ? cfg->period : pdMS_TO_TICKS(30);
  const TickType_t logInterval = (cfg != nullptr && cfg->logInterval > 0) ? cfg->logInterval : pdMS_TO_TICKS(500);

  if (sensor == nullptr) {
    vTaskDelete(nullptr);
    return;
  }

  TickType_t lastWake = xTaskGetTickCount();
  TickType_t lastLog = lastWake - ((logInterval > 0) ? logInterval : 0);

  for (;;) {
    const TickType_t now = xTaskGetTickCount();
    const bool shouldLog = log && (logInterval == 0 || (now - lastLog) >= logInterval);
    const bool connected = sensor->isConnected();

    if (!connected) {
      if (shouldLog) {
        broadcastIf(true, "[AS5600][MON] connected=NO (sin respuesta I2C)");
        lastLog = now;
      }
      vTaskDelayUntil(&lastWake, period);
      continue;
    }

    const uint8_t status = sensor->getStatus();
    const uint16_t rawAngle = sensor->getRawAngle();
    const uint16_t angle = sensor->getAngle();

    if (shouldLog) {
      const uint16_t magnitude = sensor->getMagnitude();

      String msg;
      msg.reserve(160);
      msg += "[AS5600][MON] connected=YES";

      if (status == 0xFF) {
        msg += " status=ERR magnet=UNKNOWN";
      } else {
        msg += " status=0x";
        msg += String(status, HEX);

        const bool magnetDetected = (status & 0x20) != 0;
        const bool magnetTooWeak = (status & 0x10) != 0;
        const bool magnetTooStrong = (status & 0x08) != 0;

        msg += " magnet=";
        if (!magnetDetected) {
          msg += "NO";
        } else {
          msg += "OK";
          if (magnetTooWeak) {
            msg += "-WEAK";
          }
          if (magnetTooStrong) {
            msg += "-STRONG";
          }
        }

        msg += " md=";
        msg += magnetDetected ? "1" : "0";
        msg += " ml=";
        msg += magnetTooWeak ? "1" : "0";
        msg += " mh=";
        msg += magnetTooStrong ? "1" : "0";
      }

      if (rawAngle == 0xFFFF) {
        msg += " raw=ERR";
      } else {
        msg += " raw=";
        msg += String(rawAngle);
      }

      if (angle == 0xFFFF) {
        msg += " angle=ERR";
      } else {
        const float angleDeg = (angle * 360.0f) / 4096.0f;
        msg += " angle=";
        msg += String(angleDeg, 2);
        msg += "deg";
      }

      if (magnitude == 0xFFFF) {
        msg += " mag=ERR";
      } else {
        msg += " mag=";
        msg += String(magnitude);
      }

      broadcastIf(true, msg);
      lastLog = now;
    }

    vTaskDelayUntil(&lastWake, period);
  }
}
