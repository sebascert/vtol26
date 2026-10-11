#include "DPS310.hpp"
#include <Arduino.h>

static const uint8_t DPS310_ADDRESSES[] = { 0x76, 0x77 };

struct I2CPinConfig {
    uint8_t sda;
    uint8_t scl;
    const char* label;
};

static const I2CPinConfig I2C_CONFIGS[] = {
    { PB9, PB8, "PB9/PB8" },
    { PB7, PB6, "PB7/PB6" },
    { PC9, PA8, "PC9/PA8" }
};

static const uint8_t SPI_CS_CANDIDATES[] = { PA4, PB12, PB13, PB14, PC4, PC5 };

bool DPS310::detectSensorI2C() {
    for (const auto &cfg : I2C_CONFIGS) {
        Serial.print("[DPS310] Probando I2C ");
        Serial.println(cfg.label);

        Wire.begin(cfg.sda, cfg.scl);
        Wire.setClock(400000);
        delay(10);

        for (uint8_t addr : DPS310_ADDRESSES) {
            Wire.beginTransmission(addr);
            int result = Wire.endTransmission();
            if (result != 0) {
                Serial.print("[DPS310] No ack en ");
                Serial.print(cfg.label);
                Serial.print(" addr 0x");
                Serial.println(addr, HEX);
                continue;
            }

            Serial.print("[DPS310] Ack I2C en ");
            Serial.print(cfg.label);
            Serial.print(" addr 0x");
            Serial.println(addr, HEX);

            Wire.beginTransmission(addr);
            Wire.write(0x0D);
            if (Wire.endTransmission(false) != 0) {
                Serial.println("[DPS310] Fallo lectura ID (endTransmission)");
                continue;
            }

            Wire.requestFrom(addr, (uint8_t)1);
            if (!Wire.available()) {
                Serial.println("[DPS310] Fallo lectura ID (sin datos)");
                continue;
            }

            uint8_t id = Wire.read();
            Serial.print("[DPS310] ID leido: 0x");
            Serial.println(id, HEX);
            if (id != 0x10) {
                Serial.println("[DPS310] ID incorrecto");
                continue;
            }

            _addr = addr;
            _bus_type = BusType::I2C;
            Serial.print("[DPS310] DPS310 detectado via I2C ");
            Serial.println(cfg.label);
            return true;
        }
    }
    return false;
}

bool DPS310::detectSensorSPI() {
    SPI.begin();
    Serial.println("[DPS310] Probando DPS310 via SPI...");

    for (uint8_t cs : SPI_CS_CANDIDATES) {
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
    }

    for (uint8_t cs : SPI_CS_CANDIDATES) {
        Serial.print("[DPS310] Probando CS ");
        Serial.println(cs);

        digitalWrite(cs, LOW);
        delayMicroseconds(10);

        SPI.beginTransaction(_spi_settings);
        SPI.transfer(0x8D);
        uint8_t id = SPI.transfer(0x00);
        SPI.endTransaction();

        digitalWrite(cs, HIGH);

        Serial.print("[DPS310] WHO_AM_I SPI: 0x");
        Serial.println(id, HEX);

        if (id == 0x10) {
            _spi_cs = cs;
            _bus_type = BusType::SPI;
            Serial.print("[DPS310] DPS310 detectado via SPI CS ");
            Serial.println(cs);
            return true;
        }
    }

    Serial.println("[DPS310] No se detecto DPS310 via SPI");
    return false;
}

bool DPS310::detectSensor() {
    if (detectSensorI2C()) {
        return true;
    }

    Serial.println("[DPS310] I2C fallido, intentando SPI...");
    return detectSensorSPI();
}

bool DPS310::init() {
    Serial.println("[DPS310] Buscando sensor...");

    if (!detectSensor()) {
        Serial.println("[DPS310] No se detecto DPS310 en ningun bus");
        return false;
    }

    writeRegister(0x0C, 0x09);
    delay(50);

    uint32_t t0 = millis();
    while (!(readRegister(0x08) & 0x40)) {
        if (millis() - t0 > 200) {
            Serial.println("[DPS310] Timeout ready");
            return false;
        }
    }

    uint8_t id = readRegister(0x0D);
    Serial.print("[DPS310] ID: 0x");
    Serial.println(id, HEX);

    if (id != 0x10) {
        Serial.println("[DPS310] ID incorrecto");
        return false;
    }

    if (!resetAndReadCalibration()) {
        Serial.println("[DPS310] Error calibracion");
        return false;
    }

    writeRegister(0x06, 0x74);
    writeRegister(0x07, 0xD4);
    writeRegister(0x09, 0x0C);
    writeRegister(0x08, 0x07);

    Serial.println("[DPS310] OK");
    return true;
}

// ================= I2C / SPI =================

uint8_t DPS310::readRegister(uint8_t reg) {
    if (_bus_type == BusType::I2C) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return 0xFF;
        Wire.requestFrom((uint8_t)_addr, (uint8_t)1);
        if (Wire.available()) return Wire.read();
        return 0xFF;
    }

    // SPI
    if (_bus_type != BusType::SPI) {
        return 0xFF;
    }

    digitalWrite(_spi_cs, LOW);
    SPI.beginTransaction(_spi_settings);
    SPI.transfer(0x80 | reg);
    uint8_t value = SPI.transfer(0x00);
    SPI.endTransaction();
    digitalWrite(_spi_cs, HIGH);
    return value;
}

void DPS310::writeRegister(uint8_t reg, uint8_t val) {
    if (_bus_type == BusType::I2C) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        Wire.write(val);
        Wire.endTransmission();
        return;
    }

    if (_bus_type != BusType::SPI) {
        return;
    }

    digitalWrite(_spi_cs, LOW);
    SPI.beginTransaction(_spi_settings);
    SPI.transfer(reg & 0x7F);
    SPI.transfer(val);
    SPI.endTransaction();
    digitalWrite(_spi_cs, HIGH);
}

void DPS310::readBlock(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (_bus_type == BusType::I2C) {
        Wire.beginTransmission(_addr);
        Wire.write(reg);
        if (Wire.endTransmission(false) != 0) return;
        Wire.requestFrom((uint8_t)_addr, len);
        for (uint8_t i = 0; i < len && Wire.available(); i++) {
            buf[i] = Wire.read();
        }
        return;
    }

    if (_bus_type != BusType::SPI) {
        return;
    }

    digitalWrite(_spi_cs, LOW);
    SPI.beginTransaction(_spi_settings);
    SPI.transfer(0x80 | reg);
    for (uint8_t i = 0; i < len; i++) {
        buf[i] = SPI.transfer(0x00);
    }
    SPI.endTransaction();
    digitalWrite(_spi_cs, HIGH);
}

// ================= CALIB =================

bool DPS310::resetAndReadCalibration() {

    writeRegister(0x0C, 0x09);
    delay(50);

    uint8_t coef[18];
    readBlock(0x10, coef, 18);

    _calibration.c0  = ((coef[0] << 4) | (coef[1] >> 4));
    getTwosComplement(_calibration.c0, 12);

    _calibration.c1  = (((coef[1] & 0x0F) << 8) | coef[2]);
    getTwosComplement(_calibration.c1, 12);

    _calibration.c00 = ((int32_t)coef[3] << 12) |
                       ((int32_t)coef[4] << 4) |
                       ((coef[5] >> 4) & 0x0F);
    getTwosComplement(_calibration.c00, 20);

    _calibration.c10 = (((int32_t)(coef[5] & 0x0F) << 16) |
                        ((int32_t)coef[6] << 8) |
                        coef[7]);
    getTwosComplement(_calibration.c10, 20);

    _calibration.c01 = ((int16_t)coef[8] << 8) | coef[9];
    getTwosComplement(_calibration.c01, 16);

    _calibration.c11 = ((int16_t)coef[10] << 8) | coef[11];
    getTwosComplement(_calibration.c11, 16);

    _calibration.c20 = ((int16_t)coef[12] << 8) | coef[13];
    getTwosComplement(_calibration.c20, 16);

    _calibration.c21 = ((int16_t)coef[14] << 8) | coef[15];
    getTwosComplement(_calibration.c21, 16);

    _calibration.c30 = ((int16_t)coef[16] << 8) | coef[17];
    getTwosComplement(_calibration.c30, 16);

    return true;
}

// ================= LECTURA =================

bool DPS310::readPT(float &pressure, float &temperature) {

    uint8_t pbuf[3], tbuf[3];

    readBlock(0x00, pbuf, 3);
    readBlock(0x03, tbuf, 3);

    int32_t praw = (pbuf[0] << 16) | (pbuf[1] << 8) | pbuf[2];
    if (praw & 0x800000) praw -= 0x1000000;

    int32_t traw = (tbuf[0] << 16) | (tbuf[1] << 8) | tbuf[2];
    if (traw & 0x800000) traw -= 0x1000000;

    float k = 253952.0f;

    float Praw_sc = (float)praw / k;
    float Traw_sc = (float)traw / k;

    temperature = _calibration.c0 * 0.5f + _calibration.c1 * Traw_sc;

    pressure =
        _calibration.c00 +
        Praw_sc * (_calibration.c10 +
        Praw_sc * (_calibration.c20 +
        Praw_sc * _calibration.c30)) +
        Traw_sc * _calibration.c01 +
        Traw_sc * Praw_sc * (_calibration.c11 +
        Praw_sc * _calibration.c21);

    return true;
}