#pragma once
#include <Wire.h>
#include <SPI.h>
#include "Infineon_DPS310_Registers.hpp"
#include "config.h"

class DPS310 {
public:
    bool init();  // 👈 autodetección TOTAL

    bool readPT(float &pressure, float &temperature);

private:
    enum class BusType {
        NONE,
        I2C,
        SPI
    } _bus_type = BusType::NONE;

    uint8_t _addr = 0;          // 👈 dirección detectada I2C
    uint8_t _spi_cs = 0;        // 👈 pin CS detectado SPI
    SPISettings _spi_settings{1000000, MSBFIRST, SPI_MODE0};

    Infineon_DPS310::CalibrationCoefficients _calibration{};

    bool detectSensor();
    bool detectSensorI2C();
    bool detectSensorSPI();

    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t val);
    void readBlock(uint8_t reg, uint8_t *buf, uint8_t len);

    bool resetAndReadCalibration();

    template<typename T>
    void getTwosComplement(T &raw, uint8_t length) {
        if (raw & ((T)1 << (length - 1))) {
            raw -= (T)1 << length;
        }
    }
};