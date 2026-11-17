#pragma once
#include <stm32f4xx_hal.h>
#include "Infineon_DPS310_Registers.hpp"
#include "config.h"

class HalDPS310 {
public:
    HalDPS310();
    ~HalDPS310();

    bool init();
    bool readPT(float &pressure, float &temperature);

private:
    I2C_HandleTypeDef hi2c1;
    uint16_t address = 0x76 << 1; // HAL uses shifted address
    Infineon_DPS310::CalibrationCoefficients calibration;

    bool resetAndReadCalibration();
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t val);
    void readBlock(uint8_t reg, uint8_t *buf, uint8_t len);

    template<typename T>
    void getTwosComplement(T &raw, uint8_t length) {
        if (raw & ((T)1 << (length - 1))) {
            raw -= (T)1 << length;
        }
    }
};