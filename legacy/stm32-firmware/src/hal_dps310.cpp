#include "hal_dps310.hpp"
#include <Arduino.h>

HalDPS310::HalDPS310() {
    // Initialize I2C handle
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 400000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
}

HalDPS310::~HalDPS310() {
    HAL_I2C_DeInit(&hi2c1);
}

bool HalDPS310::init() {
    // Enable clocks
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_I2C1_CLK_ENABLE();

    // Configure pins PB8 (SCL) and PB9 (SDA)
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = GPIO_PIN_8 | GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // Initialize I2C
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Serial.println("[HalDPS310] Error initializing I2C");
        return false;
    }

    // Reset sensor
    writeRegister(0x0C, 0x09);
    delay(50);

    // Wait for ready
    uint32_t t0 = millis();
    while (!(readRegister(0x08) & 0x40)) {
        if (millis() - t0 > 200) {
            Serial.println("[HalDPS310] Timeout waiting for ready");
            return false;
        }
    }

    // Check ID
    uint8_t id = readRegister(0x0D);
    Serial.print("[HalDPS310] ID: 0x");
    Serial.println(id, HEX);
    if (id != 0x10) {
        Serial.println("[HalDPS310] Wrong ID");
        return false;
    }

    // Read calibration
    if (!resetAndReadCalibration()) {
        Serial.println("[HalDPS310] Calibration error");
        return false;
    }

    // Configure sensor
    writeRegister(0x06, 0x74); // Pressure config
    writeRegister(0x07, 0xD4); // Temperature config
    writeRegister(0x09, 0x0C); // Interrupt config
    writeRegister(0x08, 0x07); // Measurement config

    Serial.println("[HalDPS310] Init OK");
    return true;
}

bool HalDPS310::readPT(float &pressure, float &temperature) {
    // Start measurement
    writeRegister(0x08, 0x07);

    // Wait for measurement ready
    uint32_t t0 = millis();
    while (!(readRegister(0x08) & 0x20)) {
        if (millis() - t0 > 100) {
            Serial.println("[HalDPS310] Timeout waiting for measurement");
            return false;
        }
    }

    // Read raw data
    uint8_t buf[6];
    readBlock(0x00, buf, 6);

    int32_t raw_pressure = (buf[0] << 16) | (buf[1] << 8) | buf[2];
    int32_t raw_temperature = (buf[3] << 16) | (buf[4] << 8) | buf[5];

    getTwosComplement(raw_pressure, 24);
    getTwosComplement(raw_temperature, 24);

    // Calculate scaled values
    float p_sc = raw_pressure / (float)(1 << 23);
    float t_sc = raw_temperature / (float)(1 << 24);

    // Calculate temperature
    float t_comp = calibration.c0 * 0.5f + calibration.c1 * t_sc;
    temperature = t_comp;

    // Calculate pressure
    float p_comp = calibration.c00 + p_sc * (calibration.c10 + p_sc * (calibration.c20 + p_sc * calibration.c30)) +
                   t_sc * calibration.c01 + t_sc * p_sc * (calibration.c11 + p_sc * calibration.c21);
    pressure = p_comp;

    return true;
}

bool HalDPS310::resetAndReadCalibration() {
    // Reset calibration
    writeRegister(0x0C, 0x89);
    delay(50);

    // Read calibration coefficients
    uint8_t buf[18];
    readBlock(0x10, buf, 18);

    calibration.c0 = (buf[0] << 4) | (buf[1] >> 4);
    getTwosComplement(calibration.c0, 12);

    calibration.c1 = ((buf[1] & 0x0F) << 8) | buf[2];
    getTwosComplement(calibration.c1, 12);

    calibration.c00 = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    getTwosComplement(calibration.c00, 20);

    calibration.c10 = ((buf[5] & 0x0F) << 16) | (buf[6] << 8) | buf[7];
    getTwosComplement(calibration.c10, 20);

    calibration.c01 = (buf[8] << 8) | buf[9];
    getTwosComplement(calibration.c01, 16);

    calibration.c11 = (buf[10] << 8) | buf[11];
    getTwosComplement(calibration.c11, 16);

    calibration.c20 = (buf[12] << 8) | buf[13];
    getTwosComplement(calibration.c20, 16);

    calibration.c21 = (buf[14] << 8) | buf[15];
    getTwosComplement(calibration.c21, 16);

    calibration.c30 = (buf[16] << 8) | buf[17];
    getTwosComplement(calibration.c30, 16);

    return true;
}

uint8_t HalDPS310::readRegister(uint8_t reg) {
    uint8_t value;
    if (HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFF;
    }
    if (HAL_I2C_Master_Receive(&hi2c1, address, &value, 1, HAL_MAX_DELAY) != HAL_OK) {
        return 0xFF;
    }
    return value;
}

void HalDPS310::writeRegister(uint8_t reg, uint8_t val) {
    uint8_t data[2] = {reg, val};
    HAL_I2C_Master_Transmit(&hi2c1, address, data, 2, HAL_MAX_DELAY);
}

void HalDPS310::readBlock(uint8_t reg, uint8_t *buf, uint8_t len) {
    if (HAL_I2C_Master_Transmit(&hi2c1, address, &reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        return;
    }
    HAL_I2C_Master_Receive(&hi2c1, address, buf, len, HAL_MAX_DELAY);
}