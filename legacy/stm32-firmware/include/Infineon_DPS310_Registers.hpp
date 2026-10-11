#pragma once

namespace Infineon_DPS310
{
    static constexpr uint8_t REV_AND_PROD_ID = 0x10;

    enum class Register : uint8_t {
        PSR_B2 = 0x00,
        PSR_B1 = 0x01,
        PSR_B0 = 0x02,
        TMP_B2 = 0x03,
        TMP_B1 = 0x04,
        TMP_B0 = 0x05,
        PRS_CFG = 0x06,
        TMP_CFG = 0x07,
        MEAS_CFG = 0x08,
        CFG_REG = 0x09,
        RESET   = 0x0C,
        ID      = 0x0D,
        COEF    = 0x10,
        COEF_SRCE = 0x28,
    };

    enum RESET_BIT : uint8_t {
        SOFT_RST = (1 << 3) | (1 << 0),
    };

    struct CalibrationCoefficients {
        int16_t c0;
        int16_t c1;
        int32_t c00;
        int32_t c10;
        int16_t c01;
        int16_t c11;
        int16_t c20;
        int16_t c21;
        int16_t c30;
    };
}
