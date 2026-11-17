#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ======= FLAGS =======
#define USE_GPS   1
#define USE_PITOT 1

// ======= IMU ICM42688-P =======
#define IMU_CS PA4

// ===== IMU CONFIG =====
#define IMU_ACCEL_RANGE 16   // ±16g
#define IMU_GYRO_RANGE  2000 // ±2000 dps

// ICM42688 conversion factors (PX4-style)
#define IMU_ACCEL_LSB_2G 16384.0f   // LSB/g para ±2g
#define IMU_ACCEL_LSB_16G 2048.0f   // LSB/g para ±16g (NUESTRO)
#define IMU_GYRO_LSB_2000DPS 16.4f  // LSB/(deg/s) para ±2000 dps (NUESTRO)

#define G_SI 9.81f                   // Gravedad en m/s²
#ifndef DEG_TO_RAD
#define DEG_TO_RAD (M_PI / 180.0f)
#endif

// ======= BARÓMETRO DPS310 =======
#define BARO_I2C Wire
#define BARO_ADDR 0x76

// ======= GPS  =======
#define GPS_UART Serial1
#define GPS_BAUDRATE 9600
#define GPS_RX PA10
#define GPS_TX PA9

// ======= PITOT (DESACTIVADO) =======
#define PITOT_PIN PA0
#define PITOT_SAMPLES 10
#define PITOT_VOLTAGE_SCALE 3.3

// ======= DEBUG =======
#define LED_PIN PC13

// ======= DPS310 =======
#define BARO_PRESSURE_RATE DPS310_64HZ
#define BARO_PRESSURE_OSR  DPS310_64SAMPLES
#define BARO_TEMP_RATE     DPS310_64HZ
#define BARO_TEMP_OSR      DPS310_8SAMPLES

// ======= CONSTANTES =======
#define DENSITY_SEA_LEVEL 1.225
#define QNH_STANDARD 1013.25

// ======= TIMEOUTS (ms) =======
#define SENSOR_TIMEOUT_MS 100
#define I2C_TIMEOUT_US 1000

// ======= SENSOR VALIDATION =======
#define ACCEL_MIN_G 0.5f
#define ACCEL_MAX_G 3.0f
#define GYRO_MAX_DPS 500.0f
#define BARO_PRESSURE_MIN 30000.0f
#define BARO_PRESSURE_MAX 120000.0f

// ======= CALIBRATION =======
#define IMU_CALIB_SAMPLES 100
#define COMPASS_CALIB_SAMPLES 50

// ======= LOOP FREQUENCIES (Hz) =======
#define SENSOR_FREQ_HZ 200
#define ATTITUDE_ESTIMATOR_FREQ_HZ 200
#define CONTROL_FREQ_HZ 50
#define DIAGNOSTIC_FREQ_HZ 2

#endif