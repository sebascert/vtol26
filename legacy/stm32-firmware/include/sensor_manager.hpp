#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <ICM42688.h>
#include "hal_dps310.hpp"
#include <TinyGPS++.h>
#include <QMC5883LCompass.h>

#include "config.h"
#include "module_data_types.hpp"
#include "state_estimator.hpp"

namespace stallion {

class SensorManager {
public:
    SensorManager();
    ~SensorManager();

    bool inicializar();
    bool inicializar_imu();
    bool inicializar_barometro();
    bool inicializar_compass();

    bool update(uint64_t now_us);
    StateEstimate state() const;
    bool is_healthy() const;
    void reset();

    bool calibrate_imu();
    bool calibrate_compass();

    void diagnostico_serial();
    void debug_serial();
    SensorData get_raw_data() const;

    void diagnostico_i2c();
    void diagnostico_spi();

    const IMURawData& imu_data() const { return imu_data_; }
    const BaroRawData& baro_data() const { return baro_data_; }
    const MagRawData& mag_data() const { return mag_data_; }

    StateEstimator* estimator() { return estimator_; }

private:
    ICM42688 imu_;
    HalDPS310 baro_;
    QMC5883LCompass compass_;

    IMURawData imu_data_;
    BaroRawData baro_data_;
    MagRawData mag_data_;

    StateEstimator* estimator_;

    uint32_t contador_lecturas_;
    uint32_t last_imu_time_us_;
    uint32_t last_baro_time_us_;
    bool sensors_healthy_;

    float imu_accel_offset_[3];
    float imu_gyro_offset_[3];

    float alt_filtro_;
    bool alt_init_;
    float alt_rate_ms_;

    void leer_imu();
    
    void leer_barometro();
    void leer_compass();
    void alimentar_estimador(uint64_t now_us);
    void actualizar_estado_estimado();

    bool imu_data_valid(const IMURawData& data);
    bool baro_data_valid(const BaroRawData& data);
};

} // namespace stallion
