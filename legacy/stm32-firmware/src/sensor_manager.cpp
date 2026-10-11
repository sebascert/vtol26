#include "sensor_manager.hpp"
#include <cmath>
#include <cstring>

namespace stallion {

SensorManager::SensorManager()
    : imu_(SPI, IMU_CS),
      estimator_(nullptr),
      contador_lecturas_(0),
      last_imu_time_us_(0),
      last_baro_time_us_(0),
      sensors_healthy_(false),
      alt_filtro_(0.0f),
      alt_init_(false),
      alt_rate_ms_(0.0f) {
    
    memset(&imu_data_, 0, sizeof(imu_data_));
    memset(&baro_data_, 0, sizeof(baro_data_));
    memset(&mag_data_, 0, sizeof(mag_data_));
    memset(imu_accel_offset_, 0, sizeof(imu_accel_offset_));
    memset(imu_gyro_offset_, 0, sizeof(imu_gyro_offset_));
    
    estimator_ = new StateEstimator();
}

SensorManager::~SensorManager() {
    delete estimator_;
}

bool SensorManager::inicializar() {
    Serial.println("\n=== INICIANDO SENSORES ===");
    
    bool ok_imu = inicializar_imu();
    bool ok_baro = inicializar_barometro();
    bool ok_compass = inicializar_compass();
    
    if (!ok_imu) Serial.println("IMU: FALLO");
    else Serial.println("IMU: OK");
    
    if (!ok_baro) Serial.println("BARO: FALLO");
    else Serial.println("BARO: OK");
    
    if (!ok_compass) Serial.println("COMPASS: FALLO");
    else Serial.println("COMPASS: OK");
    
    sensors_healthy_ = ok_imu && ok_baro;
    
    if (sensors_healthy_) {
        Serial.println("Calibrando IMU...");
        calibrate_imu();
    }
    
    return sensors_healthy_;
}

bool SensorManager::inicializar_imu() {
    SPI.begin();
    
    int status = imu_.begin();
    if (status < 0) {
        Serial.print("IMU fallo: ");
        Serial.println(status);
        return false;
    }
    
    imu_.setAccelFS(ICM42688::gpm16);
    imu_.setGyroFS(ICM42688::dps2000);
    
    last_imu_time_us_ = micros();
    return true;
}

bool SensorManager::inicializar_barometro() {
    Serial.println("[BARO] Detectando...");

    if (!baro_.init()) {
        Serial.println("[BARO] FALLO");
        return false;
    }

    Serial.println("[BARO] OK");
    last_baro_time_us_ = micros();
    return true;
}

bool SensorManager::inicializar_compass() {
    compass_.init();
    return true;
}

bool SensorManager::calibrate_imu() {
    float accel_sum[3] = {0, 0, 0};
    float gyro_sum[3] = {0, 0, 0};
    
    Serial.println("Leyendo IMU 100 veces...");
    for (int i = 0; i < IMU_CALIB_SAMPLES; i++) {
        imu_.getAGT();
        
        // Valores ya en g y dps
        float ax_raw = imu_.accX() * G_SI;
        float ay_raw = imu_.accY() * G_SI;
        float az_raw = imu_.accZ() * G_SI;
        
        float gx_raw = imu_.gyrX() * DEG_TO_RAD;
        float gy_raw = imu_.gyrY() * DEG_TO_RAD;
        float gz_raw = imu_.gyrZ() * DEG_TO_RAD;
        
        accel_sum[0] += ax_raw;
        accel_sum[1] += ay_raw;
        accel_sum[2] += az_raw;
        
        gyro_sum[0] += gx_raw;
        gyro_sum[1] += gy_raw;
        gyro_sum[2] += gz_raw;
    }
    
    // Promedios
    for (int i = 0; i < 3; i++) {
        imu_accel_offset_[i] = accel_sum[i] / IMU_CALIB_SAMPLES;
        imu_gyro_offset_[i] = gyro_sum[i] / IMU_CALIB_SAMPLES;
    }
    
    // Z debe estar cerca de -g (hacia arriba)
    imu_accel_offset_[2] -= G_SI;
    
    Serial.print("Accel offset: ");
    Serial.print(imu_accel_offset_[0]); Serial.print(", ");
    Serial.print(imu_accel_offset_[1]); Serial.print(", ");
    Serial.println(imu_accel_offset_[2]);
    
    Serial.print("Gyro offset: ");
    Serial.print(imu_gyro_offset_[0]); Serial.print(", ");
    Serial.print(imu_gyro_offset_[1]); Serial.print(", ");
    Serial.println(imu_gyro_offset_[2]);
    
    return true;
}

bool SensorManager::calibrate_compass() {
    Serial.println("Calibration de compass...");
    return true;
}

void SensorManager::leer_imu() {
    uint32_t now_us = micros();
    
    if (now_us - last_imu_time_us_ < (1000000 / SENSOR_FREQ_HZ)) {
        return; // No es tiempo de leer
    }
    
    imu_.getAGT();
    
    // Conversión a SI con escala correcta
    float ax_raw = imu_.accX() * G_SI;
    float ay_raw = imu_.accY() * G_SI;
    float az_raw = imu_.accZ() * G_SI;
    
    float gx_raw = imu_.gyrX() * DEG_TO_RAD;
    float gy_raw = imu_.gyrY() * DEG_TO_RAD;
    float gz_raw = imu_.gyrZ() * DEG_TO_RAD;
    
    const float lf = 0.85f;
    const float inv_lf = 1.0f - lf;

    if (imu_data_.timestamp_us != 0) {
        imu_data_.accel_x = lf * imu_data_.accel_x + inv_lf * (ax_raw - imu_accel_offset_[0]);
        imu_data_.accel_y = lf * imu_data_.accel_y + inv_lf * (ay_raw - imu_accel_offset_[1]);
        imu_data_.accel_z = lf * imu_data_.accel_z + inv_lf * (az_raw - imu_accel_offset_[2]);

        imu_data_.gyro_x = lf * imu_data_.gyro_x + inv_lf * (gx_raw - imu_gyro_offset_[0]);
        imu_data_.gyro_y = lf * imu_data_.gyro_y + inv_lf * (gy_raw - imu_gyro_offset_[1]);
        imu_data_.gyro_z = lf * imu_data_.gyro_z + inv_lf * (gz_raw - imu_gyro_offset_[2]);
    } else {
        imu_data_.accel_x = ax_raw - imu_accel_offset_[0];
        imu_data_.accel_y = ay_raw - imu_accel_offset_[1];
        imu_data_.accel_z = az_raw - imu_accel_offset_[2];

        imu_data_.gyro_x = gx_raw - imu_gyro_offset_[0];
        imu_data_.gyro_y = gy_raw - imu_gyro_offset_[1];
        imu_data_.gyro_z = gz_raw - imu_gyro_offset_[2];
    }
    now_us = micros();
    
    if (now_us - last_baro_time_us_ < (1000000 / (SENSOR_FREQ_HZ / 2))) {
        return; // Menor frecuencia que IMU
    }
    
    float pressure_pa, temperature_c;
    if (!baro_.readPT(pressure_pa, temperature_c)) {
        return;
    }
    
    // Validación
    if (pressure_pa < BARO_PRESSURE_MIN || pressure_pa > BARO_PRESSURE_MAX) {
        return;
    }
    if (temperature_c < -40.0f || temperature_c > 85.0f) {
        // Keep previous temperature if current reading is invalid
        temperature_c = baro_data_.temperature_c;
    }

    const float baro_lf = 0.8f;
    const float baro_inv_lf = 1.0f - baro_lf;
    if (baro_data_.timestamp_us != 0) {
        baro_data_.pressure_pa = baro_lf * baro_data_.pressure_pa + baro_inv_lf * pressure_pa;
        baro_data_.temperature_c = baro_lf * baro_data_.temperature_c + baro_inv_lf * temperature_c;
    } else {
        baro_data_.pressure_pa = pressure_pa;
        baro_data_.temperature_c = temperature_c;
    }
    
    // Altitud barométrica (standard atmosphere)
    float ratio = pressure_pa / (QNH_STANDARD * 100.0f);
    float alt = 44330.0f * (1.0f - powf(ratio, 0.1903f));
    
    // Complementary filter (85% baro, 15% rate de cambio)
    if (!alt_init_) {
        alt_filtro_ = alt;
        alt_init_ = true;
    } else {
        float dt = (now_us - baro_data_.timestamp_us) / 1e6f;
        if (dt > 0 && dt < 1.0f) {
            float alt_rate_measured = (alt - alt_filtro_) / dt;
            alt_filtro_ += 0.85f * (alt - alt_filtro_) + 0.15f * alt_rate_measured * dt;
            alt_rate_ms_ = alt_rate_measured;
        }
    }
    
    baro_data_.altitude_m = alt_filtro_;
    baro_data_.timestamp_us = now_us;
    
    last_baro_time_us_ = now_us;
}

void SensorManager::leer_barometro() {
    // El flujo actual ya procesa barómetro en leer_imu(),
    // por lo que esta función existe solo para cumplir la interfaz.
}

void SensorManager::leer_compass() {
    compass_.read();
    
    // Normalizar a -1..+1
    float mx = compass_.getX() / 32767.0f;
    float my = compass_.getY() / 32767.0f;
    float mz = compass_.getZ() / 32767.0f;
    
    mag_data_.mag_x = mx;
    mag_data_.mag_y = my;
    mag_data_.mag_z = mz;
    mag_data_.timestamp_us = micros();
}

bool SensorManager::imu_data_valid(const IMURawData& data) {
    // Verificar aceleración dentro de rango esperado
    float accel_mag = sqrtf(data.accel_x * data.accel_x + 
                           data.accel_y * data.accel_y + 
                           data.accel_z * data.accel_z);
    
    if (accel_mag < (ACCEL_MIN_G * G_SI) || accel_mag > (ACCEL_MAX_G * G_SI)) {
        return false;
    }
    
    // Verificar giroscopio dentro de rango
    float gyro_mag = sqrtf(data.gyro_x * data.gyro_x + 
                          data.gyro_y * data.gyro_y + 
                          data.gyro_z * data.gyro_z);
    
    if (gyro_mag > (GYRO_MAX_DPS * DEG_TO_RAD)) {
        return false;
    }
    
    return true;
}

bool SensorManager::baro_data_valid(const BaroRawData& data) {
    if (data.pressure_pa < BARO_PRESSURE_MIN || data.pressure_pa > BARO_PRESSURE_MAX) {
        return false;
    }
    if (data.temperature_c < -40.0f || data.temperature_c > 85.0f) {
        return false;
    }
    return true;
}

bool SensorManager::update(uint64_t now_us) {
    leer_imu();
    leer_barometro();
    leer_compass();
    
    // Alimentar al estimador con datos crudo calibrados
    alimentar_estimador(now_us);
    
    // El estimador ejecuta su update (fusión)
    if (estimator_) {
        estimator_->update(now_us);
    }
    
    actualizar_estado_estimado();
    
    contador_lecturas_++;
    return true;
}

void SensorManager::alimentar_estimador(uint64_t now_us) {
    if (!estimator_) return;
    
    // IMU: Validar y alimentar
    if (imu_data_valid(imu_data_)) {
        estimator_->feed_imu(
            imu_data_.accel_x, imu_data_.accel_y, imu_data_.accel_z,
            imu_data_.gyro_x, imu_data_.gyro_y, imu_data_.gyro_z,
            imu_data_.timestamp_us
        );
    }
    
    // BARO: Validar y alimentar
    if (baro_data_valid(baro_data_)) {
        estimator_->feed_baro(
            baro_data_.pressure_pa, baro_data_.temperature_c,
            baro_data_.timestamp_us
        );
    }
    
    // MAGNETÓMETRO: Alimentar
    if (mag_data_.mag_x != 0 || mag_data_.mag_y != 0 || mag_data_.mag_z != 0) {
        estimator_->feed_mag(
            mag_data_.mag_x, mag_data_.mag_y, mag_data_.mag_z,
            mag_data_.timestamp_us
        );
    }
    
    // GPS y PITOT: TODO (si están implementados)
}

void SensorManager::actualizar_estado_estimado() {
    // El estado es ahora delegado al estimador
    // Esta función puede ser usada para logging adicional si es necesario
}

StateEstimate SensorManager::state() const {
    if (estimator_) {
        return estimator_->state();
    }
    // Fallback
    StateEstimate fallback;
    fallback.accel_x_mps2 = imu_data_.accel_x;
    fallback.accel_y_mps2 = imu_data_.accel_y;
    fallback.accel_z_mps2 = imu_data_.accel_z;
    fallback.rate_roll_rads = imu_data_.gyro_x;
    fallback.rate_pitch_rads = imu_data_.gyro_y;
    fallback.rate_yaw_rads = imu_data_.gyro_z;
    fallback.altitude_m = baro_data_.altitude_m;
    return fallback;
}

bool SensorManager::is_healthy() const {
    if (!estimator_) return false;
    return estimator_->is_healthy();
}

void SensorManager::reset() {
    memset(&imu_data_, 0, sizeof(imu_data_));
    memset(&baro_data_, 0, sizeof(baro_data_));
    memset(&mag_data_, 0, sizeof(mag_data_));
    if (estimator_) {
        estimator_->reset();
    }
    alt_filtro_ = 0.0f;
    alt_init_ = false;
}

void SensorManager::diagnostico_serial() {
    Serial.print("IMU[m/s²]: ");
    Serial.print(imu_data_.accel_x); Serial.print(", ");
    Serial.print(imu_data_.accel_y); Serial.print(", ");
    Serial.print(imu_data_.accel_z);
    
    Serial.print(" | Gyro[rad/s]: ");
    Serial.print(imu_data_.gyro_x); Serial.print(", ");
    Serial.print(imu_data_.gyro_y); Serial.print(", ");
    Serial.print(imu_data_.gyro_z);
    
    Serial.print(" | Baro: ");
    Serial.print(baro_data_.pressure_pa); Serial.print("Pa, ");
    Serial.print(baro_data_.altitude_m); Serial.print("m, ");
    Serial.print(baro_data_.temperature_c); Serial.println("C");
}

void SensorManager::debug_serial() {
    const StateEstimate estimated = state();
    const float rad2deg = 180.0f / 3.14159265f;

    // ===== IMU =====
    Serial.print("[IMU] a: ");
    Serial.print(imu_data_.accel_x, 3); Serial.print(" ");
    Serial.print(imu_data_.accel_y, 3); Serial.print(" ");
    Serial.print(imu_data_.accel_z, 3);

    Serial.print(" | g: ");
    Serial.print(imu_data_.gyro_x, 4); Serial.print(" ");
    Serial.print(imu_data_.gyro_y, 4); Serial.print(" ");
    Serial.print(imu_data_.gyro_z, 4);
    Serial.println();

    // ===== BARO =====
    Serial.print("[BARO] P: ");
    Serial.print(baro_data_.pressure_pa);
    Serial.print(" | Alt: ");
    Serial.print(baro_data_.altitude_m, 2);
    Serial.println(" m");

    // ===== ATT =====
    Serial.print("[ATT] RPY: ");
    Serial.print(estimated.euler_roll_rad * rad2deg, 2); Serial.print(" ");
    Serial.print(estimated.euler_pitch_rad * rad2deg, 2); Serial.print(" ");
    Serial.print(estimated.euler_yaw_rad * rad2deg, 2);
    Serial.println();
}

SensorData SensorManager::get_raw_data() const {
    SensorData data;
    
    // IMU data
    data.imu_acx = imu_data_.accel_x;
    data.imu_acy = imu_data_.accel_y;
    data.imu_acz = imu_data_.accel_z;
    data.imu_gx = imu_data_.gyro_x;
    data.imu_gy = imu_data_.gyro_y;
    data.imu_gz = imu_data_.gyro_z;
    
    // Magnetometer data
    data.mag_x = mag_data_.mag_x;
    data.mag_y = mag_data_.mag_y;
    data.mag_z = mag_data_.mag_z;
    data.mag_valid = true;  // TODO: Add proper validation
    
    // Barometer data
    data.baro_alt_m = baro_data_.altitude_m;
    data.baro_temp_c = baro_data_.temperature_c;
    
    // GPS (placeholder)
    data.gps_valid = false;
    
    // Timestamp
    data.timestamp_us = imu_data_.timestamp_us;
    
    return data;
}

void SensorManager::diagnostico_i2c() {
    Serial.println("\n=== DIAGNOSTICO I2C ===");

    // Buses I2C disponibles
    TwoWire* buses[] = {
        new TwoWire(PB7, PB6),  // I2C1
        new TwoWire(PB11, PB10), // I2C2
        new TwoWire(PC9, PA8)   // I2C3
    };

    const char* nombres[] = {
        "I2C1 (PB7 SDA / PB6 SCL)",
        "I2C2 (PB11 SDA / PB10 SCL)",
        "I2C3 (PC9 SDA / PA8 SCL)"
    };

    uint8_t direcciones[] = {0x76, 0x77}; // DPS310 posibles

    for (int i = 0; i < 3; i++) {
        TwoWire* bus = buses[i];
        Serial.print("Buscando en ");
        Serial.print(nombres[i]);
        Serial.println("...");

        bus->begin();
        bus->setClock(400000);

        bool encontrado = false;
        for (uint8_t addr : direcciones) {
            bus->beginTransmission(addr);
            if (bus->endTransmission() == 0) {
                Serial.print("  -> Sensor encontrado en 0x");
                Serial.println(addr, HEX);
                encontrado = true;

                // Intentar leer registro ID del DPS310 (0x0D)
                bus->beginTransmission(addr);
                bus->write(0x0D);
                bus->endTransmission(false);
                bus->requestFrom(addr, (uint8_t)1);
                if (bus->available()) {
                    uint8_t id = bus->read();
                    Serial.print("     ID: 0x");
                    Serial.println(id, HEX);
                    if (id == 0x10) {
                        Serial.println("     -> DPS310 confirmado!");
                    } else {
                        Serial.println("     -> ID desconocido");
                    }
                }
            }
        }
        if (!encontrado) {
            Serial.println("  -> No encontrado");
        }
        Serial.println();
        delete bus;
    }

    Serial.println("=== FIN DIAGNOSTICO I2C ===");
}

void SensorManager::diagnostico_spi() {
    Serial.println("\n=== DIAGNOSTICO SPI IMU ===");

    SPI.begin();
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    // Intentar leer WHO_AM_I del ICM42688 (registro 0x75)
    digitalWrite(IMU_CS, LOW);
    SPI.transfer(0x75 | 0x80); // Read bit
    uint8_t who_am_i = SPI.transfer(0x00);
    digitalWrite(IMU_CS, HIGH);

    Serial.print("WHO_AM_I: 0x");
    Serial.println(who_am_i, HEX);

    if (who_am_i == 0x47) {
        Serial.println("-> ICM42688 detectado correctamente!");
    } else {
        Serial.println("-> ICM42688 NO detectado o error");
    }

    Serial.println("=== FIN DIAGNOSTICO SPI ===");
}

} // namespace stallion

