/**
 * @file imu_px4_driver.cpp
 * @brief Implementación del driver ICM42688 con conceptos de PX4
 */

#include "imu_px4_driver.hpp"

// Registros del ICM42688 (del datasheet)
constexpr uint8_t ICM42688_REG_CHIP_ID = 0x75;
constexpr uint8_t ICM42688_CHIP_ID = 0x47;

constexpr uint8_t ICM42688_REG_ACCEL_CONFIG = 0x14;
constexpr uint8_t ICM42688_REG_GYRO_CONFIG = 0x15;
constexpr uint8_t ICM42688_REG_ACCEL_XOUT_H = 0x0F;
constexpr uint8_t ICM42688_REG_GYRO_XOUT_H = 0x19;
constexpr uint8_t ICM42688_REG_TEMP_OUT_H = 0x1D;
constexpr uint8_t ICM42688_REG_INT_STATUS = 0x2D;

// Variables globales
IMU_PX4Driver* g_imu = nullptr;

// ============================================================================
// Implementación
// ============================================================================

IMU_PX4Driver::IMU_PX4Driver()
    : _wire(&Wire),
      _initialized(false),
      _read_count(0),
      _error_count(0) {
    
    memset(_accel_offset, 0, sizeof(_accel_offset));
    memset(_gyro_offset, 0, sizeof(_gyro_offset));
    memset(_accel_scale, 0, sizeof(_accel_scale));
    memset(_gyro_scale, 0, sizeof(_gyro_scale));
    
    memset(&_last_raw, 0, sizeof(_last_raw));
    memset(&_last_data, 0, sizeof(_last_data));
}

IMU_PX4Driver::~IMU_PX4Driver() {
    deinit();
}

bool IMU_PX4Driver::init(const Config& config) {
    _config = config;
    
    // Verificar inicialización de Wire
    if (!_wire) {
        Serial.println("[IMU] Error: Wire not initialized");
        return false;
    }
    
    // Verificar chip ID
    if (!_verify_chip_id()) {
        Serial.println("[IMU] Error: Invalid chip ID");
        return false;
    }
    
    // Configurar rango de acelerómetro
    if (!_set_accel_range()) {
        Serial.println("[IMU] Error: Failed to set accel range");
        return false;
    }
    
    // Configurar rango de giroscopio
    if (!_set_gyro_range()) {
        Serial.println("[IMU] Error: Failed to set gyro range");
        return false;
    }
    
    // Configurar velocidad de muestreo
    if (!_set_sample_rate()) {
        Serial.println("[IMU] Error: Failed to set sample rate");
        return false;
    }
    
    // Calibración inicial (si está habilitada)
    if (_config.calibrate_on_init) {
        Serial.println("[IMU] Calibrating sensors...");
        if (!calibrate(100)) {
            Serial.println("[IMU] Warning: Calibration failed");
            // Continuar de todas formas con calibración por defecto
        }
    }
    
    _initialized = true;
    Serial.println("[IMU] Initialization successful");
    
    return true;
}

void IMU_PX4Driver::deinit() {
    _initialized = false;
}

bool IMU_PX4Driver::data_ready() {
    if (!_initialized) return false;
    
    // Leer registro de estado de interrupción
    uint8_t int_status = _read_register(ICM42688_REG_INT_STATUS);
    
    // Bit 0 = data ready
    return (int_status & 0x01) != 0;
}

bool IMU_PX4Driver::read_raw(ICM42688RawData& data) {
    if (!_initialized) return false;
    
    // Leer 12 bytes: 6 bytes accel, 6 bytes gyro
    uint8_t buffer[12];
    if (!_read_registers(ICM42688_REG_ACCEL_XOUT_H, buffer, 12)) {
        _error_count++;
        return false;
    }
    
    // Parsear datos (big-endian)
    data.accel_x = (int16_t)(((int16_t)buffer[0] << 8) | buffer[1]);
    data.accel_y = (int16_t)(((int16_t)buffer[2] << 8) | buffer[3]);
    data.accel_z = (int16_t)(((int16_t)buffer[4] << 8) | buffer[5]);
    
    data.gyro_x = (int16_t)(((int16_t)buffer[6] << 8) | buffer[7]);
    data.gyro_y = (int16_t)(((int16_t)buffer[8] << 8) | buffer[9]);
    data.gyro_z = (int16_t)(((int16_t)buffer[10] << 8) | buffer[11]);
    
    data.timestamp_us = micros();
    
    _last_raw = data;
    _read_count++;
    
    return true;
}

bool IMU_PX4Driver::read(ICM42688Data& data) {
    ICM42688RawData raw;
    if (!read_raw(raw)) {
        return false;
    }
    
    _convert_raw_to_si(raw, data);
    _last_data = data;
    
    return true;
}

bool IMU_PX4Driver::calibrate(uint16_t samples) {
    if (!_initialized) return false;
    
    float accel_sum[3] = {0, 0, 0};
    float gyro_sum[3] = {0, 0, 0};
    
    Serial.println("[IMU] Calibrating... Please keep sensor stationary");
    
    for (uint16_t i = 0; i < samples; i++) {
        ICM42688RawData raw;
        if (!read_raw(raw)) {
            return false;
        }
        
        accel_sum[0] += raw.accel_x;
        accel_sum[1] += raw.accel_y;
        accel_sum[2] += raw.accel_z;
        
        gyro_sum[0] += raw.gyro_x;
        gyro_sum[1] += raw.gyro_y;
        gyro_sum[2] += raw.gyro_z;
        
        delay(10);
    }
    
    // Promediar
    for (int i = 0; i < 3; i++) {
        _accel_offset[i] = accel_sum[i] / samples;
        _gyro_offset[i] = gyro_sum[i] / samples;
    }
    
    // Ajuste: Z accel debe ser cercano a 1g (16384 LSB a 16G)
    _accel_offset[2] -= (32768.0f / 2.0f); // Restar ~1g
    
    Serial.print("[IMU] Calibration complete. Offsets: ");
    Serial.print(_accel_offset[0]); Serial.print(", ");
    Serial.print(_accel_offset[1]); Serial.print(", ");
    Serial.print(_accel_offset[2]); Serial.print(" | ");
    Serial.print(_gyro_offset[0]); Serial.print(", ");
    Serial.print(_gyro_offset[1]); Serial.print(", ");
    Serial.println(_gyro_offset[2]);
    
    return true;
}

void IMU_PX4Driver::reset() {
    _read_count = 0;
    _error_count = 0;
    memset(&_last_raw, 0, sizeof(_last_raw));
    memset(&_last_data, 0, sizeof(_last_data));
}

void IMU_PX4Driver::print_diagnostics() const {
    Serial.println("=== IMU Diagnostics ===");
    Serial.print("Initialized: ");
    Serial.println(_initialized ? "YES" : "NO");
    Serial.print("Read count: ");
    Serial.println(_read_count);
    Serial.print("Error count: ");
    Serial.println(_error_count);
    Serial.print("Last Accel (m/s²): ");
    Serial.print(_last_data.accel_x_mps2); Serial.print(", ");
    Serial.print(_last_data.accel_y_mps2); Serial.print(", ");
    Serial.println(_last_data.accel_z_mps2);
    Serial.print("Last Gyro (rad/s): ");
    Serial.print(_last_data.gyro_x_rads); Serial.print(", ");
    Serial.print(_last_data.gyro_y_rads); Serial.print(", ");
    Serial.println(_last_data.gyro_z_rads);
}

// ============================================================================
// Métodos Privados
// ============================================================================

bool IMU_PX4Driver::_write_register(uint8_t reg, uint8_t value) {
    _wire->beginTransmission(_config.i2c_address);
    _wire->write(reg);
    _wire->write(value);
    return _wire->endTransmission() == 0;
}

uint8_t IMU_PX4Driver::_read_register(uint8_t reg) {
    _wire->beginTransmission(_config.i2c_address);
    _wire->write(reg);
    _wire->endTransmission();
    
    _wire->requestFrom(_config.i2c_address, (uint8_t)1);
    if (_wire->available()) {
        return _wire->read();
    }
    
    return 0;
}

bool IMU_PX4Driver::_read_registers(uint8_t reg, uint8_t* data, uint8_t len) {
    _wire->beginTransmission(_config.i2c_address);
    _wire->write(reg);
    if (_wire->endTransmission() != 0) return false;
    
    _wire->requestFrom(_config.i2c_address, len);
    
    for (uint8_t i = 0; i < len; i++) {
        if (_wire->available()) {
            data[i] = _wire->read();
        } else {
            return false;
        }
    }
    
    return true;
}

bool IMU_PX4Driver::_set_accel_range() {
    uint8_t fs_sel = 0;
    float scale = 1.0f;
    
    switch ((int)_config.accel_range_g) {
        case 2:
            fs_sel = 0x00;
            scale = 16384.0f;  // LSB/g
            break;
        case 4:
            fs_sel = 0x01;
            scale = 8192.0f;
            break;
        case 8:
            fs_sel = 0x02;
            scale = 4096.0f;
            break;
        case 16:
            fs_sel = 0x03;
            scale = 2048.0f;
            break;
        default:
            return false;
    }
    
    _accel_lsb = 9.81f / scale;  // Conversión a m/s²
    
    return _write_register(ICM42688_REG_ACCEL_CONFIG, fs_sel);
}

bool IMU_PX4Driver::_set_gyro_range() {
    uint8_t fs_sel = 0;
    float scale = 1.0f;
    
    switch ((int)_config.gyro_range_dps) {
        case 250:
            fs_sel = 0x00;
            scale = 131.0f;  // LSB/(deg/s)
            break;
        case 500:
            fs_sel = 0x01;
            scale = 65.5f;
            break;
        case 1000:
            fs_sel = 0x02;
            scale = 32.8f;
            break;
        case 2000:
            fs_sel = 0x03;
            scale = 16.4f;
            break;
        default:
            return false;
    }
    
    _gyro_lsb = (M_PI / 180.0f) / scale;  // Conversión a rad/s
    
    return _write_register(ICM42688_REG_GYRO_CONFIG, fs_sel);
}

bool IMU_PX4Driver::_set_sample_rate() {
    // Configuración simplificada (requiere acceso a registros de frecuencia)
    // Por ahora, asumimos que los defaults funcionan
    return true;
}

void IMU_PX4Driver::_convert_raw_to_si(const ICM42688RawData& raw, ICM42688Data& data) {
    // Aplicar offset de calibración y convertir a SI
    data.accel_x_mps2 = (raw.accel_x - _accel_offset[0]) * _accel_lsb;
    data.accel_y_mps2 = (raw.accel_y - _accel_offset[1]) * _accel_lsb;
    data.accel_z_mps2 = (raw.accel_z - _accel_offset[2]) * _accel_lsb;
    
    data.gyro_x_rads = (raw.gyro_x - _gyro_offset[0]) * _gyro_lsb;
    data.gyro_y_rads = (raw.gyro_y - _gyro_offset[1]) * _gyro_lsb;
    data.gyro_z_rads = (raw.gyro_z - _gyro_offset[2]) * _gyro_lsb;
    
    // Temperatura: (valor - 0) / 333.87 + 21 (del datasheet)
    data.temperature_c = raw.temperature / 333.87f + 21.0f;
    
    data.timestamp_us = raw.timestamp_us;
}

bool IMU_PX4Driver::_verify_chip_id() {
    uint8_t chip_id = _read_register(ICM42688_REG_CHIP_ID);
    if (chip_id != ICM42688_CHIP_ID) {
        Serial.print("[IMU] Invalid chip ID: 0x");
        Serial.println(chip_id, HEX);
        return false;
    }
    return true;
}

bool IMU_PX4Driver::_self_test() {
    // Implementación del auto-test del datasheet (opcional)
    // Por ahora, retornamos true
    return true;
}
