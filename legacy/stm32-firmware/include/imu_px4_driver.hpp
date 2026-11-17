/**
 * @file imu_px4_driver.hpp
 * @brief Adaptación del driver ICM42688 con conceptos de PX4
 * 
 * Este driver es una adaptación de la arquitectura de PX4,
 * pero simplificado para STM32F405 sin NuttX.
 * 
 * Basado en:
 * - PX4 src/drivers/imu/icm42688/ICM42688.cpp
 * - ICM42688 datasheet
 * 
 * @author Adaptado de PX4 Autopilot
 * @version 1.0
 */

#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <cmath>
#include <cstring>

// ============================================================================
// Constantes y Configuración (del datasheet)
// ============================================================================

constexpr uint8_t ICM42688_I2C_ADDR = 0x68;

// Escalas del sensor
constexpr float ICM42688_ACCEL_RANGE_2G = 2.0f;
constexpr float ICM42688_ACCEL_RANGE_4G = 4.0f;
constexpr float ICM42688_ACCEL_RANGE_8G = 8.0f;
constexpr float ICM42688_ACCEL_RANGE_16G = 16.0f;

constexpr float ICM42688_GYRO_RANGE_250DPS = 250.0f;
constexpr float ICM42688_GYRO_RANGE_500DPS = 500.0f;
constexpr float ICM42688_GYRO_RANGE_1000DPS = 1000.0f;
constexpr float ICM42688_GYRO_RANGE_2000DPS = 2000.0f;

// ============================================================================
// Estructura de Datos (estilo PX4)
// ============================================================================

/**
 * Datos crudos del sensor ICM42688
 */
struct ICM42688RawData {
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    int16_t temperature;
    uint32_t timestamp_us;
};

/**
 * Datos calibrados y convertidos a SI (m/s² y rad/s)
 */
struct ICM42688Data {
    float accel_x_mps2;
    float accel_y_mps2;
    float accel_z_mps2;
    float gyro_x_rads;
    float gyro_y_rads;
    float gyro_z_rads;
    float temperature_c;
    uint32_t timestamp_us;
};

// ============================================================================
// Clase Principal (Driver PX4-Style)
// ============================================================================

/**
 * @class IMU_PX4Driver
 * @brief Driver del IMU ICM42688 adaptado de PX4
 * 
 * Características:
 * - Inicialización configurable
 * - Lectura no-bloqueante
 * - Calibración en software
 * - Conversión automática a SI
 * - Detección de errores
 */
class IMU_PX4Driver {
public:
    /**
     * Configuración del driver
     */
    struct Config {
        uint32_t accel_sample_rate_hz;      ///< 100-1000 Hz
        uint32_t gyro_sample_rate_hz;       ///< 100-2000 Hz
        float accel_range_g;                ///< 2, 4, 8, 16 g
        float gyro_range_dps;               ///< 250, 500, 1000, 2000 dps
        uint8_t i2c_address;                ///< Dirección I2C
        bool enable_fifo;                   ///< Usar FIFO interno
        bool calibrate_on_init;             ///< Calibración automática
    };
    
    // ========================================================================
    // Métodos Públicos
    // ========================================================================
    
    IMU_PX4Driver();
    ~IMU_PX4Driver();
    
    /**
     * Inicializar el driver
     * @param config Configuración
     * @return true si inicialización exitosa
     */
    bool init(const Config& config);
    
    /**
     * Deinicializar (cleanup)
     */
    void deinit();
    
    /**
     * Verificar si hay datos disponibles
     * @return true si hay datos listos para leer
     */
    bool data_ready();
    
    /**
     * Leer datos crudos
     * @param data Estructura para almacenar datos
     * @return true si lectura exitosa
     */
    bool read_raw(ICM42688RawData& data);
    
    /**
     * Leer datos calibrados en SI
     * @param data Estructura para almacenar datos
     * @return true si lectura exitosa
     */
    bool read(ICM42688Data& data);
    
    /**
     * Obtener último dato leído
     * @return Dato calibrado
     */
    const ICM42688Data& last_data() const { return _last_data; }
    
    /**
     * Realizar calibración
     * Requiere que el sensor esté estacionario
     * @param samples Número de muestras para promediar
     * @return true si calibración exitosa
     */
    bool calibrate(uint16_t samples = 100);
    
    /**
     * Resetear a estado inicial
     */
    void reset();
    
    /**
     * Obtener información de diagnóstico
     */
    void print_diagnostics() const;
    
    // ========================================================================
    // Getters (para debugging)
    // ========================================================================
    
    bool is_initialized() const { return _initialized; }
    uint32_t read_count() const { return _read_count; }
    uint32_t error_count() const { return _error_count; }
    
private:
    // ========================================================================
    // Miembros Privados
    // ========================================================================
    
    TwoWire* _wire;
    Config _config;
    ICM42688RawData _last_raw;
    ICM42688Data _last_data;
    
    // Calibración
    float _accel_offset[3];
    float _gyro_offset[3];
    float _accel_scale[3];
    float _gyro_scale[3];
    
    // Conversión
    float _accel_lsb;  // LSB por unidad de aceleración
    float _gyro_lsb;   // LSB por unidad de rotación
    
    // Estado
    bool _initialized;
    uint32_t _read_count;
    uint32_t _error_count;
    
    // ========================================================================
    // Métodos Privados
    // ========================================================================
    
    /**
     * Escribir a registro
     */
    bool _write_register(uint8_t reg, uint8_t value);
    
    /**
     * Leer de registro
     */
    uint8_t _read_register(uint8_t reg);
    
    /**
     * Leer múltiples registros
     */
    bool _read_registers(uint8_t reg, uint8_t* data, uint8_t len);
    
    /**
     * Configurar rango de acelerómetro
     */
    bool _set_accel_range();
    
    /**
     * Configurar rango de giroscopio
     */
    bool _set_gyro_range();
    
    /**
     * Configurar velocidad de muestreo
     */
    bool _set_sample_rate();
    
    /**
     * Convertir dato raw a SI
     */
    void _convert_raw_to_si(const ICM42688RawData& raw, ICM42688Data& data);
    
    /**
     * Verificar chip ID
     */
    bool _verify_chip_id();
    
    /**
     * Realizar auto-test
     */
    bool _self_test();
};

// ============================================================================
// Registro del Driver (estilo PX4)
// ============================================================================

/**
 * Instancia global singleton (opcional)
 */
extern IMU_PX4Driver* g_imu;

/**
 * Macro para acceder fácilmente
 */
#define IMU_DRIVER (*g_imu)

