# Guía Completa: Integración de PX4 en Proyecto PlatformIO/STM32F405

## Tabla de Contenidos
1. [Análisis de Viabilidad](#análisis-de-viabilidad)
2. [Arquitectura de PX4](#arquitectura-de-px4)
3. [Alternativas de Integración](#alternativas-de-integración)
4. [Guía Paso a Paso](#guía-paso-a-paso)
5. [Ejemplos de Código](#ejemplos-de-código)
6. [Limitaciones y Constraints](#limitaciones-y-constraints)
7. [Arquitectura Híbrida Recomendada](#arquitectura-híbrida-recomendada)
8. [Alternativas Más Ligeras](#alternativas-más-ligeras)

---

## Análisis de Viabilidad

### Tu Plataforma Actual
```
MCU:              STM32F405RG
Flash:            1 MB
RAM:              256 KB
Framework:        Arduino + PlatformIO
Sensores:         IMU (ICM42688), Barómetro (DPS310), Compass (QMC5883L), GPS (UART)
Arquitectura:     Modular (SensorManager, Scheduler)
Tamaño esperado:  ~50-100 KB de código + datos
```

### ¿Puedo usar PX4 completo?
**NO.** PX4 completo requiere:
- **NuttX RTOS** (real-time OS)
- **~2-3 MB Flash** mínimo para firmware compilado
- **512 KB - 1 MB RAM** para heap/stack del RTOS
- **Arquitectura modular complicada** (drivers, middleware, apps)

**Veredicto**: PX4 completo es **demasiado pesado** para STM32F405.

### ¿Qué puedo integrar?
✅ **VIABLE**: 
- Drivers de sensores (simplificados)
- EKF ligero (Extended Kalman Filter)
- Algoritmos de control de actitud
- Controlador PID simple
- MAVLink protocol (para comunicación)

❌ **NO VIABLE**:
- NuttX RTOS completo
- uORB middleware
- Sistema de logging completo
- Interfaz gráfica

---

## Arquitectura de PX4

### Estructura Conceptual
```
┌─────────────────────────────────────────────┐
│           PX4 Firmware Stack                │
├─────────────────────────────────────────────┤
│ Aplicaciones (Commander, Navigator, etc.)   │
├─────────────────────────────────────────────┤
│ Middleware                                   │
│  ├── uORB (inter-process communication)     │
│  ├── Parameters                             │
│  └── Logging                                │
├─────────────────────────────────────────────┤
│ Módulos (Control, Estimation, etc.)        │
├─────────────────────────────────────────────┤
│ Drivers (Sensores, Comunicación)           │
├─────────────────────────────────────────────┤
│ NuttX RTOS + HAL                           │
├─────────────────────────────────────────────┤
│ Hardware (STM32, MCU)                      │
└─────────────────────────────────────────────┘
```

### Componentes Interesantes de PX4
1. **Drivers**
   - MPU6000/ICM42688 (IMU)
   - DPS310 (Barómetro)
   - QMC5883L (Magnetómetro)
   - GPS (UART)

2. **Módulos de Estimación**
   - EKF2 (Extended Kalman Filter)
   - Attitude Estimator (cuaterniones)
   - Position Estimator

3. **Controladores**
   - MC_Pos_Control (multicopter position control)
   - MC_Att_Control (attitude control)
   - PID Controllers

4. **Protokolos de Comunicación**
   - MAVLink (telemetría y comandos)

---

## Alternativas de Integración

### Opción 1: Integración Directa de Módulos (⭐ Recomendada)
**Descripción**: Adaptar código específico de PX4 (drivers, algoritmos) a tu arquitectura modular.

**Ventajas**:
- Máximo control
- Mejor performance
- Adaptado a tu hardware
- Compatible con Arduino/PlatformIO

**Desventajas**:
- Trabajo manual de adaptación
- Pruebas exhaustivas necesarias
- No es trivial

**Memoria**: ~80-120 KB (manageable)

**Recomendación**: ✅ **ESTA ES LA MEJOR OPCIÓN**

---

### Opción 2: Comunicación MAVLink con PX4
**Descripción**: Tu STM32F405 corre código ligero y comunica con una Pixhawk/PX4 vía MAVLink.

```
┌─────────────────┐           MAVLink          ┌─────────────┐
│ STM32F405       │◄────────────────────────►│ Pixhawk     │
│ (Tu código)     │     (Serial/Ethernet)     │ (PX4 Full)  │
└─────────────────┘                           └─────────────┘
```

**Ventajas**:
- Fácil de implementar
- Pruebas con PX4 oficial
- Escalable

**Desventajas**:
- Latencia adicional
- Complejidad en tiempos reales críticos
- Requiere Pixhawk adicional

**Memoria**: ~30-40 KB

**Recomendación**: ⭐ Si tienes presupuesto para Pixhawk adicional

---

### Opción 3: Usar PX4 Ligero (PX4-Nano)
**Descripción**: Versión simplificada de PX4 para boards pequeños.

**Ventajas**:
- Menos custom development
- Compatible con ecosistema PX4

**Desventajas**:
- Requiere montaje especial
- Documentación limitada
- Menos mantenimiento

**Recomendación**: ⭐⭐ Si encuentras compilación lista

---

## Guía Paso a Paso: Opción 1 (Integración Directa)

### Paso 1: Descargar y Analizar Código de PX4

```bash
# Clonar repositorio de PX4
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot

# Navegar a los drivers relevantes
# src/drivers/imu/icm42688/   (Tu IMU)
# src/drivers/barometer/dps310/ (Tu barómetro)
# src/drivers/magnetometer/qmc5883l/ (Tu compass)
# src/modules/ekf2/            (Filtro de Kalman)
```

### Paso 2: Identificar Funciones Reutilizables
Para cada driver de PX4:
1. **Extraer lógica de inicialización**
2. **Extraer bucle de lectura**
3. **Eliminar dependencias de NuttX** (RTOS calls)
4. **Adaptar tipos de datos**

### Paso 3: Adaptar a Tu Estructura

Ejemplo con el driver ICM42688:

```cpp
// PX4 original (src/drivers/imu/icm42688/ICM42688.cpp)
// Tiene: init(), read(), configuration()
// Dependencias: NuttX, uORB, etc.

// Tu adaptación (include/imu_driver.hpp)
// Tendrá: init(), read(), similar interfaz
// Sin dependencias de NuttX
```

### Paso 4: Integrar en SensorManager

```cpp
// Tu archivo modificado: src/sensor_manager.cpp
#include "imu_driver_px4_adapted.hpp"

class SensorManager : public IEstimator {
    ICM42688_PX4Adapted imu_px4_;
    
    bool inicializar_imu() {
        return imu_px4_.init();  // Usando driver adaptado
    }
    
    void leer_imu() {
        imu_px4_.read(datos_raw_.imu_acx, datos_raw_.imu_acy, datos_raw_.imu_acz);
    }
};
```

### Paso 5: Implementar EKF Ligero

Crear un módulo de estimación simplificado basado en PX4:

```cpp
// include/ekf_lightweight.hpp
class EKFLightweight {
public:
    void update(const SensorRawData& sensor_data, float dt);
    StateEstimate getState() const;
    
private:
    // Matrices de estado (3x3 o 4x4 en lugar de 24x24 de PX4)
    float q[4];  // Quaternion (actitud)
    float p[3];  // Position
    float v[3];  // Velocity
};
```

### Paso 6: Control de Actitud

Adaptar controlador de actitud de PX4:

```cpp
// include/attitude_controller.hpp
class AttitudeController {
public:
    ControlDemand update(
        const StateEstimate& state,
        const PilotInput& input,
        float dt
    );
    
private:
    float pid_p[3];  // PID para roll, pitch, yaw
    float pid_i[3];
    float pid_d[3];
};
```

---

## Ejemplos de Código

### Ejemplo 1: Adapter para IMU (ICM42688)

Archivo: `include/imu_driver_px4_adapted.hpp`

```cpp
#pragma once
#include <cstdint>
#include <ICM42688.h>

/**
 * Adapter para compatibilidad con conceptos de PX4
 * Simplificado para STM32F405
 */
class ICM42688_PX4Adapted {
public:
    struct Config {
        uint32_t gyro_rate;      // Hz
        uint32_t accel_rate;     // Hz
        float gyro_range;        // deg/s
        float accel_range;       // g
    };
    
    ICM42688_PX4Adapted();
    
    bool init(const Config& cfg);
    
    /**
     * Lee sensores (estilo PX4)
     */
    bool read(
        float& accel_x, float& accel_y, float& accel_z,  // m/s²
        float& gyro_x,  float& gyro_y,  float& gyro_z,   // rad/s
        float& temperature                                 // °C
    );
    
    /**
     * Acceso raw para debugging
     */
    void getRaw(int16_t& ax, int16_t& ay, int16_t& az,
                int16_t& gx, int16_t& gy, int16_t& gz);
    
private:
    ICM42688 _hw_imu;
    Config _config;
    
    // Calibración en software
    float _accel_offset[3];
    float _gyro_offset[3];
    
    void _apply_calibration();
    void _convert_to_si_units();
};
```

Archivo: `src/drivers/imu_driver_px4_adapted.cpp`

```cpp
#include "imu_driver_px4_adapted.hpp"
#include <Wire.h>

ICM42688_PX4Adapted::ICM42688_PX4Adapted()
    : _hw_imu() {
    memset(_accel_offset, 0, sizeof(_accel_offset));
    memset(_gyro_offset, 0, sizeof(_gyro_offset));
}

bool ICM42688_PX4Adapted::init(const Config& cfg) {
    _config = cfg;
    
    // Inicialización similar a PX4 pero simplificada
    if (!_hw_imu.begin(Wire, ICM42688_I2C_ADDR_DEFAULT)) {
        return false;
    }
    
    // Configurar escalas
    _hw_imu.setAccelRange(ICM42688_ACCEL_RANGE_16G);
    _hw_imu.setGyroRange(ICM42688_GYRO_RANGE_250DPS);
    
    // Calibración inicial (offset zero)
    _apply_calibration();
    
    return true;
}

bool ICM42688_PX4Adapted::read(
    float& accel_x, float& accel_y, float& accel_z,
    float& gyro_x,  float& gyro_y,  float& gyro_z,
    float& temperature) {
    
    // Leer datos raw del sensor
    if (!_hw_imu.accelAvailable() || !_hw_imu.gyroAvailable()) {
        return false;
    }
    
    _hw_imu.readAccel();
    _hw_imu.readGyro();
    
    // Conversión a unidades SI (m/s² y rad/s)
    // Calibración y escala dependen del rango configurado
    accel_x = _hw_imu.accelX() * 9.81f / 2048.0f;  // m/s²
    accel_y = _hw_imu.accelY() * 9.81f / 2048.0f;
    accel_z = _hw_imu.accelZ() * 9.81f / 2048.0f;
    
    gyro_x = _hw_imu.gyroX() * M_PI / 180.0f / 131.0f;  // rad/s
    gyro_y = _hw_imu.gyroY() * M_PI / 180.0f / 131.0f;
    gyro_z = _hw_imu.gyroZ() * M_PI / 180.0f / 131.0f;
    
    temperature = _hw_imu.getTemperatureC();
    
    return true;
}

void ICM42688_PX4Adapted::_apply_calibration() {
    // Placeholder: implementar calibración si es necesario
}
```

---

### Ejemplo 2: EKF Ligero (Basado en Conceptos de PX4)

Archivo: `include/ekf_lightweight.hpp`

```cpp
#pragma once
#include <cmath>
#include "module_data_types.hpp"

/**
 * Extended Kalman Filter Ligero
 * Inspirado en PX4 EKF2, pero simplificado para STM32F405
 * 
 * Estado: [qw, qx, qy, qz, vx, vy, vz, bax, bay, baz, bgx, bgy, bgz]
 * (13 estados, matriz P: 13x13)
 */
class EKFLightweight {
public:
    struct State {
        float q[4];      // Quaternion (actitud)
        float v[3];      // Velocidad NED
        float ba[3];     // Bias acelerómetro
        float bg[3];     // Bias giroscopio
    };
    
    EKFLightweight();
    
    /**
     * Predicción (usando datos del giroscopio)
     */
    void predict(float gx, float gy, float gz, float dt);
    
    /**
     * Actualización con acelerómetro
     */
    void update_accel(float ax, float ay, float az);
    
    /**
     * Actualización con magnetómetro
     */
    void update_mag(float mx, float my, float mz);
    
    /**
     * Obtener estado actual
     */
    State getState() const { return _state; }
    
    /**
     * Obtener ángulos de Euler (grados)
     */
    void getEuler(float& roll, float& pitch, float& yaw) const;
    
private:
    State _state;
    float _P[13][13];  // Matriz de covarianza (13x13)
    float _Q[13][13];  // Matriz de ruido de proceso
    float _R_accel;    // Ruido acelerómetro
    float _R_mag;      // Ruido magnetómetro
    
    // Métodos auxiliares
    void _quaternion_normalize();
    void _predict_covariance(float dt);
    void _update_covariance_accel();
    void _update_covariance_mag();
    void _quaternion_from_axis_angle(float axis[3], float angle, float q[4]);
};
```

Archivo: `src/ekf_lightweight.cpp`

```cpp
#include "ekf_lightweight.hpp"

EKFLightweight::EKFLightweight() {
    // Inicializar estado
    _state.q[0] = 1.0f; _state.q[1] = 0.0f; _state.q[2] = 0.0f; _state.q[3] = 0.0f;
    _state.v[0] = 0.0f; _state.v[1] = 0.0f; _state.v[2] = 0.0f;
    _state.ba[0] = 0.0f; _state.ba[1] = 0.0f; _state.ba[2] = 0.0f;
    _state.bg[0] = 0.0f; _state.bg[1] = 0.0f; _state.bg[2] = 0.0f;
    
    // Inicializar covarianza (confianza inicial)
    memset(_P, 0, sizeof(_P));
    for (int i = 0; i < 13; i++) _P[i][i] = 1.0f;
    
    // Ruido de medición
    _R_accel = 0.1f;
    _R_mag = 0.2f;
}

void EKFLightweight::predict(float gx, float gy, float gz, float dt) {
    // Aplicar bias
    gx -= _state.bg[0];
    gy -= _state.bg[1];
    gz -= _state.bg[2];
    
    // Integrar rotación (ecuación de cuaterniones)
    float norm = sqrt(gx*gx + gy*gy + gz*gz);
    if (norm < 1e-6) return;
    
    float half_angle = 0.5f * norm * dt;
    float sin_half = sin(half_angle);
    float cos_half = cos(half_angle);
    
    float dq[4] = {
        cos_half,
        sin_half * gx / norm,
        sin_half * gy / norm,
        sin_half * gz / norm
    };
    
    // Multiplicación de quaterniones: q_new = dq * q_old
    float q_new[4] = {
        dq[0]*_state.q[0] - dq[1]*_state.q[1] - dq[2]*_state.q[2] - dq[3]*_state.q[3],
        dq[0]*_state.q[1] + dq[1]*_state.q[0] + dq[2]*_state.q[3] - dq[3]*_state.q[2],
        dq[0]*_state.q[2] - dq[1]*_state.q[3] + dq[2]*_state.q[0] + dq[3]*_state.q[1],
        dq[0]*_state.q[3] + dq[1]*_state.q[2] - dq[2]*_state.q[1] + dq[3]*_state.q[0]
    };
    
    memcpy(_state.q, q_new, sizeof(q_new));
    _quaternion_normalize();
    
    // Predecir covarianza (simplificado)
    _predict_covariance(dt);
}

void EKFLightweight::update_accel(float ax, float ay, float az) {
    // Normalizar aceleración medida
    float norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6) return;
    ax /= norm; ay /= norm; az /= norm;
    
    // Gravedad esperada en marco corporal (usando quaternión actual)
    // En NED: gravedad es [0, 0, 9.81]
    // Rotar a frame del cuerpo usando q inverso
    
    // [Implementación completa del EKF update aquí]
    // Por brevedad, incluimos solo la estructura
    
    _update_covariance_accel();
}

void EKFLightweight::getEuler(float& roll, float& pitch, float& yaw) const {
    float q0 = _state.q[0], q1 = _state.q[1], q2 = _state.q[2], q3 = _state.q[3];
    
    roll = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2));
    pitch = asin(2*(q0*q2 - q3*q1));
    yaw = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3));
    
    // Convertir a grados
    roll *= 180.0f / M_PI;
    pitch *= 180.0f / M_PI;
    yaw *= 180.0f / M_PI;
}

void EKFLightweight::_quaternion_normalize() {
    float norm = sqrt(_state.q[0]*_state.q[0] + _state.q[1]*_state.q[1] +
                      _state.q[2]*_state.q[2] + _state.q[3]*_state.q[3]);
    if (norm > 0) {
        _state.q[0] /= norm;
        _state.q[1] /= norm;
        _state.q[2] /= norm;
        _state.q[3] /= norm;
    }
}

void EKFLightweight::_predict_covariance(float dt) {
    // Simplificado: ampliar covarianza lentamente (incertidumbre crece)
    for (int i = 0; i < 13; i++) {
        _P[i][i] += 0.001f * dt;
    }
}

void EKFLightweight::_update_covariance_accel() {
    // Reducir covarianza (confianza aumenta con medición)
    for (int i = 0; i < 4; i++) {
        _P[i][i] *= 0.99f;
    }
}
```

---

### Ejemplo 3: Controlador PID de Actitud (Estilo PX4)

Archivo: `include/attitude_controller.hpp`

```cpp
#pragma once
#include <cmath>
#include "module_data_types.hpp"

/**
 * Controlador de Actitud PID
 * Inspirado en MC_AttControl de PX4
 */
class AttitudeController {
public:
    struct Gains {
        float kp;  // Proporcional
        float ki;  // Integral
        float kd;  // Derivativo
    };
    
    AttitudeController();
    
    /**
     * Actualizar controlador
     * @param state: Estado actual de estimación
     * @param target_roll, target_pitch, target_yaw: Ángulos deseados (rad)
     * @param dt: Paso de tiempo
     */
    void update(
        const StateEstimate& state,
        float target_roll,  float target_pitch,  float target_yaw,
        float dt
    );
    
    /**
     * Obtener salidas de control (velocidades angulares deseadas)
     */
    void getOutput(float& p_cmd, float& q_cmd, float& r_cmd) const;
    
    /**
     * Configurar ganancias
     */
    void setGains(const Gains& roll_gains, const Gains& pitch_gains, const Gains& yaw_gains);
    
    /**
     * Reset integral (anti-windup)
     */
    void resetIntegrals();
    
private:
    Gains _gains[3];  // [Roll, Pitch, Yaw]
    float _error_integral[3];
    float _error_prev[3];
    float _output[3];
};
```

Archivo: `src/attitude_controller.cpp`

```cpp
#include "attitude_controller.hpp"

AttitudeController::AttitudeController() {
    // Ganancias por defecto (tuning necesario)
    _gains[0] = {4.5f, 0.05f, 0.12f};  // Roll
    _gains[1] = {4.5f, 0.05f, 0.12f};  // Pitch
    _gains[2] = {4.0f, 0.05f, 0.30f};  // Yaw
    
    resetIntegrals();
}

void AttitudeController::update(
    const StateEstimate& state,
    float target_roll,  float target_pitch,  float target_yaw,
    float dt) {
    
    // Calcular errores en ángulos de Euler
    // (Asumir que state contiene euler_roll, euler_pitch, euler_yaw)
    // Para simplificar, usamos ángulos pequeños
    
    float current_roll = state.imu_gyro_rad_s[0];   // Aproximación (usar EKF en producción)
    float current_pitch = state.imu_gyro_rad_s[1];
    float current_yaw = state.imu_gyro_rad_s[2];
    
    float errors[3] = {
        target_roll - current_roll,
        target_pitch - current_pitch,
        target_yaw - current_yaw
    };
    
    // Controlador PID para cada eje
    for (int i = 0; i < 3; i++) {
        float error = errors[i];
        
        // Término integral con anti-windup
        _error_integral[i] += error * dt;
        if (fabs(_error_integral[i]) > 1.0f) {
            _error_integral[i] = (error > 0) ? 1.0f : -1.0f;
        }
        
        // Término derivativo
        float error_derivative = (error - _error_prev[i]) / dt;
        _error_prev[i] = error;
        
        // Salida PID
        _output[i] = 
            _gains[i].kp * error +
            _gains[i].ki * _error_integral[i] +
            _gains[i].kd * error_derivative;
        
        // Saturación
        if (_output[i] > 2.0f) _output[i] = 2.0f;
        if (_output[i] < -2.0f) _output[i] = -2.0f;
    }
}

void AttitudeController::getOutput(float& p_cmd, float& q_cmd, float& r_cmd) const {
    p_cmd = _output[0];  // Roll rate
    q_cmd = _output[1];  // Pitch rate
    r_cmd = _output[2];  // Yaw rate
}

void AttitudeController::setGains(const Gains& roll_gains, const Gains& pitch_gains, const Gains& yaw_gains) {
    _gains[0] = roll_gains;
    _gains[1] = pitch_gains;
    _gains[2] = yaw_gains;
}

void AttitudeController::resetIntegrals() {
    memset(_error_integral, 0, sizeof(_error_integral));
    memset(_error_prev, 0, sizeof(_error_prev));
    memset(_output, 0, sizeof(_output));
}
```

---

### Ejemplo 4: Integración en SensorManager (Adaptado)

Archivo: `src/sensor_manager.cpp` (versión adaptada con EKF)

```cpp
#include "sensor_manager.hpp"
#include "ekf_lightweight.hpp"

namespace stallion {

SensorManager::SensorManager()
    : contador_lecturas_(0), alt_init_(false), alt_filtro_(0.0f) {}

bool SensorManager::inicializar() {
    if (!inicializar_imu()) {
        Serial.println("Error: IMU init failed");
        return false;
    }
    if (!inicializar_barometro()) {
        Serial.println("Error: Baro init failed");
        return false;
    }
    if (!inicializar_compass()) {
        Serial.println("Error: Compass init failed");
        return false;
    }
    
    estado_.sensors_valid = true;
    return true;
}

bool SensorManager::update(uint64_t now_us) {
    uint64_t dt_us = now_us - datos_raw_.timestamp_us;
    float dt = dt_us / 1e6f;
    
    if (dt > 0.1f || dt < 0.0001f) return false;  // Sanity check
    
    leer_imu();
    leer_barometro();
    leer_compass();
    
    actualizar_estado_estimado();
    
    datos_raw_.timestamp_us = now_us;
    contador_lecturas_++;
    
    return true;
}

StateEstimate SensorManager::state() const {
    return estado_;
}

void SensorManager::leer_imu() {
    // Usar driver adaptado si está disponible
    if (imu_.accelAvailable() && imu_.gyroAvailable()) {
        imu_.readAccel();
        imu_.readGyro();
        
        // Convertir a m/s² y rad/s (ver Ejemplo 1)
        datos_raw_.imu_acx = imu_.accelX() * 9.81f / 2048.0f;
        datos_raw_.imu_acy = imu_.accelY() * 9.81f / 2048.0f;
        datos_raw_.imu_acz = imu_.accelZ() * 9.81f / 2048.0f;
        
        datos_raw_.imu_grx = imu_.gyroX() * M_PI / 180.0f / 131.0f;
        datos_raw_.imu_gry = imu_.gyroY() * M_PI / 180.0f / 131.0f;
        datos_raw_.imu_grz = imu_.gyroZ() * M_PI / 180.0f / 131.0f;
    }
}

void SensorManager::actualizar_estado_estimado() {
    // Copiar datos crudos a estado estimado
    memcpy(estado_.imu_accel_mps2, &datos_raw_.imu_acx, 3 * sizeof(float));
    memcpy(estado_.imu_gyro_rad_s, &datos_raw_.imu_grx, 3 * sizeof(float));
    
    // Filtro complementario simple para altitud
    if (!alt_init_) {
        alt_filtro_ = datos_raw_.baro_altitude;
        alt_init_ = true;
    } else {
        // Fusión simple: 95% baro + 5% integración
        alt_filtro_ = 0.95f * datos_raw_.baro_altitude + 
                      0.05f * (alt_filtro_ + estado_.imu_accel_mps2[2] * 0.01f);
    }
    
    estado_.alt_baro_m = alt_filtro_;
    estado_.airspeed_raw_mps = 0.0f;  // GPS/pitot aquí
    estado_.sensors_valid = true;
}

} // namespace stallion
```

---

## Limitaciones y Constraints

### Limitaciones de Hardware
| Aspecto | Tu STM32F405 | PX4 (Pixhawk) |
|--------|-------------|---------------|
| Flash | 1 MB | 2+ MB |
| RAM | 256 KB | 512 KB - 2 MB |
| CPU | 168 MHz | 168-240 MHz |
| **Implicación** | ⚠️ Ajustado | ✅ Holgado |

### Limitaciones de Software
1. **No puedes incluir**:
   - NuttX RTOS (usa bare-metal o Arduino scheduler)
   - uORB completo (usa interfaces simples)
   - Logging en SD (demasiados I/O)
   - Interfaz gráfica

2. **Puedes incluir** (con adaptaciones):
   - Drivers de sensores (reimplementados)
   - EKF ligero (matrices 13x13 en lugar de 24x24)
   - Controladores PID (simplificados)
   - MAVLink (básico)

### Limitaciones de Rendimiento
```
Presupuesto de Memoria Total:
├─ Arduino/Hardware abstraction: ~30 KB
├─ Tu código modular actual: ~50-80 KB
├─ EKF + Controladores: ~15-20 KB
├─ Drivers PX4 adaptados: ~20-30 KB
└─ Librerías (Wire, SPI, etc.): ~20 KB
─────────────────────────────────
   Total: ~135-180 KB (AJUSTADO pero viable)
```

### Limitaciones de Tiempo Real
```
Requerimientos de latencia:
- IMU lectura: 200 Hz (5 ms)
- Estimación: 100 Hz (10 ms)
- Control: 50 Hz (20 ms)
- GPS: 5 Hz (200 ms)

Arduino scheduler puede manejar esto SIN RTOS.
Pero es crítico: NO PUEDES bloquear.
```

---

## Arquitectura Híbrida Recomendada

### Opción A: Integración Modular (⭐ RECOMENDADA)

```
┌─────────────────────────────────────────────┐
│         Tu Arquitectura Actual               │
├─────────────────────────────────────────────┤
│ CoordinadorDeTareas (Scheduler)             │
│  ├─ SensorManager (IMU, Baro, Compass)    │◄──── Drivers adaptados PX4
│  ├─ EKFLightweight (Estimación)           │◄──── EKF simplificado
│  ├─ AttitudeController (Control PID)      │◄──── Controlador adaptado
│  ├─ MotorAllocator (Mezcla)               │
│  └─ FailsafeManager (Seguridad)           │
├─────────────────────────────────────────────┤
│ Hardware: STM32F405 + Sensores              │
└─────────────────────────────────────────────┘
```

**Ventajas**:
- Máximo control
- Memoria optimizada
- Sin dependencias de PX4
- Escalable

**Implementación**:
1. Mantener tu estructura modular
2. Adaptar drivers de PX4 como implementaciones de interfaces
3. Usar EKF ligero en lugar de complex estimation
4. Integrar controladores PID de PX4 con tuning propio

---

### Opción B: Hybrid con MAVLink

```
┌─────────────────────────────────────────────┐
│      Tu STM32F405 (Lightweight Stack)       │
├─────────────────────────────────────────────┤
│ Módulos: Sensores, Estimación simple       │
│ Control: Actitud manual o automático       │
│ Comunicación: MAVLink TX/RX                │
└─────────────────────────────────────────────┘
              MAVLink Protocol
              (Serial @ 57600 baud)
                     ↓↑
┌─────────────────────────────────────────────┐
│         Pixhawk + PX4 Completo              │
├─────────────────────────────────────────────┤
│ Estimación avanzada, Navigation, etc.      │
│ Comandos de mission planning                │
│ Logging y telemetría GCS                   │
└─────────────────────────────────────────────┘
```

**Ventajas**:
- Delegación de cálculos pesados
- Compatible con ecosistema PX4
- Escalabilidad

**Desventajas**:
- Necesita Pixhawk adicional (~$200+)
- Latencia en comunicación
- Complejidad en tiempos reales críticos

---

## Alternativas Más Ligeras

### Alternativa 1: SIL (Software-in-the-Loop) sin Integración
Si solo necesitas **validar algoritmos**:

```cpp
// Solo implementar EKF + Control sin hardware real
// Usar simulación Simulink (ya tienes fuerzas_v4.m)
// Ventaja: No toques hardware, solo algoritmos
```

**Recomendación**: ✅ Para prototipado rápido

---

### Alternativa 2: Usar librerías ligeras alternativas

En lugar de adaptar PX4, usa:

| Librería | Uso | Tamaño |
|----------|-----|--------|
| [Madgwick Filter](https://github.com/arduino-libraries/MadgwickAHRS) | Actitud | ~5 KB |
| [Simple Kalman](https://github.com/denyssene/SimpleKalman) | Estimación | ~2 KB |
| [ArduPilot drivers](https://github.com/ArduPilot/ardupilot) | Sensores | ~30-50 KB |
| [Betaflight flight controller](https://github.com/betaflight/betaflight) | Control | ~20-40 KB |

**Ventaja**: Documentación más clara, comunidad Arduino más grande

**Desventaja**: Menos features que PX4

---

### Alternativa 3: Emular interfaz MAVLink local
```cpp
// Tu código produce salidas MAVLink
// Se comunica localmente con Pixhawk
// Mejor performance sin overhead de serialización
```

---

## Resumen y Recomendación Final

### ¿Qué hacer?

**RECOMENDACIÓN PRINCIPAL: Opción A (Integración Modular)**

1. **Paso 1**: Adaptar drivers de PX4 (ICM42688, DPS310, etc.)
   - Tiempo: 2-3 días
   - Complejidad: Media
   - Ganancia: Drivers probados

2. **Paso 2**: Implementar EKF ligero basado en PX4
   - Tiempo: 3-5 días
   - Complejidad: Alta
   - Ganancia: Estimación robusta

3. **Paso 3**: Adaptar controladores PID de PX4
   - Tiempo: 1-2 días
   - Complejidad: Baja
   - Ganancia: Control estable

4. **Paso 4**: Integrar en tu scheduler existente
   - Tiempo: 1 día
   - Complejidad: Baja
   - Ganancia: Sistema funcional completo

**Total: ~1-2 semanas de desarrollo**

### Próximos Pasos

1. Clonar PX4: `git clone https://github.com/PX4/PX4-Autopilot.git`
2. Analizar drivers en `src/drivers/`
3. Comenzar adaptación con ICM42688 (ya tienes la librería)
4. Implementar EKF ligero (usar código Ejemplo 2)
5. Integrar en SensorManager (ver Ejemplo 4)
6. Tunear parámetros y validar en simulación

---

## Referencias

- [PX4 GitHub](https://github.com/PX4/PX4-Autopilot)
- [PX4 Developer Guide](https://docs.px4.io/main/en/development/development.html)
- [MAVLink Protocol](https://mavlink.io/)
- [ArduPilot Framework](https://ardupilot.org/)
- [Extended Kalman Filter Theory](https://en.wikipedia.org/wiki/Extended_Kalman_filter)

---

**Documento generado**: 2024-04-29
**Versión**: 1.0
**Estado**: Listo para implementación
