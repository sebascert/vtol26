# Guía Rápida: Integración de Driver PX4 en tu Código

## Resumen
Este documento te guía paso a paso cómo integrar el driver `imu_px4_driver.hpp/cpp` en tu proyecto actual sin romper la funcionalidad existente.

## Archivo: `platformio.ini`
No requiere cambios. El driver usa Solo Arduino + Wire standard.

## Paso 1: Configurar en `main.cpp`

### Antes (versión actual):
```cpp
#include <Arduino.h>
#include "scheduler.hpp"

using namespace stallion;

CoordinadorDeTareas coordinador;

void setup() {
    Serial.begin(115200);
    // ...
    coordinador.inicializar(micros());
}

void loop() {
    uint64_t ahora = micros();
    coordinador.ejecutar(ahora);
}
```

### Después (con driver PX4):
```cpp
#include <Arduino.h>
#include "scheduler.hpp"
#include "imu_px4_driver.hpp"  // ← NUEVO

using namespace stallion;

CoordinadorDeTareas coordinador;
IMU_PX4Driver imu_driver;        // ← NUEVO: Instancia del driver

void setup() {
    Serial.begin(115200);
    while (!Serial) {} 
    delay(2000);

    Serial.println("=== SISTEMA CON DRIVER PX4 ===");
    
    // === NUEVO: Inicializar driver PX4 ===
    IMU_PX4Driver::Config imu_config;
    imu_config.accel_sample_rate_hz = 200;
    imu_config.gyro_sample_rate_hz = 200;
    imu_config.accel_range_g = 16.0f;
    imu_config.gyro_range_dps = 2000.0f;
    imu_config.i2c_address = ICM42688_I2C_ADDR;
    imu_config.enable_fifo = false;
    imu_config.calibrate_on_init = true;
    
    if (!imu_driver.init(imu_config)) {
        Serial.println("FATAL: IMU initialization failed!");
        while(1);  // Halt
    }
    
    // Registrar driver globalmente (opcional)
    g_imu = &imu_driver;
    
    coordinador.inicializar(micros());

    Serial.println("INIT OK - Sistema listo");
}

void loop() {
    uint64_t ahora = micros();
    coordinador.ejecutar(ahora);

    static uint32_t t_diag = 0;
    if (millis() - t_diag >= 500) {
        t_diag = millis();
        coordinador.diagnostico();
        imu_driver.print_diagnostics();  // ← Debug adicional
    }
}
```

---

## Paso 2: Integrar en `SensorManager`

### Archivo: `include/sensor_manager.hpp`

**Cambiar esta sección:**

```cpp
// ANTES:
#include <ICM42688.h>

class SensorManager : public IEstimator {
private:
    ICM42688 imu_;  // ← Librería standard
};

// DESPUÉS:
#include "imu_px4_driver.hpp"  // ← Driver PX4

class SensorManager : public IEstimator {
private:
    IMU_PX4Driver imu_px4_;  // ← Driver con conceptos PX4
};
```

### Archivo: `src/sensor_manager.cpp`

**Reemplazar método `inicializar_imu()`:**

```cpp
// ANTES:
bool SensorManager::inicializar_imu() {
    if (!imu_.begin(Wire, ICM42688_I2C_ADDR_DEFAULT)) {
        Serial.println("IMU init failed");
        return false;
    }
    imu_.setAccelRange(ICM42688_ACCEL_RANGE_16G);
    imu_.setGyroRange(ICM42688_GYRO_RANGE_250DPS);
    return true;
}

// DESPUÉS:
bool SensorManager::inicializar_imu() {
    IMU_PX4Driver::Config config;
    config.accel_sample_rate_hz = 200;
    config.gyro_sample_rate_hz = 200;
    config.accel_range_g = 16.0f;
    config.gyro_range_dps = 250.0f;  // O 2000 si tienes estabilidad
    config.i2c_address = ICM42688_I2C_ADDR;
    config.enable_fifo = false;
    config.calibrate_on_init = true;
    
    if (!imu_px4_.init(config)) {
        Serial.println("PX4 IMU init failed");
        return false;
    }
    return true;
}
```

**Reemplazar método `leer_imu()`:**

```cpp
// ANTES:
void SensorManager::leer_imu() {
    if (imu_.accelAvailable() && imu_.gyroAvailable()) {
        imu_.readAccel();
        imu_.readGyro();
        datos_raw_.imu_acx = imu_.accelX() * 9.81f / 2048.0f;
        datos_raw_.imu_acy = imu_.accelY() * 9.81f / 2048.0f;
        datos_raw_.imu_acz = imu_.accelZ() * 9.81f / 2048.0f;
        // ...
    }
}

// DESPUÉS:
void SensorManager::leer_imu() {
    ICM42688Data imu_data;
    if (imu_px4_.read(imu_data)) {
        datos_raw_.imu_acx = imu_data.accel_x_mps2;
        datos_raw_.imu_acy = imu_data.accel_y_mps2;
        datos_raw_.imu_acz = imu_data.accel_z_mps2;
        datos_raw_.imu_grx = imu_data.gyro_x_rads;
        datos_raw_.imu_gry = imu_data.gyro_y_rads;
        datos_raw_.imu_grz = imu_data.gyro_z_rads;
        datos_raw_.timestamp_us = imu_data.timestamp_us;
    }
}
```

---

## Paso 3: Agregar EKF Ligero (Opcional pero Recomendado)

Crear archivo: `include/ekf_lightweight.hpp` (ver guía principal para contenido)

Luego, en `SensorManager`:

```cpp
#include "ekf_lightweight.hpp"

class SensorManager : public IEstimator {
private:
    IMU_PX4Driver imu_px4_;
    EKFLightweight ekf_;  // ← NUEVO
};

bool SensorManager::update(uint64_t now_us) {
    uint64_t dt_us = now_us - datos_raw_.timestamp_us;
    float dt = dt_us / 1e6f;
    
    leer_imu();
    leer_barometro();
    leer_compass();
    
    // ← NUEVO: Actualizar EKF con datos del IMU
    ekf_.predict(datos_raw_.imu_grx, datos_raw_.imu_gry, datos_raw_.imu_grz, dt);
    ekf_.update_accel(datos_raw_.imu_acx, datos_raw_.imu_acy, datos_raw_.imu_acz);
    
    actualizar_estado_estimado();
    
    return true;
}

StateEstimate SensorManager::state() const {
    // Usar estado de EKF en lugar de datos raw
    auto ekf_state = ekf_.getState();
    float roll, pitch, yaw;
    ekf_.getEuler(roll, pitch, yaw);
    
    // Convertir a StateEstimate
    // ...
    return estado_;
}
```

---

## Paso 4: Compilar y Probar

```bash
# En terminal, desde raíz del proyecto:
pio run -e f405_wing_v2 --verbose

# Si hay errores de compilación:
# 1. Verificar rutas de #include
# 2. Asegurar que imu_px4_driver.hpp está en include/
# 3. Asegurar que imu_px4_driver.cpp está en src/drivers/
```

---

## Paso 5: Validar Funcionamiento

### Serial Output esperado:
```
=== SISTEMA CON DRIVER PX4 ===
[IMU] Calibrating sensors...
[IMU] Please keep sensor stationary
...
[IMU] Calibration complete. Offsets: -12.5, 8.3, 15000.2 | -2.1, 1.5, -0.8
[IMU] Initialization successful
INIT OK - Sistema listo
```

### Si ves errores:
```
[IMU] Error: Invalid chip ID: 0x00
└─→ Verificar conexión I2C (SDA en PB9, SCL en PB8)
└─→ Verificar dirección I2C (debería ser 0x68)

[IMU] Error: Failed to set accel range
└─→ Problema en configuración de registros
└─→ Verificar datasheet del ICM42688
```

---

## Paso 6: Debugging avanzado

Si necesitas más debug, agrega esto en `loop()`:

```cpp
void loop() {
    uint64_t ahora = micros();
    coordinador.ejecutar(ahora);

    // Debug cada 100ms
    static uint32_t t_debug = 0;
    if (millis() - t_debug >= 100) {
        t_debug = millis();
        
        const auto& imu_data = imu_driver.last_data();
        Serial.print("IMU: A[");
        Serial.print(imu_data.accel_x_mps2, 2);
        Serial.print(", ");
        Serial.print(imu_data.accel_y_mps2, 2);
        Serial.print(", ");
        Serial.print(imu_data.accel_z_mps2, 2);
        Serial.print("] G[");
        Serial.print(imu_data.gyro_x_rads, 3);
        Serial.print(", ");
        Serial.print(imu_data.gyro_y_rads, 3);
        Serial.print(", ");
        Serial.print(imu_data.gyro_z_rads, 3);
        Serial.println("]");
    }
}
```

---

## Comparación: Antes vs Después

| Aspecto | Antes | Después |
|--------|-------|---------|
| Driver | ICM42688 (arduino-lib) | IMU_PX4Driver (PX4-adapted) |
| Calibración | Manual o librería | Automática en init |
| Conversión SI | Manual en SensorManager | Automática en driver |
| Estimación | Filtro complementario simple | EKF ligero (opcional) |
| Debug | Limited | Diagnósticos completos |
| Control | PID basic | PID con feedforward (próximo paso) |
| Tamaño código | ~5-10 KB | ~15-20 KB (adicional) |

---

## Próximos Pasos

1. ✅ Integrar driver PX4
2. 🔄 Implementar EKF ligero
3. 🔄 Agregar AttitudeController
4. 🔄 Integrar MAVLink (opcional)
5. 🔄 Tuning y validación en campo

---

## Troubleshooting

### "imu_px4_driver.hpp: No such file"
```
Solución:
- Ubicar archivo en: include/imu_px4_driver.hpp
- Verificar que platformio.ini incluya "include/" en build_flags
```

### "error: '_convert_raw_to_si' is private"
```
Solución:
- No llamar directamente a métodos privados
- Usar público: imu_driver.read(data)
```

### IMU no responde (I2C timeout)
```
Solución:
1. Verificar voltaje (3.3V ok, 5V puede dañar)
2. Verificar pull-ups en I2C (típicamente 2.2-4.7k ohm)
3. Ejecutar I2C scanner:
   uint8_t addr;
   for (addr = 1; addr < 127; addr++) {
       Wire.beginTransmission(addr);
       if (Wire.endTransmission() == 0) {
           Serial.print("Found device at 0x");
           Serial.println(addr, HEX);
       }
   }
```

---

**Estado**: Listo para implementar
**Complejidad**: Media (3-4 horas)
**Riesgo**: Bajo (cambios modulares, fácil de revertir)

