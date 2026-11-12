# Análisis Comparativo: Alternativas de Integración PX4

## Executive Summary

| Opción | Viabilidad | Esfuerzo | Performance | Recomendación |
|--------|-----------|---------|-------------|---------------|
| **Integración Directa de Módulos** | ✅✅✅ | 👷 1-2 sem | 🚀 Excelente | ⭐⭐⭐ MEJOR |
| **MAVLink + Pixhawk** | ✅✅ | 👷 3-4 días | 🚁 Bueno | ⭐⭐ Si tienes presupuesto |
| **Librerías Alternas (ArduPilot)** | ✅✅ | 👷 3-5 días | 🛩️ Muy bueno | ⭐⭐ Alternativa |
| **Software-in-the-Loop (SIL)** | ✅ | 👷 2-3 días | 💻 N/A | ⭐ Para prototipado |
| **PX4 Completo (NuttX)** | ❌ | 👷👷👷 1+ mes | ⚠️ N/A | ❌ NO VIABLE |

---

## 1️⃣ Integración Directa de Módulos ⭐ RECOMENDADA

### Descripción
Adaptar código específico de PX4 (drivers, EKF, controladores) a tu arquitectura modular, eliminando dependencias de NuttX.

### Arquitectura
```
┌─────────────────────────────────────────────┐
│   Tu Scheduler Arduino (CoordinadorDeTareas)│
├─────────────────────────────────────────────┤
│ SensorManager                               │
│ ├─ ICM42688 (driver PX4-adapted)           │
│ ├─ DPS310 (driver PX4-adapted)             │
│ └─ QMC5883L (driver PX4-adapted)           │
├─────────────────────────────────────────────┤
│ EstimationModule (EKF Lightweight)          │
│ ├─ Attitude Estimator (quaterniones)       │
│ ├─ Altitude Filter (complementario)        │
│ └─ State Predictor                         │
├─────────────────────────────────────────────┤
│ ControlModule                               │
│ ├─ Attitude Controller (PID)               │
│ ├─ Position Controller (simple)            │
│ └─ Thrust Allocator                        │
└─────────────────────────────────────────────┘
```

### Componentes Adaptables de PX4

#### Driver: ICM42688
**Ubicación PX4**: `src/drivers/imu/icm42688/ICM42688.cpp`
- Funciones: `init()`, `read()`, `configuration()`
- Esfuerzo: ⭐ Bajo (ya proporcionado: `imu_px4_driver.hpp`)
- Dependencias: Solo Wire (I2C)

#### Estimación: EKF2
**Ubicación PX4**: `src/modules/ekf2/EKF2.cpp`
- Funciones: `predict()`, `update_accel()`, `update_mag()`, `getAttitude()`
- Esfuerzo: ⭐⭐⭐ Alto (matrices Kalman grandes)
- Alternativa: EKF Ligero simplificado (13 estados en lugar de 24)

#### Control: Attitude
**Ubicación PX4**: `src/modules/mc_att_control/MulticopterAttitudeControl.cpp`
- Funciones: `update()`, PID control
- Esfuerzo: ⭐⭐ Medio
- Compatibilidad: Fácil de adaptar

### Ejemplo de Integración (ya proporcionado)

Ver archivos:
- `include/imu_px4_driver.hpp`
- `src/drivers/imu_px4_driver.cpp`
- `include/ekf_lightweight.hpp` (en guía principal)
- `include/attitude_controller.hpp` (en guía principal)

### Ventajas
✅ Máximo control y optimización
✅ Sin dependencias externas pesadas
✅ Compatible con arquitectura actual
✅ Excelente performance
✅ Fácil debugging
✅ Pruebas en Simulink directo

### Desventajas
❌ Trabajo manual de adaptación
❌ Testing exhaustivo necesario
❌ Tuning de parámetros requerido

### Viabilidad: STM32F405
- Flash: 1 MB (suficiente)
- RAM: 256 KB (ajustado pero viable)
- CPU: 168 MHz (suficiente)
- **Veredicto**: ✅✅✅ COMPLETAMENTE VIABLE

### Timeline
```
Semana 1:
├─ Día 1-2: Integración drivers PX4
├─ Día 3-4: Implementar EKF ligero
└─ Día 5: Testing e integración

Semana 2:
├─ Día 1: Agregar controladores
├─ Día 2-3: Tuning de parámetros
└─ Día 4-5: Validación en simulación/campo
```

### Recursos Necesarios
- Conocimiento de C++ avanzado
- Entendimiento de control aeronáutico
- Acceso a repositorio PX4
- Herramientas de debugging (scope, analizador lógico)

### Resultado
**Sistema VTOL completamente funcional en tu hardware con máximo control**

---

## 2️⃣ MAVLink + Pixhawk (Comunicación)

### Descripción
Tu STM32F405 corre estimación básica y control simple. Una Pixhawk externa corre PX4 completo y proporciona comandos vía MAVLink.

### Arquitectura
```
┌──────────────────────────┐
│  STM32F405 (Tu código)   │
├──────────────────────────┤
│ ├─ Lectura de sensores   │
│ ├─ Estimación simple     │
│ ├─ Telemetría MAVLink TX │
│ └─ Recepción de comandos │
└──────────────────────────┘
           ║ UART
           ║ MAVLink
           ║ (57600 baud)
           ∨
┌──────────────────────────┐
│  Pixhawk + PX4 Completo  │
├──────────────────────────┤
│ ├─ EKF avanzado          │
│ ├─ Navigation module     │
│ ├─ Mission planning      │
│ └─ Control avanzado      │
└──────────────────────────┘
           ║ MAVLink
           ∨
┌──────────────────────────┐
│  GCS (QGroundControl)    │
│  Telemetría en vivo      │
└──────────────────────────┘
```

### Componentes MAVLink

**Mensajes Clave**:
- `ATTITUDE`: Ángulos de Euler
- `HEARTBEAT`: Latido del sistema
- `ATTITUDE_TARGET`: Comandos de actitud deseada
- `BATTERY_STATUS`: Estado de batería
- `GPS_RAW_INT`: Posición GPS

### Ejemplo de Código

```cpp
// include/mavlink_bridge.hpp
#pragma once
#include <Arduino.h>
#include "mavlink/mavlink_types.h"
#include "module_data_types.hpp"

class MAVLinkBridge {
public:
    MAVLinkBridge();
    
    // Enviar telemetría desde tu código
    void send_heartbeat(uint32_t system_time);
    void send_attitude(float roll, float pitch, float yaw, float rollspeed, float pitchspeed, float yawspeed);
    void send_battery(float voltage, float current);
    
    // Recibir comandos desde Pixhawk
    void receive_attitude_target(
        float& roll_target, 
        float& pitch_target, 
        float& yaw_target,
        float& thrust_target
    );
    
    // Update periódicamente
    void update();
    
private:
    uint16_t message_counter_;
    uint32_t last_heartbeat_ms_;
    
    void _send_message(const mavlink_message_t& msg);
    void _parse_incoming_message();
};
```

```cpp
// src/mavlink_bridge.cpp
#include "mavlink_bridge.hpp"

MAVLinkBridge::MAVLinkBridge() 
    : message_counter_(0), last_heartbeat_ms_(0) {}

void MAVLinkBridge::send_heartbeat(uint32_t system_time) {
    if (millis() - last_heartbeat_ms_ < 1000) return;  // 1 Hz
    
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(
        1,                              // system_id
        1,                              // component_id
        &msg,
        MAV_TYPE_QUADROTOR,            // type
        MAV_AUTOPILOT_PX4,             // autopilot
        MAV_MODE_STABILIZE_ARMED,      // base_mode
        0,                              // custom_mode
        MAV_STATE_ACTIVE                // system_status
    );
    
    _send_message(msg);
    last_heartbeat_ms_ = millis();
}

void MAVLinkBridge::send_attitude(
    float roll, float pitch, float yaw,
    float rollspeed, float pitchspeed, float yawspeed) {
    
    mavlink_message_t msg;
    mavlink_msg_attitude_pack(
        1, 1, &msg,
        millis(),          // time_boot_ms
        roll,              // rad
        pitch,             // rad
        yaw,               // rad
        rollspeed,         // rad/s
        pitchspeed,        // rad/s
        yawspeed           // rad/s
    );
    
    _send_message(msg);
}

void MAVLinkBridge::_send_message(const mavlink_message_t& msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, &msg);
    
    // Enviar por Serial (UART)
    Serial1.write(buffer, len);
}
```

### Ventajas
✅ Escalable a múltiples vehículos
✅ Compatible con ecosistema PX4/ArduPilot
✅ Telemetría remota completa
✅ Mission planning avanzado
✅ Múltiples GCS disponibles

### Desventajas
❌ Requiere Pixhawk adicional (~$200-400)
❌ Latencia en comunicación (puede ser crítica)
❌ Complejidad en debugging distribuido
❌ Costo adicional de hardware

### Viabilidad: STM32F405
- Flash: 500 KB (librerías MAVLink + código)
- RAM: 100 KB (buffers de comunicación)
- CPU: Mínimo 10% para procesamiento MAVLink
- **Veredicto**: ✅✅ VIABLE pero costoso

### Timeline
```
Día 1-2: Implementar MAVLink bridge
Día 2-3: Configurar comunicación serial
Día 3-4: Testing y debugging
```

### Costo Hardware
- Pixhawk 4: $300-400
- GPS, telemetría, antenas: $100-200
- **Total**: ~$500

### Resultado
**Sistema distribuido: tu VTOL + Pixhawk externo. Compatible con GCS comercial.**

---

## 3️⃣ Librerías Alternas (ArduPilot)

### Descripción
En lugar de adaptar PX4, usar librerías del proyecto ArduPilot que tienen mejor soporte Arduino.

### Opciones

#### A) Usar solo drivers de ArduPilot
```
https://github.com/ArduPilot/ardupilot/tree/master/libraries

libraries/
├── AP_AHRS/           (Attitude and Heading Reference System)
├── AP_Math/           (Matemáticas)
├── AP_Baro/           (Barómetro)
├── AP_Compass/        (Brújula)
├── AP_GPS/            (GPS)
└── AP_InertialSensor/ (IMU)
```

#### B) Usar ArduCopter lite
- Versión simplificada de ArduPilot para boards pequeños
- Más documentación que PX4 lite

### Ejemplo: Usar AHRS de ArduPilot

```cpp
#include "AP_AHRS.h"
#include "AP_Math.h"

AP_AHRS_DCM ahrs;  // Complementary filter (simpler than EKF)

void setup() {
    ahrs.init();
}

void loop() {
    ahrs.update();
    
    float roll = ahrs.roll;
    float pitch = ahrs.pitch;
    float yaw = ahrs.yaw;
}
```

### Ventajas
✅ Mejor documentación Arduino
✅ Comunidad Arduino más activa
✅ Librerías modularizadas
✅ Menos dependencias de RTOS

### Desventajas
❌ Menos features que PX4
❌ Rendimiento potencialmente menor
❌ Menos actualizaciones

### Viabilidad: STM32F405
- Flash: 800-900 KB
- RAM: ~150 KB
- **Veredicto**: ✅✅ VIABLE

### Timeline
```
Semana 1:
├─ Día 1-2: Evaluación de librerías
├─ Día 2-3: Integración de drivers
├─ Día 4: Testing
└─ Día 5: Tuning
```

### Recursos
- [ArduPilot GitHub](https://github.com/ArduPilot/ardupilot)
- [ArduPilot Developer Guide](https://ardupilot.org/dev/)

### Resultado
**Sistema con librerías ArduPilot. Buena alternativa si PX4 te parece pesado.**

---

## 4️⃣ Software-in-the-Loop (SIL)

### Descripción
**NO** integrar en hardware real. Solo validar algoritmos en simulación.

### Arquitectura
```
Simulink (tienes: fuerzas_v4.m, construir_planta_vtol.m)
    ↓
Exportar a código C
    ↓
Tu STM32F405 ejecuta simulación
    ↓
Resultados via UART al PC
```

### Ejemplo
```cpp
// Generar C-code desde Simulink
#include "simulink_model.h"

void setup() {
    initialize_model();
}

void loop() {
    // Inputs
    double u[4] = {motor1_cmd, motor2_cmd, motor3_cmd, motor4_cmd};
    
    // Step
    step_model(u);
    
    // Outputs
    double y[13];  // state vector
    get_model_output(y);
    
    // Enviar por Serial
    for (int i = 0; i < 13; i++) {
        Serial.print(y[i]);
        Serial.print(",");
    }
    Serial.println();
}
```

### Ventajas
✅ No depende de hardware real
✅ Fácil de hacer iteraciones
✅ Debugging perfecto
✅ Rápido de prototipar

### Desventajas
❌ No valida comportamiento real
❌ No prueba actuadores
❌ No es código production

### Viabilidad
- Flash: 100-200 KB
- RAM: 50 KB
- **Veredicto**: ✅✅✅ MUY VIABLE

### Timeline
```
Día 1: Exportar Simulink a C
Día 2-3: Adaptar para STM32
Día 4: Testing
```

### Resultado
**Algoritmos validados sin hardware. Buena para prototipado inicial.**

---

## 5️⃣ PX4 Completo (NuttX)

### Descripción
Portar PX4 completo a STM32F405.

### Requisitos
- NuttX RTOS compilado y testeado para F405
- Drivers periféricos para tu board
- ~2-3 MB Flash (NO TIENES)
- ~1 MB RAM (NO TIENES)

### Limitaciones
```
┌─────────────────────┐
│ PX4 Firmware Full   │
├─────────────────────┤
│ Tamaño: 2-3 MB      │ ← Flash: 1 MB ❌
│ RAM: 512K - 1MB     │ ← RAM: 256 KB ❌
│ CPU: 168 MHz        │ ← OK
│ RTOS: NuttX         │ ← No soportado
└─────────────────────┘
```

### Veredicto
**❌ NO ES VIABLE**. 
- Tu hardware es demasiado pequeño
- NuttX tiene overhead de RTOS
- Flash insuficiente

### Alternativa
Si realmente necesitas PX4 completo:
- Cambiar a **Pixhawk 4** o **Pixhawk 4 Mini** (STM32H7, 2 MB Flash)
- O usar opción 2: MAVLink + Pixhawk externo

---

## Tabla de Decisión Rápida

```
¿Cuál es tu restricción principal?

├─ PRESUPUESTO limitado?
│  └─ Usa: Integración Directa (Opción 1) ⭐⭐⭐
│
├─ TIEMPO limitado?
│  └─ Usa: Librerías ArduPilot (Opción 3) ⭐⭐
│
├─ ¿Quieres máximo CONTROL?
│  └─ Usa: Integración Directa (Opción 1) ⭐⭐⭐
│
├─ ¿Necesitas ESCALABILIDAD futura?
│  └─ Usa: MAVLink + Pixhawk (Opción 2) ⭐⭐
│
├─ ¿Solo PROTOTIPADO?
│  └─ Usa: Software-in-the-Loop (Opción 4) ⭐⭐⭐
│
└─ ¿Quieres PX4 COMPLETO?
   └─ Cambiar hardware a Pixhawk 4 ⭐⭐
```

---

## Matriz de Evaluación

| Criterio | Opción 1 | Opción 2 | Opción 3 | Opción 4 |
|----------|----------|----------|----------|----------|
| Costo | $ | $$$$ | $$ | $ |
| Esfuerzo | 👷👷 | 👷 | 👷 | 👷 |
| Performance | ⚡⚡⚡ | ⚡⚡ | ⚡⚡⚡ | N/A |
| Control | 🎮🎮🎮 | 🎮🎮 | 🎮🎮 | 🎮🎮🎮 |
| Documentación | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐ |
| Comunidad | ⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ | ⭐ |
| Debugging | ⭐⭐⭐ | ⭐⭐ | ⭐⭐ | ⭐⭐⭐ |
| SCORE TOTAL | 16/18 | 13/18 | 14/18 | 11/18* |

*SIL no es para producción

---

## Recomendación Final

### 🏆 MEJOR OPCIÓN: Integración Directa (Opción 1)

**Por qué:**
1. Máximo control y optimización
2. Compatible con tu arquitectura actual
3. Sin costos adicionales
4. Excelente performance
5. Código completamente tunable

**Implementación:**
```
Semana 1: Adaptar drivers PX4
Semana 2: Implementar EKF + Control
Semana 3: Testing y validación

→ Sistema completo, optimizado, listo para VTOL
```

**Recursos proporcionados:**
- ✅ `imu_px4_driver.hpp` (driver ICM42688)
- ✅ `imu_px4_driver.cpp` (implementación)
- ✅ `ekf_lightweight.hpp` (en guía principal)
- ✅ `attitude_controller.hpp` (en guía principal)
- ✅ Guía de integración rápida
- ✅ Este documento comparativo

---

## Next Steps

1. **Ahora**: Revisar `px4_integration_guide.md` (guía principal)
2. **Mañana**: Integrar `imu_px4_driver` en tu código
3. **Semana 1**: Agregar EKF ligero
4. **Semana 2**: Implementar controladores
5. **Semana 3**: Testing en campo

---

**Documento**: 2024-04-29
**Versión**: 1.0
**Autor**: Análisis comparativo para VTOL con STM32F405

