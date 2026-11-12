# Arquitectura PX4-Inspired para VTOL Tricóptero Tiltrotor

## 📋 Resumen

Se ha implementado una arquitectura de control modular y escalable inspirada en **PX4 Autopilot**, específicamente diseñada para tu VTOL tricóptero tiltrotor. El sistema mantiene separación clara de responsabilidades y permitirá transiciones suaves entre diferentes modos de vuelo (hover → transición → crucero).

---

## 🏗️ Flujo de Control

```
┌─────────────────┐
│   SENSORES      │  IMU (SPI), Barómetro (I2C), Magnetómetro
└────────┬────────┘
         ↓
┌─────────────────┐
│  STATE          │  Integración de cuaterniones
│  ESTIMATOR      │  Corrección con acelerómetro/mag/baro
└────────┬────────┘
         ↓
┌─────────────────────────────┐
│ QUATERNION ATTITUDE         │  Control geométrico
│ CONTROLLER                  │  Error quaterniónico → Torques
└────────┬────────────────────┘
         ↓
┌─────────────────────────────┐
│  PSO ALLOCATOR              │  Asignación óptima de control
│  (Control Allocation)       │  Torques+Thrust → Motores+Servos
└────────┬────────────────────┘
         ↓
┌──────────────────────┐
│ ACTUADORES           │  Velocidades motores, ángulos servo
└──────────────────────┘
```

---

## 📦 Componentes Creados

### 1. **QuaternionAttitudeController** 
**Ubicación:** `include/quaternion_attitude_controller.hpp`, `src/quaternion_attitude_controller.cpp`

**Responsabilidades:**
- Recibe `StateEstimate` (con quaternión actual) + `ControlSetpoint` (roll/pitch/yaw/thrust deseados)
- Convierte setpoint Euler a quaternión deseado
- Calcula error quaterniónico: `q_error = q_desired × inv(q_current)`
- Convierte componentes imaginarios a torques (control proporcional)
- Salida: `ControlOutput` con torques roll/pitch/yaw y thrust

**Ventajas:**
- Sin singularidades (a diferencia de Euler)
- Control continuo en todas las actitudes
- Más cercano a dinámicas reales de drones

**Ganancias (configurables):**
- `kp_attitude_` = 4.5f (ganancia proporcional)
- `kp_rate_` = 0.15f (para futuro control de rates)
- `thrust_max_` = 1.0f (normalizado)

---

### 2. **PSO_Allocator**
**Ubicación:** `include/pso_allocator.hpp`, `src/pso_allocator.cpp`

**Responsabilidades:**
- Recibe: `torque_roll`, `torque_pitch`, `torque_yaw`, `thrust`
- Resuelve el problema de asignación de control
- Salida: velocidades de motores (RPM) + ángulos de servo (rad)

**Arquitectura actual:**
- Implementa un solver **simplificado** basado en acoplamiento geométrico
- Preparado para integración futura con PSO real

**Estructura para Tricóptero:**
```
Motor 0: Frontal (norte) - genera thrust vertical
Motor 1: Trasero-Izq (suroeste) - con servo tilt
Motor 2: Trasero-Der (sureste) - con servo tilt

Configuración simétrica permite control unified
```

**Estrategia de asignación:**
- **Roll:** Diferencial entre motores traseros (1 y 2)
- **Pitch:** Combinación de todos los motores + tilt servo
- **Yaw:** Copter-style (motor 0 vs 1,2 contrarotadores)

---

### 3. **Scheduler Actualizado**
**Ubicación:** `include/scheduler.hpp`, `src/scheduler.cpp`

**Cambios principales:**
```cpp
// Antes
SensorManager → (solo sensores)

// Ahora
SensorManager
    ↓
StateEstimator        (100 Hz)
    ↓
QuaternionAttitudeController  (50 Hz)
    ↓
PSO_Allocator         (50 Hz)
```

**Interfaz pública:**
```cpp
CoordinadorDeTareas* coordinador;

// Inicializar
coordinador->inicializar(micros());

// Fijar setpoint
coordinador->set_attitude_setpoint(roll, pitch, yaw, thrust);

// Ejecutar (en loop)
coordinador->ejecutar(micros());

// Diagnostico
coordinador->diagnostico();
```

**Getters para debugging:**
```cpp
coordinador->sensors()    // SensorManager
coordinador->estimator()  // StateEstimator
coordinador->controller() // QuaternionAttitudeController
coordinador->allocator()  // PSO_Allocator
```

---

## 🚀 Características Clave

### ✅ Mantenido (No modificado)
- ✓ IMU por SPI (ICM42688)
- ✓ Barómetro HAL I2C (DPS310)
- ✓ StateEstimator existente (integración cuaterniones)
- ✓ AeroForceModel
- ✓ PSO en `sim/PSO/` (preparado para integración)
- ✓ math_types, config.h
- ✓ platformio.ini estructura

### ✨ Nuevo
- ✨ Control geométrico basado en cuaterniones (PX4-style)
- ✨ Allocator modular preparado para PSO
- ✨ Flujo de control limpio y separado por responsabilidades
- ✨ Soporte para hover + transición + crucero sin cambios de código

---

## 📊 Frecuencias de Operación

| Componente | Frecuencia | Intervalo |
|-----------|-----------|-----------|
| Sensores | 100 Hz | 10 ms |
| Estimador | 100 Hz | 10 ms |
| Controlador | 50 Hz | 20 ms |
| Allocator | 50 Hz | 20 ms |

---

## 🔧 Cómo Usar

### Inicialización en `setup()`
```cpp
void setup() {
    Serial.begin(115200);
    delay(2000);
    
    // Inicializa sensores, estimador, controlador, allocator
    coordinador.inicializar(micros());
    
    Serial.println("Sistema listo");
}
```

### Loop Principal
```cpp
void loop() {
    uint64_t ahora = micros();
    
    // Ejecutar arquitectura de control
    coordinador.ejecutar(ahora);
    
    // Opcional: establecer nuevo setpoint
    coordinador.set_attitude_setpoint(roll_sp, pitch_sp, yaw_sp, thrust_sp);
    
    // Opcional: diagnóstico cada 500ms
    if ((millis() - last_diag) >= 500) {
        coordinador.diagnostico();
        last_diag = millis();
    }
}
```

---

## 📈 Pruebas en main.cpp

Se incluye una **secuencia de prueba automática** que cicla:
1. **HOVER**: 3s en actitud neutra
2. **ROLL +**: 2s con roll positivo
3. **ROLL -**: 2s con roll negativo  
4. **PITCH +**: 2s con pitch positivo
5. **HOVER**: 2s recuperación

Esto es **SOLO para diagnóstico**. En operación real, los setpoints vendrían de:
- RC transmitter
- Controlador de misión
- Navegación autónoma

---

## 🧮 Ecuaciones de Control

### Error Quaterniónico
```
q_error = q_desired * inverse(q_current)
```
Representa la **mínima rotación** necesaria para pasar de actitud actual a deseada.

### Conversión a Torques
```
τ_roll  = kp * q_error.x
τ_pitch = kp * q_error.y
τ_yaw   = kp * q_error.z
```

### Limitaciones (Safety)
- Máximo torque: 0.5 Nm
- Máximo thrust: 1.0 (normalizado)
- Rango servo: ±20° (±0.349 rad)
- RPM motores: 1000-8000

---

## 🔗 Integración con PSO (Futuro)

Actualmente `PSO_Allocator` usa un mixer geométrico simple. Para integrar el PSO real:

1. **Copiar** `sim/PSO/pso_state.hpp` y `pso_environment.cpp` a `include/` y `src/`
2. **Modificar** `PSO_Allocator::solve()` para llamar al PSO
3. El resto del sistema **NO cambia**

Ejemplo (pseudocódigo):
```cpp
bool PSO_Allocator::solve(...) {
    // Llamar PSO
    PSOEnvironment env;
    env.set_desired_torques(tau_roll, tau_pitch, tau_yaw);
    env.set_desired_thrust(thrust);
    
    // PSO resuelve
    env.evaluate_solution(motors, servos);
    
    // Copiar resultados
    memcpy(motor_speeds_, motors, sizeof(motors));
    memcpy(servo_angles_, servos, sizeof(servos));
    return true;
}
```

---

## 📝 Salida Serial Típica

```
=== PX4-INSPIRED VTOL TRICOPTERO ===
Arquitectura: Sensores → StateEstimator → QuaternionController → PSO_Allocator
BOOT OK

[BARO] OK
[SCHED] Inicializando arquitectura PX4-inspired...
[SCHED] Sensores OK
[SCHED] StateEstimator OK
[SCHED] QuaternionAttitudeController OK
[SCHED] PSO_Allocator OK
[SCHED] Arquitectura inicializada

INIT OK - Sistema listo para control

[EST] R:0.001 P:0.002 Y:-0.001          ← Estado estimado
[CTL] TR:0.021 TP:0.045 TY:-0.003 TH:0.500  ← Salida controlador
[ALLOC] M[2100,2105,2098] S[0.002,0.003,0.001]  ← Velocidades motores + ángulos servo
[SP] HOVER                              ← Setpoint actual
```

---

## ⚠️ Limitaciones Actuales

1. **Mixer geométrico simple:** Bueno para pruebas, no óptimo
2. **Sin feedback de actuadores:** Asume respuesta lineal
3. **Sin detección de anomalías:** Propagar a futuro
4. **Sin integración GPS/flujo óptico:** Control de posición futura

---

## ✅ Checklist de Integración

- [x] QuaternionAttitudeController creado y compilable
- [x] PSO_Allocator creado con interface clara
- [x] Scheduler actualizado con pipeline completo
- [x] main.cpp limpiado y funcional
- [x] Compilación exitosa
- [ ] Pruebas en hardware (próximo paso)
- [ ] Integración PSO real (cuando esté listo)
- [ ] Calibración de ganancias (tunning)
- [ ] Modo de misión autónoma

---

## 📚 Referencia de Archivos

| Archivo | Propósito |
|---------|-----------|
| `include/quaternion_attitude_controller.hpp` | Interface del controlador |
| `src/quaternion_attitude_controller.cpp` | Implementación del controlador |
| `include/pso_allocator.hpp` | Interface del allocator |
| `src/pso_allocator.cpp` | Implementación del allocator |
| `include/scheduler.hpp` | Scheduler (modificado) |
| `src/scheduler.cpp` | Implementación scheduler |
| `src/main.cpp` | Punto de entrada (simplificado) |

---

## 🎯 Próximos Pasos

1. **Subir a hardware** y verificar que funciona
2. **Tunear ganancias** `kp_attitude_` y `kp_rate_` según comportamiento
3. **Integrar PSO** cuando esté listo
4. **Agregar telemetría MAVLink** (compatible PX4)
5. **Modo de misión** con waypoints

---

**Arquitectura completa y lista para pruebas.** 🚀
