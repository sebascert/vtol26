/**
 * @file pso_integration_guide.md
 * @brief Guía para integrar el PSO real en la arquitectura
 */

# Guía de Integración PSO Real

## Estado Actual

Actualmente `PSO_Allocator` usa un **mixer geométrico simplificado**. Esta guía explica cómo integrar el PSO real sin afectar el resto del sistema.

---

## 📋 Requisitos del PSO

El PSO debe:
1. **Entrada:** torque_roll, torque_pitch, torque_yaw, thrust
2. **Salida:** velocidades de motores (RPM), ángulos de servos (rad)
3. **Restricciones:** respeto rangos operacionales

---

## 🔄 Pasos de Integración

### Paso 1: Preparar Archivos PSO

Copiar desde `sim/PSO/` a la estructura de proyecto:

```bash
# Headers
cp sim/PSO/pso_state.hpp include/
cp sim/PSO/pso_environment.hpp include/

# Implementación (si es .cpp)
cp sim/PSO/pso_environment.cpp src/
```

### Paso 2: Crear Wrapper de Inicialización

Agregar en `include/pso_allocator.hpp`:

```cpp
#ifdef USE_PSO_REAL
#include "pso_state.hpp"
#include "pso_environment.hpp"
#endif

class PSO_Allocator {
private:
#ifdef USE_PSO_REAL
    PSOEnvironment pso_env_;
    PSO_State best_solution_;
#endif
    
    bool solve_pso_real(...);  // Nueva implementación
};
```

### Paso 3: Implementar Solver PSO

Reemplazar en `src/pso_allocator.cpp`:

```cpp
#ifdef USE_PSO_REAL
bool PSO_Allocator::solve_pso_real(float torque_roll, float torque_pitch, 
                                    float torque_yaw, float thrust) {
    // 1. Configurar objetivo del PSO
    pso_env_.set_desired_torques(torque_roll, torque_pitch, torque_yaw);
    pso_env_.set_desired_thrust(thrust);
    
    // 2. Ejecutar optimización PSO
    pso_env_.optimize(NUM_ITERATIONS);  // ej: 50 iteraciones
    
    // 3. Obtener solución óptima
    best_solution_ = pso_env_.get_best_solution();
    
    // 4. Extraer velocidades y ángulos
    const float* motors = best_solution_.motor_speeds;
    const float* servos = best_solution_.servo_angles;
    
    // 5. Validar y aplicar límites
    for (int i = 0; i < 3; i++) {
        motor_speeds_[i] = fmaxf(motor_min_rpm_, 
                                 fminf(motor_max_rpm_, motors[i]));
        servo_angles_[i] = fmaxf(servo_min_rad_, 
                                 fminf(servo_max_rad_, servos[i]));
    }
    
    return true;
}
#endif
```

### Paso 4: Actualizar solve() Principal

```cpp
bool PSO_Allocator::solve(float torque_roll, float torque_pitch, float torque_yaw,
                          float thrust, float dt) {
    if (dt <= 0) {
        is_valid_ = false;
        return false;
    }

#ifdef USE_PSO_REAL
    bool success = solve_pso_real(torque_roll, torque_pitch, torque_yaw, thrust);
#else
    bool success = solve_simple(torque_roll, torque_pitch, torque_yaw, thrust);
#endif
    
    is_valid_ = success;
    return success;
}
```

### Paso 5: Actualizar platformio.ini

```ini
[env:f405_wing_v2]
platform = ststm32
board = genericSTM32F405RG
framework = arduino

build_flags =
    # ... flags existentes ...
    -D USE_PSO_REAL  ; Habilitar PSO real (comentar para usar mixer simple)
```

---

## 🧪 Validación de Integración

### Test 1: Compilación

```bash
pio run
```

Debe compilar sin errores (solo warnings sobre defines).

### Test 2: Verificación de Salidas

Agregar en `diagnostico()`:

```cpp
void PSO_Allocator::diagnostico() {
    const float* motors = get_motor_speeds();
    const float* servos = get_servo_angles();
    
    Serial.print("[PSO] Motors: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(motors[i]); Serial.print(" ");
    }
    Serial.print("| Servos: ");
    for (int i = 0; i < 3; i++) {
        Serial.print(servos[i]); Serial.print(" ");
    }
    Serial.println();
}
```

### Test 3: Respuesta a Setpoint

Verificar que cambios en setpoint generen cambios proporcionales en salidas:

```
SetPoint: ROLL +0.1   → Motors: [2050, 2150, 1950]  (diferencial)
SetPoint: PITCH +0.1  → Motors: [1950, 2050, 2050]  (mezcla)
SetPoint: THRUST +0.1 → Motors: [2200, 2200, 2200]  (simétrico)
```

---

## 📊 Comparativa: Mixer Simple vs PSO

### Mixer Simple (Actual)
```
✓ Rápido (sin iteración)
✓ Predecible y debuggeable
✓ Funciona para pruebas iniciales
✗ No óptimo para control tricóptero
✗ Acoplamiento entre ejes no ideal
```

### PSO Real
```
✓ Óptimo para tricóptero tiltrotor
✓ Manejo de restricciones complejas
✓ Respuesta coordinada de actuadores
✗ Computacionalmente más pesado
✗ Requiere tunning de parámetros PSO
```

---

## ⚙️ Parámetros PSO (Típicos)

Si el PSO tiene configuración:

```cpp
struct PSO_Config {
    int num_particles = 20;        // Tamaño del enjambre
    int num_iterations = 50;       // Iteraciones por solve()
    float inertia = 0.7f;          // Factor de inercia
    float cognitive = 1.5f;        // Coeficiente cognitivo
    float social = 1.5f;           // Coeficiente social
    float tolerance = 0.01f;       // Tolerancia de convergencia
};
```

Recomendación: Empezar con iteraciones bajas (10-20) en pruebas rápidas, aumentar en modo de campaña (50-100).

---

## 🔍 Debugging PSO

### Problema: Salidas erráticas

```cpp
// Agregar logging
if (!pso_env_.is_converged()) {
    Serial.println("[PSO] No convergió - aumentar iteraciones");
}

// Verificar restricciones
if (best_solution_.fitness > FEASIBILITY_THRESHOLD) {
    Serial.println("[PSO] Solución infactible");
}
```

### Problema: Respuesta lenta

```cpp
// Reducir iteraciones PSO
#define PSO_ITERATIONS 20  // Prueba con 20 en vez de 50

// O aumentar frecuencia de solver
static constexpr uint32_t ALLOCATOR_FREQ_HZ = 100;  // 100 en vez de 50
```

### Problema: Oscilaciones

```cpp
// Revisar ganancias del controlador de actitud
// Pueden causar setpoints erráticos que el PSO no puede seguir
controller_->set_attitude_gain(3.0f);  // Reducir de 4.5 a 3.0
```

---

## 📈 Monitoreo en Runtime

Agregar telemetría PSO a diagnostico:

```cpp
Serial.print("[PSO] Fitness: ");
Serial.print(best_solution_.fitness, 3);
Serial.print(" | Converged: ");
Serial.print(pso_env_.is_converged() ? "YES" : "NO");
Serial.print(" | Time: ");
Serial.print(pso_env_.get_solve_time_us());
Serial.println(" us");
```

---

## 🚀 Rollout Plan

1. **Fase 1:** Integrar PSO con flagging
2. **Fase 2:** Pruebas con mixer simple en paralelo
3. **Fase 3:** A/B testing PSO vs mixer
4. **Fase 4:** Calibración de parámetros PSO
5. **Fase 5:** Despliegue en modo operacional

---

## ❓ FAQ

### ¿Afecta al rest del sistema?

No. `PSO_Allocator` es una "caja negra". Cualquier cambio interno no afecta:
- StateEstimator
- QuaternionAttitudeController
- Scheduler
- Sensores

### ¿Puedo cambiar entre mixer y PSO en runtime?

Sí. El `#ifdef USE_PSO_REAL` permite compilar con o sin PSO. Para cambio en runtime:

```cpp
bool use_pso = true;  // variable global

bool PSO_Allocator::solve(...) {
    if (use_pso) {
        return solve_pso_real(...);
    } else {
        return solve_simple(...);
    }
}
```

### ¿Qué pasa si el PSO no converge?

Opciones:
1. Usar última solución válida
2. Fallback a mixer simple
3. Loguear advertencia e intentar siguiente ciclo

```cpp
if (!pso_env_.is_converged()) {
    Serial.println("[PSO] No convergió - usando solución anterior");
    // motor_speeds_ mantiene valores del ciclo anterior
    return true;
}
```

---

## ✅ Checklist de Integración

- [ ] Headers PSO copiados a `include/`
- [ ] Implementación PSO copiada a `src/`
- [ ] `pso_allocator.hpp` actualizado con includes
- [ ] `pso_allocator.cpp` implementa `solve_pso_real()`
- [ ] `platformio.ini` tiene flag `-D USE_PSO_REAL`
- [ ] Compilación exitosa
- [ ] Diagnostico muestra salidas PSO correctas
- [ ] Pruebas en hardware validan comportamiento

---

**Integración lista para cuando PSO esté disponible.** 🚀
