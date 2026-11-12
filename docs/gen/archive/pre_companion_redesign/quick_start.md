# Quick Start - Arquitectura PX4 VTOL

## 🚀 Inicio Rápido (5 minutos)

### 1. Compilar y Subir

```bash
# En la terminal del workspace
pio run --target upload

# O desde VS Code: Ctrl+Alt+U (PlatformIO: Upload)
```

### 2. Monitor Serial

```bash
pio device monitor --baud 115200

# O desde VS Code: Ctrl+Alt+M (PlatformIO: Monitor)
```

### 3. Observar Salida

```
=== PX4-INSPIRED VTOL TRICOPTERO ===
...
[SCHED] Arquitectura inicializada

[EST] R:0.001 P:0.002 Y:-0.001
[CTL] TR:0.021 TP:0.045 TY:-0.003 TH:0.500
[ALLOC] M[2100,2105,2098] S[0.002,0.003,0.001]
[SP] HOVER
```

---

## 🎮 Usar en Tu Código

### Setpoint Manual

```cpp
// Cambiar actitud deseada
float roll_sp = 0.1f;       // radianes (~6 grados)
float pitch_sp = -0.05f;    // radianes (~-3 grados)
float yaw_sp = 0.0f;        // no cambiar yaw
float thrust = 0.6f;        // 60% de empuje

coordinador.set_attitude_setpoint(roll_sp, pitch_sp, yaw_sp, thrust);
```

### Leer Salidas de Control

```cpp
// Obtener el allocator
PSO_Allocator* alloc = coordinador.allocator();

if (alloc->is_valid()) {
    const float* motors = alloc->get_motor_speeds();   // RPM
    const float* servos = alloc->get_servo_angles();   // radianes
    
    // Usar para enviar a ESCs o controladores servo
    motor1_rpm = motors[0];
    motor2_rpm = motors[1];
    motor3_rpm = motors[2];
    
    servo1_angle = servos[0];
    servo2_angle = servos[1];
    servo3_angle = servos[2];
}
```

### Leer Estado Estimado

```cpp
StateEstimator* est = coordinador.estimator();
StateEstimate state = est->state();

Serial.print("Roll: ");
Serial.println(state.euler_roll_rad * 57.3f);  // convertir a grados

Serial.print("Altitude: ");
Serial.println(state.altitude_m);

Serial.print("Airspeed: ");
Serial.println(state.airspeed_mps);
```

### Leer Salida del Controlador

```cpp
QuaternionAttitudeController* ctrl = coordinador.controller();
ControlOutput output = ctrl->get_output();

Serial.print("Torque Roll: ");
Serial.println(output.torque_roll);

Serial.print("Thrust: ");
Serial.println(output.thrust);
```

---

## ⚙️ Parámetros a Ajustar

### Ganancias del Controlador

```cpp
// En setup(), después de coordinador.inicializar()

QuaternionAttitudeController* ctrl = coordinador.controller();
ctrl->set_attitude_gain(4.5f);   // Aumentar para respuesta más rápida
                                 // Reducir para respuesta más suave
ctrl->set_rate_gain(0.15f);      // Opcional, futuro control de rates
ctrl->set_thrust_max(1.0f);      // Empuje máximo normalizado
```

### Limites de Actuadores

```cpp
// En setup(), después de coordinador.inicializar()

PSO_Allocator* alloc = coordinador.allocator();
alloc->set_motor_speed_limits(1000.0f, 8000.0f);   // RPM
alloc->set_servo_angle_limits(-0.349f, 0.349f);    // ±20 grados
```

---

## 📊 Frecuencias de Ejecución

| Componente | Frecuencia | Período |
|-----------|-----------|---------|
| Sensores | 100 Hz | 10 ms |
| Estimador | 100 Hz | 10 ms |
| Controlador | 50 Hz | 20 ms |
| Allocator | 50 Hz | 20 ms |

**Nota:** El sistema ejecuta internamente cada componente en su frecuencia. No necesitas hacer nada especial.

---

## 🔧 Debugging Básico

### Los sensores no se inicializan
```
❌ [SCHED] ERROR: Sensores no inicializaron
```
**Solución:**
- Verificar pines IMU (SPI) y Barómetro (I2C PB8/PB9)
- Revisar conexiones

### El estimador da ceros
```
❌ [EST] R:0.000 P:0.000 Y:0.000
```
**Solución:**
- Esperar a que los sensores estabilicen (5-10 segundos)
- Revisar que hay datos en sensores

### Controlador no genera torques
```
❌ [CTL] TR:0.000 TP:0.000 TY:0.000 TH:0.500
```
**Solución:**
- Cambiar setpoint: `coordinador.set_attitude_setpoint(0.1, 0, 0, 0.5)`
- Reducir ganancia: `ctrl->set_attitude_gain(2.0f)`

### Allocator no genera RPM válidas
```
❌ [ALLOC] M[0,0,0] S[0,0,0]
```
**Solución:**
- Verificar que torques en [CTL] no son ceros
- Revisar límites de motor: `set_motor_speed_limits()`

---

## 💾 Guardar Setpoints en Ejecución

Para automatizar secuencias más complejas:

```cpp
// Agregar estructura en main.cpp
struct MissionWaypoint {
    float roll, pitch, yaw, thrust;
    uint32_t duration_ms;
};

// Implementar en loop()
static uint32_t wp_start = 0;
static uint8_t wp_idx = 0;

if (millis() - wp_start >= missions[wp_idx].duration_ms) {
    wp_idx++;
    wp_start = millis();
    if (wp_idx >= NUM_MISSIONS) wp_idx = 0;
}

coordinador.set_attitude_setpoint(
    missions[wp_idx].roll,
    missions[wp_idx].pitch,
    missions[wp_idx].yaw,
    missions[wp_idx].thrust
);
```

---

## 📈 Telemetría Serial

Cada 500ms se imprime:

```
[EST] R:0.001 P:0.002 Y:-0.001
      ├─ Roll actual (rad)
      ├─ Pitch actual (rad)
      └─ Yaw actual (rad)

[CTL] TR:0.021 TP:0.045 TY:-0.003 TH:0.500
      ├─ Torque Roll (Nm)
      ├─ Torque Pitch (Nm)
      ├─ Torque Yaw (Nm)
      └─ Thrust (normalizado 0-1)

[ALLOC] M[2100,2105,2098] S[0.002,0.003,0.001]
        ├─ Motor 0 RPM
        ├─ Motor 1 RPM
        ├─ Motor 2 RPM
        ├─ Servo 0 ángulo (rad)
        ├─ Servo 1 ángulo (rad)
        └─ Servo 2 ángulo (rad)

[SP] HOVER
     └─ Etiqueta del setpoint actual
```

---

## 🎯 Ejemplos de Control

### Ejemplo 1: Hover Estable
```cpp
coordinador.set_attitude_setpoint(
    0.0f,    // Roll 0°
    0.0f,    // Pitch 0°
    0.0f,    // Yaw 0°
    0.5f     // 50% empuje
);
```

### Ejemplo 2: Giro Suave
```cpp
coordinador.set_attitude_setpoint(
    0.0f,      // Roll 0°
    0.0f,      // Pitch 0°
    0.1f,      // Yaw +5.7°
    0.5f       // Mantener empuje
);
```

### Ejemplo 3: Avance Suave
```cpp
coordinador.set_attitude_setpoint(
    0.0f,      // Roll 0° (mantener nivel)
    0.05f,     // Pitch +2.9° (avanzar)
    0.0f,      // Yaw 0°
    0.6f       // Aumentar empuje
);
```

### Ejemplo 4: Maniobra Acrobática (Prueba en simulador primero!)
```cpp
coordinador.set_attitude_setpoint(
    0.3f,      // Roll +17° (banco)
    0.1f,      // Pitch +5.7° (avanza)
    0.2f,      // Yaw +11° (gira)
    0.8f       // Empuje alto
);
```

---

## ⚠️ Límites de Seguridad

| Parámetro | Límite | Unidad |
|-----------|--------|--------|
| Máx Roll/Pitch | ±1.57 | radianes (±90°) |
| Máx Yaw | 2π | radianes (360°) |
| Máx Thrust | 1.0 | normalizado |
| Máx Torque | 0.5 | Nm |
| RPM Motor | 1000-8000 | RPM |
| Ángulo Servo | ±0.349 | radianes (±20°) |

---

## 🔗 Referencia Rápida de Archivos

| Archivo | Propósito | Modificar? |
|---------|-----------|-----------|
| `src/main.cpp` | Punto de entrada | ✏️ Sí |
| `src/scheduler.cpp` | Orquestación | 🔒 No |
| `src/quaternion_attitude_controller.cpp` | Control | 🔒 No |
| `src/pso_allocator.cpp` | Allocator | 🔒 Sí (PSO) |
| `include/` | Headers | 🔒 No |

---

## ✅ Checklist Antes de Volar

- [ ] Compilación exitosa
- [ ] Monitor serial muestra `[EST]` con valores no cero
- [ ] Monitor serial muestra `[CTL]` con torques
- [ ] Monitor serial muestra `[ALLOC]` con RPM en rango válido
- [ ] Setpoint cambia correctamente con `set_attitude_setpoint()`
- [ ] Ganancias ajustadas según tu aeronave
- [ ] Sensores calibrados (IMU offset, brújula)

---

**¡Listo para usar!** 🚀

Próximos pasos:
1. Subir a hardware
2. Verificar salidas en serial
3. Tunear ganancias
4. Pruebas de control
