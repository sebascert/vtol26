# MAVLink Implementation Guide (Opcional)

## Resumen
Esta guía es solo si eliges **Opción 2** (MAVLink + Pixhawk externo). Si implementas **Opción 1** (integración directa), puedes saltar esto.

---

## Parte 1: Descarga de MAVLink

### Paso 1: Clonar MAVLink
```bash
git clone https://github.com/mavlink/mavlink.git
cd mavlink
```

### Paso 2: Generar headers para C
```bash
python -m pymavlink.tools.mavgen \
    --lang=c \
    --output=generated/include \
    message_definitions/v1.0/common.xml
```

### Paso 3: Copiar a tu proyecto
```bash
# En tu proyecto
mkdir -p include/mavlink
cp -r generated/include/common include/mavlink/
```

### Paso 4: Agregar a platformio.ini
```ini
[env:f405_wing_v2]
platform = ststm32
board = genericSTM32F405RG
framework = arduino

build_flags =
    -I include/mavlink/common
    -D MAVLINK_CHECK_ALWAYS
    -D MAVLINK_NEED_BYTE_SWAP=0
```

---

## Parte 2: Implementación Ligera de MAVLink

### Archivo: `include/mavlink_manager.hpp`

```cpp
#pragma once

#include <Arduino.h>
#include <cstring>
#include "mavlink/common/mavlink.h"
#include "module_data_types.hpp"

/**
 * @class MAVLinkManager
 * @brief Gestor ligero de MAVLink para comunicación con GCS/Pixhawk
 * 
 * Características:
 * - Envío de telemetría a 10-50 Hz
 * - Recepción de comandos
 * - Buffer circular para mensajes
 * - Bajo overhead de CPU
 */
class MAVLinkManager {
public:
    // Configuración
    struct Config {
        HardwareSerial* serial;      // Serial puerto (Serial1, etc)
        uint32_t baud_rate;          // 57600, 115200, etc
        uint8_t system_id;           // 1-255
        uint8_t component_id;        // 1-255 (MAV_COMP_ID_AUTOPILOT1 = 1)
        uint16_t heartbeat_hz;       // 1-10 Hz
    };
    
    MAVLinkManager();
    
    /**
     * Inicializar MAVLink
     */
    bool init(const Config& config);
    
    /**
     * Actualizar (llamar en loop principal)
     */
    void update();
    
    /**
     * Enviar telemetría
     */
    void send_heartbeat();
    void send_attitude(
        float roll, float pitch, float yaw,
        float roll_rate, float pitch_rate, float yaw_rate,
        uint32_t time_boot_ms
    );
    void send_battery(
        float voltage, float current,
        int8_t battery_remaining
    );
    void send_gps(
        int32_t lat, int32_t lon, int32_t alt,
        uint16_t hdop, uint16_t vdop,
        uint16_t ground_speed, uint16_t heading,
        uint8_t satellites_visible
    );
    void send_raw_imu(
        const float accel[3],
        const float gyro[3],
        int16_t xmag, int16_t ymag, int16_t zmag
    );
    
    /**
     * Getters para datos recibidos
     */
    bool get_attitude_target(
        float& roll_target, float& pitch_target, float& yaw_target,
        float& roll_rate_target, float& pitch_rate_target, float& yaw_rate_target,
        float& thrust_target
    );
    
    bool get_manual_control(
        int16_t& roll, int16_t& pitch, int16_t& throttle, int16_t& yaw,
        uint16_t& buttons
    );
    
    /**
     * Status
     */
    bool is_connected() const;
    uint32_t heartbeat_count() const { return _heartbeat_count; }
    uint32_t received_message_count() const { return _received_count; }
    
private:
    Config _config;
    HardwareSerial* _serial;
    
    // Buffer circular
    static constexpr uint16_t BUFFER_SIZE = 256;
    uint8_t _rx_buffer[BUFFER_SIZE];
    uint16_t _rx_index;
    
    // Contadores
    uint32_t _heartbeat_count;
    uint32_t _received_count;
    uint32_t _last_heartbeat_ms;
    uint32_t _last_attitude_ms;
    
    // Estado de conexión
    bool _connected;
    uint32_t _last_message_ms;
    
    // Datos recibidos
    mavlink_attitude_target_t _attitude_target;
    mavlink_manual_control_t _manual_control;
    
    // Métodos privados
    void _send_message(const mavlink_message_t* msg);
    void _parse_incoming();
    void _handle_message(const mavlink_message_t* msg);
};

#endif // MAVLINK_MANAGER_HPP
```

### Archivo: `src/mavlink_manager.cpp`

```cpp
#include "mavlink_manager.hpp"

MAVLinkManager::MAVLinkManager()
    : _serial(nullptr),
      _rx_index(0),
      _heartbeat_count(0),
      _received_count(0),
      _last_heartbeat_ms(0),
      _last_attitude_ms(0),
      _connected(false),
      _last_message_ms(0) {
    
    memset(_rx_buffer, 0, BUFFER_SIZE);
    memset(&_attitude_target, 0, sizeof(_attitude_target));
    memset(&_manual_control, 0, sizeof(_manual_control));
}

bool MAVLinkManager::init(const Config& config) {
    _config = config;
    _serial = config.serial;
    
    if (!_serial) {
        Serial.println("[MAVLink] Error: No serial port provided");
        return false;
    }
    
    _serial->begin(_config.baud_rate);
    Serial.print("[MAVLink] Initialized at ");
    Serial.print(_config.baud_rate);
    Serial.println(" baud");
    
    return true;
}

void MAVLinkManager::update() {
    // Parsear datos entrantes
    _parse_incoming();
    
    // Enviar heartbeat periódicamente
    uint32_t now = millis();
    if (now - _last_heartbeat_ms > (1000 / _config.heartbeat_hz)) {
        send_heartbeat();
        _last_heartbeat_ms = now;
    }
    
    // Detectar desconexión
    if (now - _last_message_ms > 5000) {
        _connected = false;
    }
}

void MAVLinkManager::_send_message(const mavlink_message_t* msg) {
    uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buffer, msg);
    
    _serial->write(buffer, len);
}

void MAVLinkManager::_parse_incoming() {
    mavlink_message_t msg;
    mavlink_status_t status;
    
    while (_serial->available()) {
        uint8_t byte = _serial->read();
        
        // Parsear byte individual
        if (mavlink_parse_char(MAVLINK_COMM_0, byte, &msg, &status)) {
            _handle_message(&msg);
            _received_count++;
            _last_message_ms = millis();
            _connected = true;
        }
    }
}

void MAVLinkManager::_handle_message(const mavlink_message_t* msg) {
    switch (msg->msgid) {
        case MAVLINK_MSG_ID_ATTITUDE_TARGET: {
            mavlink_msg_attitude_target_decode(msg, &_attitude_target);
            break;
        }
        case MAVLINK_MSG_ID_MANUAL_CONTROL: {
            mavlink_msg_manual_control_decode(msg, &_manual_control);
            break;
        }
        case MAVLINK_MSG_ID_HEARTBEAT: {
            // Echo o respuesta a heartbeat remoto
            break;
        }
        default:
            // Mensaje no manejado
            break;
    }
}

void MAVLinkManager::send_heartbeat() {
    mavlink_message_t msg;
    
    mavlink_msg_heartbeat_pack(
        _config.system_id,
        _config.component_id,
        &msg,
        MAV_TYPE_QUADROTOR,         // Tipo de vehículo
        MAV_AUTOPILOT_GENERIC,       // Sistema autopilot
        MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_STABILIZE_ENABLED,
        0,                           // custom_mode
        MAV_STATE_ACTIVE             // estado
    );
    
    _send_message(&msg);
    _heartbeat_count++;
}

void MAVLinkManager::send_attitude(
    float roll, float pitch, float yaw,
    float roll_rate, float pitch_rate, float yaw_rate,
    uint32_t time_boot_ms) {
    
    mavlink_message_t msg;
    
    mavlink_msg_attitude_pack(
        _config.system_id,
        _config.component_id,
        &msg,
        time_boot_ms,           // milisegundos desde boot
        roll,                   // rad
        pitch,                  // rad
        yaw,                    // rad
        roll_rate,              // rad/s
        pitch_rate,             // rad/s
        yaw_rate                // rad/s
    );
    
    _send_message(&msg);
}

void MAVLinkManager::send_battery(
    float voltage, float current,
    int8_t battery_remaining) {
    
    mavlink_message_t msg;
    
    mavlink_msg_battery_status_pack(
        _config.system_id,
        _config.component_id,
        &msg,
        0,                                  // battery_function (MAV_BATTERY_FUNCTION_ALL)
        MAV_BATTERY_TYPE_LIPO,              // type
        (int16_t)(voltage * 1000.0f),       // voltaje en mV
        (int16_t)(current * 100.0f),        // corriente en 10*mA
        battery_remaining,                   // %
        0,                                  // temperature
        0,                                  // charge_state
        nullptr,                            // voltages
        0, 0, 0                             // current_consumed, energy_consumed
    );
    
    _send_message(&msg);
}

void MAVLinkManager::send_gps(
    int32_t lat, int32_t lon, int32_t alt,
    uint16_t hdop, uint16_t vdop,
    uint16_t ground_speed, uint16_t heading,
    uint8_t satellites_visible) {
    
    mavlink_message_t msg;
    
    mavlink_msg_gps_raw_int_pack(
        _config.system_id,
        _config.component_id,
        &msg,
        micros(),                   // time_usec
        GPS_FIX_TYPE_3D_FIX,        // fix_type
        lat,                        // lat (1e7)
        lon,                        // lon (1e7)
        alt,                        // alt (mm)
        hdop,                       // hdop (cm)
        vdop,                       // vdop (cm)
        ground_speed,               // vel (cm/s)
        heading,                    // cog (cdeg)
        satellites_visible          // satellites_visible
    );
    
    _send_message(&msg);
}

void MAVLinkManager::send_raw_imu(
    const float accel[3],
    const float gyro[3],
    int16_t xmag, int16_t ymag, int16_t zmag) {
    
    mavlink_message_t msg;
    
    // Convertir float a int16 (m/s² -> 1000*m/s²)
    int16_t xacc = (int16_t)(accel[0] * 1000.0f);
    int16_t yacc = (int16_t)(accel[1] * 1000.0f);
    int16_t zacc = (int16_t)(accel[2] * 1000.0f);
    
    // Convertir float a int16 (rad/s -> 1000*rad/s)
    int16_t xgyro = (int16_t)(gyro[0] * 1000.0f);
    int16_t ygyro = (int16_t)(gyro[1] * 1000.0f);
    int16_t zgyro = (int16_t)(gyro[2] * 1000.0f);
    
    mavlink_msg_raw_imu_pack(
        _config.system_id,
        _config.component_id,
        &msg,
        micros(),           // time_usec
        xacc, yacc, zacc,   // accel
        xgyro, ygyro, zgyro,// gyro
        xmag, ymag, zmag    // mag
    );
    
    _send_message(&msg);
}

bool MAVLinkManager::get_attitude_target(
    float& roll_target, float& pitch_target, float& yaw_target,
    float& roll_rate_target, float& pitch_rate_target, float& yaw_rate_target,
    float& thrust_target) {
    
    roll_target = _attitude_target.roll;
    pitch_target = _attitude_target.pitch;
    yaw_target = _attitude_target.yaw;
    roll_rate_target = _attitude_target.body_roll_rate;
    pitch_rate_target = _attitude_target.body_pitch_rate;
    yaw_rate_target = _attitude_target.body_yaw_rate;
    thrust_target = _attitude_target.thrust;
    
    return _connected;
}

bool MAVLinkManager::get_manual_control(
    int16_t& roll, int16_t& pitch, int16_t& throttle, int16_t& yaw,
    uint16_t& buttons) {
    
    roll = _manual_control.roll;       // -1000 a 1000
    pitch = _manual_control.pitch;     // -1000 a 1000
    throttle = _manual_control.throttle; // 0 a 1000
    yaw = _manual_control.yaw;         // -1000 a 1000
    buttons = _manual_control.buttons;
    
    return _connected;
}

bool MAVLinkManager::is_connected() const {
    return _connected;
}
```

---

## Parte 3: Uso en tu Código

### Integración en `main.cpp`

```cpp
#include <Arduino.h>
#include "scheduler.hpp"
#include "mavlink_manager.hpp"

using namespace stallion;

CoordinadorDeTareas coordinador;
MAVLinkManager mavlink;

void setup() {
    Serial.begin(115200);
    while (!Serial) {} 
    delay(2000);

    // Inicializar MAVLink
    MAVLinkManager::Config mavlink_config;
    mavlink_config.serial = &Serial1;          // UART 1 para comunicación
    mavlink_config.baud_rate = 57600;
    mavlink_config.system_id = 1;
    mavlink_config.component_id = MAV_COMP_ID_AUTOPILOT;
    mavlink_config.heartbeat_hz = 1;
    
    if (!mavlink.init(mavlink_config)) {
        Serial.println("MAVLink init failed");
        while(1);
    }
    
    coordinador.inicializar(micros());
    Serial.println("System ready with MAVLink");
}

void loop() {
    uint64_t ahora = micros();
    
    // Actualizar MAVLink (recibir comandos)
    mavlink.update();
    
    // Ejecutar scheduler
    coordinador.ejecutar(ahora);
    
    // Enviar telemetría periódicamente
    static uint32_t t_telem = 0;
    if (millis() - t_telem >= 100) {  // 10 Hz
        t_telem = millis();
        
        // Obtener estado actual del coordinador
        // (asumiendo que tienes métodos getter)
        float roll, pitch, yaw;
        // roll = coordinador.getAttitude().roll;
        // ... etc
        
        // Enviar telemetría
        // mavlink.send_attitude(roll, pitch, yaw, 0, 0, 0, millis());
        // mavlink.send_battery(12.5f, 5.3f, 75);
    }
}
```

---

## Parte 4: Validación

### Verificar conexión con QGroundControl

1. **Descargar QGroundControl**:
   - https://qgroundcontrol.com/

2. **Conectar**:
   - Puerto serial: COM# donde conectes UART1
   - Baud rate: 57600

3. **Validar**:
   - Ver heartbeat del sistema
   - Ver actitud en tiempo real
   - Enviar comandos de actitud

---

## Parte 5: Limitaciones de MAVLink en STM32F405

| Aspecto | Valor | Implicación |
|--------|-------|------------|
| Tasa máxima | ~100 Hz (5 mensajes/ciclo) | Latencia ~10 ms |
| Ancho de banda | ~57600 baud | ~7 KB/s |
| Buffer | 256 bytes | Suficiente para telemetría |
| Overhead CPU | ~2-5% | Mínimo impacto |

---

## Notas Finales

- **Esta implementación es opcional**: Solo si usas Opción 2 (MAVLink)
- **Para Opción 1**: No necesitas MAVLink (integración directa es más simple)
- **Testing**: Usa QGroundControl para validar comunicación
- **Debugging**: Habilita logs de MAVLink en QGC para diagnosticar

---

**Versión**: 1.0
**Estado**: Opcional (depende de arquitectura elegida)

