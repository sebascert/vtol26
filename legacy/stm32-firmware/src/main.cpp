#include <Arduino.h>
#include <Wire.h>
#include <stm32f4xx.h>
#include "scheduler.hpp"
#include "hal_dps310.hpp"

using namespace stallion;

CoordinadorDeTareas coordinador;
HalDPS310 halBaro;

// ============================================================================
// CONTROL DE SETPOINTS PARA PRUEBAS
// ============================================================================

struct TestSetpoint {
    float roll_rad;
    float pitch_rad;
    float yaw_rad;
    float thrust;
    uint32_t duration_ms;
    const char* label;
};

// Secuencia de prueba: hover → pitcheo → recuperación
const TestSetpoint test_sequence[] = {
    {0.0f, 0.0f, 0.0f, 0.5f, 3000, "HOVER"},
    {0.1f, 0.0f, 0.0f, 0.5f, 2000, "ROLL +"},
    {-0.1f, 0.0f, 0.0f, 0.5f, 2000, "ROLL -"},
    {0.0f, 0.1f, 0.0f, 0.5f, 2000, "PITCH +"},
    {0.0f, 0.0f, 0.0f, 0.5f, 2000, "HOVER"},
};

uint8_t test_idx = 0;
uint32_t test_start_ms = 0;

void update_test_setpoint() {
    if (test_idx >= sizeof(test_sequence) / sizeof(TestSetpoint)) {
        test_idx = 0;
    }

    const TestSetpoint& sp = test_sequence[test_idx];
    uint32_t elapsed = millis() - test_start_ms;

    if (elapsed >= sp.duration_ms) {
        test_idx++;
        test_start_ms = millis();
        if (test_idx >= sizeof(test_sequence) / sizeof(TestSetpoint)) {
            test_idx = 0;
        }
    }

    const TestSetpoint& current = test_sequence[test_idx];
    coordinador.set_attitude_setpoint(current.roll_rad, current.pitch_rad,
                                      current.yaw_rad, current.thrust);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}
    delay(2000);

    Serial.println("=== CODIGO NUEVOOO uwo ===");
    Serial.println("BOOT OK");
    
    // Inicializar barómetro HAL
    if (!halBaro.init()) {
        Serial.println("[BARO] Error inicializando");
    } else {
        Serial.println("[BARO] OK");
    }

    // Inicializar arquitectura de control
    coordinador.inicializar(micros());

    test_start_ms = millis();
    Serial.println("\nINIT OK - Sistema listo para control\n");
}

void loop() {
    uint64_t ahora = micros();

    // Ejecutar arquitectura
    coordinador.ejecutar(ahora);

    // Diagnostico cada 500ms
    static uint32_t t_diag = 0;
    if (millis() - t_diag >= 500) {
        t_diag = millis();
        coordinador.diagnostico();
    }

    // Actualizar setpoint de prueba
    static uint32_t t_sp = 0;
    if (millis() - t_sp >= 100) {
        t_sp = millis();
        update_test_setpoint();
        
        // Mostrar setpoint actual
        const TestSetpoint& current = test_sequence[test_idx];
        Serial.print("[SP] ");
        Serial.println(current.label);
    }
}