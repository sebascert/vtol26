#include <Arduino.h>
#include "scheduler.hpp"

using namespace stallion;

CoordinadorDeTareas coordinador;

void setup() {
    // LED
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    
    // Serial
    Serial.begin(115200);
    delay(2000);
    
    Serial.println("\n╔════════════════════════════════════════════════════════╗");
    Serial.println("║  VTOL F405-WING-V2 FLIGHT CONTROLLER TEST            ║");
    Serial.println("║  Sensores Integrados: ICM42688-P, DPS310             ║");
    Serial.println("║  Sensores Externos:  Neo-6M GPS, MPXV7002DP Pitot    ║");
    Serial.println("╚════════════════════════════════════════════════════════╝\n");
    
    // Inicializar coordinador (que inicializa SensorManager)
    coordinador.inicializar(0);
    
    digitalWrite(LED_PIN, LOW);
    Serial.println("\n[SETUP] ✓ Listo para operar\n");
}

void loop() {
    uint64_t now_us = micros();
    
    // Ejecutar coordinador
    coordinador.ejecutar(now_us);
    
    // Debug output cada 500ms
    static uint32_t last_debug = 0;
    if (millis() - last_debug > 500) {
        last_debug = millis();
        
        // Parpadear LED
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        
        // Aquí iría tu código de debug si lo necesitas
    }
    
    delay(1);  // Pequeño delay para evitar saturar
}
