#include <Arduino.h>
#include "scheduler.hpp"

stallion::CoordinadorDeTareas coordinador;

void setup() {
// Inicializar comunicación serie
  Serial.begin(115200);
  // Inicializar el coordinador al inicio
  coordinador.inicializar(0);
}

void loop() {
  // Ejecutar el ciclo de control periódicamente
  uint64_t ahora_us = micros();  // Microsegundos desde inicio
  coordinador.ejecutar(ahora_us);
  // Diagnóstico cada 1 segundo (ejemplo)
  static uint64_t last_diag = 0;
  if (ahora_us - last_diag > 1'000'000) {
    coordinador.diagnostico();  // Nuevo método para imprimir estado
    last_diag = ahora_us;
  }
  
  delay(10);  // 100 Hz
}