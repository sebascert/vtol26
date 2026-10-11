# Pruebas por modulo (arranque rapido)

Esta carpeta separa pruebas para aislar fallas rapido antes de integrar todo el firmware.

## Estructura

- `control/`: pruebas del modelo de fuerzas y control.
- `estimation/`: pruebas de estimador (sensores sinteticos, validez y drift).
- `allocator/`: pruebas de asignador/QP (saturaciones y residuales).
- `failsafe/`: pruebas de protocolos de emergencia.
- `comms_hil/`: pruebas de enlace HIL por serial (parseo, timeout y secuencia).
- `logging/`: pruebas de logger en buffer/SD.
- `fixtures/`: entradas fijas reutilizables.

## Criterio de uso

1. Ejecutar primero pruebas del modulo individual.
2. Solo integrar modulo cuando su prueba base pase.
3. Mantener cada prueba corta (segundos) y con pass/fail claro.
