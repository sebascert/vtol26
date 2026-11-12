# Stallion VTOL

Vehiculo aereo no tripulado (UAV) de alto rendimiento con despegue y aterrizaje
vertical (VTOL). La aeronave utiliza una configuracion de **tricoptero** para
vuelo estacionario y transiciona a vuelo de **ala fija** para crucero
eficiente.

**Objetivo principal:** asignar el empuje y los momentos entre los actuadores
(motores + servos) de forma optima durante las fases de **transicion**,
mediante un algoritmo **PSO (Particle Swarm Optimization)** que corre en un
companion computer y se comunica por MAVLink con un piloto automatico PX4.

---

## Arquitectura vigente: companion computer + Pixhawk/PX4

- **Piloto automatico:** Pixhawk 6C corriendo **PX4** (stock, sin modificar).
  Resuelve la estabilizacion base, la fusion de sensores interna (IMU/baro/mag
  propios de la placa) y los modos de vuelo estandar de PX4.
- **Companion computer:** Raspberry Pi 4, conectada al Pixhawk 6C. Ejecuta el
  algoritmo de **asignacion de control por micro-PSO** (optimiza velocidades de
  motor y angulos de servo para un vector de 9 actuadores, ver
  `sim/PSO/README.md`) y le inyecta los comandos resultantes a PX4 por
  **MAVLink**.
- El PSO puede optimizar en espacio de actuadores puro o contra una **pose
  objetivo** (posicion + orientacion en cuaterniones), segun
  `sim/PSO/config.json` (`target_pose`).

> Nota de historia: el proyecto tuvo dos direcciones anteriores que ya **no
> aplican** y fueron retiradas del repositorio (2026-08): (1) un firmware C++
> propio para un controlador STM32F405 (SpeedyBee F405 Wing V2), completamente
> standalone y sin PX4; (2) una propuesta de correr el PSO como **modulo
> interno de PX4** (uORB) compilado en la misma placa de vuelo. Ambas quedaron
> descartadas a favor de la arquitectura companion computer + Pixhawk descrita
> arriba.

## Estado actual

## La implementacion real del companion computer (enlace MAVLink + PSO corriendo en la Raspberry Pi) existe hoy solo en esa Raspberry Pi; lo que hay aqui es el material de soporte: el algoritmo PSO (`sim/PSO/`), el modelo dinamico (`sim/simulink/`, `docs/modeling/`), el CAD del airframe (`hardware/`) y la bibliografia (`docs/references/`).

## Hardware

| Area                   | Especificacion                                                                                                                                                |
| ---------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Piloto automatico**  | Pixhawk 6C, PX4 stock                                                                                                                                         |
| **Companion computer** | Raspberry Pi 4                                                                                                                                                |
| **Aeronave**           | Fabricacion hibrida: LW-PLA (espumado activo) y PLA/PETG estandar para soportes de alto esfuerzo                                                              |
| **Propulsion**         | Motores brushless de 1750 KV, helices bipala de 7.5x5                                                                                                         |
| **Alimentacion**       | Bateria de 12 V (3S)                                                                                                                                          |
| **Sensores**           | IMU, barometro y magnetometro **internos del Pixhawk 6C** (no se usan los sensores externos ICM42688/DPS310/QMC5883L del diseno anterior); GPS externo **P9** |

---

## Alcance y entregables

1. **Companion computer:** portar/formalizar el algoritmo PSO
   (`sim/PSO/pso_environment.cpp`) a un servicio en la Raspberry Pi que hable
   MAVLink con PX4, bajo control de versiones.
2. **Integracion mecanica:** montaje y **compensacion de offsets** del Pixhawk
   respecto al **centro de gravedad (CG)** del vehiculo.
3. **Simulacion y validacion:** pruebas **hardware-in-the-loop (HIL)**
   enfocadas en la logica de transicion, apoyadas en el modelo de
   `sim/simulink/`.
4. **Datos:** analisis de telemetria post-vuelo (via MAVLink/PX4) para comparar
   el comportamiento medido contra el modelo analitico / de simulacion.

---

## Estructura del repositorio

- `sim/`: entorno de simulacion y del algoritmo de asignacion.
  - `PSO/`: entorno de pruebas del algoritmo PSO (vector de estado de 9
    actuadores, costo en espacio de pose, visualizacion). Ver
    `sim/PSO/README.md`. Es la referencia a portar al companion computer.
  - `simulink/`: modelo dinamico del vehiculo (fuerzas y momentos,
    cuaterniones). Ver `docs/modeling/README.md`.
- `hardware/`: activos fisicos del proyecto ya incorporados al repo.
  - `cad/mounts/`: piezas STEP de monturas de motor y modificaciones de boom.
  - Nota: el resto del material de hardware (manuales, BOM, diagramas de
    cableado, piezas STL/STEP de fuselaje y alas por version V1/V2) vive por
    ahora en las carpetas exportadas de Drive fuera de este repo; pendiente de
    indexar/incorporar.
- `docs/`: documentacion tecnica del sistema.
  - `modeling/`: modelo dinamico y su implementacion simbolica.
  - `references/`: bibliografia usada como inspiracion tecnica.
  - Categorias adicionales (arquitectura del companion computer, integracion
    MAVLink, HIL, failsafe) se agregaran como carpetas cuando tengan contenido
    real.
- `tools/`: utilidades de analisis y post-procesamiento
  (`plot_allocator_csv.py`).
- `these_utc_hernan_abaunza_gonzalez.pdf`: tesis de referencia sobre modelado y
  control de UAV con cuaterniones (ver `docs/references/README.md`).
- `VTOL_SIMULINK.slx`: modelo Simulink del vehiculo.

> `PX4-Autopilot-main/` es una copia de referencia sin modificar del codigo
> fuente de PX4, util para consultar topicos uORB, mensajes MAVLink y la
> estructura interna de PX4 al integrar el companion computer. No se compila
> como parte de este repositorio (el Pixhawk 6C corre PX4 stock).

---

## Documentacion minima recomendada

- `sim/PSO/README.md` (algoritmo PSO: vector de estado, parametros, ejecucion).
- `docs/modeling/README.md` (modelo dinamico del vehiculo).
- `docs/references/README.md` (bibliografia de respaldo).
