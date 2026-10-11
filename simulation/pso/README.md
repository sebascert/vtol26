# Entorno de Pruebas PSO para VTOL

## Descripción

Este directorio contiene el entorno de pruebas para desarrollar un algoritmo de **Particle Swarm Optimization (PSO)** destinado a definir la configuración óptima de actuadores del dron VTOL.

## Vector de Estado (D)

El estado del sistema se representa con un vector de magnitud **9**:

| Índice | Nombre | Descripción |
|--------|--------|-------------|
| 0-2 | ω_I, ω_D, ω_T | Velocidades angulares de los rotores (izquierdo, derecho, trasero) |
| 3-4 | ε_I, ε_D | Ángulos de los rotores basculantes delanteros (izquierdo, derecho) |
| 5-6 | γ_I, γ_D | Ángulos de los ruddervators (izquierdo, derecho) |
| 7-8 | β_I, β_D | Ángulos de los alerones (izquierdo, derecho) |

## Parámetros de PSO

### K_AM (Área de Muestreo)

Define la magnitud del área de búsqueda alrededor del estado actual.

Para cada elemento `D[i]` del vector, el rango de muestreo es:
```
[D[i] - K_AM, D[i] + K_AM]
```

Esto crea un hipercubo de 9 dimensiones donde se inicializarán las partículas del PSO.

## Archivos

- **pso_state.hpp**: Definiciones de constantes, índices y estructura del estado
- **pso_environment.cpp**: Implementación del entorno de pruebas (generación de partículas, visualización)

## Funcionalidades Implementadas

### ✅ Vector de Estado D (9 dimensiones)
- ω_I, ω_D, ω_T: Velocidades angulares de rotores
- ε_I, ε_D: Ángulos rotores basculantes delanteros  
- γ_I, γ_D: Ángulos ruddervators
- β_I, β_D: Ángulos alerones
### ✅ Modelo físico básico de fuerzas y momentos
- Se usa una representación física de fuerzas aerodinámicas y de empuje
- El cálculo sigue la estructura de `sim/simulink/fuerzas_v4.m`
- Se integra posición y orientación con masa, gravedad e inercia

### ✅ Objetivo en espacio de pose (pose-space)
- El PSO sigue optimizando en espacio de actuadores (9D), pero ahora puede minimizar error contra un `target_pose` en `config.json`:
  - `target_pose.position = [x, y, z]`
  - `target_pose.orientation = [roll, pitch, yaw]`
- El costo incorpora:
  - `k_pos * ||pos_pred - pos_target||^2`
  - `k_att * error_cuaternion^2`
- Si `target_pose` no está definido, se conserva el comportamiento legado basado en actuadores.
### ✅ Área de Muestreo con K_AM dinámico
- Rangos: [D[i] - K_AM, D[i] + K_AM] para cada dimensión
- K_AM es configurable desde `config.json` y se contrae por ciclo externo (`k_am_contraction`)

### ✅ Generación de Partículas Iniciales
- Partículas aleatorias dentro de los rangos de muestreo
- Configurable número de partículas

### ✅ Visualización de Datos
- **Exportación JSON en tiempo real**: Archivo `pso_data.json` (sobrescrito por iteración)
- **Visualización ASCII 3D**: Muestra **3 dimensiones seleccionadas aleatoriamente** de las 9 disponibles
- **Script Python**: `visualize_particles.py` con panel 3D + convergencia de fitness y tema oscuro
- **Meta-iteraciones**: historial de centros de búsqueda (`center_history`) para ver la migración del área

## Compilación y Ejecución

Para compilar:
```bash
g++ -std=c++17 -o pso_environment pso_environment.cpp
```

Para ejecutar:
```bash
./pso_environment
```

## Salida del Programa

El programa genera:
1. **Estado actual** del dron VTOL
2. **Rangos de muestreo** para cada actuador
3. **Partículas iniciales** del PSO
4. **Archivo JSON** (`pso_data.json`) con partículas, gBest, fitness e historial de centros
   - Incluye además `target_pose` y `predicted_gbest_pose`
5. **Visualización ASCII** de 3 dimensiones (aleatorias)
6. **Script Python** para gráficos 3D

## Visualización

Para visualizar las partículas en 3D:
```bash
python visualize_particles.py
```