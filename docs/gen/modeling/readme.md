# Notas de modelado (VTOL)

Este espacio documenta el modelo dinamico del vehiculo y su implementacion simbolica.

## Estado actual

- Implementacion base en `legacy/simulation/simulink/fuerzas_v4.m`.
- Fuerzas aerodinamicas y de propulsion en marco cuerpo mediante rotaciones por cuaternion.
- Momentos totales modelados como:
  - suma de contribuciones `r_i x F_i` para cada superficie/actuador,
  - mas un termino de momentos directos `M_directo` (reaccion de rotor u otros efectos no representados por brazo de palanca).
- Plantilla de planta Simulink disponible:
  - `legacy/simulation/simulink/construir_planta_vtol.m`
  - `legacy/simulation/simulink/modelo_fuerzas_momentos_vtol.m`
  - `legacy/simulation/simulink/crear_parametros_vtol.m`
  - `legacy/simulation/simulink/simular_y_graficar_vtol.m`
  - `legacy/simulation/simulink/dashboard_vtol.m`

## Visualizacion rapida

Para correr una simulacion base y abrir graficas automaticamente:

- `simular_y_graficar_vtol`

Puedes pasar entrada y tiempo:

- `simular_y_graficar_vtol('planta_vtol', [m*g; 0; 0; 0; 0; 0; 0; 0; 0], 20)`

Graficas incluidas:

- Posicion y velocidad inercial.
- Cuaternion y velocidad angular.
- Fuerzas y momentos en marco cuerpo.

## Dashboard interactivo

Para abrir un panel con sliders de entradas y boton de simulacion:

- `dashboard_vtol`

Incluye:

- Ajuste de las 9 entradas del modelo.
- Presets rapidos (`hover` y `cero`).
- Ejecucion de simulacion y generacion de figuras con un clic.

## Convenciones recomendadas

- Mantener todo en unidades SI.
- Expresar posiciones `r_i` respecto al centro de gravedad (CG), en marco cuerpo.
- Fijar convencion de ejes y signos en un solo documento antes de identificar parametros.
- Usar versiones suaves de funciones no diferenciables en modelos para linealizacion/optimizacion.

## Inspiracion tecnica

- [Tesis de Hernan Abaunza Gonzalez](../../references/these_utc_hernan_abaunza_gonzalez.pdf): referencia para modelado y control basados en cuaterniones.
