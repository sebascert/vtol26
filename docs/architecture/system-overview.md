# System overview

## Intended architecture

The Pixhawk 6C runs stock PX4 and remains responsible for the flight controller
functions. A Raspberry Pi 4 is intended to run the vehicle-specific PSO actuator
allocator and communicate with PX4 over MAVLink. The vehicle model and PSO
experiment live in `simulation/`.

## What is in this repository

- A standalone C++ PSO simulation with configuration and visualization support.
- MATLAB/Simulink model sources and a saved model.
- Mechanical CAD, project photographs, and a modeling reference.
- Archived STM32 firmware and earlier implementation guides, kept separate from
  the current design.

## Companion allocation boundary

The Pi UART/MAVLink application in `companion/mavlink_uart/` now links the
shared simulation PSO, accepts local position/velocity, attitude/rates and
airspeed telemetry, and returns nine bounded model actuator candidates.
Results include fitness, predicted position error and a positional convergence
flag. JSON and MAVLink `DEBUG_FLOAT_ARRAY` carry the result. A stale input
suppresses allocation and resets the search center; UART/solver failures exit.
An optional systemd service and Linux pseudo-terminal tests are included.

This is the pose-target experiment, not a validated wrench allocator or a
physical PX4 output interface. Model axes, rotor units, geometry, coefficients,
actuator calibration/mapping and hardware timing/failsafe behavior remain to be
validated. See the companion README for configuration and the precise boundary.
The standalone simulation/visualizer remains independently buildable.
