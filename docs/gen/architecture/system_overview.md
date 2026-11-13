# System overview

## Intended architecture

The Pixhawk 6C runs PX4 and remains responsible for flight-controller functions.
The Raspberry Pi runs the vehicle-specific PSO application and communicates
with PX4 over MAVLink. The [firmware prototype](../../../controller/README.md)
in `controller/` receives validated PSO diagnostic candidates and publishes
`pso_allocation`. Applying candidates to actuator outputs remains unimplemented.

## What is in this repository

- `companion/allocator/`: self-contained C++ solver, MAVLink, tests and deployment.
- `companion/image/`: Raspberry Pi image and Ethernet/SSH configuration.
- `controller/`: pinned PX4/Pixhawk 6C receiver patch, HIL output support and build guide.
- `simulation/`: Gazebo world, vehicle model, meshes and reference controller.
- `legacy/`: standalone PSO, Simulink, CAD, STM32 and historical PX4 copies.

## Companion allocation boundary

The Pi UART/MAVLink application in `companion/allocator/` links its own PSO
runtime in `src/pso/`, accepts local position/velocity, attitude/rates and
airspeed telemetry, and returns nine bounded model actuator candidates.
Results include fitness, predicted position error and a positional convergence
flag. JSON and MAVLink `DEBUG_FLOAT_ARRAY` carry the result. A stale input
suppresses allocation and resets the search center; UART/solver failures exit.
An optional systemd service and Linux pseudo-terminal tests are included.

This is the pose-target experiment, not a validated wrench allocator or a
physical PX4 output interface. Model axes, rotor units, geometry, coefficients,
actuator calibration/mapping and hardware timing/failsafe behavior remain to be
validated. See the companion README for configuration and the precise boundary.
The earlier standalone PSO/visualizer remains archived in `legacy/simulation/`.
The Gazebo reference is still autonomous; its HIL interface and actuator-driven
plant are pending. Reorganizing these sources does not complete that control loop.
