# Raspberry Pi companion

- [`allocator/`](allocator/README.md): self-contained PSO and MAVLink application.
  The active solver, state types, parser and calibration live in `allocator/src/pso/`.
- [`image/`](image/README.md): Raspberry Pi 4B Yocto image definition, direct
  Ethernet/SSH settings and allocator packaging.

The allocator needs fresh PX4 telemetry and returns bounded model candidates
as JSON and MAVLink diagnostics. It currently has no physical actuator output
mapping. The planned PX4-side integration belongs in [`controller/`](../controller/README.md),
and Gazebo code belongs in [`simulation/`](../simulation/README.md).
