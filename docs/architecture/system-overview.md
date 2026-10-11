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

## What is not implemented here

There is no versioned Raspberry Pi program, MAVLink connection setup, tested
setpoint interface, actuator mapping, or Pi service configuration. The
standalone PSO simulation does not connect to PX4. Those pieces must be
implemented and documented before this repository can support a Pi/Pixhawk
hardware integration test.
