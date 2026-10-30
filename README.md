# Stallion VTOL

Project workspace for the Stallion tricopter-to-fixed-wing VTOL. The intended
architecture uses a stock PX4 flight controller and a Raspberry Pi companion
computer.

## Current state

- The Pixhawk 6C is intended to run stock PX4 and provide the flight-control
  functions.
- The Raspberry Pi 4 is intended to run the PSO actuator allocator and exchange
  commands and telemetry with PX4 over MAVLink.
- This repository contains the PSO simulation, vehicle model, CAD, project
  references, and a Raspberry Pi UART/MAVLink prototype in
  `companion/mavlink_uart/`.
- The companion runs the shared PSO on fresh PX4 telemetry and returns bounded
  nine-value allocations as JSON and MAVLink diagnostic data, with timeout
  suppression. Physical actuator output mapping remains unimplemented.
- Unmodified MAVLink headers are downloaded at build time from a pinned upstream
  revision; third-party source copies are excluded from the project source tree.
- Earlier STM32 firmware and its implementation guides are retained under
  `legacy/` and `docs/archive/` for reference. They do not describe the current
  architecture.

The project is therefore a source and simulation workspace, not a complete
hardware deployment package.

## Repository layout

- `simulation/pso/` — standalone C++ PSO experiment, JSON configuration, and
  Python visualization.
- `simulation/simulink/` — MATLAB scripts and the Simulink vehicle model.
- `hardware/cad/` — mechanical CAD files; `hardware/photos/` — dated project
  photographs.
- `companion/` — Raspberry Pi UART/MAVLink prototype and integration status.
- `docs/` — architecture, modeling, references, and archived guides.
- `legacy/stm32-firmware/` — retired PlatformIO/STM32 code and test stubs.
- `tools/` — CSV plotting utility.

## Run the PSO simulation

Requirements: a C++17 compiler. From the repository root:

```sh
cd simulation/pso
g++ -std=c++17 -O2 -o pso_environment pso_environment.cpp
./pso_environment
```

The program reads `config.json` from its working directory and writes
`pso_data.json`. To run the live visualization in another terminal, install its
Python dependencies and run:

```sh
python -m pip install -r requirements.txt
python visualize_particles.py
```

The checked-in sample output is under `simulation/pso/examples/`.

## Simulink model

Open `simulation/simulink/VTOL_SIMULINK.slx` in MATLAB/Simulink. The companion
`.m` files in the same directory provide the model-building, parameter, and
visualization helpers. MATLAB/Simulink is not included in this repository.

## Next integration work

The companion allocation runtime needs aircraft model/calibration validation,
a physical PX4 output mapping, and Pi-to-Pixhawk hardware timing/failsafe tests.
See [`companion/README.md`](companion/README.md) for the runnable allocation path.
