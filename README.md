# Stallion VTOL

Project workspace for the Stallion tricopter-to-fixed-wing VTOL. The intended
architecture uses a stock PX4 flight controller and a Raspberry Pi companion
computer.

## Current state

- The Pixhawk 6C is intended to run stock PX4 and provide the flight-control
  functions.
- The Raspberry Pi 4 is intended to run the PSO actuator allocator and exchange
  commands and telemetry with PX4 over MAVLink.
- This repository contains the PSO simulation, vehicle model, CAD, and project
  references. It does **not** contain the Raspberry Pi MAVLink application or
  a tested mapping from the PSO's nine outputs to PX4 setpoints.
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
- `companion/` — status and scope for the not-yet-versioned Raspberry Pi
  application.
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

The companion application still needs a versioned implementation, connection
settings for the actual Pi-to-Pixhawk link, PSO-to-PX4 command mapping, and
connection-loss behavior. See [`companion/README.md`](companion/README.md).
