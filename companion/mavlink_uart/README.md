# `mavlink_uart` prototype

Recovered from the Raspberry Pi USB and copied into this repository with its
CMake source and generated MAVLink C headers. The prebuilt `build/` directory
and its AArch64 executable were left out; rebuild locally after reviewing the
source.

## What the code currently does

- Opens `/dev/serial0` at 57600 baud for both receiving and sending.
- Decodes `ATTITUDE`, `GPS_RAW_INT`, `RAW_IMU`, `SCALED_PRESSURE`,
  `RC_CHANNELS`, `VFR_HUD`, `SYSTEM_TIME`, and `HEARTBEAT` into `DroneState`.
- The main loop calls `PSOInterface`, attempts to send its result, and sends an
  onboard-computer heartbeat.

## Known incomplete paths

- `src/pso_interface.cpp` returns a fixed nine-value vector and does not use the
  received `DroneState`.
- `src/mavlink_sender.cpp::sendPSOCommand()` only prints the vector; it does not
  serialize or transmit a PX4 setpoint.
- `src/pso/pso_environment.cpp` is present but is not included in the current
  `CMakeLists.txt` target.
- `src/main_backup.cpp` is a separate heartbeat-only experiment and is not
  included in the CMake target.
- There is no service/autostart setup or validated failsafe behavior.

This is a software prototype for inspection and development, not a tested
actuator controller. Keep propulsion disabled until the command path, control
mapping, and loss-of-link behavior have been implemented and validated.

## Build

On a Linux system with CMake and a C++17 compiler:

```sh
cmake -S . -B build
cmake --build build
```

The source expects the Raspberry Pi UART alias `/dev/serial0`; verify the Pi's
UART configuration, wiring, and PX4 port/baud settings before connecting it.
The generated MAVLink headers are vendored under `mavlink/` from upstream
MAVLink commit `e5697c4d`.
