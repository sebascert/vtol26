# Raspberry Pi companion application

[`mavlink_uart/`](mavlink_uart/) builds a Linux/Raspberry Pi C++17 application
that receives PX4 telemetry over UART, runs the shared PSO implementation in
`../simulation/pso/`, and returns nine bounded actuator candidates as JSON and
MAVLink diagnostic data. It requires fresh telemetry and suspends allocation
on timeout. Build, configuration, output format, tests and optional service
instructions are in that directory's README. On Raspberry Pi OS, run
`./companion/mavlink_uart/deploy/install.sh` from the repository root as your
normal user to install dependencies, build/test, and enable the service.

The output remains a model allocation result. Calibrated conversion and a
validated physical output mapping are required before controlling actuators.
The archived STM32/Arduino integration guides do not describe this runtime.

For a dedicated Raspberry Pi 4B Yocto image with MAVLink installed and direct
Ethernet SSH access from Ubuntu, see [`rasp-image/`](rasp-image/README.md).
