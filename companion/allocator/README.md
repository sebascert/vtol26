# VTOL26 Allocator

C++ companion application that receives aircraft state and pose targets from
PX4, runs particle swarm optimization (PSO), and returns nine bounded actuator
allocation values over MAVLink. It uses the shared VTOL26 vehicle model and
separate optimizer tuning.

Set up the workspace using the [VTOL26 README][vtol26-readme] guide. Install the
shared contract with `./config/install.sh` from the workspace root before
running or testing. The allocator reads
`~/.local/share/vtol26/config/vehicle.json` through
`~/.local/share/vtol26/config/validate_vehicle.py`, using its adjacent schema.
Missing or invalid configuration stops startup before opening UART. See
[configuration installation][vehicle-config] for updates and migration.

## Build

Build the application and its tests for the current computer. From the VTOL26
workspace root, run:

```sh
./companion/allocator/build.sh
```

The first build downloads the MAVLink headers; later builds reuse them. Build on
the companion itself when installing on an existing Linux system.

## Features

- **Allocation:** calculates three motor commands, two rotor tilts, two
  ruddervator angles and two aileron angles. It reuses the previous solution and
  limits solver work per cycle.
- **MAVLink connection:** exchanges messages with PX4 over the selected serial
  interface. PX4 supplies position, velocity, attitude and pose targets; the
  allocator sends a heartbeat every second and returns allocation results. It
  validates the source and freshness of incoming data, suspending allocation
  when required inputs are missing or stale.
- **Configuration:** reads vehicle physics and actuator limits from the shared
  vehicle file, with optimizer settings in a separate tuning file.
- **Deployment:** provides a foreground launcher and a systemd service for
  automatic startup.

## Dependencies

- Bash
- CMake 3.14 or newer
- C++17 compiler and Make
- pkg-config
- json-c development headers
- Python 3
- Python venv and pip support (`python3-venv` on Ubuntu)
- CA certificates and Internet access for the first dependency download
- sudo and systemd for service installation

## Working on Ubuntu

Ubuntu is recommended: the scripts use Bash, and development and testing have
been done on Ubuntu.

```sh
sudo apt update
sudo apt install --no-install-recommends \
  build-essential \
  cmake \
  pkg-config \
  libjson-c-dev \
  python3 \
  python3-venv \
  ca-certificates
```

## Run

Run from the workspace root:

```sh
./companion/allocator/deploy/run-companion.sh
```

The launcher forwards options such as `--port` and `--baud` to the allocator to
select the required communication interface and baud rate.

## Configuration and results

- **Vehicle:** [config/vehicle.json][vehicle-config] supplies physics, geometry
  and actuator limits from `~/.local/share/vtol26/config/vehicle.json`.
- **Optimizer:** [src/pso/config.json][pso-config] supplies costs, search
  settings and initial actuator values. Override it with `--pso-config PATH`.

Restart the application after changing either file. Invalid configuration stops
startup. The launcher resolves the tuning file automatically; the build records
the default vehicle file's absolute path.

Each allocation is printed as JSON and sent as MAVLink `DEBUG_FLOAT_ARRAY` with
name `PSO_ALLOC`, from system 245/component 191. Results include the nine
actuator values, fitness, position error and convergence status. The
[vehicle configuration reference][vehicle-config] defines actuator order, units
and coordinate frames.

The live sender uses packet version 1 in `data[13]`. `time_usec` echoes the
oldest PX4 position/attitude sample used in the solve, `data[14..16]` echoes the
position target and `data[17..19]` echoes the RPY attitude target. PX4 uses
these fields to reject stale or mismatched allocations. Version 0 remains
diagnostic-only. JSON output also records `source_state_boot_ms` and
`solve_duration_ms`.

The [complete Gazebo test](../../docs/gen/test_closed_loop_guide.md) runs this
allocator against the custom aircraft through PX4's gated output consumer. Its
faster loop and optimizer tuning are explicit test settings; service defaults
are not changed to flight-control rates. The Yocto image replaces those
observation defaults with the same generated HIL tuning and a 230400-baud, 20 ms
startup configuration. For a checkout-based HIL installation, generate its
tuning with:

```sh
python3 companion/allocator/prepare_hil.py config/vehicle.json \
  companion/allocator/src/pso/config.json companion/allocator/build/hil-pso.json
```

## Install as a service

Install the [dependencies](#working-on-ubuntu) first. On Debian/Ubuntu-based
Linux with systemd, run as your normal user:

```sh
./companion/allocator/deploy/install.sh
```

Rerun the same command to update the application and service. Existing
configuration files are preserved.

The installed service defaults to `/dev/serial0` at 57600 baud. For another
interface, adjust the service configuration.

## Testing

After building, run from the VTOL26 workspace root:

```sh
ctest --test-dir companion/allocator/build --output-on-failure
```

Tests cover configuration validation, coordinate frames, solver bounds, MAVLink
communication, input freshness, the launcher and installation. They use virtual
serial connections and mocked installation commands, without connected hardware
or system installation.

[vtol26-readme]: ../../README.md
[vehicle-config]: ../../config/README.md
[pso-config]: src/pso/config.json

Validator calls use `~/.local/share/vtol26/config/venv/bin/python3`, created by
`./config/install.sh` with the dependencies in `config/requirements.txt`.
