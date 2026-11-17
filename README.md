# VTOL26

Raspberry Pi PSO allocator, Pixhawk 6C/PX4 firmware and Gazebo simulation.

Run commands from the repository root on Ubuntu.

Dependencies:

- CMake 3.22+
- C++17 compiler
- Ninja
- Python 3.12 with `venv`
- `socat`

See the [controller guide](controller/VTOL.md) for PX4 host dependencies. First
builds need internet access to download dependencies.

## Build locally

- [Allocator](companion/allocator/build.sh): `./companion/allocator/build.sh`
- [Desktop PX4](controller/build.sh): `./controller/build.sh sitl`
- [Raspberry Pi image](companion/image/build.sh): `./companion/image/build.sh`

The controller script manages its Python environment using
`controller/requirements.txt`. For the image, prepare the host and Scarthgap
layers using the [image guide](companion/image/README.md).

## Run host tests

These test virtual serial communication and the PX4 packet validator without
starting PX4 or requiring hardware.

```sh
cmake -S controller/vtol_tests/communication -B controller/build/host-tests \
  -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON &&
cmake --build controller/build/host-tests --parallel 2 &&
ctest --test-dir controller/build/host-tests --output-on-failure
```

Expect four passing tests.

## Test live allocator/PX4 communication

Open three terminals, each at the repository root.

**Terminal 1 — PX4:**

```sh
PX4_SYS_AUTOSTART=10043 PX4_SIMULATOR=sihsim ./controller/build/sitl/bin/px4
```

**Terminal 2 — UDP/serial bridge:**

```sh
socat PTY,link="$PWD/controller/build/allocator.pty",rawer,echo=0 \
  UDP4:127.0.0.1:14580,bind=127.0.0.1:14540
```

**Terminal 3 — allocator:**

```sh
./companion/allocator/build/vtol_allocator \
  --port "$PWD/controller/build/allocator.pty" --baud 115200 \
  --pso-config companion/allocator/src/pso/config.json
```

Expect JSON allocations in terminal 3. In terminal 1's PX4 console, run:

```text
listener pso_allocation
```

Expect nine candidates and source IDs `245/191`. Repeat to check advancing
timestamps. Allocations are diagnostic; actuator application is not implemented.

Stop the allocator and bridge with Ctrl+C, then enter `shutdown` in PX4.

## Other build targets

**Pixhawk 6C firmware**

Additional dependency:

- ARM toolchain

```sh
./controller/build.sh firmware
```

Artifact: `controller/build/firmware/px4_fmu-v6c_default.px4`.

**Standalone Gazebo simulation**

Additional dependency:

- Gazebo Harmonic development libraries

```sh
./simulation/build.sh
./simulation/run.sh
```

The Gazebo model runs separately; its PX4/HIL bridge remains to be implemented.

See the [Raspberry Pi image guide](companion/image/README.md) for image builds,
and the [documentation map](docs/readme.md) for wiring and further details.
