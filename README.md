# VTOL26

Stallion VTOL: Raspberry Pi allocation, Pixhawk/PX4 integration and Gazebo
simulation. Active work is organized by runtime responsibility:

| Directory | Responsibility | Current status |
| --- | --- | --- |
| [`companion/image/`](companion/image/README.md) | Raspberry Pi image, Ethernet/SSH configuration and packaged allocator | Yocto definition; full image build/boot still pending |
| [`companion/allocator/`](companion/allocator/README.md) | Self-contained C++ PSO solver, MAVLink transport, tests and deployment | USB bench tested; returns diagnostic model allocations |
| [`controller/`](controller/README.md) | Pixhawk 6C PX4 firmware patches and allocator integration | Builds with PSO diagnostic reception and HIL output support; hardware tests pending |
| [`simulation/`](simulation/README.md) | Gazebo model, world and simulation plugin | Existing standalone Gazebo reference; HIL integration pending |
| `legacy/` | Previous PSO experiments, Simulink, CAD and archived firmware | Historical reference |

See the [documentation map](docs/readme.md) for wiring, study material and the
proposed HIL interfaces. Current allocation messages do not command actuators.

## Build the allocator

```sh
cmake -S companion/allocator -B companion/allocator/build -DCMAKE_BUILD_TYPE=Release
cmake --build companion/allocator/build --parallel 2
ctest --test-dir companion/allocator/build --output-on-failure
```

The pinned MAVLink dependency is fetched at configure time, or can be supplied
offline as described in the allocator guide. No Gazebo, legacy or controller
source is needed to build the allocator.

The executable is `companion/allocator/build/vtol_allocator`; the installed
systemd service is `vtol_allocator@<user>.service`.
