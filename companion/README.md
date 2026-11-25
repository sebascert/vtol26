# VTOL26 Companion

Companion software for VTOL26. It receives aircraft state and control targets
from PX4, runs the PSO allocator and returns nine actuator allocation values
over MAVLink.

Use the [VTOL26 README][vtol26-readme] to set up the complete workspace and the
[companion flashing guide][flashing-guide] to install the Raspberry Pi 4B image.

## Build

Run commands from the VTOL26 workspace root:

- **Allocator:** builds the application and its tests locally.

  ```sh
  ./companion/allocator/build.sh
  ```

- **Image:** builds a bootable Raspberry Pi 4B Linux image with the allocator,
  UART, Ethernet and SSH configured. This build can take several hours.

  ```sh
  ./companion/image/build.sh
  ```

See the [allocator README][allocator-guide] and [image README][image-guide] for
their dependencies and setup instructions.

## Components

- **PSO allocator:** uses aircraft state and pose targets to calculate bounded
  actuator allocations. Vehicle physics come from the shared
  [vehicle configuration][vehicle-config], with separate optimizer tuning.
- **MAVLink communication:** receives PX4 telemetry over serial and sends
  allocation results back to the flight controller. Results are also available
  as JSON output.
- **Companion image:** packages the allocator as a service that starts at boot,
  using `/dev/serial0` at 230400 baud for HIL. Direct Ethernet and SSH provide
  access from the development computer.

## Connect to the companion

After flashing and booting the companion, connect it to the host computer by
Ethernet and run:

```sh
./companion/connect-companion.sh
```

The script configures the host computer's address as `192.168.76.1/24` on
`enp46s0`; the companion uses `192.168.76.2/24`. For another interface, pass its
name: `./companion/connect-companion.sh enx1234`.

Answer `y` to set up SSH key login. Enter password **`vtol`** for user
**`vtol`** when prompted. The script checks the installation, UART access and
allocator service, then prints recent logs. For later connections, run:

```sh
ssh vtol-companion
```

## Testing

After building the allocator, run these commands from the VTOL26 workspace root:

```sh
ctest --test-dir companion/allocator/build --output-on-failure
python3 -m unittest discover -s companion/tests -p 'test_*.py'
python3 -m unittest discover -s companion/image/tests -p 'test_*.py'
```

These check allocation, MAVLink communication, deployment, connection setup and
image configuration without connected hardware. Network setup checks use mocked
commands and do not change the host computer's network settings.

[vtol26-readme]: ../README.md
[flashing-guide]: ../docs/guides/flash_raspberry_pi.md
[allocator-guide]: allocator/README.md
[image-guide]: image/README.md
[vehicle-config]: ../config/README.md
