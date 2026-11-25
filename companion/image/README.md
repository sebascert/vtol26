# VTOL26 Companion Image

Bootable 64-bit Linux image for the Raspberry Pi 4B, built with Yocto. It
includes the PSO allocator, automatic service startup, the Pixhawk UART
connection and direct Ethernet access from a development computer.

Set up the workspace using the [VTOL26 README][vtol26-readme]. Follow the
[companion flashing guide][flashing-guide] to write the image to a microSD card.

## Build

From the VTOL26 workspace root, run as your normal user:

```sh
./companion/image/build.sh
```

The script downloads the required Yocto layers, prepares the configuration and
builds the image with the current allocator source and shared vehicle settings.
It may request sudo for host setup; BitBake runs as your normal user. The first
build downloads dependencies and can take several hours.

The disk image is written under
`companion/image/build/yocto/tmp/deploy/images/raspberrypi4-64/` as
`vtol-companion-image-raspberrypi4-64*.wic.bz2`. Use the
[flashing guide][flashing-guide] after the build finishes successfully. Flashing
replaces the card's contents, including local configuration and SSH changes;
save any changes you need first.

## Included configuration

- **Allocator:** the [PSO application][allocator-readme] runs automatically as
  `vtol_allocator@vtol.service`.
- **UART:** `/dev/serial0` uses GPIO14/15 at **230400 baud**, matching Pixhawk
  external HIL profile **22001**. The serial login console and Bluetooth are
  disabled to free the port. Follow the [wiring guide][wiring-guide] to connect
  Pixhawk TELEM2.
- **Ethernet:** the companion uses `192.168.76.2/24` and the development
  computer uses `192.168.76.1/24`, with no router or gateway required.
- **SSH:** initial login is **`vtol` / `vtol`**, with passwordless sudo for the
  direct Ethernet setup. The connection script sets up SSH key login.
- **Vehicle settings:** `~/.local/share/vtol26/config/vehicle.json` contains
  physics and actuator limits; `/etc/vtol/pso.json` contains generated HIL
  optimizer tuning. Its hover starting point comes from the vehicle file. The
  service uses a 20 ms interval between solves, 96 particles, 15 iterations and
  two search passes.

UART and service setup are included in the image. Wi-Fi is disabled. The image
uses Yocto; the allocator's Raspberry Pi OS installer is for a separate setup.

## Dependencies

- Bash and standard Linux build tools
- Git and download/archive tools
- Python 3 and the Yocto host packages listed below
- Locale and user namespace support
- sudo for host setup when needed
- Internet access for source and dependency downloads
- At least 100 GB free disk space; preferably 16 GB RAM
- NetworkManager and an OpenSSH client on the host computer for connection setup

## Working on Ubuntu

Ubuntu is recommended: the scripts use Bash, and development and testing have
been done on Ubuntu. Use a host supported by the
[Yocto host requirements][yocto-requirements]. Install the build and host
computer connection dependencies:

```sh
sudo apt update
sudo apt install --no-install-recommends \
  build-essential \
  chrpath \
  cpio \
  debianutils \
  diffstat \
  file \
  gawk \
  gcc \
  git \
  iputils-ping \
  libacl1 \
  liblz4-tool \
  locales \
  python3 \
  python3-git \
  python3-jinja2 \
  python3-pexpect \
  python3-pip \
  python3-subunit \
  socat \
  texinfo \
  unzip \
  wget \
  xz-utils \
  zstd \
  network-manager \
  openssh-client \
  openssl \
  sudo
```

The build script prepares the required locale and build permissions when needed.
It does not install these packages automatically.

## Connect and check

Connect the flashed companion to the host computer by Ethernet and power it with
a suitable USB-C supply. From the VTOL26 workspace root on the host computer,
run:

```sh
./companion/connect-companion.sh
```

Answer `y` to set up key login, entering password `vtol` when prompted. For
another Ethernet interface on the host computer, pass its name to the script.
See the [companion README][companion-readme] for connection details.

Connect and inspect the service:

```sh
ssh vtol-companion
systemctl status vtol_allocator@vtol.service
journalctl -u vtol_allocator@vtol.service -f
```

Ethernet and SSH work without a Pixhawk connected. The allocator waits for fresh
PX4 state and pose targets before producing results. See the
[allocator README][allocator-readme] for the required inputs.

Edit vehicle settings in `~/.local/share/vtol26/config/vehicle.json` and
optimizer tuning in `/etc/vtol/pso.json`, then restart the service:

```sh
sudo systemctl restart vtol_allocator@vtol.service
```

For settings included in future images, update the shared
[vehicle configuration][vehicle-config] or allocator tuning before rebuilding.
The supplied vehicle values are placeholders; use measured calibration before
physical actuation.

Rebuilding regenerates the hover starting point from the shared vehicle file.
The current generator requires symmetric front motors, a centered rear motor and
zero reaction torque. Changing vehicle parameters on a running Pi also requires
updating its matching optimizer configuration. Actual Pi solve timing and armed
hardware HIL remain unverified.

## Testing

From the VTOL26 workspace root, run:

```sh
python3 -m unittest discover -s companion/image/tests -p 'test_*.py'
python3 -m unittest discover -s companion/tests -p 'test_*.py'
```

These check source packaging, vehicle settings, UART/network/SSH configuration,
service installation paths and host computer connection setup. They use
temporary files and mocked commands without changing the host computer's network
settings. A full image build and a boot check on the companion verify the
deployed system.

[vtol26-readme]: ../../README.md
[flashing-guide]: ../../docs/guides/flash_raspberry_pi.md
[allocator-readme]: ../allocator/README.md
[companion-readme]: ../README.md
[wiring-guide]: ../../docs/guides/connect_companion_uart.md
[vehicle-config]: ../../config/README.md
[yocto-requirements]: https://docs.yoctoproject.org/scarthgap/ref-manual/system-requirements.html

## Shared vehicle configuration

Before preparing an image on the host, install the shared contract with
`./config/install.sh` from the repository root. The host needs Python 3 with
venv and pip support. Image preparation reads
`~/.local/share/vtol26/config/vehicle.json` through
`~/.local/share/vtol26/config/validate_vehicle.py`. The image installs the
vehicle, schema and validator under `/home/vtol/.local/share/vtol26/config/`,
owned by `vtol`, plus an isolated Python environment containing jsonschema for
allocator startup validation. Rebuild the image after changing the installed
vehicle. See [config/README.md](../../config/README.md) for installation and
updates.

Validator calls use `~/.local/share/vtol26/config/venv/bin/python3`, created by
`./config/install.sh` with the dependencies in `config/requirements.txt`.
