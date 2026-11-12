# Raspberry Pi 4B companion image (Yocto)

This directory defines a **64-bit Raspberry Pi 4B** image, using Poky and
meta-raspberrypi **Scarthgap (Yocto 5.0)**. It follows the supplied
*Yocto Project: End-to-End Guide for Building Custom Linux Images*, particularly
sections 7.2, 8, 9 and 14.2. The example
[`sebascert/yocto_pr1`](https://github.com/sebascert/yocto_pr1) is a structural
reference only: it is not fetched, pinned, included as a layer or used as the
application source.

**Status:** host dependencies are installed, source archives are downloaded,
and image provisioning tests, upstream BitBake metadata parsing and the full
5,204-task dry run pass on Ubuntu 24.04. The companion recipe also passes its
actual fetch/unpack tasks. A complete BitBake build and physical boot have
**not** been performed here; there is no prebuilt `.wic` artifact in this repository.

The image contains:

- `mavlink_uart` compiled from a snapshot of this working tree, including the
  shared PSO source. No application repository URL or commit is required.
- The existing MAVLink service, enabled as `mavlink-uart@vtol.service`, using
  `/usr/bin/mavlink_uart`, `/dev/serial0` at 57600 baud, and
  `/etc/vtol/pso.json`.
- A PL011 UART on GPIO14/15, enabled without kernel/login serial consoles;
  Bluetooth is disabled to free that UART. The [physical wiring guide](../../docs/gen/guides/raspberry_pi_pixhawk_6c_wiring.md)
  still applies. No `raspi-config` step is required on this Yocto image.
- systemd-networkd and a static direct Ethernet connection, with no gateway or
  DHCP requirement: **Pi `192.168.76.2/24`, laptop `192.168.76.1/24`**.
- OpenSSH and a `vtol` account with key-only SSH and passwordless sudo. Root SSH
  and password authentication are disabled. SSH host keys are generated on boot
  by the upstream OpenSSH service, not copied from the build host.

Wi-Fi and Bluetooth packages/firmware are excluded; Internet sharing is not configured. Ethernet and SSH work without
connecting the Pixhawk; the allocation service waits for its required telemetry.
This is a Yocto image, not Raspberry Pi OS: the apt-based `deploy/install.sh`
is not used inside it.

## 1. Prepare the Ubuntu build host

Use a supported Ubuntu host with at least 100 GB free disk and preferably
16 GB RAM. The supplied guide and
[Yocto's Scarthgap host requirements](https://docs.yoctoproject.org/scarthgap/ref-manual/system-requirements.html)
list the prerequisites. On Ubuntu:

```sh
sudo apt update
sudo apt install build-essential chrpath cpio debianutils diffstat file gawk \
  gcc git iputils-ping libacl1 liblz4-tool locales python3 python3-git \
  python3-jinja2 python3-pexpect python3-pip python3-subunit socat texinfo \
  unzip wget xz-utils zstd openssh-client
sudo locale-gen en_US.UTF-8
```

Run the build as your normal user. Do not run BitBake with sudo. On Ubuntu 24.04,
BitBake may report that user namespaces are unavailable. Ubuntu requires an
[application profile allowing user namespaces](https://documentation.ubuntu.com/security/security-features/privilege-restriction/apparmor/#apparmor-unprivileged-user-namespace-restrictions).
For this build, load a temporary named profile and open a build shell under it:

```sh
mkdir -p companion/rasp-image/build
cat > companion/rasp-image/build/bitbake-userns.apparmor <<'EOF'
abi <abi/4.0>,
include <tunables/global>
profile vtol26-bitbake flags=(unconfined) {
    userns,
}
EOF
sudo apparmor_parser --skip-cache -r companion/rasp-image/build/bitbake-userns.apparmor
aa-exec -p vtol26-bitbake -- bash
```

Continue the configuration/build commands inside that shell. The allowance
applies to processes explicitly launched under this profile; the system-wide
restriction stays enabled. It is not installed in `/etc/apparmor.d` and does
not survive reboot. After the build shell and its processes exit, unload it with
`sudo apparmor_parser -R companion/rasp-image/build/bitbake-userns.apparmor`.

Obtain matching Scarthgap Poky and meta-raspberrypi source directories. Existing
copies can be supplied to the next step. Alternatively, from this repository's
root, these commands download source archives without creating Git checkouts:

```sh
VTOL_ROOT="$PWD"
SOURCES="$VTOL_ROOT/companion/rasp-image/build/sources"
mkdir -p "$SOURCES"
wget -O "$SOURCES/poky-scarthgap.tar.gz" \
  https://github.com/yoctoproject/poky/archive/refs/heads/scarthgap.tar.gz
wget -O "$SOURCES/meta-raspberrypi-scarthgap.tar.gz" \
  https://github.com/agherzan/meta-raspberrypi/archive/refs/heads/scarthgap.tar.gz
tar -xzf "$SOURCES/poky-scarthgap.tar.gz" -C "$SOURCES"
tar -xzf "$SOURCES/meta-raspberrypi-scarthgap.tar.gz" -C "$SOURCES"
sha256sum "$SOURCES/"*.tar.gz > "$SOURCES/sources.sha256"
```

These branches receive updates. Keep the downloaded archives and checksums to
reproduce a particular build; no upstream source is vendored into tracked project
files. Standard Yocto recipes may themselves use Git while fetching/building
upstream components. Fetching archives for the layers does not eliminate Git
fetches inside the standard OS recipes.

Only Poky and the Raspberry Pi BSP layer are needed. NetworkManager,
meta-openembedded, ROS and the example repo are not image dependencies.

## 2. Configure the image and SSH access

From the repository root, using the paths from the previous step:

```sh
python3 companion/rasp-image/configure.py \
  "$SOURCES/poky-scarthgap" "$SOURCES/meta-raspberrypi-scarthgap"
```

This creates ignored files under `companion/rasp-image/build/`:

| Path | Purpose |
| --- | --- |
| `yocto/conf/` | Build configuration, layer paths and Raspberry Pi UART settings |
| `provision/vtol-source.tar.gz` | Current companion/shared PSO source snapshot |
| `provision/authorized_keys` | Public key installed for `vtol` |
| `ssh/id_ed25519` and `.pub` | Dedicated laptop SSH identity, created only if absent |

The generated private key has no passphrase and remains on the build laptop,
with mode 0600. It is **never included in the image or source snapshot**. Keep
it private and back it up: possession grants `vtol` access including sudo.
If building on another machine, securely transfer the private key to the laptop
that will connect, or authorize that laptop's existing public key instead:

```sh
python3 companion/rasp-image/configure.py /path/to/poky /path/to/meta-raspberrypi \
  --ssh-public-key /path/to/laptop-key.pub
```

No shared default password is configured. Losing the corresponding private key
requires restoring it or changing the image/SD card's authorized key.

**Re-run configure.py after project source changes.** It refreshes the source
snapshot and generated build configuration while preserving an existing
dedicated key. It deliberately replaces generated `local.conf`/`bblayers.conf`;
make persistent project configuration changes in the `.in` templates instead.
It never fetches or modifies the source Git repositories.

## 3. Build and flash

The configure command prints the exact build-environment command. For the archive
layout above, run in Bash:

```sh
source "$SOURCES/poky-scarthgap/oe-init-build-env" \
  "$VTOL_ROOT/companion/rasp-image/build/yocto"
bitbake-layers show-layers
bitbake vtol-companion-image
```

The build downloads toolchain, kernel and OS dependencies and can take hours.
Two BitBake workers and two make jobs are configured to limit laptop resource
usage. The application recipe fetches the existing checksum-verified MAVLink
header dependency through BitBake and supplies its unpacked path to CMake;
CMake does not download dependencies during cross-compilation.

Look in `tmp/deploy/images/raspberrypi4-64/` for
`vtol-companion-image-raspberrypi4-64*.wic.bz2`, its `.wic.bmap`, and the image
package manifest. Build success and those files are the completion criteria for
the image build; the host-side tests alone do not produce a bootable image.

Use a spare microSD card, 8 GB or larger. Flash the compressed WIC with Raspberry
Pi Imager's **Use custom** image selection, or decompress it and use a disk-image
writer. Do not apply Raspberry Pi OS user/network customization to this Yocto
image. Verify the selected destination: flashing erases that card. Shut down
the Pi before removing/replacing its current card.

## 4. Connect the Ubuntu laptop directly to the Pi

Use an ordinary Ethernet patch cable between the Pi's Ethernet port and the
laptop's Ethernet port (or a USB-Ethernet adapter). Power the Pi separately with
a suitable USB-C supply; this Ethernet cable does not power it. No router or
Pixhawk is needed to establish SSH.

On Ubuntu Desktop using NetworkManager, identify the wired interface:

```sh
nmcli device status
```

Choose the interface connected to the Pi, replacing `enp3s0` below. Check that
`192.168.76.0/24` does not overlap an existing LAN or VPN. Create a dedicated
profile once:

```sh
ETH_IF=enp3s0
sudo nmcli connection add type ethernet ifname "$ETH_IF" con-name vtol-direct \
  ipv4.method manual ipv4.addresses 192.168.76.1/24 ipv4.never-default yes \
  ipv6.method disabled connection.autoconnect yes
sudo nmcli connection up vtol-direct
```

For later sessions, only `sudo nmcli connection up vtol-direct` is needed.
If the profile already exists, edit it rather than creating duplicates. This
profile replaces the active connection on that wired interface; it adds no
Internet gateway or DNS server, so a separate Wi-Fi connection can retain
Internet access. These settings follow
[NetworkManager's manual-addressing examples](https://networkmanager.pages.freedesktop.org/NetworkManager/NetworkManager/nmcli-examples.html).
Ubuntu Server/netplan users can instead assign `192.168.76.1/24` to the chosen
interface with no gateway using their existing network manager; do not run two
managers on the same interface.

Insert the flashed SD card, connect Ethernet, and power the Pi. After boot,
from the repository root on the laptop:

```sh
ping -c 3 192.168.76.2
ssh -o IdentitiesOnly=yes \
  -i companion/rasp-image/build/ssh/id_ed25519 vtol@192.168.76.2
```

If you supplied a public key, use the corresponding private key path. Accept
the first host-key prompt only for the Pi on this isolated cable. Reflashing
regenerates the Pi's host keys: investigate the change before removing a stale
`192.168.76.2` entry from `known_hosts`. Do not disable host-key checking.

To return the laptop port to its usual network later, disconnect `vtol-direct`
and activate its previous connection profile. To remove only this profile:

```sh
sudo nmcli connection down vtol-direct
sudo nmcli connection delete vtol-direct
```

## 5. Verify the installed companion over SSH

On the Pi:

```sh
uname -m
ip -4 address show eth0
sudo systemctl status systemd-networkd
sudo systemctl status mavlink-uart@vtol.service
sudo journalctl -u mavlink-uart@vtol.service -f
ls -l /dev/serial0
cat /proc/cmdline
sudo cat /etc/vtol/pso.json
```

Expect `aarch64`, `192.168.76.2/24`, and no `console=serial0`/`console=ttyAMA0`
on the kernel command line. `/dev/serial0` should identify the header UART.
A missing Pixhawk does not prevent SSH. The allocation service may initially
restart if the UART device is not ready; with a valid UART and no telemetry it
waits for fresh PX4 messages. Connect/configure the Pixhawk using the
[wiring guide](../../docs/gen/guides/raspberry_pi_pixhawk_6c_wiring.md).

Edit calibration/target settings with `sudo vi /etc/vtol/pso.json`, then:

```sh
sudo systemctl restart mavlink-uart@vtol.service
```

The config is a package-managed conffile. **Flashing a new image still replaces
the entire root filesystem**, including local calibration and SSH changes;
back up `/etc/vtol/pso.json` and authorized keys first. The default model limits
are illustrative and physical actuator output mapping remains unimplemented.

## Checks and troubleshooting

From the repository root, host-only checks are:

```sh
python3 -m unittest discover -s companion/rasp-image/tests -v
```

They cover key creation/reuse, rejection of private/invalid keys, source snapshot
refresh/exclusion of credentials, networking/UART/SSH settings and the recipe's
application install paths. They do not parse all upstream BitBake metadata,
build the full OS or prove that it boots on a physical board.

- **No ping:** check Ethernet link LEDs, the laptop interface/profile, Pi power,
  image boot and subnet conflicts. Neither endpoint uses DHCP here.
- **SSH permission denied:** use the private key matching the selected public
  key and username `vtol`; no password login is available.
- **SSH host-key warning after reflash:** verify which board is attached before
  updating the laptop's known-host entry.
- **No allocation JSON:** verify UART mapping and all four required PX4 messages;
  Ethernet connectivity does not supply telemetry to the optimizer.
- **Build failure:** retain the failed task's `log.do_*`, image manifest and
  source archive checksums; check matching Scarthgap layers and host dependencies.

References: the supplied [Yocto documentation](../../docs/references/yocto_documentation.pdf); the example repo linked
above; [meta-raspberrypi](https://github.com/agherzan/meta-raspberrypi/tree/scarthgap);
[Yocto image customization](https://docs.yoctoproject.org/scarthgap/dev-manual/customizing-images.html);
[systemd networking](https://www.freedesktop.org/software/systemd/man/latest/systemd.network.html).
