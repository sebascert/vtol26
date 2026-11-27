# Connect the companion over UART

Connect a Raspberry Pi 4B to the Pixhawk 6C's **TELEM2** port using a
three-wire, **3.3 V UART** link with **8N1 and no hardware flow control**. The
project Pi image and the default Pixhawk HIL profile (22001) both use **230400
baud**. See the [Gazebo HIL guide](hil_simulation.md) for setup.

## 1. Connect the wires

Power off both boards before wiring. Keep propellers removed and actuator power
disconnected during the bench check.

| Pixhawk TELEM2 pin | Signal | Raspberry Pi physical pin | Purpose                               |
| ------------------ | ------ | ------------------------- | ------------------------------------- |
| 2                  | TX     | 10 — GPIO15 / RX          | PX4 sends telemetry to the companion. |
| 3                  | RX     | 8 — GPIO14 / TX           | The companion sends messages to PX4.  |
| 6                  | GND    | 6 — GND                   | Both boards share a signal reference. |

Leave TELEM2 **pin 1 (+5 V), pin 4 (CTS) and pin 5 (RTS) disconnected and
insulated**. Pi GPIO inputs are not 5 V tolerant.

Use a matching six-pin JST-GH lead or breakout. Verify connector orientation
with the [Holybro pinout][telem2-pinout] and locate the Pi header pins using the
[GPIO reference][pi-gpio]. Physical pin numbers differ from GPIO numbers.
Identify wires by continuity, not colour, and check for shorts before powering
up. On a breadboard, keep each signal on a separate electrical node.

## 2. Power the boards

Power the Pi from a suitable USB-C supply. For the bench check, power the
Pixhawk through its USB connection to the host computer running QGroundControl.
Keep the UART ground connected and leave the boards' positive power rails
separate. Disconnect both supplies before changing any wire.

## 3. Check the UART settings

The [companion image][image] enables `/dev/serial0` automatically. The
[custom Pixhawk firmware][flash-pixhawk] configures TELEM2 for MAVLink at 230400
baud with flow control off in profile 22001. The generic service install and
board settings before airframe overrides use 57600 baud.

On the Pi, check that the UART exists:

```sh
ls -l /dev/serial0
```

If using Raspberry Pi OS instead, enable serial hardware and disable the serial
login console through `sudo raspi-config`, then reboot. See the
[allocator README][allocator] to install its service on an existing system.

## 4. Check communication

With Pixhawk USB connected, open **QGroundControl → Q icon → Analyze → MAVLink
Console** and run:

```text
mavlink status
```

Find the **TELEM2** instance: `/dev/ttyS3 @230400` for HIL, with flow control
**OFF**. A nonzero RX rate and received messages from system **245**, component
**191** confirm that the companion is sending messages to PX4.

Use the [companion connection instructions][companion] to establish SSH access.
On the Pi, check the service and recent logs:

```sh
systemctl status vtol_allocator@vtol.service
journalctl -u vtol_allocator@vtol.service -n 20 --no-pager
```

Allocation JSON confirms that the companion receives the required PX4 data and
runs the allocator. It waits when state or position/attitude targets are missing
or stale; a disarmed bench can have no valid targets. Keep the aircraft disarmed
for this connection check.

When allocations are available, run in the PX4 console:

```text
listener pso_allocation
```

Check for nine actuator values, source **245/191** and advancing timestamps on
repeated calls. This confirms PX4 receives the allocator's results.

## Troubleshooting

| Problem                      | Check                                                                      |
| ---------------------------- | -------------------------------------------------------------------------- |
| UART missing or inaccessible | UART configuration, serial login console and user permissions.             |
| No received messages in PX4  | Pi TX → TELEM2 RX, common ground, matching baud rate and flow control off. |
| Allocator waits for data     | TELEM2 TX → Pi RX and valid PX4 state and pose targets.                    |
| Intermittent connection      | Continuity, secure contacts, short wires and stable power.                 |

[telem2-pinout]: https://docs.holybro.com/autopilot/pixhawk-6c/pixhawk-6c-ports#telem-2-port
[pi-gpio]: https://www.raspberrypi.com/documentation/computers/raspberry-pi.html#gpio
[image]: ../../companion/image/README.md
[flash-pixhawk]: flash_pixhawk.md
[allocator]: ../../companion/allocator/README.md#install-as-a-service
[companion]: ../../companion/README.md#connect-to-the-companion
