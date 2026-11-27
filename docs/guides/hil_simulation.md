# HIL simulation

Gazebo runs on the Linux host, PX4 on the Pixhawk 6C, and the allocator on the
Raspberry Pi. Run host commands from the project root. Remove propellers and
keep motor and servo power disconnected throughout the bench run.

## One-time configuration

1. Install the shared vehicle configuration and build the host simulator.

   ```sh
   ./config/install.sh
   ./simulation/build.sh
   ```

2. [Build and flash the Pixhawk](flash_pixhawk.md) and
   [flash the project Pi image](flash_raspberry_pi.md) using the same vehicle
   configuration. `./controller/build.sh firmware` defaults a board with no
   saved airframe to Stallion HIL **22001**. That profile sets `SYS_HITL=1`, USB
   MAVLink Config mode and TELEM2 at **230400 baud**. The Pi image starts the
   matching allocator at boot. Reuse these builds for repeat runs. Rebuild and
   reflash the affected components when their software or vehicle configuration
   changes.

   **Previously configured Pixhawk only.** Flashing preserves an explicitly
   saved airframe. If switching from another airframe, save its flight
   parameters first. With QGroundControl connected directly by USB, open **Q
   icon → Analyze → MAVLink Console** and select the HIL airframe once.

   ```text
   param set SYS_AUTOSTART 22001
   param save
   reboot
   ```

3. [Wire TELEM2 to the Pi](connect_companion_uart.md#1-connect-the-wires) with
   both boards powered off. Use USB for the Pixhawk and a separate USB-C supply
   for the Pi.
4. In **QGroundControl → Settings → Comm Links → Auto-Connect**, disable
   **Pixhawk** and enable **UDP**. Quit QGroundControl once to release the USB
   port. The launcher uses USB and forwards QGroundControl traffic over loopback
   UDP port **14550**.
   [QGroundControl Comm Links reference](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html)

## Repeat a waypoint run

1. Connect the disarmed Pixhawk by USB to the simulator host and power the Pi.
   On a fresh Pi boot the allocator starts automatically. If the Pixhawk
   rebooted while the Pi stayed on, restart the allocator once so it accepts
   timestamps from the new PX4 boot. From a host with
   [companion SSH access](../../companion/README.md#connect-to-the-companion),
   run `ssh vtol-companion sudo systemctl restart vtol_allocator@vtol.service`.
2. Start the simulator on the host.

   ```sh
   ./simulation/run-hil.sh
   ```

   The launcher detects a single Pixhawk 6C and selects the waypoint route. Open
   QGroundControl on the same host. Wait for `Waypoint: on pad` and complete the
   [before-arming checks](#sanity-checks).
3. In QGroundControl's **Analyze → MAVLink Console**, enter
   `commander mode offboard`. Confirm Offboard and preflight readiness, then
   enter `commander arm`. Never force arm. The vehicle climbs 0.30 m, travels 2
   m north and 2 m east, returns diagonally and descends, facing north
   throughout. Gazebo must measure one continuous second within 5.5 cm of each
   destination before the route advances.
4. At `Waypoint: landed`, enter `commander disarm`. Confirm `Waypoint: disarmed`
   and zero motor outputs, then press Ctrl+C in the host terminal.
   `simulation/build/hil/waypoints.json` records arrivals and the final result.
   `waypoint-trace.csv` and `pose.csv` hold the requested and measured path. The
   next run reuses this directory. Pass `--output-dir PATH` to keep separate
   runs.

## Sanity checks

- **Before arming**, the host should show `"ready": true`, increasing `imu`,
  `gps` and `controls` counts, and `Waypoint: on pad`. In the PX4 console,
  `listener vehicle_local_position` should report `xy_valid`, `z_valid`,
  `v_xy_valid` and `v_z_valid` as true. `listener pso_actuator_outputs` should
  report inactive outputs and zero for the first three controls (motors).
- **After requesting Offboard**, check `listener vehicle_status` for
  `nav_state: 14` and `pre_flight_checks_pass: True` before arming.
  `listener trajectory_setpoint` should show a fresh target at the pad with yaw
  zero.
- **While flying**, watch Gazebo and the route phases in the host terminal. To
  confirm the Pi feedback loop, inspect `listener pso_allocation` for advancing
  timestamps, nine values, version 1 and source 245/191. After spool-up,
  `listener pso_actuator_outputs` should report `active: True` and `reason: 0`.
- **After disarming**, confirm `listener pso_actuator_outputs` reports inactive
  outputs and zero for the three motors.

## Overrides and troubleshooting

- **USB detection.** With multiple boards or missing USB metadata, choose the
  device with
  `./simulation/run-hil.sh --port /dev/serial/by-id/YOUR_PIXHAWK --waypoints`.
  `--port` alone preserves bridge-only operation for external target senders.
  For a busy port, check `sudo fuser -v /dev/serial/by-id/YOUR_PIXHAWK` and
  close its owner.
- **Other modes.** Add `--headless` for a host without a display. Use
  `--bridge-only` for an external target sender or `--target 0 0 -0.3` for fixed
  hover. A fixed target is absolute in PX4 local NED. Check its height against
  the pad and disarm manually at the end of the hover.
- **No bridge readiness or controls.** Check `param show SYS_AUTOSTART` in the
  PX4 console. A board with a saved airframe can require the one-time selection
  above. With profile 22001 running, `mavlink status` and
  `mavlink status streams` should show USB in Config mode with
  `HIL_ACTUATOR_CONTROLS` and TELEM2 in Onboard mode at 230400 baud with flow
  control off.
- **Invalid position.** Check `listener estimator_gps_status`,
  `listener estimator_status_flags` and `listener vehicle_magnetometer`. Look
  for passing GPS checks, `cs_yaw_align: True` and recent magnetometer
  timestamps. `heading_good_for_control: False` can be normal on the ground.
- **Missing allocations.** On the Pi, check
  `systemctl status vtol_allocator@vtol.service`,
  `systemctl show vtol_allocator@vtol.service -p ExecStart` and
  `sudo journalctl -u vtol_allocator@vtol.service -n 20`. Expect one service on
  `/dev/serial0` at 230400 baud with `--pso-period-ms 20`. It can wait for fresh
  state and targets while disarmed. Look for allocation JSON after arming.
- **Route abort or inactive outputs.** Read the reason after
  `Waypoint: aborted`. In `pso_actuator_outputs`, reason 1 indicates arming or
  mode inhibition, 2 invalid or stale state or target, 3 a missing or invalid
  allocation, and 4 a latched fault. Disarm before restarting. If PX4 refuses
  disarm while airborne in simulation, stop the bridge, reboot the Pixhawk and
  confirm disarm before restarting from the pad.

Before returning the board to physical operation, restore the saved flight
airframe and parameters, set `SYS_HITL=0` and reboot with actuator power
disconnected. The Stallion HIL profile uses simulated outputs and synthetic
vehicle values. For a board-free rehearsal of this route, see the
[desktop waypoint demo](../../simulation/README.md#record-a-waypoint-flight).
