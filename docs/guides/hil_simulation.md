# HIL simulation

Gazebo runs on the host computer, PX4 on the Pixhawk, and the allocator on the
Raspberry Pi. Run host computer commands from the project root.

For a desktop rehearsal, follow the
[SIL build and run instructions](../../README.md#testing). The same bridge,
vehicle configuration, HIL messages and PX4 profile run on both setups. Physical
UART behavior and Pi solve times require the hardware bench.

Keep physical motor/servo power disconnected throughout this simulation.

1. Follow the [Pixhawk flashing guide](flash_pixhawk.md) and
   [Raspberry Pi flashing guide](flash_raspberry_pi.md). Use the same
   `config/vehicle.json` for both builds and the simulation.

   Build the simulation on the host computer:

   ```sh
   ./simulation/build.sh
   ```

2. Follow the [wiring guide](connect_companion_uart.md).

3. Connect to the Pi by SSH in a separate terminal:

   ```sh
   ssh vtol-companion
   ```

   Keep this terminal open.

4. Open QGroundControl and connect the Pixhawk to the host computer by USB. Wait
   for the vehicle to be detected and its parameters to finish loading.

5. In **QGroundControl → Q icon → Analyze → MAVLink Console**, select the HIL
   profile and reboot:

   ```text
   param set SYS_AUTOSTART 22001
   param set SYS_HITL 1
   param set USB_MAV_MODE 5
   param save
   reboot
   ```

6. Open **Q icon → Settings → Comm Links** in the left sidebar. Under
   **Auto-Connect**, disable **Pixhawk** and leave **UDP** enabled. Older
   versions show these options under **General → AutoConnect to the following
   devices**. See the
   [QGroundControl Comm Links reference](https://docs.qgroundcontrol.com/master/en/qgc-user-guide/settings_view/comm_links.html).

   Then **quit QGroundControl completely** and unplug Pixhawk USB.

7. Reconnect Pixhawk USB to the host computer. Keep the Pi powered through USB-C
   and physical motor/servo power disconnected for the remaining steps. Start
   Gazebo from a host computer terminal:

   ```sh
   ls -l /dev/serial/by-id/
   ./simulation/run-hil.sh --port /dev/serial/by-id/YOUR_PIXHAWK --target 0 0 -0.3
   ```

   Replace `YOUR_PIXHAWK` with the listed device. Append `--headless` to hide
   the Gazebo window. Keep this terminal running; do not start another Gazebo or
   desktop PX4 instance. The target is NED position `(0, 0, -0.3)` metres
   relative to PX4's local origin, with yaw zero.

   If the port is busy, close its owner before retrying:

   ```sh
   sudo fuser -v /dev/serial/by-id/YOUR_PIXHAWK
   ```

8. Wait for `ready: true` and increasing `imu`, `gps` and `controls` counts.
   Reopen QGroundControl; it connects automatically over **UDP 14550** with UDP
   Auto-Connect enabled. Keep automatic USB connections disabled. After about 15
   seconds, open **Q icon → Analyze → MAVLink Console**:

   ```text
   listener vehicle_local_position
   listener pso_actuator_outputs
   ```

   Continue only when `xy_valid`, `z_valid`, `v_xy_valid` and `v_z_valid` are
   `True`, outputs show `active: False`, and the first three controls (motors)
   are zero. `heading_good_for_control: False` is normal on the ground. If a
   check fails, stay disarmed and see troubleshooting below.

9. In the Pi's SSH terminal, restart and check the allocator service:

   ```sh
   sudo systemctl restart vtol_allocator@vtol.service
   systemctl status vtol_allocator@vtol.service --no-pager
   systemctl show vtol_allocator@vtol.service -p ExecStart
   sudo journalctl -u vtol_allocator@vtol.service -f -o cat
   ```

   Expect the service running on `/dev/serial0` at **230400 baud**, with
   `--pso-period-ms 20`. Leave it running; do not start a second allocator.
   Restart it after each Pixhawk reboot: the allocator remembers target
   timestamps from the previous boot and can reject the new boot's targets.
   `Waiting for fresh PX4 state and pose targets.` can be expected before
   step 10. Allocation JSON appears only after fresh PX4 heartbeat, attitude,
   local position and both position/attitude targets arrive. The startup message
   alone does not confirm UART communication.

10. In QGroundControl's MAVLink console, request Offboard mode:

    ```text
    commander mode offboard
    listener vehicle_status
    listener trajectory_setpoint
    ```

    Confirm `nav_state: 14` (Offboard), `pre_flight_checks_pass: True`, and a
    fresh trajectory with position `[0, 0, -0.3]` and yaw zero. Confirm this
    target represents a small rise from the simulated ground position. While
    disarmed, PX4 can publish `nan` processed position setpoints and the Pi can
    remain waiting; check allocations after arming below.

11. With these checks passing and physical actuator power disconnected, arm:

    ```text
    commander arm
    listener vehicle_status
    listener pso_allocation
    listener pso_actuator_outputs
    listener vehicle_local_position
    ```

    Do not force arm. After spool-up, expect allocations with nine values,
    protocol version **1**, source **245/191** and advancing timestamps, plus
    allocation JSON in the Pi journal. Outputs should show `active: True` and
    `reason: 0`, with increasing bridge `controls` and movement toward the
    target in Gazebo. Together these confirm the complete Gazebo → PX4 → Pi →
    PX4 → Gazebo feedback loop.

    If outputs stop, disarm and fix the cause before rearming. Reason **1**
    means disarmed, wrong mode or another arming inhibition; check the current
    `vehicle_status` and `actuator_armed`, since PX4 may have auto-disarmed
    without takeoff. Reason **2** means stale/invalid state or target, **3**
    missing/invalid allocation, and **4** a latched fault. PX4 rejects
    allocations based on state older than **250 ms**.

12. End the virtual experiment from the MAVLink console:

    ```text
    commander disarm
    listener pso_actuator_outputs
    ```

    Confirm inactive outputs and zero motors, then press Ctrl+C in the bridge
    terminal. If disarm is refused while airborne, stop the bridge, reboot the
    Pixhawk and confirm disarmed before another run. The virtual aircraft may
    fall. Logs are in `simulation/build/hil/` and the Pi service journal.

13. Before returning to real hardware operation, restore the saved flight
    airframe/parameters, set `SYS_HITL=0` and reboot. Keep actuator power
    disconnected until the flight configuration is restored.

Vehicle values are synthetic. Physical Pi timing and armed HIL operation still
need validation; motor-stop-on-loss is a simulation bench policy.

<details>
<summary>Connection and estimator troubleshooting</summary>

- **Bridge `controls` stays zero:** run `mavlink status` and
  `mavlink status streams`. USB should use **Config** mode and send
  `HIL_ACTUATOR_CONTROLS`; TELEM2 should use **Onboard**, 230400 baud, flow
  control off. If USB uses Onboard, set `USB_MAV_MODE=5`, save, reboot and
  repeat steps 6–8.
- **Position stays invalid:** run `listener estimator_gps_status`,
  `listener estimator_status_flags` and `listener vehicle_magnetometer`. GPS
  checks should pass, `cs_yaw_align` should be `True`, and the magnetometer
  timestamp should be recent. Older bridges without magnetic sensor noise can
  trigger PX4's stuck-sensor detector; use the current bridge, reboot the
  disarmed Pixhawk and repeat steps 6–8. No simulation rebuild is needed.

</details>
