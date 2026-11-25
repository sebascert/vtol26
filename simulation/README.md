# Gazebo VTOL plant

Nine ideal actuator commands drive a rigid aircraft body in Gazebo Harmonic. The
plant and allocator share
[the force evaluator](../companion/allocator/src/pso/vehicle_model.cpp). Gazebo
integrates motion and applies gravity once.

Dependencies:

- Gazebo Harmonic development libraries (`gz-sim8`)
- CMake 3.22+
- C++17 compiler
- pkg-config
- json-c development headers
- TinyXML2 development headers
- Python 3
- Python venv and pip support (`python3-venv` on Ubuntu)
- Ubuntu system Python 3.12 and virtual environment support
- Gazebo Python bindings (`python3-gz-transport13`, `python3-gz-msgs10`)

`build.sh` prepares `build/venv` using `/usr/bin/python3` and installs
[requirements.txt](requirements.txt). It uses system Gazebo bindings so their
Python ABI matches Ubuntu's interpreter.

From the repository root:

```sh
./simulation/build.sh
ctest --test-dir simulation/build --output-on-failure
./simulation/run.sh
```

Use `./simulation/run.sh -s` for the server without a GUI. Without actuator
commands the vehicle falls onto the ground.

Install the shared contract first with `./config/install.sh` from the repository
root. Build and launch scripts read `~/.local/share/vtol26/config/vehicle.json`
using `~/.local/share/vtol26/config/validate_vehicle.py` and its adjacent
schema. There is no vehicle-path option. See
[config/README.md](../config/README.md) for updates.

Preparation validates the installed file, snapshots it to
`simulation/build/vehicle.json`, and generates SDF mass, diagonal inertia, CG
and gravity inside `simulation/build/`. Restart after editing the configuration.
Force positions are relative to the CG in FRD; the plugin converts to Gazebo's
FLU body and ENU world axes. The initial vehicle heading is north. Aerodynamics
assume still air and ideal actuators.

## Actuator input

Publish `gz.msgs.Double_V` with exactly nine `data` entries to
`/model/controlled_drone/actuators`, in this order:

1. Left front motor command
2. Right front motor command
3. Rear motor command
4. Left rotor tilt, radians
5. Right rotor tilt, radians
6. Left ruddervator angle, radians
7. Right ruddervator angle, radians
8. Left aileron angle, radians
9. Right aileron angle, radians

Motor values use the JSON's linear thrust law, not PWM or RPM. Send continuously
(for example, 10 Hz): after 0.5 seconds without a valid command, the plugin uses
the configured disarmed values. Wrong-sized, nonfinite or out-of-range messages
immediately invalidate the previous command. Resetting simulation time also
invalidates it. Passive aerodynamic forces still apply while disarmed.

PX4 profile 4501 uses `/model/controlled_drone/actuators_normalized` instead:
`gz.msgs.Actuators.normalized` carries three motor values in `[0,1]` and six
servo values in `[-1,1]`. Its required header stamp identifies the original PX4
state sample in simulation time. The plant checks both source age and receive
age, then inverts the normalization using the selected vehicle limits.

## Verification and remaining work

The headless tests exercise all nine controls with airflow, free fall, balanced
hover, shifted CG and invalid/stale commands. They compare Gazebo velocity and
angular-rate changes with the shared evaluator. Save detailed evidence with:

```sh
ctest --test-dir simulation/build -V --output-log simulation/build/plant-tests.log
```

This checks implementation agreement, not real-aircraft accuracy: configuration
values remain synthetic. The model uses STL visuals and a spherical collision;
the virtual controls are applied as forces and moments without articulated
joints.

Print-mesh origins and axes are corrected by visual assembly poses in the SDF.
See the [mesh notes](models/stallion_vtol/meshes/README.md) for provenance and
placement limits. These visual offsets do not change physical calibration.

The custom aircraft supplies IMU, pressure and GPS to PX4's native Gazebo
bridge. PX4's existing magnetic sensor simulator supplies magnetic readings from
Gazebo truth. Follow the
[sensor test guide](../docs/gen/test_stallion_sensors.md) to verify the EKF
state estimates with the sensor-only startup profile.

The [closed-loop test](../docs/gen/test_closed_loop_guide.md) runs the real
companion allocator, PX4's gated consumer and this plant, including
altitude/lateral targets and input-loss checks. It uses the same `sitl` build
with profile 4501; the stock X500 diagnostic test and profile 4500 sensor test
remain separate. The [external HIL bridge](../docs/guides/hil_simulation.md)
connects this plant to a Pixhawk over USB MAVLink, with a separate TELEM2
companion link. It sends simulated IMU, magnetic field, pressure and GPS,
receives the nine HIL controls and forwards QGroundControl traffic through
UDP 14550. It requires profile 22001, synchronized board time and fresh control
packets. It does not arm automatically. The constant world magnetic field,
Gaussian magnetic sensor noise (0.02/0.02/0.03 gauss standard deviation in body
axes, matching PX4's `sensor_mag_sim`) and GPS accuracy are synthetic sensor
assumptions. The noise keeps stationary magnetic samples from triggering PX4's
stuck-sensor detector.

Run the serial/Gazebo transport regression without a board:

```sh
simulation/build/venv/bin/python3 simulation/tests/hil_transport_test.py
```

This uses a simulated MAVLink board endpoint and saves evidence in
`simulation/build/hil-transport-test/`. Physical HIL and actuator calibration
still require hardware validation.

Validator calls use `~/.local/share/vtol26/config/venv/bin/python3`, created by
`./config/install.sh` with the dependencies in `config/requirements.txt`.
