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

The Python SIL/HIL bridge publishes on
`/model/controlled_drone/actuators_normalized`: `gz.msgs.Actuators.normalized`
carries three motor values in `[0,1]` and six servo values in `[-1,1]`. The
bridge checks the PX4 packet age using MAVLink time sync, then stamps its Gazebo
output with the current simulation time. The plant checks both source age and
receive age, then inverts the normalization using the selected vehicle limits.

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

The [closed-loop SIL test](../controller/vtol_tests/gazebo_closed_loop.py) uses
the same Python bridge and PX4 profile 22001 as
[hardware HIL](../docs/guides/hil_simulation.md). `run_hil.py` starts Gazebo,
forwards ground-station traffic and handles optional target commands.
`gazebo_sensor_model.py` samples Gazebo and generates the body-frame
magnetometer readings. `hil_bridge.py` translates sensors and actuator controls,
checks the PX4 profile and timestamps, and maps the nine actuator channels. In
SIL, two PTY pairs carry the HIL and companion links. On hardware those links
use USB and TELEM2. The constant world magnetic field, Gaussian magnetic sensor
noise (0.02/0.02/0.03 gauss standard deviation in body axes, matching PX4's
`sensor_mag_sim`) and GPS accuracy are synthetic sensor assumptions. The noise
keeps stationary magnetic samples from triggering PX4's stuck-sensor detector.

The stock X500 and the
[sensor-only profile 4500 diagnostic](../controller/vtol_tests/gazebo_sensors.py)
use the `sitl-native` build. In profile 4500, PX4 receives IMU, pressure and GPS
through its native Gazebo bridge and simulates magnetic readings from Gazebo
truth.

Run the serial/Gazebo transport regression without a board:

```sh
simulation/build/venv/bin/python3 simulation/tests/hil_transport_test.py
```

This uses a simulated MAVLink board endpoint and saves evidence in
`simulation/build/hil-transport-test/`. Physical HIL and actuator calibration
still require hardware validation.

Validator calls use `~/.local/share/vtol26/config/venv/bin/python3`, created by
`./config/install.sh` with the dependencies in `config/requirements.txt`.
