# Raspberry Pi PSO allocation runtime

The C++17 companion runs the shared solver in `simulation/pso/` against fresh
PX4 telemetry and returns nine bounded model actuator values. It warm-starts
each solve from the previous result and caps particles, iterations and external
cycles. No visualization files are written by the runtime.

For physical connections, power and matching PX4 settings, follow the
[Raspberry Pi ↔ Pixhawk 6C wiring guide](../../docs/guides/raspberry-pi-pixhawk-6c-wiring.md).

## Build and run

From the repository root on Linux (including Raspberry Pi OS), with CMake
3.14 or newer and a C++17 compiler:

```sh
cmake -S companion/mavlink_uart -B companion/mavlink_uart/build -DCMAKE_BUILD_TYPE=Release
cmake --build companion/mavlink_uart/build -j2
ctest --test-dir companion/mavlink_uart/build --output-on-failure
./companion/mavlink_uart/build/mavlink_uart --port /dev/serial0 --baud 57600
```

The first configure downloads the upstream MAVLink C headers into the ignored
build directory. CMake pins commit `e5697c4deaf09068560cb33da2a0283ad6229b3e`
and verifies the archive's SHA-256. No unchanged third-party source is stored
in the project source tree. Later builds reuse the downloaded dependency.
For offline builds, provide an extracted copy of that revision with
`-DFETCHCONTENT_SOURCE_DIR_MAVLINK=/absolute/path/to/c_library_v2` when configuring.

`--help` lists timing and solver budget options. The default config path is
relative to the repository root; pass an absolute `--pso-config` elsewhere.
Configure the Pi UART and the connected telemetry port to the same baud rate.
The serial user needs device access (typically membership in `dialout`).

The input stream must contain PX4 `HEARTBEAT`, `ATTITUDE` (including body rates
in rad/s), `LOCAL_POSITION_NED` (position and velocity in metres and m/s), and
`VFR_HUD` (airspeed in m/s). Arrange these message streams on the flight
controller before running; the companion does not configure stream rates.
The first PX4 heartbeat selects the source system/component. Other sources
are ignored. GPS altitude is not used as a substitute for local position.

No allocation is returned until all four inputs arrive. Each must be newer
than `--heartbeat-timeout-ms` (default 3000 ms). Non-finite telemetry invalidates
its input immediately. A stale input suspends allocation and resets the search
center. The companion keeps sending its own heartbeat at 1 Hz. UART failures
and configuration/solver errors terminate with a nonzero exit status.

## Allocation result

Each successful solve prints one JSON object to stdout:

```json
{"actuators":[1000,1050,950,0.5,0.5,0,0,0.1,-0.1],"fitness":1000,"position_error_m":10,"position_converged":false,"objective_evaluations":97}
```

This is an illustrative schema, not a recorded solve. The array order is:

| Indices | Values | Units |
| --- | --- | --- |
| 0–2 | Left, right, rear rotor command (`omega`) | Existing model units; see calibration limitation below |
| 3–4 | Left, right rotor tilt (`epsilon`) | rad |
| 5–6 | Left, right ruddervator (`gamma`) | rad |
| 7–8 | Left, right aileron (`beta`) | rad |

The same result is transmitted as MAVLink 2 `DEBUG_FLOAT_ARRAY`, name
`PSO_ALLOC`, array ID 0, from system 245/component 191. Data 0–8 are the actuator
values; 9 is fitness; 10 is predicted position error in metres; 11 is positional
convergence (0/1); 12 is objective evaluation count. Remaining entries are zero.
The timestamp is local monotonic time in microseconds. Consumers must expire
results when messages stop. This message reports an allocation; it does not
command PX4 outputs. MAVLink headers are obtained as the build dependency
described above.

The API `PSOInterface::runPSO` returns `pso_runtime::Result`. Successful results
are finite and within the configured bounds. `position_converged` only compares
the predicted position error with `pso.convergence_threshold`; it does not
certify attitude convergence, force/torque tracking, global optimality, or flight
stability. An unreachable target still returns the best bounded candidate with
`position_converged=false`. The previous center is retained as a candidate,
so another solve with identical inputs cannot worsen its fitness.

## Configuration and model limits

`src/pso/config.json` contains the pose target, physics, PSO settings, and
required nine-element `actuators.min`, `max`, and `initial` arrays. Invalid
numeric values, invalid inertia/mass/time step, reversed limits, and an initial
state outside the limits cause startup to fail. CLI budgets cap the configured
PSO work; defaults allow 97 objective evaluations per solve (16 particles,
5 iterations, one external cycle, plus the initial center).

The limits in the supplied config are illustrative bounds for the experiment,
not measured aircraft calibration. In particular, the inherited force model
uses **linear** `thrust_coefficient * omega`; its rotor values must not be
interpreted as calibrated RPM, rad/s, PWM or normalized motor thrust. The model
also uses fixed geometry and zero angle of attack. Its historical wing-drag
axis convention and overlapping tail/ruddervator contributions need model
validation. The companion negates NED Z/VZ for the experiment's positive-up
world convention; the complete world/body attitude convention still needs
validation against the aircraft. Measured airspeed replaces the simulation's
fixed reference airspeed, including zero airspeed during hover.

This implements the repository's pose-target experiment, not a force/torque
allocator receiving wrench demands from PX4. Its fitness is the existing
pose/dynamics objective; the legacy energy weights are not an implemented
power-consumption model. The quaternion orientation predictor now integrates
existing angular velocity and angular acceleration over the configured step.

Before physical actuation, supply calibrated motor/servo limits and conversion,
validate geometry, axes and propulsion/aerodynamic coefficients, define the
controller-demand interface and output mapping, and validate timing/failsafe
behavior on hardware. The runtime neither arms nor changes flight mode.

## Install on Raspberry Pi OS

From a copy of this repository on the Pi, run as your normal user (the script
uses sudo when needed):

```sh
./companion/mavlink_uart/deploy/install.sh
```

The installer installs build dependencies using apt, downloads the pinned
MAVLink dependency, builds and runs tests, and installs the executable and
systemd service under `/usr/local`. It adds your user to `dialout` and enables
`mavlink-uart@<your-user>.service` at boot. It starts/restarts the service if
`/dev/serial0` is present. Internet access, sudo privileges and systemd are
required. No automatic reboot is performed.

An existing installed PSO config is preserved. On first installation the
example is copied to
`/usr/local/share/vtol26/companion/mavlink_uart/src/pso/config.json`.
Edit that file with sudo to apply your model/calibration, then restart the
service. Running the installer again upgrades the executable and service while
keeping that config.

Configure the Pi UART with `sudo raspi-config`: disable the serial login
console, enable the serial hardware, and reboot if settings changed. Verify
wiring and that the PX4 telemetry port uses 57600 baud. The installer leaves
boot configuration and flight-controller settings unchanged. The supplied
service uses `/dev/serial0`; edit its `ExecStart` if a different port or baud is
needed (a reinstall replaces the supplied service file).

```sh
systemctl status "mavlink-uart@$USER.service"
journalctl -u "mavlink-uart@$USER.service" -f
sudo systemctl restart "mavlink-uart@$USER.service"
```

Log out/in for interactive serial access after a new `dialout` membership;
the service receives that access immediately through `SupplementaryGroups`.
The installer also has a mocked-command smoke test covering initial install,
config preservation, and stopping deployment when tests fail.
Tests use Linux pseudo-terminals to check solver bounds, warm-start fitness,
invalid inputs/config, source filtering, MAVLink result decoding, startup
without telemetry, and timeout suppression through the actual executable.
No Pi/Pixhawk hardware validation has been performed here. The recovered
`src/pso/pso_environment.cpp`, `src/pso/config_loader.hpp`, `main_backup.cpp`
and `ActuatorCommand` experiment are excluded from the build; the runtime uses
the shared simulation implementation instead.
