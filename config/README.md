# Vehicle configuration

Shared vehicle physics, geometry and actuator limits for PX4, the allocator and
Gazebo. [vehicle.json](vehicle.json) contains **synthetic placeholders**;
replace them with measured calibration before physical actuation. Optimizer
tuning stays in the [allocator configuration](../companion/allocator/README.md).

## Dependencies

- Bash
- coreutils
- Python 3 with venv and pip support (`python3-venv` on Ubuntu)
- Internet access for the first dependency installation

## Install

From the repository root, run as your normal user:

```sh
./config/install.sh
```

This installs the vehicle, [schema](vehicle.schema.json) and
[validator](validate_vehicle.py) in `~/.local/share/vtol26/config/`, without
sudo. The installer creates `venv/` there and installs
[requirements.txt](requirements.txt) into it. Every validator call uses
`venv/bin/python3`; activation is unnecessary. All components resolve this
location from the current user's `HOME`. The Pi image prepares the same isolated
environment for `vtol` on first boot, using its bundled Python packages without
downloading dependencies.

The installer copies missing files and leaves existing files untouched. It
prepares the Python environment first, then validates the repository's
`vehicle.json` before copying configuration files. Validation failure leaves
those files untouched. If the files and environment are ready, installation is
skipped. To replace the installed files and refresh the environment:

```sh
./config/install.sh -f
```

This also replaces any vehicle settings you edited in the installed file.

## Configuration

Edit `~/.local/share/vtol26/config/vehicle.json` to change the active vehicle.
Validate it with:

```sh
"$HOME/.local/share/vtol26/config/venv/bin/python3" -I \
  "$HOME/.local/share/vtol26/config/validate_vehicle.py"
```

Use `./config/install.sh -f` to deploy changes from this directory. Restart the
allocator and simulation after vehicle changes. Rebuild and reflash PX4 when its
actuator settings change, and rebuild the Pi image to include host configuration
changes. Use matching vehicle settings across the HIL setup.

PX4 builds generate actuator constants and record a vehicle snapshot and SHA-256
under `controller/build/<target>/`. Simulation keeps a snapshot with its
generated world. These snapshots are build artifacts; edit the installed
vehicle.

## Interface

World coordinates are NED; body coordinates are FRD. Quaternions use `[w,x,y,z]`
and rotate body vectors into NED. Positions are metres from the CG, forces are
newtons and angles are radians. See the
[model equations](../docs/model/vtol_model.tex).

Actuator array order is fixed:

| Index | Role              | Unit          | PX4 function |
| ----- | ----------------- | ------------- | ------------ |
| 0     | Left motor        | model command | Motor1       |
| 1     | Right motor       | model command | Motor2       |
| 2     | Rear motor        | model command | Motor3       |
| 3     | Left tilt         | rad           | Servo1       |
| 4     | Right tilt        | rad           | Servo2       |
| 5     | Left ruddervator  | rad           | Servo3       |
| 6     | Right ruddervator | rad           | Servo4       |
| 7     | Left aileron      | rad           | Servo5       |
| 8     | Right aileron     | rad           | Servo6       |

Motor thrust is `k_t * command`. Tilt zero means vertical thrust; surface zero
means undeflected. Virtual outputs scale motor ranges to `[0,1]` and servo
ranges to `[-1,1]`. `hil_channel` selects a unique slot from 1–16; physical pin
mapping, PWM conversion, trim and reversal still require calibration.

## Testing

From the repository root:

```sh
"$HOME/.local/share/vtol26/config/venv/bin/python3" -I \
  -m unittest discover -s config/tests -v
```

Tests cover the shared validation rules, generated PX4 constants, C++ loading
and installation with temporary user homes.
