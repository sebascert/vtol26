# Stallion VTOL Project: Architecture and Source of Truth

**Review date:** 2026-10-02\ **Reviewed input:** uploaded
`vtol_integrador(1).zip` and `docs.zip`. The project ZIP excludes `.git`;
`docs.zip` contains copies of 15 root Markdown files.\ **Status convention:**
**Observed** describes files in the reviewed archive. **Proposed** describes
the intended cleanup or future architecture. This document does not claim the
firmware is flight-ready or that the proposed HIL path has been implemented.

## 1. Purpose and scope

The repository is a prototype for a custom C++ VTOL flight-control stack
targeting a SpeedyBee F405 Wing V2 / STM32F405 board. The README describes a
three-rotor VTOL that hovers and transitions to fixed-wing cruise. The
code/configuration currently present is a PlatformIO Arduino firmware
prototype, a separate host-side PSO experiment, two MATLAB/Simulink source
files, partial mechanical CAD, and a set of design/integration notes.

The current repository is **not** the PX4-Autopilot source tree, and the
inspected files do not establish a working Gazebo or HIL integration. PX4 is an
inspiration/reference in parts of the documentation and in names/comments such
as `imu_px4_driver`; PlatformIO has no PX4 dependency.
`src/drivers/imu_px4_driver.cpp` is local project code. It may be compiled as
part of this project by PlatformIO's normal `src/` discovery, but that does not
compile or link upstream PX4.

The project needs one explicit product decision before the architecture can be
treated as settled:

1. **Custom autopilot**: continue the local PlatformIO firmware as the flight
   stack.
2. **PX4 HITL**: run a supported PX4 firmware build on a flight controller and
   connect it to a simulator.
3. **Custom firmware HIL/SIL**: connect this firmware to a simulator through a
   defined sensor/actuator protocol.

These are separate integration paths. Until a decision is recorded, describe
the repository as a **custom VTOL firmware prototype with simulation
research**, not as a completed PX4 HIL system.

## 2. Current architecture (observed)

### Firmware execution path

````text src/main.cpp ├─ sets up Serial and a DPS310 HAL instance ├─ calls
CoordinadorDeTareas::inicializar / ejecutar └─ cycles hard-coded
attitude/thrust test setpoints ↓ src/scheduler.cpp  (CoordinadorDeTareas) ├─
SensorManager → sensor reads and StateEstimator ├─ QuaternionAttitudeController
→ ControlOutput └─ PSO_Allocator → locally stored motor RPM / servo-angle
values ↓ Serial diagnostics ```

The coordinator runs synchronously from Arduino `loop()`. Although
comments/docs describe task frequencies and FreeRTOS-style scheduling,
`platformio.ini` selects the Arduino framework and this archive contains no
explicit FreeRTOS dependency or task creation in the main execution path.

### Firmware modules

| Area                          | Current files
| Observed role / limit
| | ----------------------------- |
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
|
-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
| | Entry point and orchestration | `src/main.cpp`, `include/scheduler.hpp`,
`src/scheduler.cpp`
| Runs the sensor/control/allocation calls and diagnostic output. `main.cpp`
currently drives a fixed test sequence.
| | Sensors                       | `include/sensor_manager.hpp`,
`src/sensor_manager.cpp`, `include/hal_dps310.hpp`, `src/hal_dps310.cpp`,
`include/DPS310.hpp`, `src/DPS310.cpp`
| Sensor manager uses Arduino-oriented dependencies and an estimator. The
archive contains overlapping DPS310 implementations that need consolidation.
| | State and math                | `include/module_data_types.hpp`,
`include/module_interfaces.hpp`, `include/math_types.hpp`,
`include/state_estimator.hpp`, `src/state_estimator.cpp`
| Shared state, interfaces, quaternion representation, and estimation logic.
`math_types.hpp` also defines a quaternion-like type, so duplicate
representations should be reconciled.                                     | |
Attitude control              | `include/quaternion_attitude_controller.hpp`,
`src/quaternion_attitude_controller.cpp`,
`include/attitude_pid_controller.hpp`, `src/attitude_pid_controller.cpp`,
`include/attitude_controller.hpp`, `src/attitude_controller.cpp` | Multiple
controller implementations exist. The coordinator selects
`QuaternionAttitudeController`; the other controllers are not selected by that
path.                                                                 | |
Allocation                    | `include/pso_allocator.hpp`,
`src/pso_allocator.cpp`
| The embedded class currently calls `solve_simple`; its own comments say a
real PSO solver is future work. Its outputs are copied and printed, but the
reviewed source contains no motor PWM/ESC or servo output driver. | |
PX4-inspired IMU code         | `include/imu_px4_driver.hpp`,
`src/drivers/imu_px4_driver.cpp`
| Local implementation/commentary references PX4. The project build does not
fetch or link the PX4 repository. Confirm whether this driver is used, and
document provenance/license if code was copied or adapted.        | | Force
model                   | `include/aero_force_model.hpp`,
`src/control/aero_force_model.cpp`
| A model component exists, but it is not shown in the coordinator's active
runtime path.
| | Configuration                 | `include/config.h`, `platformio.ini`
| Hardware pins, sensor constants, PlatformIO board/framework/dependencies.
Some config flags and stated hardware disagree with README and source use.
|

### Simulation, experiments, hardware, tests

- `sim/PSO/` is a host-side C++/Python optimization experiment with JSON/CSV
outputs. It is separate from the embedded `src/pso_allocator.cpp`; the embedded
allocator is currently a simplified heuristic, not this host experiment running
on the board.
- `sim/simulink/` contains `construir_planta_vtol.m` and `fuerzas_v4.m`.
`docs/modeling/README.md` describes additional MATLAB files that are not in the
archive. `VTOL_SIMULINK.slx` is at repository root rather than beside the model
scripts.
- `hardware/cad/mounts/*.step` contains mechanical STEP models. No reviewed
URDF, SDF, Gazebo model, or mesh export is present. The CAD is not consumed by
the firmware build.
- `test/` contains small stubs and a PowerShell smoke script, not a complete
executable test suite. Some documented categories have no corresponding
runnable tests.
- `mainsraros/` contains older/experimental C++ files outside PlatformIO's
default `src/` directory. They should be classified before moving or deleting;
they are not part of the normal firmware source discovery based on the current
config.
- `VTOL/docs/SENSORES_INTEGRATION.md` is a separate, generic sensor note with
placeholder diagram text and pin/frequency values that conflict with current
configuration. It should not be treated as authoritative.

## 3. Configuration and reality checks

The current `platformio.ini` sets `platform = ststm32`, `board =
genericSTM32F405RG`, `framework = arduino`, `-std=gnu++17`, and dependencies
for ICM42688, TinyGPSPlus, and QMC5883LCompass. `include/config.h` configures
an ICM42688, DPS310, GPS, and pitot pins/flags. The README instead lists an
MPU6000, pitot, microSD, FreeRTOS, and a QP allocator. These descriptions
cannot all be accepted as current implementation facts.

Before updating the architecture statement, verify the exact board revision and
peripherals fitted to the aircraft, the actual sensor models and bus/pins,
whether GPS/pitot/magnetometer are connected, and what firmware framework/task
scheduler is intended. Keep one approved hardware baseline in
`docs/reference/hardware.md` and make README and configuration agree with it.

Several claims in the root guides are aspirational or stale compared with the
source:

- `changes_summary.md` reports a successful build and exposes APIs that are
absent from the current coordinator interface.
- `quick_start.md` shows examples calling getters/setters that are not declared
in the current `scheduler.hpp`.
- `README.md` claims QP allocation, while the embedded allocator source says it
uses a simple solver for now.
- The output pipeline ends at diagnostic arrays; there is no actuator output
layer in the reviewed source.
- `test/` does not substantiate the broad completion/readiness claims in
`verification_checklist.md`.
- The docs mention Gazebo/HIL, but this archive has no Gazebo plugin/model,
transport bridge, HIL protocol implementation, or test procedure/results.

Treat these as documentation defects to resolve. Do not repeat historical
“compiled successfully” statements as a current build result unless they can be
reproduced from this exact revision and environment.

## 4. Recommended repository layout

Adopt a single clear top-level structure. Migrate in small commits, keeping the
PlatformIO build green after each move.

```text vtol_integrador/ ├── README.md                         # short project
description and entry links ├── platformio.ini                    # board,
framework, dependencies, build/test envs ├── .gitignore ├── firmware/ │   ├──
include/                      # public interfaces/types/config │   └── src/ │
├── app/                      # main, orchestration │       ├── drivers/
# sensor and actuator hardware drivers │       ├── sensors/                  #
sensor manager and sensor adapters │       ├── estimation/ │       ├── control/
│       ├── allocation/ │       ├── communications/           # only
implemented protocols │       └── logging/ ├── test/ │   ├── native/
# host unit tests for math/control/model │   ├── embedded/
# board or hardware smoke tests │   └── fixtures/ ├── sim/ │   ├── simulink/
# source model, scripts, parameter sets │   ├── pso_host/                     #
host optimizer and pinned config │   └── gazebo/                       # only
after a reproducible model exists ├── hardware/ │   ├── cad/airframe/ │   ├──
cad/mounts/ │   ├── electronics/ │   ├── bom/ │   └── manufacturing/ ├── tools/
# reproducible data/plot/conversion utilities └── docs/ ├── architecture/ │
├── PROJECT_SOURCE_OF_TRUTH.md │   └── decisions/ ├── design/
# control, estimation, allocation, conventions ├── guides/
# build, flash, bring-up, test instructions ├── simulation/                   #
model assumptions, SIL/HIL procedures/results ├── reference/
# approved hardware, interfaces, glossary └── archive/                      #
superseded material retained for traceability ```

This is the **target**, not a claim that these directories already exist. Until
the firmware is moved, preserve PlatformIO's current root `src/` and `include/`
defaults. If adopting `firmware/`, add explicit `src_dir = firmware/src`,
`include_dir = firmware/include`, and `test_dir = test` settings, then verify
that all intended translation units are compiled and that no old source is
silently dropped.

### Documentation ownership rules

1. Keep one top-level `README.md` as the landing page; it links to canonical
docs and gives only verified quick-start commands.
2. Put the authoritative architecture and system boundary in
`docs/architecture/PROJECT_SOURCE_OF_TRUTH.md` (this document after it is moved
there).
3. Put detailed build/flash steps in one `docs/guides/build-and-flash.md`;
avoid duplicating them in multiple integration guides.
4. Put hardware values, axis conventions, units, frame IDs, and pin maps in
versioned reference documents. Code constants should link to those references.
5. Record major choices as short Architecture Decision Records (ADRs): custom
autopilot vs PX4, simulator/HIL interface, estimator, controller, allocator,
and actuator protocol.
6. Every guide should say its status (`current`, `proposal`, or `historical`)
and identify the firmware revision/environment it was checked against.

## 5. Cleanup recommendations

### P0 — establish one truthful baseline

- Decide which integration path is the goal: custom stack, PX4 HITL, or custom
firmware HIL/SIL. Record it in an ADR.
- Update README hardware, flight-control, allocator, scheduling, and HIL claims
to match measured hardware and current code.
- Label PX4-related guides as **proposal/reference** unless their steps have
been executed. Clarify that local “PX4-style” files do not make this a PX4
build.
- Replace `index.md` and `index_new.md` with one maintained docs index, or
remove the obsolete one after checking incoming links.
- Mark historical status in `changes_summary.md`, `compilation_fixes.md`, and
`verification_checklist.md`; remove pass/readiness claims that are not tied to
reproducible evidence.

### P1 — clarify and modularize firmware

- Identify the one authoritative sensor stack. Resolve duplicate/overlapping
DPS310 implementations and the unused/legacy `src/Sensores.h`.
- Document which estimator/controller/allocator is active. Remove unused
implementations only after checking references and preserving useful
experiments.
- Add a real actuator abstraction and driver only after motor/servo mapping,
units, failsafe behavior, arming, and hardware limits are specified. Until
then, call allocation output “computed command values,” not actuator output.
- Decide whether `IMU_PX4Driver` is used. If retained, rename to communicate
provenance accurately and add attribution/license review for any
upstream-derived code.
- Separate pure control/math code from Arduino headers where practical so it
can be unit-tested on a host.
- Define frame and sign conventions once (body/inertial frames, NED/ENU,
quaternion ordering, Euler order, SI units, thrust sign) and test conversions.

### P2 — organize simulation, CAD, and tests

- Move `VTOL_SIMULINK.slx` beside its Simulink sources and reconcile
`docs/modeling/README.md` with files that actually exist.
- Separate host PSO experiment and embedded allocator in names/docs; keep
generated `pso_data.json` and `particles.csv` in a results/output directory
unless they are intentional reference fixtures.
- Add a hardware README for STEP files: component name, revision, units,
coordinate frame, mass/material if known, and whether the file is source CAD or
an exported derivative. Keep original CAD; add simplified collision meshes only
for a simulator when needed.
- Replace test stubs with executable tests and explicit pass/fail criteria.
Start with quaternion math, estimator input validation, controller limits,
allocator saturation, and sensor timeout behavior.
- Add simulator files/procedures only for a concrete chosen integration. A HIL
document should list firmware and simulator versions, wiring/transport, message
mapping, start sequence, expected signals, pass criteria, and known
limitations.

### P3 — make the repository reproducible

- Pin PlatformIO platform/library versions (exact or controlled ranges),
document the tested PlatformIO Core version and board upload procedure, and
capture clean build output in CI.
- Add a `pio run` CI job and host test job. Do not label a checklist complete
without a passing run linked to a commit.
- Extend `.gitignore` for generated/cache/build data such as `.mypy_cache/`,
`__pycache__/`, `sim/simulink/slprj/`, MATLAB autosaves/codegen output,
`.pio/`, and local editor state. Retain intentional model sources and
explicitly curated result files.
- Keep `mainsraros/` only if its purpose is known; otherwise move it to
`archive/legacy/` with a short README and a decision date. Do not delete it
solely because it is outside the build path.
- Avoid checking in duplicate documentation archives such as `docs.zip`;
distribute source docs or tagged project releases instead. Excluding `.git`
from a share ZIP is reasonable, but it does not replace source control.

## 6. Build and validation truth

The intended firmware build entry is PlatformIO from the repository root:

```bash pio run -e f405_wing_v2 ```

The upload target and serial monitor should be documented only after verifying
the selected board and upload protocol on the actual board. The uploaded
project archive was reviewed statically; PlatformIO was not available in the
review environment, so no new build or board test is claimed here.

For every release candidate, capture:

- commit ID, PlatformIO Core, platform, framework and library versions;
- clean build result and memory report;
- test results and test hardware/configuration;
- board bring-up checks with propellers removed;
- sensor health and timing evidence;
- actuator direction/range/failsafe checks;
- simulation/HIL trace and measurable acceptance criteria, if that path is
selected.

## 7. Ownership and update policy for this source of truth

Update this document in the same change that alters module boundaries,
supported hardware, build targets, data/control flow, units/frames, simulator
interfaces, or validation status. Keep the “Observed” section tied to what is
in the repository; move completed proposals into observed facts only after code
and evidence exist. Put dated implementation history in release notes or ADRs
instead of allowing multiple competing architecture descriptions to accumulate.

## 8. Evidence map in the reviewed archive

- Project claims: `README.md`, `platformio.ini`, `include/config.h`.
- Runtime composition: `src/main.cpp`, `include/scheduler.hpp`,
`src/scheduler.cpp`.
- Active attitude controller: `src/scheduler.cpp`,
`src/quaternion_attitude_controller.cpp`.
- Allocation behavior: `src/pso_allocator.cpp`.
- Host simulation: `sim/PSO/README.md`, `sim/PSO/pso_environment.cpp`,
`sim/simulink/*.m`, `VTOL_SIMULINK.slx`.
- Documentation status/conflicts: root `INDEX*.md`, `quick_start.md`,
`changes_summary.md`, `compilation_fixes.md`, `verification_checklist.md`, and
`docs/**`.
- Test coverage: `test/**`.
- Mechanical sources: `hardware/cad/mounts/*.step`.
````
