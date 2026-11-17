# Gazebo HIL plan: Raspberry Pi PSO and Pixhawk 6C

Prepared: 2026-11-03. Status: researched design and implementation plan; no HIL
bridge, firmware changes, parameter changes, or simulation runs were performed
for this document. References to PX4 `main` describe inspected upstream source,
not the firmware currently installed on the Pixhawk.

Update, 2026-11-13: the [controller firmware prototype](../../../controller/VTOL.md)
now builds against pinned PX4 v1.17.0, publishes validated Pi candidates to
`pso_allocation`, and includes `pwm_out_sim` on FMUv6C. Hardware receipt,
allocation application and the Harmonic HIL bridge remain unverified or pending.
The original inspection below describes the earlier integration state.

## Proposed outcome

**Current input clarification:** the running PSO already receives and uses PX4
telemetry: position, velocity, attitude, angular rates and airspeed. It does
not receive PX4's demanded thrust/torque or a live PX4 target. Its pose target
comes from the companion's startup JSON configuration. These are different
interfaces; the missing demanded-wrench interface does not mean telemetry is
absent. See [the actual solver call](../../../companion/allocator/src/pso_interface.cpp).

Run vehicle physics on the Ubuntu laptop, PX4 on the real Pixhawk, and PSO on
the real Raspberry Pi. Send a target, record what PX4 actually accepts and
outputs, apply those outputs to the simulated vehicle, and feed simulated
sensors back into PX4. This closes the loop:

```text
target -> PX4 controllers -> Pi PSO -> PX4 output handling -> Gazebo plant
              ^                                               |
              +---------- estimated state <- HIL sensors ------+
```

**This is achievable, but the current project does not yet close that loop.**
Three pieces need development: a modern-Gazebo HIL bridge, an actuator-driven
vehicle plant, and a control interface through which PX4 can apply PSO results.
The role of PSO must be decided before implementing the last piece.

Recommended sequence: establish a baseline HIL loop, observe PSO alongside it,
then integrate PSO into the output path. Start with hover and small setpoint
steps; add transition and forward flight after the hover model works.

## What was inspected

The initial reference was inspected read-only at `~/projects/vtol/pvtol_gz`.
Its source and model assets have since been copied into
[`simulation/`](../../../simulation/README.md) in this workspace; the original
reference repository remains unchanged. The source links below point to that
local copy. This reorganization does not implement the proposed HIL interfaces.

| Evidence | Finding and implication |
| --- | --- |
| [Companion main loop](../../../companion/allocator/src/main.cpp), [defaults](../../../companion/allocator/src/application_config.hpp) | Consumes PX4 telemetry, solves at a default period of 1000 ms, and emits candidates. A one-second solve period is a bench setting, not an established flight-control rate. |
| [MAVLink sender](../../../companion/allocator/src/mavlink_sender.cpp) | Sends `DEBUG_FLOAT_ARRAY`, named `PSO_ALLOC`. It does not command PX4 actuators. |
| [PSO API](../../../companion/allocator/src/pso/pso_runtime.hpp), [configuration](../../../companion/allocator/src/pso/config.json) | Optimizes nine actuator values against a startup pose target. No online target-update or demanded-wrench API exists in this runtime. |
| [Telemetry conversion](../../../companion/allocator/src/pso_interface.cpp) | Negates NED Z/VZ but passes attitude and body rates through. A complete frame convention is still unresolved. |
| [Gazebo build](<../../../simulation/CMakeLists.txt>) | Requires `gz-sim8`: modern Gazebo Harmonic. The inspected laptop runs Ubuntu 24.04.5 and reports Gazebo Sim 8.15.0. |
| [Gazebo controller](<../../../simulation/src/pvtol_controller.cc>) | Reads simulated ground truth, runs its own quaternion controller, and calls `AddWorldWrench` with ideal thrust and torque. There is no MAVLink endpoint. |
| [Gazebo model](<../../../simulation/models/stallion_vtol/model.sdf>) | One rigid body with visual meshes, a spherical collision, and an autonomous controller plugin. No actuator joints or sensor elements. Meshes alone do not model motor, servo, or aerodynamic behavior. |
| [Gazebo world](<../../../simulation/worlds/pvtol.sdf>) | Uses a 1 ms physics step and requests real-time factor 1.0. Starts at `(8,-6,1)`; the model's local controller targets `(0,0,3.5)`. |
| USB bench test earlier in this session | Pi Ubuntu 22.04.5, Pixhawk USB, 14 allocations in 15 seconds, finite values within bounds. This verified telemetry and numerical execution, not HIL or actuator application. |

Upstream PX4 currently handles `DEBUG_FLOAT_ARRAY` as a debug topic. An
allocation appearing in a debug log is therefore not evidence that it reached
the actuator pipeline. See the [PX4 MAVLink receiver](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp).

### Comparison with `legacy/`

The following source paths were inspected after the initial plan. They are
separate implementations and do not establish the same integration status:

| Implementation | Input actually used | Output/integration status |
| --- | --- | --- |
| Current `companion/allocator` | Live PX4 state telemetry plus the startup pose target; [state values are passed into `optimizer_.solve`](../../../companion/allocator/src/pso_interface.cpp) | Runs the shared PSO and transmits diagnostic candidates. No PX4 demanded-wrench input or actuator application. |
| [`legacy/mavlink_uart`](../../../legacy/mavlink_uart/src/main.cpp) | The receiver decodes PX4 telemetry and passes `DroneState` to `runPSO`, but [`runPSO` ignores that argument](../../../legacy/mavlink_uart/src/pso_interface.cpp) | Returns hardcoded values `[1000,1000,1000,0.1,0.1,0,0,0,0]`. [`sendPSOCommand`](../../../legacy/mavlink_uart/src/mavlink_sender.cpp) only prints them; it does not encode/write actuator results over MAVLink. Its heartbeat sender is separate. |
| [`legacy/vtol_integrador/sim/PSO`](../../../legacy/vtol_integrador/sim/PSO/pso_environment.cpp) | Standalone model/configuration, with internally initialized state; no live PX4 telemetry interface | Contains the swarm experiment, not a complete Pi/PX4 runtime. Its README's intended architecture is not evidence of a working command link. |
| [`legacy/stm32-firmware`](../../../legacy/stm32-firmware/src/scheduler.cpp) | A local quaternion controller supplies roll/pitch/yaw torque and thrust to `allocator_->solve` | This is the older standalone STM32 architecture, not PX4. [`PSO_Allocator::solve`](../../../legacy/stm32-firmware/src/pso_allocator.cpp) calls `solve_simple`, a hand-coded mixer rather than a particle-swarm optimizer. |

Thus the current companion has a real telemetry-to-PSO path that the legacy
MAVLink placeholder did not implement. The old STM32 design illustrates a
controller-to-allocator boundary, but does not supply a working external PX4
wrench-to-PSO integration. This comparison is source inspection, not a new
runtime or hardware test of the legacy code.

The subsequent [backup and saved-history audit](backup_pso_px4_audit.md) found
a real six-variable demand-based PSO in archived reflog commit `44e9ee2`.
That implementation belongs to standalone STM32/Arduino firmware, not a
Pi/PX4 interface. It is a potentially useful algorithm reference; no complete
external PX4 integration was found in the audited backups.

## Gazebo compatibility: choose the route explicitly

The [official HITL guide](https://docs.px4.io/main/en/simulation/hitl) describes
Gazebo **Classic**, identifies HITL as community maintained, and warns about
compatibility with current PX4 releases. Its example airframes are HIL quad X
(`1001`) and standard VTOL (`1002`). It requires checking `pwm_out_sim` support.
These are useful baseline examples, not a ready-made Stallion configuration.

The supplied [UTMSYS tutorial](https://wiki.utmsys.org/en/Algorithm_Development/SIMULATOR/HITL/PX4%20HITL%20Tutorial/)
also describes Classic, including legacy jMAVSim steps. Some command blocks
contain translation artifacts. Use its architecture as background and verify
commands against the selected PX4 source revision.

Modern Gazebo SITL uses PX4's native Gazebo transport, rather than the MAVLink
simulator connection. Running `make px4_sitl gz_x500` starts a software PX4
instance; it does not put the physical Pixhawk in the loop. See
[PX4's simulation interfaces](https://docs.px4.io/main/en/simulation/) and
[modern Gazebo setup](https://docs.px4.io/main/en/sim_gazebo_gz/).

| Route | Reuse | Work and tradeoff |
| --- | --- | --- |
| **Keep Harmonic; recommended for this reference** | Existing Gazebo runtime, world, meshes, and plugin structure | Implement/adapt an external Gazebo-to-MAVLink-HIL bridge. The stock SITL `gz_bridge` is not a USB HIL adapter. Compatibility is a milestone to prove. |
| Use Classic for an initial HIL baseline | Documented HIL model and MAVLink plugin | Use an isolated compatible environment and pinned PX4 revision. Porting the Harmonic plugin is additional work; do not expect a Classic plugin to load into `gz sim`. |

The stock [Gazebo bridge build](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/gz_bridge/CMakeLists.txt)
links Gazebo transport libraries. Treat a new external HIL adapter as an
engineering task, not a launch flag. Do not install a second simulator stack
until the route is chosen.

## Connections and ownership

Preferred final bench topology:

```text
Ubuntu laptop                         Pixhawk 6C                 Raspberry Pi
Gazebo + HIL bridge <---- USB -------> PX4 <--- TELEM2 UART ---> PSO
       |
       +--- routed MAVLink ---> QGroundControl / target sender

Ubuntu laptop <----------- Ethernet / SSH --------------------> Raspberry Pi
192.168.76.1                                                   192.168.76.2
```

This separates high-rate simulated sensor traffic from the companion link.
Use the existing [TELEM2 wiring guide](raspberry_pi_pixhawk_6c_wiring.md): TX/RX
crossed, common ground, separate power, no TELEM2 +5 V connection to the Pi.
Confirm three-wire/no-flow-control settings on both ends. Begin with the tested
bench baud setting; raise it only after the rate budget and UART stability are
verified.

The current **Pi USB -> Pixhawk** cable can also be retained, but requires a
different software topology: one router/bridge on the Pi owns USB, and the
laptop exchanges HIL traffic over Ethernet. The companion then needs UDP
transport or a tested local pseudo-terminal adapter. It currently opens serial
devices only. Do not let a router and the existing companion independently read
the same USB port. Ethernet connectivity alone does not multiplex that port.

Give QGroundControl a forwarded MAVLink endpoint; it must not compete with the
HIL bridge for USB. Preserve system/component IDs, direct traffic to explicit
endpoints, and avoid forwarding loops. Log the final routing table. The chosen
serial/UDP topology must be identical in the acceptance run and its replay.

## Decide what PSO means in this experiment

The current solver is a pose-target optimizer. Calling it an allocator does
not make it consume PX4 controller demands. Choose one of these experiments:

| Experiment | Target goes to | PSO input and output | PX4 responsibility |
| --- | --- | --- | --- |
| **PX4 controllers + PSO allocator: recommended end state** | PX4 mission/position controller | Desired body force/torque plus estimated state -> nine actuator commands | Estimation, navigation, attitude/rate control, validated application and failsafe handling |
| Pose-target PSO, preserving the current research objective | Pi target interface | Estimated pose/velocity/rates plus pose target -> nine actuator commands | Estimation, explicit output gating and HIL output production; competing controllers must not overwrite PSO |
| PSO observation alongside a stock HIL baseline | PX4 | Telemetry -> logged PSO candidates, never applied | Complete control pipeline; useful comparison and latency measurement |

The third row is an integration milestone, not completion of PSO-controlled
HIL. Forwarding Pi debug output straight to Gazebo would also bypass PX4's
application path and would not satisfy the requested output-loop experiment.

PX4 separates controller torque/thrust demands from geometry-dependent
allocation; see [control allocation](https://docs.px4.io/main/en/concept/control_allocation).
For the recommended end state, change the solver objective to demanded-wrench
tracking with actuator constraints and effort/slew penalties. Reuse its bounded
search and validated model pieces. Keep the standalone pose experiment working.

The PX4 [torque](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleTorqueSetpoint.msg)
and [thrust](https://github.com/PX4/PX4-Autopilot/blob/main/msg/VehicleThrustSetpoint.msg)
setpoints are normalized. Define their conversion/effectiveness model before
comparing them with forces in N and moments in N m. Do not relabel normalized
values as physical units.

For pose-target control, a release-compatible ROS 2 direct-actuator interface
is an alternative to a custom MAVLink receiver/module. It adds transport,
message-version, topic-export and Offboard configuration work. PX4 documents
`ActuatorMotors`/`ActuatorServos` with direct-actuator Offboard control; confirm
availability in the actual firmware. See [Offboard interfaces](https://docs.px4.io/main/en/flight_modes/offboard).
The existing MAVLink companion does not implement that interface.

## Interfaces to implement

### Target input

For the PX4-controller path, start with a single local position + yaw target
sent through an existing mission/Offboard client. Define the local origin,
frame, units and setpoint ID in each test record. An Offboard client must keep
streaming supported setpoints and implement mode/loss handling; the 1 Hz PSO
debug heartbeat is not a substitute. Use a separate setpoint publisher so its
liveness does not depend on optimizer completion. PX4 requires a stream above
2 Hz; propose 20 Hz initially, subject to the selected vehicle's supported
setpoint interface. [Offboard requirements](https://docs.px4.io/main/en/flight_modes/offboard)

For the pose-target path, the smallest first experiment is a target/config file
copied over SSH before each run, followed by a companion restart. Record its
hash and acknowledge startup. Editing the file while the process is running
does not update its loaded optimizer. If changing targets mid-run is required,
add one validated live-target interface with a target ID and acknowledgement;
do not independently send different targets to PX4 and the Pi.

### Gazebo sensors -> PX4

Build a bridge using the MAVLink headers already used by the project and the
installed Gazebo libraries. Prefer one process/plugin with a small, explicit
configuration over adding ROS solely as a message relay.

The initial sensor contract is `HIL_SENSOR` plus `HIL_GPS`: body acceleration,
angular rate, magnetic field and pressure, and synthetic GNSS from a defined
world origin. Match the MAVLink units, timestamps and updated-field mask;
pressure fields use hPa, while GPS fields have their specified integer scales.
See the [MAVLink message definitions](https://mavlink.io/en/messages/common.html#HIL_SENSOR).

Use Gazebo sensor systems or a carefully tested sensor model; the reference
world currently provides neither. The [Gazebo sensor guide](https://gazebosim.org/docs/harmonic/sensors/)
shows how sensor elements and their systems are configured. A stationary IMU
must report specific force, not zero world acceleration: at a sensor located
at the body origin, `f_body = R_world_to_body * (a_world - gravity_world)`.
Account for sensor location if using an offset IMU. Test gravity signs before
letting PX4 estimate attitude.

Generate pressure altitude/magnetics/GPS consistently from the same origin.
Generate airspeed from velocity relative to simulated wind, not automatically
from groundspeed. Confirm PX4 emits fresh `VFR_HUD` even at zero airspeed; the
current companion requires it along with heartbeat, attitude and local position.
Never mask a missing input by substituting unrelated real-board telemetry.

Inspect the pinned receiver before deciding to use `HIL_STATE_QUATERNION`.
The inspected upstream [receiver implementation](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/mavlink_receiver.cpp)
can publish attitude and local position directly from that message. For an
estimator-in-the-loop experiment, keep ground truth in the laptop log and feed
raw simulated sensors instead; avoid accidentally mixing estimator output
with a direct truth injection. Hardware HIL inputs are gated by HIL mode.

### PX4 <-> Pi control contract

For the existing MAVLink stack, propose a small explicit control contract and
a PX4 integration module. Message names below are proposed, not implemented:

| Message | Required content |
| --- | --- |
| Allocation request, PX4 -> Pi | Protocol/model version, PX4 boot/session ID, request sequence, state sample ID, demand timestamp, thrust/torque and their declared units, relevant estimated state, deadline |
| Allocation result, Pi -> PX4 | Matching session/request/sample IDs, nine values in a declared order/representation, validity, computation time and diagnostic fitness |
| Application status, PX4 -> logger/Pi | Accepted/rejected request ID, actual applied values, clamp/slew changes, rejection reason and active fallback/mode |

Bind the message source to the expected Pi system/component, check lengths,
finite values and limits, reject stale/out-of-order/replayed results, and apply
results non-blockingly. Use PX4's own request timestamp for age checks; do not
subtract unrelated Pi/PX4 clocks. Expire a request at a locally measured
deadline. A result after that deadline must not become valid by arrival alone.

Choose exactly one active producer of motor/servo commands. Integrate selection
with the allocator/output path so stock allocation cannot overwrite PSO while
both appear to run successfully. Retain arming, disarming and explicit HIL-only
gating for this prototype. Define a tested loss action: for the first isolated
HIL trial, reject/stop the experiment and safe the simulated outputs. A later
fallback allocator needs matched geometry, state and bumpless-transfer tests;
it is not automatically safe just because it is the stock allocator.

A custom MAVLink dialect requires matching generated definitions on every
endpoint that interprets it and appropriate receiver/stream integration. Keep
that maintenance cost explicit. Debug arrays can remain diagnostics; they
should not silently become the command contract. See [custom MAVLink integration](https://docs.px4.io/main/en/mavlink/custom_messages).

### PX4 outputs -> Gazebo

Use `HIL_ACTUATOR_CONTROLS` as the logical HIL output boundary. It supplies a
16-element control array and mode information; indices are defined by the
airframe mapping, not by the PSO array's order.
[MAVLink output definition](https://mavlink.io/en/messages/common.html#HIL_ACTUATOR_CONTROLS)

In inspected PX4 `main`, [the output stream](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/mavlink/streams/HIL_ACTUATOR_CONTROLS.hpp)
reads `actuator_outputs_sim`. [The simulated output driver](https://github.com/PX4/PX4-Autopilot/blob/main/src/modules/simulation/pwm_out_sim/PWMSim.cpp)
normalizes nonreversible motors to `[0,1]` and other enabled functions to
`[-1,1]`. Check armed/disarmed, invalid, unused and failsafe behavior against the
pinned revision rather than treating every received float as usable thrust.
Keep the normal HIL output driver in the loop; publishing a second synthetic
output array beside it would create ambiguous ownership.

Proposed mapping for a dedicated Stallion HIL airframe:

| PSO index | Function | PX4 logical function | Proposed HIL array index |
| --- | --- | --- | --- |
| 0 | Left rotor | Motor 1 | 0 |
| 1 | Right rotor | Motor 2 | 1 |
| 2 | Rear rotor | Motor 3 | 2 |
| 3 | Left tilt | Servo 1 | 3 |
| 4 | Right tilt | Servo 2 | 4 |
| 5 | Left ruddervator | Servo 3 | 5 |
| 6 | Right ruddervator | Servo 4 | 6 |
| 7 | Left aileron | Servo 5 | 7 |
| 8 | Right aileron | Servo 6 | 8 |

This is a proposal to encode and test, not an existing assignment. Current
upstream uses `HIL_ACT_FUNCn` mappings; inspect the selected release. Its
[standard VTOL HIL airframe](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/airframes/1002_standard_vtol.hil)
defines five motors and three control surfaces, unlike this vehicle's three
rotors and six servos. Do not select airframe `1002` and assume it models
Stallion correctly.

Logical HIL outputs test firmware/software behavior. Capturing actual PWM or
DShot waveforms is a separate electrical-timing experiment requiring a capture
interface; it is not necessary for this first HIL loop.

## Vehicle physics and frames must agree

Reuse the reference world/model organization and visuals. Replace or disable
its autonomous quaternion controller when HIL owns the vehicle. Otherwise a
local ideal controller can stabilize the model independently of PX4 and make a
broken hardware loop appear successful.

The smallest useful plant can still be a single rigid body. Decode all nine
PX4 outputs, convert them to physical motor/servo states, compute each force
and moment including its lever arm, then apply the summed world wrench through
Gazebo. Let Gazebo integrate motion and apply gravity once. Detailed moving
joint visuals are optional initially; the nine independent actuator effects
are not. Include motor lag, servo slew, limits and stale-command behavior as
the first model is validated. Forward-flight testing additionally needs
credible aerodynamic and transition behavior.

Store the normalization/physical conversion beside the channel map. Motor
fractions require a declared thrust/speed curve. A servo value needs neutral,
travel and sign: an illustrative centered deflection is `angle = 0.5 * s`
radians for `s` in `[-1,1]`, while a tilt range `[0, pi/2]` would use
`angle = (s + 1) * pi/4` before trims/reversals. These illustrate the current
bounds, not measured calibration. Account for PX4 trims/reversals exactly once.

Resolve these concrete mismatches before attempting a hover claim:

| Quantity | Current Gazebo reference | Current companion model |
| --- | --- | --- |
| Mass | 1.3 kg | 4.0 kg |
| Diagonal inertia | `(0.0325,0.0325,0.0325)` kg m² | `(0.12,0.15,0.22)` kg m² |
| Target | World position `(0,0,3.5)` | Model position `(0,0,10)` |
| Propulsion | Ideal controller wrench, thrust limited to 30 N | Linear `0.00024 * omega`, rotor commands bounded by 2000 |
| Geometry | One body plus meshes | Hardcoded force application points and simplified aerodynamics |

With the companion's current bounds, maximum rotor thrust is
`3 * 2000 * 0.00024 = 1.44 N`, versus `4 * 9.81 = 39.24 N` weight. It cannot
hover at zero airspeed under that model. Even adopting the reference's 1.3 kg
mass would not fix that thrust shortfall. Choose measured parameters or an
explicitly declared synthetic HIL model with adequate authority; do not tune
PSO iteration counts to compensate for an impossible plant.

Declare one world/body convention and convert at boundaries. If Gazebo uses
ENU world and FLU body, use:

```text
p_NED = [dy, dx, -dz], where [dx,dy,dz] = p_Gazebo - chosen_origin_Gazebo
v_NED = [v_ENU.y, v_ENU.x, -v_ENU.z]
v_FRD = [v_FLU.x, -v_FLU.y, -v_FLU.z]
```

Transform rotations and angular rates consistently; a Z sign change is not a
complete ENU/NED and FLU/FRD conversion. Audit the mesh/body axes and the PSO
model's force axes before assuming those conventions. The reference initially
spawns away from the origin; PX4's estimator origin need not equal the Gazebo
world origin. Record the transform and verify it using axis-specific motions.

An early plant may reuse model equations for interface tests, but validation
must include independent physical checks/parameters. Optimizer and simulator
agreeing with the same modeling mistake is not evidence of aircraft accuracy.

## Rates, clocks, and loss handling

Treat these as initial engineering budgets, not validated operating settings:

| Path | Initial proposal |
| --- | --- |
| Gazebo physics | Retain 1 ms steps; measure real-time factor |
| HIL IMU | 250 Hz to begin; verify estimator requirements and jitter |
| HIL GPS | 5–10 Hz |
| Magnetometer/barometer | Schedule appropriate lower rates using correct updated flags |
| PX4 output stream | 100–250 Hz, matched to selected control/update schedule |
| Companion estimated state | Rates that support the chosen optimizer period; not just the existing three-second freshness check |
| PSO in observation mode | Keep the existing 1 Hz setting initially and measure timing |
| PSO applying outputs | Choose only after measuring end-to-end deadlines and closed-loop stability |

Benchmark solve duration and transport/apply latency on the Pi, including
p95/p99, observed maxima and deadline misses. A faster command publication
timer does not make an old optimization result fresh. Preserve sample IDs,
account for state skew, and reject commands based on source-state age as well
as receipt time. The existing 3000 ms telemetry timeout is insufficient as an
inner-loop command deadline.

At 57600 baud with 8N1, each direction carries at most 5760 bytes/s before
headroom. A hypothetical 80-byte frame at 100 Hz needs 8000 bytes/s in its
direction, excluding telemetry. Budget request, response and telemetry
separately. Existing serial code supports rates up to 230400 where the platform
provides them, but UART clock stability and actual achievable timing still
need testing. Do not assume the bench's 57600 default supports the final loop.

Begin with wall-clock-paced HIL. The real Pixhawk and Pi have running clocks;
an SDF request for real-time factor 1 does not enforce deterministic pacing or
lockstep. On pause, reset, severe slowdown or disconnect: invalidate cached
commands, stop the experiment, and reinitialize sessions/state before resuming.
Do not continue feeding frozen measurements as if they were fresh. Reset
Gazebo, PX4 and Pi run state coherently between repeatable trials.

## Implementation milestones and pass criteria

1. **Freeze the experiment and inventory the Pixhawk.** Decide controller
   ownership and transport. Save firmware version/hash, board target, parameter
   backup, Gazebo version, model/config hashes and the active source revisions.
   In the PX4 console, inspect `ver all`, `param show SYS_AUTOSTART`,
   `param show SYS_HITL`, `pwm_out_sim status`, and `mavlink status`.
   **Pass:** a recorded compatible configuration and confirmed HIL module
   availability; “not running” is distinct from “command not found.”

2. **Prove the HIL sensor/output connection independently of PSO.** For
   Harmonic, implement the bridge first against a simple known actuator plant;
   alternatively establish the documented Classic baseline in isolation.
   Configure the actual board for external HIL using its pinned startup logic.
   **Pass:** simulated motion changes PX4 estimates, physical board motion does
   not contaminate them, and changing PX4 simulated outputs changes the plant.
   No laptop PX4 SITL process may be mistaken for the board.

3. **Build the nine-actuator Stallion plant and output mapping.** Reuse the
   reference visuals, remove its autonomous controller, set consistent mass,
   inertia and actuator physics, and configure a matching HIL airframe.
   **Pass:** one-channel-at-a-time impulses produce the expected signs and
   forces/moments; zero, saturation, gravity and trim/hover checks pass. Test
   normalized-to-physical conversion in both directions.

4. **Run the Pi in observation mode.** Deliver the required PX4 estimated
   telemetry and a recorded target, log PSO candidates and benchmark latency.
   Use a viable baseline controller/allocator or scripted plant tests according
   to which model is ready. **Pass:** reproducible input/output logs, measured
   deadlines and identified authority/frame limitations. This milestone does
   not establish that PSO controls the model.

5. **Implement the selected PSO application interface.** For allocator mode,
   add demand export, wrench objective, validated response handling and output
   ownership. For pose mode, add its target input and direct-actuator pathway.
   Keep the experiment gated to HIL initially. **Pass:** deliberately changing a
   valid PSO result changes the corresponding PX4 output and Gazebo actuator;
   stale, malformed and out-of-order results do not. Log proposed, accepted
   and applied values separately.

6. **Close the loop on target steps.** Start near a feasible hover trim,
   then test small vertical, horizontal and yaw steps separately. Define the
   acceptance envelope before each run. **Pass:** bounded attitude/actuators,
   agreed tracking/settling error, and acceptable measured latency for a
   recorded duration. Compare against the baseline on the same plant and
   scenario. Only then extend the envelope toward transition/forward flight.

7. **Exercise faults and replay.** Drop companion responses, stop one required
   telemetry stream, introduce delay/reordering, unplug a link, pause/reset
   Gazebo, restart the Pi and test disarm. **Pass:** the declared loss policy
   activates within its deadline, stale commands never regain authority, and
   the run is explainable from saved logs. Leave motor/servo power disconnected
   during HIL and keep simulation-only parameter changes separate from flight.

Firmware inclusion must be checked for **fmu-v6c**, not copied blindly from a
v6x example. The inspected [v6c default board configuration](https://github.com/PX4/PX4-Autopilot/blob/main/boards/px4/fmu-v6c/default.px4board)
does not explicitly enable `CONFIG_MODULES_SIMULATION_PWM_OUT_SIM`; this does
not identify what is on the board now. If absent, provide a pinned board build
with the module enabled. The inspected [PX4 startup](https://github.com/PX4/PX4-Autopilot/blob/main/ROMFS/px4fmu_common/init.d/rcS)
distinguishes external HIL from internal SIH and starts the simulated output
driver in HIL. Verify that path for the chosen release rather than merely
setting a mode flag while real-sensor startup remains active.

## Project changes that would be needed

These are proposed locations, not files created by this planning task:

| Area | Smallest complete change |
| --- | --- |
| `simulation/gazebo/` | Project-owned world/model, nine-actuator plant, simulated sensors, external HIL adapter, launch/config files, mapping/frame tests and recording tools; bring over only the reference assets actually needed and record provenance. |
| `companion/allocator/src/` | Selected target/demand receiver, typed result envelope, state/deadline handling and diagnostic timing; UDP transport only if keeping Pi-owned USB. |
| `companion/allocator/src/pso/pso_runtime.hpp` and implementation | Demanded-wrench solve interface/objective if allocator mode is chosen; preserve the standalone pose-target API/callers or update them explicitly. |
| `companion/allocator/src/pso/config.json` | A separate declared HIL model/calibration and limits, rather than silently replacing aircraft parameters. |
| `companion/allocator/tests/`, build files | Contract round trips, limits, expiry/replay, target/demand changes and source filtering; include the existing PTY regression tests. |
| Project-owned PX4 integration directory, e.g. `firmware/px4/` | Version manifest, board/airframe config, message definitions and source patch/module for validated PSO application. Build the matching supplied PX4 source; keep upstream third-party code in build inputs rather than vendoring a reference tree. |
| `companion/image/` and deployment guide | Include the eventual runtime/config/protocol assets and selected port/rate, plus reproducible launch instructions. Do not leave a working HIL setup only in an ad hoc Pi directory. |

No full ROS stack, GUI dashboard, detailed servo animations or physical PWM
capture is necessary for the first complete logical HIL loop. Source acquisition
and any Git staging/history operations remain with the user under project policy.

## Unanswered design questions

| Decision | Proposed starting point | What still needs an answer |
| --- | --- | --- |
| What does PSO control? | PX4 force/torque allocation as the eventual architecture; observation first | Confirm allocator versus current pose controller. This changes the objective and firmware boundary. |
| Must PX4 firmware stay stock? | Allow a small HIL-only integration module if choosing the current MAVLink stack | If stock is mandatory, assess a release-supported direct-actuator interface and its extra transport requirements. |
| Which PX4 release/build? | Pin the version actually tested | Installed version, HIL module availability, v6c build compatibility and custom-airframe support. |
| Which Gazebo route? | Harmonic, matching the inspected reference and installed runtime | Accept bridge development, or choose Classic baseline first in isolation. |
| Which cable topology? | Laptop USB for HIL + Pi TELEM2 | Whether GPIO harness is ready; otherwise budget router and companion transport work. |
| What is a target? | Position + yaw, one reference frame and origin | Online target changes? Full attitude constraints? Mission versus Offboard? Acceptance/acknowledgement behavior? |
| What outputs are measured? | Logical HIL actuator values after PX4 handling | Is electrical PWM/DShot timing also an experimental requirement? |
| Which physical model is authoritative? | One explicit HIL parameter set with sufficient hover authority | Mass/inertia, geometry, propulsion curve, servo trims/signs/rates, drag/lift and rotor reaction torque. |
| What frame does the PSO model use? | Document and test complete transforms | Forward axis, angle signs, quaternion convention and estimator-to-world origin relationship. |
| How fast must PSO run? | Choose after measured latency and stability experiments | Required control bandwidth, maximum result age, CPU budget and feasible link rate. |
| What happens on loss or infeasibility? | Abort the first isolated HIL experiment and safe outputs | Later fallback strategy, arbitration, anti-windup/saturation feedback and recovery rules. |
| How much flight envelope? | Hover and small position/yaw steps | Whether transition, forward flight, wind and sensor noise belong to the first acceptance scope. |
| What counts as success? | Agreed tracking, attitude, saturation and deadline bounds | Numeric thresholds, trial duration, baseline and repetitions. |

## Evidence to retain for every run

Keep the target/scenario, source/config/model/firmware hashes and parameter
export; PX4 ULog; Pi inputs, demands, candidates and compute times; the
accepted/applied PX4 outputs; Gazebo truth and sensor samples; monotonic sample
IDs/timestamps, packet gaps, mode/arming/failsafe events and real-time factor.
Trace one target ID through demand, result, accepted output and plant response.

Use the same scenario and model for baseline comparisons. The optimizer
currently seeds randomness from `std::random_device`; deterministic algorithm
replay needs an explicit test seed or recorded candidates. Playback of outputs
can reproduce a plant response but is not the same as rerunning PSO.

The immediate next step is to confirm PSO's role and inventory the installed
PX4 build, then prove the selected HIL sensor/output bridge. Hardware firmware,
Gazebo physics, link deadlines and closed-loop stability remain unverified.
