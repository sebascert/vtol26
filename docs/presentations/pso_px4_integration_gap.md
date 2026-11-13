---
marp: true
theme: default
size: 16:9
paginate: true
title: "PSO and PX4: the missing control loop"
description: "Project evidence, historical implementations, and the integration needed for HIL."
style: |
  section { font-size: 25px; }
  h1 { font-size: 40px; }
  table { font-size: 22px; }
  pre { font-size: 21px; }
---

# PSO and PX4: the missing control loop

**The current companion uses PX4 telemetry and runs PSO.**
**Applying PSO results through PX4 remains unfinished.**

Stallion VTOL · Technical status · 3 November 2026

Evidence: current source, legacy implementations, and four backup archives.

Sources: [Current implementation boundary](../gen/architecture/system_overview.md#companion-allocation-boundary)
· [Backup audit](../gen/guides/backup_pso_px4_audit.md#result)

---

# 1. The current PSO does receive PX4 input

```text
PX4 telemetry ──> Raspberry Pi ──> actual PSO ──> nine candidates
                                      ↑
                              target from config.json
```

- Inputs: position, velocity, attitude, angular rates and airspeed.
- The adapter passes those values into `optimizer_.solve(...)`.
- Freshness checks suspend allocation when required telemetry expires.

**The missing interface concerns controller demands and actuator application.**

Sources: [State passed to the optimizer](../../companion/allocator/src/pso_interface.cpp#L8)
· [Runtime freshness checks](../../companion/allocator/src/main.cpp#L46)

---

# 2. Vehicle state and controller demand differ

| Information | Meaning | Current companion |
| --- | --- | --- |
| State telemetry | What the vehicle is doing | Received from PX4 and used |
| Pose target | Where the vehicle should go | Loaded from startup JSON |
| Thrust/torque demand | What PX4's controllers request | No receiving interface |

The current objective selects actuators against a pose target.
It is not yet an allocator tracking PX4 thrust/torque demands.

Sources: [Optimizer input API](../../companion/allocator/src/pso/pso_runtime.hpp)
· [Target configuration](../../companion/allocator/src/pso/config.json)
· [Input conversion](../../companion/allocator/src/pso_interface.cpp)

---

# 3. Reporting a result does not apply it

The current sender transmits:

```text
DEBUG_FLOAT_ARRAY
name: PSO_ALLOC
data: nine candidates + fitness/convergence diagnostics
```

**This is diagnostic output, with no implemented PX4 actuator application.**

Allocation JSON confirms computation. It does not demonstrate that PX4
accepted the values or changed its motor/servo outputs.

Sources: [Actual message encoding](../../companion/allocator/src/mavlink_sender.cpp#L46)
· [Documented output contract](../../companion/allocator/README.md#allocation-result)

---

# 4. The legacy files contain separate prototypes

| Implementation | Actual behavior |
| --- | --- |
| Legacy MAVLink companion | Receives telemetry; `runPSO(state)` ignores it and returns fixed values |
| Legacy allocation sender | `sendPSOCommand()` only prints values |
| Standalone simulation | Contains real swarm code, without live PX4 communication |
| STM32 working-tree allocator | Receives local controller demands but calls a simple mixer |

**Function names and intended architecture overstate the completed integration.**

Sources: [Fixed result](../../legacy/mavlink_uart/src/pso_interface.cpp#L5)
· [Print-only sender](../../legacy/mavlink_uart/src/mavlink_sender.cpp#L72)
· [Standalone PSO](../../legacy/vtol_integrador/sim/PSO/pso_environment.cpp)
· [Simple mixer call](../../legacy/stm32-firmware/src/pso_allocator.cpp#L18)

---

# 5. The backups do not supply the missing link

**Four archives extracted and inspected; saved Git history included.**

| Backup content | Result |
| --- | --- |
| Two Stallion asset archives | CAD files and manuals |
| PX4 archive and nested copies | General firmware and a board-port scaffold; no project PSO integration found |
| `vtol_integrador.zip` | Standalone PSO, modeling assets and historical STM32 code |

The audit covered **57 commits and 150 historical code/configuration blobs**.
No complete recoverable Pi–PX4 control loop was found within that scope.

Source: [Audit scope, findings and archive fingerprints](../gen/guides/backup_pso_px4_audit.md)

---

# 6. A real historical allocator was found

Saved reflog commit **`44e9ee2`** contains actual particle-swarm optimization:

- Receives roll/pitch/yaw torque and thrust from a **local controller**.
- Optimizes **six variables: three motors and three servos**.
- Includes bounded particles, best-solution tracking and diagnostics.
- Targets standalone **STM32/Arduino**, with no Pi↔PX4 transport.

**It is a useful algorithm reference, not a recovered PX4 integration.**

Source: [Historical source evidence and recovery identifiers](../gen/guides/backup_pso_px4_audit.md#historical-source-findings)

---

# 7. Select one control architecture

| Option | PSO responsibility | Required change |
| --- | --- | --- |
| **PX4 controllers + PSO allocator** | Convert PX4 thrust/torque demands into nine actuator commands | Demand interface, wrench-tracking objective and validated PX4 application |
| Pose-target PSO controller | Choose actuators directly from state and target | Target interface and direct-output ownership/application |

**Recommended end state:** retain PX4's flight controllers and use PSO for
allocation, subject to timing and model validation.

This is a design recommendation. The current pose-target solver does not
implement the first option yet.

Sources: [Architecture choices](../gen/guides/gazebo_pso_hitl_plan.md#decide-what-pso-means-in-this-experiment)
· [Current solver contract](../../companion/allocator/src/pso/pso_runtime.hpp)

---

# 8. The HIL loop must include PX4 application

Proposed flow for the allocator architecture:

```text
Target ──> PX4 controllers ──> Pi PSO ──> PX4 validates/applies
               ↑                                  │
         PX4 estimation                     actuator outputs
               ↑                                  ↓
         HIL sensor input <────── Gazebo vehicle physics
```

Missing work: demand/result contract, output ownership, command expiry,
actuator mapping, Gazebo plant and HIL bridge.

Applying Pi diagnostic values directly to Gazebo would omit the PX4
application step required by this experiment.

Source: [Detailed HIL interfaces and milestones](../gen/guides/gazebo_pso_hitl_plan.md#interfaces-to-implement)

---

# 9. Resolve model and timing limits first

| Current setting | Consequence |
| --- | --- |
| Maximum modeled rotor thrust: **1.44 N** | Below the configured 4 kg vehicle's **39.24 N** weight; hover is infeasible at zero airspeed |
| Default PSO period: **1000 ms** | Bench cadence; suitability for control is unverified |
| Three-second telemetry timeout | Does not define an acceptable actuator-command deadline |

Choose a consistent physical model, benchmark Pi latency, and define a
failure policy before evaluating closed-loop tracking.

Sources: [Model coefficients and bounds](../../companion/allocator/src/pso/config.json)
· [Timing defaults](../../companion/allocator/src/application_config.hpp#L5)
· [Model and timing analysis](../gen/guides/gazebo_pso_hitl_plan.md#vehicle-physics-and-frames-must-agree)

---

# 10. What would demonstrate completion?

Trace one command through the entire system:

**Target → PX4 demand → PSO result → accepted PX4 output → simulated motion.**

1. Record proposed, accepted and actually applied values separately.
2. Demonstrate a matching change in the simulated actuator and vehicle.
3. Reject stale results and exercise loss/disarm behavior.
4. Repeat against defined tracking and latency criteria.

**Current evidence establishes telemetry and optimization, not this full loop.**
The backup search cannot exclude a version stored elsewhere. Historical
binaries were not executed or reverse engineered.

Sources: [Acceptance milestones](../gen/guides/gazebo_pso_hitl_plan.md#implementation-milestones-and-pass-criteria)
· [Audit limits](../gen/guides/backup_pso_px4_audit.md#limits-and-cleanup)
