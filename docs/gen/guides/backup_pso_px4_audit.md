# Backup audit: PSO interaction with PX4

Date: 2026-11-03. Source directory:
`/home/sebastian/projects/vtol/backup` (the existing path corresponding to the
requested `~/project/vtol/backup`).

## Result

**No complete recoverable Raspberry Pi PSO ↔ PX4 implementation was found in
these four backups, their nested project content, or the saved project history
examined.** This is a source-audit conclusion about these backups, not proof
that a working version never existed elsewhere.

There is an important additional finding: a saved Git reflog contains a real
six-variable PSO allocator in commit
`44e9ee2bfa04a82694f1ace7617d20c402052275`. It runs inside standalone
STM32/Arduino firmware, with demands from a local controller. It has no PX4
MAVLink/uORB interaction and is not the current nine-actuator companion.

## What was checked

All four top-level ZIPs were extracted into a session-created temporary
directory for inspection. None was encrypted. Original archives were read
only. Their contents were searched for PSO/swarm implementations, companion
programs, serial/MAVLink interfaces, control demands and actuator application.
Candidate source was then traced through its callers and build configuration.

For a complete integration, the inspected code would need to connect actual
PX4 input to a real optimizer and return its results through an implemented
PX4-facing transport/application path. Standalone optimization, architecture
descriptions, generic PX4 MAVLink support, and functions named “PSO” do not
establish this connection by themselves.

| Archive | Entries | Finding |
| --- | ---: | --- |
| `PX4-Autopilot-main.zip` | 16,617 | General PX4 source/reference material; no project-specific PSO integration found in its firmware source, board configuration, messages or startup files. |
| `STALLION VTOL FILES-20260323T230358Z-3-001.zip` | 31 | 18 STL, 11 STEP and 2 PDF files. Mechanical assets/manuals, no executable project source. |
| `STALLION-FILES-20260409T012023Z-3-001.zip` | 187 | 156 STL, 29 STEP and 2 PDF files. Mechanical assets/manuals, no executable project source. |
| `vtol_integrador.zip` | 21,494 | Standalone PSO/modeling material, archived project Git metadata, Python environment, PX4 source copies and historical STM32 firmware. No complete Pi/PX4 integration found. |

Nested ZIP contents were also inventoried. They contain CAD/PCB assets and a
generic bootloader-update package. The nested
`vtol_integrador/PX4-Autopilot-main.zip` has exactly the same SHA-256 as the
top-level PX4 ZIP, so it is a duplicate rather than a distinct project version.
The nested Python `dateutil` tarball contains timezone data.

## Working-tree source findings

Inside `vtol_integrador.zip`, archive-relative paths:

- `vtol_integrador/README.md` states that the companion implementation existed
  only on the Raspberry Pi and that the repository contained supporting
  simulation/modeling material. This is a documentary claim consistent with
  the inspected files, not independent proof about the old Pi's contents.
- `vtol_integrador/sim/PSO/pso_environment.cpp` contains a standalone swarm
  experiment. Its state is initialized internally, its target comes from
  `config.json`, and its entry point runs locally; it has no live PX4 receiver
  or allocation-result transport.
- No `mavlink_uart` companion, `PSOInterface` adapter or project-specific
  MAVLink allocation sender was found in that archived working tree.

The expanded nested PX4 tree is not entirely unchanged: it adds 27 files,
excluding `.git` metadata. Of these, 22 are under `boards/matek/CMakeLists.txt`
and `boards/matek/f405-wing-v2/`; the other five are `f405wing_datasheet.pdf`,
`build/.last_target`, and three NuttX configuration/build files. All 13,882
files shared with the top-level PX4 archive matched byte-for-byte by SHA-256;
the nested tree had no missing common files. The source additions are an
STM32F405 board port/scaffold, not a PSO module or companion protocol. Its board
configuration enables general MAVLink but explicitly leaves the control
allocator and several flight controllers disabled. Its separate `.git`
directory has no commits/branches to inspect.

## Direct comparison with official PX4 upstream

Checked again on **4 November 2026**, using the extracted legacy tree now at
`legacy/vtol_integrador/PX4-Autopilot-main/PX4-Autopilot-main/` in this repository.

**The legacy copy contains a local Matek F405-WING V2 board port/scaffold and
extra build/reference files. Every file already present in the matching
official PX4 snapshot is unchanged.** No project-specific PSO or actuator
application was added to PX4's existing modules in this copy.

The saved GitHub ZIP comment identifies commit
`1403709f658724e7938ea1a1eb2dc561b5f7a25e`. To verify that the saved ZIP itself
was upstream content, a fresh archive was downloaded directly from
[PX4/PX4-Autopilot at that exact commit](https://codeload.github.com/PX4/PX4-Autopilot/zip/1403709f658724e7938ea1a1eb2dc561b5f7a25e).
This avoids attributing differences from a later `main` branch to the project.

The comparison stripped only the archive's enclosing directory, calculated
SHA-256 for every non-directory entry, and compared the resulting relative-path
manifests. All 13,882 entries in the official archive and saved archive matched.
The extracted legacy tree was then compared to the official manifest, excluding
`.git` metadata.

| Comparison result | Count |
| --- | ---: |
| Official upstream files | 13,882 |
| Legacy files, excluding `.git` | 13,909 |
| Upstream files with identical contents in legacy | 13,882 |
| Modified upstream files | 0 |
| Deleted upstream files | 0 |
| Added files | 27 |

The additions are:

| Location | Count | Meaning |
| --- | ---: | --- |
| `boards/matek/CMakeLists.txt` | 1 | Includes the added board directory. |
| `boards/matek/f405-wing-v2/` | 21 | Board/toolchain configuration, startup, pin/bus/timer/USB definitions, NuttX configuration and a bootloader placeholder. |
| `f405wing_datasheet.pdf` | 1 | Board reference material. |
| `build/.last_target` | 1 | Records `matek_f405-wing-v2_default`; not a firmware executable or successful-build record. |
| `platforms/nuttx/NuttX/nuttx/{.config,Make.defs,defconfig}` | 3 | Additional NuttX configuration/build files, including STM32F405RG selection. |

The added board targets STM32F405 hardware. It is separate from the existing
Pixhawk 6C target. Its `default.px4board` enables MAVLink but explicitly disables
the control allocator, EKF2, local-position estimator, navigator, multicopter
attitude/position/rate controllers, fixed-wing attitude controller and VTOL
attitude controller. It therefore does not demonstrate the intended full
PX4-controller/companion-allocator architecture. A bootloader placeholder is
also present; this comparison did not establish a successful board build.

The following particularly relevant files were individually confirmed identical
to the official snapshot:

| File | SHA-256 in both copies |
| --- | --- |
| `src/modules/mavlink/mavlink_receiver.cpp` | `2501b310023d5ac6c70febce5cc9b16ab2231aa0fe745a868348b1ede33b9d55` |
| `src/modules/control_allocator/ControlAllocator.cpp` | `ead6ed29053855d73cff8531a4b14468f4e7cf7aa17ad4fd564e9e18fbd413e4` |
| `boards/px4/fmu-v6c/default.px4board` | `7cd7905d199be611bedf487f7b62372c2bb730045e97fcefb499e1d755b8093c` |
| `ROMFS/px4fmu_common/init.d/rcS` | `9b3d087ce6fc24cec4960303254391439aa314197fb677a038863927d855c369` |

Archive SHA-256 values:

- Saved `PX4-Autopilot-main.zip`:
  `7134d029e86412461255fd43206f83c69301df64773406931c9270508b48c6cf`.
- Fresh commit-addressed official archive:
  `1d84eac3e91830c7d7fdf4f81be5dbab415392eea29ca6deba016ebac5f9a249`.

The archives have different enclosing directory names (`PX4-Autopilot-main`
versus `PX4-Autopilot-<commit>`). Their byte-level hashes are not used as source
equality evidence; the per-file content comparison establishes that equality.
Archive packaging metadata is not used to infer a firmware source change.

Machine-readable results and complete file-hash manifests are retained locally
under the ignored `companion/allocator/build/px4-upstream-audit/` directory as
`comparison.json`, `upstream-sha256.json` and `legacy-sha256.json`. No Git clone,
fetch, checkout or repository modification was used for this comparison.

This is a comparison of saved file contents, not of permissions, all possible
external submodule contents, or the firmware currently flashed on the Pixhawk.
It does not identify who authored the added files. No board firmware was built,
flashed or executed during the comparison.

## Historical source findings

The archived `vtol_integrador/.git` was inspected read-only, including saved
refs, reflogs, merge-parent changes, and index entries. The inspection covered
57 distinct reachable commits, 150 distinct historical code/configuration
blobs in the searched source extensions, and 49 indexed code/configuration
files. No historical source matches were found for the searched MAVLink/uORB
integration identifiers, such as `mavlink_*`, `ORB_ID`,
`vehicle_torque_setpoint`, `actuator_controls`, or `SET_ACTUATOR`.

Two materially different allocator implementations exist in that history:

| Version | What it implements | Why it does not complete PX4 interaction |
| --- | --- | --- |
| `253b179`, `src/pso_allocator.cpp` | `PSO_Allocator::solve` calls `solve_simple`, a hand-coded mixer. `PSO_INTEGRATION_GUIDE.md` describes future integration work. | The actual caller is the standalone STM32 scheduler; no swarm or PX4 transport is implemented in that path. |
| `44e9ee2`, `src/pso_allocator.cpp` | Actual swarm initialization, particle/personal/global bests, velocity/position updates, bounded outputs and torque/thrust tracking cost with regularization. | Six decision variables: three motors plus three servos. Demands originate in the local Arduino quaternion controller; no Pi/PX4 communications or PX4 application module. |

For `44e9ee2`, the supporting evidence is:

- `include/pso_allocator.hpp`: `kDim = 6`, particle structures, a
  `solve(torque_roll, torque_pitch, torque_yaw, thrust, dt)` API.
- `src/pso_allocator.cpp`: real particle-swarm iteration and a simplified
  algebraic actuator-to-demand cost model. The allocator implementation blob
  is `2168f45402a4b98cc6e7704625f6c329eee399a3`.
- `src/scheduler.cpp`: local sensor update → local controller → allocator;
  results are copied to local arrays and made available for diagnostics.
- `src/main.cpp`: local test attitude/thrust sequence and serial diagnostics.
- `platformio.ini`: Arduino framework, `genericSTM32F405RG` board, not PX4
  firmware or a Raspberry Pi Linux companion target.

This improves the earlier current-file comparison: **some saved historical
code does contain real demand-based PSO**, even though the inspected
`legacy/stm32-firmware` working-tree version uses a simple mixer. Neither
finding establishes a complete external PX4 control loop.

## How to recover the useful historical reference later

The original `vtol_integrador.zip` retains its `.git` data. After extracting it
again, inspect these files without checking out a branch or changing history:

```sh
git -C <extracted>/vtol_integrador show 44e9ee2:src/pso_allocator.cpp
git -C <extracted>/vtol_integrador show 44e9ee2:include/pso_allocator.hpp
git -C <extracted>/vtol_integrador show 44e9ee2:src/scheduler.cpp
```

Recover matching headers/configuration/controller dependencies together if
reusing it. This audit did not build it, run it, validate its physical model,
or establish suitability for the nine-actuator HIL design. In particular,
its simplified effectiveness model is not measured aircraft calibration.

## Archive fingerprints

These fingerprints identify exactly which backups were audited. They were
checked again before temporary-file cleanup.

| Archive | SHA-256 |
| --- | --- |
| `PX4-Autopilot-main.zip` | `7134d029e86412461255fd43206f83c69301df64773406931c9270508b48c6cf` |
| `STALLION VTOL FILES-20260323T230358Z-3-001.zip` | `1ee9b59a597e19c42bf48df0ddb4e5a1075d584d39df4c6e3ed3134432e39bb1` |
| `STALLION-FILES-20260409T012023Z-3-001.zip` | `961ec68bf9a6e7401ee6ad4951d828eab337c25c0b995f13c827dc5d0cbed323` |
| `vtol_integrador.zip` | `3b7b856220a32f0ba2cc8a2fea41fdf5b162b7992f709ab1f93013452b1f3dfe` |

## Limits and cleanup

No backup programs were executed or firmware flashed. Bundled binaries were
not reverse engineered, and external locations mentioned in documentation
were not assumed to exist. The conclusion concerns recoverable source in
these archives and the saved history/index examined.

The temporary extraction directory was removed after inspection; original
archives were unchanged. Only this report and a cross-reference in the
[HIL plan](gazebo_pso_hitl_plan.md) are retained in the project.
