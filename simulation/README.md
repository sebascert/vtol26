# Gazebo simulation

The Gazebo reference project previously located at `~/projects/vtol/pvtol_gz`
is included here as project source. Its original directory was left unchanged;
its Git metadata, build products, caches and unused ROS description were not
copied. Model meshes referenced by the SDF are included under `models/`.

| Path | Purpose |
| --- | --- |
| `src/`, `include/` | Quaternion controller and Gazebo plugin |
| `models/stallion_vtol/` | SDF model, metadata and visual meshes |
| `worlds/pvtol.sdf` | Reference world and initial vehicle pose |
| `build.sh`, `run.sh` | Location-independent build and launch helpers |

## Build and run

Requires Gazebo Harmonic / `gz-sim8` development libraries, CMake 3.22 or newer
and a C++17 compiler. From the repository root:

```sh
cmake -S simulation -B simulation/build -DCMAKE_BUILD_TYPE=Release
cmake --build simulation/build --parallel 2
./simulation/run.sh
```

The copied project/library names remain `pvtol_gz` and `pvtol_controller` so
the SDF plugin reference continues to work. The launch helper resolves plugin
and model paths relative to this directory.

## Current boundary

The model has one rigid body, STL visuals, a spherical collision and an
autonomous quaternion controller that applies ideal forces and moments. It
does not yet provide a PX4 HIL bridge, simulated sensor interface or a calibrated
nine-actuator plant driven by PSO outputs.

The [Pixhawk firmware prototype](../controller/README.md) now includes MAVLink
HIL output support and diagnostic PSO reception. The Harmonic bridge and
actuator-driven plant are still separate implementation work.

The PSO runtime belongs to [`companion/allocator/src/pso/`](../companion/allocator/src/pso/).
Earlier standalone PSO/visualization and Simulink experiments remain in
[`legacy/simulation/`](../legacy/simulation/). See the
[HIL plan](../docs/gen/guides/gazebo_pso_hitl_plan.md) for the proposed integration.
