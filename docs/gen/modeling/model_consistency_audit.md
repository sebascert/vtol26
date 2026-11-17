# Drone dynamics and actuator model assessment

Reviewed: 2026-11-17. This is a documentation and source assessment, with small
numerical checks of the existing equations. It does not change the models or
establish agreement with the physical aircraft.

## Answers

**There is no single consistent, calibrated vehicle model across the reviewed
material.** There is a common idea: a fixed-wing tricopter with two front tilting
rotors, one rear vertical rotor, two ruddervators and two ailerons, using
quaternions and Newton–Euler rigid-body dynamics. However, actuator definitions,
frames, mass, inertia, propulsion laws and aerodynamic contributions disagree.
The project paper describes this intended vehicle clearly, but does not supply
the coefficients and actuator dynamics needed to close its equations.
See [the paper, pp. 3–5][paper-model] and the comparisons below.

**There is a workable modelling foundation, but no complete model here that can
be accepted unchanged for hover, transition and cruise.** The most complete
*aircraft-specific actuator model* is the current
[`companion/allocator/src/pso/pso_environment.cpp`][pso-forces] with its
[`config.json`][pso-config]. It explicitly represents all nine intended actuator
variables, sums component forces and lever-arm moments, predicts pose and is
connected to live state telemetry. It also has serious physical inconsistencies.
Its older copy under `legacy/vtol_integrador/sim/PSO/` has fewer corrections.

For the most complete *continuous rigid-body plant*, use the MATLAB/Simulink
family under [`legacy/simulation/simulink/`][matlab-plant]. It integrates
position, velocity, quaternion and body angular velocity, but its nine inputs
represent a different, simplified vehicle. The thesis is the strongest formal
reference for rigid-body dynamics, not a calibrated model of this tricopter.
These distinctions matter: completeness of a program is different from physical
correctness. [MATLAB plant][matlab-plant], [MATLAB force model][matlab-forces],
[thesis, printed pp. 18, 26–27][thesis-dynamics].

## What was examined

The inventory covered every file in `docs/` and
`legacy/vtol_integrador/docs/`: 45 and 3 files respectively before this report.
All Markdown, TeX and link files were searched for dynamics, geometry, forces,
moments and actuators. Text was extracted from all six PDFs; the equation images
on Gustavo's presentation PDF pages 9–10 were also rendered and inspected.
Relevant implementation references were followed into the current PSO, archived
PSO, MATLAB, legacy STM32 force model and current Gazebo source.

| Material | What it contributes |
| --- | --- |
| [Project paper][paper-model], 9 PDF pages | Sections III.B–E give 6DOF equations, force contributors and the nine-variable PSO vector; section IV.A gives a claimed cost formula. This is the clearest aircraft-level narrative, not a complete parameterized derivation. |
| [Final evidence][evidence-model], 27 PDF pages | PDF pp. 20–23 repeat/extend the model and actuator description. PDF p. 25 says physical experiments and future HILS validation remain in development. Treat its implementation claims accordingly. |
| [Gustavo's presentation][gustavo-forces], 19 PDF pages | PDF p. 9 (slide 8) shows quaternion force transformations; PDF p. 10 (slide 9) shows component moment formulas. These closely match the standalone TeX files. |
| [Project overview][overview-hardware], 6 PDF pages | PDF pp. 5–6 describe three 5010 750KV motors, six servos, and an approximately 30 N **per motor design target**. They do not provide measured thrust or power curves. |
| [`main.tex`](../../main.tex), [`fbd.png`](../../fbd.png) | Vehicle/free-body illustration: three rotors, wings, ailerons and V-tail. The PNG is the underlying aircraft drawing; the TeX adds force, moment and angle annotations. No identified mass properties. |
| [`force_eq.tex`][tex-forces], [`torque.tex`][tex-moments], [`fuerzas conjugadas.tex`][tex-expanded] | Component force rotations, scalar moment sketches and an expanded resultant-force expression. They are incomplete and mutually inconsistent in several conventions, detailed below. |
| [`misc.tex`](../../misc.tex), [`poster.tex`](../../poster.tex) | Small beta/quaternion notation fragments; no additional force or motion equations. |
| [Current modelling README](readme.md), [legacy modelling README][legacy-model-readme], [legacy references README][legacy-ref-readme] | Point to the symbolic/Simulink family and the quaternion thesis. Their shared nine-input description does **not** establish the same nine physical actuators as PSO. Legacy paths are historical. |
| [Thesis][thesis-dynamics], 155 PDF pages | Quaternion/Newton–Euler formulation, rotor thrust/reaction laws and quadrotor control. Useful mathematical foundation; its four-rotor mixer is not this vehicle's allocation map. |
| [Yocto PDF](../../references/yocto_documentation.pdf), 29 PDF pages | Linux image/build reference, not aircraft physics. |
| [Current generated architecture](../architecture/system_overview.md), [progress guide](../guides/project_progress_study_guide.md), [HIL plan][hil-physics], [integration-gap presentation](../../presentations/pso_px4_integration_gap.md), [backup audit](../guides/backup_pso_px4_audit.md), [wiring guide](../guides/raspberry_pi_pixhawk_6c_wiring.md) | Implementation status, previous model feasibility findings and integration proposals. They add no calibrated aircraft parameter set. The HIL output-channel table is explicitly proposed. |
| [Archived pre-companion documentation](../archive/pre_companion_redesign/readme.md), all 17 Markdown files | Older STM32 architectures, examples and integration plans. In particular, `architecture_px4_inspired.md` describes one front fixed rotor and two rear tilting rotors, contrary to the current paper/PSO geometry. The archived `vtol_project_source_of_truth.md` already distinguishes intended features from inspected source. |
| [Documentation map](../../readme.md), [reference notes](../references/readme.md), `gen/images/` PNG/SVG pairs | Navigation, bibliography and bench wiring illustrations; no additional dynamic model. |
| [Overleaf link file](../../resources/dynamic_module_latex.txt), [Onshape link file](../../resources/onshape.txt) | External source pointers only. Access attempts in this review did not expose either document. Their unseen contents cannot establish coefficients, CG or inertia. |
| `legacy/vtol_integrador/docs/.gitkeep` | Empty placeholder. |

## What agrees, and what does not

### Rigid-body equations agree at the structural level

The paper and thesis both use translational force balance, Euler's rotational
equation and quaternion kinematics. The MATLAB plant supplies an explicit
13-component state:

$$
x=[p_I^T,\;v_I^T,\;q_{BI}^T,\;\omega_B^T]^T.
$$

With **our explicit notation here**, let the scalar-first unit quaternion
$q_{BI}$ rotate body vectors into the inertial frame. Then the reusable
plant structure is

$$
\dot p_I=v_I,\qquad
\dot v_I=R(q_{BI})F_B/m+g_I,
$$
$$
\dot q_{BI}=\tfrac12q_{BI}\otimes[0,\omega_B],\qquad
\dot\omega_B=J^{-1}\{M_B-\omega_B\times(J\omega_B)\}.
$$

Here $F_B$ excludes gravity, $g_I=[0,0,-g]^T$ for a positive-up world,
and $J$ and $M_B$ are expressed about the CG in body coordinates. These
equations make the frame transformation explicit; the paper's compact
$m\dot v=F_{aero}+F_{prop}+mg$ requires all forces to be in the same frame.
The quaternion adds one redundant coordinate to six degrees of freedom.
[Paper, section III.B][paper-model]; [MATLAB plant, generated dynamics block][matlab-plant];
[thesis, equations (2.29)–(2.31), PDF pp. 39–40][thesis-dynamics].

This shared structure does not resolve the disagreements in $F_B$, $M_B$
or their inputs.

### The two nine-input vectors are different

The paper and current PSO agree on the following ordering. Angle limits are
the **current JSON search bounds**, not measured servo travel or a PX4 output
channel assignment. [Paper, section III.E][paper-actuators];
[PSO indices][pso-indices]; [configuration, `actuators`][pso-config].

| Zero-based PSO index | Variable | Intended physical effect | Configured range |
| --- | --- | --- | --- |
| 0, 1, 2 | `omega_I`, `omega_D`, `omega_T` | Left-front, right-front, rear rotor commands | Each 0–2000, physical units unresolved |
| 3, 4 | `epsilon_I`, `epsilon_D` | Independent front-rotor tilt about body Y | 0–π/2 rad; zero gives +Z thrust, π/2 gives +X thrust in the equations |
| 5, 6 | `gamma_I`, `gamma_D` | Left/right V-tail control, also used for the extra ruddervator force terms | −0.5–0.5 rad |
| 7, 8 | `beta_I`, `beta_D` | Left/right ailerons | −0.5–0.5 rad |

In contrast, MATLAB consumes

```text
[total_thrust_N, common_tilt,
 gamma_left, gamma_right,
 alpha_rud_left, gamma_rud_left, alpha_rud_right, gamma_rud_right,
 common_aileron_deflection]
```

It has one lumped motor force, one common motor tilt, extra independently supplied
tail angles and one differential aileron input. It cannot represent independent
three-motor thrusts or independent front tilts. In particular, local incidence
`alpha_rud_*` and servo deflection are not automatically independent physical
actuators. Connecting the PSO array directly to this plant would reinterpret
almost every input incorrectly. [MATLAB force model, lines 13–22, 37–44][matlab-forces].

The older STM32 description is a third layout: three motors and three servo
slots, with the paired tilting motors described as **rear** motors. Its current
`solve()` calls a heuristic mixer. Neither this description nor its force-only
helper supplies a more complete nine-actuator aircraft model.
[Archived architecture, “Estructura para Tricóptero”][old-architecture];
[legacy mixer, lines 18–93][old-mixer]; [legacy force helper][old-forces].

### The physical parameters do not describe one agreed aircraft

| Quantity | Current PSO JSON | MATLAB starter parameters | Gazebo reference |
| --- | --- | --- | --- |
| Mass | 4.0 kg | 1.80 kg | 1.3 kg |
| Diagonal inertia, kg m² | (0.12, 0.15, 0.22) | (0.040, 0.050, 0.080) | (0.0325, 0.0325, 0.0325) |
| Main wing area | 0.28 m² | 0.22 m² | No aerodynamic wing model |
| Tail cant used in force rotation | 0.785 rad, about 45° | 32° | No tail force model |
| Motor geometry relative to assumed CG | Front (0.5, ±0.25, 0), rear (−0.6, 0, 0) m | One resultant at (0.12, 0, 0) m | Ideal net wrench on one body |
| Wing force location | (0.6, 0, 0) m | (0, 0, 0) m | No wing force |

Sources: [PSO configuration][pso-config], [hardcoded PSO lever arms, lines 411–420][pso-forces],
[MATLAB parameters][matlab-params], [Gazebo SDF, lines 5–16][gazebo-model],
[Gazebo wrench application, lines 121–141][gazebo-controller]. The MATLAB
parameter file explicitly asks for adjustment using CAD, identification and
tests. No reviewed source traces these competing values to one measured,
assembled vehicle. The Gazebo controller's own mass/inertia defaults match its
SDF, but not PSO or MATLAB. [Controller parameters][gazebo-params].

### Specific inconsistencies that affect the dynamics

1. **Rotor command units and thrust are unresolved, and current hover is
   infeasible.** PSO evaluates $T_i=0.00024\,\omega_i$, linearly, with a
   maximum command of 2000. Thus even with all rotors vertical:

   ```text
   maximum rotor thrust = 3 × 0.00024 × 2000 = 1.44 N
   configured weight    = 4.0 × 9.81         = 39.24 N
   ```

   At zero airspeed all its aerodynamic terms vanish. The maximum is only
   about 3.67% of the weight, before imposing moment balance. If `omega` means
   actual rotor angular speed, the thesis instead gives quadratic thrust and
   reaction torque, $T_i=k_T\Omega_i^2$, $Q_i=k_Q\Omega_i^2$.
   A linear command-to-thrust approximation could be used over an identified
   range, but it needs declared command units and measured coefficients. Do
   not simply square the current numbers or interpret them as PWM. The
   presentation's 30 N per motor is a design target, not a calibration of this
   coefficient. [PSO, lines 377–382][pso-forces]; [JSON][pso-config];
   [thesis, section 2.1, equations (2.1)–(2.3), PDF p. 31][thesis-rotors];
   [overview, PDF p. 6][overview-motors].

2. **Drag changes axes between implementations.** The TeX and symbolic MATLAB
   encode a pure force quaternion `[0, -D, 0, L]`, i.e. the 3-vector
   `[-D, 0, L]`. Current and archived C++ PSO instead rotate the 3-vector
   `[0, -D, L]`. At zero angle of attack, drag therefore lies along −Y in PSO,
   although rotor tilt produces forward thrust along +X. With zero actuator
   commands and airspeed 20 m/s, the current equations produce
   `F_B = (0, -2.24349, 19.208) N`. This is not the longitudinal drag encoded
   by the TeX/MATLAB family. The same misplaced drag component occurs in the
   C++ surface terms. [TeX force rotations][tex-forces];
   [symbolic MATLAB, wing/aileron blocks][matlab-symbolic];
   [PSO, lines 364–398][pso-forces].

3. **Angle-of-attack and frame handling are incomplete.** PSO fixes `alpha=0`;
   live scalar airspeed overrides the configured reference speed, but body
   velocity does not determine incidence. MATLAB derives incidence from body
   velocity, with no wind input, but uses `rot_y(+alpha)` for its wing where
   the symbolic version uses a quaternion rotation by `-alpha`; its aileron
   forces also omit the symbolic wind-to-body rotation. Meanwhile the PX4
   adapter negates world Z and VZ while passing attitude and body rates
   unchanged. That is not a complete NED/positive-up frame transformation.
   A consistent right-handed world/body convention and corresponding attitude
   conversion must be specified before these can be compared.
   [PSO, lines 241–246 and 348–352][pso-state];
   [MATLAB, lines 6–44][matlab-forces]; [symbolic rotations][matlab-symbolic];
   [telemetry adapter, lines 8–17][telemetry].

4. **Aerodynamic tail surfaces may be counted twice.** PSO adds separate
   `F_cola_*` and `F_ruddervator_*`, driven by the same two `gamma` inputs,
   with different areas and lever arms. The MATLAB/TeX family also separates
   these terms. That can be valid if they describe distinct fixed and moving
   areas, with suitable baseline/incidence and incremental coefficients.
   The reviewed material does not establish that partition. As coded, both
   PSO terms scale with servo deflection and vanish at zero `gamma`, rather
   than modelling a fixed tail's incidence-dependent lift. The paper describes
   two physical V-tail controls, not four additional independent servos.
   Reconcile the areas and meanings before summing these forces.
   [PSO, lines 384–409][pso-forces]; [paper, PDF pp. 4–5][paper-actuators];
   [TeX][tex-forces]; [MATLAB, lines 46–66][matlab-forces].

5. **Rotor reaction torque is not consistently represented.** `torque.tex`
   includes signed rotor reaction terms. PSO computes only $\sum r_i\times
   F_i$, without rotor reaction torque; vertical, untilted rotors at zero
   airspeed consequently have no yaw moment in this model. MATLAB adds
   `[0,0,k_reaccion*total_thrust]`, a single fixed-Z term rather than three
   signed reactions aligned with the tilted rotor axes. The symbolic
   `M_directo` is an unparameterized placeholder. Motor spin directions,
   reaction coefficients and tilt effects still need one definition.
   [Moment TeX][tex-moments]; [PSO, lines 422–433][pso-forces];
   [MATLAB, lines 68–79][matlab-forces]; [symbolic model][matlab-symbolic].

6. **The handwritten resultants are not an authoritative consolidated model.**
   `force_eq.tex` makes zero rotor tilt produce +Z thrust, while
   `fuerzas conjugadas.tex` uses `T*cos(epsilon)` in X and `T*sin(epsilon)` in
   Z. Their tilt zeros differ. The component TeX gives positive X aileron
   drag while wing/tail drag is negative X. Its tail coefficients include
   an unexplained `602`, and its quaternions use `cos(0.48), sin(0.48)`,
   implying a rotation of 0.96 rad, about 55°, rather than 32° or 45°.
   The scalar moment sketches do not define signed force components and
   geometry sufficiently to replace the cross-product sum. These require a
   derivation with units and signs before reuse; apparent constants should
   not be silently treated as corrected measurements.
   [Component TeX][tex-forces]; [expanded TeX][tex-expanded];
   [moment TeX][tex-moments]; [Gustavo, PDF pp. 9–10][gustavo-forces].

7. **The MATLAB “hover” preset is not a trim solution.** Its default supplies
   `T=m*g` and sets every angle to zero. At rest the force is vertical, but
   the off-CG motor and reaction term produce
   `M_y = -0.12*T = -2.11824 N m` and
   `M_z = 0.012*T = 0.211824 N m`, using its starter parameters. The preset
   initially balances translational weight, not all six force/moment conditions.
   [Simulation helper, default input][matlab-run]; [parameters][matlab-params];
   [force/moment sum][matlab-forces].

8. **The older PSO attitude prediction is dimensionally wrong.** The archived
   PSO adds angular acceleration times `dt` directly to orientation, omitting
   existing angular velocity and the attitude kinematics. Current PSO instead
   forms a quaternion increment from
   `omega*dt + 0.5*omega_dot*dt^2`, then converts back to Euler angles.
   Prefer the current predictor over the archived one, while recognizing
   that it is a short-horizon approximation with frozen forces, not the
   continuous 13-state plant. [Archived PSO, lines 422–448][old-pso];
   [current PSO, lines 436–471][pso-predictor].

9. **Energy and allocation claims exceed the implemented model.** The paper
   describes demanded-force/moment tracking and actuator-category energy
   penalties. Current PSO's cost is a weighted sum of predicted displacement
   squared, generated moment squared and target-pose error. The category
   weights `k_w`, `k_epsilon`, `k_gamma`, `k_beta` are loaded but do not enter
   this cost; no measured power model is present there. Its runtime input is
   vehicle state plus a startup pose target, not a demanded wrench. Therefore
   convergence alone establishes neither energy optimality nor a correct
   controller-demand allocation. [Paper, section IV.A, PDF p. 5][paper-cost];
   [current cost, lines 481–514][pso-cost]; [runtime interface][pso-api].

## Which model should be retained as the starting point?

| Candidate | Best use | Why it is insufficient unchanged |
| --- | --- | --- |
| **Current nine-actuator C++ PSO model** | **Primary aircraft-specific actuator topology and existing executable force/moment implementation** | Incorrect drag direction, infeasible propulsion defaults, fixed zero incidence, unresolved tail partition, no reaction torque or actuator dynamics. |
| MATLAB numeric force model + `construir_planta_vtol.m` | Continuous 6DOF integration and explicit state/frame structure | One lumped motor and a different input vector; placeholder parameters and force-convention differences. |
| `fuerzas_v4.m` | Symbolic component rotations and general moment sum | No identified parameter set or three-rotor actuator map; requires comparison with numeric equations. |
| Thesis | Formal dynamics and rotor force/reaction reference | Quadrotor examples, not this airframe's geometry/aerodynamics/calibration. |
| Gazebo reference | Rigid-body simulator, visual assets and quaternion-controller experiment | One body receives an ideal resultant thrust/torque; no nine-actuator force plant or wing/tail aerodynamics. |
| Archived PSO/STM32 models | Historical context | Older integration error, alternate geometry, or force-only/mixer implementations; no superior complete plant found. |

Sources: the implementation links above and [Gazebo's stated boundary][gazebo-readme],
confirmed by [its actual wrench application][gazebo-controller].

The smallest defensible unification is to retain the **current PSO actuator
layout**, correct and calibrate its force/moment map, and drive the **explicit
rigid-body equations** above with that map. If Gazebo integrates the state,
apply the resultant wrench there and let Gazebo apply gravity once. If MATLAB
integrates it, replace its current input interpretation with the agreed physical
actuator map. This is a proposed consolidation, not an existing connection.

The reusable force/moment structure is

$$
F_B=\sum_iF_{rotor,i}+\sum_jF_{aero,j},\qquad
M_B=\sum_k r_k\times F_k+\sum_i s_iQ_i a_i+M_{aero,direct}.
$$

Here $r_k$ is relative to the measured CG, $a_i$ is each rotor's thrust
axis and $s_i$ is its signed reaction convention. For the present positive-up
rotor convention, the front axes are
$a_{L,R}=[\sin\epsilon_{L,R},0,\cos\epsilon_{L,R}]^T$, and the rear axis is
$[0,0,1]^T$. The cross-product and direct-moment split already exists in
the MATLAB family; signed rotor reactions follow the thesis's rotor laws.
No numerical calibration is supplied by writing this expression.
[Current rotor rotations][pso-forces]; [MATLAB moments][matlab-forces];
[thesis rotor laws][thesis-rotors].

To turn that foundation into a workable aircraft plant, the missing work is:

1. Define one right-handed body/world convention, quaternion direction/order,
   actuator ordering and physical units; reconcile PX4 conversion with it.
2. Identify the assembled mass, CG, inertia, component lever arms, areas and
   cant angles in one parameter set with measurement/CAD provenance.
3. Measure motor/propeller/ESC command-to-thrust and reaction curves, servo
   neutral/sign/travel and time response. Include motor lag and servo slew/limits.
4. Correct force directions, derive incidence from relative airflow and define
   fixed versus moving tail contributions. Transition/cruise additionally require
   a justified aerodynamic envelope, including stall and propwash effects where
   significant; the current linear/quadratic surface approximations do not
   demonstrate that envelope.
5. Verify zero-input fall, each isolated actuator's force/moment sign, feasible
   hover trim (`F_I + m*g_I = 0`, `M_B = 0`), and forward-flight trim. Feed
   identical states/actuators/parameters into the optimizer and simulator and
   compare resulting wrenches before using closed-loop behavior as evidence.

These are requirements inferred from the documented disagreements and missing
terms, not claims that the measurements have been performed. The existing
[HIL modelling plan][hil-physics] independently calls for calibrated conversions,
actuator dynamics and agreement between simulator and companion physics.

## Checks and limits of this review

- A temporary C++17 assertion harness directly included the current PSO source
  with `PSO_RUNTIME_ONLY`, loaded the actual JSON, and called
  `calcularFuerzaMomento`. It confirmed 1.44 N maximum vertical rotor thrust,
  39.24 N configured weight, zero rotor yaw moment for untilted rotors at zero
  airspeed, and the lateral aerodynamic force reported above. Compilation
  succeeded with an anonymous-namespace subobject-linkage warning from the
  included source; assertions passed.
- The MATLAB preset moments were checked algebraically from its parameter
  file and cross-product/reaction expressions. MATLAB/Simulink and Gazebo were
  not run, and the serialized `.slx` was not validated against the builder.
  No flight, actuator bench test, thrust-curve identification or CAD mass-property
  extraction was performed.
- This review establishes what is present in the local sources and where they
  disagree. The inaccessible Overleaf/Onshape documents or measurements elsewhere
  could contain additional information. The local PDFs remain the cited evidence;
  none of their flight-readiness or energy-saving claims was independently tested.

## Citation notation

PDF page links use **physical PDF page numbers**, which can differ from
printed page/slide numbers. Code line ranges in the prose refer to this review's
working-tree contents.

[paper-model]: ../../reports/project_paper.pdf#page=3
[paper-actuators]: ../../reports/project_paper.pdf#page=4
[paper-cost]: ../../reports/project_paper.pdf#page=5
[evidence-model]: ../../reports/final_evidence.pdf#page=20
[gustavo-forces]: ../../presentations/project_overview_presentation_gustavo.pdf#page=9
[overview-hardware]: ../../presentations/project_overview_presentation.pdf#page=5
[overview-motors]: ../../presentations/project_overview_presentation.pdf#page=6
[thesis-rotors]: ../../references/these_utc_hernan_abaunza_gonzalez.pdf#page=31
[thesis-dynamics]: ../../references/these_utc_hernan_abaunza_gonzalez.pdf#page=39
[tex-forces]: ../../force_eq.tex
[tex-moments]: ../../torque.tex
[tex-expanded]: ../../fuerzas%20conjugadas.tex
[legacy-model-readme]: ../../../legacy/vtol_integrador/docs/modeling/README.md
[legacy-ref-readme]: ../../../legacy/vtol_integrador/docs/references/README.md
[matlab-plant]: ../../../legacy/simulation/simulink/construir_planta_vtol.m
[matlab-forces]: ../../../legacy/simulation/simulink/modelo_fuerzas_momentos_vtol.m
[matlab-params]: ../../../legacy/simulation/simulink/crear_parametros_vtol.m
[matlab-symbolic]: ../../../legacy/simulation/simulink/fuerzas_v4.m
[matlab-run]: ../../../legacy/simulation/simulink/simular_y_graficar_vtol.m
[pso-indices]: ../../../companion/allocator/src/pso/pso_state.hpp
[pso-config]: ../../../companion/allocator/src/pso/config.json
[pso-forces]: ../../../companion/allocator/src/pso/pso_environment.cpp#L348-L433
[pso-state]: ../../../companion/allocator/src/pso/pso_environment.cpp#L241-L352
[pso-predictor]: ../../../companion/allocator/src/pso/pso_environment.cpp#L436-L471
[pso-cost]: ../../../companion/allocator/src/pso/pso_environment.cpp#L481-L514
[pso-api]: ../../../companion/allocator/src/pso/pso_runtime.hpp
[telemetry]: ../../../companion/allocator/src/pso_interface.cpp
[old-pso]: ../../../legacy/vtol_integrador/sim/PSO/pso_environment.cpp#L422-L448
[old-architecture]: ../archive/pre_companion_redesign/architecture_px4_inspired.md
[old-mixer]: ../../../legacy/stm32-firmware/src/pso_allocator.cpp
[old-forces]: ../../../legacy/stm32-firmware/src/control/aero_force_model.cpp
[gazebo-model]: ../../../simulation/models/stallion_vtol/model.sdf
[gazebo-controller]: ../../../simulation/src/pvtol_controller.cc
[gazebo-params]: ../../../simulation/include/control/control_algorithm.hh
[gazebo-readme]: ../../../simulation/README.md
[hil-physics]: ../guides/gazebo_pso_hitl_plan.md#vehicle-physics-and-frames-must-agree
