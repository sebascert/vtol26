# Documentation map

All project documentation collected from the workspace's former `docs/` directory
and this repository lives here. Generated Markdown and documentation images are
grouped under `gen/`; original PDFs and resource links are organized separately.

## Technical documentation

- [Project progress — study and presentation guide](gen/guides/project_progress_study_guide.md)
  — implementation walkthrough, bench evidence, output interpretation, remaining
  milestones and a ten-minute presentation plan.
- [System overview](gen/architecture/system_overview.md) — intended hardware/software
  architecture and implementation status.
- [Raspberry Pi ↔ Pixhawk 6C wiring](gen/guides/raspberry_pi_pixhawk_6c_wiring.md) —
  physical pinout, separate power, UART/PX4 setup and bench checks.
- [Gazebo PSO HIL plan](gen/guides/gazebo_pso_hitl_plan.md) — inspected interfaces,
  simulator compatibility, proposed architecture, milestones and open decisions.
- [PSO/PX4 integration gap — presentation](presentations/pso_px4_integration_gap.md)
  — slide-ready explanation with project source citations and backup findings.
- [PX4 upstream comparison and backup audit](gen/guides/backup_pso_px4_audit.md#direct-comparison-with-official-px4-upstream)
  — exact-commit file comparison, legacy board additions and historical PSO findings.
- [Vehicle modeling](gen/modeling/readme.md) — dynamics model and Simulink helpers.
- [Reference notes](gen/references/readme.md) — context for the external PDFs.
- [Historical notes](gen/archive/pre_companion_redesign/readme.md) — earlier STM32
  guides, source README and architecture review. These describe an older design;
  use the system overview above for the current architecture.
- `gen/images/` — PNG and editable SVG figures used by the wiring guide.
- [Raspberry Pi companion image and Ubuntu Ethernet/SSH setup](../companion/rasp-image/README.md)
  — Yocto image definition, build/flash instructions and direct laptop connection.

## Original documents and resources

- `references/` — [quaternion control thesis](references/these_utc_hernan_abaunza_gonzalez.pdf)
  and [Yocto documentation](references/yocto_documentation.pdf).
  The thesis was also supplied as `quaternion_control_algorithms_thesis.pdf`;
  the two files were byte-for-byte identical, so only one copy is retained.
- `reports/` — [project paper](reports/project_paper.pdf) and
  [final evidence](reports/final_evidence.pdf).
- `presentations/` — [project overview](presentations/project_overview_presentation.pdf)
  and [Gustavo's overview](presentations/project_overview_presentation_gustavo.pdf).
- `resources/` — original link files for the
  [Overleaf dynamics document](resources/dynamic_module_latex.txt) and
  [Onshape CAD](resources/onshape.txt).
