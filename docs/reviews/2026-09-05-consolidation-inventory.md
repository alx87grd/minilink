# Consolidation inventory — 2026-09-05 (S41)

Ranked by *maintenance cost removed per hour of work*, per the ROADMAP
principle: consolidation targets text edited twice when code changes, dead
API, boilerplate a flag would replace, and twins one `xp` body covers — never
features, never clean well-placed code, never a deliberate ladder (the DP
planner's three backends stay). Nothing here is executed without a pick;
each row says what it does **not** remove. Numbers were measured on
`dev-alex` @ `5c41f83` and re-checked on `dev-fable` where noted.

| # | Item | Size | Blast radius | Does **not** remove | Suggested |
| --- | --- | ---: | --- | --- | --- |
| 1 | **Demo-script headers → one-line title** (rule approved) | 434 lines / 63 scripts | examples only; demo sweep verifies | any code, any lesson prose (notebooks untouched) | Phase 1 (S39), folder by folder |
| 2 | **`_jit` alias methods** on the six JAX evaluators (`register_jit_aliases`) | 24 methods, 0 call sites, 1 test asserting `f_jit is f` | `core/compile`; DESIGN §5 one sentence | `f` / `outputs` / `step` themselves | **your pick** — public attribute names (S16) |
| 3 | **Unreferenced evaluator integration methods** (`*_ivp_p`, `*_ivp_trace*`, `euler_*_trace*`, `rk4_step_ivp*`, `rollout_p`, `step_block`, `f_scipy`, `f_ivp_scipy`, …) | 28 public methods, 0 call sites outside the package | `core/compile` NumPy + JAX evaluators; DESIGN §5 "stable-internal" paragraph | the frozen subset DESIGN names (`f`, `f_p`, `outputs`, `step`, `rollout`, `rk4_step`, `integrate_zoh*`, trace twins) | **your pick** (S17) |
| 4 | **`*Ports` vehicle twins** → `named_ports=` constructor flag | 6 classes, ~150 lines | `jax_vehicles.py`; hybrid/MPC demos that build them | any rung's math; `DynamicBicycle`'s named `w_rear`/`delta` ports (settled) | Phase 2 (S25, approved ladder) |
| 5 | **Research rungs → `examples/projects/`** (`BicycleKin/Acc/Dyn`, `TauRate/Servo/Engine`, `ConstantSpeedKinematicCar`, `DynamicHolonomicMobileRobot`, `HolonomicMobileRobot3D`, `UdeSRacecar`, `CarProfile`) | ~1 900 lines out of `catalog/` | catalog `__all__`, tests that import them, project scripts | the four teaching rungs; nothing is deleted, only moved | Phase 2 (S25) — moving user-importable names: **confirm list** |
| 6 | **NumPy/JAX twin plants** after the `xp` sweep: `JaxCartPole`↔`CartPole`, `BicycleKin`↔`KinematicBicycle`, `Holonomic`↔`HolonomicMobileRobot`, `BicycleDyn`↔`DynamicBicycle` | up to ~700 lines | catalog names, trajopt demos/notebooks that import `JaxCartPole` | either plant's behaviour — the survivor runs on both backends | Phase 2 (S22–S24) — name removals: **your pick per pair** |
| 7 | **`blocks/sources.Source.show_signal`** — 80 lines of bespoke matplotlib duplicating `compute_trajectory()` + `plot_trajectory()` on a static leaf; carries its own `TODO: fold into graphical` | 80 lines; 7 call sites (2 notebooks, 2 demos, module `__main__`) | `blocks/sources.py`, `00_core`/`01_blocks` cells, `blocks_sources.py` | the ability to plot a source — same picture via `source.plot_trajectory(tf=…)` | **your pick** — it is a user-callable method |
| 8 | **Dead modules**: `planning/spatial/overlays.py` (imported by nothing), `symbolic/mechanics/utils.py` (0 references, 0% covered) | 27 + 31 statements | none found | — | **your pick** (importable names) |
| 9 | **Deprecated benchmark shims**: `run_pendulum_f_speed.py`, `run_diagram_f_speed.py` (README already calls them "deprecated shim → run_study"), and possibly `run_step_speed.py`, `run_step_diagram_speed.py`, `run_simulator_speed_*.py` if `run_study.py` presets cover them | 6 scripts | `benchmarks/README.md` table | the regression gates, `run_study.py`, baselines | agent lane; verify coverage first, then delete the two declared shims |
| 10 | **Parametric evaluator duplication**: `planning/…/parametric_evaluator.py` is 54% line-identical to `optimization/evaluators/jax_evaluator.py` | ~100 duplicated lines | research lane (MPC) | either evaluator's behaviour | Later — design note exists ([optimizer-parametric-wiring.md](../plans/optimizer-parametric-wiring.md)) |
| 11 | **MPC debug figure inside the controller** (`init_debug_figure` / `update_debug_figure`, matplotlib in `control/mpc/controller.py`) | ~45 lines | `control/mpc`; `viz.py` is the natural home | the debug figure feature | agent lane (plotting): move to `control/mpc/viz.py` |
| 12 | **`HybridDiagram` hand-copied facades** (`compute_trajectory` / `plot_trajectory` / `animate` / `plot_diagram` bodies duplicated from `SharedSystemFacades`) | ~150 lines | hybrid research lane | any facade | Phase 3, together with the `HybridLoop` decision |
| 13 | **Two mechanical bases** (`GeneralizedMechanicalSystem`, 154 lines, 2 users; `H` vs `M` naming) | 154 lines + one naming split | `Boat2D`, `Plane3D`, DESIGN §3 | the generalized-velocity formulation (kept as `N ≠ I` on the unified base) | Phase 3 (S32) — core |
| 14 | **Two geometry vocabularies** (`core.geometry.Sphere/Box` SDF solids vs `graphical.animation.primitives.Sphere/Box` glyphs — same names, different meaning) | rename only; ~25 call sites | catalog skins, demos | either type | Phase 3 (S30) — public names |
| 15 | **DESIGN.md research-lane prose** (realtime, MPC dual-rate, spatial pipeline paragraphs in §4/§6 — ~250 lines that describe provisional contracts in the frozen doc) | ~250 lines | docs | the contracts — they move next to their plan docs | agent lane: move to the plan docs, leave one-line pointers |
| 16 | **`_method` names on `System` subclasses** (43, against the style rule) | 43 renames | catalog + control internals | behaviour | Later, per module — mechanical but touches the catalog |
| 17 | **Plotting in eight homes** (`planning/*/plotting.py` 1 558 lines, `analysis/frequency.py`, `control/mpc/viz.py`, `port_map.py` in `graphical/`) | placement only | wide | any plot | Later — write the rule down first (DESIGN), move `port_map.py` next to `control/` if you agree |

Already done tonight (no pick needed): wheel excludes the research lane
(S18); parity audit shrunk 49 → 36 KB and two landed plan docs deleted (D6);
`solve_disp` / `step_disp` / `disp` unified to `verbose` (S07).

**Suggested picks for the next session:** 1 (in progress), 2, 3, 7, 8, 9 —
together roughly 600 lines of pure maintenance surface with no behaviour
change; then 4–6 as part of the Phase 2 catalog work.
