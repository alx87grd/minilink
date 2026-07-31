# Minilink operational backlog

Actionable workboard. Strategy and maturity stay in [ROADMAP.md](../../ROADMAP.md).
Multi-step designs live as separate docs under this folder (see [README.md](README.md)).
Pyro parity rows stay in [pyro-port-remaining.md](pyro-port-remaining.md) — link, do not copy.

| Section | Use for |
| --- | --- |
| [Small fixes](#1-small-fixes) | Cheap guards and one-liners |
| [Pre-v0.2 hardening](#2-pre-v02-hardening-provisional-bands) | Deferred provisional-band findings |
| [Teaching demos & pyro ports](#3-teaching-demos--pyro-ports) | Next demo pulls (not the full pyro matrix) |
| [New modules / upgrades](#4-new-modules--upgrades) | Milestone-adjacent library work |
| [Later / big ideas](#5-later--big-ideas) | Post teaching-release feature ideas |
| [Maturity follow-ups](#6-maturity-follow-ups) | Thin “next” pointers from the TRL table |

Each item stays one line when possible. If a change needs architecture tradeoffs,
open a short design doc here and leave a one-liner below.

---

## 1. Small fixes

Cheap guards and nits (promote from hardening when they are one-shot):

- [ ] **Multiple shooting parametric guard** — `MultipleShootingTranscription`
  inherits collocation `transcribe_parametric`; `compile_parametric_program()`
  with MS silently builds wrong defects. Override to `NotImplementedError` (or
  implement true MS parametric); replace planner `hasattr` with an explicit
  capability flag. **Highest priority of the provisional backlog.**

---

## 2. Pre-v0.2 hardening (provisional bands)

Findings from the v0.1 pre-freeze architecture review that touch
**provisional** bands ([README.md — API stability](../../README.md#api-stability-v01)).
Deferred by decision — the v0.1 freeze fixed only the stable tier. Each row is
problem → proposed solution.

| Problem | Proposed solution |
| --- | --- |
| **`MultipleShootingTranscription` inherits collocation `transcribe_parametric`** (`planning/trajectory_optimization/multiple_shooting.py`): `compile_parametric_program()` with multiple shooting silently builds collocation defects. **Highest priority — cheap guard.** | Override to raise `NotImplementedError` (or implement true MS parametric); replace the planner's `hasattr(..., "transcribe_parametric")` feature check with an explicit capability flag. (Also listed under [§1](#1-small-fixes).) |
| RRT `KinodynamicExtender` ignores `problem.params.system` (`planning/search/extenders.py`): propagates with frozen-params `rk4_step`; `U.sample` also drops `params.sets`. | Use `rk4_step_p` with `problem.params.system` (frozen compile when `None`); thread `params.sets` into input sampling. |
| MPC port computes drop `params` (`control/mpc/controller.py` `del params` in `_compute_*` / `step`): online params cannot reach the hybrid tick path. | Document that ports ignore `params` (or raise on non-`None`); align `step_disp` with the `verbose` convention. |
| Planning band exports empty: `planning/`, `trajectory_optimization/`, `search/`, `policy_synthesis/` `__init__` export nothing while `spatial/` exports richly. | Decide band facade (`_EXPORTS` + lazy `__getattr__` like `catalog/`) vs a documented deep-import policy; apply uniformly. |
| `ShootingTranscription` orphaned: implemented and tested, absent from `transcription=` string presets. | Add a `"shooting"` preset or delete the class (pre-1.0 no-alias rule: no half-exposed API). |
| `ParametricMathematicalProgram` / `JaxParametricProgramEvaluator` placement and duplication: live under `planning/`, duck-copy `optimization/evaluators/jax_evaluator.py` helpers; parametric path supports only `scipy_slsqp` vs offline `Optimizer` presets. | Relocate beside `MathematicalProgram` in `optimization/`; dedupe evaluator helpers; document or close the optimizer-method cliff. Design: [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md). |
| Dual online-params façades: `ProblemParameters.scene` field is never read while online `params={"scene": …}` raises — two things named "params". | Unify naming/messaging when pipeline B (`J(z, p)` bind) lands; until then keep the loud `NotImplementedError`. Design: [planning-pipeline-architecture.md](planning-pipeline-architecture.md). |
| `HybridSimulator` conventions drift: ad-hoc one-line verbose vs `sim_reporting` panels; `last_result` vs `last_traj` cache naming; forced-input coercion copy; kwargs-only ctor differs from `Simulator`. | Adopt `sim_reporting` panels; pick one cache name (no aliases); reuse the shared forced-input helper from `simulation/input_coercion`; align ctor style. |
| `HybridDiagram` hand-copies facade bodies (`animate` / plot vs `SharedSystemFacades`). | Share the facade implementation once the hybrid API stabilizes. |
| DP metadata: `DynamicProgrammingOptions.final_time` ignores `problem.tf`; `SolveMetadata.success` is always `True` even when tolerance is not met. | Read horizon from `problem.tf` when set; report convergence honestly in metadata. |
| Verbose flag naming varies: planner `solve_disp` / `step_disp`, optimizer `disp`, DP `verbose`. | Standardize on `verbose` for setup/report printing; keep genuinely different knobs (`live_plot`) distinct. |
| `interfaces/c_export.py`: a real experimental JAX→C transpiler ships inside a "placeholder" band with no TRL row. | Move to quarantine or add an honest TRL row in ROADMAP; do not let it ride the freeze implicitly. |
| Realtime band: `TODO: User Architectural Review` markers stand; `compile_backend=None`→auto default diverges from offline simulators. | Architectural review (ROADMAP review queue / TRL row); align auto-backend policy with the shared helper. |
| Spatial exports: `CostDensityField` / `WorkspaceField` are public-ish but unexported from `planning/spatial/__init__.py`. | Decide export or keep internal; document either way. |

---

## 3. Teaching demos & pyro ports

Milestone contract: [ROADMAP.md §4 items 2–3](../../ROADMAP.md#4-teaching-release-priorities).
Full matrix: [pyro-port-remaining.md](pyro-port-remaining.md) (do not duplicate rows here).

Next pulls (representative closed-loop demo per major plant family; not every pyro script):

- [ ] SMC trajectory-following demo (control teaching band)
- [ ] Remaining pyro **game** demos → `simulation/realtime/` or explicit drop (also on ROADMAP review queue)
- [ ] One closed-loop demo per major plant family still missing from pyro-port example table
- [ ] TRL-8 demos for landed teaching bands once criteria say so

---

## 4. New modules / upgrades

Tied to [ROADMAP.md §4](../../ROADMAP.md#4-teaching-release-priorities) when they are teaching-release work:

- [ ] Frequency / classical MIMO analysis — `pole_zero_map()`, `nyquist()`, `margin()`, `ss2tf()` (ROADMAP priority 1)
- [ ] `trajectory_generation/` port
- [ ] Estimation — Luenberger, Kalman
- [ ] Identification — `fitting.py`
- [ ] Robotic PID wrappers + trajectory LQR
- [ ] README pyro → minilink migration guide; keep intro/showcase aligned (ROADMAP priority 4)
- [ ] Release hardening — compile vs reference parity; pre-release gate green (ROADMAP priority 5)

Design writeups when needed: [neural-blocks-collection.md](neural-blocks-collection.md),
[vehicle-abstraction.md](vehicle-abstraction.md), etc.

---

## 5. Later / big ideas

Post teaching release — do **not** displace [ROADMAP.md §4](../../ROADMAP.md#4-teaching-release-priorities).
One-line ideas land here; multi-step designs get their own plan doc and a link below.

- [ ] Scene params / `J(z, p)` bind (moving obstacles, terrain SDFs without JIT rebuild) — [planning-pipeline-architecture.md](planning-pipeline-architecture.md)
- [ ] `SolverFactory` — unify SciPy / Ipopt / CasADi wiring for trajopt and MPC — [optimizer-parametric-wiring.md](optimizer-parametric-wiring.md)
- [ ] `MjxPlant` under `interfaces/mjx.py` (`minilink[mjx]`); prefer deprecate hand-rolled contact in `dynamics/engines/`
- [ ] Gymnasium / RL bridges; Pacejka; stochastic forcing; neural MLP
- [ ] ROS2 / FMI; sparse long-horizon trajopt; parametric `core/` Shape/Set/Cost call-time overrides; trajectory post-filter; RRT-Connect / informed sampling
- [ ] **Shared RNEA serial-chain stack** (not copy-paste per 6-DoF arm): extract DH + spatial RNEA helpers (or a thin `SerialRneaManipulator` base) so catalog plants only supply `a`/`d`/`alpha`, mass/COM/inertia; keep public `H`/`C`/`g` on the mechanical API
- [ ] **ABA on other RNEA arms** (pattern from UR5): keep public `H` / `C` / `g` for teaching; use Articulated-Body Algorithm for `forward_dynamics` / `f` so integration does not form \(H\) each step. UR5 catalog plant done; generalize when adding the next spatial manipulator

---

## 6. Maturity follow-ups

Do not duplicate the TRL table. Source of truth: [ROADMAP.md §2](../../ROADMAP.md#2-maturity-trl).
Useful next actions called out there:

- [ ] Compile teaching-API review; backend parity (TRL 4 → higher)
- [ ] Optimization: harden SciPy/Ipopt before TRL 6
- [ ] Geometry / spatial: architecture validation
- [ ] Graphics / animation: renderer polish
- [ ] Realtime: architectural review (also §2 hardening / review queue)
- [ ] Analysis frequency completion (same as §4 priority 1)
