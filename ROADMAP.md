# Minilink Roadmap

Maturity and priorities. Contracts: [DESIGN.md](DESIGN.md). Agent rules:
[agent.md](agent.md).

## 1. Maturity

| Area | TRL | Rationale | Next |
| --- | --- | --- | --- |
| Core + diagrams | 7 | Public API and diagram API are probably stable after the final port-declaration update. | Keep stable; finish export policy and remaining edge cases. |
| Compile (`core/compile/`) | 4 | Integrated, but dynamic evaluator methods and exposed surface still need user review. | Review evaluator API, diagram parametric tier, and backend parity. |
| Simulation | 7 | Mature workflow with stable API and solver/forcing coverage. | Keep behavior stable; treat `SimulationOptions` as ergonomic cleanup, not a redesign. |
| Optimization | 5 | `MathematicalProgram` and `Optimizer` are integrated and useful, but backend details still need hardening. | Harden SciPy/Ipopt behavior and evaluator details before test-gated promotion. |
| Planning/trajopt | 2 | Some user review happened, but much of the module remains AI one-shot prototype work. | Re-evaluate architecture/API before deeper integration. |
| Graphical | 3 | Useful, but plotting/diagram APIs are still evolving. | Do not freeze public APIs until kinematic/visual hooks use composition (dynamics vs skin separation); re-evaluate after that review. |
| Animation | 3 | Substantial work exists, but renderer, camera, and live-loop contracts may still change. | Same gate as Graphical: composition-based kinematic contract before TRL promotion or API freeze. |
| Dynamics catalog | 6 | Pyro models ported, QA'd term-by-term against pyro, and covered by tests (see `docs/plans/catalog-migration-notes.md`); `DynamicBicycle` params now thread fully. | Review naming/details per module toward TRL 7. |
| Symbolic mechanics | 1 | One-shot AI-generated demos, not a validated subsystem. | Keep isolated until clear use cases justify review. |
| Contact engine (`dynamics/engines/`) | 1 | Moved out of quarantine by maintainer decision (June 2026); math not yet QA-validated. | Add validation tests (energy, analytic contact cases) toward TRL 2. |
| Analysis | 4 | `analysis/linearize.py` (→ `LTISystem`), `analysis/structural.py` (ctrb/obsv), `analysis/equilibria.py` (trim) migrated from pyro with finite-difference Jacobians; covered by tests; demos in `examples/scripts/analysis/`. | Add `frequency.py` (Bode/pole-zero), `modal.py`; consider JAX-exact linearization. |
| Control | 5 | `control/linear.py` (`ProportionalController` (SISO+MIMO)/`PDController`/`PIDController`/`LinearStateFeedbackController`), `control/lqr.py` (`lqr_gain`/`lqr`/`lqr_at_operating_point`), and `control/pid.py` (`FilteredPIDController` with anti-windup) integrated and tested. | Port computed-torque, sliding-mode, robotic controllers. |

TRL definitions: [agent.md §8](agent.md#8-trl-lifecycle).

## 2. Done (architecture)

- Separated model / compile / simulate / optimize / plan / graphics.
- `ExecutionPlan` + NumPy/JAX evaluators; shared `Trajectory`.
- Composition shortcuts (`+`, `>>`, `@`, `autowire`) → ordinary `DiagramSystem`.
- Explicit ports; `DynamicSystem` textbook ports via constructor options.
- Pure `MathematicalProgram` + `Optimizer`; backend-native trajopt transcriptions.
- Phase-plane plotting (`plot_phase_plane`, matplotlib).
- User docs: [README.md](README.md) (workflows and minimal call chains).
- Package taxonomy: four bands (framework / system libraries / tools /
  quarantine) with a dependency law and placement algorithm
  ([DESIGN.md §3](DESIGN.md)); `compile/` folded into `core/compile/`;
  generic blocks in top-level `blocks/`; generic control laws in
  `control/linear.py` and `control/pid.py`; `System` facades split into `core/facades.py`
  (API unchanged).

## 3. Priorities

**P0** — Docs/contracts aligned with code; compiled vs reference path parity.

**P1** — Dynamic evaluator API review; ~~diagram parametric evaluators (`f_p`,
`h_p`)~~ done (nested `{sys_id: {…}}` params, `jacobian_f_params`; see
DESIGN.md §4 Parameters and `examples/scripts/identification/demo_params_gradient.py`); diagram validation;
top-level `minilink` exports; NLP hardening.

**P2** — ~~`analysis/` seed (linearization → `LTISystem`)~~ done (also ctrb/obsv,
equilibria); frequency and modal still pending. ~~`control/lqr.py` (design fn +
state-feedback block)~~ done. ~~blocks round-out (Sum, Gain, Saturation; PID in
`control/linear.py`)~~ done (routing, nonlinear, filters, `TrajectorySource`,
PID, MIMO proportional). Remaining: nested-diagram ergonomics; forced-input
helpers; swappable live graphics backends; refactor `System` visualization hooks
(`get_kinematic_*`, camera) to delegate to composable kinematic models (pilot
one catalog plant, e.g. pendulum) before calling `graphical/` TRL ≥ 4.

## 4. Review queue (needs maintainer sign-off)

- Public export policy for `minilink/__init__.py`.
- Diagram validation as separate `validate()` vs inline wiring.
- Trajopt transcription internal consolidation.
- Dynamic bicycle module split.
- Graphics/camera contract consolidation, including kinematic composition
  (optional `KinematicModel` delegate on `System`, fate of `get_dynamic_geometry`,
  diagram aggregation unchanged) as a prerequisite for finalizing `graphical/`.

## 5. Future

Pre-decided homes (bands and placement rules in [DESIGN.md §3](DESIGN.md)),
in rough build order:

1. **`analysis/`** — linearization (→ `LTISystem`), ctrb/obsv, and equilibria done;
   still pending: frequency (Bode, pole-zero), modal (eigenmodes + animation).
   Phase-plane math migrates here from `graphical/` when touched.
2. **`control/`** — `lqr.py`, `linear.py`, and `control/pid.py`
   (`FilteredPIDController`) done; still pending: `computed_torque.py`,
   `sliding_mode.py`, `robotic.py` (impedance), `mpc.py` (uses `optimization/`),
   `neural.py` (NN policies).
3. **`blocks/`** — routing, nonlinear, filters, and `TrajectorySource` done;
   still pending: `neural.py` (MLP block, pure `jnp`, `params` = weights).
4. **`estimation/`** — `luenberger.py`, `kalman.py`, later `ekf.py` (uses
   `analysis/` linearization) and `recursive.py` (online parameter
   estimators: RLS, adaptive laws). Offline fitting stays in
   `identification/`.
5. **`identification/`** — fit parametric systems to data; one verb for
   physical params and network weights (`fitting.py`, first-order optimizers,
   datasets). Depends on the parametric evaluator tier (P1).
6. **`interfaces/`** — `gymnasium.py` (RL trains outside; policies return as
   `control/` blocks), `torch.py`/`flax.py` model wrappers, cosimulation/FMI,
   multibody import.
7. **Quarantine graduation** — `symbolic/` as a dynamics-authoring tool.
   (The JAX contact engine moved to `dynamics/engines/` in June 2026 —
   still experimental; validation tests pending.)

Out of scope by decision: discrete time (digital control, ZOH/delay blocks,
RNNs, mixed-rate simulation) — minilink stays continuous-time only; see
[DESIGN.md §3](DESIGN.md).

Also: differentiable rollouts; hybrid/events; RRT and dynamic programming
(removed as stubs — re-design before reintroducing, under `planning/search/`
and `planning/policy_synthesis/`); phase-plane follow-ups (Plotly, 3D,
compiled grid eval).
