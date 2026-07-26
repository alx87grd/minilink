# Minilink Roadmap

Maturity and priorities. Contracts: [DESIGN.md](DESIGN.md). Agent rules:
[AGENTS.md](AGENTS.md).

**Pyro 2.0 remaining backlog:**
[docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md).

## 1. Maturity

| Area | TRL | Rationale | Next |
| --- | --- | --- | --- |
| Core + diagrams | 7 | Public API and diagram API are probably stable after the final port-declaration update. | Keep stable; finish export policy and remaining edge cases. |
| Compile (`core/compile/`) | 4 | Integrated, but dynamic evaluator methods and exposed surface still need user review. | Review evaluator API, diagram parametric tier, and backend parity. |
| Simulation | 7 | Mature workflow with stable API and solver/forcing coverage. | Keep behavior stable; treat `SimulationOptions` as ergonomic cleanup, not a redesign. |
| Optimization | 5 | `MathematicalProgram` and `Optimizer` are integrated and useful, but backend details still need hardening. | Harden SciPy/Ipopt behavior and evaluator details before test-gated promotion. |
| Planning/trajopt | 5 | Direct collocation, shooting, MS transcriptions integrated; `PlanningProblem` + live-plot hooks. | Scene params tier; trajectory post-filter. |
| Planning/policy synthesis | 4 | `DynamicProgrammingPlanner` + `StateSpaceGrid`; `loop` / `numpy` / `jax` backends; lookup policy + `PolicyEvaluator`. | Raster cost maps; GPU tuning. |
| Planning/search | 4 | `RRTPlanner`, `RRTStarPlanner`, extenders, steering, KD-tree nearest; spatial `Scene` via `X`. | RRT-Connect; informed sampling. |
| Geometry / spatial | 4 | Integrated architecture proposed for obstacle and terrain planning: `core/geometry.py` SDF primitives + cost algebra (`SumCost`/`ScaledCost`), and `planning/spatial/`: `Scene` (obstacles + `workspace_fields`), `WorkspaceField`/`StateField`, `RobotBody`/`TranslationBody`, export via `as_constraint`/`as_cost`. Tested incl. JAX twins. | User architecture validation; scene params (`ProblemParameters.scene`, future); RRT consumers; oriented/multi-sphere bodies and raster cost maps. |
| Graphical | 4 | Frame-keyed ``tf`` / geometry / overlay contract integrated. | Renderer polish; optional ``KinematicModel`` delegate review. |
| Animation | 4 | ``Animator`` + overlays (``SceneHistory``, ``Replay``); collision reuses ``tf``; live sessions delegated to `simulation/realtime/`. | Renderer polish. |
| Realtime simulation | 2 | `RealtimeSimulator` + `RealtimeInput`/`RealtimeOutput` contracts (`simulation/realtime/`): wall-clock loop on evaluator `integrate_zoh`, pygame keyboard + joystick input, live `Animator` draw, `Trajectory` return. | User architectural review; cosimulation adapters under `interfaces/`. |
| Dynamics catalog | 6 | Pyro plants ported, QA'd term-by-term; catalog arms on `Manipulator`. | Optional `JaxManipulator`; `f_ext` port if approved. |
| Dynamics abstraction | 6 | `MechanicalSystem` + `Manipulator` (`p`, `pdot`, FK/J); catalog arms rebased. | `JaxManipulator` if needed; external wrench port. |
| Symbolic mechanics | 1 | One-shot AI-generated demos, not a validated subsystem. | Keep isolated until clear use cases justify review. |
| Contact engine (`dynamics/engines/`) | 1 | Experimental hand-rolled contact; math not QA-validated. | Validate toward TRL 2 **or** prefer external engine leaf ([§5.9](#59-quarantine-graduation), [§5.7](#57-interfaces)). |
| External multibody leaf (MJX, …) | 0 | Optional `DynamicSystem` wrapper around an external JAX multibody/contact engine — backlog under [§5.7](#57-interfaces) / [§5.9](#59-quarantine-graduation), not the product-vision center ([§5.11](#511-product-vision--landscape-position)). | Architecture sign-off + spike when scheduled. |
| Analysis | 5 | Linearize, structural, equilibria, modal, selected-channel Bode. | Pole-zero, Nyquist, margins, `ss2tf`; reachability costs. |
| Control | 6 | Linear, LQR, filtered PID; `modelbased.py` (CT, Pyro-parity SMC); `robotic.py` (impedance, kinematic, nullspace). | SMC traj-following demos; dynamic joint/effector PID wrappers; trajectory LQR. |
| Blocks | 5 | Routing, nonlinear, filters, sources, transfer function, 1-layer NN. | Multi-layer `MLP`, atomic layers (see neural-blocks plan). |
| Estimation | 1 | Placeholder only. | Luenberger, Kalman, EKF. |
| Identification | 2 | Parametric-tier prototype demo only. | `fitting.py` for physical + NN params. |
| Interfaces | 1 | Placeholder only. | Gymnasium, Flax/Torch adapters. |
| Pyro 2.0 overall | 3 | Catalog + core framework + planning search/DP/trajopt done; ~143/195 pyro demos not ported. | Phased port per [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md). |

### TRL definitions

Readiness levels are an internal maturity scale for planning and review—not a
release process by themselves.

| Level | Name | Description |
| --- | --- | --- |
| **TRL 1** | Agent MVP | Initial code exists and works |
| **TRL 2** | User-check MVP | User performs a high-level functional review |
| **TRL 3** | Architecture Validated | High-level architecture is approved |
| **TRL 4** | Integration Proposed | Final integration/refactor is proposed |
| **TRL 5** | Integration Validated | User approves main-codebase integration |
| **TRL 6** | Automated Tests Pass | Final pytest coverage exists and passes |
| **TRL 7** | Details Validated | Naming and implementation details are approved |
| **TRL 8** | Demo Released | Demo script is created and validated |
| **TRL 9** | Mission Complete | Tests, demo, and user approval are all complete |

## 2. Done (architecture)

- Separated model / compile / simulate / optimize / plan / graphics.
- `ExecutionPlan` + NumPy/JAX evaluators; shared `Trajectory`.
- **JAX evaluator trace tier** — fast vs trace execution tiers on compiled
  evaluators ([DESIGN.md](DESIGN.md) §5).
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
- **Pyro catalog plants** — all EoM models ported and QA'd.
- **Kinematic graphics contract** — string-keyed ``tf``, frame-keyed geometry,
  ``skin`` attribute, overlays at ``animate(overlays=…)``, collision ``bind()``
  reusing ``tf`` (DESIGN §4, §6).
- **Pyro tool tranche 1** — blocks routing/nonlinear/filters/sources,
  linear control + LQR, analysis linearize/structural/equilibria/modal/Bode.
- **Planning search + DP** — RRT/RRT*, value iteration, spatial scene integration.
- **DP/RRT on continuous `PlanningProblem`** — discrete pyro framework remains out of scope.

## 3. Priorities

**P0** — Docs/contracts aligned with code; compiled vs reference path parity.

**P1** — **Parametric primitives** (`Shape`, `Set`, `CostFunction` reading `params` at call time); Dynamic evaluator API review; ~~diagram parametric evaluators (`f_p`,
`h_p`)~~ done; diagram validation; top-level `minilink` exports; NLP hardening.

**P2** — ~~Analysis seed~~ done (remaining frequency tools). ~~Blocks round-out~~
done (routing, nonlinear, filters, `TrajectorySource`, PID, MIMO proportional).
~~`control/lqr.py`~~ done. Remaining:

- ~~`Manipulator` base + catalog rebase~~ — done (`manipulator.py`, `arms.py`).
- ~~`control/modelbased.py`, `control/robotic.py`~~ — done.
- Nested-diagram ergonomics; forced-input helpers

**P3** — Remaining pyro 2.0 tools (see [pyro-port-remaining.md](docs/plans/pyro-port-remaining.md)):

- ~~`planning/policy_synthesis/`~~ done; ~~`planning/search/rrt.py`~~ done
- `planning/trajectory_generation/` (polynomial / min-snap)
- `estimation/luenberger.py`, `estimation/kalman.py`
- `identification/fitting.py`
- `interfaces/gymnasium.py`

**P4** — Demo parity + release polish (phases D–G in gap doc):

- Port representative `demos_by_system/` closed-loop scripts per plant
- Frequency completion; obstacle/Pacejka/stochastic layers (if approved)
- Pyro migration guide in README; TRL 8 demos per tool band

**P5 (vision, not scheduled)** — Product vision + landscape position; see
[§5.11](#511-product-vision--landscape-position). Does not displace P0–P4;
gated on review-queue sign-off of the wording.

## 4. Review queue (needs maintainer sign-off)

- Public export policy for `minilink/__init__.py`.
- Diagram validation as separate `validate()` vs inline wiring.
- Trajopt transcription internal consolidation.
- Dynamic bicycle module split.
- Graphics/camera contract consolidation (`KinematicModel` delegate) — optional follow-up.
- **Pyro game demos** — superseded by `simulation/realtime/` game demos
  (`examples/scripts/realtime/`); port remaining ones there or explicitly drop.
- **Product vision §5.11** — wording of vision, positioning table, and
  practices list
  ([§5.11](#511-product-vision--landscape-position)).
- **External multibody leaf (MJX, …)** — package home (`dynamics/engines/` vs
  `interfaces/`), continuous vs discrete adapter split, deprecate-vs-keep for
  hand-rolled contact ([§5.7](#57-interfaces), [§5.9](#59-quarantine-graduation)).

## 5. Future

Pre-decided homes ([DESIGN.md §3](DESIGN.md)), build order adjusted for pyro 2.0:

### 5.0 Compile (evaluators)

- [x] **Integration API rename** — `rk4_integrate_{zoh,linear,ivp}`, `euler_integrate_{zoh,ivp}`,
  full `_p` / JAX `_trace` grid, lazy rollout JIT, by-backend evaluator layout
  ([DESIGN.md](DESIGN.md) §5)

### 5.1 Analysis

- [x] Linearize (→ matrices/`LTISystem`), ctrb/obsv, equilibria, modal, Bode (selected channel)
- [ ] Pole-zero, Nyquist, margins, `ss2tf`
- [ ] Reachability / domain-check costs (for DP demos)
- [ ] Phase-plane math migration from `graphical/` when touched

### 5.2 Control

- [x] `lqr.py`, `linear.py`, `pid.py` (`FilteredController`)
- [x] `modelbased.py` — computed torque, sliding mode (Pyro parity)
- [x] **Continuous SMC closed loop** — SMC `solver_info` flag, diagram aggregation, auto **Euler** + finer `dt`, forced-solver warnings (see also [DESIGN.md §5 — Discontinuous closed loops](DESIGN.md#discontinuous-closed-loops--known-issues))
- [ ] `robotic.py` — joint/effector PD/PID wrappers (kinematic + nullspace landed)
- [ ] `trajectory_lqr.py` — time-varying LQR along a reference
- [x] **MPC** — `control/mpc/` (`ModelPredictiveController`, warm-start, dual-rate);
  see §5.5 for trajopt compile-once / NumPy rebuild
- [ ] `neural.py` — policy wrappers ([neural-blocks-collection.md](docs/plans/neural-blocks-collection.md))

### 5.3 Blocks

- [x] Routing, nonlinear, filters, `TrajectorySource`, `TransferFunction`, 1-layer `NeuralNetwork`
- [ ] Multi-layer `MLP`, `Dense`/activation layers, `mlp_diagram` factory
- [ ] `Switch` (deferred — selection semantics)
- [ ] `RateLimiter`, `Hysteresis` (stateful nonlinear)

### 5.4 Dynamics abstraction

- [x] `MechanicalSystem`, `JaxMechanicalSystem`, `StateSpaceSystem`, `GeneralizedMechanicalSystem`
- [x] `MechanicalSystem` ports `q`, `dq` (`mechanical.py`)
- [x] `Manipulator` base — `p`, `pdot`, `forward_kinematics`, `J` (`manipulator.py`)
- [x] Rebase `dynamics/catalog/manipulators/arms.py` on `Manipulator`
- [ ] Optional `f_ext` input port for external end-effector forces

### 5.5 Planning

- [x] Trajectory optimization (direct collocation, shooting, multiple shooting)
- [x] **MPC / trajopt compile-once** —
  `TrajectoryOptimizationPlanner.compile_parametric_program` +
  `solve_trajectory_from` (parametric `x0`); controllers in `control/mpc/`;
  primary demos under `examples/scripts/mpc/`
  (`demo_mpc_minimal`, `demo_mpc_minimal_numpy`, `demo_mpc_dual_rate`, `demo_mpc_path`,
  `demo_mpc_circuit`, `demo_mpc_slalom`, `demo_mpc_spatial`)
- [x] **NumPy MPC rebuild mode** — `compile_backend='numpy'` or `'direct'` skips
  parametric compile; each replan tick transcribes + solves via
  `solve_trajectory_from` (no JAX required).
- [x] **Online `params` façade (E7 slim)** — `params=None`/`{}` for x0-only;
  reserved `scene` key and `ProblemParameters.scene` placeholder;
  `NotImplementedError` until full bind; latch forwards `params` on
  `compute_command`.
- [x] **Broadcast + dual-rate (E8)** —
  `generate_nominal_interpolator` + `get_nominal_u/x/u_dot/x_dot`;
  `dual_rate_computer(dt_broadcast)`; default `@` stays `u_ff` ZOH.
- [x] **Phase F — MPC package home** — product System family in
  `control/mpc/` (`controller.py`, `utilities.py`, `viz.py`); hard-cut
  former `planning/mpc`.
- [x] **Planning UI simplification** — Simulator-style planner constructors
  (flat kwargs for trajopt / RRT / DP); teach demos use `control.mpc` and
  explicit `transcription="…"` without `*Options` imports.
- [ ] **Scene params / online bind (pipeline B)** — *wanted later.*
  `ObstacleBank`, `J(z, p)` / `bind(p)`, indexed scene overrides (moving
  obstacles, scenario sweeps without NLP rebuild). Notes in
  [planning-pipeline-architecture.md](docs/plans/planning-pipeline-architecture.md).
- [ ] **Optimizer wiring — parametric trajopt / MPC** — unify
  `optimizer_method` (SciPy SLSQP, IPOPT, …) across offline rebuild and
  `compile_parametric_program` fast path; shared `make_optimizer_backend`
  factory instead of planner-local SciPy-only registry. Analysis and steps in
  [optimizer-parametric-wiring.md](docs/plans/optimizer-parametric-wiring.md).
- [ ] **Parametric `core/` primitives** (promoted to P1) — call-time `params`
  overrides on `Shape.sdf`, `Set.margin`, and `CostFunction.g`/`h` (e.g. `BallSet`
  center/radius, `QuadraticCost` weights). Signatures exist; frozen attributes are the
  only source of truth today.
- [ ] `trajectory_generation/` — polynomial / min-snap
- [x] `policy_synthesis/` — `DynamicProgrammingPlanner`, `StateSpaceGrid`,
  `loop` / `numpy` / `jax` backends, `LookupTableController`, `PolicyEvaluator`
- [x] `search/` — `RRTPlanner`, `RRTStarPlanner`, extenders, steering, tree
- [ ] Trajectory post-filter (Butterworth `filtfilt`)

### 5.5a Step / hybrid simulation (subsidiary)

Narrow add-on for discrete control in the loop (MPC, SMC, sampled regulators) on
continuous plants — not full Simulink parity. Minilink remains focused on continuous-time as the primary framework. If the hybrid module matures significantly, it may become the default for discontinuous dynamics.

- [x] **Phase 0** — `WiredDiagramMixin` in `core/wiring.py`; `DiagramSystem` delegates;
  continuous diagram API unchanged
- [x] **Phase 1** — `StepSystem`, `StepRollout`, `compile()` step branch, `compute_rollout` / `plot_rollout`
- [x] **Phase 2** — `StepDiagramSystem`, `compile_step_diagram`, `compute_rollout`
- [x] **Phase 3** — `discretize()` in `minilink/analysis/discretize.py`
- [x] **Phase 4** — `StepSchedule`, `Computer` (`.tick(u)`, double buffer)
- [x] **Phase 5** — `HybridDiagram`, `HybridSimulator`, `HybridSimResult`, multi-rate hybrid demo, Pyro SMC hybrid compare *(5a)*
- [x] **Phase 5** — fine plant recording (`integrate_zoh_rollout`, `plant_dt_inner`); façade `traj` / `last_result` / `rollout` / `animate()`
- [x] **Phase 5c** — `HybridDiagram.plot_diagram`, `build_hybrid_topology`, Mermaid/Graphviz export, `abstract_boundary` topology *(shortcuts: `hybrid_closed_loop`, `Computer @ plant`)*
- [x] **Phase 6a–6b / E2–E5** — `ModelPredictiveController` (algebraic or warm-start
  `StepSystem`, ports `u_ff`/`x_ff`/`z`), hybrid demos, `mpc_plans_from_rollout`

### 5.6 Estimation and identification

- [ ] `estimation/luenberger.py`, `kalman.py`, `ekf.py`, `recursive.py`
- [ ] `identification/fitting.py` — one verb for physical params and NN weights

### 5.7 Interfaces

- [ ] `gymnasium.py` — diagram as RL env (train outside)
- [ ] `flax.py`, `torch.py` — external model → static `System`
- [ ] `ros2.py` — ROS 2 node exporter (wraps a minilink block as a ROS 2 node)
- [ ] Cosimulation / FMI
- [ ] **External multibody engine leaf (MJX-first candidate)** — optional
  `DynamicSystem` (or sibling `StepSystem`) wrapper for complex multibody +
  contact plants; thin adapter, lazy/`minilink[mjx]` extra; catalog teaching
  plants stay default. Spike when scheduled: load model → ports →
  `ctl @ plant`. Not the product center ([§5.11](#511-product-vision--landscape-position)).

### 5.8 Catalog capability gaps (not EoM)

- [ ] Obstacle collision layer (replace pyro `*withObstacles` plants)
- [ ] `Pacejka` tire model
- [ ] Stochastic forcing / `NoiseSignal` blocks

### 5.9 Quarantine graduation

- [ ] `symbolic/` as dynamics-authoring tool
- [ ] Hand-rolled contact in `dynamics/engines/` — graduate **or** deprecate in
  favor of an external engine leaf ([§5.7](#57-interfaces)) once that leaf is
  TRL ≥ 3

### 5.10 Out of scope (by decision)

Full Simulink / arbitrary multi-clock hybrid parity, event-driven switching
(guards, impacts) as a *framework* feature, RNN policy blocks, becoming a
standalone batched RL physics engine — see [DESIGN.md §3](DESIGN.md). The
**narrow** step/hybrid subset in §5.5a is in scope as a subsidiary program; it
does not replace the continuous-time core. Pyro's `sys2game` framework is not
ported; live interaction is minilink's own `simulation/realtime/` tool
(`RealtimeSimulator` + I/O contracts), not a pyro port.

**In scope as leaves (not as core rewrite):** optional wrappers around external
multibody/contact engines ([§5.7](#57-interfaces)) — *using* those engines for
complex plants, not rebuilding a physics OS inside minilink.

### 5.11 Product vision & landscape position

**Status:** directional vision — wording needs maintainer sign-off
([§4](#4-review-queue-needs-maintainer-sign-off)). Companion to the feature
backlog (§5.0–5.10) and pyro port (§6). Does **not** change core contracts
([DESIGN.md](DESIGN.md)).

#### Product vision

Minilink is a **Python/JAX block-diagram toolbox** for **modeling, simulating,
controlling, optimizing, and learning** with dynamical systems — with equations
that read like textbook math (`dx = f(x,u,t;p)`).

**Distinct edge:** one object model for plants, controllers, diagrams, and
NN/ID blocks. Compose (`@`, `>>`), simulate, analyze, optimize
trajectories/policies, and differentiate / `jit` through the same `f` via
compile backends — without splitting a “sim stack” from a “learning stack.”

**Primary use cases**

1. **Model & teach** — readable continuous plants (catalog or custom
   `DynamicSystem`) and closed-loop diagrams.
2. **Control** — classical, model-based, and hybrid digital loops (sampled
   MPC/SMC) on those diagrams.
3. **Optimize** — trajopt / MPC / planning (search, DP) posed on the same
   `System` and costs/sets.
4. **Learn** — identify parameters, residual dynamics, or NN policies with
   gradients through compiled dynamics.
5. **Scale out plants later** — optional external multibody engines as leaves
   when needed ([§5.7](#57-interfaces)); not the product center.

**Not trying to be:** a Simulink GUI/DAE product, a Multibody/contact OS, an
OCP modeling language, or a batched RL physics engine.

#### Positioning vs other toolboxes

| Toolbox | They own | Minilink vs them |
| --- | --- | --- |
| **Simulink** (+ Stateflow/Simscape) | Industrial diagrams, GUI, DAE, codegen | Same block-diagram idea; code-first, causal, open, differentiable — no GUI/DAE ambition |
| **MATLAB** (CST etc.) | Classical LTI / frequency design | Neighbor for LTI; we center nonlinear systems + optimize/learn in one Python stack |
| **Drake** | Multibody, contact, events, deep MathProg | Complement for teaching / reduced-order / JAX-learning loops; not a second MultibodyPlant |
| **MuJoCo / MJX** | Fast multibody + contact physics | Physics backend we can wrap later; they don’t own control-diagram + optimize/learn UX |
| **CasADi** | Symbolic AD → NLP/OCP | Opt is a *tool on Systems*; we own diagram/sim/control/learn surface around it |
| **acados / Crocoddyl** | Fast deployed MPC/DDP | Solver peers; we stay the Systems lab that can call solvers |
| **Pinocchio** | Fast RBD + derivatives | Algorithm/engine peer — not a diagram framework |
| **Modelica** | Acausal physical networks | We stay causal ODE/blocks (better fit for AD and learning) |
| **python-control** | Classical control in Python | Interop; they stop at LTI, we continue nonlinear + optimize/learn |
| **Brax / similar** | Batched differentiable physics for RL | Neighbor in JAX; we are Systems+control+opt, not an RL physics engine |
| **Pyro** | Teaching dynamics lineage | Successor: keep readability; add diagrams, compile/JAX, optimize, learn |

**Claim:** *Systems-first lab for simulate → control → optimize → learn in
Python/JAX — not a physics OS, not an OCP language, not a Simulink
replacement.*

#### Practices to incorporate

- **Freeze the composition grammar early** — `System` / ports / diagrams /
  continuous `f` stay stable; grow capability in leaves and tools, not by
  redefining the core.
- **Put hard physics behind optional leaves** — deep multibody/contact (when
  needed) as adapters, never as a rewrite of `DynamicSystem` or
  `DiagramSystem`.
- **Expose structure, don’t black-box dynamics** — compiled evaluators,
  parametric `f_p`, Jacobians, and optimize/learn paths share one dynamics
  surface.
- **Succeed in parallel, retire on a calendar** — new APIs land beside old ones
  (quarantine / extras); migrate demos/tests, then deprecate with an explicit
  window.
- **Demo-gate maturity** — representative closed-loops and cross-pillar scripts
  (sim + control + optimize/learn) before calling a band done.
- **Document hybrid sample semantics** — ZOH / sample timing for discrete
  control on continuous plants is part of the contract, not tribal knowledge.
- **Validate wiring, don’t solve diagram DAEs** — detect algebraic loops; keep
  the graph causal/explicit.
- **Lock build vs run** — wire/validate/compile freezes structure; simulation
  and optimization run on that freeze (params overrides stay call-time where
  designed).
- **Be hard where the identity is** — parity gates on compile vs reference, JAX
  twins, discontinuous closed-loop solver behavior; don’t spend the budget on a
  second physics OS.

## 6. Pyro 2.0 port status

Snapshot vs [SherbyRobotics/pyro](https://github.com/SherbyRobotics/pyro) (June 2026).

| Bucket | Pyro | Minilink | Gap |
| --- | ---: | ---: | ---: |
| Catalog plants | 40+ | 40+ | **0** (QA complete) |
| Library modules | 38 | ~25 equivalent | **~10 tools** (control nonlinear/robot, DP, RRT, traj gen, estimation, interfaces) |
| Example scripts | 195 | 60 | **~135** |
| Course notebooks | 3 | 7 (new topics) | different mix |

### Port phases (from gap doc)

| Phase | Focus | Unblocks |
| --- | --- | --- |
| **A** | ~~`Manipulator` + computed torque + sliding mode + `robotic.py`~~ (library landed) | representative closed-loop demos per plant band |
| **B** | ~~DP/RRT~~ + polynomial traj gen + trajectory LQR | remaining: traj gen, traj LQR |
| **C** | Estimation + identification + Gym interface | LQG + RL demos |
| **D** | Frequency completion + planning cost variants | analysis + DP reachability |
| **E** | Obstacles, Pacejka, stochastic | specialized catalog |
| **F** | Demo audit (`demos_by_system/`, courses, projects) | pyro parity visibility |
| **G** | Export policy, compile review, migration guide, TRL 8 | release criteria |

### Definition of done (pyro 2.0)

1. Every **in-scope** pyro library module has a minilink home or documented replacement.
2. Every **in-scope** pyro plant folder has ≥1 representative closed-loop demo.
3. DP/RRT approved and landed **or** explicitly dropped with alternatives documented.
4. Robot control suite (computed torque, sliding mode, robotic) at TRL ≥ 6.
5. `identification/fitting.py` covers the params-gradient workflow.
6. README includes a pyro → minilink API migration guide.

Full per-module backlog:
[docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md).

## 7. Suggested improvements (architecture review)

Directional suggestions from a 2026-07 architecture / API review pass over core
contracts, compile/evaluators, composition, optimization/planning, simulation
facades, and graphics. **Not scheduled priorities** — does not displace P0–P5
or §5 backlog items already tracked above. Use as a hardening backlog after
maintainer triage; promote selected rows into §3 / §4 / §5 when accepted.

### 7.1 Core contracts (`f` / ports / params)

- [ ] Teach the three-layer output story in one place: model `h` (primary `"y"`)
  → per-port `compute` → evaluator `outputs` dict (boundary only).
- [ ] Document flat-`u` unpacking (`get_port_values_from_u`) as the multi-port
  idiom; optional small unpack helper only if it stays out of equation lines.
- [ ] Name params shapes explicitly: leaf flat dict vs diagram nested
  `{sys_id: …}`; keep `params is None` → `self.params` semantics.
- [ ] Keep arrays for signals and dict/pytree for params — do **not** homogenize
  to all-array, all-dict, or Drake-style Context on the leaf equation path.

### 7.2 Compile and evaluators

- [ ] Publish a compact evaluator API matrix (user / simulator / trajopt–ID;
  mark JAX-only trace methods).
- [ ] Clarify or rename params freeze: leaf snapshot vs diagram live-default /
  `bind_params`; one `compile(..., snapshot_params=…)` entry for leaves and
  diagrams.
- [ ] Shrink advertised public surface to `f`/`f_p`, `outputs`/`outputs_p`,
  `step`/`step_p`, plus a short integrate list; treat full `_jit` / trace /
  integrate grids as tool-author API.
- [ ] Normalize JAX return types (device arrays in JAX methods; NumPy at
  Trajectory / plot boundaries); complete or shrink `_jit` alias contract.
- [ ] Make `auto` backend fallback visible under `verbose=` (and surface the
  swallowed JAX exception when useful).
- [ ] Table-driven NumPy↔JAX parity tests for leaf + diagram; prefer splitting
  `jax_evaluators.py` by concern over redesigning typed evaluators.

### 7.3 Diagram composition

- [ ] Keep `add_subsystem` / `connect` as the documented canonical API; freeze
  sugar grammar (`+`, `>>`, `@`, `autowire`, `feedback=`).
- [ ] Add verbose / “explain wiring” mode for shortcuts (chosen ids, default
  ports, feedback path, Mux insertion).
- [ ] Contract tests for multi-output `>>`, nonstandard port names,
  diagram–diagram flatten edges, hybrid multi-leaf cases — before more sugar.
- [ ] Keep `autowire` conservative (no Mux); document Mux only via
  `feedback="qdq"` / `closed_loop_qdq`.
- [ ] Split `composition.py` by responsibility only after tests pin behavior.

### 7.4 Optimization and planning

- [ ] Unify optimizer wiring (`make_optimizer_backend`) so offline trajopt and
  parametric MPC share SciPy / Ipopt methods.
- [ ] Promote `ParametricMathematicalProgram` into `optimization/` when
  `J(z, p)` / `h(z, p)` land; keep transcriptions in planning.
- [ ] Finish scene / online param pipeline B (bind `p` without NLP rebuild)
  before adding more planning features.
- [ ] Sparse derivative story before climbing past TRL 5 on long-horizon trajopt.
- [ ] Keep `PlanningProblem` declarative (no solver grid knobs on the problem);
  keep MPC in `control/mpc` as a product wrapper, not a planning subtype.
- [ ] Optional CasADi / acados as backends later — behind the same program /
  planner façade, not a rewrite.

### 7.5 Trajectory, simulation, facades

- [ ] `SimulationOptions` ergonomic bag (solver, grid, backend, verbose,
  warnings) — cleanup, not redesign.
- [ ] Trajectory metadata (solver, backend, dt policy, discontinuous caveats);
  optional readonly arrays / `readonly()` helper.
- [ ] `reconstruct_internal_signals` reuse sim evaluator/backend (do not always
  force NumPy recompile).
- [ ] Hard warning when discontinuous controllers sit on continuous
  `DiagramSystem`; recommend `Computer` / `HybridDiagram`.
- [ ] Keep `self.traj` / `rollout` / `last_result` as convenience caches only;
  document “latest result” semantics for scripts.

### 7.6 Graphical rendering

- [ ] Cache namespaced diagram kinematic geometry; keep shared `"world"` rule.
- [ ] Lazy renderer registry (import backend only when selected).
- [ ] Backend capability flags (`supports_3d`, `supports_dynamic_geometry_native`,
  …) with early clear errors.
- [ ] Primitive parity smoke tests across matplotlib / plotly / meshcat / pygame.
- [ ] Decide optional `KinematicModel` delegate: adopt only if it cuts plant
  boilerplate without moving math off `System`; otherwise drop from §4.
- [ ] Keep overlays (`SceneHistory`, `Replay`, MPC viz) as the extension point —
  do not special-case more viz inside `Animator`.

### 7.7 Cross-cutting (docs / mental model)

- [ ] Short “mental model” page or DESIGN subsection: System / ports / `f` vs
  `step` / params tiers / compile tiers / when to drop from `@` to explicit
  wiring.
- [ ] Demo-gate the product vision with a few cross-pillar scripts (sim +
  control + trajopt/MPC + params gradient) before broad API freeze.
- [ ] Resolve stale wording (e.g. ROADMAP `h_p` vs DESIGN `outputs_p`; step /
  hybrid parity notes vs older drop rows in pyro gap doc).
