# Rulings on the open questions (2026-10-25)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the questions of [QUESTIONS.md](../dev/QUESTIONS.md) that need no reading of code,
settled one by one in a session. Each ruling names the question, the answer, and what changed.
Numbering continues the rulings record of 2026-10-22 (R1–R90).

- **R91 (2026-10-25) Gradient descent by hand: the closing PID example, for now.** The one place
  that writes the descent loop out, then the `Optimizer` doing the same (RULES 6.15), is the
  closing PID example (`examples/demos/compile/pid_autotuning_jax.py`), not tutorial 07 as R75
  said. A tutorial follows later, once the generic optimizer and the mathematical-program system
  are extended. Changed: RULES 6.15; a Later line in TODO.
- **R92 (2026-10-25) The problem's key reaches the noise inside the loop, now.** Each trial of
  `evaluate` realizes every random block of the evaluated loop (`System.realize`) under a seed the
  library's cipher derives from the evaluation's key and the trial, so the same integer key gives
  the same noise on both backends. Sized first: about half a day, no current number changes (no
  test, demo or notebook evaluates a noisy loop over several trials). The other randomness jobs
  stay in v0.4: the same key giving the same starts and parameters on both backends (the rest of
  RN-4, every stochastic number changing once), the disturbance ports as signals (the rest of
  RN-5), the distributions reading `params` (RN-2), `NoiseSource` (RN-3).
- **R93 (2026-10-25) A diagram's `params=None` is the snapshot taken at compile, on both
  backends.** A compiled diagram copies each block's `params` at compile, as a compiled single
  system does; an edit after compile needs a recompile or an explicit `params`. Today's
  `bind_params=True` becomes the behaviour, the flag goes. Settles the NumPy / JAX split found on
  2026-10-24 (NumPy read live parameters, JAX the first call's).
- **R94 (2026-10-25) A diverged run scores the cost bound plus the price.** In `score_run` a run
  that is not finite is still integrated on zeros (no NaN), but its cost term in `J` is the
  problem's cost bound, the largest cost a finite run can reach, where it was `0`: with the price
  on, `J = cost_bound + price`, worse than any finite run, failed or not, and finite. Amends R31,
  R32; only diverged runs with the price on change.
- **R95 (2026-10-25) The tabular learner steps with its grid's solver by default.**
  `TabularLearningPlanner`'s `solver` defaults to the grid's (Euler, R8), so with the defaults the
  learner and value iteration solve the same Markov decision process. No course notebook or demo
  uses the learner today (checked); only its tests' expected numbers move.
- **R96 (2026-10-25) The Gym bridge stops at the horizon.** `Sys2Gym` truncates when `t_new` reaches
  `tf` (a float tolerance), not past it, so an episode of `tf = 1`, `dt = 0.1` is 10 steps, as in
  minilink's RL environment; the class the GRO860 `gymnasium_interface` notebook teaches changes
  the same way. The SB3 notebooks train one step shorter per episode.
- **R97 (2026-10-25) The same cost object added twice is refused.** A sum that would hold one cost
  object twice (`q + 10 * q`, `cost + cost`) is refused with a message: write `11 * q` (or
  `2 * cost`), or pass a copy. Today the value was right but the object's parameters sat twice in
  the parameter tree, so tuning them moved one copy.
- **R98 (2026-10-25) `closed_loop` and `feedback` join the `minilink.control` facade**, beside the
  controllers and the design verbs; they stay on the root prelude.
- **R99 (2026-10-25) A chain carries every operand's name.** `Step() >> PID() >> Pendulum()` is
  named `"Step >> PID >> Pendulum"`; today the left diagram's name was kept (`"Step >> PID"`).
  Display only.
- **R100 (2026-10-25) Three dependency-law exceptions, written down.** DESIGN §3 gains: the LQR
  planner imports `control.design` and `control.state` (it solves with the design verb and returns
  its state-feedback law); `core/discrete/` and `System.__mod__` import `simulation.discrete`
  lazily (the `%` operator builds a `Computer`, a hybrid diagram simulates through the
  `HybridSimulator`); `graphical/diagrams/` imports `blocks.routing.Sum` and `StepSchedule` (to
  draw a summing junction and a rate). Moving `Computer` into `core/discrete/` is reconsidered with
  the sampled loop (S31, v0.4).
- **R101 (2026-10-25) Computed torque on a body-frame plant** (the one mechanical family, S32,
  question 1): `v̇_des = Kp N(q)⁻¹ (q_d − q) + Kd (v_d − v)`, today's law when `N = I` (identical
  numbers for every current plant); a plant whose `N` is not square is refused.
- **R102 (2026-10-25) A model-based law refuses an underactuated plant** (S32, question 2): a clear
  error at construction (the plant has fewer actuators than coordinates), not a wiring failure; no
  least-squares allocation. S32's done-when then runs `ComputedTorque` on a fully actuated
  body-frame test body, not on the boat or the plane (both underactuated). Question 3
  (`DynamicBicycle`'s base) stays open, postponed to later work.
- **R103 (2026-10-25) Sixteen of the overnight run's decisions accepted as taken**
  ([the run's log, §2](2026-10-24-v03-run-decisions.md#2-decisions)): OD-1 (`ProblemEnv` kept as
  internal machinery of the Gym bridge, off the facade), OD-2 (the teaching-surface test reads the
  root prelude as a subset of the bands), OD-3 (the neural-controller demo on
  `MLP(2, 1, hidden=(8,), seed=2)`), OD-4 (the package moves and renames before the catalog
  clean-up), OD-5 (MPC under `planning/trajectory_optimization/mpc/`, offered by
  `minilink.control`), OD-6 (`ModelPredictiveController` in `mpc/predictive.py`, its constructor
  frozen), OD-11 (the step formulas stay in `core/compile`), OD-13 (the recorded plant input is
  `plant.u`), OD-15 (the new optimizer presets named by algorithm, no aliases), OD-16
  (`Optimizer`'s `z0` optional, read from the program), OD-19 (the `car_trajopt` notebook's
  syntax fix), OD-22 (cost term ids are the class name verbatim, as block ids), OD-28
  (`compare`'s two forms), OD-31 (no leaf scaling), OD-34 (the README snippet gains
  `U=plant.input_box`), OD-36 (`Set.contains` derived from the margin on the array module). With
  OD-9 confirmed and OD-10 superseded, sixteen stay open for review and two wait on their own
  questions (OD-32, OD-33).
- **R104 (2026-10-25) A sampled controller's memory is its state, its draws its seed and the
  tick** (end-state-api.md D14, the sampled loop S31): no channel of its own. MPPI's warm start or
  a diffusion policy's chunk lives in the `StepSystem`'s state; its draws come from its own
  `params["seed"]` and the tick, as `WhiteNoise`'s do, so `loop.realize(key)` seeds it like any
  random block. The RL collectors' hand-threaded keys move to the same rule. The run-cache
  question (D13) was put off to the sampled-loop work.
- **R105 (2026-10-25) `problem.compile(policy)` dispatches on the policy's kind**
  (end-state-api.md D15, V1 step 7b): a continuous law runs the loop as a diagram, a `Computer`
  (built by `%`) runs the sampled kernel's JAX face; one `compile`, one `evaluate`, the same `J`,
  criterion and Monte Carlo for both. v0.3 keeps the refusal (`require_continuous`); the sampled
  branch lands with the sampled loop (S31, v0.4).
- **R106 (2026-10-25) The path-integral planner is `MPPIPlanner`, its record `MPPIRecord`**
  (end-state-api.md D26, mppi.md ask 1), by the `LQRPlanner` precedent; the docstring's first line
  carries the long form, path integral control.
- **R107 (2026-10-25) MPPI takes one plant draw per control sample by default**
  (end-state-api.md D28, mppi.md ask 3): `n_plant_samples=1` on a stochastic problem, the bias
  stated in the docstring; the user raises it knowingly, as with `n_trials`.
- **R108 (2026-10-25) The rollout environment moves to `planning/`** (end-state-api.md D30,
  mppi.md ask 5), amending D30's recommendation: `RolloutEnvironment` is the planning problem
  sampled at `dt` (a Markov decision process: one step, its reward `−g·dt`, the failure price, the
  horizon), so it is a planning object, beside `score_run`; not core, which never imports the
  problem. It moves to `planning/environment.py` as a housekeeping step before the MPPI planner
  (v0.4), the `minilink.learning.reinforcement_learning` import kept; it still depends on the
  step model only (R13). The failure, price and horizon rules, written twice today (the
  environment's step and `score_run`), get one owner later.
- **R109 (2026-10-25) Pyro's game demos stay dropped, ported on demand.** The `sys2game` games
  (pendulum, double pendulum, rocket, boat, the joystick tests) are not ported; the two realtime
  games (`examples/demos/realtime/game_cartpole.py`, `game_ur5_setpoint.py`) are the pattern, and
  a game is written on `simulation/realtime/` only when a course or behavior cloning needs one.
- **R110 (2026-10-25) The parity table's hybrid row splits.** Mixed-rate simulation (a sampled
  controller on a continuous plant) is done: `HybridSimulator` and `Computer`. Event-driven
  switching (guards and resets) stays out of scope (ROADMAP §7).
- **R111 (2026-10-25) Lyapunov certificates are a research-lane analysis tool**, not a GRO860
  checklist row: `region_of_attraction` and `LyapunovCertificate` stay provisional, the lecture
  uses them as they are, and the field work (A1, v0.4) reshapes `V` with no course deadline.
- **R112 (2026-10-25) `PlanningProblem.metadata` is retired.** No library tool reads it and no
  demo or notebook passes it; the keyword, its coercion and its one test go before the freeze.
  A user keeps notes in their own variables. (Removing a user-importable keyword, approved here.)
- **R113 (2026-10-25) The pinned Colab cell clones a tag:** `git clone --depth 1 -b v<tag>`, not
  `pip install minilink==<tag>`, so the course notebooks' relative imports of the examples tree
  keep working (repository-structure.md §6 ask 2; V6). Where the course material lives (a course
  repository at v1.0 or not) was put off.
- **R114 (2026-10-25) minilink raises its own error family** (S70): `WiringError(ValueError)`,
  `ShapeError(ValueError)`, `SolverError(RuntimeError)`, each a subclass of what is raised today,
  so every `except ValueError` still catches them; the wiring-time checks (S66) raise them. The
  deprecation-shim question for v1.0 (V4) was put off.
- **R115 (2026-10-25) A Zenodo DOI now; JOSS after v1.0** (V2): the maintainer links the GitHub
  repository on Zenodo once, so each release from v0.3 is citable, with a `CITATION.cff` at the
  root; the JOSS paper and the RULES 6.9 carve-out it needs are decided after v1.0.
- **R116 (2026-10-25) An undeclared feedthrough raises at compile** (S66): the compile probe finds
  an output that moves with an input missing from its `dependencies` and raises, naming the
  block, the port and the fix (`dependencies=("u",)` or `"all"`); the library's under-declared
  blocks are fixed first, in the same work.
- **R117 (2026-10-25) A block missing from a partial params tree reads its compiled snapshot**
  (completing R93): with `ev.f(x, u, t, params={"plant": {...}})`, every block the tree leaves
  out falls back to the parameters taken at compile, on NumPy and JAX alike, not to its live
  `self.params`. **Superseded by R121** (the merge that would have implemented this was
  not written; a missing block is an error).
- **R118 (2026-10-25) `params` are numeric only (no `bool` or `str` switches).** A parameter dictionary
  holds scalars, numeric arrays and integer PRNG seeds only; structural choices that change
  which equation or branch runs (`gravity_comp`, `tracking_ref`, `hold`, `interpolation`) are
  fixed at `__init__` and never stored in `params`. `JointImpedance` and `TaskImpedance` keep
  `gravity_comp=` in `__init__`, remove `"gravity_comp"` from `self.params`, and bind output
  port `"u"` to `ctl_with_gravity` when `gravity_comp=True` and `ctl` when `False`; `@compiled`
  drops `params_filtered_jit` (every leaf of `params` is numeric).
- **R119 (2026-10-25) The three-foundation spine in CONSTITUTION §1.1.** CONSTITUTION §1.1 names
  the three foundations (`System`, `PlanningProblem` / `PlanningSolution`,
  `MathematicalProgram` / `Optimizer`) and the one differentiable path from the model and its
  parameters to the cost that every control, planning, learning, identification and estimation
  tool sits on.
- **R120 (2026-10-25) CONSTITUTION invariant 1 names the sampled loop a simulation tool.** Invariant 1
  states that discrete-time systems (`StepSystem`, `StepDiagramSystem`) form a closed sibling
  algebra on an integer tick, and the sampled loop (`Computer`, `HybridDiagram`) is a
  simulation tool coupling the two clocks rather than an open `System` seam (matching R13).
- **R121 (2026-10-25) A params override is `None` or a complete nested tree.** The compiled
  loop and the uncompiled diagram `f` see `params=None` (the compile snapshot /
  live defaults) or a mapping that names every block. A missing block is an error;
  there is no partial-tree merge in the compile pipeline (`op_params_of` and
  `params_filtered_jit` are gone). Copy `evaluator.params` and replace the block
  you change. `ProblemEvaluator.merged` stays the one convenience merge, at the
  problem boundary, and hands a complete tree down. Supersedes R117.
- **R122 (2026-10-25) Discrete step analysis comes back when solver stability needs it**
  (native-optimizers.md §4.7, option B). The linearization of a step system at a fixed point
  (`A = ∂step/∂x`, from the existing `jacobian`) and its eigenvalues against the unit circle are
  planned, built with NS-5 (convergence of an optimization process analysed as stability) when
  that work requires them. The rest of the z-domain family the 2026-10-17 ruling dropped
  (exact `c2d`, the z-plane pole map, discrete Bode, root locus) stays dropped.
- **R123 (2026-10-26) The MPC block's tick latch goes: the computational delay** (sampled-path.md
  §8.3, option A). The block solves in `step` from the measurement `y_k` and its outputs read
  the state, so the command applied at tick `k` comes from the measurement at `k − 1`, as on a
  real computer whose solve takes time; one solve per tick with no hidden cache, and the block is
  a pure step block. Lands with the sampled loop (S31); the MPC's closed-loop numbers move by the
  one-period delay and its baselines are refreshed then (R80). A combined `update → (x_next, y)`
  for a zero-delay output stays a study (option B, a core contract change).
