# Tool Migration Notes

Working notes for porting pyro's **tools** (verbs and control laws) into minilink,
the analog of `catalog-migration-notes.md` for the non-plant code. Pyro's formulas
are preserved; its legacy interfaces (`ContinuousDynamicSystem`, `StaticController`,
`ClosedLoopSystem`, `x_lb/u_lb`, `ObservedSystem`) are dropped in favor of `System`
+ explicit ports, diagram composition, `core/sets`, and `Trajectory`.

## Conventions (decided in review)

- **Autonomy gate**: a component is migrated only when its minilink home is
  unambiguous, its interface has one obvious shape, and no maintainer decision is
  needed. Anything that raises a real design question is deferred and logged, not
  guessed.
- **Design factories return blocks** (dependency law): a control/estimation library
  ships `factory(arrays...) -> block` with array-in / block-out signatures and never
  imports a tool. Linearization (the array source) lives in `analysis/`.
- **Native-array equation paths**: `xp = array_module(u)`, bare signatures, params
  unpacked to locals, no `self.`/dict-indexing in the math.

## Migrated (this tranche)

### blocks/
- **routing.py** — `Sum` (signed junction), `Gain` (matrix/diag/scalar, gain in
  `params["K"]`), `Mux`, `Demux`. `StaticSystem`s, explicit ports, dims fixed at
  construction. `Switch` deferred (selection-semantics decision).
- **nonlinear.py** — `Saturation` (`xp.clip`), `DeadZone`, `Relay` (`xp.sign`).
  Memoryless `StaticSystem`s, JAX-traceable. `RateLimiter`/`Hysteresis` (stateful)
  deferred to Tier B.
- **filters.py** — `LowPassFilter` (Butterworth, cutoff in **Hz**, first-order
  default), `NotchFilter`, `Washout` as thin `TransferFunction` subclasses.
- **sources.py** — `TrajectorySource` (replay a stored `u(t)` by linear
  interpolation; `from_trajectory` classmethod). Modern replacement for pyro's
  `OpenLoopController`; no prior equivalent existed in `sources.py`.

### control/
- **linear.py** — `ProportionalController` now covers SISO **and** MIMO
  `u = K(r - y)` (scalar or `(m, p)` gain in `params["K"]`); the old scalar
  `PropController` was merged into it and removed. Added `PIDController`
  (continuous P+I+D, integral state, **no anti-windup** in v1) and
  `LinearFeedbackController` (`u = ubar - K(x - r)`, reference port `r` defaulting
  to `xbar`). `PDController` unchanged.
- **lqr.py** — `lqr_gain(A,B,Q,R)` (continuous ARE via `scipy.linalg.solve_continuous_are`)
  and `lqr(A,B,Q,R,xbar,ubar) -> LinearFeedbackController`. Pure arrays-in/block-out,
  no tool imports.

### analysis/
- **linearize.py** — `linearize_matrices(sys, x_bar, u_bar, ...) -> A,B,C,D`
  is the base API; `linearize(...) -> LTISystem` is the wrapper. Supports FD
  and JAX-preferred linearization with warning-backed FD fallback.
- **structural.py** — `controllability(A,B)`, `observability(A,C)` → `StructuralResult`
  (Kalman matrix, rank, full-rank verdict).
- **equilibria.py** — `find_equilibrium(sys, x_guess, u, t, params)` via
  `scipy.optimize.fsolve` on `f`.

### docs
- Fixed stale `docs/api/*.rst` module paths (`minilink.compile.*` →
  `minilink.core.compile.*`; `core.blocks.*` → top-level `blocks.*`;
  `physics.*` → `dynamics.engines.*`; dropped `control.pendulum_pd`). Added
  `api/blocks.rst`, `api/control.rst`, `api/analysis.rst`; commented out the
  not-yet-existing `planning.search.rrt` / `policy_synthesis.dynamic_programming`
  automodules.

Verification: each module has a `__main__` demo; unit tests in
`tests/unittest/test_signal_blocks.py`, `test_control_linear.py`,
`test_analysis_control.py` (incl. closed-loop PID steady-state and LQR
inverted-pendulum stabilization). `examples/scripts/analysis/demo_linearize_lqr.py`
is the end-to-end demo.

## Deferred (logged for later sessions)

- **`linearize_and_lqr(sys, …)`** convenience — would require the `control` library
  to import the `analysis` tool, violating the dependency law. Belongs in a tool
  (e.g. `analysis/` or a future control-design tool), not in `control/lqr.py`.
  Workaround today: the two-step `linearize(...)` → `lqr(...)` shown in the demo.
- **`Switch`** routing block — selection semantics (by control signal / by index /
  by time) undecided.
- **PID anti-windup / output saturation** — v1 is plain PID.
- **JAX-exact linearization** — available via ``linearize_matrices(..., method='jax')``,
  ``linearize(..., method='jax')``, and ``modal_analysis(..., method='jax')``.
- **Tier B tools** (see ROADMAP §5): `analysis/frequency.py`,
  `control/computed_torque.py`, `sliding_mode.py`, `robotic.py`,
  `estimation/{luenberger,kalman}.py`, `planning/trajectory_generation/`,
  `interfaces/gymnasium.py`, catalog `Pacejka` tire, stateful nonlinear blocks.
- **Big dedicated jobs**: RRT (`planning/search/`) and value iteration /
  dynamic programming (`planning/policy_synthesis/` + `control/policy.py`).
