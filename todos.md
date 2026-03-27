# Minilink Development TODOs

The following tasks are structured in phases to systematically improve the API, architecture, and maintainability of the project, culminating in full Pyro 2.0 feature parity.

> **Current maturity:**
> - **`core/` + `blocks/`** — **1.0 stable.** The port/signal model, `System` hierarchy, `DiagramSystem` compilation, `Simulator`, and built-in blocks are the settled foundation. Build on top of these.
> - **`compile/`** (IR, `CompiledNumpyDiagram`) — **MVP prototype.** Functional but API may change; not yet hardened.
> - **`graphical/`** (renderers, animation, plotting, graphviz) — **MVP prototype.** Works for demos but expect rough edges and API evolution.
> - **JAX paths** (`compile_jax`, `f_fast_jax`, `jax_utils`) — **MVP prototype.** Proof of concept for benchmarking; not production-ready.
> - **`control/`, `planning/`, `analysis/`** — **Planned.** Empty stubs.

---

### Phase 1: Modern Python Project Tooling
- [x] **Implement `pyproject.toml`:** Use a modern build backend (Hatch) to manage dependencies and project metadata.
- [x] **Continuous Integration (CI):** Set up `.github/workflows/test.yml` to run `pytest` on push/PR.
- [x] **Linting and Formatting:** Introduce `ruff` for linting and formatting.
- [x] **Type Hinting:** Add `mypy` and use Python type hints extensively.

---

### Phase 2: Critical Bug Fixes (P0)
- [ ] **Fix `solve_ivp` External Inputs:** The `solver == "scipy"` branch in `Simulator.solve()` calls `f_fast(x, np.array([]), t)`, ignoring external diagram inputs. Fix to poll `get_u_from_input_ports(t)` and pass the full `u` vector, matching the Euler/discrete paths.
- [ ] **Fix Trajectory Shape Documentation:** The `Trajectory` docstring says `(time-steps, n)` but actual layout is `(n, time-steps)`. Rename `traj.n` / `traj.m` to `traj.state_dim` / `traj.input_dim` to avoid shadowing the system dimension convention.
- [x] **Fix Stochastic Input Logging:** Make stochastic inputs deterministic via seed-based pre-generation. *(Addressed for `WhiteNoise`.)*

---

### Phase 3: Core Architecture Fixes (P1)
- [x] **Decouple `framework.py` from Simulation/Graphics:** Converted all top-level imports of `Simulator`, `Animator`, `graphe`, and `primitives` to lazy imports inside method bodies. `framework.py` and `diagram.py` now only import `numpy` at module level. Convenience methods (`compute_trajectory`, `animate`, `render`, `game`, `get_graphe`, etc.) remain on `System` for UX. Headless/JAX-only usage is now unblocked.
- [ ] **Eliminate Shared Mutable `global_signals`:** Make `f_fast` allocate a fresh buffer per call (like `CompiledNumpyDiagram` already does). The cost of one `np.zeros()` per ODE step is negligible vs. compute. This fixes reentrancy for parallel scenarios and adaptive ODE steppers.
- [ ] **Extract Gather/Dispatch Helper:** The `src_type == 0/1/2` signal-gathering loop appears 6+ times. Extract into a shared `gather_local_u(gather_sources, u_dim, global_signals, u)` function. Reduces maintenance hazard when adding new source types (e.g., parameters).
- [ ] **Populate `__init__.py` with Public API:** Define the public surface in `minilink/__init__.py` (e.g., `from minilink import System, DynamicSystem, StaticSystem, DiagramSystem, Integrator, Simulator, Trajectory`). Users should not need to know internal module paths.
- [ ] **Thread `params` Through Compiled Paths:** Add `params` to the `f_fast` signature and propagate through execution plans. Required for parametric optimization and JAX differentiation through system parameters.

---

### Phase 4: API UX Improvements
- [ ] **Declarative Port Definitions:** Use Python class properties or dataclass-like fields to define ports on systems cleanly, removing verbose `add_input_port` calls inside `__init__`.
- [ ] **Reference-Based Connections:** Allow connections using port object references (e.g., `diagram.connect(step.outputs.y, ctl.inputs.ref)`).
- [ ] **Operator Overloading for Composition:** Introduce `>>` (series), `+` (closed-loop), and `|` (parallel) syntactic sugar (e.g., `closed_loop = ctl + plant` or `pipeline = source >> ctl >> plant`).
- [ ] **Trajectory Interpolation:** Add `traj.t2u(t)`, `traj.t2x(t)` methods for querying trajectories at arbitrary times via interpolation. Required for feedforward controllers and trajectory-following planners.

---

### Phase 5: Refactoring and Separation of Concerns
- [ ] **Refine Default Dependencies:** Provide a cleaner way to designate or infer MIMO algebraic-loop dependencies to avoid artificial loop exceptions.
- [ ] **Automated Output Port Dependency Inference:** Extend `DiagramSystem` to automatically detect exact dependencies on external inputs when exposing a subsystem's output via `connect_new_output_port(..., dependencies="auto")`. Involves topological trace through `self.connections`.
- [ ] **Minor Cleanup:** Fix typos (`gloabl`, `Comuting`), remove French strings in `jax_utils.py`, rename `graphe` module, remove duplicate `DummySystem` definitions, replace `tempfile.mktemp` with `tempfile.NamedTemporaryFile`, gitignore/remove stray PDFs at repo root.

---

### Phase 6: Pyro 2.0 — Controller Framework
- [ ] **Controller Base Classes:** Implement `StaticController(StaticSystem)` and `DynamicController(DynamicSystem)` with the `c(y, r, t)` / `c(z, y, r, t)` + `b(z, y, r, t)` protocol. Controllers are systems with specific port conventions (`"y"`, `"ref"` inputs; `"u"` output).
- [ ] **ClosedLoopSystem via DiagramSystem:** Implement `controller + plant` as a helper that builds a `DiagramSystem` internally (wiring controller output to plant input, plant output to controller feedback). Unlike Pyro's hardcoded `ClosedLoopSystem`, this is a composition pattern, not a special class.
- [ ] **Linear Controllers:** Port `ProportionalController`, `PIDController` from Pyro.
- [ ] **Nonlinear Controllers:** Port `ComputedTorqueController`, `SlidingModeController` from Pyro (require `MechanicalSystem` — see Phase 8).
- [ ] **LQR:** Port `TrajectoryLQRController` (requires linearization — see Phase 9).
- [ ] **Robot Controllers:** Port `JointPD`, `EndEffectorPD`, `JointPID`, `EndEffectorPID`, `EndEffectorKinematicController` (require `Manipulator` — see Phase 8).
- [ ] **RL Controller Wrapper:** Port `stable_baseline3_controller` (wraps SB3 `.predict()` into a `StaticController`).

---

### Phase 7: Pyro 2.0 — Cost Functions and Analysis
- [ ] **Cost Function Abstraction:** Implement `CostFunction` base class with `g(x, u, t)` (running cost) and `h_cost(x)` (terminal cost). Add `QuadraticCostFunction` with Q, R matrices and `from_sys` factory.
- [ ] **Trajectory Cost Evaluation:** `cost_function.trajectory_evaluation(traj)` returning accumulated J.
- [ ] **Phase Plots:** Implement `PhasePlot` / `PhasePlot3` as standalone functions that take a `System` and plot vector fields. Not methods on `System`.
- [ ] **Bode/PZ Analysis:** Implement `bode_plot(sys)` and `pz_map(sys)` as standalone functions for linearized or `StateSpaceSystem` objects.
- [ ] **Extended Trajectory Plots:** Add `plot='x'`, `plot='u'`, `plot='xu'`, `plot='z'` (controller state) variants, and trajectory comparisons (open-loop vs. closed-loop overlay).

---

### Phase 8: Pyro 2.0 — Mechanical System Templates
- [ ] **`MechanicalSystem` Base Block:** Implement `MechanicalSystem(DynamicSystem)` with the `H(q)`, `C(q,dq)`, `B(q)`, `g(q)`, `d(q,dq)` → `f(x,u,t)` template. User overrides matrix/vector methods, not `f` directly.
- [ ] **`MechanicalSystemWithPositionInputs`:** Variant where `B` and `d` also depend on `u`.
- [ ] **`GeneralizedMechanicalSystem`:** With `M`, `C`, `N`, `B`, `g`, `d` and `dq = N(q) * v` for nonholonomic systems.
- [ ] **`RigidBody2D`:** Planar rigid body on `GeneralizedMechanicalSystem`.
- [ ] **`Manipulator`:** Serial arm base with forward kinematics, Jacobian, end-effector methods, manipulability.
- [ ] **Concrete Plants — Pendulums:** Port `SinglePendulum`, `DoublePendulum`, `Acrobot`, `InvertedPendulum`.
- [ ] **Concrete Plants — Cart-Pole:** Port `CartPole`, `RotatingCartPole`.
- [ ] **Concrete Plants — Manipulators:** Port `OneLinkManipulator`, `TwoLinkManipulator`, `FiveLinkPlanarManipulator`.
- [ ] **Concrete Plants — Vehicles:** Port `KinematicBicycleModel`, `HolonomicMobileRobot`, `DynamicBicycle` (with tire models).
- [ ] **Concrete Plants — Aerospace:** Port `Drone2D`, `Rocket`, `Plane2D`.
- [ ] **Concrete Plants — Marine:** Port `Boat2D`.
- [ ] **Concrete Plants — Linear:** Port `SingleMass`, `TwoMass`, `ThreeMass` (as `StateSpaceSystem`).
- [ ] **Concrete Plants — Canonical:** Port `VanderPol`, `SimpleIntegrator`, `DoubleIntegrator`, `TripleIntegrator`, `MountainCar`.

---

### Phase 9: Pyro 2.0 — State-Space and Linearization
- [ ] **`StateSpaceSystem` Block:** Implement `StateSpaceSystem(DynamicSystem)` with `A`, `B`, `C`, `D` matrices and `f = Ax + Bu`, `h = Cx + Du`.
- [ ] **Numeric Linearization:** Implement `linearize(sys, xbar, ubar)` → `StateSpaceSystem` using finite-difference Jacobians.
- [ ] **Transfer Function:** Implement `TransferFunction` block with `tf2ss` conversion, Bode plot, pole-zero map.
- [ ] **State Observer:** Implement `StateObserver(StateSpaceSystem)` with Luenberger and Kalman gain computation.
- [ ] **Observed System Composition:** Implement `ObservedSystem` that stacks plant + observer states (as a `DiagramSystem` internally).
- [ ] **Eigenmode Analysis:** Implement `compute_eigen_modes`, `animate_eigen_mode` for `StateSpaceSystem`.

---

### Phase 10: Pyro 2.0 — Planning
- [ ] **`Planner` Base Class:** Abstract base with `compute_solution()`, `show_solution()`, `animate_solution()`. Holds `sys`, `cost_function`, `x_start`, `x_goal`.
- [ ] **`OpenLoopController`:** A `StaticController` that replays a `Trajectory` as feedforward: `c(y, r, t) = traj.t2u(t)`.
- [ ] **RRT:** Port `RRT(Planner)` with random tree expansion, nearest-neighbor, path extraction, trajectory smoothing.
- [ ] **Direct Collocation:** Port `DirectCollocationTrajectoryOptimisation(Planner)` using `scipy.optimize.minimize`. Decision variables = stacked `x,u` at collocation points.
- [ ] **Dynamic Programming:** Port `DynamicProgramming` with value iteration, `GridDynamicSystem` discretizer, `LookUpTableController`, policy evaluation.
- [ ] **Polynomial Trajectory Generation:** Port `SingleAxisPolynomialTrajectoryGenerator` and multi-point variant for differential-flatness-style planning.
- [ ] **Trajectory Filter:** Port `TrajectoryFilter` (Butterworth `filtfilt` smoothing).

---

### Phase 11: Pyro 2.0 — Tools and Bridges
- [ ] **Gymnasium Bridge:** Implement `sys2gym(sys, dt, tf, cost_function)` → `gymnasium.Env`. Euler-step dynamics, reward from cost, optional render via Animator.
- [ ] **Pygame Interactive Mode:** Implement `sys2game(sys, dt, ctl, renderer)` for real-time keyboard/joystick control with Euler integration. *(Partially done: `game()` method exists on `System`.)*
- [ ] **Stochastic Wrapper:** Implement `StochasticSystemWrapper` that adds process/measurement noise to any system's `f`/`h`.

---

### Phase 12: Visualization Robustness
- [ ] **Harden Matplotlib Backend Selection:** Prefer non-interactive backends by default with clearer configuration hooks, degrade gracefully in headless environments.
- [ ] **Kinematic Drawing Utilities:** Port `geometry.py` / `drawing.py` helpers (2D transforms, arrow drawing) from Pyro's `kinematic/` module.

---

### Phase 13: JAX Compilation & Future-Proofing
- [ ] **Refactored Execution Plans:** Modify `f_fast` to avoid in-place numpy array mutations. Use Python lists then `concatenate`. Enables `jax.jit` to trace natively.
- [ ] **Duck-Typing by Default:** Ensure all library blocks use array constructors (`np.array([...])`) instead of in-place mutation (`dx[0] = ...`).
- [ ] **Graceful `f_jax` Fallback:** Dual-method architecture where `System` can optionally override `f_jax(x, u, t)` for JAX-incompatible logic.
- [ ] **JAX-Based Direct Collocation:** Leverage autodiff for constraint Jacobians in trajectory optimization (replaces scipy finite-diff).
- [ ] **JAX-Based LQR:** Differentiate through Riccati solutions for trajectory-LQR with learned dynamics.

---

### Phase 14: Documentation and Examples
- [ ] **Comprehensive README:** Installation, quickstart, API overview, comparison with Pyro.
- [ ] **Example Notebooks:** One per major feature (diagram assembly, pendulum simulation, closed-loop control, planning, JAX optimization).
- [ ] **Pyro Migration Guide:** User-facing document showing Pyro patterns and their minilink equivalents.
- [ ] **API Reference:** Auto-generated from docstrings (e.g., Sphinx/mkdocs).
