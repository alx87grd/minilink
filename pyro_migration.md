# Pyro → Minilink Migration Plan

This document maps every Pyro module, class, and feature to its minilink equivalent, specifying exactly how each should be integrated into the minilink port-based architecture. The goal is full Pyro feature parity while leveraging minilink's more flexible foundations.

---

## Table of Contents

1. [Guiding Principles](#1-guiding-principles)
2. [Core Dynamics (`pyro.dynamic`)](#2-core-dynamics)
3. [Controllers (`pyro.control`)](#3-controllers)
4. [Analysis (`pyro.analysis`)](#4-analysis)
5. [Planning (`pyro.planning`)](#5-planning)
6. [Kinematics (`pyro.kinematic`)](#6-kinematics)
7. [Tools (`pyro.tools`)](#7-tools)
8. [Concrete Plant Library](#8-concrete-plant-library)
9. [Migration Order and Dependencies](#9-migration-order-and-dependencies)

---

## 1. Guiding Principles

### What changes from Pyro to Minilink

| Aspect | Pyro | Minilink |
|--------|------|----------|
| Signal model | Single flat `u` (m×1) and `y` (p×1) | Named `InputPort`/`OutputPort` dictionaries |
| Composition | `controller + plant` → hardcoded `ClosedLoopSystem` | `DiagramSystem` wires N blocks in any topology |
| Controller interface | `c(y, r, t)` standalone class | `StaticSystem` with ports `"y"`, `"ref"` → `"u"` |
| Closed loop | Special class with algebraic-loop hack (`ubar` in `h`) | DiagramSystem with proper topological ordering |
| Analysis on System | Methods on `ContinuousDynamicSystem` (660 lines) | Standalone functions that accept a `System` |
| Animation | `forward_kinematic_lines(q)` returns polylines | `get_kinematic_geometry()` + `get_kinematic_transforms(x,u,t)` protocol |
| Cost functions | `sys.cost_function` attribute | Separate `CostFunction` object, optionally attached |
| Execution | Pure Python loops | Compiled execution plans (1.0), optional JAX (MVP prototype) |

### What stays the same

- `f(x, u, t) → dx` is the universal dynamics contract.
- `h(x, u, t) → y` is the output equation.
- Subclasses override physics methods, not simulation/compilation.
- SciPy `solve_ivp` as default integrator.
- matplotlib as default plotting/animation backend.

### Current minilink maturity

All migration work should build on the **1.0 stable** layer and treat prototypes as subject to change:

- **1.0 stable:** `core/framework.py` (ports, signals, `System`, `StaticSystem`, `DynamicSystem`), `core/diagram.py` (`DiagramSystem`, compilation, `f_fast`), `core/analysis.py` (`Trajectory`, `Simulator`), `blocks/` (Integrator, PropController, Source, Step, WhiteNoise, Pendulum, PendulumPDController).
- **MVP prototype:** `compile/` (IR, `CompiledNumpyDiagram`), `graphical/` (all renderers, animation, plotting, graphviz), JAX paths (`compile_jax`, `f_fast_jax`, `jax_utils.py`). These are functional but their APIs may evolve.
- **Planned (empty stubs):** `control/`, `planning/`, `analysis/`.

---

## 2. Core Dynamics

### 2.1 `ContinuousDynamicSystem` → `DynamicSystem`

Pyro's `ContinuousDynamicSystem` is minilink's `DynamicSystem`. The mapping:

| Pyro attribute/method | Minilink equivalent | Notes |
|----------------------|---------------------|-------|
| `n, m, p` | `n, m, p` | Same |
| `state_label, state_units` | `self.state.labels`, `self.state.units` | Via `VectorSignal` |
| `input_label, input_units` | `self.inputs["u"].labels`, `self.inputs["u"].units` | Via `InputPort` |
| `output_label, output_units` | `self.outputs["y"].labels`, `self.outputs["y"].units` | Via `OutputPort` |
| `x_ub, x_lb` | `self.state.upper_bound`, `self.state.lower_bound` | Same concept, on `VectorSignal` |
| `u_ub, u_lb` | `self.inputs["u"].upper_bound`, `self.inputs["u"].lower_bound` | On `InputPort` |
| `xbar, ubar` | `self.state.nominal_value`, `self.inputs["u"].nominal_value` | On `VectorSignal` |
| `x0` | `self.x0` | Same |
| `f(x, u, t)` | `f(x, u, t, params=None)` | Added `params` |
| `h(x, u, t)` | `h(x, u, t, params=None)` | Added `params` |
| `t2u(t)` | **Not a method on System.** Use a `Source` block connected to input ports in a diagram. | More composable: any signal source can drive any input |
| `xut2q(x, u, t)` | **Not needed.** The `get_kinematic_transforms(x, u, t)` protocol replaces this. | Minilink uses transform matrices, not a separate `q` vector |
| `fsim(x, t)` | `fsim(t, x)` | Same, but note arg order matches scipy convention |
| `x_next(x, u, t, dt)` | Not yet implemented. Add as utility: `x + f(x,u,t)*dt` | Simple Euler step helper |
| `cost_function` | Separate `CostFunction` object, optionally attached as `sys.cost_function` | Decouple but preserve convenience |
| `isavalidstate(x)` | Add as method checking `lower_bound <= x <= upper_bound` | On `System` or standalone |
| `isavalidinput(x, u)` | Same pattern | On `System` or standalone |
| `domain` | `self.state.lower_bound` / `upper_bound` | Already exists |

**Convenience methods that stay on System** (using lazy imports to avoid coupling):

| Pyro method | Minilink approach |
|-------------|-------------------|
| `compute_trajectory(tf, n, solver)` | `simulate(sys, tf=10, solver="scipy")` or keep as thin wrapper |
| `plot_trajectory(plot)` | `plot_trajectory(sys, traj, plot="x")` |
| `plot_phase_plane(x_axis, y_axis)` | `plot_phase_plane(sys, x_axis=0, y_axis=1)` |
| `animate_simulation(...)` | `animate(sys, traj, ...)` |
| `show(q)` / `show3(q)` | `render(sys, x, u, t)` (exists, keep as thin wrapper) |
| `plot_linearized_bode(...)` | `bode_plot(linearize(sys, xbar, ubar))` |
| `plot_linearized_pz_map(...)` | `pz_map(linearize(sys, xbar, ubar))` |
| `animate_linearized_mode(i)` | `animate_eigenmode(ss_sys, i)` |
| `convert_to_gymnasium(...)` | `sys2gym(sys, ...)` |
| `convert_to_pygame(...)` | `sys2game(sys, ...)` or `sys.game(...)` (exists) |

### 2.2 `StateSpaceSystem` → `StateSpaceSystem(DynamicSystem)`

**Location:** `minilink/blocks/statespace.py`

```python
class StateSpaceSystem(DynamicSystem):
    def __init__(self, A, B, C, D):
        n = A.shape[0]
        m = B.shape[1]
        p = C.shape[0]
        super().__init__(n, m, p)
        self.A, self.B, self.C, self.D = A, B, C, D

    def f(self, x, u, t=0, params=None):
        return self.A @ x + self.B @ u

    def h(self, x, u, t=0, params=None):
        return self.C @ x + self.D @ u
```

**Key methods to port from Pyro:**
- `compute_eigen_modes()` — eigenvalues/eigenvectors of A
- `animate_eigen_mode(i)` — animate one mode

**Standalone functions:**
- `linearize(sys, xbar, ubar)` → `StateSpaceSystem` via finite-difference Jacobians
- `ss2tf(ss_sys, u_index, y_index)` → `TransferFunction`

### 2.3 `TransferFunction`

**Location:** `minilink/blocks/transferfunction.py`

Port as a `DynamicSystem` that converts numerator/denominator polynomials to controllable canonical form (`tf2ss`). Standalone analysis functions: `bode_plot(tf_sys)`, `pz_map(tf_sys)`.

### 2.4 `StateObserver` / `ObservedSystem`

**Minilink approach:** An observer is a `DynamicSystem` with inputs `"y"` (measurement) and `"u"` (known input), and output `"x_hat"` (state estimate). Composing observer + plant is a `DiagramSystem` that wires plant output to observer measurement input. This replaces Pyro's special-purpose `ObservedSystem` class with general-purpose diagram composition.

```
┌──────────┐  y   ┌──────────┐
│  Plant   ├─────►│ Observer │
│          │      │          │
│          │◄──┐  │     x_hat├──► (state estimate)
└──────────┘   │  └──────────┘
               │       ▲
               │  u    │
               └───────┘ (shared known input)
```

### 2.5 `MechanicalSystem` Template

**Location:** `minilink/blocks/mechanical.py`

This is a high-value template. Users subclass and override `H(q)`, `C(q,dq)`, `B(q)`, `g(q)`, `d(q,dq)` instead of writing `f` directly.

```python
class MechanicalSystem(DynamicSystem):
    """
    Standard rigid-body mechanical system:
        H(q) ddq + C(q,dq) dq + d(q,dq) + g(q) = B(q) u

    State: x = [q, dq]  (2*dof states)
    """
    def __init__(self, dof, m):
        n = 2 * dof
        super().__init__(n, m, p=dof)
        self.dof = dof

    def H(self, q):              # Inertia matrix (dof × dof)
    def C(self, q, dq):          # Coriolis matrix (dof × dof)
    def B(self, q):              # Input mapping (dof × m)
    def g(self, q):              # Gravity vector (dof,)
    def d(self, q, dq):          # Damping/friction (dof,)

    def ddq(self, q, dq, u, t=0):
        return np.linalg.solve(self.H(q), self.B(q) @ u - self.C(q,dq) @ dq - self.g(q) - self.d(q,dq))

    def f(self, x, u, t=0, params=None):
        q, dq = x[:self.dof], x[self.dof:]
        return np.concatenate([dq, self.ddq(q, dq, u, t)])

    def kinetic_energy(self, q, dq):
        return 0.5 * dq @ self.H(q) @ dq
```

**Variants to implement:**
- `MechanicalSystemWithPositionInputs` — `B` and `d` also depend on `u`
- `GeneralizedMechanicalSystem` — adds `N(q)` such that `dq = N(q) * v` (nonholonomic)
- `RigidBody2D(GeneralizedMechanicalSystem)` — planar rigid body with default 2D geometry

### 2.6 `Manipulator` Template

**Location:** `minilink/blocks/manipulator.py`

Extends `MechanicalSystem` with:
- `forward_kinematics(q)` → end-effector position
- `jacobian(q)` → end-effector Jacobian
- `end_effector_velocity(q, dq)` → `J(q) @ dq`
- `manipulability(q)` → `sqrt(det(J J^T))`
- Graphical: link geometry from DH or explicit polylines

Concrete subclasses: `OneLinkManipulator`, `TwoLinkManipulator`, `FiveLinkPlanarManipulator`, `ThreeLinkManipulator3D`.

---

## 3. Controllers

### 3.1 Design: Controllers as Systems

In Pyro, `StaticController` is a standalone class with `c(y, r, t)`. In minilink, controllers are regular `StaticSystem` or `DynamicSystem` instances with specific port conventions:

```python
class StaticController(StaticSystem):
    """
    Memoryless controller: u = c(y, r, t)

    Ports:
        Inputs:  "y" (p-dim sensor), "ref" (k-dim reference)
        Outputs: "u" (m-dim control)
    """
    def __init__(self, k, m, p):
        super().__init__(m=k+p, p=m)
        self.k, self.ctrl_m, self.ctrl_p = k, m, p
        self.inputs = {}
        self.add_input_port(p, "y")
        self.add_input_port(k, "ref")
        self.outputs = {}
        self.add_output_port(m, "u", function=self._compute_u, dependencies=("y", "ref"))

    def c(self, y, r, t=0):
        raise NotImplementedError

    def _compute_u(self, x, u, t=0, params=None):
        y = self.u2input_signal(u, "y")
        r = self.u2input_signal(u, "ref")
        return self.c(y, r, t)
```

This means controllers are wirable like any other block in a `DiagramSystem`. The Pyro `__add__` operator becomes:

```python
def __add__(self, plant):
    """Build closed-loop: controller + plant → DiagramSystem"""
    diagram = DiagramSystem()
    diagram.add_subsystem(plant, "plant")
    diagram.add_subsystem(self, "ctl")
    diagram.connect("plant", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")
    # Diagram external input is the reference
    diagram.add_input_port(self.k, "ref")
    diagram.connect("input", "ref", "ctl", "ref")
    # Diagram output is the plant output
    diagram.connect_new_output_port("plant", "y", "y")
    diagram.compile()
    return diagram
```

**Advantage over Pyro:** No special `ClosedLoopSystem` class needed. The algebraic-loop detection handles feedthrough correctly based on port dependencies. No `ubar` hack.

### 3.2 `DynamicController`

```python
class DynamicController(DynamicSystem):
    """
    Controller with internal state z: u = c(z, y, r, t), dz = b(z, y, r, t)

    Ports:
        Inputs:  "y" (sensor), "ref" (reference)
        Outputs: "u" (control)
    State:   z (controller internal state)
    """
    def c(self, z, y, r, t=0):
        raise NotImplementedError

    def b(self, z, y, r, t=0):
        raise NotImplementedError

    def f(self, x, u, t=0, params=None):
        y = self.u2input_signal(u, "y")
        r = self.u2input_signal(u, "ref")
        return self.b(x, y, r, t)

    def h(self, x, u, t=0, params=None):
        y = self.u2input_signal(u, "y")
        r = self.u2input_signal(u, "ref")
        return self.c(x, y, r, t)
```

### 3.3 Specific Controllers to Port

| Pyro class | Minilink location | Minilink base | Key notes |
|------------|-------------------|---------------|-----------|
| `ProportionalController` | `control/linear.py` | `StaticController` | `c(y, r, t) = K @ (r - y)` |
| `PIDController` | `control/linear.py` | `DynamicController` | Internal state = integral of error; `b` = error, `c` = Kp*e + Ki*z + Kd*de |
| `ComputedTorqueController` | `control/nonlinear.py` | `StaticController` | Requires access to plant's `H, C, g` — pass plant reference or matrices as params |
| `SlidingModeController` | `control/nonlinear.py` | `StaticController` | Sliding surface + switching law |
| `TrajectoryLQRController` | `control/lqr.py` | `StaticController` | Time-varying K(t) from backward Riccati; requires `linearize`, `CostFunction` |
| `JointPD` | `control/robot.py` | `StaticController` | `c = Kp*(r-q) + Kd*(0-dq)` |
| `EndEffectorPD` | `control/robot.py` | `StaticController` | Uses Jacobian transpose: `u = J^T(q) (Kp*e_pos + Kd*e_vel)` |
| `JointPID` / `EndEffectorPID` | `control/robot.py` | `DynamicController` | PID variants of above |
| `EndEffectorKinematicController` | `control/robot.py` | `StaticController` | Resolved-rate: `u = J^{-1}(q) * v_desired` |
| `stable_baseline3_controller` | `control/rl.py` | `StaticController` | `c(y, r, t) = sb3_model.predict(y)` |

---

## 4. Analysis

### 4.1 `Trajectory`

Minilink already has `Trajectory`. Enhancements needed:

| Pyro feature | Implementation |
|-------------|----------------|
| `t2u(t)` | Interpolate `u` at arbitrary time `t` (e.g., `np.interp` or `scipy.interpolate.interp1d`) |
| `t2x(t)` | Same for states |
| `re_sample(n)` | Resample trajectory to `n` uniform time points |
| `save(filename)` / `load(filename)` | Pickle or npz serialization |
| `dx`, `y`, `r`, `J`, `dJ` | Computed lazily from `x`, `u`, `t` and a reference `sys` |

### 4.2 `Simulator`

Already exists in minilink. Differences to address:

| Pyro | Minilink |
|------|----------|
| `CLosedLoopSimulator` records true `u = c(y,r,t)` | Not needed: `DiagramSystem` + `compute_internal_signals` reconstructs all signals |
| `DynamicCLosedLoopSimulator` handles augmented state | Not needed: `DynamicController` is just another subsystem in the diagram |
| `solver='odeint'` option | Consider adding as alternative to `solve_ivp` |

### 4.3 `CostFunction`

**Location:** `minilink/analysis/costfunction.py`

```python
class CostFunction:
    def g(self, x, u, t):
        """Running cost (integrand)."""
        return 0.0

    def h(self, x):
        """Terminal cost."""
        return 0.0

    def trajectory_evaluation(self, traj):
        """Compute total cost J = integral(g) + h(x_final)."""
        ...

class QuadraticCostFunction(CostFunction):
    def __init__(self, Q, R, xbar=None, ubar=None):
        ...

    def g(self, x, u, t):
        e_x = x - self.xbar
        e_u = u - self.ubar
        return e_x @ self.Q @ e_x + e_u @ self.R @ e_u

    @classmethod
    def from_sys(cls, sys):
        return cls(Q=np.eye(sys.n), R=np.eye(sys.m),
                   xbar=sys.state.nominal_value,
                   ubar=sys.inputs["u"].nominal_value)
```

Pyro variants to port: `TimeCostFunction`, `QuadraticCostFunctionWithDomainCheck`, `Reachability`.

### 4.4 Phase Plots

**Location:** `minilink/analysis/phaseanalysis.py`

Standalone functions (not methods on System):

```python
def plot_phase_plane(sys, x_axis=0, y_axis=1, n_grid=20, ...):
    """2D phase portrait (vector field) for an autonomous system."""

def plot_phase_plane_3d(sys, x_axis=0, y_axis=1, z_axis=2, ...):
    """3D phase portrait."""

def plot_phase_plane_trajectory(sys, traj, x_axis=0, y_axis=1, ...):
    """Phase portrait with trajectory overlay."""

def plot_phase_plane_closed_loop(plant, controller, x_axis=0, y_axis=1, ...):
    """Open-loop vs closed-loop vector field comparison."""
```

### 4.5 `TrajectoryPlotter` / `Animator`

Minilink already has `Animator` and `plot_trajectory` (both MVP prototypes — functional but expect API evolution). Enhancements:

- `TrajectoryPlotter` class that supports `plot='x'`, `plot='u'`, `plot='xu'`, `plot='z'` modes
- Internal signal plotting via `traj.internal_signals` (already supported by `compute_internal_signals`)
- End-effector trajectory plot for manipulators

---

## 5. Planning

### 5.1 Design Principle

In Pyro, planners hold a reference to the plant and its cost function, then produce a `Trajectory` solution. In minilink, the same pattern works, but planners should:
- Accept any `System` (not just `ContinuousDynamicSystem`)
- Use the `CostFunction` abstraction
- Return trajectories with `t2u` interpolation for feedforward replay

### 5.2 `Planner` Base

**Location:** `minilink/planning/planner.py`

```python
class Planner:
    def __init__(self, sys, cost_function=None, x_start=None, x_goal=None):
        self.sys = sys
        self.cost_function = cost_function or sys.cost_function
        self.x_start = x_start or sys.x0
        self.x_goal = x_goal

    def compute_solution(self):
        raise NotImplementedError

    def show_solution(self, plot='xu'):
        ...

    def animate_solution(self, **kwargs):
        ...
```

### 5.3 `OpenLoopController`

A `StaticController` that replays a trajectory:

```python
class OpenLoopController(StaticController):
    def __init__(self, trajectory):
        self.trajectory = trajectory
        super().__init__(k=0, m=trajectory.m, p=0)
        # No "y" or "ref" inputs needed — this is a source-like controller

    def c(self, y, r, t=0):
        return self.trajectory.t2u(t)
```

Alternatively, since minilink has `Source` blocks, an `OpenLoopController` could simply be a `Source` whose `get_signal(t)` interpolates the trajectory.

### 5.4 `RRT`

**Location:** `minilink/planning/rrt.py`

Port `RRT(Planner)` largely unchanged. Key API:
- `compute_solution()` → builds tree, finds path, converts to `Trajectory`
- `plot_tree()` / `plot_tree_3d()` for visualization
- Uses `sys.x_next(x, u, t, dt)` for forward simulation of nodes

Minilink prerequisite: `x_next` helper on `System` or as a standalone function.

### 5.5 Direct Collocation

**Location:** `minilink/planning/collocation.py`

Port `DirectCollocationTrajectoryOptimisation(Planner)`. Decision variables = stacked `[x_0, ..., x_N, u_0, ..., u_N]`. Uses `scipy.optimize.minimize` with dynamics as equality constraints.

Future enhancement (depends on JAX MVP prototype maturing): when JAX backend is ready, use `jax.jacfwd` for analytic constraint Jacobians (massive speedup over finite-diff).

### 5.6 Dynamic Programming

**Location:** `minilink/planning/dp.py`

Port the DP toolchain:
1. `GridDynamicSystem` — discretizes state/input space into grids, precomputes `x_next` lookup tables
2. `DynamicProgramming(Planner)` — backward value iteration over the grid
3. `LookUpTableController(StaticController)` — interpolates the optimal policy grid into a real-time controller
4. `PolicyEvaluator` — forward rollout of a fixed policy to evaluate cost

### 5.7 Polynomial Trajectory Generation

**Location:** `minilink/planning/polynomial.py`

Port `SingleAxisPolynomialTrajectoryGenerator` and `MultiPointSingleAxisPolynomialTrajectoryGenerator`. These solve for polynomial coefficients that match boundary conditions (position, velocity, acceleration) and minimize snap/jerk — useful for differential-flatness-based planning.

### 5.8 Trajectory Filter

**Location:** `minilink/planning/filters.py`

Port `TrajectoryFilter` — applies Butterworth low-pass `filtfilt` to smooth noisy trajectories (e.g., from RRT).

---

## 6. Kinematics

### 6.1 `geometry.py` / `drawing.py`

**Location:** `minilink/graphical/primitives.py` (extend existing) or `minilink/kinematic/`

Pyro's kinematic module is small — just helper functions:
- `transformation_matrix_2D(theta, x, y)` → 3×3 homogeneous transform
- `transform_points_2D(T, pts)` → apply transform to point cloud
- `arrow_from_length_angle(...)` → draw arrow primitives
- `arrow_from_components(...)` → draw arrow from vector components

Minilink already has `translation_matrix` in `primitives.py` (MVP prototype). Extend with rotation and full homogeneous 2D/3D transforms. Used by `RigidBody2D`, `Manipulator`, vehicle models for animation.

### 6.2 Animation Protocol: Lines vs. Transform Matrices

Pyro uses `forward_kinematic_lines(q)` which returns raw polyline point arrays `(pts, style, color)`. Minilink uses a cleaner protocol:

1. `get_kinematic_geometry()` → list of `GraphicPrimitive` objects (defined once)
2. `get_kinematic_transforms(x, u, t)` → list of 4×4 transform matrices (per frame)
3. `get_dynamic_geometry(x, u, t)` → additional primitives that change structure per frame

When porting Pyro plants, convert each `forward_kinematic_lines` implementation to this two-step protocol:
- **Links/bodies** → `Circle`, `Rectangle`, `CustomLine` primitives with transforms
- **Forces/arrows** → `get_dynamic_geometry` returning arrow primitives
- **Overlay graphics** (`forward_kinematic_lines_plus`) → `get_dynamic_geometry`

---

## 7. Tools

### 7.1 `Sys2Gym` → `sys2gym`

**Location:** `minilink/tools/sys2gym.py`

```python
class MiniLinkGymEnv(gymnasium.Env):
    def __init__(self, sys, dt=0.05, tf=10.0, cost_function=None, render_mode=None):
        self.sys = sys
        self.dt = dt
        self.cost_function = cost_function or sys.cost_function
        # observation_space from sys.state bounds
        # action_space from sys input bounds

    def step(self, action):
        # Euler: x_next = x + f(x, u, t) * dt
        # reward = -cost_function.g(x, u, t)
        ...

    def reset(self, seed=None):
        ...

    def render(self):
        # Use Animator for optional rendering
        ...
```

**Convenience:** `sys.to_gymnasium(dt, tf)` as a thin wrapper (or standalone function).

### 7.2 `InteractiveContinuousDynamicSystem` → `sys2game` / `sys.game()`

Minilink already has `sys.game()` using `Animator.game()` with pygame input (MVP prototype). Enhancements needed from Pyro:
- Joystick support (Pyro's `process_events` handles joystick axes)
- Score tracking / reset
- Configurable key-to-input mapping
- Optional controller in the loop (`ctl` parameter)

---

## 8. Concrete Plant Library

All concrete plants go in `minilink/blocks/` organized by category. Each port the physics (`f` or `H,C,B,g,d`) and the animation protocol (`get_kinematic_geometry` + `get_kinematic_transforms`).

### 8.1 Priority Order for Porting

Port in this order based on teaching/demo value and dependency requirements:

**Tier 1 — Already done or trivial:**
| Plant | Base class | Notes |
|-------|-----------|-------|
| `Integrator` | `DynamicSystem` | Already in minilink |
| `Pendulum` (single) | `MechanicalSystem` | Already in minilink (basic version) |

**Tier 2 — Core teaching examples (port first):**
| Plant | Base class | Pyro source |
|-------|-----------|-------------|
| `SinglePendulum` | `MechanicalSystem` | `pendulum.py` |
| `DoublePendulum` | `MechanicalSystem` | `pendulum.py` |
| `CartPole` | `MechanicalSystem` | `cartpole.py` |
| `SimpleIntegrator` | `DynamicSystem` | `integrator.py` |
| `DoubleIntegrator` | `DynamicSystem` | `integrator.py` |
| `VanderPol` | `DynamicSystem` | `equation.py` |

**Tier 3 — Robotics core:**
| Plant | Base class | Pyro source |
|-------|-----------|-------------|
| `OneLinkManipulator` | `Manipulator` | `manipulator.py` |
| `TwoLinkManipulator` | `Manipulator` | `manipulator.py` |
| `FiveLinkPlanarManipulator` | `Manipulator` | `manipulator.py` |

**Tier 4 — Vehicles and aerospace:**
| Plant | Base class | Pyro source |
|-------|-----------|-------------|
| `KinematicBicycleModel` | `DynamicSystem` | `vehicle_steering.py` |
| `HolonomicMobileRobot` | `DynamicSystem` | `vehicle_steering.py` |
| `Drone2D` | `MechanicalSystem` | `drone.py` |
| `Rocket` | `MechanicalSystemWithPositionInputs` | `rocket.py` |
| `DynamicBicycle` | `RigidBody2D` | `vehicle_dynamic.py` |

**Tier 5 — Extended library:**
| Plant | Base class | Pyro source |
|-------|-----------|-------------|
| `Acrobot` | `MechanicalSystem` | `pendulum.py` |
| `InvertedPendulum` | `MechanicalSystem` | `pendulum.py` |
| `Boat2D` | `RigidBody2D` | `boat.py` |
| `Plane2D` | `MechanicalSystemWithPositionInputs` | `plane.py` |
| `MountainCar` | `MechanicalSystem` | `mountaincar.py` |
| `SingleMass` / `TwoMass` | `StateSpaceSystem` | `massspringdamper.py` |
| `TransferFunction` | `DynamicSystem` | `tranferfunction.py` |
| `QuarterCarOnRoughTerrain` | `DynamicSystem` | `suspension.py` |

---

## 9. Migration Order and Dependencies

The dependency graph dictates implementation order. Here's the critical path:

```
Phase 2-3: Bug fixes + decouple framework.py
    │
    ▼
Phase 4: API UX (operator overloading, trajectory interpolation)
    │
    ├──────────────────────────────┐
    ▼                              ▼
Phase 6: Controllers           Phase 7: Cost Functions
    │                              │
    ├──────────────────────────────┘
    ▼
Phase 8: MechanicalSystem + Tier 2 plants
    │
    ├──────────────────────────────┐
    ▼                              ▼
Phase 9: StateSpace/Linearize  Phase 10: Planning
    │                              │
    ├──────────────────────────────┘
    ▼
Phase 8 (cont): Tier 3-5 plants + robot controllers
    │
    ▼
Phase 11: Tools (Gym, Game)
    │
    ▼
Phase 13: JAX compilation
```

### Specific Dependency Chains

- **`ComputedTorqueController`** requires `MechanicalSystem` (needs `H`, `C`, `g`)
- **`TrajectoryLQRController`** requires `linearize()` + `CostFunction` + `Trajectory.t2u()`
- **`EndEffectorPD`** requires `Manipulator` (needs `jacobian(q)`)
- **`RRT`** requires `x_next()` helper + `CostFunction`
- **`DirectCollocation`** requires `CostFunction` + `Trajectory`
- **`DynamicProgramming`** requires `GridDynamicSystem` + `CostFunction` + `LookUpTableController`
- **`Sys2Gym`** requires `CostFunction` + `Animator` (for render)
- **JAX Direct Collocation** requires all of Phase 13 + `DirectCollocation`

### Estimated Scope

| Phase | Files to create/modify | Rough effort |
|-------|----------------------|--------------|
| 2-3 (bugs + decouple) | 3-4 files | 1-2 days |
| 4 (UX) | 2-3 files | 1 day |
| 6 (controllers) | 4-5 new files | 2-3 days |
| 7 (cost/analysis) | 3-4 new files | 1-2 days |
| 8 (mechanical + plants) | 8-10 new files | 3-5 days |
| 9 (statespace) | 2-3 new files | 1-2 days |
| 10 (planning) | 5-6 new files | 3-5 days |
| 11 (tools) | 2 files | 1 day |
| 13 (JAX) | 3-4 files | 3-5 days |

**Total estimated: ~4-6 weeks of focused development for full Pyro feature parity.**
