# Pycollimator Analysis — Lessons for Minilink

> Cross-codebase analysis of [pycollimator](https://github.com/kuzja111/pycollimator) patterns,
> features, and design strategies relevant to the ongoing `minilink` refactoring.
>
> **Scope**: Architecture, compilation, simulation, JAX integration, and library design.
> Each section maps a pycollimator feature → actionable insight for minilink.

---

## 1. Architecture Comparison at a Glance

| Dimension | minilink (current) | pycollimator | Gap / Opportunity |
|---|---|---|---|
| **Block base class** | `System` (single class) | `SystemBase` → `LeafSystem` (two-layer) | Consider a thin read-only `SystemBase` for shared API between Leaf and Diagram |
| **Diagram construction** | `DiagramSystem.add_subsystem()` + `connect()` (inline) | `DiagramBuilder` (separate builder) | Builder pattern decouples wiring from execution; adopt or keep aware |
| **Compilation** | `compile/` package (new `dev-compile` branch) | Lazy-eval + `dependency_graph.py` + `DependencyTicket` | minilink's compile-once IR is cleaner; add ticket-style invalidation later |
| **Execution backend** | `NumpyEvaluator`, `JaxEvaluator` | `cnp` switchable backend (`collimator.backend`) | minilink's approach (separate evaluator classes) is more explicit; keep it |
| **Simulation** | SciPy `solve_ivp` (loose coupling) | `Simulator` + `SimulatorOptions` + `ODESolver` | Formalize simulator options; decouple solver from system |
| **Context / State** | `x`, `u`, `t` passed as flat arrays | `Context` (frozen dataclass tree, immutable) | Immutable context enables JAX tracing; consider for advanced features |
| **Block library** | `systems/` (growing) | `library/` (LTI, PID, MPC, MuJoCo, estimators…) | Grow library using pycollimator's organisation model |
| **Parameters** | Attributes on `System` | `Parameter` first-class objects + `@parameters` decorator | Track dynamic vs static params explicitly for JAX |
| **Events** | Not implemented | Periodic updates, zero-crossing events, mode transitions | Long-term roadmap item; design around pycollimator's event model |

---

## 2. Key Design Patterns Worth Adopting

### 2.1 DiagramBuilder Pattern

**Pycollimator**: Uses a dedicated `DiagramBuilder` class that:
- Validates uniqueness of system names on `add()`.
- Guards against double-connection of input ports.
- Prevents re-use of a builder after `build()` is called (`_already_built` flag).
- Exports child ports as diagram-level I/O via `export_input()` / `export_output()`.

```python
# pycollimator style
builder = DiagramBuilder()
plant, controller = builder.add(Plant(), Controller())
builder.connect(controller.output_ports[0], plant.input_ports[0])
builder.export_input(controller.input_ports[0])
diagram = builder.build()
```

**minilink current pattern**:
```python
# minilink style
diagram = DiagramSystem()
diagram.add_subsystem(plant, "plant")
diagram.add_subsystem(controller, "ctl")
diagram.connect("ctl", "u", "plant", "u")
diagram.connect_new_output_port("plant", "y", "y")
```

> [!TIP]
> **Recommendation**: minilink's string-based wiring is concise for education/prototyping.
> **Don't** switch to a full builder class unless complexity demands it.
> **Do** consider adding validation guards (unique names, orphan ports, double-connection
> checks) inside `add_subsystem()` and `connect()`.

---

### 2.2 Immutable Context for Simulation State

**Pycollimator** separates the *model* (systems, ports, connections) from the *runtime data*
(state values, parameters, time) using an immutable `Context` tree:

```python
@dataclasses.dataclass(frozen=True)
class ContextBase:
    owning_system: SystemBase
    time: Scalar = None
    
    def with_time(self, value):
        return dataclasses.replace(self, time=value)
    
    def with_continuous_state(self, value):
        return dataclasses.replace(self, state=self.state.with_continuous_state(value))
```

Key properties:
- **Frozen dataclasses** (`frozen=True`) → no accidental mutation.
- **`with_*()` methods** return new instances → JAX-traceable, pure-functional.
- **JAX PyTree registration** for both `LeafContext` and `DiagramContext` → enables
  `jax.jit`, `jax.grad`, `jax.vmap` over simulation state.

> [!IMPORTANT]
> **Recommendation**: minilink currently passes `(x, u, t)` as flat arrays, which is
> simple and sufficient for NumPy. When JAX support matures, consider an optional
> lightweight `SimState` frozen dataclass to bundle `(x, u, t, params)` — but **keep
> the raw `f(x, u, t)` interface as the primary API** for mathematical clarity.

---

### 2.3 Dependency Ticket System

Pycollimator's `dependency_graph.py` (inspired by Drake) assigns integer *tickets* to
every value a computation might depend on:

| Ticket | Meaning |
|---|---|
| `DependencyTicket.time` | Current time |
| `DependencyTicket.xc` | Continuous state |
| `DependencyTicket.xd` | Discrete state |
| `DependencyTicket.u` | All input ports |
| `DependencyTicket.p` | Parameters |
| `DependencyTicket.xcdot` | State derivatives |

Each `DependencyTracker` knows its prerequisites and subscribers, forming a
directed graph used for:
1. **Algebraic loop detection** — topological sort of port dependencies.
2. **Cache invalidation** — mark downstream computations as stale when a prerequisite
   changes.
3. **Feedthrough analysis** — determine if outputs depend on inputs.

> [!TIP]
> **Recommendation**: minilink's current `port.dependencies` list already captures the
> essential feedthrough information. A full ticket system is **overkill now** but keep
> the concept in mind for when you add:
> - Discrete state updates
> - Cache/sample-and-hold ports
> - Automatic feedthrough inference

---

### 2.4 Backend Dispatcher (`collimator.backend`)

Pycollimator has a **switchable math backend** at `collimator.backend.numpy_api`:

```python
# collimator.backend.numpy_api
_backend = "numpy"  # or "jax"

def set_backend(name):
    global _backend
    _backend = name

def zeros_like(x):
    if _backend == "jax":
        return jnp.zeros_like(x)
    return np.zeros_like(x)
```

All library blocks call `cnp.matmul()`, `cnp.zeros()`, etc. instead of `np.*` directly.

**minilink's approach** (separate `NumpyEvaluator` / `JaxEvaluator` classes) is
**architecturally superior** because:
- No global mutable state → thread-safe, no surprises.
- The backend choice is explicit at compile time, not buried in a global switch.
- Each evaluator can optimise for its backend (e.g., `jnp.concatenate` vs in-place
  `local_u[idx:idx+dim] = ...`).

> [!TIP]
> **Recommendation**: Keep the current evaluator-class approach. It's better than
> a global dispatcher. However, **do** ensure that the block library functions
> (`f`, `h`, port `compute`) remain backend-agnostic — they receive arrays and return
> arrays, letting the evaluator decide which array library is in play.

---

## 3. Simulation Architecture

### 3.1 SimulatorOptions (Structured Configuration)

Pycollimator bundles all solver/simulation knobs into a frozen dataclass:

```python
@dataclass
class SimulatorOptions:
    enable_tracing: bool = True
    max_major_step_length: float = None
    max_major_steps: int = None
    rtol: float = 1e-6
    atol: float = 1e-8
    min_minor_step_size: float = 1e-10
    max_minor_step_size: float = 0.1
    ode_solver_method: str = "auto"
    enable_autodiff: bool = False
    recorded_signals: dict = None
    save_time_series: bool = True
    math_backend: str = "jax"
    buffer_length: int = None
```

> [!IMPORTANT]
> **Recommendation**: Create a `SimulationOptions` dataclass for minilink to replace the
> current ad-hoc parameter passing to `solve_ivp`. This is a near-term win that:
> - Makes simulation configuration explicit and documented.
> - Enables consistent defaults across analysis scripts.
> - Prepares the ground for JAX solver integration.

Suggested minimal version:
```python
@dataclass(frozen=True)
class SimulationOptions:
    rtol: float = 1e-6
    atol: float = 1e-8
    method: str = "RK45"
    max_step: float = 0.01
    backend: str = "numpy"  # "numpy" or "jax"
```

---

### 3.2 Hybrid Simulation (Discrete + Continuous Events)

Pycollimator's `Simulator.advance_to()` implements a full hybrid simulation loop:

1. **Periodic updates**: Discrete state updated at fixed intervals.
2. **Zero-crossing detection**: Guard functions monitored during ODE integration;
   bisection used to localise crossings.
3. **Mode transitions**: State machine semantics (start_mode → end_mode).
4. **Custom autodiff rules**: `jax.custom_vjp` for differentiating through the
   simulation, including correct handling of terminal events.

> [!NOTE]
> **Recommendation**: This is advanced functionality that minilink doesn't need yet.
> However, **design for it** by:
> 1. Keeping `f()` and evaluator methods as pure functions.
> 2. Not baking `solve_ivp` assumptions into the core framework.
> 3. Planning for a `DiscreteUpdateEvent` concept when hybrid systems are needed.

---

## 4. JAX Integration Strategy

### 4.1 What Pycollimator Does

- **All state** stored in frozen dataclasses registered as JAX PyTrees.
- **All array ops** go through the switchable `cnp` backend.
- **`jax.custom_vjp`** on the simulator's `advance_to()` for correct autodiff
  through the ODE integration boundary (including zero-crossing times).
- **`Parameter`** objects wrap JAX arrays with metadata (static vs dynamic, bounds,
  constraints) — essential for `jax.grad`.

### 4.2 minilink's Current JAX Support

The new `JaxEvaluator` on `dev-compile` already handles:
- Functional array updates (`.at[].set()`) for signal buffer.
- `jnp.concatenate` instead of in-place mutation for `_gather_u_jax`.
- `get_jit_compute_dx()` / `get_jit_compute_outputs()` convenience wrappers.

**What's missing vs pycollimator**:
1. **No PyTree-registered state** — flat `(x, u, t)` arrays work but can't be
   `vmap`'d over structured state.
2. **No `custom_vjp`** for simulation — autodiff stops at `solve_ivp`.
3. **No parameter tracking** — dynamic vs static classification.

> [!TIP]
> **Recommendation**: These are Phase 4+ concerns. The current flat-array approach is
> a feature, not a bug, for the "math-first" philosophy. Add PyTree support only when
> a concrete use-case demands it (e.g., differentiable MPC).

---

## 5. Block Library Organisation

### 5.1 Pycollimator's Library Structure

```
collimator/library/
├── __init__.py
├── linear_system.py     # LTISystem, TransferFunction, PID, Derivative, linearize()
├── state_estimators.py  # Kalman filters, observers
├── nmpc.py              # Nonlinear MPC
├── mujoco.py            # Physics engine integration
├── clock.py             # Time sources
├── primitives.py        # Gain, Sum, Integrator, etc.
├── ...
```

Key patterns:
- Each file is self-contained with `__all__` exports.
- Complex blocks (LTI, PID) use inheritance: `LTISystemBase` → `LTISystem` / `LTISystemDiscrete`.
- The `@parameters(dynamic=[...], static=[...])` decorator auto-wires parameter handling.
- The `linearize()` function is a standalone utility that operates on any system +
  context → returns a new `LTISystem`.

### 5.2 Recommendations for minilink

| Priority | Feature | Notes |
|---|---|---|
| **P0** | `StateSpaceSystem(A, B, C, D)` | Direct port of pycollimator's `LTISystem`; uses `dx = A@x + B@u` (math-readable) |
| **P1** | `TransferFunction(num, den)` | Convert via `scipy.signal.tf2ss`, then wrap `StateSpaceSystem` |
| **P1** | `PID(kp, ki, kd, N)` | State-space form from pycollimator; 2nd-order LTI under the hood |
| **P2** | `linearize(diagram, x0, u0)` | Use `jax.jvp` JVP approach from pycollimator (requires JAX evaluator) |
| **P3** | `DiscreteStateSpaceSystem` | Requires discrete events infrastructure |

> [!TIP]
> Follow pycollimator's pattern of declaring feedthrough explicitly:
> ```python
> self.is_feedthrough = not np.allclose(D, 0.0)
> ```
> This directly maps to minilink's `dependencies` field on output ports.

---

## 6. Concrete Action Items for minilink

### Near-Term (dev-compile branch)

- [ ] **Add validation guards** to `DiagramSystem.add_subsystem()` and `.connect()`
      (unique names, port existence, double-connection detection).
- [ ] **Create `SimulationOptions` dataclass** to replace ad-hoc `solve_ivp` kwargs.
- [ ] **Implement `StateSpaceSystem`** block following pycollimator's LTI pattern:
      feedthrough detection via `D`, explicit `dependencies` on output port.
- [ ] **Add `compute_outputs()` to `DiagramSystem`** as a thin wrapper around the
      compiled evaluator, matching pycollimator's `export_output` concept.

### Mid-Term (after merge to main)

- [ ] **Port `linearize()`** utility function using JAX JVP on the `JaxEvaluator`.
- [ ] **Add `Parameter` metadata** — at minimum, `dynamic` vs `static` classification
      so JAX can trace through dynamic params while treating static ones as constants.
- [ ] **Formalise `DiscreteState`** support, drawing on pycollimator's
      `declare_discrete_state()` + `declare_periodic_update()` pattern.

### Long-Term (research roadmap)

- [ ] **Immutable `SimState`** frozen dataclass for JAX-traced simulation.
- [ ] **`jax.custom_vjp`** on the integrator for differentiable simulation.
- [ ] **Zero-crossing events** for mode transitions (hybrid systems).
- [ ] **Full dependency ticket system** for automatic cache invalidation.

---

## 7. Anti-Patterns to Avoid

Several pycollimator patterns should **not** be adopted:

| Pattern | Why to Avoid |
|---|---|
| **Global backend switch** (`cnp.set_backend("jax")`) | Mutable global state → not thread-safe, surprising; minilink's evaluator-class approach is better |
| **Metaclass for `__init__`** (`InitializeParameterResolver`) | Complex metaclass magic for auto-calling `initialize()` after `__init__`; minilink should use explicit `__init__` |
| **Equinox dependency** | Heavy JAX dataclass framework; minilink should stay with stdlib `dataclasses` |
| **PyTree registration for state** | Adds significant complexity; not needed until differentiable simulation is a concrete goal |
| **Lazy evaluation model** | Pycollimator evaluates ports lazily via context → cache system; minilink's compile-once topological evaluation is faster and simpler |

---

## 8. Summary

The core takeaway is that **pycollimator and minilink share the same fundamental architecture**
(block-diagram, port-based, topological evaluation) but diverge in execution strategy:

- **Pycollimator** optimizes for **JAX-traced, differentiable simulation** with immutable
  state trees and lazy evaluation.
- **minilink** optimizes for **mathematical readability and compiled evaluation** with
  flat arrays and explicit execution plans.

Both approaches are valid. minilink's explicit compilation pipeline (`ExecutionPlan` +
`NumpyEvaluator` / `JaxEvaluator`) is arguably a more maintainable architecture than
pycollimator's global backend switch. 

The key features to adopt are:
1. **Structured simulation configuration** (`SimulationOptions`)
2. **Validation guards** on diagram construction
3. **LTI / PID block patterns** from the library
4. **`linearize()` utility** once JAX support is mature
5. **Event system design awareness** for future hybrid systems
