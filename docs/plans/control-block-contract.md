# Control-block contract: `StaticController`, declared feedback ports, `plot_control_law`

Design writeup for a foundational control-block contract: every controller
declares its feedback pattern (measurement / ref / control ports) as class
metadata usable by any `System`; memoryless laws additionally subclass
`StaticController` and implement the pure-array `control_law(y, r, t, params)`.
`@` composition reads the declaration instead of guessing. Static-map plotting
gets one engine and two facades: generic `plot_input_output_map` (any static
block sweeps its own input port) and `plot_control_law()` (controller teaching
shortcut — regulation slices over the controller's own measurement/reference
ports, **no plant argument**). Overlaying a control map on a plant phase plane
is a separate optional tool for later, not part of the controller API.

Status: **Draft — awaiting architectural review** (DESIGN §4 contract change).

## 0. The signature decision

Constraint scan across **existing laws**:

| Law | Needs | `e → u`? | `x → u`? | `(y, r) → u`? |
| --- | --- | --- | --- | --- |
| `ProportionalController` `K(r-y)` | error only | yes | no | yes |
| `StateFeedbackController` `ubar - K(x-r)` | error + offset | almost (loses `ubar` anchoring semantics) | yes (r baked in) | yes |
| `ImpedanceController` `Kp(r-pos) - Kd·rate` | **absolute** `rate`, error `pos` | **no** | — | yes |
| `ComputedTorqueController` `ID(q, dq, qdd(e))` | **absolute** `q, dq` for the model + error | **no** | — | yes |
| `SlidingModeController` | absolute `q, dq` + error | **no** | — | yes |
| `LookupTableController` `π(x)` | absolute state | no | yes | yes (`r` unused) |

`e → u` fails every model-based law (they need absolute coordinates for
`H(q)`, `g(q)`), and `x → u` fails output feedback. **`(y, r)` is the minimal
signature that covers all of them** — exactly Pyro's `StaticController.c(y, r, t)`.
Error feedback is a *special case computed inside the law* (`e = r - y` as the
first line), not a separate contract.

**Chosen contract** — pure native-array in/out, equation-path style (bare, like `f`):

```python
def control_law(self, y, r, t=0, params=None):
    """u = c(y, r, t).   y: (p,) measurement, r: (k,) reference, u: (m,)."""
```

**Future-proofing** (why this doesn't paint us into a corner):

- **Stateful laws** (adaptive, filtered PID, integral impedance): Pyro's own
  answer is `DynamicController.c(z, y, r, t)` — the state is *prepended*,
  `(y, r, t)` stays. A later `DynamicController(DynamicSystem)` sibling declares
  the same port contract with `control_law(x, y, r, t, params)`. Nothing in
  phase 1 blocks it.
- **MPC / solvers**: never get `control_law` (not a static map), but **do** get
  the declared port contract (`measurement_port="y"`, `control_port="u_ff"`,
  `ref_port=None`) — pure metadata usable by `@`. The contract and the base
  class are deliberately decoupled.
- **NN / RL policies** (`SB3Controller`, future learned laws): declare
  `measurement_port="y"`, wrap `predict` in `control_law` →
  `plot_control_law` over the policy input port, no plant.
- **JAX**: `(y, r, t, params)` with `xp = array_module(y)` traces like `f` —
  no object state in the hot path.
- **Regulation shortcut** (Pyro `cbar`): `regulation_action(y, t=0)` =
  `control_law(y, rbar, t)` with `rbar` from the ref port nominal — defined
  once on the base.

The *two-level* pattern is the foundation:

```mermaid
flowchart TB
  subgraph contract [Level 1 — declared contract, ANY System, pure metadata]
    attrs["measurement_port / ref_port / control_port / feedback_profile"]
  end
  subgraph static [Level 2 — StaticController base, memoryless laws]
    law["control_law(y, r, t, params)"]
    wrap["ctl() port wrapper + regulation_action() — defined once"]
    plotfacade["plot_control_law()"]
  end
  subgraph consumers [Consumers]
    matmul["@ composition (reads Level 1)"]
    plotter["graphical/port_map (uses Level 2, duck-typed)"]
    mpc["MPC, DynamicController later (Level 1 only)"]
  end
  attrs --> static
  attrs --> matmul
  law --> plotter
  attrs --> mpc
```

Dispatch is **duck typing** (`getattr(block, "measurement_port", None)`), never
`isinstance` — so `LookupTableController` in `planning/` complies without
importing `control/` (dependency law intact).

## 1. Exact impact on existing code

### New: `minilink/control/controller.py` (~90 lines)

```python
class StaticController(System):
    feedback_profile = "output"
    measurement_port = "y"
    ref_port = "r"          # None on subclasses without a reference
    control_port = "u"

    def __init__(self, measurement_dim, control_dim, ref_dim=None, rbar=None, ...):
        # builds ports from the declaration; dependencies=(ref, measurement)
        # rbar → ref port nominal_value

    def ctl(self, x, u, t=0, params=None):        # defined ONCE
        k = self.inputs[self.ref_port].dim if self.ref_port else 0
        r, y = u[:k], u[k:]
        return self.control_law(y, r, t, params)

    def regulation_action(self, y, t=0, params=None):
        return self.control_law(y, self.rbar, t, params)

    def plot_control_law(self, **kwargs):   # lazy graphical import; no sys=
```

Port-bundle order `(r, y)` is **internal** — diagrams wire by port name, so
standardizing the dependencies order changes nothing externally (verified by
the full-pytest gate in Phase C).

### Per-file migration (before → after)

| File | Today | After | Removed |
| --- | --- | --- | --- |
| [output.py](../../minilink/control/output.py) | 43 lines: port setup + `ctl` unpacking | declaration + 3-line `control_law`: `return K @ (r - y)` | own `ctl`, `add_*_port` calls |
| [state.py](../../minilink/control/state.py) | port setup + `ctl` unpacking (`x` then `r`) | `measurement_port = "x"`; `control_law`: `return ubar - K @ (y - r)`; `rbar=xbar` | own `ctl`, port setup |
| [impedance.py](../../minilink/control/impedance.py) `ImpedanceController` | `ctl` unpacks `r/pos/rate`, branches on ref dim | `control_law(y, r)`: `pos, rate = y[:n], y[n:]`, same branch | own `ctl`, port setup (labels passthrough kept) |
| [impedance.py](../../minilink/control/impedance.py) `ImpedanceIntegralController` | `DynamicSystem` (integral state) | **not migrated** — declares Level-1 attrs only | nothing |
| [robotic.py](../../minilink/control/robotic.py) `ModelJointImpedance`, `TaskImpedance`, `TaskKinematic` | each has `ctl` + port setup | declaration + `control_law`; embedded-model rule untouched | own `ctl`s, port setup |
| [modelbased.py](../../minilink/control/modelbased.py) `ComputedTorqueController` | binds `_ctl_tracking` **or** `_ctl_regulation` at init; extra `ctl` indirection | single `control_law(y, r)` branching on `len(r)` (same pattern as impedance) | both `_ctl_*` methods, the `ctl` indirection |
| [modelbased.py](../../minilink/control/modelbased.py) `SlidingModeController` | **mutates** `self.outputs["u"].compute = ctl_fn` after `super().__init__` | overrides `control_law` only; `solver_info` kept | the compute-swap hack |
| [siso.py](../../minilink/control/siso.py) `FilteredController` | `DynamicSystem` (filter states) | **not migrated** — Level-1 attrs only | nothing |
| [lookup_policy.py](../../minilink/planning/policy_synthesis/lookup_policy.py) | `System` + `action()` + `ctl` | declares Level-1 attrs (`measurement_port="x"`, `ref_port=None`) + `control_law(y, r)` → interp; **no `control/` import** | nothing (additive) |
| [mpc/controller.py](../../minilink/control/mpc/controller.py) | port-name heuristics find `u_ff` | declares Level-1 attrs only (`control_port="u_ff"`) | nothing (additive) |
| `lqr.py`, factories | return `StateFeedbackController` | unchanged signatures | — |

**Constructor signatures do not change** → zero churn in demos, notebooks,
tests in Phases A–B. The diff is *negative* in `control/`: each migrated file
loses its port boilerplate and `ctl` unpacking.

### What stays exactly as-is

- `System` / `DiagramSystem` / compile path — untouched (the wrapper `ctl` is
  an ordinary port compute).
- `feedback_profile` strings — kept (docs + error hints), now backed by real
  port declarations.
- Embedded-model params rule (DESIGN §4) — `control_law` reads `self.plant`
  live params the same way `ctl` does today.
- DP/VI plotting (`planner.plot_policy` = discrete table) — unrelated, unchanged.

## 2. `@` composition: before → after

Today `_resolve_feedback_path` in
[composition.py](../../minilink/core/composition.py) **guesses** from port
names/dims:

```
try y↔y dims → try Mux(q,dq)→y (dim 2n) → try x↔x → error
```

After, for any block with a declared contract, controller-side ambiguity
disappears:

```python
# resolve_standard_feedback — new first step
measurement = getattr(controller, "measurement_port", None)
if measurement is not None:
    control_out = getattr(controller, "control_port", "u")
    # only PLANT-side resolution remains:
    #   measurement == "x"          → plant x output
    #   measurement == "y", dim 2n  → plant y, else Mux(q,dq)
    #   ref_port is None            → no boundary ref (lookup, MPC)
```

Simplifications this buys:

1. **The lookup-controller patch generalizes**: `ref_port is None` replaces the
   `if ref_port in controller.inputs` check added for `@` with
   `LookupTableController` — declaration-driven instead of port-probing.
2. **`_default_control_out` heuristic** (`u` → `u_ff` → first output) becomes a
   fallback only; MPC declares `control_port="u_ff"` explicitly.
3. **Precise errors** from the declaration: *"ImpedanceController declares
   measurement y=[q;dq] dim 4; plant y has dim 2 — plant must expose q/dq (or
   pass feedback='qdq')"* instead of the generic dim-mismatch.
4. **`default_computer_boundary_ports`** (hybrid `Computer @ plant`) reads the
   same declaration — one contract for flow and step paths.

Heuristics are **kept as fallback** for undeclared/third-party systems:
behavior-preserving, gated by full `pytest` in Phase C (matmul tests in
`test_core.py`, SMC loop in `test_simulation.py`, lookup matmul in
`test_planning.py`).

## 3. Static-map plotting: one engine, two facades (controller-local)

**Design rule:** a controller plots **its own input–output map**. It never
takes a plant (`sys=`). The plant appears only when wiring a closed loop
(`ctl @ plant`); plotting stays in the coordinates the law actually sees.

Build order:

1. **Core** — `plot_input_output_map` on any static `System` (generic IO slice).
2. **Teaching shortcut** — `plot_control_law` on controller blocks (regulation
   defaults, feedback vocabulary, same engine).
3. **Later (optional)** — standalone helper to overlay a control map on a
   plant phase plane (needs `h(x)` / plant labels); not a controller method.

```mermaid
flowchart TB
  subgraph engine [Shared engine — graphical/port_map.py]
    mesh["mesh: pick 1–2 axes, pin the rest at nominal"]
    eval["evaluate output port / control_law per node"]
    render["1D curve / 2D heatmap → PlotResult"]
  end
  subgraph facades [Facades — build in this order]
    iomap["System.plot_input_output_map — generic static block"]
    ctllaw["plot_control_law — controller teaching shortcut"]
  end
  subgraph later [Later — not Phase D]
    overlay["optional: control map over plant phase plane"]
  end
  iomap --> engine
  ctllaw --> engine
  overlay -.-> engine
```

### 3.1 Engine — `graphical/port_map.py`

Mirror [phase_plane](../../minilink/graphical/phase_plane/phase_plane.py):
spec dataclass → mesh builder → evaluate → matplotlib render →
[PlotResult](../../minilink/graphical/common/plot_result.py).

- 1-D swept axis → line plot; 2-D → heatmap; higher-D → pick `x_axis` /
  `y_axis`, pin remaining coordinates.
- Bounds from explicit `bounds=` or port `nominal_value` ± range /
  `lower_bound` / `upper_bound`.
- Labels from port `labels` / `units`.
- Reject blocks with internal state (`DynamicSystem` with `n > 0`) for the
  static IO path unless a future dynamic slice API pins controller state
  (§5).

Implementation lives in `graphical/port_map.py` as module functions; facades
delegate lazily (repo rule: no `graphical/` import at controller import time).

```python
# graphical/port_map.py (module-level — usable without control/ import)
def plot_input_output_map(block, *, in_port=None, out_port=None, ...)
def plot_control_law(block, *, x_axis=0, y_axis=None, ...)
```

### 3.2 Core facade — `System.plot_input_output_map`

Generic static IO map. No feedback semantics.

```python
def plot_input_output_map(self, in_port=None, out_port=None, *, x_axis=0,
                          y_axis=None, out_axis=0, bounds=None,
                          grid_shape=(101, 101), show=True) -> PlotResult
```

Defaults: first input port → first output port. Sweeps components of the
**input port vector**; pins other input components at `nominal_value`.

Examples: `Saturation` curve, `Relay` step, `Gain` line,
`NeuralNetwork` response surface.

Eligibility: static `System` only (no diagram state in the output compute path).
`DynamicSystem` raises with a clear message pointing to §5 for dynamic slices.

### 3.3 Teaching shortcut — `plot_control_law`

Controller-specific wrapper over the **same engine**. Sweeps the controller's
**measurement port** (name from `measurement_port`, usually `y` or `x`); pins
**reference** at `rbar` (`ref_port.nominal_value`) unless `r=` is passed.

```python
def plot_control_law(self, *, x_axis=0, y_axis=None, u_axis=0,
                     r=None, t=0.0, bounds=None,
                     grid_shape=(101, 101), show=True) -> PlotResult
```

**No `sys` argument.** Axis indices refer to the measurement port vector. When
`ref_port is None` (lookup, RL), sweep the sole input port (`x`).

At each mesh node:

1. Build measurement vector `y` (or `x`) from swept/pinned coordinates.
2. `u = regulation_action(y, t)` if `r is None`, else `control_law(y, r, t)`.
3. Plot `u[u_axis]` (scalar heatmap/curve).

Available on:

- `StaticController` subclasses (method delegates to `port_map.plot_control_law`).
- Duck-typed blocks with `control_law` + Level-1 attrs (`LookupTableController`
  in `planning/` — thin method, no `control/` import).

**Why a separate name from `plot_input_output_map`?** Same engine, but
controllers merit a one-liner teaching API: regulation default, measurement-port
semantics, `u` output, no need to name ports. A `Saturation` block should never
see `plot_control_law` (selector-orchestrator split).

### 3.4 Demo payoff — [vi_pendulum_lqr.py](../../examples/demos/planning/value_iteration/vi_pendulum_lqr.py)

The manual `lqr_law` array already sweeps LQR's **`x` port** with `r` fixed at
`UPRIGHT` — no plant object involved.

```python
# before: manual array bypassing the block
K = lqr.params["K"][0]; ubar = lqr.params["ubar"][0]
lqr_law = ubar - (grid.states - UPRIGHT) @ K
plotting.plot_value(grid, lqr_law, title="LQR control law", ...)

# after — controller-local, same slice
lqr.plot_control_law(x_axis=0, y_axis=1)   # r pinned at rbar = UPRIGHT
planner.get_controller().plot_control_law(x_axis=0, y_axis=1)   # VI lookup, same API
```

`PolicyEvaluator` cost-to-go stays on `plotting.plot_value` (that's `J`, not
the law). Replaces the hand-rolled `lqr_policy` def:
`PolicyEvaluator(..., policy=lqr.regulation_action)`.

Side-by-side in the demo: `planner.plot_cost2go()` (value) +
`lqr.plot_control_law()` (law) — parallel teaching views, no shared `sys=`.

### 3.5 Later — plant phase-plane overlay (out of Phase D scope)

Optional standalone utility (not on the controller), e.g. in `graphical/port_map.py`:

```python
def plot_control_on_plant_phase_plane(controller, plant, *, ...)
```

Would mesh plant state, build `y` via `h(x)` when needed, evaluate the law,
and optionally underlay `plant.plot_phase_plane` vector field. Nice for
teaching when measurement ≠ state, but **not required** for the core contract.
Ship only if a demo clearly needs it after Phase D.

## 4. Phases + gates (each behavior-preserving)

| Phase | Content | Gate |
| --- | --- | --- |
| A | Base class + migrate `output.py`, `state.py`, `impedance.py` | new equivalence tests (`ctl` bundle == `control_law` direct); `pytest tests/unittest/test_core.py test_simulation.py` |
| B | `robotic.py`, `modelbased.py`; Level-1 attrs on lookup + MPC | `pytest tests/unittest/test_planning.py` + robotic demos smoke |
| C | Composition reads declaration (fallback kept) | **full `pytest`** — zero behavior change allowed |
| D | `graphical/port_map.py` engine + `plot_input_output_map` (core) + `plot_control_law` (controller shortcut); facades on `System` / `StaticController` | Agg smoke: LQR, lookup, impedance, `Saturation`, `NeuralNetwork`; run `vi_pendulum_lqr.py`, `vi_pendulum_swingup.py` |
| E | Demo migration + DESIGN §feedback-profiles rewrite + pyro-port row (`StaticController.plot_control_law` → Done) | `ruff check .`, `ruff format --check .` |
| F | `DynamicController` + dynamic `plot_control_law` slices (`x_ctrl` axes); optional `plot_control_on_plant_phase_plane` | extend `FilteredController` smoke; dynamic axis pin tests |

**Risk inventory**

- Port-bundle packing order: internal to each block's `ctl`; standardized in
  base; full pytest catches any leak.
- `SlidingModeController.solver_info["discontinuous_behavior"]` must survive
  the migration.
- `params is None → self.params` inside `control_law` (never
  `params or self.params`).
- No leading-underscore methods on the base (repo rule); unpack logic inlined
  in `ctl`.
- `graphical/` imported lazily from the facade only.
- DESIGN §4 contract change → `TODO: User Architectural Review` marker until
  maintainer sign-off on the migrated files.

## 5. Dynamic controllers (Phase F — after static plotting lands)

Same Level-1 port metadata. New base sibling:

```python
class DynamicController(DynamicSystem):
    def control_law(self, x, y, r, t=0, params=None):
        """u = c(x, y, r, t).   x: (n_ctrl,) internal state."""

    def ctl(self, x, u, t=0, params=None):
        r, y = ...   # same bundle unpack as StaticController
        return self.control_law(x, y, r, t, params)
```

`f(x, u, t)` stays explicit (anti-windup, filter dynamics — not derived from
`control_law` alone).

**Plotting — same API, wider workspace, still no plant:**

Plot workspace `z = [x_ctrl ; measurement]` (reference pinned at `rbar` by
default, same as static). `x_axis` / `y_axis` index into `z`:

- Default regulation slice: sweep measurement port, pin `x_ctrl = x0` (often
  zeros → “no integral / filter memory”).
- Teaching slices: e.g. measurement vs `e_int` for PID windup.

Examples:

```python
pid.plot_control_law(x_axis=0, y_axis=1)              # sweep y, ctrl pinned at x0
pid.plot_control_law(x_axis=0, y_axis=pid.n + 0)      # y[0] vs e_int (axis into x_ctrl)
```

Migrate: `FilteredController`, `ImpedanceIntegralController`. Still no `sys=`.
Optional plant phase-plane overlay (§3.5) remains a separate tool if added.
