# Minilink Technical Design

Architecture and public contracts. User guide: [README.md](README.md). Call chains:
[flows.md](flows.md).

## 1. Design Principles

1. **Math readability first**: e.g. `dx = A @ x + B @ u`.
2. **Pure contracts, convenient boundaries**: equation paths stay functional;
   facades (`compute_trajectory`, `plot_*`, `animate`) live at API boundaries.
3. **Explicit data flow**: visible objects and direct calls; no global backend
   switches or hidden registries.
4. **Backend-native math where simple**: one class for traceable NumPy/JAX algebra.
5. **Specialize only when it clarifies**: `Jax<Plant>` twins when a single class
   would sacrifice readability.

### NumPy and JAX

NumPy required; JAX optional (`minilink[jax]`), imported lazily via
`minilink.compile.backend_policy` / `require_jax_numpy()`. No `minilink.jax`
package, no global mode. Explicit `compile_backend` and evaluator backend args.
`array_module()` only for small hybrid helpers.

## 2. Interface Layers

| Layer | Use | Examples |
| --- | --- | --- |
| 1 Facades | Default | `compute_trajectory`, `plot_trajectory`, `+`/`>>`/`@` |
| 2 Orchestrators | Repeat runs, trajopt, NLP | `Simulator`, `TrajectoryOptimizationPlanner`, `Optimizer` |
| 3 Contracts | Custom wiring, extension | `DiagramSystem.connect`, `compile()`, `MathematicalProgram` |

Import from defining modules; `minilink/__init__.py` is a namespace marker until
exports are frozen ([ROADMAP.md](ROADMAP.md) P1).

## 3. Package Map

Component maturity is tracked only in [ROADMAP.md](ROADMAP.md); this table
describes package ownership.

| Package | Role |
| --- | --- |
| `core/` | `System`, `DiagramSystem`, ports, `Trajectory`, sets, costs, blocks |
| `compile/` | `ExecutionPlan`, NumPy/JAX evaluators |
| `simulation/` | `Simulator`, solvers, forcing |
| `optimization/` | `MathematicalProgram`, `Optimizer` |
| `planning/` | problems, trajopt, search prototypes |
| `dynamics/` | abstractions + catalog plants |
| `graphical/` | signals, phase plane, diagrams, animation |
| `symbolic/`, `physics/` | experimental symbolic mechanics and JAX physics demos |
| `control/` | controller and static-law blocks |

**Dynamics root:** `DynamicSystem` with `f`, `h`. Reusable bases in
`dynamics/abstraction` (`StateSpaceSystem`, `LTISystem`, `MechanicalSystem`,
`GeneralizedMechanicalSystem`). `StateSpaceSystem` builds its matrices through
methods `A(t, params)`, `B(t, params)`, `C(t, params)`, `D(t, params)` (so
`dx = A(t, params) @ x + B(t, params) @ u`), mirroring `MechanicalSystem.H(q,
params)`; subclasses assemble matrices from `params`. `LTISystem` is the
constant-matrix convenience built from `A, B, C, D` arrays (introspect via
`sys.A()`). Catalog names: `Pendulum`, `CartPole`,
`DynamicBicycle`. Mixed inputs → named ports + concrete allocation hooks; no
`WithPositionInputs` inheritance branches.

## 4. Core Object Contracts

### `System`

- **Math:** `f(x,u,t,params)`, `h(x,u,t,params)`; ports use same signature.
- **Dims:** `n` from constructor; `m` from input ports; `p` from primary output
  `"y"` only (aux `"x"` does not change `p`; no `"y"` ⇒ `p==0`).
- **Ports:** explicit, ID-first; infer `dim` from metadata or default 1. Extract
  slices with `get_port_values_from_u(u, "r", "y")`.
- **DynamicSystem shortcut:** `input_dim`, `output_dim`, `expose_state`,
  `y_dependencies` create standard `u`/`y`/`x`.
- **Control naming:** `r` reference, `y` measurement, `u` control.
- **Facades:** lazy simulation/graphics; latest rollout on `self.traj`.

### Native-array equation rule

Applies to `f`, `h`, port compute, sets, costs, transcriptions, `MathematicalProgram`
`J`/`h`/`g`. Native in, native out; no `np.asarray` / `float()` inside equation
paths. Convert at boundaries (evaluators, solvers, plotting, `Trajectory`, I/O).

### Parameters

- `params is None` → `self.params`; any other value overrides (never
  `params or self.params`).
- Compile: `bind_params=True` copies params; diagram parametric tier still WIP—
  recompile when JAX diagram params change.

### `DiagramSystem`

Composes subsystems by named ports; flattens state; compiled `ExecutionPlan` is
the main execution path (reference recursive path must stay equivalent).

Shortcuts (`core.composition`): `+` flat add only, `>>` series, `@` closed loop,
`autowire()` conservative fill. Diagram operands are flattened, not nested.
Explicit `add_subsystem` / `connect` remains canonical for general topology.

### `Trajectory`, sets, costs

- `Trajectory`: `t (N,)`, `x (n,N)`, `u (m,N)`, optional `signals`; NumPy reporting object.
- Sets: `margin ≥ 0` feasible; `contains`/`sample` may convert to NumPy.
- Costs: `g(x,u,t)`, `h(x,t)` on `CostFunction` in `core`; attach to
  `PlanningProblem`, not the plant.

## 5. Compilation And Simulation

`compile(system, backend)` → `DynamicsEvaluator` (`numpy`|`jax`|`auto`|`direct`).

Diagrams → `ExecutionPlan` → diagram evaluator. Internal outputs via
`reconstruct_internal_signals`; **`outputs()` / `outputs_p()` are boundary outputs
only** (not diagram internals). Keep `ExecutionPlan.output_slices` and
`external_output_slices` aligned. Do not reintroduce `compute_outputs(..., ports=...)`.

**Default sim API:** `compute_trajectory` / `compute_forced` → `Simulator` →
`Trajectory`. Unconnected inputs use port nominals; time-varying sources belong
in the diagram; forcing via `compute_forced`. Facades default
`compile_backend="numpy"`.

Solver presets: `scipy`, `scipy_stiff`, `scipy_max`, `scipy_ultra`, `scipy_lsoda`,
`euler`, `rk4_fixedsteps` (auto-picked when omitted). Planned: `SimulationOptions`
([ROADMAP.md](ROADMAP.md) P1).

## 6. Optimization And Planning

**NLP:** `minimize J(z)` s.t. `h=0`, `g≥0`, bounds. Pure `MathematicalProgram`;
`Optimizer` binds method preset (`scipy_slsqp`, `scipy_trust_constr`, `ipopt`).

**Planning:** `PlanningProblem` owns system, sets, cost. `X0`/`Xf` authoritative;
`x_start`/`x_goal` are shortcuts/representative points.

**Trajopt:** planner → transcription → `MathematicalProgram` → `Optimizer` →
`Trajectory`. Single backend-native transcription classes; no parallel JAX
transcription types.

## 7. Graphics And Benchmarks

Facades delegate to `graphical/`. Time plots: `signals=("x", "u", "block:port")`.
Phase plane: matplotlib default. Diagrams: Graphviz display, Mermaid export;
Plotly under `plotting` extra.

**Camera:** `get_camera_transform` → 4×4 matrix (`camera_matrix`); one contract
for all renderers. Override on `System` for custom views. Camera and kinematic
hooks are still under graphical/animation API review.

Benchmark helpers live beside subsystems; runners under `tests/benchmark/`—not
public contracts.

**Repo conventions:** Python 3.10+; typed public APIs; lazy optional imports;
namespace `__init__.py` files; plot subpackages may re-export small facades.
