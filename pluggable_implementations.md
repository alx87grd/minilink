# Pluggable implementations layout

This document is the canonical reference for how `minilink` names and arranges **abstract contracts** and **concrete implementations** when a subsystem supports multiple backends (optimizers, solvers, renderers, compilers, …).

Design and roadmap may link here; avoid duplicating the full spec elsewhere.

## Principles

1. **Role-specific folders** — Prefer `optimizers/`, `solvers/`, `renderers/`, and (within a domain) family subpackages such as `search/` over a generic `backends/` folder. The directory name should say *what* is being swapped. A contract that orchestrates *all* families in a package may sit at that package’s root next to core types (e.g. `planning/planner.py`).

2. **Singular contract module** — The abstract API lives in a module named after the role, not `base.py`:
   - `optimizers/optimizer.py` — ABC for mathematical-program solvers (`Optimizer`).
   - `planning/planner.py` — shared planning orchestration (`Planner[ResultT]`, plus `TrajectoryPlanner` when the artifact is a trajectory), at the `planning` package root because it spans all planner families.

3. **One file per concrete implementation** — e.g. `optimizers/scipy_minimize.py` for SciPy’s `minimize` adapter.

4. **No barrel re-exports** — Import from the module that defines each symbol (see `agent.md`). Package `__init__.py` files stay docstring-only namespace markers unless the project explicitly freezes a different policy.

5. **Domain types stay at package root** — e.g. `mathematical_program.py` holds `MathematicalProgram` and constraint dataclasses; they are not “optimizers.”

## Canonical layout template

```text
minilink/<package>/
  <core_types>.py
  <role>/
    __init__.py              # docstring only
    <role_singular>.py       # ABC / protocol
    <impl>.py                # concrete implementation(s), one primary mechanism per file
```

## Implemented today (reference)

### `minilink/optimization`

```text
minilink/optimization/
  mathematical_program.py
  optimizers/
    __init__.py
    optimizer.py             # Optimizer
    scipy_minimize.py        # ScipyMinimizeOptimizer
```

### `minilink/planning`

```text
minilink/planning/
  planner.py                 # Planner[ResultT], TrajectoryPlanner
  problems.py
  sets.py
  costs.py
  search/                    # e.g. RRTPlanner(TrajectoryPlanner)
  trajectory_optimization/   # e.g. DirectCollocationPlanner(TrajectoryPlanner)
  policy_synthesis/          # e.g. DynamicProgrammingPlanner(Planner[StaticSystem])
```

## Legacy layouts (migrate when touched)

These packages predate this document; they remain valid until refactored:

| Package | Current pattern | Notes |
|--------|------------------|--------|
| `minilink/compile` | `evaluator.py` + `numpy_evaluator.py` + `jax_evaluator.py` + `compiler.py` | See **Compile migration** below |
| `minilink/simulation` | `solver_backends.py` (ABC + implementations in one module) | Future: `solvers/solver.py` + `solvers/scipy_ivp.py`, … |
| `minilink/graphical` | `renderers/base.py` + `*_renderer.py` | Already close; optional rename to `renderers/renderer.py` + `matplotlib.py` |

## Future: `minilink/compile` — evaluator as domain, compilers as strategies

**Vocabulary:** The **evaluator** is the domain-level *product* of compilation: the object that exposes `f`, `h`, `outputs`, and simulation-facing hooks (`DynamicsEvaluator` today). **Compilers** are *pluggable strategies* that turn a `System` / `DiagramSystem` and `ExecutionPlan` **into** a concrete evaluator (NumPy, JAX, …).

Today `compiler.py` mixes orchestration (plan build, dispatch) with backend-specific paths. A future layout should separate:

- `evaluators/evaluator.py` — `DynamicsEvaluator` ABC.
- `evaluators/numpy.py`, `evaluators/jax.py` — concrete evaluator classes.
- `compilers/` — compile strategies that emit evaluators (exact split TBD in a migration PR).
- A thin public façade (e.g. `compile.py`) that routes `backend="numpy"|"jax"` and returns an **evaluator**.

## Future: `minilink/simulation`

Target shape (illustrative):

```text
minilink/simulation/
  solvers/
    __init__.py
    solver.py                # SolverBackend ABC
    scipy_ivp.py
    euler.py
    rk4_fixed.py
  simulator.py
  input_interpolation.py
```

## Future: `minilink/graphical`

Target shape (illustrative):

```text
minilink/graphical/renderers/
  __init__.py
  renderer.py                # AnimationRenderer ABC
  matplotlib.py
  meshcat.py
  pygame.py
```

## Role name cheat sheet

| Concern | Subpackage | Contract module (example) |
|--------|------------|---------------------------|
| Finite-dimensional NLP / mathematical programs | `optimizers/` | `optimizer.py` |
| Planning orchestration across families | `planning/` (root) | `planner.py` |
| ODE integration / IVP | `solvers/` | `solver.py` (future) |
| Compiled dynamics evaluation | `evaluators/` | `evaluator.py` (future) |
| Draw / animate | `renderers/` | `renderer.py` (future) |
| Compile strategies (produce evaluators) | `compilers/` | `compiler.py` (future, name TBD) |
