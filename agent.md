# Minilink AI Agent Instructions

This file defines project-specific expectations for AI agents working on
`minilink`. The short version: keep the math readable, keep interfaces thin,
and keep docs synchronized with code.

## 1. Core Directives

- **Math readability first**: equations should read like textbook math, for
  example `dx = A @ x + B @ u`.
- **Mechanical-engineering audience**: code should be reviewable by a reader who
  thinks in systems, signals, and equations before Python abstractions.
- **Coach the architecture**: when a requested change risks adding ceremony,
  name the tradeoff and steer toward the simplest clear interface.
- **Prototype honestly**: unvalidated architecture may exist, but mark it with
  `TODO: User Architectural Review` and keep it easy to replace.
- **Incremental refactoring**: avoid broad restructures unless the user asks for
  that scope.
- **Docs are contract**: update `DESIGN.md` and `ROADMAP.md` when public
  contracts, package layout, or maturity claims change.

## 2. Coding Standards

- Python 3.10+.
- Public APIs need type hints and NumPy-style docstrings.
- Keep optional heavy imports lazy.
- Match the neighborhood before editing an existing file.
- Change only what the task requires; every diff line should earn its place.
- Prefer explicit loops and named temporaries over dense Python tricks when the
  data flow is mathematical.
- Use dataclasses for transparent records such as trajectories, execution-plan
  operations, benchmark rows, and results.
- Do not turn scientific objects such as systems, plants, and controllers into
  dataclasses when explicit initialization makes the model clearer.
- Use `ABC` only when contract enforcement is valuable. Plain base classes with
  `NotImplementedError` are fine for Pyro-style mother classes with shared
  defaults and optional hooks.

### Math Naming

- matrices: `A`, `B`, `H`, `M`, `K`;
- vectors: `x`, `u`, `y`, `q`, `v`, `dq`;
- dimensions: `n`, `m`, `p`;
- programmatic ids: `sys_id`, `port_id`, `block_id`;
- display strings: `label`, `labels`.

In public equation paths (`f`, `h`, output-port compute functions, mechanical
hooks, linearization, transcriptions), map inputs once and then use textbook
locals. Avoid repeated `np.asarray`/reshape checks inside internal math paths
unless the helper is a public boundary.

### Docs And Comments

- Main public classes and modules should be Sphinx-ready enough to render later:
  NumPy sections, inline code literals, and cross-references where useful.
- Internal helpers and secondary conveniences can use short plain-English
  docstrings.
- Comments stay plain and sparse; add them only where they clarify non-obvious
  logic.
- Do not add new markdown guides unless asked. Prefer updating `README.md`,
  `DESIGN.md`, `ROADMAP.md`, or this file.

## 3. Core Architecture Rules

- A `System` is dynamics and outputs: `f(x, u, t, params)`,
  `h(x, u, t, params)`, ports, dimensions, defaults, and optional visualization
  hooks.
- `System.f`, `System.h`, and output-port compute functions are native-array
  equation paths. NumPy-like inputs should produce NumPy-compatible expressions;
  JAX inputs should produce JAX-compatible expressions when the formula is meant
  to be traceable.
- `System` may expose convenience methods such as `compile`,
  `compute_trajectory`, `compute_forced`, `render`, and `animate`; these are
  boundary facades over compile/simulation/graphics modules.
- `params is None` means use object defaults. Any non-`None` `params` value is
  explicit and must override defaults.
- A `Trajectory` is sampled time/state/input data plus optional sampled signals.
- `PlanningProblem.X0/Xf` are authoritative boundary sets. `x_start/x_goal` are
  representative points and shortcuts; when both a set and a representative are
  supplied, the representative must belong to the set.
- `Set.margin`, `SingletonSet.residual`, and `CostFunction.g/h` are native-array
  math paths. Boundary helpers such as constructors, `contains`, `sample`, and
  reporting methods may convert to NumPy/Python.
- `MathematicalProgram` is pure NLP data: `J`, aggregate `h`, aggregate `g`,
  bounds, optional derivatives, metadata, and no initial guess or solver state.
- Keep `float(...)`, forced `np.asarray(...)`, and Python reporting conversion
  out of equation paths unless the object is explicitly NumPy-only. Put those
  conversions in evaluators, solvers, plotting/reporting helpers, or other
  public boundaries.
- `Optimizer` is the bound method-preset interface. It owns one program, one
  compiled program evaluator, `z0`, and a method such as `scipy_slsqp`,
  `scipy_trust_constr`, or `ipopt`.

## 4. JAX And NumPy Rules

NumPy is the baseline. JAX is optional. A NumPy-only install must import the
library and collect tests without JAX failures.

Six rules keep backend behavior clean:

1. **Complex plants use optional twins**: add a `Jax<Plant>` subclass only when a
   concrete use case needs traceable plant dynamics.
2. **Simple algebra uses one class**: sets, costs, and transcriptions should use
   backend-native array math where possible.
3. **Backend choice is explicit**: use `compile_backend` or optimizer/program
   evaluator backend arguments, not global switches.
4. **Backend strings live in one module**: use constants and helpers from
   `minilink.compile.backend_policy`.
5. **JAX imports are lazy**: call `require_jax_numpy()` or backend-policy helpers
   inside JAX-only methods; do not import `jax` at module import time in library
   code.
6. **Use `array_module` only for small hybrid math**: do not hide complex plant
   differences behind generic dispatch if a twin class would be clearer.

Do not create a top-level `minilink.jax` package, global NumPy/JAX mode, or twin
classes whose only purpose is to wrap simple traceable algebra.

## 5. Package And File Layout

- Import symbols from the module that defines them; package `__init__.py` files
  are namespace markers unless a future public API freeze says otherwise.
- Swappable roles use role-specific folders and singular contract modules:
  `compile/evaluators/evaluator.py`, `simulation/solvers/solver.py`,
  `graphical/renderers/renderer.py`, and
  `optimization/optimizers/optimizer_backend.py`.
- `compile/compiler.py` is the compile orchestrator. Do not add
  `compile/compilers/`.
- Benchmarks live beside the subsystem they measure, with runnable scripts under
  `tests/benchmark/`. Import timing helpers from the defining package (for example
  `minilink.compile.benchmark` or `minilink.simulation.benchmark`); there is no
  top-level `minilink.benchmark` package.
- Demo and manual scripts should stay flat, direct, and runnable from the repo
  root.

## 6. Verification

At feature completion, verify in proportion to risk:

1. automated tests with `pytest`;
2. manual smoke scripts in `tests/manual/` when useful;
3. demo scripts in `examples/` for major user-facing workflows.

JAX twin plants need tests showing JAX equations match the NumPy reference in a
nominal case and a non-trivial parameter regime.

Run ruff on touched Python files. Markdown-only changes do not need ruff unless
Python docstrings were edited.

## 7. Workflow Rules

Do directly:

- fix typos and stale docs;
- add missing public docstrings/type hints in the files you are already touching;
- make small style cleanups that directly support the requested change.

Ask first:

- deleting or renaming files unless the user explicitly asked;
- architecture refactors or public API changes;
- adding dependencies;
- changing evaluator, execution-plan, or optimizer contracts;
- removing user-authored scratch/convenience code.

If a small request turns into a large job after inspection, stop and explain the
smallest useful slice.

## 8. TRL Lifecycle

Readiness levels are an internal maturity scale for planning and review. They
are not a release process by themselves.

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

The subsystem maturity matrix in `ROADMAP.md` uses these definitions.

## 9. Local Environment

The project targets **Python 3.10+** with optional extras for JAX, SymPy,
visualization, and Ipopt (`pyproject.toml`). Do not rely on macOS
`/usr/bin/python3` when it is older than 3.10 (for example `|` in type hints).

Maintainers often use a **conda** environment for local development (for example
named `dev-h26` on some machines). That environment name is **not contractual**:
any equivalent setup—conda, venv, or another tool—with Python 3.10+ and the
extras you need for the task is acceptable. Align optional dependency versions
with CI or teammates when running tests, examples, and benchmarks.

Example with conda (substitute your environment name):

```bash
conda activate dev-h26   # or your own env
python -m pytest
```

From a fresh shell:

```bash
conda run -n dev-h26 python -m pytest
conda run -n dev-h26 python examples/scripts/animation/demo_animations.py
```
