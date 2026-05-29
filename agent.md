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
- **Minimalist UX**: keep the main workflow beginner-friendly; push complexity to
  orchestrators and backend modules, not reader-facing scripts.
- **Prototype honestly**: unvalidated architecture may exist, but mark it with
  `TODO: User Architectural Review` and keep it easy to replace.
- **Incremental refactoring**: avoid broad restructures unless the user asks for
  that scope.
- **Docs are contract**: update `DESIGN.md` and `ROADMAP.md` when public
  contracts, package layout, or maturity claims change.

## 2. Coding Standards

- Python 3.10+; keep `DESIGN.md`, `agent.md`, and `pyproject.toml` aligned when
  behavior or dependencies change.
- Public APIs need type hints and NumPy-style docstrings.
- Keep optional heavy imports lazy.
- Match the neighborhood before editing an existing file.
- Change only what the task requires; every diff line should earn its place.
- Prefer explicit, readable code over clever Python when the data flow is
  mathematical; use named temporaries in equation paths.
- **Tests only when justified**: add or update tests for stable public APIs, TRL
  milestones, documented contracts, or explicit user requests—not for trivial or
  obvious behavior.
- **Validation in proportion**: avoid defensive error-handling sprawl on internal
  paths unless the interface is public or the failure mode is real.
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

**Reader-facing imports:** in tutorials, demos, and examples, keep the top of the
file light so the math stays visible. Internal packages (`compile/`, `simulation/`,
benchmarks, tests) may use richer imports when the benefit is clear.

### Docs And Comments

- Main public classes and modules should be Sphinx-ready enough to render later:
  NumPy sections, inline code literals, and cross-references where useful.
- Internal helpers and secondary conveniences can use short plain-English
  docstrings.
- Comments stay plain and sparse; add them only where they clarify non-obvious
  logic.
- Do not add new markdown guides unless asked. Prefer updating `README.md`,
  `DESIGN.md`, `ROADMAP.md`, or this file. Keep [flows.md](flows.md) to minimal
  call chains only—do not grow step-by-step internal traces in markdown.

## 3. Architecture And Contracts

Canonical contracts live in [DESIGN.md](DESIGN.md). User workflows:
[README.md](README.md). Minimal call chains: [flows.md](flows.md). Update those
when you change public behavior.

Quick reminders (details in DESIGN):

- Equation paths (`f`, `h`, ports, sets/costs, programs, transcriptions) stay
  **native-array**; conversions belong at boundaries (evaluators, solvers,
  plotting, `contains`, …).
- `params is None` uses object defaults; any other `params` overrides—never
  `params or self.params`.
- Use **inheritance** for core system types; use **composition** for diagrams
  and optional behaviors. Keep readable modeling in `core/`; isolate compile,
  simulation, optimization, and graphics.
- **Compiled-evaluator vocabulary:** `outputs()` / `outputs_p()` are **boundary
  outputs only**; diagram internals use internal-signal APIs, not `outputs()`.
  Do not reintroduce `compute_outputs(..., ports=...)`. Keep
  `ExecutionPlan.output_slices` and `external_output_slices` aligned. Preserve
  JAX traceability of `f` and port compute paths.
- Changes to evaluator contracts, `ExecutionPlan`, or diagram compile behavior
  must update `DESIGN.md` and, if scope changed, `ROADMAP.md`.

## 4. NumPy And JAX

Library-wide policy is under [DESIGN.md §1](DESIGN.md#numpy-and-jax) (NumPy and
JAX). When implementing: explicit backend arguments (`compile_backend`, evaluator
backends), vocabulary from `minilink.compile.backend_policy`, lazy JAX imports, and
no `minilink.jax` package or global NumPy/JAX mode.

## 5. Package And File Layout

- Import symbols from the module that defines them; package `__init__.py` files are
  namespace markers unless a future public API freeze says otherwise. **Keep each
  `__init__.py`** so subpackages stay discoverable to Hatch and import tooling;
  do not use package `__init__` as a barrel re-export layer.
- Swappable roles use role-specific folders and singular contract modules:
  `compile/evaluators/evaluator.py`, `simulation/solvers/solver.py`,
  `graphical/renderers/renderer.py`,
  `optimization/optimizers/optimizer_backend.py`.
- `compile/compiler.py` is the compile orchestrator—do not add `compile/compilers/`.
- Benchmark helpers live beside their subsystem; runners live under
  `tests/benchmark/`. Import from defining packages (for example
  `minilink.compile.benchmark`, `minilink.simulation.benchmark`)—there is no
  top-level `minilink.benchmark` package.
- Demo and manual scripts stay flat and runnable from the repo root.

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

**Scope:**

- **Small tweaks** — quick, minimal source change; no broad refactors unless asked.
- **Larger work** — write a concise implementation plan and wait for explicit
  approval before extended multi-step execution.
- **Scope surprise** — stop and ask which slice the user wants rather than
  grinding forward.

Demo and manual scripts stay flat and runnable from the repo root. Benchmark
runners live under `tests/benchmark/`; import helpers from defining subsystem
packages (see §5), not a top-level `minilink.benchmark` package.

## 8. TRL Lifecycle

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

Subsystem maturity in [ROADMAP.md](ROADMAP.md) uses these definitions.

## 9. Local Environment

Target **Python 3.10+** with optional extras from `pyproject.toml` (JAX, SymPy,
visualization, Ipopt). Do not rely on macOS `/usr/bin/python3` when it is older
than 3.10.

Maintainers often use a conda env for local work (for example `dev-h26`). That
name is **not contractual**—any conda/venv with Python 3.10+ and the extras you
need is fine; align versions with CI or teammates when running tests and examples.

```bash
conda activate dev-h26   # or your own env
python -m pytest
```

```bash
conda run -n dev-h26 python -m pytest
conda run -n dev-h26 python examples/scripts/animation/demo_animations.py
```
