# Minilink AI Agent Instructions

This file defines the collaboration preferences and architectural expectations for AI agents working on `minilink`.

## 1. Core Directives

- **Math readability first**: equations should read like textbook math such as `dx = A@x + B@u`.
- **Minimalist UX**: keep the main workflow beginner-friendly.
- **MVP prototyping is allowed**: mark unvalidated architecture with `TODO: User Architectural Review`.
- **Incremental refactoring**: avoid broad unapproved restructures.
- **Docs are part of the contract**: keep `DESIGN.md` and `ROADMAP.md` aligned with the code.

## 2. Coding Standards

- **Python**: 3.10+; keep `DESIGN.md`, `agent.md`, and `pyproject.toml` in sync
- **Type hints**: required on public APIs
- **Docstrings**: NumPy style on public classes and methods
- **Math naming**:
  - matrices: `A`, `B`, `H`, `M`, `K`
  - vectors: `x`, `u`, `y`, `q`, `v`, `dq`
  - dimensions: `n`, `m`, `p`
- **Readability over cleverness**: prefer straight, explicit code
- **Edits stay on task**: change only what the request needs; no drive-by refactors, unrelated files, or scope creep; every line in the diff should earn its place
- **Match the neighborhood**: before writing, read surrounding code and align with its naming, types, imports, and documentation density
- **Tests only when justified**: add or update tests for stable public APIs, TRL milestones, documented contracts, or explicit user requests
- **Validation in proportion**: avoid defensive error-handling sprawl in internal paths unless the interface is public or the risk is real
- **Documentation**: do not add new markdown guides or expand unrelated docs unless asked; `DESIGN.md` / `ROADMAP.md` stay in sync when behavior or scope changes (§1, §3)
- **Imports and “math-first” surfaces**: In tutorials, demos, and other **reader-facing** code, keep the top of the file light—fewer imports and less Python ceremony so the math stays visible. This is **judgment, not dogma**: internal packages (`compile/`, `simulation/`, benchmarks, tests) may use richer imports when the benefit is clear. Prefer moving heavy setup into helpers or modules casual readers do not need to open.
- **Package `__init__.py` (entire `minilink`)**: Import from the module that defines each symbol (e.g. `from minilink.compile.compiler import ...`, `from minilink.benchmark.simulation_speed import ...`, `from minilink.blocks.dynamic_bicycle import ...`). Do not use package `__init__` as a barrel re-export layer; those files are namespace markers (module docstring only, optional) unless an explicit API freeze and docs say otherwise. **Keep each `__init__.py` file** so subpackages stay regular packages and the build (Hatch) can discover them reliably; an empty or docstring-only `__init__.py` is fine, but deleting it risks tooling and import edge cases.

## 3. Architectural Guidance

- Use **inheritance** for core system types and **composition** for diagrams and optional behaviors.
- Keep the readable modeling path clean; isolate optimization in `compile/` and `simulation/`.
- Support JAX when it stays clean; use specialized JAX paths when needed instead of complicating the main path.
- **KISS and thin surfaces**: Prefer fewer files, fewer lines, and fewer dependencies when a simpler design is enough. When complexity is justified, keep **user-facing scripts and examples** minimal and push mechanics into backend modules or utilities.

Compiled-evaluator vocabulary:

- `outputs()` / `outputs_p()` means **boundary outputs only**
- diagram internals belong to the internal-signal APIs, not `outputs()`
- do not reintroduce `compute_outputs(..., ports=...)`
- keep `ExecutionPlan.output_slices` and `external_output_slices` aligned
- preserve JAX traceability of `f` and port compute paths

Any change to evaluator contracts, `ExecutionPlan`, or diagram compile behavior should update `DESIGN.md` and, if scope changed, `ROADMAP.md`.

## 4. 3-Level Verification

At feature completion, verify through:

1. **Automated tests** with `pytest`
2. **Manual test script** in `tests/manual/`
3. **Demo script** in `examples/` for major user-facing features

## 5. TRL Lifecycle

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

**Supplemental: recent feature bands (read with the module table in `DESIGN.md` §2 and `ROADMAP.md` §1).** These are not a second scale; they map the same TRL definitions to the latest integration work:

| Area | Effective band | Notes |
| --- | --- | --- |
| `matplotlib_style`, stacked-figure policy, `plot` modes on `plot_trajectory` | **TRL 6–7** | Covered by unit tests; user-visible contract; details may still move before a “final” TRL 9 sign-off. |
| `COMPILE_BACKEND_AUTO` on `Simulator` / `compile_backend` on `System` | **TRL 4–6** | Public API with tests; `"auto"` is opt-in on `Simulator`, default remains NumPy on high-level `System` for predictability. |
| Auto `rk4_fixedsteps` selection (long uniform grid + JAX + non-stiff) | **TRL 3–5** | Heuristic; `solve_forced` still requires a SciPy or Euler-style path; document and test, but do not treat as immutable policy yet. |
| `System.plot_trajectory` / `compute_trajectory(..., plot=...)` return behavior | **TRL 6+** | Aligned with plotting API and tests. |

## 6. Workflow Rules

**Do directly**

- fix typos and wording
- add missing type hints or public docstrings
- make small cosmetic PEP8 cleanups outside math expressions

**Always ask first**

- deleting or renaming files
- architectural refactors or core logic changes
- adding heavy dependencies
- changing the public evaluator or execution-plan contract

Demo and manual scripts should stay flat and directly runnable at module top level.

Benchmark example scripts live under `tests/benchmark/` (same flat style). Import from `minilink.benchmark.f_speed` or `minilink.benchmark.simulation_speed` as needed (`benchmark_f_speeds`, `benchmark_sim_speed_matrix`, `DEFAULT_SWEEP_PAIRS`, etc.); see `DESIGN.md` §4.6.

### Scope: small edits vs larger work

- **Small chat tweaks**: expect a **quick, minimal source change**—not a long autonomous loop (many tool rounds, long terminal sessions, broad refactors) unless the user clearly asked for that depth.
- **Larger work**: before extended execution (multi-step implementation, heavy CI/debug iteration, wide exploration), write a concise **implementation plan** and wait for **explicit approval** to proceed at that scale.
- **Scope surprise**: if a “little modif” turns into a **big job** after reading the code, **stop and ask** what slice or outcome they want rather than grinding forward.

## 7. Local Environment

**Use the `dev-h26` conda environment** for this repository: run tests, examples, and benchmarks only with that env’s `python` (or equivalent), not system Python or an ad-hoc venv. The project targets **Python 3.10+**; macOS `/usr/bin/python3` is often 3.9 and will not work (e.g. `|` in type hints).

```bash
conda activate dev-h26
# then: python, pytest, etc. from the repo root with PYTHONPATH=. as documented
```

On a typical Anaconda install the interpreter is `.../envs/dev-h26/bin/python` (for example `/opt/anaconda3/envs/dev-h26/bin/python` on the maintainer machine). From a fresh shell you can also use:

```bash
conda run -n dev-h26 python -m pytest
conda run -n dev-h26 python examples/scripts/demo_animations.py
```

Reserve `dev-h26` (or a clone of it) for work on `minilink` so optional deps (JAX, SymPy, visualization) stay aligned with the team and with what agents and CI are expected to use.
