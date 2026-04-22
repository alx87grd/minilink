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
- **Tests only when justified**: add or update tests for stable public APIs, TRL milestones, documented contracts, or explicit user requests
- **Validation in proportion**: avoid defensive error-handling sprawl in internal paths unless the interface is public or the risk is real

## 3. Architectural Guidance

- Use **inheritance** for core system types and **composition** for diagrams and optional behaviors.
- Keep the readable modeling path clean; isolate optimization in `compile/` and `simulation/`.
- Support JAX when it stays clean; use specialized JAX paths when needed instead of complicating the main path.

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

Benchmark example scripts live under `tests/benchmark/` (same flat style). Prefer `from minilink.benchmark import ...` for `benchmark_f_speeds`, `benchmark_sim_speed_matrix`, and related helpers; see `DESIGN.md` §4.6.

## 7. Local Environment

Use the `dev-h26` conda environment for local development:

```bash
conda activate dev-h26
```
