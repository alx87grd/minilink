# Minilink AI Agent Instructions

Source of truth for AI agents. User API: [README.md](README.md). Contracts:
[DESIGN.md](DESIGN.md). Maturity: [ROADMAP.md](ROADMAP.md). Pytest policy:
[tests/README.md](tests/README.md).

Keep math readable, interfaces thin, and docs synchronized with code.

## Doc map

| Doc | When to update |
| --- | --- |
| [README.md](README.md) | User workflows, install, examples table |
| [DESIGN.md](DESIGN.md) | Public contracts, package layout, evaluator behavior |
| [ROADMAP.md](ROADMAP.md) | TRL / maturity claims, priority checkboxes |
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Pyro parity rows when library or demos land |
| [tests/README.md](tests/README.md) | Marker policy, test philosophy |

Do not add new markdown guides unless asked. Keep [README call chains](README.md#call-chains) minimal.

## Core directives

- **Math readability first**: equations read like textbook math, e.g. `dx = A @ x + B @ u`.
- **Mechanical-engineering audience**: reviewable by someone who thinks in systems and equations first.
- **Coach the architecture**: name tradeoffs; steer toward the simplest clear interface.
- **Minimalist UX**: beginner-friendly main workflow; complexity in orchestrators and backends.
- **Prototype honestly**: unvalidated architecture gets `TODO: User Architectural Review`.
- **Incremental refactoring**: no broad restructures unless the user asks.
- **Preserve user edits**: never revert or "clean up" manual changes the user made in demos, notebooks, examples, or scratch code — commented-out plots, tuning constants (`TF`, gains, step times), disabled sections, exploratory variables — unless they explicitly ask you to change those lines. Commit/review passes must not overwrite user-tuned script state.
- **Docs are contract**: update DESIGN / ROADMAP / README when public behavior or maturity claims change.

## Textbook style

Reading minilink should feel like a controls/dynamics textbook.

1. **Two-audience principle**: student reader (`core/system.py`, `blocks/`, `dynamics/`, `control/`) vs library developer (`core/compile/`, evaluators). Write each file for its primary reader.
2. **First-screen rule**: docstring and primary class within the first screen; validation below the contract or in its own module.
3. **Module section order**: primary contract → subclasses → public functions → private helpers (`# Public API`, `# Internal machinery`).
4. **Selector-orchestrator split**: public math tools read as choose method → get `f`/`h` callables → compute Jacobians in place; ceremony in selector helpers below.
5. **Bare signatures in equation paths**: no type hints in `f`/`h`/port computes; shapes in docstrings. Full hints on tools and structural APIs.
6. **The `xp` idiom**: `xp = array_module(x)` right after params unpacking — hybrid NumPy/JAX in one line.
7. **Derived, not cached**: computable quantities are read-only properties, never stale cached attrs.
8. **No shadow state**: initialize in `__init__`, never `hasattr`-or-create at use sites.
9. **Libraries are silent**: no `print` except explicit `verbose=`; delete debug scaffolding.
10. **Pre-1.0 no-alias rule**: rename cleanly; fix call sites in the same change.
11. **Backend imports from `core/backends.py`**: never import from `core/compile/` in system libraries.
12. **`__main__` hello-worlds**: ~10 lines max in core modules; bigger examples in `examples/`.

### General coding

- Match the neighborhood; change only what the task requires.
- Public APIs: type hints and NumPy docstrings — **except equation paths** (rule 5).
- Lazy optional imports; explicit readable code with named temporaries in equation paths.
- Low helper count in math tools; avoid single-use private methods (inline unless reused).
- **Tests only when justified**: stable public APIs, TRL milestones, contracts, or user requests.
- Validation in proportion; dataclasses for transparent records; `ABC` only when enforcement helps.

### Math naming

- Matrices `A`, `B`, `H`, `M`, `K`; vectors `x`, `u`, `y`, `q`, `v`, `dq`; dims `n`, `m`, `p`.
- **Unpack `params` before equations**; **no `self.` in core equation lines** — bind locals first.
- Lay out 2-D literals one row per line; use `# fmt: off` / `# fmt: on` for alignment.
- Reader-facing imports stay light in demos; internal packages may import richly when clear.

## Architecture reminders

Details in [DESIGN.md](DESIGN.md).

- Equation paths stay **native-array**; conversions at boundaries only.
- `params is None` → object defaults; any other `params` overrides — never `params or self.params`.
- **Inheritance** for core system types; **composition** for diagrams and optional behaviors.
- **`outputs()` / `outputs_p()` are boundary outputs only**; no `compute_outputs(..., ports=...)`.
- Package placement and dependency law: [DESIGN.md §3](DESIGN.md#package-map). Benchmarks in repo-root `benchmarks/`.
- NumPy/JAX policy: [DESIGN.md §1](DESIGN.md#numpy-and-jax).

## Workflow

**Do directly:** typos and stale docs; docstrings/types in files you are already changing for the task; small cleanups that directly support the requested change.

**Never without explicit ask:** revert, uncomment, rename, or "polish" user manual edits in `examples/`, notebooks, or scratch files (tuning params, commented plot/animate calls, exploratory locals).

**Ask first:** delete/rename files; architecture refactors; new dependencies; evaluator/optimizer contract changes; removing user scratch code.

**Scope:** stop and explain the smallest slice if a small request grows large. For larger work, write a concise plan and wait for approval. Chat conflicts with this file → ask before proceeding.

**Notebooks:** skip review unless updating renamed imports or user asks; outputs stripped by pre-commit (`nbstripout`).

Demos: flat under `examples/scripts/`, runnable from repo root.

## Before push or PR (local CI gate)

GitHub **CI** (`.github/workflows/test.yml`) runs exactly: `ruff check .`, `ruff format --check .`, `pytest` on Python 3.10–3.13. Run the same checks **locally before push or PR** so CI does not fail on lint/format — do **not** poll GitHub Actions after every small commit unless the user asked you to push or verify remote CI.

**Always before push** (fast; mirrors CI lint steps):

```bash
conda activate minilink
ruff check .
ruff format --check .
```

Fix with `ruff check --fix .` and `ruff format .` when either fails. CI runs these on the **whole repo**, not only touched files.

**Pytest — proportionate** (same command CI uses; scope by change):

| Change | Run |
| --- | --- |
| Docs/markdown only | skip pytest |
| Narrow module + tests already updated | `pytest path/to/test_foo.py` |
| Cross-cutting or before handoff/push | `pytest` |

Optional extras (not required every push): `SDL_VIDEODRIVER=dummy pytest` for pygame smoke; `sphinx-build` only when editing `docs/` (separate Docs workflow).

**After push:** only check GitHub CI when the user asked to push, open a PR, or debug a reported failure — not as a routine step on every edit.

## Verification

Use conda env **`minilink`** from [environment.yml](environment.yml); setup in [README.md#install](README.md#install) (`PYTHONPATH` = repo root).

After substantial changes: `pytest` (proportionate to risk), ruff on touched Python, smoke scripts when user-facing. JAX twin plants: nominal + nontrivial parameter test. Headless: `MPLBACKEND=Agg`; full suite notes in [tests/README.md](tests/README.md).

## Revision pass

Final pass after substantial changes — smaller, clearer diff:

1. Re-read diff for scope creep and stale comments; **preserve user manual edits** in demos/notebooks.
2. Simplify; match local patterns; lazy optional imports.
3. Math-first locals in equation paths; conversion at boundaries only.
4. Fold or update examples; runnable from repo root.
5. Sync README (user API), DESIGN (contracts), ROADMAP (maturity if changed).
6. Tests for new behavior; benchmarks only when performance claims matter.
7. Verify: **pre-push gate** (ruff + pytest per table above); headless graphics smoke when relevant.
8. Handoff: clean `git status`, short summary of changes and verification; run ruff before push if committing.
