# Minilink AI Agent Instructions

Source of truth for AI agents. User API: [README.md](README.md). Contracts:
[DESIGN.md](DESIGN.md). Maturity: [ROADMAP.md](ROADMAP.md). Pytest policy:
[tests/README.md](tests/README.md).

Keep math readable, interfaces thin, and docs synchronized with code.

## Doc map

| Doc | When to update |
| --- | --- |
| [README.md](README.md) | User workflows, install, examples table |
| [DESIGN.md](DESIGN.md) | Public contracts, package layout, evaluator behavior, **product identity & scope** |
| [ROADMAP.md](ROADMAP.md) | TRL / maturity claims, teaching-release criteria & priorities, review queue, out-of-scope |
| [docs/plans/TODO.md](docs/plans/TODO.md) | Operational backlog: small fixes, pre-v0.2 hardening, demo pulls, new modules, Later ideas |
| [docs/plans/](docs/plans/) | Active **design** writeups only (multi-step plans; delete finished plan docs) |
| [docs/plans/pyro-port-remaining.md](docs/plans/pyro-port-remaining.md) | Pyro parity rows when library or demos land |
| [tests/README.md](tests/README.md) | Marker policy, test philosophy, **entry points (human · agent · CI)** |

Do not add new markdown guides unless asked. Keep [README call chains](README.md#call-chains) minimal.

**Intro-doc scope:** [README.md](README.md), marketing showcases
([showcase/minilink.ipynb](examples/learn/intro/showcase_minilink.ipynb),
[showcase/jax.ipynb](examples/learn/intro/showcase_jax.ipynb)), and the
[`intro/`](examples/learn/intro/) module API notebooks present the **main
core tools and features** — `System` / diagrams / simulate / compile / analysis /
planning trajopt / the hybrid step path (`StepSystem`, `StepDiagramSystem`,
`Computer`, `HybridDiagram`) / MPC as the hybrid exemplar. Do **not** update those
intro surfaces to track every new demo, compare script, or `examples/projects/`
experiment. New demos land under `examples/`; update DESIGN/ROADMAP when contracts
or maturity change. Add a README examples-table row only when a demo is a
**canonical** teaching entry for a core tool (e.g. `examples/demos/mpc/mpc_car_minimal.py`).

**Examples buckets** (see [examples/README.md](examples/README.md)):
`learn/` (intro = library curriculum; teaching/ = domain lessons),
`demos/<topic>/` (canonical single-file
scripts), `projects/`, `tooling/` (notebooks and/or scripts), `sandbox/`
(+ `scratch/`). Do not mix `.ipynb` and `.py` in the same leaf module folder
(except inside one named project). Demo stems keep the topic when needed
(e.g. `mpc/mpc_car_minimal.py`; no `demo_` prefix). Pedagogical compares stay
in `demos/`; mission matrices go to `sandbox/` or `projects/`. Scripts are
top-level open-and-run (no `main()`); do not add plant-only smokes — use catalog
`__main__` instead. Tooling lives under `examples/tooling/`, plus
`tests/demo_checks/` and `benchmarks/`.

## Core directives

- **Math readability first**: equations read like textbook math, e.g. `dx = A @ x + B @ u`.
- **Mechanical-engineering audience**: reviewable by someone who thinks in systems and equations first.
- **Coach the architecture**: name tradeoffs; steer toward the simplest clear interface.
- **Minimalist UX**: beginner-friendly main workflow; complexity in orchestrators and backends.
- **Prototype honestly**: unvalidated architecture gets `TODO: User Architectural Review`.
- **Incremental refactoring**: no broad restructures unless the user asks.
- **Preserve user edits**: never revert or "clean up" manual changes the user made in demos, notebooks, examples, or scratch code — commented-out plots, tuning constants (`TF`, gains, step times), disabled sections, exploratory variables — unless they explicitly ask you to change those lines. Commit/review passes must not overwrite user-tuned script state.
- **No test harness in demos**: `examples/demos/` and `examples/learn/` never read CI/smoke env vars or branch on “are we in a test?” (`MINILINK_NOTEBOOK_SMOKE`, etc.). Smoke runners adapt outside the demo (`MPLBACKEND=Agg`, timeouts, optional-dep skips). Teaching code may fall back on missing optional packages (e.g. Ipopt → SciPy) — that is user UX, not a test hook.
- **Docs are contract**: update DESIGN / ROADMAP / README when public behavior or maturity claims change.
- **Familiar patterns first**: do not introduce programming concepts or advanced Python styles absent from the repo and the user's prior choices (e.g. `typing.Protocol`, metaclasses) unless there is a strong runtime or maintainability reason. Static-typing-only wins are not enough on their own — prefer patterns already in use (mixins, unions, duck typing). If the tradeoff is unclear, validate with the user before landing the pattern.

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
- **No leading-underscore method names**: do not mark helpers “private” with a `_` prefix
  on system / facade / simulator classes. Prefer a plain descriptive name even for
  shared helpers — the file section (`# Public API` / `# Internal machinery`) is enough
  to signal intent. Leading `_` is fine for module-level constants and truly local temps.
- **Tests only when justified**: stable public APIs, TRL milestones, contracts, or user requests.
- Validation in proportion; dataclasses for transparent records; `ABC` only when enforcement helps.

### Math naming

- Matrices `A`, `B`, `H`, `M`, `K`; vectors `x`, `u`, `y`, `q`, `v`, `dq`; dims `n`, `m`, `p`.
- **Leaf = diagram role only** — reserve *leaf* for a subsystem node inside `DiagramSystem` / `StepDiagramSystem` (compile/plan context). Standalone `DynamicSystem` / `StepSystem` wrappers use descriptive type names (e.g. `DiscretizedDynamicSystem`), not `*Leaf`.
- **Unpack `params` before equations**; **no `self.` in core equation lines** — bind locals first.
- Lay out 2-D literals one row per line; use `# fmt: off` / `# fmt: on` for alignment.
- Reader-facing imports stay light in demos; internal packages may import richly when clear.

## Architecture reminders

Details in [DESIGN.md](DESIGN.md).

- **Continuous-time core is the priority** — `DynamicSystem`, flow diagrams, `Simulator`, and analysis on `f` are the main framework. `StepSystem` / hybrid are subsidiary utilities for discrete control in the loop (MPC, SMC). On trade-offs, keep the continuous path clean; step/hybrid add-ons use sibling types and separate compile/sim paths — do not complicate flow `compile()`, `DiagramSystem`, or `Simulator`.
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

**Notebooks:** skip review unless updating renamed imports or user asks; outputs
stripped by pre-commit (`nbstripout`). After notebook edits, smoke-check with
`MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py` (CI
``regression`` job runs the same).

Demos: under `examples/demos/` (canonical) or `examples/sandbox/`
(non-core / WIP); runnable from repo root. Map: [examples/README.md](examples/README.md).

## Before push or PR (local CI gate)

**Entry points:** [tests/README.md#entry-points](tests/README.md#entry-points) — humans use **`tests/run/`** (IDE Run); agents and CI use the CLI table in that doc.

GitHub **CI** (`.github/workflows/test.yml`) runs exactly: `ruff check .`, `ruff format --check .`, `pytest` on Python 3.10–3.13, then the **`regression`** job (regression gates + flagship demos + notebook smoke with JAX). Run the same checks **locally before push or PR** so CI does not fail on lint/format — do **not** poll GitHub Actions after every small commit unless the user asked you to push or verify remote CI.

**Always before push** (fast; mirrors CI `test` job):

```bash
conda activate minilink
ruff check .
ruff format --check .
```

Fix with `ruff check --fix .` and `ruff format .` when either fails. CI runs these on the **whole repo**, not only touched files.

**Pytest — proportionate** (same command CI `test` job uses; scope by change):

| Change | Run |
| --- | --- |
| Docs/markdown only | skip pytest |
| Narrow module + tests already updated | `pytest tests/unittest/test_<domain>.py` |
| Cross-cutting or before handoff/push | `pytest` |
| Compile backend, simulator, or trajopt changes (big review pass) | Regression gates: `PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny --factor 10 --speed-gate-suffixes solve_s,nlp_s,speedup` |
| Teaching notebooks | `MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py` |

Regression gates full command and CI `regression` job flags: [tests/README.md#entry-points](tests/README.md#entry-points).

Optional extras (not required every push): `SDL_VIDEODRIVER=dummy pytest` for headless pygame; `sphinx-build` only when editing `docs/` (separate Docs workflow).

**After push:** only check GitHub CI when the user asked to push, open a PR, or debug a reported failure — not as a routine step on every edit.

## Verification

Use conda env **`minilink`** from [environment.yml](environment.yml); setup in [README.md#install](README.md#install) (`PYTHONPATH` = repo root). **Test entry points:** [tests/README.md#entry-points](tests/README.md#entry-points).

After substantial changes: `pytest` (proportionate to risk), ruff on touched Python, demo-check scripts when user-facing. JAX twin plants: nominal + nontrivial parameter test. Headless: `MPLBACKEND=Agg`; full suite notes in [tests/README.md](tests/README.md).

**Big review pass** (compile backend, `Simulator`, trajectory optimization, or cross-cutting dynamics changes): run the committed regression baselines before handoff:

```bash
python benchmarks/run_regression_check.py --suite all
```

Use `--update` only after intentional perf or trajectory changes; review the JSON diff before committing. See [benchmarks/README.md](benchmarks/README.md).

## Revision pass

Final pass after substantial changes — smaller, clearer diff:

1. Re-read diff for scope creep and stale comments; **preserve user manual edits** in demos/notebooks.
2. Simplify; match local patterns; lazy optional imports.
3. Math-first locals in equation paths; conversion at boundaries only.
4. Fold or update examples; runnable from repo root.
5. Sync README (user API), DESIGN (contracts), ROADMAP (maturity if changed).
6. Tests for new behavior; benchmarks only when performance claims matter.
7. Verify: **pre-push gate** (ruff + pytest per table above); headless graphics checks when relevant.
8. Handoff: clean `git status`, short summary of changes and verification; run ruff before push if committing.
