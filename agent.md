# Minilink AI Agent Instructions

This file is the **source of truth** for AI agents working on `minilink`.
[AGENTS.md](AGENTS.md) and [.cursor/rules/read-agent-first.mdc](.cursor/rules/read-agent-first.mdc)
are thin auto-injected entry points that point here.

The short version: keep the math readable, keep interfaces thin, and keep docs
synchronized with code.

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

### Textbook Style

The core promise: reading minilink should feel like reading a controls and
dynamics textbook. These rules keep it that way as the library grows.

1. **Two-audience principle** (headline rule): every file has a primary
   reader — the *student* (`core/system.py`, `core/signals.py`, `blocks/`,
   `dynamics/`, `control/`) or the *library developer* (`core/compile/`,
   `core/composition.py`, evaluators). Write each file for its
   primary reader; push the other audience's machinery elsewhere.
2. **First-screen rule**: a student-facing module shows its docstring and
   primary class within the first screen. Validation and inference plumbing
   never sits above the contract — it goes below the class or in its own
   module (e.g. `core/signals.py`).
3. **Module section order**: primary contract class → subclasses/variants →
   public functions → private helpers at the bottom, separated by short
   section banners (`# Public API`, `# Internal machinery`).
4. **Equation-block docstrings**: every `System` subclass and contract class
   opens its docstring with the governing equations (e.g. `dx = f(x, u, t; p)`;
   model: `costs.py`'s `J = ∫ g(x,u,t) dt + h(x(tf), tf)`).
5. **Bare signatures in equation paths**: `def f(self, x, u, t=0, params=None):`
   — no type annotations in `f`/`h`/port computes; shapes and types live in
   the docstring. Full type hints belong on tools, orchestrators, and
   structural APIs.
6. **The `xp` idiom**: `xp = array_module(x)` right after params unpacking is
   *the* hybrid NumPy/JAX pattern — one line, always the same shape, and the
   entire price of JAX support in basic blocks.
7. **Derived, not cached**: quantities computable from owned state are
   read-only properties (`System.m`, `System.p`), never cached attributes
   guarded by `recompute_*()` call discipline — no staleness class of bugs.
8. **No shadow state**: attributes are initialized in `__init__`, never via
   `hasattr(...)`-or-create at use sites.
9. **Libraries are silent**: no `print` outside explicit `verbose=` flags,
   default quiet. Delete debug scaffolding rather than gating it.
10. **Pre-1.0 no-alias rule**: no compatibility aliases or shims inside the
    library; rename cleanly and fix call sites in the same change.
11. **Backend imports come from `core/backends.py`**: backend vocabulary and
    the `array_module` / `require_jax_numpy` helpers live there; system
    libraries never import from `core/compile/`.
12. **`__main__` hello-worlds**: core modules may end with a ~10-line runnable
    example (`python -m minilink.core.system`); anything bigger belongs in
    `examples/`.

### General

- Python 3.10+; keep `DESIGN.md`, `agent.md`, and `pyproject.toml` aligned when
  behavior or dependencies change.
- Public APIs need type hints and NumPy-style docstrings — **except equation
  paths**, which stay bare (Textbook Style rule 5).
- Keep optional heavy imports lazy.
- Match the neighborhood before editing an existing file.
- Change only what the task requires; every diff line should earn its place.
- Prefer explicit, readable code over clever Python when the data flow is
  mathematical; use named temporaries in equation paths.
- **Avoid single-use private methods.** Do not split a constructor or equation
  into `self._helper()` steps that run only once (e.g. `_set_metadata`,
  `_abcd`); inline them at the call site. Keep a helper only when it is reused,
  recursive, or genuinely clarifies. Module-level functions shared by several
  classes are fine.
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

**Unpack `params` before equations:** in equation paths (`H`, `C`, `g`, `d`,
`f`, `h`, …) read each needed value out of the `params` dict into a short local
at the top of the method (`m1 = params["m1"]`, `l1 = params["l1"]`), then write
the formula in those symbols. Keep dict indexing out of the math so the core
equations read like textbook math.

**No `self.` in core equations:** bind matrices and terms returned by methods or
attributes to short locals first (`A = self.A(t, params)`, `H = self.H(q,
params)`), then write the formula in those symbols (`dx = A @ x + B @ u`). The
final equation line should read like math, free of `self.` and dict indexing.

**Let core equations breathe:** separate the core equation from surrounding
bookkeeping with a blank line, and add a short comment naming the relation or
physics it encodes (e.g. the governing ODE), not one that narrates the syntax.

**Lay out matrices as matrices:** write 2-D array literals with one row per line
and columns visually aligned so the structure reads like written math. The
formatter collapses alignment spacing, so wrap an aligned literal in
`# fmt: off` / `# fmt: on` to preserve it.

**Reader-facing imports:** in tutorials, demos, and examples, keep the top of the
file light so the math stays visible. Internal packages (`core/compile/`, `simulation/`,
benchmarks, tests) may use richer imports when the benefit is clear.

### Docs And Comments

- Main public classes and modules should be Sphinx-ready enough to render later:
  NumPy sections, inline code literals, and cross-references where useful.
- Internal helpers and secondary conveniences can use short plain-English
  docstrings.
- Comments stay plain and sparse; add them only where they clarify non-obvious
  logic.
- Do not add new markdown guides unless asked. Prefer updating `README.md`,
  `DESIGN.md`, `ROADMAP.md`, or this file. Keep [README call chains](README.md#call-chains)
  minimal—do not grow step-by-step internal traces in markdown.

## 3. Architecture And Contracts

Canonical contracts live in [DESIGN.md](DESIGN.md). User workflows and minimal
call chains: [README.md](README.md). Update those when you change public behavior.

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
backends), vocabulary and helpers from `minilink.core.backends`, lazy JAX imports, and
no `minilink.jax` package or global NumPy/JAX mode.

## 5. Package And File Layout

- Import symbols from the module that defines them; package `__init__.py` files are
  namespace markers unless a future public API freeze says otherwise. **Keep each
  `__init__.py`** so subpackages stay discoverable to Hatch and import tooling;
  do not use package `__init__` as a barrel re-export layer.
- Packages belong to one of four bands — framework (`core/`, incl.
  `core/compile/`), system libraries (`blocks/`, `dynamics/`, `control/`,
  `estimation/`), tools (`simulation/`, `analysis/`, `planning/`,
  `optimization/`, `identification/`, `graphical/`, `interfaces/`), and
  quarantine (`symbolic/`, `physics/`). Dependency law and placement
  algorithm: [DESIGN.md §3](DESIGN.md). Shelve library content by *role in
  the diagram*, never by implementation technology; tools never define
  user-facing `System` subclasses (factories are fine).
- Swappable roles use role-specific folders and singular contract modules:
  `core/compile/evaluators/evaluator.py`, `simulation/solvers/solver.py`,
  `graphical/renderers/renderer.py`,
  `optimization/optimizers/optimizer_backend.py`.
- `core/compile/compiler.py` is the compile orchestrator—do not add `compilers/` folders.
- All performance benchmarking (helpers, fixtures, runners) lives in the
  repo-root `benchmarks/` directory — never inside `minilink/` or `tests/`.
  It imports minilink like an external user; nothing in `minilink/` may
  import it (unittest smoke tests may).
- Demo and manual scripts stay flat and runnable from the repo root.

## 6. Verification

After substantial changes, run the full checklist in [§10 Revision Pass](#10-revision-pass).

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

If instructions in chat conflict with this file, ask for clarification before
proceeding.

**Notebooks:** exclude notebooks from normal review/style passes unless the task
is only to update renamed imports/API names or the user explicitly asks for
notebook cleanup. Never review notebook outputs by default.

**Scope:**

- **Small tweaks** — quick, minimal source change; no broad refactors unless asked.
- **Larger work** — write a concise implementation plan and wait for explicit
  approval before extended multi-step execution.
- **Scope surprise** — stop and ask which slice the user wants rather than
  grinding forward.

Before handoff on larger work, complete [§10 Revision Pass](#10-revision-pass).

Demo and manual scripts stay flat and runnable from the repo root. Benchmark
helpers, fixtures, and `run_*` scripts live in repo-root `benchmarks/` (see §5).

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
than 3.10. For terminal commands in this repo (tests, `examples/scripts`,
benchmarks), use Python 3.10+ with the extras your task needs.

Install from repo root:

```bash
pip install -e ".[dev]"
```

Optional extras: `.[jax]`, `.[symbolic]`, `.[visualization]`, `.[plotting]`,
`.[ipopt]`. Diagram bindings need a system `graphviz` package
(`apt install graphviz`, or conda-forge `graphviz` + `python-graphviz`).

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

Run demos from repo root: `python examples/scripts/<script>.py`.

Headless: set `MPLBACKEND=Agg` before graphical imports. `plot_diagram()` D-Bus
warnings in headless environments are harmless. `animate()` needs a display;
plotting and simulation work headless with `show=False`.

## 10. Revision Pass

Final pass after substantial changes. Goal: a smaller, clearer diff.

1. **Re-read diff** — scope creep, dead code, stale comments; preserve user edits.
2. **Simplify** — match local patterns; thin public APIs; lazy optional imports;
   fail clearly on unsupported paths.
3. **Math-first** — textbook locals in equation paths; conversion at boundaries only
   ([Textbook Style](#textbook-style)).
4. **Examples** — fold or update demos; runnable from repo root; no new guides unless asked.
5. **Docs** — sync [README.md](README.md) (user API), [DESIGN.md](DESIGN.md)
   (contracts), [ROADMAP.md](ROADMAP.md) (maturity only if changed). Keep
   [README call chains](README.md#call-chains) minimal—update chains, not
   step-by-step internals
   ([Docs And Comments](#docs-and-comments), [§3 Architecture And Contracts](#3-architecture-and-contracts)).
6. **Tests** — new behavior, regressions, optional-extra skip messages; benchmarks
   only when performance claims matter.
7. **Verify** — `ruff check .`, `ruff format --check .`, `pytest` (proportionate
   to risk; see [§6 Verification](#6-verification)); `MPLBACKEND=Agg` for headless
   graphics smoke.
8. **Handoff** — `git status`, no stray artifacts; short summary of changes and
   verification.
