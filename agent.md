# Minilink AI Agent Instructions

This file defines the collaboration preferences and architectural expectations for AI agents working on `minilink`.

## 1. Core Directives

- **Math readability first**: equations should read like textbook math such as `dx = A@x + B@u`.
- **Minimalist UX**: keep the main workflow beginner-friendly.
- **Client-readable code is a requirement**: the maintainer/client is a mechanical engineer, not a professional programmer, and expects to review detailed code before it enters the library. If code cannot be understood by an engineering reader who thinks in equations, the design is not finished.
- **Agent as coach**: the agent is not only an executor; it should actively keep the maintainer on track toward clean, textbook-like, engineering-readable code. When a choice risks drifting toward professional-library ceremony, the agent should name the tradeoff and steer back to the simplest readable math formulation.
- **Validate deviations from textbook style**: if the agent believes the code should deviate from the math-first standard (for performance, JAX tracing, public API stability, packaging, or safety), it should pause and ask when the choice affects architecture, public APIs, teaching surfaces, or code the maintainer must review line-by-line. For small internal details, give a short explanation in the change summary.
- **MVP prototyping is allowed**: mark unvalidated architecture with `TODO: User Architectural Review`.
- **Incremental refactoring**: avoid broad unapproved restructures.
- **Docs are part of the contract**: keep `DESIGN.md` and `ROADMAP.md` aligned with the code. For **on-disk package layout, folder names, and pluggable *role* naming**, follow `DESIGN.md` §2; update §2 if you move or rename packages.

## 2. Coding Standards

- **Python**: 3.10+; keep `DESIGN.md`, `agent.md`, and `pyproject.toml` in sync
- **Type hints**: required on public APIs
- **Docstrings**: NumPy section structure on public classes and methods. See **Sphinx and docstring depth** (next item).
- **Sphinx and docstring depth**: **Main public API**—module docstrings for primary entry points, exported / user-facing classes, and their **core** public methods that define the product contract—use docstrings that are **Sphinx-ready** (reStructuredText that numpydoc and autodoc can render in a future `sphinx-build`): NumPy `Parameters` / `Returns` / `Attributes` / `Notes` as needed, long-form intros per earlier bullets, `:`-roles for cross-references when helpful (`:class:`, `:meth:`, `:func:`, `:data:`, `:mod:`), double backticks for inline code literals, `:math:` for nontrivial equations. The intent is that automatic HTML (e.g. Read the Docs) can be added later without rewriting those strings. **Internal and secondary surfaces**—`_*` private helpers, internal utilities, and **secondary** public methods (convenience, plotting, thin helpers)—prioritize **readability in the source**: short plain-English docstrings, type hints on signatures, **minimal** reST (no required roles or link targets). **Comments** (`#`) stay plain; never use Sphinx or RST markup in comments.
- **Math naming**:
  - matrices: `A`, `B`, `H`, `M`, `K`
  - vectors: `x`, `u`, `y`, `q`, `v`, `dq`
  - dimensions: `n`, `m`, `p`
- **Math naming in method bodies (reinforced)**: In public dynamic paths (`f`, `h`, `outputs`, `forward_kinematic_*`, linearization, and similar), prefer these textbook names for **locals** (`x`, `u`, `t`, and matrix symbols as above). Longer names are fine when they disambiguate (e.g. bounds vs. a single state component); at the API boundary, map once, then use `x` / `u` inside the core math. Do not rename public `System` (or other stable) attributes solely for style; apply this rule inside methods and in new code.
- **Long-form docstring intros**: For **class** and **module** docstrings, start with 2–6 lines: what it models, key equations (e.g. `dx = f(x, u, t)`), and a short line on signal shape when useful (e.g. state in R^n, input in R^m). Then use standard NumPy sections (`Parameters`, `Returns`, `Attributes`, `Notes`, `Examples` as needed). For **contract** methods (`f`, `h`, etc.), a one-line summary is enough; avoid repeating full dimension tables on every method if the class docstring already defines them.
- **Visual sectioning and module/class order**: **Default:** **thin** separators between **groups** of code (e.g. one line like `# --- Helpers ---` or `# === Internal ===`, up to about 72 characters), not around every method. **Optional, sparingly:** a **long** hash-stripe line (e.g. `###...###` to margin) may mark an exceptional **“core”** block once or twice in a file—e.g. the main dynamics integration or the primary public API the reader must not miss. Do not use long stripes for routine helpers or every subsection; that defeats scanability. **Class** order (recommended when layout is not constrained by a base class): docstring, `__init__` (if any), **core contract** methods (`f`, `h`, model-relevant port computes), then **secondary** public (convenience, plotting, `linearize` when not the main read), then **private** `_*` helpers grouped at the end. **Module** order: main public API first, then a labeled block for helpers / internals. Follow normal PEP 8 blank lines and Ruff; do not add triple blank lines. Do not add banners that only add length without new information. Within one file, pick either **thin-only** or **thin + an occasional long stripe** for core, not a mix of competing “heavy” styles.
- **Style rollout (forward only)**: New and meaningfully edited code should follow the bullets above. **Match the neighborhood** when changing an existing file: do not reflow a whole file solely to apply sectioning or docstring shape unless the user asked. There is no obligation to backfill the rest of the repository in one pass.
- **Readability over cleverness**: prefer straight, explicit code
- **Explain or ask before Python-heavy machinery**: If an agent believes a Python-heavy pattern is worth adding—deep typing, abstract protocols, metaprogramming, decorators, factories, hidden registries, broad error wrapping, dataclass layers, or other professional-library ceremony—it should either ask first or clearly explain the value before doing it. Ask first when the choice affects public APIs, architecture, teaching surfaces, or code the maintainer is expected to review line-by-line; a short explanation is enough for small internal helpers with an obvious payoff.
- **Edits stay on task**: change only what the request needs; no drive-by refactors, unrelated files, or scope creep; every line in the diff should earn its place
- **Match the neighborhood**: before writing, read surrounding code and align with its naming, types, imports, and documentation density
- **Tests only when justified**: add or update tests for stable public APIs, TRL milestones, documented contracts, or explicit user requests
- **Validation in proportion**: avoid defensive error-handling sprawl in internal paths unless the interface is public or the risk is real
- **Documentation**: do not add new markdown guides or expand unrelated docs unless asked; `DESIGN.md` / `ROADMAP.md` (and §2 when the tree changes) stay in sync with the code and each other (§1, §3)
- **Imports and “math-first” surfaces**: In tutorials, demos, and other **reader-facing** code, keep the top of the file light—fewer imports and less Python ceremony so the math stays visible. This is **judgment, not dogma**: internal packages (`compile/`, `simulation/`, benchmarks, tests) may use richer imports when the benefit is clear. Prefer moving heavy setup into helpers or modules casual readers do not need to open.
- **Notebook policy**: Most code reviews and style passes should **exclude notebooks** (`*.ipynb`). Do not inspect, summarize, or rewrite notebook outputs; outputs are often huge and not useful for source review. Touch notebooks only for narrow maintenance such as updating renamed imports, renamed public APIs, or small source-cell fixes explicitly requested by the user. Do not run notebook formatters or output-stripping unless the user asks for a notebook cleanup pass.

### Glue-code readability (adapter layers)

Prioritize **readability** over compact Python in glue layers (third-party APIs, I/O, callbacks). Avoid **unnecessary** list/dict comprehensions and `**`-unpack tricks when a plain loop is clearer—engineering readers may not parse dense Python idioms quickly.

### Textbook-first style, thin interfaces, and minimal ceremony (Pyro-informed)

Borrow the **scientific object model** and equation readability of the legacy Pyro toolbox where it helps. Do **not** copy Pyro's outdated Python style: old file headers, excessive hash banners, typo-prone public strings, weak public contracts, legacy compatibility branches, or `np.linalg.inv(...)` in places where a direct solve is clearer. Minilink code should feel like a robotics/control textbook with a thin Python skin.

- **Mental model for objects**: A *system* is dynamics and outputs (`f`, `h`, ports, bounds); a *trajectory* is time with state/input (and outputs when relevant); *blocks* compose in diagrams; an optional *symbolic* layer derives readable equations; *compilation* produces flat evaluators (NumPy/JAX) that are still recognizable as those same equations. Prefer names and layouts that make this pipeline obvious.
- **Pipeline story in the code**: The narrative **readable model → optional symbolic derivation → simulate / optimize → compiled evaluator** should be easy to follow from folder and API names. Keep the middle “readable Python equations” layer the main teaching surface; push optimizer/solver/backend glue to `optimization/`, `simulation/`, `compile/`, and renderer modules without dragging notational clutter back into modeling.
- **Equation methods read like equations**: In `f`, `h`, `H`, `C`, `B`, `g`, `d`, `ddq`, linearization, and similar paths, map inputs once at the top, then use textbook variables. Prefer short, named intermediate quantities (`q`, `dq`, `ddq`, `rhs`, `dx`, `y`) over dense Python idioms. Use `@` for matrix products and `np.linalg.solve` / `jnp.linalg.solve` for linear solves.
- **Thin boundaries**: At layer edges, pass a **small vocabulary** of clear types (`System`, `Trajectory`, `PlanningProblem`, evaluators, transcribers). Signatures and short methods should read like lecture pseudocode (`roll out x`, `stack constraints`) rather than deep stacks of helpers.
- **Audience (MATLAB-adjacent, engineering students)**: When it clarifies the math, prefer explicit loops and named temporaries over compact one-liners; put dimensions and signal shapes in class/module docstrings; avoid Python idioms that hide indices, tensor shapes, or data flow. A reader should think in state equations first and Python second.
- **Type hints that teach the math are good**: Keep annotations that communicate mathematical shape or interface, such as `Callable[[np.ndarray], np.ndarray]`, `np.ndarray -> float`, `Trajectory`, or `PlanningProblem`. Remove or avoid annotations that only add vague plumbing, such as broad `Any` usage or deep generic stacks that do not help an engineering reader.
- **Normalize once, then trust the contract**: Public entry points may accept list-like inputs and validate shapes (`Trajectory`, `Simulator`, file IO, user-facing convenience methods). Internal equation, rollout, transcription, and evaluator paths should usually trust `ndarray`-like inputs and let NumPy/SciPy/JAX fail naturally. Do not scatter `np.asarray`, reshaping, or broad shape checks through every helper unless that helper is the boundary.
- **Minimize distracting ceremony**: Avoid broad `try`/`except` inside internal integrators, rollouts, transcribers, and equation code unless the failure mode is truly confusing or an optional dependency is being handled. Skip elaborate typing machinery (deep `Protocol` nests, nested `TypeVar` games, `Any` annotations that do not clarify a public API) unless a stable public contract needs it.
- **Imports—keep headers light**: Prefer the shortest import block that preserves clarity. **`from __future__ import annotations` is optional**: use it when it genuinely simplifies annotations (forward references to classes in the same module, heavy composite types) or matches an established file pattern; do **not** add it to every new file “by policy.” Avoid importing `typing` symbols that only support rare edge cases.
- **Dataclasses and objects**: Use dataclasses for transparent records (`Trajectory`, execution-plan operations, benchmark results). Avoid turning scientific objects (`System`, plants, controllers) into dataclasses when explicit initialization makes the model easier to read.
- **ABC vs plain “mother class”**: **`ABC` + `@abstractmethod`** fits when **several implementations must** honor the same contract and you want failure at **instantiation** plus clearer static checking (for example `Planner`). A **plain base class** with `NotImplementedError` in methods meant to be overridden (Pyro-style “mother class”) is fine when the pattern is mostly **shared defaults** and **partial** or optional specialization—failure is deferred until an unimplemented method is called. Choose **contract enforcement** (ABC) vs **template with hooks** (plain base) on purpose; do not rip out existing `ABC` usage without cause.

- **Package `__init__.py` (entire `minilink`)**: Import from the module that defines each symbol (e.g. `from minilink.compile.compiler import ...`, `from minilink.simulation.benchmark import ...`, `from minilink.dynamics.catalog.vehicles.dynamic_bicycle import ...`). Do not use package `__init__` as a barrel re-export layer; those files are namespace markers (module docstring only, optional) unless an explicit API freeze and docs say otherwise. **Keep each `__init__.py` file** so subpackages stay regular packages and the build (Hatch) can discover them reliably; an empty or docstring-only `__init__.py` is fine, but deleting it risks tooling and import edge cases.

## 3. Architectural Guidance

- Use **inheritance** for core system types and **composition** for diagrams and optional behaviors.
- Keep the readable modeling path clean; isolate optimization in `compile/` and `simulation/`.
- Support JAX when it stays clean; use specialized JAX paths when needed instead of complicating the main path.
- **KISS and thin surfaces**: Prefer fewer files, fewer lines, and fewer dependencies when a simpler design is enough. When complexity is justified, keep **user-facing scripts and examples** minimal and push mechanics into backend modules or utilities.

### 3.1 JAX vs NumPy coexistence (five rules)

`pip install minilink` (no extras) gives a fully working **NumPy pipeline**;
`pip install minilink[jax]` unlocks JAX-traced plants, JIT rollouts, and
analytic gradients. Five rules keep that contract honest:

1. **Plants and costs use twin classes (NumPy + `Jax<X>` subclass).** The JAX
   class lives in the **same module** as the NumPy class, **subclasses** it,
   and overrides only the equation methods (`H`, `C`, `B`, `g`, `d`, `f`,
   `h`, custom force terms). Geometry, parameter dataclasses, port labels,
   bounds, and visualization are inherited. References:
   `JaxDynamicBicycle(DynamicBicycle)`, `JaxCartPole(JaxMechanicalSystem)`,
   `JaxMechanicalSystem(MechanicalSystem)`, `JaxPendulum(NumpyPendulum)`,
   `JaxQuadraticCost(QuadraticCost)`.
2. **Backend roles use one ABC + two implementations.** Evaluators
   (`compile/evaluators/{numpy_evaluator.py, jax_evaluator.py}` behind
   `evaluator.py`) and trajectory-optimization transcriptions
   (`direct_collocation.py` / `jax_direct_collocation.py`, etc.) follow this
   shape. Backend selection goes through the constants in
   `minilink.compile.backend_policy`; do not hard-code `"numpy"` /
   `"jax"` / `"auto"` / `"direct"` strings elsewhere.
3. **Thin helpers and blocks use array-module dispatch.** Code in
   `core/blocks/*` and other small math helpers writes one body and
   dispatches on the runtime input type via
   `minilink.compile.jax_utils.array_module(t)`. Do **not** use
   array-module dispatch inside complex plant `f`; plants get the twin-class
   treatment.
4. **Optional-import policy is two patterns, no more.** In library code,
   either call `require_jax_numpy()` (or `require_jax_backend()` from
   `compile.backend_policy`) lazily inside JAX-only methods, or use
   `array_module(...)` for hybrid code. Never `import jax` at module level
   in `minilink/`. Module-level `try/except ImportError + _HAS_JAX` is
   reserved for **tests** and **runner scripts**, plus the documented
   exception in `minilink/planning/trajectory_optimization/benchmark.py`
   (its public contract is "skip JAX rows when JAX is missing").
5. **Backend strings live in one place.** `BACKEND_NUMPY`, `BACKEND_JAX`,
   `BACKEND_AUTO`, `BACKEND_DIRECT`, `normalize_backend(...)`, and
   `require_jax_backend()` are defined in `minilink.compile.backend_policy`.
   The `JaxQuadraticCost` rule is documented and enforced by
   `minilink.core.costs.require_jax_traceable_cost`. JAX trajopt
   transcriptions delegate; they do not re-implement either rule.

**When to add a `Jax<X>` twin.** New plants default to **NumPy-only**. Add a
JAX subclass when a concrete use case needs traceable dynamics: JAX trajectory
optimization (gradients through `f`), simulator use of `jit` / batched rollouts,
or differentiable physics. Calling `compile(..., backend="jax")` on a NumPy plant
already uses the JAX evaluator and does **not**, by itself, require a separate
`Jax<Plant>` class.

**Non-goals.** No global NumPy/JAX toggle (prefer explicit `compile_backend` on
call sites); no replacing hand-written twins with codegen or AST tooling; no
typing layers whose only purpose is to abstract twins; no new top-level
`minilink.jax` package. Larger roadmap items (e.g. scan rollouts, more catalog
twins) live in `ROADMAP.md`.

**Verification.** A **NumPy-only** install must run `pytest tests/unittest/`
with JAX-marked tests **skipped**, not failing at **collection**. With
`minilink[jax]`, those tests run.

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

**JAX twin plants** must additionally include a canonical
`<plant>_jax_matches_numpy` test pair: one nominal / linear-regime
match and one non-trivial regime check (custom inertia, gravity, damping,
or a saturating force) so the JAX trace and the NumPy reference disagree
at most by ULP-level rounding (with `configure_jax(enable_x64=True)`).
References: `tests/unittest/test_jax_dynamic_bicycle.py`,
`tests/unittest/test_jax_pendulum_subclass.py`,
`tests/unittest/test_mechanical_jax.py::test_f_matches_numpy_2dof_nontrivial_regime`.

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
| Auto `rk4_fixedsteps` selection (long uniform grid + JAX + non-stiff) | **TRL 4–6** | Heuristic; fixed-step RK4 now supports nominal and forced rollouts, but auto-selection policy should still stay conservative. |
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
- removing, relocating, or “cleaning up” **user-authored convenience code** the user added locally (for example an `if __name__ == "__main__":` quick test in a module, a scratch print, or a temporary debug hook), unless the user explicitly asked to delete or replace it

Demo and manual scripts should stay flat and directly runnable at module top level.

Benchmark example scripts live under `tests/benchmark/` (same flat style). Import benchmark helpers from subsystem-local `benchmark.py` modules such as `minilink.compile.benchmark`, `minilink.simulation.benchmark`, or `minilink.planning.trajectory_optimization.benchmark`; see `DESIGN.md` §4.6.

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
