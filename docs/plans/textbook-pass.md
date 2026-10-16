# The textbook pass, second run

**Status:** draft for the maintainer, 2026-10-16. The run starts once §10 is approved.
**Serves:** workboard steps TB-b, T1–T7 and the rename remainder ([TODO.md](TODO.md) §2–§3),
and the style half of S68 (the ratchet over the whole library).
**Base:** `dev`, pinned at the run's start (today `8b4c4d5`). **Run branch:**
`claude/textbook-pass-dev`. **Probe branch (never merged):** `claude/textbook-pass-probes`.
**Example, not merged:** `claude/vigilant-hopper-poekls`, the first run, made on `main` by
mistake. This plan treats it as a dry run (335 commits, its record at
`docs/reviews/2026-10-15-textbook-pass.md` on that branch). Its modules are examples;
its history is the evidence for §2.

The goal does not change. Every module reads like the textbook page (`dp.py` and
`control/design.py` are the references), behaviour is preserved byte for byte, and the
public surface stays intact.

---

## 1. The maintainer's choices (2026-10-16)

| Topic | Choice |
| --- | --- |
| Staffing | One orchestrator and at most **3 writer agents** at a time, each on its own files. One read-only reviewer per wave. Peak: 4 agents. |
| Bugs | **Recorded, not fixed.** |
| Backend | **No backend changes.** Byte-identical on NumPy and on JAX alike. Every array-in-array-out opportunity is recorded, none is made (the dry run's §3 changes are not repeated). |
| Removals | A public name that is **redundant**, with no capability a demo would use going with it, and **provably unreachable** branches. Each one is counted afresh on `dev` and listed in the PR. |
| T6 | `core/composition.py` and `control/mpc/controller.py` run during the night at the same bar as any other module. |
| Scope added | `planning/spatial/` (no longer waiting for A3), and the **library-like modules of `examples/projects/`** (models, laws, `common/` helpers). Scripts and notebooks keep their tuning constants, commented-out calls and exploratory locals; they take renames, markers and titles only. |
| Scope kept out | The animation renderers, `core/compile/`, `interfaces/` and `simulation/realtime/` (exempt tier); `examples/experimental/`; the naming of `tests/` and `benchmarks/`. |
| Probes | Out of the repo, on the probe branch, so the morning review can rerun them. |
| Failures | A module still failing its checks after 2 fix attempts is reverted, logged, and skipped. After a usage-limit pause the run resumes itself (§7.6). |
| Delivery | The run branch and **one PR into `dev`** in the morning, one commit per module grouped by step. A merge commit is recommended over a squash so the per-module history stays. |

## 2. What the dry run taught

| Lesson | Evidence | Change in this run |
| --- | --- | --- |
| Check the base first. | The clone tracked only `main`; a day of work went onto the wrong base. | `git fetch --all --prune` before the run, at every wave and before every push. The base is pinned and written in the PR. |
| Parallel writers invent their own styles. | 181 commits by nine writers, then 128 commits only to make them agree: markers in 44 modules, banners, titles, quoted annotations, `x_{k+1}`, verb families, copy comments, padded delegations. Two writers wrote the same change. | The style sheet (§3) is ruled before any module. Writers are capped at 3 and own disjoint files. Every style item a script can see is checked at each commit (§4.1). A reviewer reads each wave before it merges. |
| The probes must be one instrument. | Three fallback-lookup styles, four refusal recorders, order-dependent caches. 182 probes could not run on the old tree until a rename shim was improvised. `planning/spatial/` had no band. | One package, written once by the orchestrator (§4.2). One encoder, one recorder, one name resolver from the start, a band per package, and a coverage report. |
| Semantic slips hide in big diffs. | A narrowed condition in `core.feedback.error_input` surfaced only in the final audit. | An audit at every commit against the pinned base: literals, raises, attribute strings, signatures, API surface (§4.3). |
| Consolidation came last. | The duplicates were found only by a separate simplicity round. | The duplicate scanners run before each wave. Each lane's queue says which modules share logic, and the consolidation happens in that module's commit. |
| Rulings came mid-way. | The array-kind and dead-code rulings arrived after the work had started. | All choices are in §1 before the start. |
| A dead-code call depends on the base. | `riccati_transition` was unused on `main`; on `dev`, `control/design.py` calls it. | No removal is carried over from the dry run without a fresh count on `dev`. |
| The equation paths had no fixed shape. | The dry run packed six unpack lines together, grouped some and not others, built arrays on the `return` line, and moved `xp` around. | One fixed pattern of blank-line-separated blocks (§3.2), one parameter per line, simple arrays on their equation line, complex ones by named entries; the ratchet flags arrays built on `return` and unnamed complex matrices (§4.1). |
| Several commits per module. | T, TR and TS commits on the same module. | One commit per module, at the full bar. A later commit only for a cross-lane ripple (§7.4). |
| Tools must refuse, not tidy. | The commit script regenerated the allowlist. | The commit script refuses an allowlist that grows. |
| Sessions get interrupted. | Usage limits stopped the work twice; the worker restarted twice. | The ledger (§7.5) is the run's memory, every commit is pushed, and an hourly check-in resumes a stalled run. |

## 3. The style sheet

Every module of the pass meets all of it in its one commit. RULES §5 is the law; this
section settles what the law leaves open, so that no two modules decide differently.
Writers receive this section verbatim.

### 3.1 Module shape
- Line 1 is a one-line title naming the domain, not the process:
  `"""Discretization: a continuous-time system as a discrete-time step model."""`.
  A longer note moves to the class or function whose contract it describes (5.23).
- The order: imports; the module constants the primary code reads, next to their owner's
  section; `# Public API`; the primary class or function first, within the first 60 lines
  (5.9); then `# Internal machinery`, with the helpers in the order the body calls them
  (5.10). A class with helpers takes its own `# Internal machinery` title after its public
  methods.
- **The two section titles stand out.** `Public API` and `Internal machinery` sit between
  two full lines of `#`, with two blank lines around them. Inside a class, the banner is
  indented with the class body. Smaller sub-sections are plain sentence-case comments
  (`# Frequency grid`).

  ```python
  _2PI = 2.0 * math.pi


  ###############################################################################
  # Public API
  ###############################################################################


  def shortest_path(q0, q1, radius):
      ...


  ###############################################################################
  # Internal machinery
  ###############################################################################


  def word_segments(a, b, d):
      ...
  ```
- **Air:** blocks are separated by blank lines, never packed together.
- A constant has one owner; the other modules import it. There is no lazy import of a
  minilink module, except to break a real cycle or for an optional dependency, with a
  comment saying which.

### 3.2 Equation paths and algorithms (5.3)
- **One pattern for every equation path and algorithm body.** The blocks below appear in
  this order, each separated by a blank line, and any block with nothing to say is left out:
  1. `params = self.params if params is None else params`, then on the next line the
     backend: `xp = array_module(...)` (or `jnp = require_jax_numpy()` in a JAX-only
     algorithm).
  2. The parameters and the object's fields, **one per line**.
  3. The pre-computation: the state and input unpacked (`theta = q[0]`), trigonometric
     shorthands, intermediate quantities.
  4. The core equation(s), each a named line, with its equation comment above when the
     code cannot write the textbook form (5.22).
  5. The matrix assembly, when the result is a complex array (below).
  6. `return` of a name.
- **Assembly.**
  - A simple array (a vector or a matrix with short entries) is built on its own equation
    line: `g = xp.array([m * gravity * l * xp.sin(theta)])`.
  - A complex array gets one named line per entry (`J11`, `J12`, …) in block 4, then its own
    assembly block, a matrix row by row (5.5).
  - The array is never built on the `return` line.
- No `self.` in a math line. Linear algebra is part of the story and stays in the body.
  Plumbing goes to helpers: coercion, shapes, defaults, validation, dispatch, caching,
  wrapping.

  ```python
  def g(self, q, params=None):
      params = self.params if params is None else params
      xp = array_module(q)

      m = params["m"]
      l = params["l"]
      gravity = params["gravity"]

      theta = q[0]

      g = xp.array([m * gravity * l * xp.sin(theta)])

      return g


  def C(self, q, dq, params=None):
      params = self.params if params is None else params
      xp = array_module(q, dq)

      m2 = params["m2"]
      l1 = params["l1"]
      lc2 = params["lc2"]

      _, _, _, s2, _, _ = self.trig(q)
      h = m2 * l1 * lc2 * s2

      # Coriolis and centrifugal coupling, driven by the second joint's rate
      C11 = -h * dq[1]
      C12 = -h * (dq[0] + dq[1])
      C21 = h * dq[0]
      C22 = 0.0

      # fmt: off
      C = xp.array([
          [C11, C12],
          [C21, C22],
      ])
      # fmt: on

      return C
  ```

  The constructor keeps the array module it has on `dev` (`np` or `xp`): no backend change
  (§1). On `dev` the arms' `J` reads `self.params` instead of its argument; that is a
  recorded bug, so the pass keeps the `self.params` reads exactly as they are.
- A delegation, a constant, a dict literal or a frame composition stays the one line it
  is; the beats are for equations. `tf` returns its frames as one dict literal.
- Equation comments use Unicode math in the textbook's symbols: `ẋ`, `x_{k+1}`, `Aᵀ`, `R⁻¹`,
  `σᵢ`, `≤`, `√`, `∏`, `𝒞`, `𝒪`. Identifiers stay ASCII (`x_next`, `theta`, `sigma`). A
  comment states the equation; it never copies the code line.

### 3.3 Names
- Plain names everywhere (5.8, module functions included). Underscores only for private
  module constants and local closures.
- One verb family per role:
  - `require_*` raises when a precondition fails;
  - `check_*` validates a shape and returns nothing;
  - `build_*` constructs a runtime object (evaluator, simulator, computer);
  - `*_of` resolves an object's options or attributes (`trajopt_options_of`);
  - `plot_*` draws.

  Twin helpers in sibling modules share one name.
- Textbook symbols, one meaning each:
  - `rho` is the discount rate; a probability ratio is `ratio`.
  - `tau` is a torque in mechanics and a time constant in filters, never both in one module.
  - `eta` is a step size, `eta_k` its value per update.
  - Losses are written `L_pi`, `L_V`.

### 3.4 Comments and docstrings
- Timeless (5.24): no "now", "was", dates or step ids in the code.
- No comment restates the line below it. A comment says why, or states the equation.
- A public function's docstring opens with one summary line. NumPy sections appear only
  where the parameters need them. A note on a contract lives on the object that owns it.

### 3.5 Types and imports
- Under `from __future__ import annotations`, forward references are unquoted.
- No type hints in equation paths (5.1). Hints go on constructors, public methods and
  orchestrators.
- `require_jax()` / `require_jax_numpy()`, never a bare `import jax` (5.12).

### 3.6 Backends
- `xp = array_module(...)` is the second line (§3.2), and every array operation after it
  uses `xp` (5.2), **unless the change would alter a result's backend type**. No
  backend changes (§1): an `np.` call under `xp` that decides the returned type stays,
  recorded and allowlisted.

### 3.7 One owner
- Logic written twice (identical once named alike) gets one owner. The owner is either a
  shared module helper, or a base class when one class truly is the other (the parametric
  evaluator is a `MathematicalProgramEvaluator`).
- **A textbook law is not a helper.** The law stays in the body of every method that uses
  it, even when two methods share it (the PID's `f` and `ctl`). A helper holds math only
  when it is a generic building block reused across the family or the library (the
  classical family's `filtered_rate`, `held_integral`, `clipped`).
- No class-attribute aliases (`Cl = Plane2D.Cl`).
- Plant equations stay on each plant's own page, even when two plants share them.
- Copies that differ in behaviour stay as they are, and the record names the difference.

### 3.8 What does not change
- Results, the backend type of every result, refusals (exception type and message),
  warnings, the order of side effects, defaults, public names and signatures. Removals are
  allowed only under §1, each one listed.
- The maintainer's manual edits in `examples/`, notebooks and scratch files. A research-lane
  script takes renames, markers and titles only.

### 3.9 The reference pages
- Algorithm: `planning/policy_synthesis/dp.py`. Design verb: `control/design.py`.
- From the dry run, one example per kind (paths on `claude/vigilant-hopper-poekls`):
  - a plant: `dynamics/catalog/pendulum/pendulum.py`;
  - an evaluator: `planning/policy_synthesis/policy_eval.py`;
  - an orchestrator: `simulation/simulator.py`;
  - a plot module: `graphical/port_map.py`.

## 4. Instruments (wave 0, before any writer starts)

### 4.1 The ratchet (`tests/unittest/textbook_style.py`, committed)
- **The dry run's six AST checks:** underscore, return-math, self-in-math, np-under-xp,
  first-screen, bare-jax.
- **New, from the review round's hand fixes:**
  - a module-level `Public API` or `Internal machinery` title without its `#` banners;
  - a multi-line module title;
  - a quoted forward reference under postponed annotations;
  - `x_k+1` / `x[k+1]` spellings in comments;
  - `# Internal machinery` without `# Public API`;
  - a lazy `import minilink…` without a reason comment;
  - a class-attribute alias to another class's method;
  - an array built on a `return` line in an equation path;
  - a matrix literal in an equation path with an entry holding more than one operator,
    instead of named entries (§3.2, "Assembly").
- **Tiers:** `estimation/`, `control/design.py`, `analysis/riccati.py` and `planning/spatial/`
  join the textbook tier.
- **The `dev` census today, six checks:** 435 offences in 91 files.

### 4.2 The probe set (on the probe branch)
- **Content:** one package, written once from the dry run's sixteen bands. It is rewritten
  where `dev`'s API changed:
  - the controller family (`P` / `PI` / `PD` / `PID`, the impedance pair);
  - `control/design.py` and `analysis/riccati.py`;
  - the ports `d` / `n` and the `observer=` loop;
  - the composition operators.

  The dry run's package does not even import on `dev` (`minilink.control.output` is gone).
- **New bands:** `estimation/`, the sensitivity functions, `planning/spatial/` and the
  library-like project modules.
- **Mechanics:**
  - one encoder (array bytes in hex, floats as `float.hex`, the backend kind);
  - one recorder each for refusals and for warnings;
  - a fresh seeded generator per probe and no cache shared across probes;
  - one name resolver: the plain name, then `_name`, then the run's rename table, so a
    probe still runs after the rename it covers.
- **Baseline:** the pinned base, captured twice and compared with `cmp`.
- **Coverage report:** every in-scope module that no probe imports; the list must be empty
  or justified.

### 4.3 The per-commit audit
Against the pinned base, the audit diffs:
- string and number literals;
- `raise` / `except` / `warn`;
- `getattr` / `hasattr` / `setattr` strings;
- public signatures (names, kinds, defaults);
- the runtime API surface, inheritance included;
- the facades' `__all__`.

Each difference is explained in the commit body (a rename, a consolidation, a listed
removal), or the commit is refused.

### 4.4 The commit script
1. `ruff format`, then `ruff check --fix`.
2. The module's tests (the test files that import it).
3. The probe bands that import it, compared with `cmp`.
4. The audit, then the ratchet, which refuses an allowlist that grows.
5. The commit, with the attribution lines.

The script is the only way a writer commits.

### 4.5 Docs
The Sphinx hook that keeps `# Internal machinery` members off the API pages (from the dry
run's `docs/conf.py`).

## 5. Scope and module classes

The 252 library modules on `dev` (experimental excluded) fall into five classes:

| Class | Meaning | Count |
| --- | --- | ---: |
| A | Unchanged since `main`, with a dry-run version | 106 |
| B | Changed on `dev` and in the dry run | 51 |
| C | Changed on `dev` only | 11 |
| D | Touched by neither (25 of them exempt) | 78 |
| E | New on `dev` | 6 |

A module of class A starts from the dry run's version: it is read against §3 and passed
again. The dry run's array-kind changes and its aliases are dropped, and every removal is
recounted. Classes B, C, D and E get a fresh pass, with the dry run open next to them where
it exists. Class D modules found already at the bar take no commit.

| Step | Scope | Modules | Offences | Lines |
| --- | --- | ---: | ---: | ---: |
| TB-b | `control/siso.py`, `state.py`, `design.py`, `blocks/transfer_function.py`, `dynamics/abstraction/state_space.py`, `core/feedback.py` | 6 | 5 | 1 666 |
| T1 | `core/` except `composition.py`, `feedback.py` and `compile/` | 18 | 26 | 6 221 |
| T2 | `blocks/`, `control/` except TB-b and the MPC controller, `estimation/` | 20 | 4 | 3 198 |
| T3 | `dynamics/` | 34 | 37 | 7 161 |
| T4 | `analysis/`, `simulation/` except `realtime/` | 30 | 40 | 6 360 |
| T5 | `optimization/`, `planning/` (`spatial/` included) | 75 | 158 | 15 026 |
| T6 | `core/composition.py`, `control/mpc/controller.py` | 2 | 52 | 2 047 |
| T7 | `graphical/` except the renderers | 39 | 112 | 8 308 |
| X | the library-like modules of `examples/projects/` | about 15 | — | — |

## 6. Waves and lanes

A lane is one writer's queue of modules, with its own git worktree and lane branch. Lanes in
a wave never share a file. Waves run in order; inside a wave, up to three lanes run at once.

| Wave | Lane 1 | Lane 2 | Lane 3 |
| --- | --- | --- | --- |
| 0 | Orchestrator alone: §4, the baseline, the duplicate and dead-code scans, the lane queues | — | — |
| 1 | T1: `system`, `diagram`, `wiring`, `signals`, `hybrid_diagram`, `hybrid_composition` | T1: `trajectory`, `step_rollout`, `costs`, `sets`, `geometry`, `distributions`, `kinematics`, `backends` | TB-b |
| 2 | T2 | T3 | T4 |
| 3 | T5: `optimization/`, `trajectory_optimization/` | T5: `policy_synthesis/`, `reinforcement_learning/` | T5: `search/`, `spatial/`, the planning core |
| 4 | T6: composition, then the MPC controller | T7: `catalog/`, `animation/` (except the renderers), `meshes` | T7: `signals/`, `control/`, `diagrams/`, `phase_plane/`, `port_map`, `common/` |
| 5 | Orchestrator alone: X, the rename remainder, R, C (§8), the PR | — | — |

Order inside a lane: the modules other modules import come first. A shared helper's owner
comes before its users.

## 7. Execution

### 7.1 The orchestrator
This session, running as a scripted workflow:
- It owns the run branch.
- It builds wave 0 itself.
- It starts the lanes and merges them.
- It applies the ripples.
- It runs the reviewer and the step-end verification.
- It writes the ledger and the morning report.

It never writes a module itself, except in wave 5 and for the reviewer's findings.

### 7.2 A writer agent: one agent per module
A fresh agent per module keeps its context small and its instructions identical to every
other writer's.

**Prompt template:**
- the module path and its class (A–E);
- the lane worktree path;
- §3 verbatim and the reference pages;
- for class A or B, the dry run's version of the module;
- the module's tests and its callers, as counted by the orchestrator;
- the rename table so far;
- the ripple protocol (§7.4);
- the commit script.

**What it returns:**
- the commit hash, or "no change needed", or "skipped: <reason>";
- the renames;
- the ripples it recorded;
- the removals, with their counts;
- the bugs it found;
- the questions for the maintainer.

**What it may not do:**
- touch a file outside its module and that module's own tests;
- change behaviour, or change a backend type;
- remove a name outside §1;
- edit a script's manual state;
- push.

### 7.3 Fix attempts and skips
When the commit script refuses, the writer fixes and retries, at most twice. On a third
refusal it reverts the module, records the refusal output in the ledger, and moves on.
Nothing is ever forced past the probes, the tests or the audit.

### 7.4 Ripples across lanes
A writer that renames a name used outside its lane leaves a one-line alias in the module,
`_old_name = new_name  # ripple`, and adds a line to the ripple table.

When the wave ends, the orchestrator:
1. merges the lanes;
2. renames every caller (scripted, word-bounded, limited to the importers);
3. deletes the aliases;
4. runs the full probes and pytest;
5. commits one ripple commit per renamed module.

### 7.5 The ledger
The ledger lives in the run's scratch space and is mirrored on the probe branch at each
wave. It holds one row per module (lane, status, commit, renames, removals, bugs, questions)
and one row per wave (merge commit, verification numbers).

On resume, the orchestrator recomputes what is left from the ledger and from the run
branch's commit subjects. A module counts as done only when its commit is on the branch.

### 7.6 Limits and resume
- **Concurrency:** never more than 3 writers, plus the orchestrator. The reviewer runs alone,
  between waves.
- **Hourly check-in:** a scheduled check-in into this session runs every hour. If the
  workflow has stopped (a usage limit, a worker restart), it resumes from the ledger. The
  check-in is cancelled when the PR is opened.
- **Pause:** after a usage-limit error, the run waits for the next check-in instead of
  retrying in a loop.

### 7.7 The reviewer (read-only, between waves)
One agent reads the wave's whole diff against §3 and the other lanes. Its findings are
inconsistencies, missed duplicates and §3 violations. The orchestrator applies the ones
that change no behaviour before the next wave, one commit per module touched. A finding
that would change behaviour goes into the record.

### 7.8 What the night never decides
These go into the morning report as questions, and the code stays as it is:
- a removal a demo, a notebook, a test or a facade uses;
- any behaviour or backend change;
- an edit to a script's manual state;
- anything RULES or AGENTS reserves for the maintainer that §1 did not settle.

## 8. Verification and delivery

**At every wave's end:**
- the full probe set against the baseline, byte-identical with no exception;
- full `pytest`;
- the ratchet;
- the audit summed over the wave;
- the regression gates after waves 2, 3 and 4 (AGENTS table, CI flags).

**Wave 5:**
- **The review round (R):** one consistency read over the whole pass. It should be small if
  §3 and §4 did their job.
- **The central review (C)**, against the pinned base:
  - the API surface and signature diff;
  - the facades' `__all__`;
  - the full probes;
  - an audit showing no test assertion was weakened;
  - no stale reference left in examples, tests or docs;
  - the literal, raise and attribute-string audit;
  - a readability verdict with the ranked list of what is still not simple.
- **Final checks:** the notebook check, the Sphinx build, ruff over the repo.

**The record:** `docs/reviews/<date>-textbook-pass.md`, one section per step. It holds:
- the removals;
- the bugs, the dry run's list of 43 re-checked on `dev` (some are fixed there, e.g.
  `ctl % dt @ plant`);
- the recorded array-kind opportunities;
- the inconsistencies left in place;
- the questions.

**The PR into `dev`:**
- **Body:** one section per step, the verification numbers, the base commit, and a link to
  the probe branch.
- **TODO rows:** TB-b, T1–T7 and the renames leave TODO; this plan doc is deleted.

**In the morning:** if `dev` moved during the night, the orchestrator merges `dev` into the
run branch (a merge commit; any conflict waits for the maintainer) and reruns the full
verification.

## 9. Risks

| Risk | Mitigation |
| --- | --- |
| A probe band does not fit `dev`'s API | Wave 0 rewrites it before any writer starts; the coverage report shows the gaps. |
| Writers drift in style | §3 verbatim in every prompt, §4.1 at every commit, the reviewer at every wave, at most 3 writers. |
| A semantic slip in a large module (T6) | The per-commit audit; the full probes at the wave's end; the reviewer reads T6 first. |
| A rename breaks another lane | The ripple alias keeps every lane green until the orchestrator applies the ripple. |
| A usage limit or a restart | The ledger, one pushed commit per module, the hourly check-in. |
| The night does not finish | The run continues the next night from the ledger; the PR is opened only when wave 5 ends. |
| `dev` moves during the night | The base is pinned; `dev` is merged in the morning, and conflicts wait for the maintainer. |

## 10. Approval checklist
- [ ] §1 matches the choices
- [ ] §3 is the style wanted
- [ ] §6 waves and lanes
- [ ] §7.6 limits (3 writers, hourly check-in)
- [ ] The run may push to `claude/textbook-pass-dev` and `claude/textbook-pass-probes`, and
      open one PR into `dev`
