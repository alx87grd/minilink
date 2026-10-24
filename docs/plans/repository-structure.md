# The long-term shape of the repository

**Status:** ideas to review, not decided — options and a recommendation, 2026-10-20.  
**Lane:** maintainer-owned (student-facing paths and public names; AGENTS.md "Ask first").  
**Rung:** serves the open decision "Where the course material and the projects live"
([ROADMAP §6](../dev/ROADMAP.md#6-decisions), steps K5 / K6 of [TODO.md](TODO.md)); the
package moves it names are existing steps (A3, C4, V3).  
**Constraint:** the course notebooks' names are frozen until v1.0, with the freeze paused
between terms (ROADMAP §4.1 gate 7); any move happens **between terms only**.

Three questions, decided mostly independently: **A** how the package splits into modules,
**B** how the examples tree is laid out, **C** where the teaching material lives. Each
section states the facts, the options with what each changes, and one recommendation. The
recommendation is a position to argue with, not a ruling; §5 lists what the maintainer has
to decide.

## 1. Why now

Minilink has a written vision (the constitution, the two lanes, the path to v1.0) and more
shipping features than when the folders were laid out. Three things no longer fit:

- **The examples tree carries four axes on one tree.** By *form* (`demos/` scripts vs
  notebooks), by *audience* (`tutorial/` API walk, `teaching/topics/` textbook pages,
  `teaching/courses/<id>/` homework pins), by *package* (`demos/<chapter>/` mirrors the
  library) and by *lane* (`projects/`, `experimental/`). One lesson exists three times
  (the pendulum value-iteration-vs-LQR-vs-RL comparison is a topic notebook, a GRO860
  notebook and a demo script), and the reader needs the README table to know which copy is
  "the" one.
- **The package has hot spots and stubs.** `core/` is 8.8k lines, with `composition.py`
  (1.3k), `facades.py` (1.2k) and `wiring.py` (0.9k); `planning/` is a 16k-line umbrella over
  five subpackages of different kinds (geometry, graph search, trajectory optimization,
  dynamic programming, reinforcement learning); `identification/` is a five-line stub;
  `catalog/` is a 135-line alias layer; `graphical/` is the largest package at ~13k lines
  over six subpackages.
- **The course notebooks pin no library version.** Every Colab cell runs
  `git clone https://github.com/alx87grd/minilink` (one adds `-b main`), so a push to `main`
  can break a running course the same afternoon. Releases exist on PyPI and as GitHub tags
  since `0.1.1`, so a pin is available and unused.

## 2. The package (A)

### 2.1 Facts

| Band | Packages | Lines (approx.) |
| --- | --- | --- |
| framework | `core` + `core/compile` | 8.8k + 1.0k |
| system libraries | `blocks`, `dynamics` (abstraction + catalog), `control` (+ `mpc`), `estimation` | 1.5k, 9k, 2.8k + 1.7k, 0.3k |
| tools | `simulation` (+ `realtime`, `solvers`), `analysis`, `planning` (five subpackages), `optimization`, `identification` | 2.4k + 1.4k, 4.3k, 16k, 1.4k, 5 lines |
| presentation | `graphical` (six subpackages), `interfaces` | 13k, 0.5k |
| other | `catalog` (aliases), `experimental` | 135 lines, 2.5k |

The top level already reads as a textbook's table of contents; the trouble is inside
`planning/` and `core/`, and at the two small packages that say less than they promise.

### 2.2 Options

**A1 — keep the four bands, fix the hot spots.** No top-level change. Targeted moves, each
its own small step, most already on the workboard:

- `planning/spatial/` → `core/geometry/` ([geometry-module.md](geometry-module.md), step
  A3): sets and posed geometry are core objects a controller may import; the placement test
  there ("would you still import it with no `PlanningProblem`?") decides.
- Landed 2026-10-24 ([the run's log](../reviews/2026-10-24-v03-run-decisions.md) §10):
  `core/composition.py` split into `operators.py` and `closed_loop.py`, `planning/engine.py`
  into `evaluator.py` and `evaluation.py`, the discrete-time pieces moved to `core/discrete/`
  and `simulation/discrete/`, `core/compile/` to one evaluator class per system kind, and
  `core/facades.py` into `core/facades/` by what the shortcuts do.
- `wiring.py`: split by concern when the
  differentiable-problem work ([differentiable-problem.md](differentiable-problem.md), V1)
  touches them anyway; no public name changes, so it is agent-managed internal structure.
- `identification/`: remove the stub until `fitting.py` lands (C4), or keep it with a
  docstring that says what it will hold; a five-line package misleads a reader of the tree.
- `catalog/`: keep the alias layer until v0.9, then decide with the frozen-surface step (V3)
  whether `from minilink.catalog import Pendulum` or `minilink.dynamics.catalog` is the
  student path; one of the two is enough.
- `experimental/`: stays the research lane's package; the repo-contract test already keeps
  it out of the teaching surface.

Pros: zero churn for students and the course notes; every move is a workboard step.
Cons: `planning/` stays a wide umbrella; `graphical/` stays the biggest package.

**A2 — textbook-chapter split of the tools band.** Split by *what the student learns*:
`planning/` keeps the problem objects and the open-loop planners (search, trajectory
optimization) and dynamic programming; a new `learning/` takes reinforcement learning and,
later, imitation and system identification; `control/mpc` moves next to trajectory
optimization (MPC is receding-horizon trajopt).

Pros: the tree then matches the course ladder (modelling → control → planning → learning).
Cons: the RL planner is a planner by design (the constitution's one solution object); a
`learning/` package re-opens that argument and renames a public path during GRO860's freeze.
Not before v1.0, if ever.

**A3 — flatten to four literal packages:** `minilink.systems.{blocks,dynamics,control,
estimation}`, `minilink.tools.{simulation,analysis,planning,optimization}`, `core`,
`graphics`.

Pros: the band is visible in every import path; nothing can be misfiled.
Cons: every deep import in notebooks, docs and the Sphinx API breaks, while the prelude hides
the path anyway (the notebooks import from `minilink` itself 78 times and from a subpackage a
handful). High cost, low visible gain.

### 2.3 Recommendation

> **Reopened 2026-10-23** (the maintainer): `planning/` is to be regrouped and breaking names is
> accepted, so A2's objection (the name freeze) no longer holds. The proposal and the open choice are
> workboard row K10 (TODO §4).

**A1.** The top level says the right thing; the churn budget goes to the hot spots and to
B and C. Revisit A2 only if a `learning/` chapter (imitation, identification, RL together)
becomes a course in its own right.

## 3. The examples tree (B)

### 3.1 Facts

| Folder | Content |
| --- | --- |
| `tutorial/` | 12 numbered API chapters (`00_core` … `11_reinforcement_learning`) + 3 showcases |
| `teaching/topics/<domain>/` | 8 notebooks in five domains |
| `teaching/courses/udes_gro860/`, `udes_gro501/` | 14 + 6 notebooks, cited by the course documents |
| `demos/<chapter>/` | scripts in 17 chapters: the package names plus `rrt`, `hybrid`, `mpc`, `realtime`, `robotic`, `udes_racecar` |
| `projects/` | 3 multi-file applications |
| `experimental/<topic>/` | 8 topics of prototypes and scratch |

### 3.2 Options

**B1 — keep the six kinds, tidy the names.** Rename the demo chapters to the tutorial
numbering (`demos/09_planning/` rather than `rrt`, `trajopt`, `value_iteration`); name one
copy of each duplicated lesson canonical (the topic) and the other a pin (the course).

Pros: the smallest change. Cons: the four axes stay, and the README table stays the only map.

**B2 — two axes only: chapter and course.**

```
examples/
  <NN>_<chapter>/            # one folder per textbook chapter, in tutorial order
    README.md                # the chapter index: what each file teaches, in reading order
    <chapter>.ipynb          # the tutorial notebook (today's tutorial/NN_<chapter>.ipynb)
    <plant>_<method>.py      # the canonical demos (today's demos/<chapter>/*.py)
    <plant>_<method>.ipynb   # the textbook lessons (today's teaching/topics/<domain>/*.ipynb)
  courses/<id>/              # live course pins; paths inside unchanged (see C)
  research/
    projects/<name>/         # multi-file applications (today's projects/)
    experimental/<topic>/    # prototypes and scratch (today's experimental/)
```

Three rules replace the README table: **a demo is a script, a lesson is a notebook, the
chapter notebook is the chapter index**; `courses/` holds pins only (a course notebook is a
copy of a chapter lesson, never the only copy); `research/` is outside the release
contract. The current "no `.ipynb` and `.py` in one leaf" rule goes, since the chapter
README carries the form. Promotion becomes one arrow: `research/` → a chapter.

Pros: one place per lesson, and the tree *is* the textbook. Cons: every demo and topic
notebook moves (`git mv`, history kept); `tests/demo_checks` and the nightly sweep paths
change; the chapter list has to be agreed, and `hybrid`, `mpc`, `realtime`, `robotic` and
the racecar plant folder need a chapter each.

**B3 — notebook-first with jupytext pairing.** Every example is a notebook; the scripts
are generated twins (`.py:percent`), and diffs happen on the `.py`.

Pros: solves script-vs-notebook once; reviewable diffs. Cons: a new tool in the student
path and in CI, and the demos' value is being short scripts a student reads top to bottom
(RULES 6.1, 6.10); a build step on the one thing that should have none.

**B4 — course-first: the tree is the syllabus** (`examples/<course>/<week>/`). Rejected:
the library is course-neutral by the constitution (v1.0: "a colleague can build a course
on it"); the course axis belongs in `courses/<id>/` and nowhere else.

### 3.3 Recommendation

**B2**, staged between terms: (1) agree the chapter list and write the three rules in
`examples/README.md`; (2) move `demos/<chapter>/` and `teaching/topics/` into the chapters,
one commit per chapter, `git mv`; (3) leave `courses/` paths untouched (the GRO860 freeze);
(4) `projects/` and `experimental/` under `research/`. B1's renames are a fallback that
keeps the option open if B2 has to wait a term.

## 4. Where the teaching material lives (C)

### 4.1 Facts

The course notebooks are cited by the GRO860 notes and webpage and by GRO501's `\colab{}`.
Forty-two notebooks clone the default branch unpinned. The co-evolution of the API and the
notebooks in one checkout is what made v0.1 and v0.2 fast: a rename migrated the notebooks
in the same commit, and a notebook that needed a feature got it the same day.

### 4.2 Options

**C1 — one repo, as today.** Pros: one checkout, one CI, renames migrate the notebooks in
one commit. Cons: a push to `main` can break a running course; the library repo carries
forty notebooks and their smoke checks; a course term and a library release are coupled.

**C2 — two repos, `minilink` and a course repo pinned on a release.** The course repo's
Colab cell does `pip install minilink==0.2.0` (or clones the tag); one term pins one
release; the library moves freely. Pros: a running course cannot break; the library repo is
only the library; the course repo can hold solutions, slides and grading scripts that have no
place in a library. Cons: two CIs; a notebook that needs a new feature waits for a release;
a rename needs a second commit in a second repo, after the release; co-evolution ends.

**C3 — one repo, pinned Colab cells, drift checked nightly.** Everything stays here, but
the **course** notebooks' install cell pins the release the course runs on
(`pip install minilink==0.2.0`, or `git clone -b v0.2.0` when the notebook needs the
examples tree), while the tutorial and the chapter lessons keep tracking `main`. The
nightly workflow also runs the course notebooks against `main` and reports drift without
failing the merge gate; the term-boundary step bumps the pin. Pros: a running course is safe
from `main` today, for one line per notebook; co-evolution stays for everything that is not
a live course; the pin discipline is exactly what C2 needs, so the move stays open and
cheap. Cons: the course copies carry a version string to maintain; a pinned course notebook
cannot use a feature newer than its pin (which is the point).

**C4 — the teaching material as its own installable package.** Rejected: nothing in the
notebooks is a library, and packaging them adds a release process for prose.

### 4.3 Recommendation

**C3 now, C2 at v1.0.** This keeps ROADMAP §6's "move between terms only" and matches
v1.0's promise ("a colleague can build a course on it"): by then the course repo is the
template an outside instructor forks. The pin is also the first concrete piece of the trust
contract (pinned installs, step V5). Whether `projects/` follows the courses or stays as
research material is a separate call (§5).

## 5. The combined path, if the recommendations are taken

| When | Package (A) | Examples (B) | Teaching material (C) |
| --- | --- | --- | --- |
| now (docs and one-line changes) | drop or document the `identification/` stub | write the three rules and the chapter list in `examples/README.md` | pin the course notebooks' install cell on `0.2.0`; the nightly drift run |
| between terms (after GMC714) | `planning/spatial/` → `core/geometry/` (A3); the `core/` splits ride V1 | the chapter moves (B2), one commit per chapter; `research/` | bump the pin to the term's release |
| v0.9 | the `catalog/` alias decision with the frozen surface (V3) | — | — |
| v1.0 | — | — | the course repo (C2), forked from `courses/` with history |

## 6. The asks

1. **The chapter list** for B2: the twelve tutorial chapters as the spine, and a home for
   `hybrid`, `mpc`, `realtime`, `robotic` and the racecar plant folder (a chapter each, or
   folded into `05_simulation`, `09_planning`, `02_dynamics`).
2. **The pin form** for C3: `pip install minilink==<tag>` (fast, no examples tree) or
   `git clone -b v<tag>` (keeps the notebooks' relative imports of the examples tree).
3. **`projects/`**: research material that stays (`research/projects/`), or course material
   that moves with the course repo at v1.0.
4. **The `identification/` stub**: remove now, or keep with a docstring until C4.
5. Whether A2's `learning/` package is worth a row in ROADMAP §7 (out of scope) so the
   question stays closed.

When the maintainer rules, the ruling goes to a dated record under
[../reviews/](../reviews/) and the steps to [TODO.md](TODO.md); the open decision then
leaves ROADMAP §6 (one fact, one home: [README.md](README.md)).
