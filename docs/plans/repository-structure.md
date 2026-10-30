# The examples tree and the teaching material

**Status:** ideas to review, not decided — options and a recommendation, 2026-10-20. The
package question (A) is settled: `minilink.planning` regrouped by pillar and reinforcement
learning moved to `minilink.learning`, the `identification/` stub removed, and the package folders restructured on 2026-10-24. Still open,
and maintainer-owned: the examples tree (B, §3) and where the course material lives (C, §4).  
**Lane:** maintainer-owned (student-facing paths and public names; AGENTS.md "Ask first").  
**Rung:** serves the decision "Where the course material and the projects live"
(steps K5 / K6 of [TODO.md](TODO.md), v0.9, rows waiting for the maintainer's go);
the pinned Colab cell waits in step V6.  
**Constraint:** the course notebooks' names are frozen until v1.0, with the freeze paused
between terms (ROADMAP §4.1 gate 7); any move happens **between terms only**.

Two questions remain, decided mostly independently: **B** how the examples tree is laid out,
**C** where the teaching material lives. Each section states the facts, the options with what
each changes, and one recommendation. The recommendation is a position to argue with, not a
ruling; §6 lists what the maintainer has to decide.

## 1. Why now

Minilink has a written vision (the constitution, the two lanes, the path to v1.0) and more
shipping features than when the folders were laid out. Two things no longer fit:

- **The examples tree carries four axes on one tree.** By *form* (`demos/` scripts vs
  notebooks), by *audience* (`tutorial/` API walk, `teaching/topics/` textbook pages,
  `teaching/courses/<id>/` homework pins), by *package* (`demos/<chapter>/` mirrors the
  library) and by *lane* (`projects/`). One lesson exists three times
  (the pendulum value-iteration-vs-LQR-vs-RL comparison is a topic notebook, a GRO860
  notebook and a demo script), and the reader needs the README table to know which copy is
  "the" one.
- **The course notebooks pin no library version.** Every Colab cell runs
  `git clone https://github.com/alx87grd/minilink` (two add `-b main`), so a push to `main`
  can break a running course the same afternoon. Releases exist on PyPI and as GitHub tags
  since `0.1.1`, so a pin is available and unused.

## 2. The package (A): settled

The package question (keep the four bands and fix the hot spots, split the tools band by
textbook chapter, or flatten to literal bands) was reopened by the maintainer on 2026-10-23
and settled by pillar. `minilink.planning` keeps the problem and its solutions with
`dynamic_programming/`, `trajectory_optimization/` (MPC moved in from `control/`) and
`search/`; reinforcement learning moved to `minilink.learning`, the home of identification
and imitation when they land; the `identification/` stub was deleted; the
catalog is `minilink.catalog`, the flat index of tested plants (step K8);
`planning/spatial/` goes to `core/geometry/` with step A3
([geometry-module.md](geometry-module.md)). The hot spots were split on 2026-10-24
(`composition.py`, `engine.py`, `facades.py`, the discrete-time pieces).
DESIGN §3 is the package map.

## 3. The examples tree (B)

### 3.1 Facts

| Folder | Content |
| --- | --- |
| `tutorial/` | 12 numbered API chapters (`00_core` … `11_reinforcement_learning`) + 3 showcases |
| `teaching/topics/<domain>/` | 7 notebooks in four domains |
| `teaching/courses/udes_gro860/`, `udes_gro501/` | 14 + 6 notebooks, cited by the course documents |
| `demos/<chapter>/` | scripts in 17 chapters: the package names plus `rrt`, `hybrid`, `mpc`, `realtime`, `robotic`, `udes_racecar` |
| `projects/` | 3 multi-file applications |

### 3.2 Options

**B1 — keep the five kinds, tidy the names.** Rename the demo chapters to the tutorial
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
(4) `projects/` under `research/` (the prototype scripts already left `examples/` for the
top-level `experimental/` folder). B1's renames are a fallback that
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

**C3 now, C2 at v1.0.** This keeps the rule "move between terms only" (TODO, K5) and matches
v1.0's promise ("a colleague can build a course on it"): by then the course repo is the
template an outside instructor forks. The pin is also the first concrete piece of the trust
contract (pinned installs, step V6). Whether `projects/` follows the courses or stays as
research material is a separate call (§6, ask 3).

## 5. The combined path, if the recommendations are taken

| When | Examples (B) | Teaching material (C) |
| --- | --- | --- |
| now (docs and one-line changes) | write the three rules and the chapter list in `examples/README.md` | pin the course notebooks' install cell on `0.2.0`; the nightly drift run |
| between terms (after GMC714) | the chapter moves (B2), one commit per chapter; `research/` | bump the pin to the term's release |
| v1.0 | — | the course repo (C2), forked from `courses/` with history |

## 6. The asks

1. **The chapter list** for B2: the twelve tutorial chapters as the spine, and a home for
   `hybrid`, `mpc`, `realtime`, `robotic` and the racecar plant folder (a chapter each, or
   folded into `05_simulation`, `09_planning`, `02_dynamics`).
2. **The pin form** for C3, ruled 2026-10-25: `git clone -b v<tag>` (keeps the
   notebooks' relative imports of the examples tree), not `pip install minilink==<tag>`.
3. **`projects/`**: research material that stays (`research/projects/`), or course material
   that moves with the course repo at v1.0.

When the maintainer rules, the ruling updates this doc and the steps go to
[TODO.md](TODO.md) in the same change; the commit message carries its why (one fact, one
home: [README.md](README.md)).
