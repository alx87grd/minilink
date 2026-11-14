# The examples tree and the teaching material

**Status:** ideas to review, not decided — options and a recommendation, 2026-10-20. The
package question (A) is settled: `minilink.planning` regrouped by pillar and reinforcement
learning moved to `minilink.learning`, the `identification/` stub removed, and the package folders restructured on 2026-10-24. Still open,
and maintainer-owned: the examples tree (B, §3) and where the course material lives (C, §4).
§7 holds the plan of 2026-11-13 for the repositories and the teaching one, ruled and partly
done the same day (§7.0): the course material (C) is decided; the examples tree (B) is not.  
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
  library) and by *lane* (`projects/`). One lesson exists twice (the pendulum
  value-iteration-vs-LQR-vs-RL comparison is a GRO860 notebook and a demo script; its topic
  copy went in v0.4), and the reader needs the README table to know which copy is "the" one.
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
| `tutorial/` | three layers, `basics/` (3), `core/` (8), `advanced/` (5), + 1 showcase |
| `teaching/topics/<domain>/` | 5 notebooks in three domains (two duplicates of GRO860 notebooks deleted in v0.4) |
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
    <chapter>.ipynb          # the tutorial notebook (today's tutorial/<layer>/NN_<chapter>.ipynb)
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
Colab cell does `pip install minilink==0.3.0` (or clones the tag); one term pins one
release; the library moves freely. Pros: a running course cannot break; the library repo is
only the library; the course repo can hold solutions, slides and grading scripts that have no
place in a library. Cons: two CIs; a notebook that needs a new feature waits for a release;
a rename needs a second commit in a second repo, after the release; co-evolution ends.

**C3 — one repo, pinned Colab cells, drift checked nightly.** Everything stays here, but
the **course** notebooks' install cell pins the release the course runs on
(`pip install minilink==0.3.0`, or `git clone -b 0.3.0` when the notebook needs the
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
| now (docs and one-line changes) | write the three rules and the chapter list in `examples/README.md` | pin the course notebooks' install cell on `0.3.0`; the nightly drift run |
| between terms (after GMC714) | the chapter moves (B2), one commit per chapter; `research/` | bump the pin to the term's release |
| v1.0 | — | the course repo (C2), forked from `courses/` with history |

## 6. The asks

1. **The chapter list** for B2: the three tutorial layers as the spine, and a home for
   `hybrid`, `mpc`, `realtime`, `robotic` and the racecar plant folder (a notebook each, or
   folded into the existing layers).
2. **The pin form** for C3, ruled 2026-10-25: `git clone -b <tag>`, the tag as published
   (bare: `0.3.0`, no `v`) (keeps the
   notebooks' relative imports of the examples tree), not `pip install minilink==<tag>`.
3. **`projects/`**: research material that stays (`research/projects/`), or course material
   that moves with the course repo at v1.0.

When the maintainer rules, the ruling updates this doc and the steps go to
[TODO.md](TODO.md) in the same change; the commit message carries its why (one fact, one
home: [README.md](README.md)).

## 7. The repositories and the teaching repo's layout (working plan, 2026-11-13)

The maintainer's preferred direction so far, from the v0.4 consolidation discussion. It moves
the course material out sooner than §4.3 proposed. Names are candidates until the
repositories exist.

### 7.0 Ruled and done (the maintainer, 2026-11-13)

- **The course and topic notebooks moved** to the public repository `alx87grd/teaching`, on
  `dev-consolidation`, with `examples/teaching/` deleted here and the checks that named it.
- **Sorted by topic, not by course** (supersedes §7.2's `courses/<id>/` folders): `python/`,
  `dynamics/`, `control/`, `optimal_control/`, `dynamic_programming/`,
  `reinforcement_learning/`, `machine_learning/` (`planning/` with its first notebook). A course is a list,
  `courses/<course>.md`, linking the notebooks its notes cite in the notes' order; a notebook
  on an active course's list is not renamed or moved during the term.
- **The bleeding edge until 0.4.0:** copied from `dev`, not `main` (supersedes §7.3 step 1);
  the install cells clone minilink's `dev`. Nothing links to the repository yet.
- **Its own CI:** `tests/run_notebooks.py`, one job per course on every push, every notebook
  each night.
- **Left, all at once after the fall term:** `dev` merged into `main` and `0.4.0` tagged; every
  notebook pinned to `0.4.0`; the course notes' links moved to the new repository.
- The private side went to `alx87grd/workbench` (log, reference, prototypes, private course
  material), which replaces `minilink-private` in §7.1.

### 7.1 Four repositories, one question each

| Repository | Visibility | Holds | Rule |
| --- | --- | --- | --- |
| `minilink` | public, exists | the library, the tutorials, the curated demos, tests, docs | depends on nothing else of the maintainer's; every file is one to show a stranger |
| `teaching` (candidate name) | public, to create | the course notebooks and free-standing teaching demonstrations, mostly on minilink but not limited to it | every notebook pins a released tag; a course folder changes between terms only |
| `minilink-lab` (candidate name) | private, to create when a student needs it | research projects, one folder each; the experimental package | each project pins a tag or a commit; a feature that matures is rewritten into `minilink`, never imported from here |
| `minilink-private` | private, exists | the maintainer's sandbox: pet ideas, old demos worth a second look | no rules; nothing imports from it |

The sorting question for any file: is it for anyone, for my students, for my lab, or for me?
Dependencies flow one way: the other three depend on `minilink`, never the reverse. Until
`minilink-lab` exists, the research projects can sit in `minilink-private` under `projects/`.

Promotion goes `minilink-private` → `minilink-lab` → `minilink`, and code is rewritten on the
way up, not moved.

### 7.2 The teaching repository: name and top level

The rule that avoids a path such as `control-notebooks/control/`: the repository's name says
what kind of thing it is, the folders say which subject, and no word appears twice in a path.

**Preferred: `teaching`, subjects at the top level.**

```
teaching/
  README.md            the index: one line per notebook, with its Colab badge
  courses/
    gro860/
    gro501/
    gmc714/
  dynamics/            modelling, simulation, phase plane
  control/             PID, root locus, Bode, state feedback, observers
  optimal-control/     dynamic programming, LQR, trajectory optimization, MPC
  planning/            RRT, search
  learning/            Q-learning, policy gradient, PPO
  robots/              arms: kinematics, impedance, computed torque
  vehicles/            bicycle models, tires, the racecar
```

Two rules keep it tidy:

- **A course folder holds only what its PDF links to.** A course notebook that is really a
  topic notebook lives in the topic folder and the course's README links to it: one copy of
  each notebook.
- **A notebook goes where its main idea is, not its plant.** LQR on a cart-pole is
  `optimal-control/`, not a `cartpole/` folder.

Keep the top level to about eight folders.

**Alternatives considered:**

| Name | Top level | For | Against |
| --- | --- | --- | --- |
| `robotics-notebooks` | `courses/`, `modeling/`, `feedback/`, `estimation/`, `optimal-control/`, `planning/`, `learning/`, `systems/` | clear to a stranger | "robotics" undersells GRO501's classical control |
| `notes-code` | one folder per set of course notes, by chapter, plus `extras/` | a student goes from the PDF's chapter to the code | ties the layout to documents that also change |
| `by-example` | `courses/`, `dynamics/`, `control/`, `planning/`, `learning/`, `robots/` | paths read like a sentence | vague in a list of repositories |

Other names on the list: `control-notebooks`, `control-by-example`, `teaching-notebooks`,
`course-notebooks`, `dynamics-and-control`, `phase-space`, `cost-to-go`. Avoid a course code or
`minilink` in the name: the content outlives both.

### 7.3 The move, without touching a running course

Checked 2026-11-13: every course link and Colab badge points at the `main` branch
(`…/minilink/blob/main/examples/teaching/…`), and `main` is the tag `0.3.0` plus one README
commit. So the notebooks can leave on `dev` now; students see nothing until `main` moves.

1. Create the teaching repository and copy the notebooks **from `main`** (they run on `0.3.0`;
   the `dev` versions differ in 22 files and wait on a branch for the v0.4 tag).
2. Pin every install cell on the tag (§7.4), and check each notebook in Colab from its new
   address.
3. Remove `examples/teaching/` on a `dev` branch, with the checks that name it.
4. Update the links of the course PDFs and pages; the new repository is live, so this can be
   done at any time.
5. Push to `main` after the term.

Leaving the library's CI is the cost: a rename no longer fails on a course notebook. The
replacement is a check in the teaching repository that runs every notebook against its pin.

### 7.4 The pin

A **tag**, never a branch (it moves) and not a commit hash (unreadable for a student; fine for
a research project that needs an unreleased state). Tags are bare (`0.3.0`, no `v`), published
to PyPI by the publish workflow: `0.1.0`, `0.1.1`, `0.2.0`, `0.3.0` are on PyPI. A release tag
is never moved; a fix is a new tag and one commit bumping every notebook of the course.

**Open, for the maintainer:** the form of the pin in the new repository. The ruling of
2026-10-25 (§6, ask 2) chose `git clone -b <tag>` over `pip install minilink==<tag>` because
the notebooks import the examples tree by relative path. In a repository of their own that
reason holds only for notebooks that still need files of `examples/`; for the others
`pip install -q minilink==0.3.0` is shorter and faster. Two facts to check per notebook: does
it import from the examples tree, and does it use `minilink.experimental` (not in the wheel;
`ode_simulation.ipynb` of GRO501 mentions it)?

