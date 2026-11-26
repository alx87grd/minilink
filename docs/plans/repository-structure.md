# The examples tree and the repositories around minilink

**What it is:** the target tree of the user-facing folders (`tutorials/`, `examples/`,
`benchmarks/`; agreed with the maintainer 2026-11-26, the demo split still open), the
repositories the other material lives in (ruled 2026-11-13; what is left of the move), and how
each of them follows minilink: its branches, its pin, and how a fix reaches a course during a
term (§3, planned 2026-11-20, not set up).  
**Lane:** maintainer-owned (student-facing paths and public names; AGENTS.md "Ask first").  
**Where it sits:** the decision "Where the course material and the projects live" (steps K5
and K6 of [TODO.md](TODO.md), v0.9); the pinned Colab cell waits in step V6. The examples
tree is decided with the maintainer's pass over the demos
([consolidation.md](consolidation.md) §4).

## 1. The repositories (ruled 2026-11-13)

| Repository | Visibility | Holds | Rule |
| --- | --- | --- | --- |
| `minilink` | public | the library, the tutorials, the curated demos, tests, docs | depends on nothing else of the maintainer's; every file is one to show a stranger |
| `alx87grd/teaching` | public | the course and topic notebooks, sorted by topic, and the advanced examples, as notebooks; mostly on minilink but not limited to it | a course is a list, `courses/<course>.md`, linking the notebooks its notes cite in the notes' order; a notebook on an active course's list is not renamed or moved during its term |
| `alx87grd/workbench` | private | the maintainer's log, reference, prototypes and private course material; old demos worth a second look | nothing imports from it; its branches and pins follow §3: prototypes and course material on `main` and `dev`, the log frozen, each entry pinned by commit |
| a research repository (name not chosen; created when a student needs it) | private | research projects, one folder each; the experimental package | each project pins a tag or a commit; a feature that matures is rewritten into `minilink`, never imported from there |

The sorting question for any file: is it for anyone, for my students, for my lab, or for me?
Dependencies flow one way: the other three depend on `minilink`, never the reverse. Until the
research repository exists, the research projects can sit in the private one. Code is
rewritten on the way up (private → research → `minilink`), not moved.

**The teaching repository's layout:** topics at the top level (`python/`, `dynamics/`,
`control/`, `optimal_control/`, `dynamic_programming/`, `reinforcement_learning/`,
`machine_learning/`, `planning/`). A notebook goes where its main idea is, not its plant: LQR
on a cart-pole is optimal control, not a `cartpole/` folder. One copy of each notebook. Its
own CI runs every notebook (`tests/run_notebooks.py`: one job per course on every push, every
notebook each night); the cost of leaving the library's CI is that a rename here no longer
fails on a course notebook.

## 2. What is left of the move

All at once, after the fall term:

1. `dev` merged into `main` and `0.4.0` tagged. Until then the teaching notebooks clone
   minilink's `dev`, and nothing links to the teaching repository; the course links and Colab
   badges point at `main` (`…/minilink/blob/main/examples/teaching/…`), which is the tag
   `0.3.0` plus one README commit, so students see nothing until `main` moves.
2. Every notebook pinned to `0.4.0` (§3).
3. The links of the course PDFs and pages moved to the teaching repository.

## 3. Branches, pins and fixes

Planned with the maintainer on 2026-11-20. It starts with the move (§2): until then the course
links point at minilink `main` (`examples/teaching/`), and teaching `main` plays the `dev` role,
unpinned, following minilink's twin branches. At the move, minilink publishes the release,
teaching `dev` is created from `main`, `main` is pinned, and the course links move to it. The
tooling is in place (§3.6); the branches and the pins wait for the move.

### 3.1 Three levels of stability

Every piece of material follows minilink at one of three levels, and the level decides its pin:

| Level | Material | Pinned to | The pin moves |
| --- | --- | --- | --- |
| **Frozen** | workbench `log/` | one exact minilink commit | never |
| **Stable** | teaching `main` (the public links), workbench `main` (prototypes, course material) | a minilink release, from PyPI (`minilink[full]==0.4.0`) | to a patch release during a term; to a new minor release between terms |
| **Following** | teaching `dev`, workbench `dev`, twin branches | minilink `dev` (a twin: the minilink branch of its name) | with every change of minilink `dev` |

### 3.2 The branch map

| | minilink | teaching | workbench |
| --- | --- | --- | --- |
| **`main`**, stable | the latest release: tagged, on PyPI | pins that release; **the links the course notes, PDFs and web pages use** | prototypes and course solutions pinned to that release |
| **`dev`**, beta | the latest API and its beta features | follows minilink `dev`; fixed as soon as minilink `dev` changes | prototypes follow minilink `dev`; fixed as soon as possible |
| **`dev-<name>`**, alpha (`dev-alex`, a feature) | experiments, usually branched from `dev` | a twin of the minilink branch of the same name, created only when a change spans repositories; installs that branch | the same; may break |

- **Work flows one way: alpha → `dev` → `main`.** An alpha branch merges into `dev` in every
  repository it spans, together; `dev` reaches `main` with a release. A fix during a term goes
  to `main` and a patch release (§3.3), then back into `dev`.
- **The public link is teaching `main`, and it never changes.** A link names a file on
  `main` (`…/teaching/blob/main/control/frequency_response.ipynb`); what that file runs is the
  one pin inside it. A course note never has to be edited for a release or a fix.
- **The `dev` branches move together.** When minilink `dev` changes an API, teaching `dev` and
  workbench `dev` follow soon after. Teaching's nightly CI runs its `dev` against minilink
  `dev` and reports a break the next morning. A large change keeps the twin-branch convention:
  the same branch name in each repository, merged into `dev` together.
- **Teaching `main` takes a new minor release (0.4 → 0.5) only between terms.** During a term
  it takes fixes and patch releases only.
- **A patch release (`0.4.x`) holds fixes only, never an API change.** That is what makes
  moving every notebook to it during a term safe; CI checks it before the merge.

### 3.3 A fix during a term

**A mistake in a notebook** (a typo, a wrong value, a cell to change):

1. In teaching, branch from `main` (`fix/<what>`), fix the notebook, push.
2. CI runs the course's notebooks on the pinned release.
3. Merge into `main`. The link is unchanged; students get the fix the next time they open
   it (or after restarting the runtime).
4. Merge `main` into `dev`.

**A bug in minilink that a notebook hits:**

1. At once, when possible: a workaround in the notebook, as above.
2. The fix in minilink: branch from `main` (the release), fix with a test, merge into `main`,
   tag the patch (`0.4.1`); the publish workflow puts it on PyPI. Merge `main` into `dev`.
3. In teaching: branch from `main`, `python tests/set_minilink.py 0.4.1` (the requirements and
   every notebook's install cell), CI runs every notebook on `0.4.1`, merge into `main`. The
   next Colab session installs `0.4.1`.

If minilink `main` has already moved to the next minor release while a course still runs on
the previous one, the patch is cut from that release's tag on a `release/0.4` branch. With minor
releases only between terms this is rare.

### 3.4 Between terms

1. Teaching `dev` and workbench `dev` are already up to date with minilink `dev`.
2. minilink: `dev` merged into `main`, the minor release tagged and published.
3. Teaching: on `dev`, `set_minilink 0.5.0`, CI passes, `dev` merged into `main`.
4. Workbench: the same, `dev` into `main`; the course solutions follow teaching's pin.

**To check with the calendar:** if the courses run in different sessions, one is always in
progress and `main` takes only patch releases; a minor release then waits for the gap between
sessions. If that is too slow, each notebook's install cell can carry its own pin, so a notebook
outside the active course moves ahead. Kept in reserve.

### 3.5 Workbench

- **A demo while the code moves.** Two conda environments: `minilink` (the editable install of
  the local checkout, whatever branch it is on) and `minilink-stable` (`minilink[full]` at the
  release pin, `reference/envs/minilink-stable.yml`). Two checkouts: `~/code/workbench` on
  `dev`, where the work happens, and `~/code/workbench-main`, a git worktree that stays on
  `main`. A demo runs from `workbench-main` in `minilink-stable`, whatever state the rest is in.
  A prototype reaches `main` once it runs on a released minilink; one that needs a feature of
  `dev` reaches it with the next release.
- **Course material** (`courses/`) follows teaching: pinned to the same release on `main`,
  moved between terms or by a patch release.
- **The log is frozen, and each entry runs on Colab.** `python archive.py <script> "<note>"`
  freezes a script: it refuses when the local minilink has uncommitted changes or a commit not
  on GitHub, copies the script to `log/<year>/<MM-DD>_<name>/`, and writes a header (the date,
  the note, `# minilink: <commit>`, and the versions of JAX, NumPy and SciPy it ran with). Beside
  it, `run.ipynb`: a title cell, one install cell
  (`%pip install "minilink[full] @ git+https://github.com/alx87grd/minilink@<commit>"` and the
  recorded versions), the script embedded (workbench is private, so a Colab session never
  clones it), and a cell that runs it. Opened on Colab, the entry runs on a fresh machine at its
  commit, with nothing installed locally. An entry is never updated to follow minilink. A script
  that opens a window (pygame, meshcat, a matplotlib animation) shows only its inline figures
  there. The entries older than minilink (pyro) stay as they are.

### 3.6 The setup, in order

1. **Written down:** a "Branches and pins" section in each repository's README; in minilink, the
   release and patch rules in ROADMAP and the term rule in AGENTS.md.
2. **minilink, at the move:** the first teaching release (a `0.4.0b1` or `0.4.0`, the
   maintainer's call); `main` is the latest release; the publish workflow runs on a tag.
3. **teaching:** `tests/set_minilink.py` and a CI test that every notebook's pin matches
   `requirements.txt`; the Colab cells from `git clone -b dev-consolidation` to `%pip install`
   at the pin; `dev` created from `main` and `main` pinned to the release; CI on `main` and
   `dev` at every push, `dev` nightly against minilink `dev`.
4. **workbench:** a `dev` branch and the `workbench-main` worktree;
   `reference/envs/minilink-stable.yml`; `archive.py` and the log rule in its README.

**Checked for step 3** (the ruling of 2026-10-25 chose `git clone -b <tag>` because notebooks
imported the examples tree by path): on 2026-11-20 no teaching notebook imports from
`examples/` or `minilink.experimental`, so every one installs the wheel.

## 4. The examples tree

### 4.1 The target tree (agreed 2026-11-26)

The maintainer's demo pass (consolidation.md §4) leaves a small user-facing set. Public links
break at this phase anyway, so the tree is drawn on a clean slate, not around today's paths.

```
minilink/                              the library

tutorials/                             notebooks, opened in Colab, read in order
  showcase_minilink.ipynb
  01_… .ipynb  →  0N_… .ipynb            6–8 numbered notebooks (18 today, in basics/ core/ advanced/)

examples/                              scripts only
  pendulum_classical_control.py        ┐
  pendulum_value_iteration.py          │
  pendulum_compare_planner.py          │
  cartpole_swingup_trajopt.py          │  the showcase: 8–10 flat scripts,
  cartpole_lqg.py                      │  one story each, nothing else at this level
  racecar_kinematic_mpc.py             │
  rrt_star_obstacles.py                │
  ur5_impedance.py                     │
  drone_learns_to_fly.py               ┘
  games/
    game_cartpole.py
    game_udes_racecar.py
    game_ur5_setpoint.py
    offroad_car_game.py

benchmarks/
  automatic/                           run by CI: the regression gate
    run_regression_check.py
    suites/
    scenarios/
    baselines/
    host_profiles/
  manual/                              run by the maintainer
    simulator_speed_matrix.py
    trajopt_solver_presets.py
    diagram_compiling.py
  systems/                             fixture systems, used by both
  simulation.py, trajopt.py, optimization.py,
  f_evaluators.py, dynamic_programming.py,
  baseline.py, common.py, host_profiles.py      measuring helpers, used by both

tests/
  unittest/                            (gains public_names.py, the public-name inventory)
  demo_checks/
  …

docs/
```

The script names are today's, in `example_minimal/`; which of them are the showcase and
which games stay is open (§4.4). minilink has no `advanced/` folder (ruled 2026-11-26): a
deeper example is a notebook in the teaching repository (§4.3).

### 4.2 The rules the tree carries

- **One folder per kind of reader.** `tutorials/` is for someone learning minilink: notebooks,
  read in order, opened in Colab. `examples/` is for someone who knows it and wants a result:
  scripts, picked by topic, run locally. `benchmarks/` answers how fast or how accurate
  minilink is; no student opens it.
- **Position is meaning in `examples/`.** The top level holds the showcase and nothing else;
  the cap (at most ten scripts) is a one-line test, like the knob allowlist.
- **`examples/` holds scripts only**, so the rule "no `.ipynb` beside `.py`" holds by
  construction.
- **`games/` apart** because a game is a different kind of script: interactive, it needs a
  display or a gamepad and never ends on its own; CI gives it a short headless smoke run.
- **No second tier of examples in minilink** (ruled 2026-11-26). A second, less-tested tier
  is how `examples/demos/` reached 71 scripts. An example that is not in the showcase is a
  notebook in the teaching repository: public, run by that repository's CI, pinned to a
  minilink release, so it breaks only when its pin moves. This is the split well-staffed
  libraries make between a curated set in the library and a broad catalog beside it (a
  course's notes, an examples or "zoo" repository); a catalog nobody runs is the one rule
  they all avoid.
- **A benchmark is a script that measures minilink; an example shows how to use it.** In
  `benchmarks/`, one folder per way of running (`automatic/` by CI, `manual/` by hand); the
  top level is what both import (two of the three manual studies use the gate's helpers and
  fixture systems).
- **CI follows the tree:** the showcase on every pull request (it replaces the flagship
  manifest), `games/` a headless smoke run, `tutorials/` the notebook job; the advanced
  notebooks in the teaching repository's own CI.

### 4.3 What leaves

- `example_minimal/` and `examples/demos/` fold into `examples/`. The scripts that are not
  in the showcase, `example_minimal/`'s and the 71 still in `examples/demos/` (2026-11-26),
  each go one of three ways: to the teaching repository as a notebook, under its topic
  folder, when a course or a student uses it; to workbench, unsupported; or deleted. No
  `demos/` folder remains.
- `examples/tutorial/` becomes `tutorials/`.
- `examples/projects/` and `experimental/` go to workbench (consolidation.md §7).
- `benchmarks/public_names.py` moves beside the unit test that reads it: it is an inventory,
  not a benchmark.

The move touches the CI workflows, `AGENTS.md`, `tests/README.md`, `benchmarks/README.md`,
`examples/README.md`, the demo-check manifest and allowlists, and the unit tests that import
from `benchmarks`, all in one change; nothing in `minilink/` imports from these folders.

### 4.4 Still open

- Which scripts are the showcase (the list drawn in §4.1 is a proposal from the 19 scripts of
  `example_minimal/`), and which of the others go to the teaching repository.
- Whether one game joins the showcase.
- Which 6–8 notebooks remain in `tutorials/`, and whether they stay grouped by layer
  (basics, core, advanced) or become one numbered sequence as drawn.

### 4.5 Options set aside

- **A top-level showcase folder beside `examples/`** (`example_minimal/` today): two doors for
  the same kind of file, and "examples" then reads as the leftovers.
- **By system** (`examples/pendulum/`, `cartpole/`, `racecar/`, `ur5/`): a ladder per plant,
  but it hides which script is the showcase one and spreads ten files over five folders.
- **Numbered showcase scripts** (`01_pendulum_classical_control.py`): a reading order at the
  cost of a rename whenever the set changes; the tutorials carry the order.
- **The tutorials inside `examples/`**: one fewer top-level folder, but it mixes two readers
  and two media, and puts the "start here" one click deeper.
- **The manual benchmarks under an `examples/advanced/` folder**: they import the gate's helpers and
  teach nothing about the API.
- **The tutorials under `docs/`**, rendered by the documentation site: worth a look once the
  notebooks are down to the ones that stay; it needs a notebook renderer in the docs build.
- **The earlier options** of this section, before the demo pass: B1, renaming the demo
  chapters; B2, one folder per textbook chapter holding its notebook and demos; B3, notebooks
  first with paired scripts; B4, the tree as the syllabus (rejected: the course axis lives in
  the teaching repository). The demo pass reduced the demos to a set of about ten, which
  none of them was drawn for.

## 5. The asks

1. **The split of the demo set** between the showcase, `games/` and the teaching repository,
   and the notebooks that remain in `tutorials/` (§4.4). The tree itself is agreed (§4.1);
   `advanced/` is ruled out (2026-11-26).
2. **`projects/`:** to workbench (consolidation.md §7), as §4.3 draws it.
3. **The form of the pin** in the teaching repository: `pip install` at the release, with
   `git clone -b <tag>` kept for a notebook that needs the examples tree (§3.6).

When the maintainer rules, the ruling updates this doc and the steps go to
[TODO.md](TODO.md) in the same change; the commit message carries its why (one fact, one
home: [README.md](README.md)).
