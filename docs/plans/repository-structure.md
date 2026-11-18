# The examples tree and the repositories around minilink

**What it is:** the layout of `examples/` (options and a recommendation, not decided) and the
repositories the other material lives in (ruled 2026-11-13; what is left of the move).  
**Lane:** maintainer-owned (student-facing paths and public names; AGENTS.md "Ask first").  
**Where it sits:** the decision "Where the course material and the projects live" (steps K5
and K6 of [TODO.md](TODO.md), v0.9); the pinned Colab cell waits in step V6. The examples
tree is decided with the maintainer's pass over the demos
([minimal-surface.md](minimal-surface.md) §4).

## 1. The repositories (ruled 2026-11-13)

| Repository | Visibility | Holds | Rule |
| --- | --- | --- | --- |
| `minilink` | public | the library, the tutorials, the curated demos, tests, docs | depends on nothing else of the maintainer's; every file is one to show a stranger |
| `alx87grd/teaching` | public | the course and topic notebooks, sorted by topic; mostly on minilink but not limited to it | a course is a list, `courses/<course>.md`, linking the notebooks its notes cite in the notes' order; a notebook on an active course's list is not renamed or moved during its term |
| `alx87grd/workbench` | private | the maintainer's log, reference, prototypes and private course material; old demos worth a second look | no rules; nothing imports from it |
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

## 3. The pin

A **tag**, never a branch (it moves) and not a commit hash (unreadable for a student; fine for
a research project that needs an unreleased state). Tags are bare (`0.3.0`, no `v`), published
to PyPI by the publish workflow. A release tag is never moved; a fix is a new tag and one
commit bumping every notebook of the course.

**Open, for the maintainer:** the form of the pin in the teaching repository. The ruling of
2026-10-25 chose `git clone -b <tag>` over `pip install minilink==<tag>` because the notebooks
imported the examples tree by relative path. In a repository of their own that reason holds
only for notebooks that still need files of `examples/`; for the others
`pip install -q minilink==0.4.0` is shorter and faster. Two facts to check per notebook: does
it import from the examples tree, and does it use `minilink.experimental` (not in the wheel)?

## 4. The examples tree

### 4.1 Facts

| Folder | Content |
| --- | --- |
| `tutorial/` | three layers, `basics/` (3 notebooks), `core/` (9), `advanced/` (5), and one showcase |
| `demos/<chapter>/` | 94 scripts in 18 chapters: the package names plus `rrt`, `hybrid`, `mpc`, `realtime`, `robotic`, `trajopt`, `value_iteration`, `udes_racecar` |
| `demos_proposed_minimal/` | 9 scripts, the draft of the curated set |
| `projects/` | 3 multi-file applications (`car_trajopt`, `mpc`, `pathtracking`) |

The tree carries three axes: by *form* (scripts or notebooks), by *package*
(`demos/<chapter>/` mirrors the library) and by *lane* (`projects/`).

### 4.2 Options

**B1 — keep the kinds, tidy the names.** Rename the demo chapters to the tutorial numbering
(`demos/09_planning/` rather than `rrt`, `trajopt`, `value_iteration`).

Pros: the smallest change. Cons: the axes stay, and the README table stays the only map.

**B2 — one axis: the chapter.**

```
examples/
  <NN>_<chapter>/            # one folder per textbook chapter, in tutorial order
    README.md                # the chapter index: what each file teaches, in reading order
    <chapter>.ipynb          # the tutorial notebook (today's tutorial/<layer>/NN_<chapter>.ipynb)
    <plant>_<method>.py      # the canonical demos (today's demos/<chapter>/*.py)
  research/
    projects/<name>/         # multi-file applications (today's projects/)
```

Two rules replace the README table: **a demo is a script, the chapter notebook is the
chapter index**; `research/` is outside the release contract. The "no `.ipynb` and `.py` in
one leaf" rule goes, since the chapter README carries the form. Promotion becomes one arrow:
`research/` → a chapter.

Pros: the tree *is* the textbook. Cons: every demo moves (`git mv`, history kept);
`tests/demo_checks` and the nightly sweep paths change; the chapter list has to be agreed,
and `hybrid`, `mpc`, `realtime`, `robotic` and the racecar folder need a chapter each.

**B3 — notebook-first with jupytext pairing.** Every example is a notebook; the scripts are
generated twins (`.py:percent`), and diffs happen on the `.py`.

Pros: solves script-vs-notebook once; reviewable diffs. Cons: a new tool in the student path
and in CI, and the demos' value is being short scripts a student reads top to bottom (RULES
6.1, 6.10); a build step on the one thing that should have none.

**B4 — course-first: the tree is the syllabus** (`examples/<course>/<week>/`). Rejected: the
library is course-neutral by the constitution; the course axis lives in the teaching
repository.

### 4.3 Recommendation

**B2**, staged between terms: (1) agree the chapter list and write the two rules in
`examples/README.md`; (2) move `demos/<chapter>/` into the chapters, one commit per chapter,
`git mv`; (3) `projects/` under `research/`. B1's renames are a fallback that keeps the
option open if B2 has to wait a term.

**To weigh against it:** the draft of the curated demo set (minimal-surface.md §4.2) is one
flat folder of about ten scripts, each named by its plant and its task, with the tutorials as
the tool-by-tool walk. If that set replaces `demos/`, the chapter folders hold one notebook
each and B2 reduces to the tutorials' own layers.

## 5. The asks

1. **The layout of `examples/`** (§4), with the choice of the demo set.
2. **The chapter list** for B2: the three tutorial layers as the spine, and a home for
   `hybrid`, `mpc`, `realtime`, `robotic` and the racecar folder.
3. **`projects/`:** to the research repository
   ([consolidation-recap.md](consolidation-recap.md) §4), or kept as `research/projects/`.
4. **The form of the pin** in the teaching repository (§3).

When the maintainer rules, the ruling updates this doc and the steps go to
[TODO.md](TODO.md) in the same change; the commit message carries its why (one fact, one
home: [README.md](README.md)).
