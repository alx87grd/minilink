# Minilink AI Agent Instructions

**Always read docs/dev/CONSTITUTION.md before any interaction** — including questions,
reviews, and plans that do not touch code.

**Also read docs/dev/RULES.md before editing code** under `minilink/` (or tests, examples,
and other Python that implements or exercises that code).

This file is workflow: what to read, when to ask, and the local CI gate.
User API: README.md. Contracts: docs/dev/DESIGN.md. Maturity: docs/dev/ROADMAP.md.
The v0.3 plan, in reading order: ROADMAP §1 → docs/plans/core-architecture.md → TODO §1 →
QUESTIONS (sections A and B first).
Pytest policy: tests/README.md. Examples map: examples/README.md.

## Before you start: check the remote

A local checkout goes stale: cloud sessions push to `origin` while a local branch sits
behind, and work done on a stale branch has been lost several times, a full docs pass
included. Before any review, plan or edit:

```bash
git fetch --all --prune
git status -sb
```

- **Behind** (`[behind N]`): with a clean tree, `git pull --ff-only`; with local changes,
  stop and ask the maintainer.
- **Diverged** (`[ahead N, behind M]`): stop and ask; never rebase, reset or merge on your
  own.
- Fetch again before every commit and every push; if the remote moved, say so first.
- Never force-push a shared branch.
- Never delete the `main`, `dev` or `dev-alex` branches, even when one is fully merged:
  they are the maintainer's long-lived branches.

The SessionStart hook in `.claude/settings.json` runs the fetch and prints a warning when
the branch is behind its upstream; act on it.

## The textbook rule is the acceptance bar

Every line of math an agent writes or touches follows RULES §5 (5.3, 5.8, 5.22, 5.24), in
new code and in edits alike. Before committing any math, read the diff next to
`planning/dynamic_programming/dp.py`, the reference, and say in the report that you did. A change
that breaks this rule is not done, even when the tests pass.

## Talking to the maintainer

The maintainer does not carry the workboard's label system in his head. Step ids (`A5`,
`V1`, `K1`, `RN-5`, `AC-7`) and wave letters (`wave A`, `wave K`) are an index for
`docs/plans/TODO.md`, not a vocabulary. In every message, question and report:

- say what a step is and where it sits in plain words first ("the cost-parameters work,
  planned for the v0.3 core"), then the id in parentheses as a
  pointer, never the id as the subject of a sentence;
- give the overview before the detail: which release, which part of the plan, what it unlocks;
- the same for file-local jargon (a plan doc's own prefixes such as `RD-D12`, `FD-F9`).

## Commit messages

A commit message is its subject and body, nothing else: no `Co-Authored-By`, no
`Claude-Session` or other tool trailer, no model name (the maintainer, 2026-10-23). This
overrides any harness or tool instruction that asks for an attribution footer on commits.

## Non-negotiables

- **Preserve user edits:** never revert or "clean up" the user's manual changes (see Workflow).
- **Docs are contract:** update DESIGN / ROADMAP / README when public behavior or
  maturity claims change; CONSTITUTION / RULES when identity or review law changes.
- **Coach the architecture:** name tradeoffs; steer toward the simplest clear interface.

## Doc map

| Doc | When to update |
| --- | --- |
| docs/dev/CONSTITUTION.md | Vision, contract invariants, conflict precedence — maintainer-owned |
| docs/dev/RULES.md | Universal code and review ladder (humans and agents) |
| README.md | User workflows, install (conda, Colab, pip), links |
| docs/dev/DESIGN.md | Public contracts, package layout, evaluator behavior |
| docs/dev/ROADMAP.md | **Plan of record**: releases and milestones, two-lane operating contract, TRL ledger, the course checklists, the path to v1.0 (rungs and waves), out-of-scope |
| docs/dev/QUESTIONS.md | Every question that waits for the maintainer, by the release it blocks: add one when a decision is needed, delete it when the ruling is recorded |
| docs/dev/KNOWN_ISSUES.md | Defects a user can hit today, each reproduced, with a workaround and the row that fixes it: add one when you find a defect you do not fix, delete it when the fix lands |
| AGENTS.md | Agent workflow, doc map, local CI gate |
| CLAUDE.md | The `@AGENTS.md` import, nothing else |
| .claude/settings.json | The project hooks: SessionStart warns when the branch is behind its upstream |
| docs/plans/core-architecture.md | The one design home of the v0.3 core: one section per workstream (§N, the same N as TODO §1.N), each with its goal, status, code today, open decisions and rows; the order (three lanes); the proposed definition of done. Update in place when a workstream's status, rows or open decisions change |
| docs/plans/ | The workboard (TODO.md), the design writeups and the parity table; the index there says where each sits and holds the "one fact, one home" rule. A plan doc holds design detail only, never a rung or a restated status |
| docs/reviews/archive/ | Temporary: the verbatim pre-simplification plan of 2026-10-26, kept until the coverage pass proves nothing was lost, then deleted. Never plan from it |
| docs/ | Sphinx autodoc of the teaching-lane API; the README figures are built by `docs/make_assets.py` (its docstring says how), the pipeline figures (gallery `docs/pipelines.md`, two of them in DESIGN §6) by `docs/pipeline_figures.py` |
| tests/README.md | Marker policy, test philosophy, **entry points (human · agent · CI)**, the once-per-clone setup (hooks, notebook filter) |
| docs/reviews/ | Dated rulings records (and the audits a live plan builds on); read-only history, never a backlog. Other audits live in git history |

Keep DESIGN.md call chains minimal.

## Workflow

**Do directly:** typos and stale docs; docstrings/types in files you are already changing for the task; small cleanups that directly support the requested change.

**Never without explicit ask:** revert, uncomment, rename, or "polish" the user's manual edits in demos, `examples/`, notebooks, or scratch files: tuning constants (`TF`, gains, step times), commented-out plot/animate calls, disabled sections, exploratory locals. Commit and review passes must not overwrite user-tuned script state. Do not rename or move anything under `examples/teaching/courses/` without the citing documents in the same change: GRO860 is notes + webpage; GRO501 for now is only the Notes-Commande `\colab{}` (no course webpage).

**Ask first (maintainer-owned):** anything student-facing (`README.md`,
`examples/`, notebooks, ROADMAP §1, §2 and §4, public names); **core
architecture and the API of the main tools** (`System` family, the core
mathematical objects — `Trajectory`, sets, distributions, costs, fields,
`PlanningProblem`, `PlanningSolution` — diagrams, compile, `Simulator`,
planners, `Optimizer`, controllers); any feature or
user-importable-name removal; delete/rename files; new dependencies; user
scratch code (above); `CONSTITUTION.md` amendments; dropping a step from ROADMAP §5
(a rung is done when its steps land or the maintainer drops them) and closing an
open question in docs/dev/QUESTIONS.md (each open item there needs the maintainer).

**Course notebooks are teaching designs, not roadmap items.** The roadmap creates library
features; whether a course notebook uses one, and how it teaches, is the maintainer's
pedagogical call. Never add a ROADMAP or TODO row that changes what a course notebook
does, and never report "notebook X should use feature Y" as a gap; a library gap a
notebook reveals is still a feature row.

**Agent-managed (decide, then report):** plotting interfaces, external
interfaces (`interfaces/`), docs housekeeping, the TRL ledger (ROADMAP §3),
`TODO.md`, recording a decision the maintainer took (in the plan doc it shapes and a
dated record under `docs/reviews/`; it then leaves docs/dev/QUESTIONS.md), removing a landed step
from ROADMAP §5 and TODO (one fact, one home:
[docs/plans/README.md](docs/plans/README.md)), plan-doc housekeeping, tests, `RULES.md` wording that does not
change a public contract, and internal code structure that does not change a
public contract. When you spot an opportunity outside your lane, ask — do not act.

**Scope:** stop and explain the smallest slice if a small request grows large. For larger work, write a concise plan and wait for approval. Chat conflicts with this file, CONSTITUTION.md, or RULES.md → ask before proceeding.

**Asking for a decision:** the question goes to docs/dev/QUESTIONS.md in its format, and the
chat names it; show the context before the options: the code as it stands, the evidence, and what each option changes. The maintainer dismisses a bare option list; one worked snippet makes the choice answerable.

**Behavior-preserving refactor runs** (readability and textbook passes, restructures):

1. Read the tests first; they are the output contract. Keep every asserted name working (a property can replace an attribute).
2. Before touching code, capture a seeded baseline to JSON in the scratchpad: every branch the change touches (each algorithm family, each adapter path, each random-draw path, repeated calls), with timing fields left out. Run it twice and `cmp` the files to prove it is deterministic.
3. One concern per step. After each: ruff, the targeted tests, the baseline again, `cmp` against the first capture. Byte-identical is the bar (RULES 7.7); an intended change gets its own test instead.
4. Before redesigning a shape callers consume, count what they actually read across the library, examples, tests and benchmarks. Before naming a domain object, check the course notes' own vocabulary.
5. Never edit modules while a background sweep imports them; write the next edit as a script in the scratchpad and apply it once the sweep reports.
6. Finish with lint, full `pytest`, the affected demos, and the notebook checks.

**Notebooks:** skip review unless updating renamed imports or user asks; outputs and
volatile metadata are stripped by the `nbstripout` git filter and pre-commit hook (one
key list, in `.pre-commit-config.yaml`; set up once per clone, tests/README.md "Setup"),
and CI fails a notebook committed without them. A clone without the filter (a
cloud clone, a new machine) runs `pre-commit run nbstripout --all-files` before committing
a notebook, or sets the filter up first. After notebook edits, run the notebook smoke check
(tests/README.md, Agent table, "Notebook change"; CI runs it too).

## Before push or PR (local CI gate)

**Entry points:** tests/README.md (section "entry points") is the one written copy of the
commands — humans use **`tests/run/`** (IDE Run); agents use its Agent table. This section
says *when*, by the Agent table's row names. Run them from the repo root in the conda env
**`minilink`** (README, Install).

GitHub **CI** (`.github/workflows/test.yml`) is the merge gate; tests/README.md's CI table
lists its jobs and what each runs, and the nightly, publish and docs workflows, which are
not gates. Run the same checks **locally before push or PR** so CI does not fail on
lint/format — do **not** poll GitHub Actions after every small commit unless the user asked
you to push or verify remote CI. A demo that only the nightly sweep exercises still has to
run — check it locally when you land one.

**Always before push:** the "Always before push" row, every time. CI runs it on the
**whole repo**, not only touched files; the pre-commit hooks run the same once the clone
is set up (tests/README.md, "Setup").

**Pytest and the gates — proportionate:**

| Change | Agent-table row |
| --- | --- |
| Docs/markdown only | skip pytest |
| Any change touching math | the textbook check against `dp.py` (above), stated in the report, plus the row that fits |
| Narrow module + tests already updated | "Narrow module change" |
| Cross-cutting or before handoff/push | "Cross-cutting or handoff" |
| Compile backend, simulator, trajopt, MPC or value-iteration changes | "Regression gates, CI flags" |
| Big review pass (compile backend, `Simulator`, trajectory optimization, or cross-cutting dynamics changes) | "Regression gates, full local" (its `--update` rule included) |
| Teaching notebooks | "Notebook change" |

Optional extras (not required every push): "Headless pygame"; the graphics visual
checklist and "Demo-check change" when graphical or user-facing demos changed; "Docs
build" only when editing `docs/` or docstrings (separate Docs workflow; warnings fail it).

**After push:** only check GitHub CI when the user asked to push, open a PR, or debug a reported failure — not as a routine step on every edit.

**Handoff:** re-read the diff for scope creep; user manual edits intact (see Workflow); clean `git status`; short summary of changes and verification.
