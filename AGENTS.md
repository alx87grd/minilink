# Minilink AI Agent Instructions

**Always read CONSTITUTION.md before any interaction** — including questions,
reviews, and plans that do not touch code.

**Also read RULES.md before editing code** under `minilink/` (or tests, examples,
and other Python that implements or exercises that code).

This file is workflow: what to read, when to ask, and the local CI gate.
User API: README.md. Contracts: DESIGN.md. Maturity: ROADMAP.md.
Pytest policy: tests/README.md. Examples map: examples/README.md.

## Non-negotiables

- **Preserve user edits:** never revert or "clean up" manual changes the user made in
  demos, notebooks, examples, or scratch code — commented-out plots, tuning constants
  (`TF`, gains, step times), disabled sections, exploratory variables — unless they
  explicitly ask. Commit/review passes must not overwrite user-tuned script state.
- **Docs are contract:** update DESIGN / ROADMAP / README when public behavior or
  maturity claims change; CONSTITUTION / RULES when identity or review law changes.
- **Coach the architecture:** name tradeoffs; steer toward the simplest clear interface.

## Doc map

| Doc | When to update |
| --- | --- |
| CONSTITUTION.md | Vision, contract invariants, conflict precedence — maintainer-owned |
| RULES.md | Universal code and review ladder (humans and agents) |
| README.md | User workflows, install, examples table |
| DESIGN.md | Public contracts, package layout, evaluator behavior |
| ROADMAP.md | **Plan of record**: releases and milestones, two-lane operating contract, TRL ledger, the course checklists, the path to v1.0 (rungs and waves), open decisions, out-of-scope |
| AGENTS.md | Agent workflow, doc map, local CI gate |
| CLAUDE.md | Entry stub that points here; update only if this reading order changes |
| docs/plans/TODO.md | The workboard: every open step of ROADMAP §5, by rung, with files and "done when"; Later ideas at the end |
| docs/plans/ | Design writeups for steps that need one (delete a plan doc once it lands; the index there says which rung each serves) |
| docs/plans/pyro-port-remaining.md | Pyro parity rows when library or demos land |
| docs/ | Sphinx autodoc of the teaching-lane API (`docs/api/`, `docs/index.rst`); `experimental/` is repo-only and not on the site. The user guide is `examples/tutorial/`. README GIFs and diagram PNG from `docs/make_assets.py` (`ur5_meshcat.gif` is a screen recording, not rebuilt) |
| tests/README.md | Marker policy, test philosophy, **entry points (human · agent · CI)** |
| docs/reviews/ | Dated architecture audits and the interview decision records; read-only history, never a backlog |

Keep DESIGN.md call chains minimal.

## Workflow

**Do directly:** typos and stale docs; docstrings/types in files you are already changing for the task; small cleanups that directly support the requested change.

**Never without explicit ask:** revert, uncomment, rename, or "polish" user manual edits in `examples/`, notebooks, or scratch files (tuning params, commented plot/animate calls, exploratory locals). Do not rename or move anything under `examples/teaching/courses/` without the citing documents in the same change: GRO860 is notes + webpage; GRO501 for now is only the Notes-Commande `\colab{}` (no course webpage).

**Ask first (maintainer-owned):** anything student-facing (`README.md`,
`examples/`, notebooks, ROADMAP §1, §2 and §4, public names); **core
architecture and the API of the main tools** (`System` family, the core
mathematical objects — `Trajectory`, sets, distributions, costs, fields,
`PlanningProblem`, `PlanningSolution` — diagrams, compile, `Simulator`,
planners, `Optimizer`, controllers); any feature or
user-importable-name removal; delete/rename files; new dependencies; removing
user scratch code; `CONSTITUTION.md` amendments; dropping a step from ROADMAP §5
(a rung is done when its steps land or the maintainer drops them) and closing an
open decision in ROADMAP §6 (each open item there needs the maintainer).

**Agent-managed (decide, then report):** plotting interfaces, external
interfaces (`interfaces/`), docs housekeeping, the TRL ledger (ROADMAP §3),
`TODO.md`, recording in ROADMAP §5 and §6 a step that landed or a decision the
maintainer took, plan-doc housekeeping, tests, `RULES.md` wording that does not
change a public contract, and internal code structure that does not change a
public contract. When you spot an opportunity outside your lane, ask — do not act.

**Scope:** stop and explain the smallest slice if a small request grows large. For larger work, write a concise plan and wait for approval. Chat conflicts with this file, CONSTITUTION.md, or RULES.md → ask before proceeding.

**Asking for a decision:** show the context before the options: the code as it stands, the evidence, and what each option changes. The maintainer dismisses a bare option list; one worked snippet makes the choice answerable.

**Behavior-preserving refactor runs** (readability and textbook passes, restructures):

1. Read the tests first; they are the output contract. Keep every asserted name working (a property can replace an attribute).
2. Before touching code, capture a seeded baseline to JSON in the scratchpad: every branch the change touches (each algorithm family, each adapter path, each random-draw path, repeated calls), with timing fields left out. Run it twice and `cmp` the files to prove it is deterministic.
3. One concern per step. After each: ruff, the targeted tests, the baseline again, `cmp` against the first capture. Byte-identical is the bar (RULES 7.7); an intended change gets its own test instead.
4. Before redesigning a shape callers consume, count what they actually read across the library, examples, tests and benchmarks. Before naming a domain object, check the course notes' own vocabulary.
5. Never edit modules while a background sweep imports them; write the next edit as a script in the scratchpad and apply it once the sweep reports.
6. Finish with lint, full `pytest`, the affected demos, and the notebook checks.

**Notebooks:** skip review unless updating renamed imports or user asks; outputs
stripped by pre-commit (`nbstripout`). After notebook edits, run the notebook smoke check
(tests/README.md, Agent table, "Notebook change"; the CI `regression` job runs the same).

## Before push or PR (local CI gate)

**Entry points:** tests/README.md (section "entry points") is the one written copy of the
commands — humans use **`tests/run/`** (IDE Run); agents use its Agent table, and its CI
table says what each job runs. This section says *when*, by the Agent table's row names.

GitHub **CI** (`.github/workflows/test.yml`) is the merge gate. It has three jobs: **`test`** (ruff and `pytest` on Python 3.10–3.13), **`packaging`** (build the sdist and wheel, check them, import the installed wheel), and **`regression`** (`pytest` with JAX, the regression gates, the flagship demos, the notebook smoke). Run the same checks **locally before push or PR** so CI does not fail on lint/format — do **not** poll GitHub Actions after every small commit unless the user asked you to push or verify remote CI.

Three workflows are **not** merge gates: `publish.yml` uploads a `0.*` tag to PyPI once its **`check`** job (ruff and `pytest`) and **`build`** job pass; `nightly.yml` runs every script under `examples/demos/` and every teaching notebook with the full optional stack; `docs.yml` builds the Sphinx site. A demo that only the nightly sweep exercises still has to run — check it locally when you land one.

**Always before push** (fast; mirrors CI `test` job; the pre-commit hooks run the same two
once `pre-commit install` has run in the clone):

```bash
conda activate minilink
ruff check .
ruff format --check .
```

Fix with `ruff check --fix .` and `ruff format .` when either fails. CI runs these on the **whole repo**, not only touched files.

**Pytest and the gates — proportionate** (the command of each row is in the tests/README.md Agent table):

| Change | Agent-table row |
| --- | --- |
| Docs/markdown only | skip pytest |
| Narrow module + tests already updated | "Narrow module change" |
| Cross-cutting or before handoff/push | "Cross-cutting or handoff" |
| Compile backend, simulator, trajopt, MPC or value-iteration changes | "Regression gates, CI flags" |
| Big review pass (compile backend, `Simulator`, trajectory optimization, or cross-cutting dynamics changes) | "Regression gates, full local" (its `--update` rule included) |
| Teaching notebooks | "Notebook change" |

Optional extras (not required every push): `SDL_VIDEODRIVER=dummy pytest` for headless pygame; graphics visual checklist and demo-check runners in tests/README.md when graphical or user-facing demos changed; `sphinx-build -W --keep-going -b html docs docs/_build/html` only when editing `docs/` or docstrings (separate Docs workflow; warnings fail it).

**After push:** only check GitHub CI when the user asked to push, open a PR, or debug a reported failure — not as a routine step on every edit.

Use conda env **`minilink`** from environment.yml; setup in README.md (install) (`PYTHONPATH` = repo root).

**Handoff:** re-read the diff for scope creep; preserve user manual edits in demos/notebooks; clean `git status`; short summary of changes and verification.
