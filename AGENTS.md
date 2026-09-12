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
| ROADMAP.md | **Plan of record**: releases and milestones, two-lane operating contract, TRL ledger, GRO860 checklist, phases, review queue, out-of-scope |
| AGENTS.md | Agent workflow, doc map, local CI gate |
| docs/plans/TODO.md | Operational backlog: small fixes, pre-v0.2 hardening, demo pulls, new modules, Later ideas |
| docs/plans/ | Active **design** writeups only (multi-step plans; delete finished plan docs) |
| docs/plans/pyro-port-remaining.md | Pyro parity rows when library or demos land |
| docs/pitch/ | The five-slide pitch (`slides.html` + `pitch.css`): single source for the docs landing page (`docs/index.rst`) and the standalone deck `docs/_static/pitch.html` built by `docs/make_assets.py` (also the README GIFs and diagram PNG; `ur5_meshcat.gif` is a screen recording, not rebuilt) |
| tests/README.md | Marker policy, test philosophy, **entry points (human · agent · CI)** |
| docs/reviews/ | Dated architecture audits and the interview decision records; read-only history, never a backlog |

Keep DESIGN.md call chains minimal.

## Workflow

**Do directly:** typos and stale docs; docstrings/types in files you are already changing for the task; small cleanups that directly support the requested change.

**Never without explicit ask:** revert, uncomment, rename, or "polish" user manual edits in `examples/`, notebooks, or scratch files (tuning params, commented plot/animate calls, exploratory locals).

**Ask first (maintainer-owned):** anything student-facing (`README.md`,
`examples/`, notebooks, ROADMAP §1–§4, public names); **core architecture and
the API of the main tools** (`System` family, diagrams, compile, `Simulator`,
planners, `Optimizer`, controllers); any feature or user-importable-name
removal; delete/rename files; new dependencies; removing user scratch code;
`CONSTITUTION.md` amendments.

**Agent-managed (decide, then report):** plotting interfaces, external
interfaces (`interfaces/`), docs housekeeping, the TRL ledger, `TODO.md`,
plan-doc housekeeping, tests, `RULES.md` wording that does not change a
public contract, and internal code structure that does not change a public
contract. When you spot an opportunity outside your lane, ask — do not act.

**Scope:** stop and explain the smallest slice if a small request grows large. For larger work, write a concise plan and wait for approval. Chat conflicts with this file, CONSTITUTION.md, or RULES.md → ask before proceeding.

**Notebooks:** skip review unless updating renamed imports or user asks; outputs
stripped by pre-commit (`nbstripout`). After notebook edits, smoke-check with
`MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py` (CI
``regression`` job runs the same).

## Before push or PR (local CI gate)

**Entry points:** tests/README.md (section "entry points") — humans use **`tests/run/`** (IDE Run); agents and CI use the CLI table in that doc.

GitHub **CI** (`.github/workflows/test.yml`) runs exactly: `ruff check .`, `ruff format --check .`, `pytest` on Python 3.10–3.13, then the **`regression`** job (regression gates + flagship demos + notebook smoke with JAX). Run the same checks **locally before push or PR** so CI does not fail on lint/format — do **not** poll GitHub Actions after every small commit unless the user asked you to push or verify remote CI.

**Always before push** (fast; mirrors CI `test` job):

```bash
conda activate minilink
ruff check .
ruff format --check .
```

Fix with `ruff check --fix .` and `ruff format .` when either fails. CI runs these on the **whole repo**, not only touched files.

**Pytest — proportionate** (same command CI `test` job uses; scope by change):

| Change | Run |
| --- | --- |
| Docs/markdown only | skip pytest |
| Narrow module + tests already updated | `pytest tests/unittest/test_<domain>.py` |
| Cross-cutting or before handoff/push | `pytest` |
| Compile backend, simulator, or trajopt changes (big review pass) | Regression gates: `PYTHONPATH=. python benchmarks/run_regression_check.py --suite all --tiny --factor 10 --speed-gate-suffixes solve_s,nlp_s,speedup` |
| Teaching notebooks | `MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py` |

Regression gates full command and CI `regression` job flags: tests/README.md (entry points).

Optional extras (not required every push): `SDL_VIDEODRIVER=dummy pytest` for headless pygame; graphics visual checklist and demo-check runners in tests/README.md when graphical or user-facing demos changed; `sphinx-build` only when editing `docs/` (separate Docs workflow).

**After push:** only check GitHub CI when the user asked to push, open a PR, or debug a reported failure — not as a routine step on every edit.

Use conda env **`minilink`** from environment.yml; setup in README.md (install) (`PYTHONPATH` = repo root).

**Big review pass** (compile backend, `Simulator`, trajectory optimization, or cross-cutting dynamics changes):

```bash
python benchmarks/run_regression_check.py --suite all
```

Use `--update` only after intentional perf or trajectory changes; review the JSON diff before committing. See benchmarks/README.md.

**Handoff:** re-read the diff for scope creep; preserve user manual edits in demos/notebooks; clean `git status`; short summary of changes and verification.
