# Evening session log — 2026-09-05 (autonomous, branch `dev-fable`)

Mandate from the maintainer: implement everything in the plan that is
straightforward, commit in small steps, no architecture or user-API changes,
postpone any design fork for a joint session. Plan of record:
[ROADMAP.md](../../ROADMAP.md); step specs: [docs/plans/TODO.md](../plans/TODO.md).

## What landed (32 commits, `5c41f83..HEAD`, incl. this log)

**Phase D — docs as plan of record (all seven).** ROADMAP rewritten
(milestones, two lanes, TRL ledger with lane column, GRO860 checklist,
phases, review queue, consolidation principle); DESIGN (two-lane stability,
wheel scope, float64 policy, unconnected-input contract, control-block
decision record); AGENTS (student-facing import rule, demo-header rule,
consolidate-never-strip, delegation split); TODO re-based; two self-marked
complete plan docs deleted and the parity audit shrunk 49 → 36 KB with a
landed-name map; README (custom-plant example composes, two-lane table);
examples README labels the research lane.

**Phase 0 — first-hour safety (all approved steps).**

| Step | Commit | Effect a student sees |
| --- | --- | --- |
| S40 | `2bc97de` | `DynamicSystem(output_dim=n)` outputs `y = x` by default (pyro convention); a textbook plant composes with `controller @ plant` |
| S05 | `77acd2e` | JAX runs float64 by default — the canonical trajopt problems succeed without `configure_jax` |
| S01 | `08080e5` | default grid is 1 001 points with the solver chosen first; JAX default is `scipy` (200 RHS evals, was 400 000) |
| S07 | `1b69825` | one `verbose` flag on planner / optimizer / MPC (panels unchanged) |
| S08 | `30d6df7` | `nbstripout` hook matches notebooks again |
| S04 | `dae31d3`, `b2e3727` | missing `'y'` port names the fix instead of "dim None" |
| S06 | `4fc6e98` | forgotten `super().__init__()` gives a named error |
| S42 | `9a56f0e` | multiple shooting refuses the parametric build instead of silently building collocation defects |
| S02 | `ffb0a4a` | wrong-shape `f` / `h` fail at `compile()` on both backends with the block, hook, and shapes named |
| S09 | `e568098` | trajopt `success` = the plan satisfies the constraints to `feasibility_tol`; violations recorded on the metadata |

**Phase 1 — teaching contract and the GRO860 path.**

| Step | Commit | What it is |
| --- | --- | --- |
| S18–S20 | `90f6fbf` | wheel excludes `symbolic/`, `dynamics/engines/`, `c_export`; `c_export` demos in the JAX regression job; nightly demo + notebook workflow |
| S21 | `51544f2` | branch-cleanup script for you to run ([2026-09-05-branch-cleanup.md](2026-09-05-branch-cleanup.md)) |
| S41 | `de2ccfb` | ranked consolidation inventory ([2026-09-05-consolidation-inventory.md](2026-09-05-consolidation-inventory.md)) |
| S13 | `699b497` + follow-up | `minilink.planning`, `minilink.core`, `minilink.optimization` band facades (lazy `_EXPORTS`, additive) |
| S11 | `fb6222b` | teaching-surface registry test (resolves, docstring, teaching lane) + Basic-tier clean-environment smoke (sim / plots / animate / linearize / LQR / VI with every optional package blocked) |
| S12 | `b7e6760` | import-layer test over `examples/learn` + `examples/demos` with a per-file allowlist that can only shrink |
| S14, S15 | `d6a547c`, `a0a2215`, `9e6bc3a` + follow-up | every student-facing file imports through the teaching surface; allowlist 339 → ~100 rows, all research-lane names (`core.backends.configure_jax` — now redundant after S05 — spatial scenes, geometry, JAX vehicle ladder, extenders) |
| — | `acc299e` | `plot_diagram()` warns instead of raising when the Graphviz binary is missing (bare Colab) |

## Verification at the end of the session

Full unit suite, catalog checks (49/49), demo sweep (60 pass / 3 interactive
skips), notebook smoke (15/15 with `dot` on `PATH`). Regression gates ran green
after S01 (the `Simulator` change).

**Final state of `dev-fable`:** `ruff` clean · `pytest` **915 passed, 2 skipped**
(was 884 on `dev-alex`) · catalog checks 49/49 · demo sweep **60 passed, 0 failed**
(3 interactive skips) · notebook smoke **15/15**.

## Postponed on purpose — need you

| Item | Why it waits |
| --- | --- |
| S16 `_jit` aliases, S17 28 unreferenced evaluator methods | deletions of public attribute names — your pick (inventory rows 2–3) |
| S33 `Sys2Gym` compiled step | value is unclear without vectorized envs (the SB3 loop, not `f`, dominates); needs a measurement first |
| S38 DP metadata (`final_time` reads `problem.tf`, honest `success`) | changes result semantics of a GRO860 tool |
| S39 demo headers → one-line | a dry run showed the headers mix run instructions (drop) with *lesson* prose (move next to the code); editorial, one joint pass |
| S22–S26 catalog `xp` sweep, twins, four-rung ladder, `rollout_batch` | catalog names and evaluator API — Phase 2, with you |
| `configure_jax(enable_x64=True)` calls in 34 demos/tests | now no-ops after S05; removing them is a demo edit — trivial once you say so |
| `examples/learn/teaching/mpc.ipynb` | your uncommitted work; only the `step_disp → verbose` kwarg was renamed in the working tree, never staged |

## Gotchas recorded for future sessions

`dot` needs the env's `bin` on `PATH` (the one "failing" notebook was that);
zsh needs `${=var}` splitting and `set -o pipefail` for gated chains; the
project's `addopts` already carries `-q`.

## Addendum — textbook pass on the student-facing API (same day, with the maintainer)

Measured first: 63 demos, 3 504 code lines, of which 293 import lines,
193 `np.array([...])`, 56 bound assignments, 101 `.params[...] =`. Four API
proposals were put to the maintainer; **A** (scalar/list bounds and `x0`) and
**B** (scalar `Q`/`R`/`S` in `QuadraticCost.from_system`) were **declined** —
bounds and cost matrices stay explicit arrays. **C** and **D** landed:

| Commit | Change |
| --- | --- |
| `3c3ce90` | `DynamicProgrammingPlanner(problem, x_grid=, u_grid=, dt=, ...)` builds its own grid (one object); `grid=` stays for custom grids |
| `7296cb6` | the root prelude **is the teaching surface** — one `from minilink import ...` line; catalog delegated to `minilink.catalog`; tested as a set |
| `7f13f79` | 80 student-facing files rewritten to one root import statement |
| `6defb17` | no-op `configure_jax` lines out of the `.py` demos; trajopt demo uses `plot_solution()` / `animate_solution()` and loses two unused flags; VI demos and the three VI notebooks use the one-object DP setup; the VI-vs-LQR(-vs-PPO) notebooks wire their loops with `controller @ plant` |
| follow-ups | allowlist regenerated (93 rows, all research-lane); merged root imports collapsed to one physical line where they fit |

Corpus effect: import lines **329 → 228**, `configure_jax` calls **7 → 0**.
Verification after the pass: `pytest` 919 passed / 2 skipped, demo sweep
60/60, notebook smoke 15/15.
