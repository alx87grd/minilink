# The second roadmap review: the rulings (2026-10-22)

**Date:** 2026-10-22
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** A second review of the roadmap and the short-term plan on `dev`, one day after
the reorganization around the spine ([record](2026-10-21-roadmap-reorganization.md)). The
review found the order sound and six places where the plan contradicted itself or left a
step without a home. The maintainer ruled: "apply the recommended option for all those;
the order matters, the sync with class teaching is a secondary concern". Nothing here
changes an architecture decision still open in ROADMAP §6; every item is a scheduling or
bookkeeping ruling, to be reviewed again before the code it schedules is written.

## Rulings

- **R1 The order is the commitment; the course calendar is secondary.** ROADMAP §5 states
  the rule once: GRO860 runs in the fall term, GMC714 in the winter term v0.3 prepares;
  *between terms* is the late-December 2026 window; *between cohorts* means between GRO860
  cohorts, the course whose material reads the Monte Carlo evaluator and the RL numbers.
  v0.4 is winter 2027 in that sense; the GMC714 term runs meanwhile and does not gate it.
  A step that would move a running course's numbers ships behind the pinned release the
  course runs on (V6) rather than waiting. Lives in: ROADMAP §1 (v0.4 row), §5 intro, §5.4;
  TODO §5 order.
- **R2 One home per consolidation item.** The evaluator-grid trim (one `_p` body per
  integrator, the bound and jitted variants generated) and the integrators' move to
  `simulation/solvers/` are S75's step SM-6, in v0.3; S37 is absorbed by S75 (SM-1 its one
  scannable step, SM-6 the rest) and leaves the workboard; S27 (Diffrax) stays in v0.9 as its
  own row. `StepRollout` folded into `Trajectory` is the last commit of the differentiable
  problem's step 1. No K row carries either. Lives in: ROADMAP §1 (v0.9 row), §3 compile
  row, §5.3, §5.5, §6; TODO §4 S75, §6 S27; sampled-path.md header, §2, §3.4, §5, §7;
  differentiable-problem.md §3, §4.2, §6, §8.
- **R3 Wave G is ranked by G1; C6 and S76 are v0.4 rows.** G1 (running now) ranks the wave G
  rows into what the GMC714 term needs in December and what moves to v0.4; two weeks hold
  lessons and small features, not a core design. C6 (the general serial chain) moves to
  v0.4 unless G1 shows a second arm is taught in January. S76 (the parametric MPC program)
  moves to v0.4: the MPC band is research lane, its cost and set leaves wait on A2 anyway,
  and nothing GMC714 teaches in December needs it; its names (PP-0) are still ruled in V1's
  step 0 session. Lives in: ROADMAP §1 (v0.3 and v0.4 rows), §3 hybrid row, §5.3, §5.4, §6,
  the Gantt; TODO §4 order, G1, wave V, wave G, §5; parametric-program.md header;
  plans/README.
- **R4 Two rulings sessions, ordered by what each unblocks.** Session 1: the shape of the
  framework (K7's first ask), K7's tier 1 and the exit rule (so the December tag's removals
  follow a stated rule), the S75 contract; it unblocks wave K and SM-0 to SM-2 on November 1.
  Session 2, a week later: S31 with its tick asks, the names of V1's step 0 with S76's PP-0,
  the terminal cost, the differentiable problem's blocking asks; it unblocks wave V. The
  future-policies questions are discussed in session 2 and block nothing. Lives in: ROADMAP
  §5.3, §6 ("Blocking code now, session 1" and "session 2"), the Gantt; TODO §4 order.
- **R5 `tune` is continuous-only in v0.3; the sampled face is V1's step 7b in v0.4.** The
  compiled problem integrates the continuous closed-loop diagram with SM-1's compiled step
  (no second integrator) and refuses a sampled controller with a message naming step 7b.
  Step 7b, after S31's kernel, makes `compile` dispatch on the policy's kind: a sampled
  controller (a digital PID, a held neural policy, MPPI, the RL policies) runs the kernel's
  JAX face, so one `compile`, `evaluate` and `tune` serve both kinds. The tick carrying
  controller memory and the computer's draws from its own seed and the tick are S31's asks.
  Lives in: ROADMAP §5.3 (V1, S31), §5.4, §6; TODO §4 S31 and V1, §5 step 7b;
  differentiable-problem.md §4.1, §6, §8; sampled-path.md §3.4, §7; future-policies.md §6.
- **R6 The missing rows.** The standard stochastic policy as a seeded block (the
  maintainer's direction of 2026-10-22, future-policies.md §5 point 4) is a C5 row in v0.4,
  with the RL heads rebuilt on it and the one-time change of the RL numbers recorded there.
  The "pin the course notebooks on `0.2.0` now" option of repository-structure.md §4 waits
  for V6: course sync is secondary, and the course notebooks keep tracking `main` and
  migrating in the same commit until then. Lives in: ROADMAP §5.4; TODO §5 C5, §6 V6.
- **R7 Which rows ROADMAP §5 names.** ROADMAP §5 names the rows of the architecture and
  course waves; wave D's rows (the T rows, D1–D3, S43, S53, S65–S69, A4, P4) are named in
  TODO only. S68's planned id-agreement test reads this rule. Lives in: ROADMAP §5 intro.

## What did not change

Every open decision of ROADMAP §6 keeps its recommendation and stays the maintainer's: the
shape of the framework, S75's contract and knobs, S31 as a tool, the differentiable
problem's names and asks, S76's family. The step ids are unchanged except S37, absorbed by
S75.
