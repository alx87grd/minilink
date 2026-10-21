# The roadmap reorganized around the spine (2026-10-21)

**Ruling of the maintainer, 2026-10-21:** "apply your recommendations and do it", after a
rescan of every plan and the roadmap. Three things were decided: a **v0.4** rung is added
between GMC714 and the freeze candidate; the GMC714 baseline audit (**G1**) runs **now**, in
parallel with the rulings; and the roadmap and workboard are reordered so the sampled spine
comes before the differentiable one. No open decision was closed: every ask kept its
recommendation and stays the maintainer's.

## Why

The analyses of 2026-10-20 and 2026-10-21 (repository structure, core consolidation, the
sampled loop S31, the step model S75, params always an argument) left the roadmap
inconsistent with itself. The rescan found:

- **Order conflicts.** S75 and S31 said "before the differentiable problem's step 0" but sat
  in v0.9, while that problem's steps 0–5 were scheduled "now, beside K1". S37's one
  scannable step is S75's. G1, which writes December's release contract, was scheduled after
  the whole architecture block.
- **Duplicates.** Wave K (K1–K4) and K7 were the same convergence job; C4 stood in v0.3 and
  again in v0.9; RN-4 and RN-5 stood in two blocks; OPW and S74 both build the one optimizer
  backend table; the `discretize` follow-ups and S65's input-hold item are inside S75.
- **Facts in the wrong home.** ROADMAP §6 carried three long arguments (K7, S31, S75), and
  the S31 recommendation had no plan doc.
- **Stale lines.** The optimizer-wiring plan's "v0.2 wave D", MPPI's open
  `RolloutEnvironment` question (answered by S75 and S31 if ruled), the ledger's "no new
  hybrid features before S31 (v0.9)".
- **Scale.** v0.3 held about 25 steps for December, plus the three foundations that belong
  before them.

## What moved

| Item | Before | After |
| --- | --- | --- |
| G1 GMC714 audit | v0.3, first in wave C, after the architecture | v0.3 wave G, now, in parallel with the rulings |
| K1–K4 and K7 | wave K (November) and v0.9 | one convergence wave K in v0.3; K1's inventory is core-consolidation.md §3–§4 |
| S75 the step model | v0.9 foundations | v0.3 wave S, first |
| S31 the sampled loop | v0.9 foundations | v0.3 wave S, after S75 |
| the `discretize` follow-ups | v0.9 syllabus holes | under S75 as its step SM-2 |
| S37 | v0.9, the one step and the integrators' move | v0.9, the integrators' move only (the step is S75's SM-1) |
| V1 steps 0–5, A2, V1 steps 7–9 | v0.3, "now, beside K1" | v0.4, on the sampled spine |
| S74 and OPW-1–OPW-5 | S74 unscheduled; OPW in wave D | v0.4, together, as V1's step 5 |
| RN-3 / RN-4 / RN-5 and AC-7 | v0.3 wave C and the textbook-objects block | v0.4, with V1's steps 7–9 |
| C4 identification | v0.3 wave C and v0.9 | v0.4, V1's `tune` with a data cost |
| A1 fields, A3 geometry | v0.3 | v0.4 |
| S72 MPPI, C5 RL follow-ups, LQGPlanner, S73 | v0.3 wave C | v0.4 |
| C1, C2, C3, C6, S63, S64 | v0.3 wave C | v0.3 wave G, ordered by G1 |
| `step-model.md` | the step model only | renamed `sampled-path.md`, with the sampled loop as its §8 |

ROADMAP §5 opens with the shape the rungs build (the spine, the two step paths, the six
pillars), marked as the proposed answer to K7's first ask. ROADMAP §6 is regrouped by what
each decision blocks: blocking code now, before wave K, before wave G, standing, before v0.4,
before v0.9, before v1.0.

## The order now

(Superseded the same day by the amendment below: the differentiable path's deterministic
half moved into v0.3.)

1. The rulings that block code (ROADMAP §6, "Blocking code now").
2. v0.3 (December 2026): wave G (the course, G1 now), wave K (convergence, November), wave S
   (S75, then S31).
3. v0.4 (winter 2027, between cohorts): the differentiable spine (V1 with S74 and OPW, A2,
   the draws, identification), the textbook objects (A1, A3), the pillars on the spine (MPPI,
   the RL follow-ups).
4. v0.9: what can still break a name, and the trust contract.
5. v1.0: the freeze.

Checked: every workboard row survived the move (the same 80 row headers before and after),
and the docs gate (ruff, the repo contract test) is green.

## Amended the same day: the differentiable path joins the cleanup

The maintainer set the order more precisely: "first next step is core cleaning and full diff
path to cost with nice api, then later December is GMC714 feature adding missing". So the
deterministic half of the differentiable problem moves from v0.4 into v0.3, as wave V:

| Item | After the first pass | After the amendment |
| --- | --- | --- |
| V1 steps 0–5 with S74 and OPW-1–OPW-5 | v0.4 | v0.3 wave V, November into early December, beside wave S (steps 0–2 need neither S75 nor S31; step 3 meets the sampled seam) |
| A2 cost parameters (V1 step 6) | v0.4 | v0.3 wave V, so the path reaches the cost's own weights |
| the GMC714 features (wave G's rows) | December | December, after waves K, S and V |
| V1 steps 7–9, RN-3 / RN-4 / RN-5, AC-7, C4 | v0.4 | v0.4 (unchanged: the evaluator's numbers change once, between cohorts) |

v0.3 is now "a cleaner core and the differentiable path, then GMC714"; v0.4 is "the draws and
the textbook objects". The API comes first in wave V: the differentiable problem's step 0 is a
names brainstorm with the maintainer. November now carries four waves (K, S, V and the audit),
which the timeline in ROADMAP §5 shows overlapping; the order, not the dates, is the
commitment.

