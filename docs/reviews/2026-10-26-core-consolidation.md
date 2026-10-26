# The core consolidation: v0.3 the complete core (2026-10-26)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** many topics were iterated at once between 2026-10-22 and 2026-10-26. The vision
of the core ended up spread over ROADMAP, TODO, QUESTIONS, seventeen plan docs (five of them
with tentative draft sections) and the rulings records. The maintainer asked for that material
to be consolidated around the core's pillars and re-planned in the natural order of the code.
This record holds his direction and how the docs were reorganized. **It rules nothing else:**
the contradictions and gaps the consolidation found became open questions, not answers.

## 1. The maintainer's direction (2026-10-26, in session)

- **v0.3 is the complete core.** One architecture from the plant to the evaluation and the
  optimizer, supporting stochastic problems and the hybrid sampled loop, built in the natural
  order of the code. Versions and class releases do not shape it.
- **v0.4 is for features built on the core**, not intertwined with it.
- **v0.9 stays** the freeze candidate.
- **v1.0 comes when the API feels mature and stable.**
- **The courses:** class material stays on `main` until the new v0.3 core is ready.
- **The core's pillars:**
  - **The hybrid loop: `HybridDiagram` and `Computer`, two runners.** A pure step diagram for
    analysis, and a clock orchestrator for realtime and compute efficiency, similar to today.
  - **One discrete judge** (his idea of 2026-10-26, differentiable-problem.md §9).
  - **Stochastic problems.**
  - **The generic parametric `MathematicalProgram`, finished all at once.** It is compiled with
    an explicit, optional `params` argument (for the MPC: `x0`, the plant's parameters, the
    cost's parameters, and so on), so nothing recompiles, for speed.
  - **The stochastic program, which is related.** The plan is all deterministic, but the seed is
    a parameter, so a stochastic program is solved over multiple seeds.
  - **A native solver and an MPC using it, as a proof of concept.** A simple fixed-step solver,
    to demonstrate the pipeline, not a state-of-the-art JAX solver. Improving the optimizers and
    the MPC is for v0.4.
  - **Fields.**
  - **The seeded stochastic policy.**

  Eight items; core-architecture.md groups the parametric and the stochastic programs as one
  pillar, as the maintainer related them, so it counts seven.

## 2. What the direction re-assigns

- **R86** ("v0.3 stays continuous"; the sampled loop, the draws and the parametric MPC solve in
  v0.4) is superseded on the release: those rows are v0.3's core. The rulings that shaped them
  (R13, R14, R104, R105, R123, R26, R121) stand.
- **G3's January gate** is lifted: the GMC714 names are pinned when v0.3 merges.
- **The rung of the fixed-iteration MPC** (a QUESTIONS entry) is answered and the entry left
  QUESTIONS: a proof of concept in v0.3 (NS-1, NS-2's fixed-`K` form, NS-8, NS-9), the rest of
  the native-optimizer plan in v0.4.
- **The draft visions named pillars keep their status.** The judge (differentiable-problem.md
  §9), the two runners (sampled-path.md §8.1–§8.5) and the stochastic program
  (native-optimizers.md §7) are now pillars. Their refinements and the CONSTITUTION amendment
  stay open. The maintainer's 2026-10-25 leaning on programs is now his direction: every program
  is deterministic given its parameters, the seed one of them; solving over seeds.

## 3. How the docs were reorganized

- **`docs/plans/core-architecture.md` (new)**, the first read for the core. For each pillar and
  the two foundations under them (every object pure, every number a parameter; the step algebra
  first-class) it gives:
  - the vision, with its status;
  - what the code does today;
  - the open points;
  - the steps.

  It also holds the proposed build order (phases 0–8, the agent's, not ruled; first labelled
  P0–P8, renamed the same day because P4, P8 and P10 are workboard ids) and the core against
  the features.
- **`end-state-api.md` deleted.** Its open entries moved verbatim to the plan docs they shape:
  - D13–D15: sampled-path §8.6;
  - D25: parametric-program §2.1;
  - D26–D28 and D30: mppi §10;
  - D29: future-policies §5.1.
- **TODO:** its rows moved verbatim, by line range:
  - §1 the core by pillar;
  - §2 the clean-up lane;
  - §3 the standing work;
  - §4 the v0.4 features;
  - §5 v0.9 and v1.0;
  - §6 Later.

  New rows were written only where a pillar had none (the judge, the two runners, the stochastic
  program, the proof of concept, V1's steps 8 and 9), their text lifted from the plan docs.
  Landed bullets and checked items left the file, and the duplicate D2 line merged into S31.
- **QUESTIONS:** its entries were re-bucketed by pillar, kept as written, with their links
  re-pointed. A direction line was added where the direction answers part of an entry.
- **ROADMAP:**
  - §1's v0.3 and v0.4 rows;
  - the ledger's "Next" pointers;
  - §4.3's gate reworded;
  - §5's path rewritten around the pillars, the Gantt chart replaced by the phase graph. The
    history of v0.3's first phase is kept verbatim in §5.3.
- **The plan docs:** a rung line each, pointers to their pillar, and one-line "open point"
  markers where a passage conflicts with a ruling or another doc. The passages themselves are
  unchanged.

The coverage check: every step id and plan-local id that appeared in the old docs (289) appears
in the new ones. The only exceptions are landed history (R16, R40, R117, SM-3, SM-4, SM-5, SM-7),
kept here in the rulings records and the run's log.

## 4. The open points the consolidation recorded (new QUESTIONS entries)

1. Which release each borderline row takes (S29, S70, the realtime runner's re-base, the hybrid
   orchestrator's adaptive plant integration, the RL heads' rebuild).
2. The same-tick semantic of a step diagram (two today).
3. Composition operators on step systems.
4. The discrete problem's failure rule, hold and scope (shooting's score, the planners' policies,
   S63).
5. A seeded policy's draw convention: the held sample or the tick.
6. `compute_trajectory` on a hybrid loop: which runner by default, and how dense a plant
   trajectory.
7. Whether `problem.compile` refuses the stateless MPC until the sampled face lands.
8. One home for `as_cost` and the shaping (three plans).
9. mppi.md's design text against R104, R107 and R123.

**The order review.** Before the order was written, the agent asked a planning subagent, in
the same session, to stress-test it against the code. Its points are in core-architecture.md §4
and QUESTIONS.md:
- R123 lands with the MPC as a step block;
- S70 is wanted before S66;
- S29 is a core gap;
- the discrete problem needs one failure rule and one hold;
- the `Trajectory` fold must allow dense substeps;
- the number changes are grouped for one re-pin.

**The double-check (same day).** Three read-only audits ran on the result: nothing lost,
accuracy against the sources and the code, and consistency across the docs. They found no lost
ruling or step. They did find citation errors, status blurs, stale dates and labels, and the
phase-id collision; one follow-up commit fixed them.

## 5. Found while consolidating, not changed

- DESIGN §6 says `Sys2Gym.from_problem` draws the problem's parameters and disturbances; the code
  applies no draw (its docstring says so). The rest of RN-5 closes it.
- DESIGN §6's stochastic figure draws the distributions' parameters as leaves of `θ`; today they
  are attributes, not leaves. RN-2 closes it.
- TODO T2 says `TrajectorySource.h` cannot trace. That has been stale since OD-30, and the row is
  annotated.
- **The roadmap's dated plan, kept here as history** (the Gantt chart of ROADMAP §5 before
  2026-10-26, indicative dates, "the order is the commitment, the dates are not"). These
  durations were the only sizing estimates in the docs.
  - Rulings sessions 1 and 2 (R1–R29), 2026-10-22, 2 days, done.
  - The GMC714 audit G1, 2026-10-23, done.
  - Wave K's renames and moves (K1–K3, K10, A6), 2026-10-23, 2 days, done.
  - Wave K's rest (K4, K8, K9), S32 and G3, from 2026-11-01, 3 weeks.
  - The step model S75, 2026-10-23, 2 days, done.
  - V1's steps 1–6 with A2, S74 and S77, 2026-10-23, 2 days, done.
  - The v0.3 tag, 2026-12-21.
  - The sampled loop S31 and OPW, from 2027-01-04, 3 weeks.
  - V1's steps 7–9, the draws and C4, from 2027-01-04, 4 weeks.
  - S76 and the wave G rest, from 2027-01-04, 4 weeks.
  - The V1 sampled face, A1 fields and A3, after the draws, 6 weeks.
  - MPPI, the C6 serial chain and the RL follow-ups, after the draws, 5 weeks.
  - The v0.4 tag, 2027-04-30.
  - The v0.9 foundations and the trust contract, from 2027-05-03, 8 weeks.
  - v1.0, two cohorts then the freeze, from 2027-07-01, 20 weeks.
