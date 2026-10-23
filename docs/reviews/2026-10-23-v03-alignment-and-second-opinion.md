# The v0.3 alignment pass and a second opinion (2026-10-23)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** after the rulings of 2026-10-22/23 (R1–R29,
[record](2026-10-22-rulings-session-1.md)), the plan docs that
[continuous-core.md](../plans/continuous-core.md) supersedes were aligned to it and its steps
written as workboard rows (TODO §4, §5). In parallel an independent reviewer, given the
documents and the code but not this pass's conclusions, read the design afresh. This record
holds what both found: the inconsistencies fixed in place, and the points raised for the
maintainer (listed in ROADMAP §6, "Raised 2026-10-23"). Nothing here is ruled.

## 1. What the alignment pass did

- **The plan docs.** differentiable-problem.md rewritten to the design (the detail of V1: the
  target, the implementation rules, the guardrails, the steps); sampled-path.md, cost-params.md
  and randomness.md's evaluator step aligned; the `tune`, `score_trajectory`, `failure_rate`
  and engine-verb leftovers replaced in the other plans.
- **The workboard.** TODO §4: the step model as SM-0 to SM-7 (SM-7, `compute_forced` on the one
  rollout, is new: R17 and R18 had no row), V1's steps 1–6 with gates, A2, S31 after the core,
  S74 deferred, one "Raised" row; §5: V1's step 7 (the draws) had no row and has one.
- **The roadmap.** §1, §3, the gantt, §5.3 (the continuous core replaces waves S and V), §5.4,
  §6 (the two sessions' blocks replaced by what they ruled; the raised points added).

## 2. Inconsistencies fixed in place

| Where | What it said | Now |
| --- | --- | --- |
| continuous-core.md §2 | "gone: … `evaluate_trajectory`" (the ruled scoring verb) | gone are the cost's four members; `score_trajectory` becomes the problem-level `evaluate_trajectory` |
| continuous-core.md §3.1, sampled-path.md §3.6 | a `StepSystem`'s `step` has "the same signature" as the compiled `step(x, u, t, params)` | the `StepSystem` takes the tick `k` (R14); `discretize` binds `t_k = t0 + k·dt` |
| continuous-core.md §3.4 vs R9 | `tf=None`: no `h` | no `h`, and a nonzero `h` warns once (R9) |
| continuous-core.md §7 vs R19 | "the cut at exit becomes the integral to `tf`" listed outside the phase | the single-run scorer changes in v0.3 (V1's step 2); the Monte Carlo re-pin is step 7 |
| continuous-core.md, end-state-api.md D9 | `dt=None`: "0.01 fallback" | the code's fallback (1e-4 s); raised (§3) |
| ROADMAP §5.3, §6, TRL table; TODO C4 | `tune`, "the engine under `tune`", S74 as V1's step 5, session 2 open | the program export (R23); S74 deferred (R25); the sessions ruled |
| TODO S76 row | builds on "S74's backend factory (landed in v0.3)" | S74 is deferred, so it must land before S76 |
| end-state-api.md header | "deleted once the sessions have ruled" | kept as the ledger and the home of D13–D15 and D25–D30 until those are ruled |
| randomness.md RD-D11, RD-D24, RN-5 | the evaluator as a loop of `Simulator` runs on a NumPy test set | superseded in part by R10, R15, R27: the compiled run is the judge, one key realized once |

## 3. Raised by the alignment pass

1. **The v0.3 / v0.4 line cuts through the evaluator** (major). V1's step 2 changes the
   `Evaluation` record and the scoring formula in v0.3; the Monte Carlo evaluator keeps its own
   copy of the old rule (the cut at exit, the held static law) until step 7 in v0.4. For one
   release two scorers give two numbers for one run, against "one formula". The 2026-10-22
   calendar rule (numbers may move behind the pinned release the course runs on) removed the
   reason step 7 waited for the cohort window. *Options:* pull the fold into v0.3 (step 7
   without RN-4's per-block seeds, which can join later under the same verb); or keep it in
   v0.4 and accept the two scorers. *Recommended:* pull it in, if the calendar allows after
   V1's step 5; it is mostly deletion.
2. **`evaluate`'s automatic `dt`** (moderate). The design says "a tenth of the declared
   smallest time constant; 0.01 fallback". The code (`simulation/simulator.py`, the fixed-step
   rule; `DEFAULT_SMALLEST_TIME_CONSTANT = 0.001` in `core/system.py`) gives 1e-4 s on a loop
   that declares nothing: 100 000 RK4 steps for 10 s, per trial. *Recommended:* `evaluate` and
   `compare` require `dt` when no time constant is declared, the message giving the line.
3. **The reference from `x_goal`** (moderate). `r` is filled from `x_goal`, a state; a
   controller whose `r` is an output reference (a PID on `y`) needs the output at the goal.
   *Recommended:* `r = h(x_goal)` when the controller's `r` has the output's size, `x_goal` when
   it has the state's, refused otherwise, the message naming the built-loop way out.
4. **A diverging run inside the program export** (moderate). With no cut and the saturating
   cost wrapper outside the phase, a solver step into an unstable gain gives an infinite or NaN
   objective; SLSQP fails instead of backing off. *Recommended:* the program caps each run's
   `value` at `infeasible_cost` and warns, or `ClippedCost` enters the phase.
5. **`evaluate` on a stochastic problem in v0.3** (minor). With `n_trials = 1`: the nominal
   problem, or the draw of `key`? *Recommended:* refused in v0.3 (the message names
   `MonteCarloEvaluator`), unless item 1 pulls the draws in.
6. **The slots of S31 and S74** (moderate, calendar). The gantt held S31 in November; the
   rulings put its code after the continuous core. S74 is deferred with no slot, and S76 (v0.4)
   needs its backend factory. *Recommended:* S31 in v0.3 only if V1's step 6 lands by early
   December, else the first core row of v0.4; S74 the first v0.4 row before S76.
7. **`criterion_value`**: the proposed name, to confirm.
8. **The `params["dt"]` fallback of `discretize`**: R7 says `dt` is required. *Recommended:*
   drop the fallback with a ledger line (SM-2).
9. **The course audit (G1) has not started** (major, calendar). It is the one input with a hard
   date (the GMC714 term in January) and it fixes what December's two weeks must hold.
   *Recommended:* start it now, beside wave K.

## 4. The second opinion

Pending at the time of this commit; appended when it returns.
