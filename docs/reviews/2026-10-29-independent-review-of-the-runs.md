# Independent review of the v0.3 core run and the night run (2026-10-29)

**Audience:** Prof. Alexandre Girard (Maintainer).
**What was reviewed:** the 155 commits of `dev-v0.3` between `9be4f023` (2026-10-27) and
`1df28c76` (2026-10-29 06:54): the v0.3 core run ([its log](2026-10-29-v03-run-log.md), RD-1 to
RD-69), the zero-delay MPC session ([its record](2026-10-28-zero-delay-mpc.md)) and the night run
([brief](2026-10-29-night-run-brief.md), [log](2026-10-29-night-run-log.md), N-1 to N-22), against
the CONSTITUTION, RULES §5, DESIGN §4–§6 and the rulings R124–R167. Six independent read-only
reviewers took one slice each (the sampled loop and time; compile once; the one score; the
planner's face, the MPC and MPPI; the math objects and the draws; the documents as contract),
each reproducing its findings in this container; the lead consolidated, fixed what was clearly a
mistake, and left the design choices to the maintainer (QUESTIONS.md). The reviewers' reports
and reproduction scripts stayed in the session's scratchpad; this record keeps their findings.

## 1. The verdict in one paragraph

The architecture the rulings describe is what landed: the time contract of R158 holds under probe
on every face, the two run verbs agree bit for bit at the ticks on both backends, the one-score
formula gives one `J` through the face, the judge, the RL return and the planner's held law (to
1e-16), every core object reads its `params` with a finite gradient through its float leaves, the
origin convention holds in every planner's `z`, MPPI matches Williams et al. (2017), and the
gates are green (ruff, the full suite, the regression gates with the CI flags, the docs build,
the seeded baseline byte-identical before and after this review's fixes). The runs also left
about twenty real defects at the edges, most of them small and fixed here (§2), a handful of
contract tensions the maintainer must rule on (§3, now in QUESTIONS.md), a layer of stale
documentation now brought back to the code (§4), and two process gaps (§5).

## 2. Fixed in this review (one commit)

Every fix was checked against the seeded baseline (`benchmarks/v03_baseline.py`, 116 entries,
byte-identical before and after), the touched modules' tests and the full suite; each has a
regression test unless noted.

| Area | Defect (reviewer's id) | Fix |
| --- | --- | --- |
| Host clock runner | `t_eval` overshot the period's end by an ulp for ordinary `(Δ, S)` pairs (`(0.05, 3)`, `(0.1, 6)`, 39 pairs with `S ≤ 100`), so `solve_ivp` refused (A2) | the grid's last point is the period's end exactly |
| Dense run of a lone sampled plant | integrated twice per period (b45f9198 fixed the loop evaluator only) (A5) | the path's last row is the tick |
| Read-back of a ticks run | `plant_run_of` assumed a dense grid, so `loop.compute_rollout(); loop.animate()` animated a wrong held input, silently (A1) | the samples per period are read from the run's grid |
| A recorded run of no period | `compute_trajectory(tf=t_start)` on a loop crashed unpacking the records (A3) | a refusal naming what to pass |
| `discretize` of a sampled law with no period | died formatting `None` (A7) | a refusal naming `law % dt` |
| The MPC's `verbose` | crashed a traced native MPC in both forms (the guard tested one value) while the docstring promised silence (D-1) | the line prints only when every value it prints is concrete |
| The planner's compiled face | a partial `params` on the NumPy face (and `initial`, `solution`, `pack` on any backend) merged onto the live problem while the origin read the snapshot: the defect 7f2a1080 claimed to close, open on those paths (D-2) | the partial tree merges onto the compiled tree |
| The MPPI face | read the problem live where the trajectory-optimization face snapshots (D-7) | snapshots at compile, as the other face does |
| `NoiseSource` without a seed | ignored a moved tree for a law without a `"mean"` key (`Uniform`, `Particles`) (E1) | `law.mean_at(params)` |
| `Set.with_params` / `InputSet.with_params` | mutated and returned `self`; raised `FrozenInstanceError` on a frozen set (E2; the night brief's item 7, not landed and not logged) | a copy; a set without numbers refuses |
| `IntersectionSet.with_params` | a partial write to one member died with a bare `KeyError` (E6) | a member left out keeps its numbers; an unknown member is refused |
| `nominal_trajectory` with a source policy | the held table indexed by absolute time, so `t_start ≠ 0` ran off the table (C1) | indexed from the grid's first sample |
| The RL environment | the reward was `nan` when a step blew up (C2) | the stage cost of a non-finite step counts for nothing; the failure pays the price |
| The Gymnasium view of a problem | ignored `problem.t_start` (C3) | starts and ends the episode from it (no test: the Gym tests need `gymnasium`, installed here but the view is an external adapter) |
| The JAX program evaluator | the frozen form stayed on the values compiled first after an in-place edit of `program.params` (B1) | the frozen form is used only while the bound tree is the compiled one |
| Parameter leaves | a Python list in `params` crashed every JAX convenience call with a bare NumPy error (B5) | a numeric list digests and canonicalizes as the array it is |
| The compile cache's key | `NOT_EDITS` and `NOT_STRUCTURE` disagreed on `own_x0`, so a native law's start sweep recompiled per start (B6) | the two lists agree (no test: a recompile count) |
| RULES 5.24 | a ruling id in a library docstring (A8); ten ruling ids in test docstrings and comments (C11) | removed |
| RULES 5.3 | `self.` in the math lines of the two faces' `shift`, `command`, `prediction`, `pack`, `planned_trajectory` and an unexplained `1e-9` (D-15, D-16); the Euler stage cost's equation on a `return` with `self.dt` in it (C9); an expression on the `return` of `QuadraticField.value` and `EllipsoidSet.sample` (E13) | bound and named; byte-identical |
| Docstrings | `tree_digest` and `DeviceTree` said JAX leaves are digested by identity where the code reads their bytes (B7); `plant_run` said a read-back re-solves (D-12); the fields' wrappers stated the parameters gap as a design (E5); the RL collector restarted "at time zero" (C12) | corrected |

## 3. Design questions raised (QUESTIONS.md, in this order)

1. **The compile cache against CONSTITUTION §4.4** (a module-level registry of compiled
   evaluators; the night brief itself said it sits uneasily there; a module global read by `f` is
   a third blind spot beside KNOWN_ISSUES' two).
2. **The RL training reward**: the one score `−c_k` (today) against a training reward `−g·dt`
   recorded beside it, with the cart-pole evidence (9/12 → 4/12 at 2M; 3/8 at 3M).
3. **Value iteration's `alpha=` on a discounting problem** gives `Σ αᵏ ∫ e^{−ρτ} g dτ`, two
   discounts; invisible under Euler (why N-5 called it second order), measured under RK4
   (`J` 11.19 against 12.28 for the same `alpha`).
4. **The MPC**: the default form (`delay=True`) that no student-facing artifact uses (14 of 16
   constructions pass `delay=False`); what `lookahead` means without a delay (the zero-delay and
   pure forms predict under a command they do not apply); the print inside `update` that RD-51
   rejected and 44129d91 landed.
5. **One tree shape for every planner**: MPPI has `method` and `seed` but not the cost's leaves
   (so `controller.cost.Q` tunes nothing on an MPPI MPC where DESIGN promises it); trajectory
   optimization the reverse; the RL planner no tree.
6. **What marks the plant of a sampled loop**: the words say a role, the code reads a solver kind
   (`resolves`), so `(pid % Δ) >> discretize(plant, Δ)` is refused and a lone `pid % Δ` runs
   `solver="scipy"` as an adaptively integrated plant.
7. **One failure price or one per tool** (193.8, 198.7, 314.3 on one pendulum; a discretized
   problem answers a fourth).
8. **`%` on a native law copies, on a view shares**; the copy also shares its port tables with
   functions bound to the unclocked original (a standard `y` port on such a law fails under `%`:
   a bug left for the ruling, A6).
9. **Who owns `hold_x0`**, the start of a sampled loop's held outputs (a fourth owner beside
   R124's three).
10. **Which lane `minilink.planning` is for MPPI** (ROADMAP §2 makes every band facade the
    teaching surface; N-14 called it the provisional band).
11. **Whether the compile probe should cover port functions** (today only an `h`-computed
    output is probed; KNOWN_ISSUES' own reproduction still compiles silently).

The fields' parameters question (N-19) stays first in the file, with the review's worked harm
(a box as `X` and as `X.as_cost()` has two homes).

**For the maintainer's confirmation, not questions:** the CONSTITUTION's five amendments
(`d56dd7a7`, "the wording open to his later review") and "`Field` (planned)" in §2, which
landed; the one-ulp move of the region-of-attraction baseline after Lyapunov's `P` was
symmetrized (the night ledger records it, the committed JSON was not re-pinned; the last bit is
host dependent, so the re-pin is for the maintainer's machine); ROADMAP §2's breaking-changes
ledger, which stops at 2026-10-24 and records none of the two runs' renames and removals
(`compile_backend=` → `backend=`, `optimizer_method=` → `method=`, `t0=` → `t_start=`,
`NeuralPolicyController` → `NeuralPolicy`, `Discretization` → `SampledSystem`,
`StochasticPlanningProblem` and the hybrid runtime gone, `as_constraint` → `as_set`,
`bind` → `mount`, the MPC's constructor, `planner.solve(guess=)`, `Sys2Gym(x_start=)`,
`realize` one tree, `connect` refusing a rewire; §2 is the maintainer's); the README's flagship
"one problem, every planner" table, whose value iteration stops at its iteration cap and whose
three plans miss the goal (the same before the runs; student-facing).

## 4. Documentation brought back to the code

DESIGN §4–§6 (the RL reward written both ways; `problem.compile` "refuses a sampled controller";
`realize` returning a tuple; `StepSystem` caching `self.rollout`; the hybrid simulator as a
runner; the R140 exceptions naming closed gaps; the online façade "planned as S76"; the package
map without the new modules); KNOWN_ISSUES (the rewire entry whose refusal landed; the probe's
reach); ROADMAP §3 (MPPI at TRL 1 "two scripts" while the planner landed; the probe's claim;
`NoiseSource`; the MPC's three forms); core-architecture.md's "Today" paragraphs of §1, §2, §3,
§4, §5, §8 and its §12 row, all describing the pre-run code; TODO's stale ask on the course
notebooks (ruled 2026-10-29) and its three "MPPI is v0.4" lines; mppi.md (`accepts_stochastic`
contradicting the code; the "Today" sentences); planner-and-mpc.md (`warm_start` listed as gone
while it is the zero-delay form's switch); compile-once.md (the attribute-fingerprint key N-2
replaced); plans/README.md; the night log's empty textbook-check section, filled from this
review. Left for a housekeeping pass: the fifteen TODO §1 rows whose text says everything landed
and are still open; the "Today" bullets of sampled-path.md, differentiable-problem.md,
randomness.md, parametric-program.md, native-optimizers.md, fields.md and
automation-by-convention.md; API pages for `SampledSystem`, `SampledDiagramSystem`,
`RolloutEnvironment`, the planner's face, `core.params`, the compile cache and the host clock
runner.

## 5. Process findings

- The night log's §6 "Textbook check" was empty, although AGENTS.md makes the statement the
  acceptance bar and the night touched math in six commits (filled here; the offences it would
  have caught, `self.` in the faces' math lines and an equation on a `return`, are fixed).
- Two of the night brief's items were reported landed without landing: item 7 (`with_params`
  returning a copy, no shared class-level `params = {}`: the copy is in now; the shared class
  attribute on `Set`, `InputSet`, `Field` and `Distribution` remains, a user set that writes
  `self.params["r"]` edits every set's defaults, E3, a one-line hazard left for the maintainer's
  choice of pattern) and the claim of 7f2a1080 (the face's snapshot, open on four paths).
- The RD-34 sentence "Adam on the NumPy loop stays jitted when JAX is importable" stopped being
  true with the compile-once commit 51899e51 (the plain loop differs from the jitted one by one
  ulp; the baseline's Adam row runs on JAX so it could not see it) (B2); N-18's reason for
  keeping `Adam` beside `AdamLaw` ("changes its numbers") does not hold for the forward pass,
  which is byte-identical over 50 steps (B3): the two are one owner's worth of code (RULES 5.6).
- KNOWN_ISSUES' cache entry names two blind spots; a module global read by an equation is a
  third (B8).

## 6. Findings left open (not fixed, not design questions)

- A6: a native sampled law's `%` shares its port tables (bug, waits on question 8).
- A10: `compute_rollout` takes no `record=` (N-1 said "every run verb").
- A11: the hold slot in `traj.x` is one period behind the recorded signal and its label reads
  like the signal.
- A12: a bare static block in a hand-built nested sampled box is read at the loop's base.
- D-3/D-4: `lookahead` in the zero-delay and pure forms; the traced first guess falls back to
  the planner's own start (`has_values` is false under a trace), so a traced zero-delay loop's
  first tick differs from the host's.
- D-5: MPPI's tick key is on the planner's `dt`, not the loop's period (a loop faster than `dt`
  draws one key twice).
- D-8: the MPC reads inside a collocation `z` (`pinned_shift`, `predicted_start`) against
  planner-and-mpc §1; viz re-derives the layout under a second `require_state_knots`.
- C4: `RolloutEnvironment` refuses a `StepPlanningProblem` with an `AttributeError`, although
  DESIGN says the environment reads only the face.
- C5–C8: the price's owners (question 7); the planner's evidence scored by the trapezoid (a
  second `J`, 7e-4, that nothing "says so" about); three copies of the Euler stage-cost rule.
- C10: `StepCost.params` is a property that writes on read (RULES 5.7).
- E3, E7, E8, E9, E11, E15–E19: the shared class-level `params`; `problem.x_start` and
  `X0.point` two copies that `problem.x_start = v` desyncs; `hold_x0` (question 9); the RL heads
  tie to `GaussianPolicy` at the level of `ε` only; a law's `params` is a view so an edit is
  lost silently; `BoxSet.sample` checks the frozen box, not `params`; the cipher's 24-bit
  normal tails and crc32 stream names, unstated.
- B9: `function_identity` is by object, not `__code__`, as N-2 says.
- F-30, F-22 and the plan docs' "Today" bullets (§4 above).

## 7. Verification of this review's changes

`ruff check .` and `ruff format --check .` clean; `benchmarks/v03_baseline.py` captured twice
before the fixes (`cmp` identical) and once after (`cmp` identical to the first); the touched
modules' tests and the full `pytest` on `tests/unittest` green; "Regression gates, CI flags"
green before the fixes (unchanged paths after: the baseline is the gate's own numbers);
`sphinx-build -W` green after the docstring edits. Every math line this review edited was read
beside `planning/dynamic_programming/dp.py`: the fixes bind the numbers above the equation, keep
one named assignment per line and return a name.
