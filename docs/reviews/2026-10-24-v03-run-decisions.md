# The v0.3 overnight run: decisions to review (2026-10-24)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the long run of TODO §4 on `dev-v0.3` (the names and structure, the step model,
the path to the cost, the program export, the optimizer, the closing PID example), asked for on
2026-10-23 with the instruction: where a review would have been asked, decide as the maintainer
would, write the decision here, and continue. Rulings R1–R90 are in
[the rulings record](2026-10-22-rulings-session-1.md).

**How to read it.** §1 is the morning summary. §2 lists every decision taken without the
maintainer, each with the step, the choice, the reason, the alternative and the commit to revert.
§3 is the ledger of intended number changes (R80: the same problem and cost before and after).
§4 is what was not done or is blocked. §5 lists the tests skipped in this container.

**Decision rules used** (in order): the rulings, read literally; the Constitution's precedence
(purity, closure, continuous first, textbook readability); the course notes' vocabulary; the
shortest clear name; an existing pattern over a new one; one path, no alias; when torn, the
reversible and smaller option. Never without the maintainer: removing what no ruling removes,
tuned constants in examples, CONSTITUTION, the demo pass (K4).

## 1. Morning summary

(written at the end of the run)

## 2. Decisions

| Id | Step | Question | Choice and reason | Alternative | Commit |
| --- | --- | --- | --- | --- | --- |
| OD-1 | K2 | R54 deletes `ProblemEnv`, but it is the class behind `Sys2Gym.from_problem`, which the `gymnasium_interface` notebook teaches | Kept as internal machinery of the gym module, off the `interfaces` facade; `to_gymnasium` deleted. The ruling's aim (no public name without a caller) holds | Fold its reset/step into `Sys2Gym` (more branches in the plain env) | — |
| OD-2 | K2, K8-1 | R64 takes names off the root but the teaching-surface test required every band name on the root | The test now reads the root prelude as a subset of the band facades: a band name need not be on the root, and where it is, it is the same object | Keep the test and leave the names on the root (contradicts R64) | — |
| OD-3 | K2 | Folding `NeuralNetwork` into `MLP` changes the neural-controller demo's initial weights (Gaussian × 0.2 → orthogonal) and so its numbers | `MLP(2, 1, hidden=(8,), seed=2)`, every tuned constant of the demo kept; it still trains (batch loss 0.32 → 0.0011). No `scale=` option added to `MLP` for one demo | Give `MLP` a Gaussian `scale=` initialisation to keep the old numbers | — |
| OD-4 | Order | Within the clean-up, which items must precede the new core? | K10 (the package moves) and A6 (the renames) go first, since the path to the cost edits those files; the catalog items (K8's cars, root trim, experimental plants, twins), S32 and K3b touch nothing the new core writes, so they run after it. R68 lands with V1.2, which reads the plants' boxes | Every clean-up item first, as planned (the evaluation pipeline would then land later in the night) | — |
| OD-5 | K10 | Where MPC lives after R66 and how students import it | `minilink/planning/trajectory_optimization/mpc/` (the package moved whole, its `__init__` the MPC sub-facade); `minilink.control` re-exports `ModelPredictiveController` (R66) and drops its `mpc` submodule alias (no alias, R82); the demos import from the new sub-facade, which the teaching-import check accepts | Keep a `minilink.control.mpc` alias module | — |
| OD-6 | G3 | Where the teaching name `ModelPredictiveController` lives, and its frozen constructor | A module of its own, `planning/trajectory_optimization/mpc/predictive.py` (teaching lane); the MPC classes, utilities and plots stay research lane by module. The frozen constructor is today's: `(planner, *, dt_mpc, warm_start=True, verbose=False, t0=0.0, debug=False)`. Students import it from `minilink.control`; it is not on the root (GMC714 does not teach MPC, R48) | Put it on the root prelude; or drop `debug=` before freezing | — |
| OD-7 | SM-1, SM-2 | The plan has discretize 'on the compiled step', but a StepSystem's step must stay an equation path on both backends | The RK4/Euler formula is written once (core/compile/evaluators/stepping.py) and used by the compiled evaluators' step/rollout and by discretize, which applies it to the source's f; compiling the discretized system traces through it. Same numbers, one formula | discretize compiles its source per backend and caches the evaluator (a cache on a System) | — |
| OD-8 | SM-1, V1.3 | Should rollout return (t, xs, signals) as the plan writes? | rollout returns the states (N+1, n) only; t is t0 + k dt, and the recorded signals come after the scan by evaluating the pure output map on the samples (vmap on JAX), which is identical, traceable and costs nothing to callers that do not need them (RL, DP) | Record the signals inside the scan, changing rollout's return type for every caller | — |
| OD-9 | SM-1, SM-6 | The evaluator's port map: rename outputs to h as the plan's four bodies say? | Kept as outputs(x, u, t, params=None), which returns the dict of output ports; h is a system's vector output, and a dict named h would mislead | Rename to h | — |
| OD-10 | SM-7 | The NumPy evaluator has no jit tier; how does a speed-minded caller write one line for both backends? | On NumPy `ev.jit` is the evaluator itself (nothing to compile), so `ev.jit.rollout(...)` runs on both backends; the fixed-step solvers use it (the JAX one stays as fast as the old jitted rollouts) | Raise on NumPy and branch on the backend in every caller | — |
| OD-11 | SM-6 | Move the integrators to simulation/solvers/ (S37) as the row says? | No: the step model's formulas (stepping.py) stay in core/compile, since the compiled evaluator, a core object, calls them and core never imports simulation (dependency law); simulation/solvers/ keeps the solver backends, which now call ev.jit.rollout. The relocation the row asks for has nothing left to move | Move stepping.py to simulation/solvers and have core import it (breaks the dependency law) | — |

## 3. Intended number changes (R80)

| Step | What changed | Problem and cost | Before | After | Relative difference |
| --- | --- | --- | --- | --- | --- |
| SM-7 | compute_forced on rk4_fixedsteps: a sampled input matrix is held (zero-order) over each period instead of interpolated; a callable u(t) is read at the RK4 stage times instead of sampled and interpolated | Pendulum, x0 = [0.4, −0.2], u = 0.5 sin 2t on [0, 1], 20 steps (SM-0 entries forced.rk4_fixedsteps.*) | linear input between samples | held / at the stages | sampled 4.4e-3, callable 9.5e-5 (max over the run, relative) |

## 4. Not done, or blocked

## 5. Tests skipped in this container

No Ipopt (`cyipopt`) in the cloud container: the `ipopt` tests skip and trajectory optimization
falls back to SciPy where a test or demo asks for Ipopt.
