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
| OD-12 | Order (V1) | The path to the cost bundles semantic migrations (V1.2 problem fields, A2 params view, V1.4b Monte Carlo fold) with the new engine; which goes first? | The engine first, additive, in its own module planning/engine.py (ProblemEvaluator, the scoring formula score_run, the new Evaluation record, the evaluate facade, PlanningProblem.compile), so the PID pipeline the run must finish works end to end on a green suite; the semantic migrations then land one at a time against a working engine, and the old MonteCarloEvaluator module is deleted when its callers move (V1.4b). | The plan's order (V1.2, A2, then the engine): every step breaks callers before anything new runs. | — |
| OD-13 | V1.3 | What name does the recorded plant input take in the loop's trajectory (the role 'plant' points at it)? | plant.u: a block output is recorded as 'id:port' (the existing compute_internal_signals_dict convention), so the plant's gathered input gets a different separator and can never collide with a plant output named u; roles = {'plant': {'x': (a, b), 'u': 'plant.u'}}. | 'plant:u' as the plan writes, which collides with an output port named u. | — |
| OD-14 | V1.5 | pe.program(wrt) reads its base point from pe.params, i.e. the objects; how does a caller tune with a free input (the reference r of an error-driven PID) at a value other than its port's nominal? | program(wrt, bounds, key, n_trials, *, params=None): params overrides entries of pe.params at the base point, as in pe.evaluate(params); the leaves not tuned keep those values and z0 reads the tuned ones from the same tree; z0 is also in program.metadata['z0']. | Require the caller to set loop.inputs['r'] nominal before program(): state on the loop the caller never built. | — |
| OD-15 | S74 | The preset names and the class table: algorithm names with the scipy_* keys as aliases (D23), and which methods each class admits? | No aliases (R82): the new presets are named by algorithm, gd, adam and l-bfgs (SciPy's L-BFGS-B), and the existing keys scipy_slsqp, scipy_trust_constr and ipopt stay as they are; the class (unconstrained, bounded, constrained) is read from g, h and the finite box faces; method='auto' picks l-bfgs for the first two and scipy_slsqp for constrained; gd and adam refuse constraint rows. The default stays scipy_slsqp (§4.5). Differential evolution and scipy_bfgs are not added (no caller); the stochastic class waits for a program that declares itself stochastic. | Rename the scipy_* keys to algorithm names: a sweep of the trajopt planner, demos and baselines with no gain this release. | — |
| OD-16 | S74 | The plan writes Optimizer(program).solve() with no z0, but MathematicalProgram stores no initial guess | Optimizer's z0 is optional: unset, it reads program.metadata['z0'], which pe.program fills with the packed current leaves; a program without one is refused with that message. The program stays a pure description (metadata is its diagnostic slot). | A z0 field on MathematicalProgram: solver state on the pure NLP, which its docstring rules out. | — |
| OD-17 | S74 | The first-order backend: move RL's Adam 'unchanged', but the backend steps a NumPy z | Adam and clip_by_global_norm moved verbatim to optimization/optimizers/first_order.py (the RL algorithms import them from there; the RL baselines run unchanged); FirstOrderOptimizer jits Adam.update once per solve (eager JAX dispatch cost 37 ms a step) and keeps gd in NumPy, so gd needs no JAX. Adam's eps in the Optimizer preset is 1e-8 (the textbook value; RL keeps its 1e-5 default), max_grad_norm None. | Write a NumPy twin of Adam for the backend: two copies of one formula. | — |

## 3. Intended number changes (R80)

| Step | What changed | Problem and cost | Before | After | Relative difference |
| --- | --- | --- | --- | --- | --- |
| SM-7 | compute_forced on rk4_fixedsteps: a sampled input matrix is held (zero-order) over each period instead of interpolated; a callable u(t) is read at the RK4 stage times instead of sampled and interpolated | Pendulum, x0 = [0.4, −0.2], u = 0.5 sin 2t on [0, 1], 20 steps (SM-0 entries forced.rk4_fixedsteps.*) | linear input between samples | held / at the stages | sampled 4.4e-3, callable 9.5e-5 (max over the run, relative) |

## 4. Not done, or blocked

## 5. Tests skipped in this container

No Ipopt (`cyipopt`) in the cloud container: the `ipopt` tests skip and trajectory optimization
falls back to SciPy where a test or demo asks for Ipopt.
