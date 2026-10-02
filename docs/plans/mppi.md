# Path integral control (MPPI) in Minilink

**Status:** design draft (2026-10-02), for the maintainer's ruling on the asks of §10.
**Lane:** research lane first (`examples/experimental/`), provisional band with the MPC
block once MP-5 lands, teaching-surface decision at V3 (v0.9).
**Rung proposed:** prototype in v0.3 wave C (S72 on the workboard), hardened before the
v0.9 freeze; see §9.

MPPI (model predictive path integral control, Williams et al. 2016–2018) is the
sampling-based MPC modern robotics stacks run: no gradients, no NLP, thousands of
rollouts of a nominal input sequence perturbed by noise, a softmin over their costs, a
weighted average as the new sequence, one step applied, shift, repeat. Minilink already
owns every piece it needs — a batched JAX rollout, the stochastic problem's draws, the
scoring contract of an episode, a receding-horizon block — so the work is a planner of a
few screens plus one contract narrowing in `control/mpc`.

## The design in one screen

- **A planner, not a controller.** `PathIntegralPlanner(problem, dt=, n_samples=,
  temperature=, noise=)` is a trajectory-family planner beside
  `TrajectoryOptimizationPlanner`: `solve()` returns a `PlanningSolution` whose policy is
  the open-loop schedule `u = pi(t)` and `solve_trajectory_from(x0)` is the online tick.
  Receding horizon is the existing wrapper: `ModelPredictiveController(planner, dt_mpc=)`
  — that composition *is* MPPI, as `ModelPredictiveController(trajopt)` is MPC.
- **Both problem classes.** A `PlanningProblem` gives the textbook algorithm: noise on
  the inputs only. A `StochasticPlanningProblem` adds a plant realization per sample —
  parameter draws and disturbance draws from the problem's own distributions — so the
  softmin scores paths under model uncertainty, not only under input noise. The
  deterministic twin is byte-identical to the stochastic one with no draws (the test).
- **The rollout is the one Minilink already has.** `rollout_batch` /
  `rk4_integrate_zoh_trace` vmap `K` held-input RK4 rollouts in one jitted call; the
  `RolloutEnvironment` owns what a step costs (exit from `X` charged `infeasible_cost`,
  `h` at the horizon, disturbance draws on their ports). MPPI reads those; it does not
  redefine them.
- **JAX first, the ladder later.** The prototype is JAX-only (one `jit(vmap(scan))` per
  tick). The three-backend ladder of `dp.py` (`loop` / `numpy` / `jax`) is the teaching
  form and comes with the teaching-surface decision, not before.
- **One narrowing in `control/mpc`.** The block today reads trajopt internals
  (`planner.transcription`, `last_optimization_result.z`, `compile_parametric_program`).
  It needs three planner verbs instead: `decision_dimension()`, `solve_trajectory_from()`
  returning a record that carries `z`, and `warm_start_guess()`. That is the T6
  conversation already on the workboard, done with its first second consumer.

## 1. The code as it stands

What exists, what it is coupled to, and what MPPI reuses.

**The receding-horizon block is not planner-agnostic.** `ModelPredictiveController`
(`minilink/control/mpc/controller.py`) accepts "a duck-typed
`TrajectoryOptimizationPlanner`", and `validate_mpc_planner` (line 740) requires
`compile_parametric_program`, `has_parametric_program`, `solve_trajectory_from`,
`planner.transcription.options.n_steps`, `planner.options.compile_backend` and
`planner.transcription.decision_dimension(problem)`. The tick latch (`MPCTickLatch`,
line 597) reads `planner.last_optimization_result.z` after every solve and builds the
warm start through `mpc_warm_start_guess`, which unpacks `z` with
`planner.transcription.unpack`. So the block takes exactly one planner family. The
workboard already carries the fix as T6 ("one record per MPC tick, `Command` and
`MPCTickSolve` folded into the `PlanningSolution`") and D3 ("MPC port computes drop
`params`").

**The planner base.** `Planner` (`minilink/planning/planner.py`) gives `solve`,
`solve_trajectory`, `solve_trajectory_from(x0)`, `last_solution`, `open_loop_policy`
(a `TrajectorySource` on the plant's action port), `evaluate` (the Monte Carlo
evaluator on the problem's draws) and the `accepts_stochastic` flag that silences the
"deterministic planner on a stochastic problem" warning. A new planner is one class
that overrides `solve` and `solve_trajectory_from` and returns a `PlanningSolution`
with its own frozen record (`TrajectoryOptimizationRecord`, `RiccatiRecord`, … each
with `success` and a one-line `str`).

**The batched rollout.** `JaxDynamicEvaluator.rollout_batch(x0s, u_sequences, dt=,
params=)` (`minilink/core/compile/evaluators/jax_evaluators.py`, line 348) rolls `B`
RK4 / zero-order-hold trajectories in one `jit(vmap)`: `u_sequences` may be `(B, N, m)`
(one sequence per member — the MPPI samples), and `params` may carry a leading family
axis on any leaf (a parameter draw per member — the stochastic variant). The pre-JIT
twins `rk4_integrate_zoh_trace` / `_trace_p` compose under a caller's own `jit` and
`vmap`, which is what a planner that also samples and weighs on device wants. Measured
2026-09: 1000 rollouts × 1000 RK4 steps in 27 ms (ROADMAP §3); an MPPI tick is
`K = 1024` × `N = 40`.

**The stochastic problem and the episode semantics.** `StochasticPlanningProblem`
(`minilink/planning/problems.py`) owns `sample_x0(key)`, `sample_params(key)` (one
`{name: value}` override of `sys.params`, dotted names into a diagram) and
`sample_disturbances(key)` (one `{port: value}` draw held on the port), plus the
`criterion` (`"expectation"` / `"worst_case"`). `RolloutEnvironment`
(`minilink/planning/reinforcement_learning/environment.py`) is "a planning problem
seen by a learner": `step(x, t, u, key, params)` integrates one control period with the
input held (`rk4_step_trace_p`), scores it `-g dt`, charges `infeasible_cost` when the
state leaves `X`, charges `h` at a finite horizon, and fills the disturbance ports from
`key`. DESIGN §6 names it the one owner of what an episode means, shared by the RL
collectors, the tabular learners and the NumPy Monte Carlo trials. The randomness plan
(`randomness.md`, RN-4) will turn `disturbances` into signals realized per episode by
`realize(key)`; the env's `reset(key)` / `step` keep their shape.

**The MPC exemplar.** `examples/demos/mpc/mpc_car_minimal.py` is the whole user story:
a `PlanningProblem` with a quadratic cost, a planner, `ModelPredictiveController(planner,
dt_mpc=)`, `mpc @ sys`, `compute_trajectory`, `animate` with `mpc_animation_overlays`.
MPPI should read the same with one line changed.

## 2. The core math (what the planner body shows)

The textbook form, Williams et al. 2017 ("Information-theoretic MPC", the
`ModelPredictiveController` of their paper being the receding-horizon loop). Symbols as
they will appear in the body, one named line each.

Discretize the horizon `tf` into `N` control periods of length `dt`, inputs held. The
nominal sequence is `U = (u_0, …, u_{N-1})`, each `u_k ∈ ℝ^m`.

1. **Sample.** For `K` samples draw input noise `ε^k = (ε^k_0, …, ε^k_{N-1})`,
   `ε^k_t ~ N(0, Σ)`, and form the perturbed sequence `V^k = U + ε^k`, clipped to the
   input set `U_box` (the box of `problem.U`).
2. **Roll out.** `x^k_{t+1} = Φ(x^k_t, v^k_t, t_k; θ^k, w^k_t)`, the RK4 step of `f`
   over `dt` with the input held, from `x^k_0 = x_0`. On a deterministic problem
   `θ^k = θ` and `w^k_t` is the nominal value of the disturbance ports for every `k`.
3. **Score.** The path cost under the problem's contract:
   `S^k = Σ_t g(x^k_t, v^k_t, t_k) dt + h(x^k_N, t_N) + c_X^k`, with `c_X^k` the
   problem's `infeasible_cost` charged once when the path leaves `X` (the exit rule of
   `RolloutEnvironment`), plus the information-theoretic control term
   `λ (1 − α) Σ_t u_tᵀ Σ⁻¹ ε^k_t` (`α = 1` turns it off; the default of the paper is
   `α = 0`, the term on).
4. **Weigh.** `w^k = exp(−(S^k − S_min) / λ) / Σ_j exp(−(S^j − S_min) / λ)`, the softmin
   with temperature `λ`; `S_min` keeps the exponent finite.
5. **Update.** `U ← U + Σ_k w^k ε^k`, optionally smoothed along `t` (Savitzky–Golay in
   the paper; a later option, not the prototype). `n_iterations` repeats 1–5 on the
   same `x_0`; the default is one pass per tick, the MPPI of the literature.
6. **Apply and shift.** The tick's output is `U`; the receding-horizon block applies
   `u_0`, and the next tick starts from `U ← (u_1, …, u_{N-1}, u_{N-1})`.

The record of a solve carries the things a user tunes against: `S_min`, the weighted
cost `Σ w^k S^k`, the effective sample size `1 / Σ (w^k)²` (a few means `λ` is too
small, `≈ K` means too large), and the fraction of samples that left `X`. `success` is
"the nominal rollout of the returned `U` stays in `X`" — the same meaning
`TrajectoryOptimizationRecord.success` gives to feasibility.

**What is standard and what is not.** The derivation of MPPI assumes the plant's
stochasticity enters through the input channel: the noise injected on `u` *is* the
model's noise, so the textbook algorithm on a `PlanningProblem` needs nothing else.
Sampling the plant's own uncertainty per rollout is the literature's robust branch:
Tube-MPPI (Williams 2018) tracks the nominal plan with an ancillary controller,
Robust MPPI (Gandhi et al. 2021) augments the sample set, Risk-Aware MPPI (Yin et al.
2023) scores each control sample on `M` plant draws and replaces the mean by a CVaR.
Section 4 says how this plan takes the simplest honest form of that branch and why one
plant draw per control sample is not it.

## 3. Input and output: the contract

```python
PathIntegralPlanner(
    problem,                 # PlanningProblem or StochasticPlanningProblem (finite tf, a cost)
    *,
    dt=0.05,                 # control period; N = round(tf / dt) held inputs
    n_samples=1024,          # K
    temperature=1.0,         # lambda
    noise=None,              # Sigma: a float, an (m,) diagonal or an (m, m) covariance;
                             # default: (u_half / 2)^2 from the input box
    control_cost=0.0,        # alpha: 0 charges the information term, 1 turns it off
    n_iterations=1,          # passes of sample-score-weigh-update per solve
    n_plant_samples=1,       # M plant realizations per control sample (stochastic problem)
    integrator="rk4",
    seed=0,
    verbose=False,
)
```

- **`solve(*, initial_guess=None, evaluate=False, n_trials=50)`** → `PlanningSolution`:
  `policy` the held schedule as a `TrajectorySource` (`interpolation="previous"`: the
  rollout held its inputs, so `policy >> plant` replays the plan — the RRT precedent),
  `solver` a `PathIntegralRecord`, `trajectory` the nominal rollout of `U` from
  `x_start` on the nominal plant (the planner's evidence, as every planner), and the
  evaluation when asked. `initial_guess` is a `Trajectory` or an `(N, m)` array;
  default the nominal input of the ports.
- **`solve_trajectory_from(x0, *, params=None, initial_guess=None)`** — the online tick:
  the same solve from the measured state, warm-started from `initial_guess`
  (the shifted previous `U`). The MPC block calls this.
- **`accepts_stochastic = True`**: a stochastic problem's draws are used (§4); a
  deterministic problem is the textbook algorithm. A deterministic planner's warning
  is not raised.
- **Returned `trajectory`** has `N + 1` samples, so the MPC block's `x_ff = plan.x[:, 1]`
  and `u_ff = plan.u[:, 0]` read as they do for trajopt.
- **`problem.tf` must be finite** (`require_finite_tf`, as trajopt). `problem.U` must
  bound a box (`U.bounding_box()`); `X` is any set, read through `margin`.
- **Backend.** The prototype requires JAX (`require_jax()` in the constructor, RULES
  5.12); `problem.sys`, the cost and `X` must trace. A NumPy path is step MP-6, not the
  prototype (constitution invariant 5 holds for the teaching surface; this lands on the
  research lane).

Why `dt` and not `n_steps`: the MPPI grid is a control grid, the same object
`ReinforcementLearningPlanner(dt=)`, `RolloutEnvironment(dt=)` and
`MonteCarloEvaluator(dt=)` name; trajopt's `n_steps` counts collocation knots on a
continuous plan, a different object. The receding-horizon shift is then one control
period, so `dt_mpc` should be a multiple of `dt` (checked at wrap time, §6).

## 4. The stochastic question

The maintainer's question: should MPPI take a stochastic problem and sample plant
noise as well as input noise, or only the deterministic problem with injected input
noise? Both, and the distinction is one branch in the sampling beat.

**Deterministic problem** (the textbook): `θ^k = θ` nominal, `w^k_t` nominal. The only
randomness is `ε^k`. This is what every MPPI paper runs on a simulator and what the
GMC714 lesson would show.

**Stochastic problem** (the robust branch): each sample `k` also draws a realization of
the plant, `θ^k ~ params_distribution` through `problem.sample_params(key)` and a
disturbance path `w^k_t ~ disturbances` through `problem.sample_disturbances(key)` per
step (after RN-4, a realized signal per sample). `x0_distribution` is *not* sampled:
the tick starts from the measured state, so the start law is the evaluator's business.

The honest form: `n_plant_samples = M` realizations per control sample, the control
sample's score being the problem's criterion over them —
`S^k = mean_m S^{k,m}` for `"expectation"`, the CVaR of the `M` for `"worst_case"`
(Risk-Aware MPPI). With `M = 1` the softmin rewards a lucky plant draw as much as a
good input sequence, which biases the update toward optimism; the paper that makes
this point is the reason `M` is a knob and its default on a stochastic problem should
be small but above one (`M = 4` proposed). On a deterministic problem `M` is ignored
(there is nothing to draw; the twin test pins byte-identity with `M = 1` and no draws).

Cost of the stochastic branch: `K × M` rollouts per tick instead of `K`. The JAX
batch handles it (one `vmap` over the flattened `K M` axis with a family `params`
pytree, exactly the `rollout_batch` contract); the body stays the six lines of §2
with one `mean` / CVaR line added.

What stays out: a learned residual model of the plant (the data-driven variants), the
ancillary tracking controller of Tube-MPPI (a `Controller` composed in the script, not
a planner feature), and a `RobustPlanningProblem` with set-bounded uncertainty (a
Later idea in TODO; the minimax consumer would be exactly this planner's
`"worst_case"` branch, so the two decisions go together when they come).

## 5. Where it lives, and the shape of the code

**Home:** `minilink/planning/trajectory_optimization/path_integral.py`. It is a
trajectory-family planner (its output is a schedule, its policy an open-loop source),
so it shelves with the transcription planner it is the gradient-free sibling of; the
package name says what the verb does, not how (RULES 2.1). It is not an NLP, so it
imports nothing from `optimization/`. Exports: `PathIntegralPlanner` and
`PathIntegralRecord` on the `minilink.planning` facade once provisional (an [ask]:
public names).

**Name:** `PathIntegralPlanner` (the textbook verb, Kappen / Theodorou: path integral
control), so that `ModelPredictiveController(PathIntegralPlanner(...))` reads as "model
predictive path integral" — the composition spells the acronym. The alternative,
`MPPIPlanner`, is the name students will search for; a docstring's first line carries
it either way. [ask]

**Dependencies:** `core` (backends, trajectory, sets, costs), `planning.problems`,
`planning.planner`, `planning.results`, and `planning.reinforcement_learning.environment`
for `RolloutEnvironment` — an intra-band import, legal under the dependency law; if the
maintainer prefers, `environment.py` moves up to `planning/environment.py` first (it is
already shared by the evaluator and the tabular learners, DESIGN §6 says so; the move is
one import rewrite with no behaviour change and a Later row of its own).

**The body, in the `dp.py` style** (three beats, no `self.` in the math, plumbing in
helpers under `# Internal machinery`):

```python
def solve_trajectory_from(self, x0, *, params=None, initial_guess=None):
    """One path-integral solve from ``x0``: sample, roll out, weigh, update."""
    # Unpack
    jax, jnp = require_jax(), require_jax_numpy()
    env, cost, dt, N = self.env, self.env.cost, self.env.dt, self.n_steps
    K, M, lam, alpha = self.n_samples, self.n_plant_samples, self.temperature, self.control_cost
    Sigma, Sigma_inv, u_lower, u_upper = self.noise, self.noise_inv, *self.input_box
    U = self.sequence_of(initial_guess)                     # (N, m), the nominal sequence
    self.key, k_eps, k_plant = jax.random.split(self.key, 3)

    # Sample: V^k = U + eps^k, eps^k_t ~ N(0, Sigma), clipped to the input box
    eps = jax.random.multivariate_normal(k_eps, jnp.zeros(U.shape[1]), Sigma, (K, N))
    V = jnp.clip(U + eps, u_lower, u_upper)

    # Roll out: x^k_{t+1} = Phi(x^k_t, v^k_t; theta^k, w^k_t) over dt, input held,
    # on M plant realizations per control sample (one, nominal, on a deterministic problem)
    theta, keys_w = self.plant_realizations(k_plant, K * M)
    x, g_path, left_X = rollout_sequences(env, x0, jnp.repeat(V, M, axis=0), theta, keys_w)

    # Score: S^k = sum_t g dt + h(x_N) + infeasible price, then the criterion over the M draws,
    # plus the information-theoretic control term lam (1 - alpha) sum_t u_t^T Sigma^-1 eps^k_t
    S_path = self.criterion(g_path.reshape(K, M))
    S = S_path + lam * (1.0 - alpha) * jnp.einsum("tm,mn,ktn->k", U, Sigma_inv, eps)

    # Weigh: w^k = exp(-(S^k - S_min) / lam), normalized
    S_min = jnp.min(S)
    w = jnp.exp(-(S - S_min) / lam)
    w = w / jnp.sum(w)

    # Update: U <- U + sum_k w^k eps^k
    U_new = U + jnp.einsum("k,ktm->tm", w, eps)

    # Output: the nominal rollout of U_new as the plan, the record of this tick
    trajectory = self.nominal_rollout(x0, U_new)
    record = PathIntegralRecord(
        S_min=float(S_min), S_weighted=float(w @ S),
        effective_samples=float(1.0 / jnp.sum(w**2)),
        exit_rate=float(jnp.mean(left_X)), z=np.asarray(U_new).reshape(-1),
        success=bool(self.stays_in_X(trajectory)),
    )
    return self.store_solution(self.trajectory_solution(trajectory, record))
```

`rollout_sequences` is the `jit(vmap(scan(env.step)))` helper — the scan over `t` and
the vmap over samples are plumbing, the step is `env.step` so the exit rule and the
disturbance draw have one owner; `plant_realizations` is the `sample_params` /
`sample_disturbances` branch (nominal on a deterministic problem). One compiled
function per planner, shapes fixed at construction, so a tick is one device call. The
`n_iterations` loop wraps the five beats and is written as a `for` in the body (the
textbook form), not hidden.

## 6. The receding-horizon block: the contract it should ask for

To wrap any trajectory-family planner, `ModelPredictiveController` should read three
verbs and nothing else:

| Today (trajopt internals) | Proposed (planner verbs) |
| --- | --- |
| `planner.transcription.decision_dimension(problem)`, `transcription.options.n_steps >= 2` | `planner.decision_dimension()` (trajopt: the packed `z`; MPPI: `N m`), and the plan's `n_samples >= 2` checked on the returned trajectory |
| `planner.options.compile_backend == "jax"` → `compile_parametric_program()` | `planner.prepare_online()`: trajopt compiles its parametric program, MPPI jits its rollout; the block calls it once at construction |
| `planner.last_optimization_result.z` after the solve | `solution.solver.z` on the tick's record (`TrajectoryOptimizationRecord` gains `z`; `PathIntegralRecord` has it) — the T6 row "one record per tick" |
| `mpc_warm_start_guess(z_prev, y, planner, dt_mpc=, k=)` (unpacks with the transcription) | `planner.warm_start_guess(z_prev, y, *, dt_mpc, k)`: trajopt shifts the knot schedule as today; MPPI shifts `U` by `round(dt_mpc / dt)` periods and repeats the last input |

No `hasattr` probing (RULES 4.3): the three verbs become methods of the `Planner` base
that raise `NotImplementedError` by name, as `solve_trajectory_from` does today, and
`validate_mpc_planner` calls them. `warm_start=True` keeps packing the planner's `z` on
`Computer.x`, which for MPPI is exactly the shifted nominal sequence — the state the
algorithm carries between ticks in every implementation. `Command`, the tick latch, the
dual-rate broadcast, `export_to_computer`, `mpc @ plant` and the overlays
(`mpc_plans_from_rollout`, `mpc_animation_overlays`) are unchanged: they read the
trajectory of the tick's solution. The check that `dt_mpc` is a multiple of the
planner's `dt` belongs to `warm_start_guess` (MPPI raises; trajopt interpolates as today).

This is the T6 `control/mpc/controller.py` conversation the workboard already owes (and
the D3 rows "MPC port computes drop `params`" and "dual online-params façades"). Doing
it with MPPI as the second consumer is what makes the contract honest instead of a
refactor in the abstract.

## 7. Options weighed

| Question | Options | Proposed | Why |
| --- | --- | --- | --- |
| Planner or controller | (a) a planner wrapped by `ModelPredictiveController`; (b) an `MPPIController` block of its own | (a) | One receding-horizon loop, one latch, one export, one animation path; `compare(MPC=, MPPI=)` on one problem is then one line; (b) duplicates 900 lines of `controller.py` |
| Problem class | (a) deterministic only; (b) stochastic only via `as_stochastic`; (c) both, one branch in the sampling beat | (c) | The base class already makes every planner accept both; `accepts_stochastic=True` is one attribute; the deterministic twin is the test |
| Where the step semantics live | (a) rewrite `x_{t+1}`, the exit rule and `h` inside the planner on `rk4_integrate_zoh_trace`; (b) `RolloutEnvironment.step` | (b) | One owner of what an episode costs (S65); the equations still appear as the named lines of §5; the env already carries the disturbance and parameter draws and will carry `realize(key)` after RN-4 |
| Plant draws per control sample | `M = 1` always; `M` a knob | `M` a knob, default 4 on a stochastic problem | §4: `M = 1` rewards lucky draws |
| Home | `planning/trajectory_optimization/`; a new `planning/sampling/`; `control/mpc/` | `trajectory_optimization/` | Output is a schedule; a package per algorithm family is the existing taxonomy; `control/mpc` is the wrapper, not the solver |
| Backend | JAX only; `xp` body from day one | JAX only for the prototype, NumPy `loop` backend with the teaching decision | The vectorized-over-`K` `xp` path needs the NumPy evaluator to batch, which it does not; the per-sample loop is the `loop` rung of the DP ladder and is fine for a 2-D teaching plant |
| Grid | `dt=`; `n_steps=` | `dt=` | §3 |
| Name | `PathIntegralPlanner`; `MPPIPlanner` | `PathIntegralPlanner` [ask] | §5 |
| Smoothing, adaptive `λ`, CVaR | in the prototype; later | later, each one option with a test | the prototype is the paper's Algorithm 2 |

## 8. Steps

Ids are local to this doc (`MP-1` …); the workboard row is S72. Each step: `ruff check
.`, `ruff format --check .`, the planning tests, the textbook check next to `dp.py`
stated in the report.

- [ ] **MP-1 The planner, deterministic** (JAX). `planning/trajectory_optimization/path_integral.py`:
  `PathIntegralPlanner` and `PathIntegralRecord`, `solve` / `solve_trajectory_from`,
  the body of §5 on a `PlanningProblem`, `rollout_sequences` on `RolloutEnvironment`,
  `decision_dimension`, `warm_start_guess`. Tests
  (`tests/unittest/test_planning_path_integral.py`): seeded determinism (two solves,
  equal arrays); the double integrator with the LQR cost — the MPPI plan's cost within
  a tolerance of the `LQRPlanner` nominal rollout's as `K` grows; the pendulum swing-up
  reaches `Xf` (the `test_planning` trajopt problem); `solution.policy >> plant` replays
  the plan; the record's `str`. Done when both tests pass and the body reads next to
  `dp.py`.
- [ ] **MP-2 The hand-loop demo** (research lane).
  `examples/experimental/mppi/pendulum_mppi.py`: the planner, `compute_command`-style
  hand loop (`solve_trajectory_from`, apply `u_0`, step the plant with `discretize`,
  shift) until MP-4; `print(solution)`; the plan over the closed-loop trajectory.
  Done when it runs in the nightly sweep's wall-clock budget.
- [ ] **MP-3 The stochastic branch.** `n_plant_samples`, `plant_realizations`,
  the criterion over `M` (mean; CVaR for `"worst_case"`), on `sample_params` and
  `sample_disturbances` as they stand (one held draw per step), rewritten to the
  realized signals when RN-4 lands. Tests: the deterministic twin (stochastic problem
  with a `Particles` start and no draws, `M = 1`: byte-identical to MP-1); a pendulum
  with a mass distribution plans a more conservative swing than the nominal one (the
  weighted cost rises with the spread). Done when both pass.
- [ ] **MP-4 The MPC block takes the planner** **[ask — main-tool API, T6]**. The §6
  contract: the three `Planner` verbs, `z` on `TrajectoryOptimizationRecord`,
  `validate_mpc_planner` reads the verbs, `mpc_warm_start_guess` becomes trajopt's
  `warm_start_guess`. Behaviour-preserving for trajopt: the MPC baselines
  (`test_mpc.py`, `mpc_car_minimal`, `mpc_integrator_numpy`) byte-identical before and
  after (AGENTS recipe). Then `ModelPredictiveController(PathIntegralPlanner(...),
  dt_mpc=)` runs the pendulum and `mpc_car_minimal` with one line changed. Done when
  `test_mpc.py` gains the MPPI twin of its warm-start and stateless tests and the
  baselines `cmp`.
- [ ] **MP-5 Demos to the rule and the comparison.** `examples/demos/mpc/mppi_car_minimal.py`
  (the `mpc_car_minimal` story with the planner swapped) and the lesson
  `examples/demos/mpc/mpc_vs_mppi_pendulum.py` (`compare(MPC=, MPPI=)` on one
  problem: the solve times, the costs, the plans on one axis — the compare rule of
  examples/README). `mpc_animation_overlays` draws the sampled rollouts' spread as an
  option (`samples=`) only if the picture earns it. The ROADMAP §3 row moves to TRL 5.
  Done when both demos run in the nightly sweep and `examples/README.md` lists them.
- [ ] **MP-6 Teaching form and the freeze decision** (v0.9, with V3). The `loop` backend
  (per-sample NumPy, the textbook double loop; `backend=` as `DynamicProgrammingPlanner`),
  so the Basic tier runs a small MPPI; the `minilink.planning` facade names; the GMC714
  notebook cell if G1 put MPC on the teaching surface; DESIGN §6 paragraph; the
  smoothing / adaptive-temperature options if a course asks. Done when the teaching
  surface test lists the names, or V3 rules them provisional.

Dependencies: MP-1–MP-3 need nothing that is not on `dev` today. MP-3's disturbance
path is rewritten once by RN-4 (held draws → realized signals); landing it before RN-4
means one small rewrite, landing it after means waiting for the v0.3 late-term rung —
the proposal is to land MP-3 on the current draws so the planner is demonstrable, and
fold the RN-4 rewrite into RN-4's blast radius (one more consumer of `realize`). MP-4
is the T6 conversation and needs the maintainer in the room.

## 9. Timing

The question asked: when. The proposal, against the rungs of ROADMAP §5:

- **v0.3 wave C (October–December 2026): MP-1, MP-2, MP-3** on the research lane,
  beside G1. GMC714 is the robotics course and §4.3 names MPC as a topic row whose
  surface G1 decides; an MPPI prototype in hand when G1 runs lets the audit decide "MPC
  joins the teaching surface" with both flavours on the table instead of one. The three
  steps touch no public name and no teaching file, so they do not compete with wave B
  (GRO501, October) for maintainer time beyond the two asks of §10.
- **Before the v0.9 freeze (early 2027): MP-4 and MP-5.** MP-4 is T6's
  `control/mpc/controller.py` conversation, already queued before v0.9 because V3 must
  decide whether `control.mpc` joins the frozen surface; narrowing the planner contract
  is a prerequisite of freezing that block's name, so MPPI costs the freeze nothing it
  did not already owe. MP-5's demos land as soon as MP-4 does.
- **v0.9 / v1.0: MP-6** with V3 — the teaching form (`loop` backend, Basic tier), the
  facade names, the DESIGN paragraph. Not before: a planner reaches the teaching
  surface through a demo, a both-backends test and a cohort (ROADMAP §2).

So: prototype in v0.3 (the maintainer's "before v1" is met with a term to spare),
provisional in v0.9, frozen or visibly provisional at v1.0 with the rest of the MPC
band. The one thing that moves earlier if the maintainer wants the MPPI car demo in
the GMC714 notebook for December: MP-4, which then runs in November on `dev` as the
T6 step, with the trajopt baselines as its safety net.

## 10. Asks for the maintainer

1. **The name** (§5): `PathIntegralPlanner` or `MPPIPlanner`. One decides the record's
   name and the facade entry.
2. **The MPC block contract** (§6): the three `Planner` verbs (`decision_dimension`,
   `prepare_online`, `warm_start_guess`) and `z` on the tick's record, as the shape of
   T6 for `controller.py`. A yes schedules MP-4; a different shape is written here
   before any code.
3. **The stochastic default** (§4): `n_plant_samples = 4` on a stochastic problem, or
   `1` with the bias stated in the docstring.
4. **Timing** (§9): MP-1–MP-3 in v0.3 wave C as S72, or held until G1 reports.
5. **`RolloutEnvironment`'s home**: stays in `reinforcement_learning/` (an intra-band
   import from `trajectory_optimization/`), or moves to `planning/environment.py` as a
   housekeeping step first.

Sources (the method): Williams, Aldrich, Theodorou, "Model predictive path integral
control: from theory to parallel computation" (JGCD 2017); Williams et al.,
"Information-theoretic MPC for model-based reinforcement learning" (ICRA 2017);
Williams et al., "Robust sampling based model predictive control with sparse objective
information" (RSS 2018, Tube-MPPI); Gandhi et al., "Robust model predictive path
integral control" (RA-L 2021); Yin, Zhang, Theodorou, Tsiotras, "Risk-aware model
predictive path integral control using conditional value-at-risk" (ICRA 2023).
