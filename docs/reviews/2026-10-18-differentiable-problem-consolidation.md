# The differentiable problem: three analyses consolidated into one plan (2026-10-18)

**Date:** 2026-10-18
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Three analyses of the same design existed on `dev` at f0d0a86: (A) the plan of
2026-10-17 and its morning-after companions, `differentiable-cost.md`,
`differentiable-cost-options.md`, `target-architecture.md` §1–4 and §6–7, with
[the core-objects review](2026-10-18-core-objects-review.md), written with the maintainer
(Claude); (B) a second opinion appended the same afternoon through Cursor (Gemini): the
guardrails of `target-architecture.md` §5, the four constraint modes and the `discount=`
option on `score_trajectory`, the `Set.as_cost` row, the bottom-up order of §7.1; (C) this
review, independent of both, which probed each claim against the code with four prototypes
(NumPy 2.5.3, JAX 0.11.2, SciPy 1.18.1, in the scratchpad, nothing landed). The maintainer
asked for one plan consolidating the three, written to the constitution. That plan is
[differentiable-problem.md](../plans/differentiable-problem.md); this file records what each
analysis said, what the probes found, and why the consolidation went the way it did. The three
source documents are deleted; their text is in git history before this commit.

---

## Verdict

A and B agree on the architecture and disagree on one thing: how the problem's hard constraints
enter the scoring kernel. A kept the kernel to the one scoring contract DESIGN §6 states and
left the constraint question as an ask; B put four modes on the kernel (`"cut"`, `"ignore"`,
`"soft"`, `"hard"`) and a `discount=` switch so the transcriptions could keep ignoring the
cost's rate. C sides with A on the kernel and with B on the sharp edges it names: the
constraints are real, their home is the program bridge, where sets already lower at the solver
boundary (RULES 4.3), not the number every tool reports. B's bottom-up order is adopted with
one change (the cost parameters land after the deterministic evaluator, not before the
kernel). Two of B's factual claims did not survive the probes (the masked `sum` is not
bit-identical; the discount switch protects no baseline), and two things neither A nor B had
seen were found (a pytree `Trajectory` needs a validation-free unflatten; the shooting
transcription and the compiled rollout differ by input model). The prototype confirmed A's
central claim end to end on the real pendulum loop.

## Checked against the code

Facts the plan rests on, confirmed by probes on `dev` at f0d0a86:

- **The loop traces in its nested params and holds the controller's state.**
  `ImpedanceIntegralController() @ Pendulum()` compiles on JAX with blocks `controller` and
  `plant`, `n = 3` (the integrator inside `x`), and `rk4_integrate_zoh_trace_p` scans with
  `{"controller": …, "plant": …}` threaded. The plant's command along the run is the
  `("controller", "u")` slice of the port-signal buffer `_compute_port_signals_p` fills at
  every step.
- **The gradient is right.** With a masked trapezoid kernel on that run (`QuadraticCost`, a
  `BoxSet` for `X`, `tf = 10 s`, `dt = 0.05 s`), `jax.grad` with respect to `Kp`, `Kd` and the
  plant's mass matched central differences to `1.6·10⁻¹¹`, `1.3·10⁻⁹` and `4.9·10⁻¹¹` relative.
- **The bridge works with the optimizer that exists.** The three gains ravelled to `z`
  (`jax.flatten_util.ravel_pytree`), a `MathematicalProgram(backend="jax", lower=0)` and
  `Optimizer(method="scipy_slsqp")` reached `‖∇J‖ = 2·10⁻⁸` in 49 iterations and 2.2 s, at
  `J* = 0.261`; 400 steps of the demo's projected gradient loop at `lr = 0.5` stopped at
  `J = 0.536`. The whole path was 60 lines of prototype; the plan's verbs make it six.
- **The masked kernel is bit-identical only as a running sum.** On 300 random trajectories
  (2 to 400 samples, 60 % with an exit), `alive = cumprod(inside)` with `np.cumsum(alive ·
  increments)[-1]` reproduced today's sliced `score_trajectory` in 300 cases; `np.sum(alive ·
  increments)` in 70, because NumPy's `sum` is pairwise and `cumsum` sequential. `jnp.cumsum`
  matched in 73 with a worst relative difference of `2.4·10⁻¹⁵`.
- **A pytree `Trajectory` cannot validate in unflatten.** A frozen dataclass registered with
  an unflatten that goes through `__post_init__` passes `jit` and `grad` but fails `vmap`
  (`x` arrives as `(B, n, N)`: "t must be 1-D") and `tree_map` with `None` leaves; an
  unflatten that sets the fields without checks passes all four, the public constructor keeping
  every check.
- **The discount switch protects nothing.** No trajectory-optimization test, demo or benchmark
  baseline declares a `discount_rate` (the rates in the tests are in the RL, stochastic and
  cost-algebra tests only), so the transcriptions taking the cost's rate leaves every baseline
  bit-identical (`e⁰ = 1`).
- **Single shooting integrates a linear input between knots** (`rk4_step_between_knots`,
  `umid = ½ (u0 + u1)`), while the compiled rollout holds it; the two are different
  discretizations of the same problem, not one path.
- **The duplication counts hold**: five scoring rules with four quadratures (the NumPy loop in
  `evaluate_trajectory`, `score_trajectory`, `trajectory_cost` without discount,
  `evaluate_jax`'s inline scan, the RL reward), two signal reconstructions, the 36-method
  integrator grid in 1763 lines, four demos and three notebook cells with a hand-written scan
  or descent loop (`pid_autotuning_jax`, `params_gradient`, `neural_controller_jax`,
  `cartpole_rollout_gradients`; `07_compile`, `11_reinforcement_learning`, `showcase_jax`).
- **Open-loop plans already go through the evaluator**: a `TrajectorySource` policy is wired
  `source >> plant`, so the open-loop case of the compiled problem costs no special case.
- **The NumPy program evaluator supplies no derivatives**; SciPy's `minimize` takes `jac=None`
  and differentiates by finite differences itself, which is the path a NumPy-only law keeps.

## The three analyses, topic by topic

| Topic | A (the plan and the review) | B (the second opinion) | C (this review) → the consolidated plan |
| --- | --- | --- | --- |
| The object | `problem.compile(controller)` → an evaluator with verbs; the program as one verb; absorbs `MonteCarloEvaluator` | the same, `controller=None` for open loop | kept; the prototype confirms it. Open loop is the plant alone with `u=`; no merge of the RL environment (its step keeps the policy outside the plant, the learner's contract) |
| The verbs | `trajectory`, `final_state`, `cost`, `mean_cost`, `success_rate`, `feasibility`, `program`; the options doc adds `evaluate` → `Evaluation` | `cost(..., constraints=)`, `feasibility`, `update_params` | `trajectory(of=)`, `cost`, `margin`, `evaluate`, `mean_cost`, `program`, and a `tune` facade (an ask); `final_state` is one slice, `success_rate` is `1 − failure_rate`, `update_params` is `program.metadata["unravel"]` |
| `Trajectory` native | decided; "no float cast, no copy, shape checks unchanged" | the same, "step 2b" first | kept, plus the two rules the probe found: unflatten without validation, signals in a fixed key order; the monotonic check NumPy-only |
| The scoring kernel | one kernel, `score_rollout` beside the NumPy `score_trajectory`; the contract's cut and price | one dual-backend `score_trajectory` with `constraints="cut" \| "ignore" \| "soft"` and `discount=True \| False \| float`; "bit-identical" through a static `alive` mask | one `score_trajectory`, no modes, no discount switch; the running-sum form (the only one that keeps the bits); the uncut integral is the problem without `X`; the transcriptions take the cost's rate |
| Hard constraints | ask 5 (hard rate plus a soft stand-in) and ask 9 (metrics as constraints) left open | four modes across every pipeline, a table per tool; `Set.as_cost`; `softmin` margins in the bridge | the modes become the bridge's lowering (`sets="hard" \| "soft" \| "ignore"`, `"hard"` default), the kernel stays the contract; `Set.as_cost` kept with the bridge as its first consumer; the margin is the exact minimum, the smooth form an option |
| The whole run | `rollout_p` returning `(t, xs, signals)` | the same | kept; the signal of the step's first stage, as the NumPy reconstruction reads it; the simulator-agreement test |
| Draws | `mean_cost(params, keys)` over `vmap`; held disturbance draws; RN-4 later | the same, RN-4 and RN-5 merged into one step | kept as RN-5's one path; one key per experiment, split inside; `Evaluation` as a pytree with `xp` statistics; the recorded trials not stored (replayed on demand) |
| The bridge | `program(decision, lower, upper, constraints, keys)`; keys drawn once | `program(wrt=, constraints="hard")`, flat `z`, `program.unravel` | `program(wrt, objective=, constraints=, sets=, key=, n_trials=, lower=, upper=)`; sample-average keys; identification as the same bridge |
| The optimizer (S74) | the maintainer's three parts | keep `MathematicalProgram` flat; one hand loop in a tutorial | the class table with the admitted methods and the `"auto"` pick, the default kept at `scipy_slsqp`; the first-order backend with the RL `Adam` moved; one global derivative-free preset (`scipy_differential_evolution`), never auto |
| Traceable analysis (S73) | poles, Bode, margins through `xp`; Newton steps on the crossovers | the sensitivity peaks `M_s`, `M_t` as smooth surrogates | both kept, as one row beside the plan |
| Shooting vs collocation | the open-loop `compile()` makes single shooting the bridge with `wrt="u"` | do not position it as a replacement for the transcriptions | B's guardrail kept and sharpened: the two differ by input model as well |
| The order | steps 0–3 now, A2, then 4–6 | bottom-up: nouns, then kernels, then the object, A2 before the kernel | B's order with A2 after the deterministic evaluator (its `xp` part first, its `params` part sixth), so the first demo lands before the cost dictionary exists |
| The `CostIntegrator` block | an ask (the trapezoid kernel proposed) | silent | closed: the kernel on samples is the contract; the block stays a demo idea |

## What was dropped, and why

- B's `constraints=` and `discount=` keywords on the kernel: a second owner of the discount
  (RULES 5.6) and three numbers for one trajectory where DESIGN §6 promises one; the uses they
  served are the bridge's lowering and the problem without `X`.
- B's "bit-identical" claim for the masked sum: false as written; true for the running sum,
  which is what the plan specifies.
- A's `final_state` and `success_rate` verbs and B's `update_params`: each one line on what
  the caller already holds (RULES 2.5, 4.6).
- A's `score_rollout` as a JAX twin beside the NumPy `score_trajectory`: one body on `xp`
  replaces both (the figures' label changed accordingly).
- A's §4.6 merge of the RL environment into the evaluator: the environment steps the plant
  with the policy outside, the shape the collectors and the sampling planner need; what is
  shared is the exit test (`Set.contains` on `xp`) and `g`, not the step.
- B's `stochastic` program class in the first cut: the bridge draws once, so its programs are
  deterministic and every preset applies; the flag waits for a resampling consumer.

## What the maintainer is asked

The twelve asks are §9 of the plan, each with its proposed answer. The ones that change a
ruling or a number: the evaluator's numbers changing once (already accepted with RN-5's
RD-D24), the transcriptions taking the cost's discount (no baseline moves), and the default
optimizer method staying `scipy_slsqp` until the V3 freeze decides on `"auto"`.
