# Standard planning problems (taxonomy plan)

Status: draft plan (July 2026). Design-only — no implementation in this
phase.

Related: [planning-pipeline-architecture.md](planning-pipeline-architecture.md)
(algorithm / I/O stack), [DESIGN.md](../../DESIGN.md) §6,
[ROADMAP.md](../../ROADMAP.md) §5.5.

---

## Intent

Define a small set of **problem description** classes for planning and control
tasks. A problem states **what world** and **what we want**. It must **not**
fork by whether we later *synthesize* a solution or *evaluate* a fixed
controller — those are verbs on the same object:

```text
Problem  =  dynamics + uncertainty + constraints + criterion
Solve    =  find κ or u(·)     (trajopt, LQR, gain search, RL, …)
Evaluate =  fix κ, score it    (one-shot, Monte Carlo, policy eval, …)
```

---

## Everyday tasks → math, not algorithms

| Everyday activity | Mathematical task | Uncertainty |
| --- | --- | --- |
| LQR / infinite-horizon LQ | Find linear κ minimizing quadratic cost | None (nominal model) |
| Trajectory optimization | Find \(u(\cdot)\) / knots, Bolza + path/boundary sets | Usually none; \(x_0\) fixed or set |
| Tune PID gains over many ICs | Find policy params \(\phi\) minimizing **expected** cost | \(x_0 \sim p\) (maybe \(\theta\) too) |
| Robustness check of a controller | Score criterion under \(\theta,w,x_0\) uncertainty | Distributions or adversary set |
| RL on “the full problem” | Find κ for expected return / \(E[J]\) | \(p(x_0)\), process noise, often random params |

All share: **plant + constraints + criterion**.  
They differ by **uncertainty model** and **what “optimal” means** — not by
solver name.

---

## Design laws

1. **One description, two verbs** — `solve(problem)` and `evaluate(problem, κ)`
   never force different types.
2. **Split on uncertainty + criterion**, not on algorithm, and not on
   “are we checking or finding.”
3. **Two classes now** (deterministic + stochastic); **optional third**
   (robust / minimax) when worst-case is first-class.
4. **Sets = support; distributions = probability** — do not overload `X0` as
   both a Set and a measure.
5. **Policy family is not a problem class** — open-loop \(u(t)\), LQR gains,
   PID \(\phi\), NN weights attach at solve time.
6. **RL does not get a special problem** — it consumes the stochastic class;
   Gym stays an `interfaces/` adapter.
7. **Horizon / grids stay method-side** for deterministic NLP (see
   transcription options). Stochastic / RL tasks may carry an evaluation
   horizon as part of the stochastic task design (or on solve/eval options).

---

## Three problem classes

### 1. `PlanningProblem` — deterministic (mainline, exists)

```text
ẋ = f(x, u, t; θ)
x ∈ X,  u ∈ U
x(0) ∈ X0,   x(T) ∈ Xf?      # X0 often singleton; may be a fat set (support)
θ fixed (point params)
min J[x,u]   (optional — feasibility-only allowed)
```

**Covers:** LQR (as LQ method on this task), trajopt, RRT, classical DP,
single-IC checks.

**Solve:** find \(u(\cdot)\) or κ.  
**Evaluate:** representative IC(s) from `X0`, report \(J\).

Keep current [`PlanningProblem`](../../minilink/planning/problems.py) as this
class. Clarify in docs that fat `X0` is set-valued **support**, not a
probability law. Horizon/`n_steps` remain on method options
(transcription / MPC), not required problem fields.

---

### 2. `StochasticPlanningProblem` — probabilistic uncertainty

Engineering nickname: **motion problem** (uncertain motion task). Prefer the
precise type name in code; “motion problem” is fine in prose.

```text
same plant / X / U / cost scaffold as PlanningProblem
x0 ~ p_x0
θ  ~ p_θ          # mass, tire, scene id, …
w  ~ p_w          # optional process / disturbance model
criterion: typically E[J]  (or a declared risk: CVaR, …)
```

**Covers:** PID/gain tuning over IC ensembles, Monte Carlo robustness, domain
randomization, **RL**, future stochastic MPC.

**Solve:** find κ (or κ_φ) minimizing the stochastic criterion.  
**Evaluate:** same criterion with κ fixed (Monte Carlo).

This is **not** evaluation-only. RL and gain search are synthesis on this
description.

**Perturbation channels (keep crisp):**

1. Additive forcing on known ports (start here)
2. Parametric via `p_θ` (start here)
3. Process noise inside `f` (defer until plants advertise a standard `w`
   channel or wrapper)

**Distributions:** tiny duck-typed surface (`sample(rng)`, optional support
Set) — e.g. Gaussian, Uniform, ParticleSet — not a full probability library.

---

### 3. `RobustPlanningProblem` — set-bounded uncertainty (optional third)

```text
x0 ∈ X0,  θ ∈ Θ,  w ∈ W     # only supports / boxes / unions
criterion: min_κ max_{x0,θ,w} J   (or soft robust variants)
```

**Covers:** classical robust checks (“for all masses in \([m_1,m_2]\)”),
worst-case tubes — when densities are intentionally refused.

\(E[J]\) under Uniform is **not** the same as \(\max J\). Keep this class only
when minimax is first-class; until then, Uniform MC + reporting
`mean` / `worst_sample` as metrics is enough.

---

## Unified picture

```text
                    uncertainty model
              none          probabilistic       set-adversarial
           ┌────────────┬──────────────────┬─────────────────┐
criterion  │ Planning   │ Stochastic       │ Robust          │
J or E/max │ Problem    │ PlanningProblem  │ PlanningProblem │
           └────────────┴──────────────────┴─────────────────┘
                         ▲
                         │  shared spine: sys, X, U, cost?, Xf?, params, metadata
                         │  differ by: X0 vs p_x0, θ vs p_θ, w vs W / p_w
                         │
              solve(problem) → κ
              evaluate(problem, κ) → score
```

**Shared spine**

```text
sys, X, U, cost?, Xf?, params-scaffold, metadata
+ uncertainty { none | measures | adversary sets }
+ criterion   { J | E[J] | max J | … }
```

---

## Everyday workflows on this taxonomy

| Workflow | Problem class | Solve | Evaluate |
| --- | --- | --- | --- |
| LQR | Deterministic | Riccati / `lqr(A,B,Q,R)` | simulate from `x_start` |
| Trajopt to a goal | Deterministic | collocation / shooting | replay trajectory |
| PID gains over ICs | **Stochastic** \(x_0\sim p\) | CMA-ES / grid / outer loop | MC estimate of \(E[J]\) |
| Robustness report | Stochastic *or* Robust | — | MC or worst-case sweep |
| Full RL | **Stochastic** | PPO/SAC/… | same env = same problem |

Same stochastic description for “tune PID” and “train RL”; only the **solver**
and **policy parameterization** change.

---

## What not to add as problem classes

| Tempting split | Better as |
| --- | --- |
| SynthProblem vs EvalProblem | Same problem; different verbs |
| LQRProblem / TrajoptProblem / RLProblem | Methods / policy structure |
| PID tuning problem | Stochastic problem + policy family (κ_φ = PID) |
| Finite vs infinite horizon | Method options + whether `Xf` is set |
| MotionProblem as “eval only” | Rejected — motion/stochastic is a full task description |

---

## Bridges

- `StochasticPlanningProblem.nominal()` → `PlanningProblem`  
  (mean/mode θ, representative `x_start`, supports → sets). Enables
  certainty-equivalent trajopt/LQR inside a richer task.
- Optional `MotionProblem` prose alias for the stochastic class only — do not
  use it to mean “evaluation problem.”
- Gymnasium / RL: `interfaces/` adapter that `reset ~ sample()`, `step`
  simulates, reward from criterion — math stays in planning.

[`PolicyEvaluator`](../../minilink/planning/policy_synthesis/policy_eval.py)
remains the **grid DP** expected cost of a fixed policy on a
`PlanningProblem`. Continuous Monte Carlo scoring of a stochastic problem is a
separate evaluator tool later — not a second problem taxonomy.

---

## Placement (when implemented)

| Piece | Home |
| --- | --- |
| `PlanningProblem` | `minilink/planning/problems.py` (exists) |
| `StochasticPlanningProblem` | same module (or `planning/standard_problems.py`) |
| `RobustPlanningProblem` | same, when needed |
| Distributions duck types | `planning/distributions.py` or thin `core/` helpers |
| Monte Carlo / risk evaluators | planning tools (not interfaces) |
| Gym adapter | `interfaces/gymnasium.py` |

Dependency law unchanged: problem types do not import catalog plants;
engines remain optional leaves.

---

## Phased delivery

| Phase | Deliverable | Exit |
| --- | --- | --- |
| **0** | This plan + maintainer sign-off; ROADMAP/DESIGN pointer when approved | Review queue |
| **1** | Doc-only: clarify `PlanningProblem` / fat `X0` vs distributions in DESIGN §6 | Docs aligned |
| **2** | `StochasticPlanningProblem` + minimal distributions + `sample()` | TRL 2–3 |
| **3** | `evaluate(problem, κ)` Monte Carlo reporter (mean/std, failure rate) | Demo: PID-over-ICs or robustness |
| **4** | `nominal()` bridge; optional Gym adapter over stochastic problem | RL path unblocked without new problem class |
| **5** | `RobustPlanningProblem` only if minimax consumers exist | Optional |

---

## Open decisions (review)

1. Code name: `StochasticPlanningProblem` vs engineering alias `MotionProblem`
   as the class name.
2. Criterion field: hardwire default `E[J]` vs small `Criterion` object
   (`Expectation`, `CVaR`, …) from day one.
3. Where evaluation horizon / `n_trials` live: on the stochastic problem vs
   `EvaluateOptions`.
4. Whether robust (class 3) is scheduled or deferred indefinitely.
5. How `ProblemParameters` tiers grow into parameter **distributions** without
   breaking deterministic callers.

---

## Bottom line

- Keep **`PlanningProblem`** as the deterministic standard task.  
- Add **one stochastic class** for \(p(x_0)\), \(p(\theta)\), \(p(w)\) and
  \(E[J]\) (or declared risk): covers PID-over-ICs, robustness MC, and RL.  
- Optionally later **`RobustPlanningProblem`** for true minimax over sets.  
- Categorize by **uncertainty + criterion**, never by synthesizer vs evaluator
  or by algorithm name.
