# Native optimizers: the solve as a rollout

Status: written 2026-10-25 from the maintainer's vision (§1), with the agent's read (§3–§4);
its asks answered (§6). Lane: research. The proof of concept landed in v0.3 (§2); the rest is a
v0.5 row, [TODO.md](TODO.md) §4 ("Native solvers beyond the proof of concept"). Contracts in
code today: [DESIGN.md](../dev/DESIGN.md) §6 (Planning / NLP, the planner's compiled face, the
MPC).

**Choices of 2026-10-27** (the architecture of record since 2026-10-28):
- the two-layer split of §3: a pure kernel run by `Optimizer`, a step-system view for the
  lesson; §4.1–§4.2's categories and algorithms added only with the first solver that needs them;
- the fixed-count solve: `Optimizer(program, method=, n_iterations=K)`, exactly `K` steps in one
  `lax.scan`; `n_iterations=None` runs to a tolerance, `maxiter` its cap; a cap and a fixed count
  stay two things;
- programs, parameters and randomness: §7's convention, on one program noun,
  `MathematicalProgram(..., params=None)`.

**Where each idea lives** (one fact, one home):

| Idea | Home |
| --- | --- |
| Native solvers as pure laws, program categories, the solve as a rollout, the three ways to differentiate a solve | this doc, §3–§4 |
| The fixed-iteration MPC, MPPI on the face, the planner's compiled face, the program's parameters | DESIGN §6 (landed) |
| Convergence analysed as stability (goal 2) | this doc, §4.7, NS-5 |
| The sampled loop as one scan; its differentiable evaluation | DESIGN §5 and §6 |
| A gradient-descent tutorial on the extended `Optimizer` | [TODO.md](TODO.md) §6 |
| Stochastic programs in the solvers: seeds, the convention | this doc, §7; randomness in simulation DESIGN §4 |

---

## 1. The vision (the maintainer, 2026-10-25)

- **An upgraded `MathematicalProgram` with categories**, and an upgraded `Optimizer` whose
  solvers fall in two families: **wrapped backends** (Ipopt, `scipy.optimize.minimize`, other
  libraries) and **native algorithms** implemented in minilink.
- **Native algorithms are pure array code, like a `System`**: the equations are stateless
  functions of arrays, with optional `params` the way `f(x, u, t; p)` has them.
- **The solve is a rollout.** The sequence of iterates of gradient descent is the analog of
  the rollout of a step system, `z_{k+1} = step(z_k, k; p)`.
- **The ultimate goal: some solvers fully JAX-compatible**, so that the whole solve can be
  jitted, differentiated and vmapped: a solve inside a loss, a batch of starts or of problem
  parameters in one call.

### 1.1 The two goals (the maintainer, 2026-10-25)

1. **First: an MPC whose solver runs a fixed number of iterations**, so the controller's tick
   is fully compilable and even differentiable; the closed loop with the MPC then simulates
   end to end in JAX, very fast.
2. **Long term: analyse the convergence of an optimization process with the control tools**:
   the algorithm as a feedback loop, its convergence as stability.

## 2. What landed (the proof of concept, v0.3)

NS-1, NS-2's fixed-`K` form, NS-8 and NS-9: the laws `GradientDescentLaw` and `AdamLaw`
(`optimizers/first_order.py`), `Optimizer(..., n_iterations=K)` (a native method on a
box-bounded JAX program is a pure loop: `jit`, `vmap` over starts, `grad` through the solve; a
traced solve returns its `OptimizationResult` with JAX arrays), `optimizer.as_step_system()`
(the solve as a step system, the iterates its rollout), the fixed-iteration MPC on the
planner's compiled face and its closed loop as one scan tuned by gradient
(`problem.compile(mpc % Δ)`), and MPPI on the same face with `n_iterations` per tick. The
parametric program folded into `MathematicalProgram(..., params=)`. Still open: the
`lax.while_loop` form of `n_iterations=None` (NS-2's rest), and NS-3 to NS-7 (§5).

## 3. The agent's read: the optimizer as a step system

The idea is the control-theory view of optimization algorithms (Lessard, Recht and Packard,
2016). It fits the CONSTITUTION's first article better than any other option.

### 3.1 The textbook picture: an update law in feedback with a gradient oracle

Gradient descent is a discrete integrator with gain `−η`, in feedback with a static
nonlinearity, the gradient `∇J`:

```text
        ┌──────────────┐  z_k   ┌───────────────┐
  ┌────▶│ update law   │───────▶│ oracle  ∇J(z) │──┐
  │     │ z⁺ = z − η u │        │ (static)      │  │
  │     └──────────────┘        └───────────────┘  │
  └──────────────────── u_k = ∇J(z_k) ◀────────────┘
```

- The **update law** is a `StepSystem` with input `u` (a gradient) and state `z`, plus the
  method's memory: the momentum of heavy ball, the moments `m, v` of Adam. Gradient descent and
  heavy ball are *linear* step systems; Adam is a nonlinear one.
- The **oracle** is a static system `y = ∇J(u; p)` built from the program, with the problem's
  parameters `p` as its `params`.
- **The solve is the closed loop's rollout**: a `StepDiagramSystem` of the two, compiled and
  rolled out like any other. Its parameter tree follows the diagram contract, every block
  named: `{"law": {"eta": …}, "oracle": {…problem parameters…}}`.

This ran wired by hand (checked 2026-10-25): a one-state `StepSystem` law `z⁺ = z − η u`
with `η = 0.1`, a `Gain(2)` as the oracle of `J = z²`, connected in a `StepDiagramSystem` and
rolled out, gives `z = 1, 0.8, 0.64, 0.512`: the contraction `1 − 2η`. The step algebra's
`>>`, `+` and `@` wire the two, and `linearize` reads a step system in `z` (its poles against
the unit circle).

What this buys, beyond reuse:

- **Convergence is stability.** Linearize the loop at the optimum `z*`:
  `z_{k+1} − z* = (I − ηH)(z_k − z*)`. The step size is stable when every eigenvalue of
  `I − ηH` is inside the unit circle, and the rate is the spectral radius: a lesson GRO860 and
  the control courses get almost for free.
- **The iterates are a `Trajectory`** (`compute_rollout` on the view): the plot of `J` against
  `k`, the path of `z` on the contour.
- **`vmap` over the start** is a multi-start; over `η`, a step-size sweep; over `p`, a family
  of problems.

### 3.2 Where the analogy stops

- **Stopping.** A rollout runs a fixed count; a solver stops on a tolerance. A fixed count is a
  `lax.scan`, traceable in every mode; a tolerance is a `lax.while_loop`, which `jit` and
  `vmap` take but reverse-mode `grad` does not. Both stay, as one structural keyword:
  `n_iterations=K` the process, `n_iterations=None` the answer.
- **Differentiating the answer is not differentiating the process.** Two different
  gradients, both useful (§4.3).
- **Constraints.** A projection on the box fits the step model as is. Equality and inequality
  rows need an outer loop (augmented Lagrangian: a rollout of unconstrained rollouts) or
  linear algebra inside the step (Newton on the KKT system, interior point). The production
  constrained solver stays Ipopt.
- **The `System` contract costs something.** `x` is a flat vector; Adam's state is a pytree
  `(z, m, v, k)`. It flattens to `x = [z; m; v]` with `k` the tick, fine for a vector `z` and
  awkward for a neural network's weight tree. The RL learners keep their pytree path.

### 3.3 The two layers

1. **The kernel**: each native method is a pure law, `init(z0, params) -> state` and
   `step(state, params) -> state`, Optax-shaped (as `Adam` already is), on the array module of
   its inputs (one equation path, CONSTITUTION §2 invariant 5). `Optimizer` runs it: a Python
   loop on NumPy, `lax.scan` (`n_iterations=K`) or `lax.while_loop` (`n_iterations=None`) on JAX.
2. **The step-system view**: `as_step_system(solver, program)`, reached as
   `optimizer.as_step_system()`, the law as a `StepSystem` update block and the oracle as
   a static block, so a lesson can wire the loop, roll it out and linearize it. A *view* of the
   kernel, not a second implementation.

Every solver a `StepSystem` by construction would force the pytree state into a flat `x`; none
of them one would lose the best lesson in the plan.

## 4. The design, in outline

### 4.1 Program categories: few, textbook, declared by the program's fields

Today's three classes stay. The additions, each with its consumer (a category lands with
the first solver that needs it):

| Category | The form | What it unlocks | First consumer |
| --- | --- | --- | --- |
| unconstrained / bounded / constrained | as today | the method table of S74 | every program |
| **least squares** | `J(z) = ½‖r(z)‖²`, the residual `r` given | Gauss–Newton, Levenberg–Marquardt (native); `scipy.optimize.least_squares` | identification (C4), inverse kinematics (`Manipulator` calls `scipy.optimize.least_squares` by hand today) |
| **stochastic** | the seed a leaf of `params` (no `J(z, key)`); comes with the optimizer seed strategies | SGD and Adam on minibatches | the Monte Carlo evaluator's objective, the ML lessons |
| **parametric** | `J(z, params)`, `h(z, params)`, `g(z, params)` on the one noun | one compile, many solves; `vmap` over `params`; the sensitivity `dz*/dp` | the MPC (S76, landed) |

A category is read from the program's fields (a `residual` field makes it least squares, as `h`
makes it constrained today), not from a modelling language: CONSTITUTION §1.4 ("not an
optimization modeling language or solver framework") holds. Quadratic and linear programs
(a linear MPC with OSQP) are a later row, if a consumer appears.

### 4.2 The native set: the textbook algorithms, not a solver library

- First order: gradient descent, heavy ball and Nesterov, Adam; projected on the box.
- Second order: Newton (with a line search), Gauss–Newton, Levenberg–Marquardt.
- Constraints: augmented Lagrangian over any of the above.

Each is a few lines of textbook math (RULES §5), on NumPy and JAX; the core needs gradient
descent and Adam. Out of scope: competing with Ipopt on large constrained NLPs, or with
Optax and Optimistix on breadth. `Adam` already says any Optax-style object can replace it.

### 4.3 Three ways to differentiate a solve

| Mode | How | Cost | When |
| --- | --- | --- | --- |
| **Unrolled** | `grad` through the `lax.scan` of `n_iterations=K` | memory grows with `K` | gradients with respect to the method's own parameters (`η`, a learned step size), a fixed small budget, the fixed-iteration MPC (§4.6) |
| **Implicit** | at the answer, `∇_z J(z*, p) = 0`, so `dz*/dp = −H⁻¹ ∂²J/∂z∂p` (the KKT system once there are constraints); a `custom_vjp` around the solve | one linear solve, memory constant | the sensitivity of the answer to the problem: a solve inside a loss, bilevel tuning (later) |
| **None** | `lax.while_loop` to a tolerance (`n_iterations=None`) | cheapest | a batch of solves under `jit` and `vmap`, without gradients |

The implicit mode needs only the converged `z*` and the program's derivatives, so it can wrap
any solver whose answer is accurate, a wrapped one included (through `jax.pure_callback`); that
is a later step.

### 4.4 Wrapped backends

Unchanged in contract: an adapter takes a compiled evaluator and `z0` and returns an
`OptimizationResult`. They stay host code; their improvements are OPW-1–OPW-5 (one backend
table for the offline and the compiled paths, landed in v0.3).

### 4.5 The surface, sketched

`n_iterations=`, `backend=`, `params=` and `z0=` on the solve and
`as_step_system` landed; the other names are placeholders.

```python
# Today, unchanged: a hand-written program, solved to a tolerance (maxiter its cap)
result = Optimizer(program, method="adam").solve()

# A native method on JAX, exactly K steps: the whole solve one lax.scan
opt = Optimizer(program, method="adam", backend="jax", n_iterations=200)
z_star = opt.solve(params=p, z0=z0).z                       # jit-able, vmap-able, differentiable
dz_dp = jax.jacobian(lambda p: opt.solve(params=p, z0=z0).z)(p)

# The process: the iterates as a Trajectory over k
traj = opt.as_step_system().compute_rollout(200)
traj.plot("J")

# The teaching face: the loop as a step diagram
loop = gradient_loop(GradientStep(eta=0.1), GradientOracle(program))  # a StepDiagramSystem
loop.linearize(z_star).poles()                       # the convergence rate
```

### 4.6 The first target, landed: a fixed-iteration MPC

The fixed-iteration MPC is no special block: a trajectory-optimization planner whose method is
native and whose `n_iterations=K` is forwarded to its `Optimizer`, wrapped by
`ModelPredictiveController` like any planner; one tick is

```text
z_k = S^K( pv.shift(z_{k−1}, t_k) ; P_k )               P_k: the planner's tree, x0 and t0 bound
```

with `S` one iteration of the method. Its contract (the tick, the delay, tick 0, the bindings,
the closed loop as one scan and its gradient) is DESIGN §6. Why a fixed `K`, not a tolerance:
the tick's cost is known in advance; `lax.scan` over `K` is traceable in every mode
(`while_loop` refuses reverse-mode `grad`); the gradient of the closed-loop cost is exact for
the controller that runs, truncation included; with the warm start the iterations accumulate
across ticks (Diehl's real-time iteration, the fast-gradient MPC of Richter, Jones and Morari).
Single shooting is the transcription (the input box the program's box, no constraint rows);
state constraints come by penalty or augmented Lagrangian over the same `K` budget (NS-6);
direct collocation stays on Ipopt and SciPy. MPPI fits the same template with a
derivative-free iteration, its seed a leaf of `planner.params` (DESIGN §6, "Path integral
control"); the implicit mode (§4.3) stays for a solve run to convergence.

### 4.7 The long-term target: convergence as stability

§3.1's loop, analysed with the control tools: the step size's stability region from the poles
of `I − ηH`, heavy ball and Nesterov as second-order linear filters in feedback with `∇J`, and
later the robust view (the gradient a sector-bounded nonlinearity, the rate certified for a
whole class of `J`: the integral quadratic constraints of Lessard, Recht and Packard). Its
pieces exist: `as_step_system`, `linearize` and the poles in `z` on a step system, and the step
operators that wire the law to the oracle.

**Ruled 2026-10-25: the slice comes back, later, when solver stability needs it.** The
2026-10-17 ruling dropped the z-domain
analysis family for GRO501; goal 2 needed only the linearization of a step system at a
fixed point and its eigenvalues against the unit circle. Since v0.4 the linear tools read a
step or sampled system in `z` (DESIGN §3, `analysis/`).

## 5. Steps left (local ids; NS-1, NS-8, NS-9 and NS-2's fixed-`K` form landed)

The order and each row's "done when" are TODO §4's.

- **NS-2's rest, the tolerance form**: `n_iterations=None` as a `lax.while_loop` for the native
  methods on a JAX program; a batch of starts solves in one `vmap` call and matches the loop.
- **NS-3 Implicit differentiation** of the unconstrained and box-bounded answer
  (`custom_vjp`); `dz*/dp` matches finite differences on a parametric quadratic and on a small
  least-squares identification.
- **NS-4 The least-squares category** with Gauss–Newton and Levenberg–Marquardt;
  `Manipulator`'s inverse kinematics can use it in place of its hand SciPy call.
- **NS-5 The teaching face**: the loop of §3.1 wired from `as_step_system`, rolled out and
  linearized; one notebook-sized demo (the step-size stability lesson).
- **NS-6 Constraints natively**: augmented Lagrangian over NS-1's kernel, then its implicit
  derivative through the KKT system.
- **NS-7 The stochastic category**: the optimizer seed strategies, SGD and Adam advancing
  the seed, the problem evaluator's Monte Carlo objective their first consumer.

## 6. The asks (all answered)

1. The two-layer split, a pure kernel run by `Optimizer` with the `StepSystem` loop as a view:
   **yes.**
2. Which categories and native algorithms (§4.1, §4.2): **added with the first solver that
   needs them, minimalist.**
3. The rung: a proof of concept in v0.3 (landed), the rest in v0.5.
4. The fixed-count keyword: **`n_iterations=K` on `Optimizer`**; rejected, an `iterate`
   verb and `solve(n_iterations=K)`.
5. The discrete step analysis comes back with NS-5 (§4.7).

## 7. Parametric and stochastic programs

**Choice, 2026-10-27,
on one program noun:**
- inside, a program's functions always see a complete tree; at the user boundary
  (`solve(params=...)`) a partial tree is merged once onto the defaults; no `bind` on programs;
- the seed is an integer leaf found by the `seed` key, as on blocks, policies and sources; integer leaves are allowed in `params` (amending the rule that keeps switches out of `params`) and never enter `wrt`;
  the raw gradient over a whole tree is `jax.grad(f, allow_int=True)`;
- sizes (the horizon `n_steps`, the iterations `K`, batch sizes) are constructor arguments; only
  values live in `params`;
- the warm start is an argument of the solve (`z0=`), not of the program;
- sensitivities through a solve come later, with implicit differentiation (§4.3);
- `J(z, key)` is dropped: the seed lives in `params`.

The core keeps this convention and the sample-average export (`pe.program(wrt, key=,
n_trials=)`: a deterministic program whose `J` is the mean over draws realized once). How a
solver uses the seed (keep it fixed, `vmap` over it, advance it per iteration) is an `Optimizer`
strategy, taken up with the native solvers beyond the proof of concept (NS-7). The design notes
below are for that work.

### 7.1 The maintainer's leaning (2026-10-25)

> Seeds are params, as on a system: **every program is deterministic given its params**. A
> stochastic solver may decide internally to change the seed, to work directly with the
> sampled, noisy function.

It mirrors the system side: a stochastic simulation is the deterministic simulation of one
realized tree, the seeds inside it; a stochastic program is the deterministic program at
one `params` with its seed inside.

### 7.2 Three kinds of stochastic program

| Kind | The objective | Draws | Solvers |
| --- | --- | --- | --- |
| **Sample average** (common random numbers) | `J(z) = 1/N Σᵢ J(z, ωᵢ)` | fixed at build, from one seed | any, Ipopt and SciPy included: it is deterministic (today's `pe.program`) |
| **Stochastic approximation** | `E[J(z, ω)]`, estimated per iteration | fresh at every iteration `k` | native first order only (SGD, Adam); L-BFGS and SLSQP break on a noisy `J` |
| **Risk and chance** | `CVaR`, `P(x ∈ X) ≥ 1 − α` | either | later; the problem's `criterion=` is the hook |

With the seed a leaf of `params`:
- a sample-average program is a parametric program whose seed is bound: a new seed is a new
  instance, solved without recompiling; `vmap` over seeds is a Monte Carlo over solves;
- stochastic approximation is the solver advancing the seed itself, `seed_k = child(seed, k)`,
  the same rule as the sampled controller's, with §3.3's kernel unchanged.

### 7.3 What makes a solver that changes the seed sound

1. **The solve stays deterministic given `params`.** The solver derives iteration `k`'s seed
   from the user's, `seed_k = child(seed, k)`; the same `params` and `z0` give the same iterates,
   on replay and under `jit`; the seed stream is part of the result's record.
2. **One seed per iteration, shared inside it.** Every evaluation within iteration `k` (the
   gradient, a line search, a second sample for a variance estimate) uses `seed_k`: otherwise a
   line search compares two different functions.
3. **The solver touches only the seed leaf.** The rest of `params` is the problem; the seed
   takes no gradient.
4. **The answer is scored at a stated seed**, the user's or a held-out set, never the last
   iteration's noisy sample.

| Solver | Seed handling | Program it sees |
| --- | --- | --- |
| Wrapped (Ipopt, SciPy) and deterministic natives (L-BFGS, Newton) | keeps the seed fixed | the sample average at that seed |
| Stochastic natives (SGD, Adam) | advances it, `child(seed, k)` | a fresh sample per iteration |
| A Monte Carlo over solves | `vmap` over seeds, each solve keeping its own | a family of sample averages |

So the stochastic category is a property of how a *solver* uses the program, not a different
kind of program. Sensitivities `dz*/dp` of a sample-average program are the derivative for a
fixed sample; for stochastic approximation they are not defined the same way.
