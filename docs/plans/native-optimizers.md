# Native optimizers: the solve as a rollout

Status: **ideas to review**, written 2026-10-25 from the maintainer's vision (§1), with the
state of the code (§2) and the agent's read (§3–§6). The maintainer named the two goals the same
day (§1.1): the fixed-iteration MPC first, convergence analysis with control tools long term.
The design choices are not ruled; the asks are in
[QUESTIONS.md](../dev/QUESTIONS.md) ("Blocks v0.9 / v1.0"). Lane: research. Rung: none yet; the
natural slot is after the sampled loop (S31) and the parametric program (S76), both v0.4,
because this plan builds on the step model's rollout and on the program's parameters.

Contracts in code today: [DESIGN.md](../dev/DESIGN.md) §6 (Planning / NLP). The neighbours:
[parametric-program.md](parametric-program.md) (the program's parameters and one backend table,
S76 and OPW-1–OPW-5), [differentiable-problem.md](differentiable-problem.md) (the path from any
parameter to the cost), [sampled-path.md](sampled-path.md) §8 (the one rollout kernel).

---

## 1. The vision (the maintainer, 2026-10-25)

- **An upgraded `MathematicalProgram` with categories**, and an upgraded `Optimizer` whose
  solvers fall in two families: **wrapped backends** (Ipopt, `scipy.optimize.minimize`, other
  libraries) and **native algorithms** implemented in minilink.
- **Native algorithms are pure array code, like a `System`**: the equations are stateless
  functions of arrays, with optional `params` the way `f(x, u, t; p)` has them.
- **The solve is a rollout.** The sequence of iterates of gradient descent is the analog of
  the rollout of a step system, `z_{k+1} = step(z_k, k; p)`. An idea to weigh: the
  optimization algorithm *is* a `StepSystem`, and solving is `rollout`.
- **The ultimate goal: some solvers fully JAX-compatible**, so that the whole solve can be
  jitted, differentiated and vmapped: a solve inside a loss, a batch of starts or of problem
  parameters in one call.

### 1.1 The two goals (the maintainer, 2026-10-25)

1. **First: an MPC whose solver runs a fixed number of iterations**, so the controller's tick
   is fully compilable and even differentiable; the closed loop with the MPC then simulates
   end to end in JAX, very fast.
2. **Long term: analyse the convergence of an optimization process with the control tools**:
   the algorithm as a feedback loop, its convergence as stability. A connection between the two
   fields.

## 2. What exists today

| Piece | Where | What it does | Traceable end to end? |
| --- | --- | --- | --- |
| `MathematicalProgram` | `optimization/mathematical_program.py` | Frozen record of `J(z)`, `h(z) = 0`, `g(z) ≥ 0`, the box, optional derivatives, the native backend | the callables are, if the user writes them so |
| Program classes | `optimizer.py`, `program_class` | Three, read from the fields: `unconstrained`, `bounded`, `constrained`; `method="auto"` picks the class default (S74, landed 2026-10-24) | n/a |
| `Optimizer` | `optimizer.py` | A stateful orchestrator: owns the program, its compiled evaluator, `z0`, the method; `solve()` returns an `OptimizationResult` | no |
| Wrapped backends | `optimizers/scipy_minimize.py`, `optimizers/ipopt.py` | L-BFGS-B, SLSQP, trust-constr; Ipopt through optional `cyipopt` | no (host callbacks) |
| Native backend | `optimizers/first_order.py` | `gd` and `adam`, projected on the box: a Python `for` loop on NumPy arrays; Adam's update is `jax.jit`-ed, so `adam` needs JAX even on a NumPy program | no |
| Program evaluators | `optimization/evaluators/` | NumPy and JAX; the JAX one jits `J`, `∇J`, `∇²J`, the Jacobians, then **returns NumPy arrays** to the solver | the derivatives are; the solve is not |
| Parameters | `planning/trajectory_optimization/parametric_program.py` | `ParametricMathematicalProgram`: only `x0` varies between solves (S76 widens it, v0.4) | partly (the JAX MPC path) |
| `Adam` | `optimizers/first_order.py` | Optax-style `init` / `update` on a pytree; also the RL learners' optimizer (`sac.py`, `algorithms/base.py`) | **yes**: the RL planner runs its gradient steps in one `lax.scan` (`planner.py`), PPO too |
| The step model | `core/discrete/step_system.py`, `core/compile/evaluators/discrete.py`, `stepping.py` | `step(x, u, k, params)`; `rollout` is a Python loop on NumPy, one `lax.scan` on JAX; no `rollout_batch` for step systems yet (the continuous evaluator has one) | yes |
| Other solvers in disguise | `planning/dynamic_programming/dp_jax.py` (value iteration in a `lax.while_loop`), `learning/.../ppo.py` (scan of minibatch steps), MPPI (planned, S72) | each owns its own loop | yes |

Nothing in the library differentiates *through* a solve today: no `custom_vjp`, no implicit
differentiation, no traceable optimizer loop outside RL. The problem evaluator's program
export (`pe.program(wrt=…)`) does give a JAX-traceable `J(z)`, so the inputs a traceable solver
needs already exist.

## 3. The agent's read: the optimizer as a step system

The idea is sound, and it is older than minilink: it is the control-theory view of
optimization algorithms (Lessard, Recht and Packard, 2016). It fits the CONSTITUTION's first
article better than any other option, so it is worth doing properly. Three points decide it.

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

This already runs today, wired by hand (checked 2026-10-25): a one-state `StepSystem` law
`z⁺ = z − η u` with `η = 0.1`, a `Gain(2)` as the oracle of `J = z²`, connected in a
`StepDiagramSystem` and rolled out, gives `z = 1, 0.8, 0.64, 0.512`: the contraction
`1 − 2η`. Two gaps: the `@` shortcut refuses step systems (explicit `connect` only), and a
step system has `jacobian` but no `linearize` or poles yet.

What this buys, beyond reuse:

- **Convergence is stability.** Linearize the loop at the optimum `z*`:
  `z_{k+1} − z* = (I − ηH)(z_k − z*)`. The step size is stable when every eigenvalue of
  `I − ηH` is inside the unit circle, and the rate is the spectral radius. The library's own
  analysis tools would answer "why does this step size diverge?" on the solver itself, once
  step systems get `linearize` and discrete poles (today: `jacobian` only), a lesson GRO860 and
  the control courses would get almost for free.
- **The iterates are a `Trajectory`** (once `StepRollout` folds into it, V1's rest, v0.4):
  the plot of `J` against `k`, the path of `z` on the contour, all with the existing plotting.
- **`vmap` over the start** is a multi-start; over `η`, a step-size sweep; over `p`, a family
  of problems, the same way `rollout_batch` takes a family of rod lengths today.

### 3.2 Where the analogy stops

- **Stopping.** A rollout runs `n_steps`; a solver stops on a tolerance. A fixed count is a
  `lax.scan`, traceable in every mode; a tolerance is a `lax.while_loop`, which `jit` and
  `vmap` take but reverse-mode `grad` does not. So a native solver needs both: `rollout`
  (fixed `n_steps`, the process) and `solve` (stop on tolerance, the answer).
- **Differentiating the answer is not differentiating the process.** Two different
  gradients, both useful (§4.3).
- **Constraints.** A projection on the box fits the step model as is. Equality and inequality
  rows need an outer loop (augmented Lagrangian: a rollout of unconstrained rollouts) or
  linear algebra inside the step (Newton on the KKT system, interior point). Both are pure
  arrays; neither is a two-line law. The production constrained solver stays Ipopt.
- **The `System` contract costs something.** `x` is a flat vector; Adam's state is a pytree
  `(z, m, v, k)`. It flattens to `x = [z; m; v]` with `k` the tick, which is fine for a vector
  `z` and awkward for a neural network's weight tree. The RL learners keep their pytree path.

### 3.3 The recommendation

The two layers, and keep them separate:

1. **The kernel**: each native method is a pure law `init(z0, params) → state` and
   `step(state, grad, k, params) → state`, Optax-shaped (as `Adam` already is), on the array
   module of its inputs (NumPy or JAX, one equation path, CONSTITUTION §2 invariant 5). The
   `Optimizer` runs it: a Python loop on NumPy, `lax.scan` or `lax.while_loop` on JAX.
2. **The teaching face**: the same law exposed as a `StepSystem` update block, and the
   oracle as a static block, so a lesson can wire the loop, roll it out, and linearize it.
   It is a *view* of the kernel, not a second implementation.

Making every solver a `StepSystem` by construction would force the pytree state into a flat
`x` and route the RL learners through ports for nothing; making none of them one loses the
best lesson in the plan. The two-layer split keeps both.

## 4. The design, in outline

### 4.1 Program categories: few, textbook, declared by the program's fields

Today's three classes stay. The additions, each with its consumer:

| Category | The form | What it unlocks | First consumer |
| --- | --- | --- | --- |
| unconstrained / bounded / constrained | as today | the method table of S74 | every program |
| **least squares** | `J(z) = ½‖r(z)‖²`, the residual `r` given | Gauss–Newton, Levenberg–Marquardt (native); `scipy.optimize.least_squares` | identification (C4), inverse kinematics (`Manipulator` calls `scipy.optimize.least_squares` by hand today) |
| **stochastic** | `J(z, key)`, a sampled objective | SGD and Adam on minibatches | the Monte Carlo evaluator (`pe.evaluate(p, key)`), the ML lessons |
| **parametric** | `J(z, p)`, `h(z, p)`, `g(z, p)` | one compile, many solves; `vmap` over `p`; the sensitivity `dz*/dp` | the MPC (S76's `ParametricProgram`) |

A category is read from the program's fields (a `residual` field makes it least squares, as `h`
makes it constrained today), not from a modelling language: CONSTITUTION §1.4 ("not an
optimization modeling language or solver framework") holds. Quadratic and linear programs
(for a linear MPC with OSQP) are a later row, if a consumer appears.

### 4.2 The native set: the textbook algorithms, not a solver library

- First order: gradient descent, heavy ball and Nesterov, Adam; projected on the box.
- Second order: Newton (with a line search), Gauss–Newton, Levenberg–Marquardt.
- Constraints: augmented Lagrangian over any of the above.

Each is a few lines of textbook math (RULES §5), on NumPy and JAX. Out of scope: competing with
Ipopt on large constrained NLPs, or with Optax and Optimistix on breadth. A bridge stays open:
`Adam` already says any Optax-style object can replace it.

### 4.3 Three ways to differentiate a solve

| Mode | How | Cost | When |
| --- | --- | --- | --- |
| **Unrolled** | `grad` through the `lax.scan` of `n_steps` | memory grows with `n_steps` | gradients with respect to the method's own parameters (`η`, a learned step size), a fixed small budget, the meta-learning lessons |
| **Implicit** | at the answer, `∇_z J(z*, p) = 0`, so `dz*/dp = −H⁻¹ ∂²J/∂z∂p` (the KKT system once there are constraints); a `custom_vjp` around `solve` | one linear solve, memory constant | the sensitivity of the answer to the problem: a solve inside a loss, a differentiable MPC (parametric-program.md §5), bilevel tuning |
| **None** | `lax.while_loop` to a tolerance | cheapest | a batch of solves under `jit` and `vmap`, without gradients |

The implicit mode needs only the converged `z*` and the program's derivatives, so it can wrap
any solver whose answer is accurate, a wrapped one included (through `jax.pure_callback`); that
is a later step.

### 4.4 Wrapped backends

Unchanged in contract: an adapter takes a compiled evaluator and `z0` and returns an
`OptimizationResult` (parametric-program.md §6's design law). They stay host code; their
improvements are OPW-1–OPW-5 (one backend table for the offline and the compiled paths).

### 4.5 The surface, sketched

Names are placeholders for the naming session; the snippet shows the shape.

```python
# Today, unchanged: a hand-written program, solved once
result = Optimizer(program, method="adam").solve()

# A native method on JAX: the whole solve traced
opt = Optimizer(program, method="adam", compile_backend="jax")
z_star = opt.solve_traced(z0, params)                 # jit-able, vmap-able, implicit grad w.r.t. params
dz_dp = jax.jacobian(lambda p: opt.solve_traced(z0, p))(params)

# The process: the iterates as a trajectory
traj = opt.rollout(z0, n_steps=200)                   # unrolled, a Trajectory over k
traj.plot("J")

# The teaching face: the loop as a step diagram
loop = gradient_loop(GradientStep(eta=0.1), GradientOracle(program))  # a StepDiagramSystem
loop.linearize(z_star).poles()                        # the convergence rate (needs step linearize)
```

### 4.6 The first target: a fixed-iteration MPC, compiled end to end

**What exists.** `MPCStatefulController` (`planning/trajectory_optimization/mpc/controller.py`)
is already the ruled shape (R104: a sampled controller's memory is its state): a `StepSystem`
whose state `x` is the packed plan `z` of the last tick, warm-starting the next. What stops it
compiling is the inside of its `step`: a host solve (SciPy SLSQP) through a tick latch with side
effects. The sampled loop's JAX face that would run it (S31, R105: a `Computer` runs the sampled
kernel's JAX face under `problem.compile(policy)`) is v0.4 work.

**The law.** One tick is `K` iterations of a native method on the parametric program, bound
with the measurement:

```text
z_{k+1} = step_MPC(z_k, y_k; p) = S^K( shift(z_k) ; x0 = y_k, p )
u_k     = first input of z_{k+1}
```

`shift` moves the last plan one tick forward (the warm start), `S` is one iteration of the
method (projected fast gradient, Adam), and `K` is a structural constant fixed at
construction, like a transcription's `n_steps`. Every piece is pure array code, so the tick is
one jitted function and the closed loop `controller % dt @ plant` is one `lax.scan`.

**Why a fixed `K` is the right contract here**, not a tolerance:
- The tick's cost is known in advance: a real-time budget, the same on every tick.
- `lax.scan` over `K` is traceable in every mode; a tolerance would need `while_loop`, which
  reverse-mode `grad` refuses.
- The gradient of the closed-loop cost is then **exact for the controller that runs**,
  truncation included: tuning sees the truncated solver honestly, not an idealised optimum.
- With the warm start, the iterations accumulate across ticks: the loop keeps converging while
  the plant moves (the real-time-iteration idea of Diehl, and the fast-gradient MPC of Richter,
  Jones and Morari).

**The transcription.** Single shooting first (`shooting.py`, already a `lax.scan` rollout): the
dynamics leave the program, the input box is the program's box, so the program is box-bounded
and projected first-order methods solve it with no constraint rows. State constraints come
second, by penalty or augmented Lagrangian over the same `K` budget (NS-6). Direct collocation,
whose dynamics are equality rows, stays on Ipopt and SciPy.

**What it unlocks.**
- The closed loop with an MPC simulates as fast as any compiled diagram, and `vmap` runs a batch
  of starts or of plant parameters in one call (the Monte Carlo evaluator's trials).
- `jax.grad` of the closed-loop cost with respect to the MPC's cost weights, its horizon's
  terminal weight, or the step size: MPC tuning by gradient, through the same
  `problem.compile(policy)` path the PID tuning uses (DESIGN §6, the problem evaluator).
- A first differentiable MPC without implicit differentiation; the implicit mode (§4.3) stays
  for a solve run to convergence.

**What it needs first.** S76's parametric program (the tick binds `x0 = y` and the plant's
parameters without a recompile); NS-1 and NS-2 (the native kernel and its fixed-`K` traced
form); for the closed loop, S31's sampled kernel with its JAX face.

### 4.7 The long-term target: convergence as stability

§3.1's loop, analysed with the control tools: the step size's stability region from the poles
of `I − ηH`, heavy ball and Nesterov as second-order linear filters in feedback with `∇J`, and
later the robust view (the gradient a sector-bounded nonlinearity, the rate certified for a
whole class of `J`: the integral quadratic constraints of Lessard, Recht and Packard). It needs
`linearize` and discrete poles on step systems (today `jacobian` only), and the `@` shortcut on
step systems or a helper that wires the law to the oracle.

## 5. Steps (local ids NS-1–NS-9; none scheduled)

- **NS-1 The kernel**: `init` / `step` laws for `gd` and `adam`, on the array module of their
  input; the `Optimizer` loop on NumPy unchanged in behaviour (the AGENTS refactor recipe, a
  byte-identical baseline), and `adam` runs on NumPy without JAX.
- **NS-2 The traced solve**: a `lax.while_loop` and a `lax.scan` form for the native methods on
  a JAX program; done when a batch of starts solves in one `vmap` call and matches the loop.
- **NS-3 Implicit differentiation** of the unconstrained and box-bounded answer
  (`custom_vjp`); done when `dz*/dp` matches finite differences on a parametric quadratic and
  on a small least-squares identification.
- **NS-4 The least-squares category** with Gauss–Newton and Levenberg–Marquardt; done when
  `Manipulator`'s inverse kinematics can use it in place of its hand SciPy call.
- **NS-5 The teaching face**: the update law and the oracle as step blocks, their loop rolled
  out and linearized (needs `linearize` and discrete poles on step systems, a row of its own);
  one notebook-sized demo (the step-size stability lesson).
- **NS-6 Constraints natively**: augmented Lagrangian over NS-1's kernel, then its implicit
  derivative through the KKT system.
- **NS-7 The stochastic category**: `J(z, key)` with SGD and Adam, the problem evaluator's
  Monte Carlo objective its first consumer.
- **NS-8 The fixed-iteration MPC tick** (§4.6): the stateful MPC block's `step` as `K`
  iterations of a native method on a single-shooting parametric program, warm-started by the
  shift; done when one tick jits, its result matches the host SLSQP plan within a tolerance on
  the pendulum and the cart-pole after enough ticks, and `jax.grad` of one tick's input with
  respect to a cost weight matches finite differences.
- **NS-9 The closed loop in one scan** (§4.6): the MPC loop simulated through the sampled
  kernel's JAX face; done when `problem.compile(mpc_computer).evaluate(p)` runs, `vmap`s over
  starts, and its gradient with respect to the MPC's cost weights tunes a demo by `Optimizer`.

**Order, by the maintainer's two goals.** The MPC path first: NS-1 (any time, internal) →
NS-2's fixed-`K` form → NS-8, after S76's `ParametricProgram` (so the tick binds `x0` and the
plant without a recompile) → NS-9, after S31's sampled kernel (v0.4); NS-6 when the MPC needs
state constraints. The control-analysis path is long term: NS-5 with `linearize` and poles on
step systems, after the `StepRollout` fold into `Trajectory` (V1's rest, with S31). NS-3, NS-4
and NS-7 when their consumers are scheduled (a solve to convergence inside a loss, C4
identification, the Monte Carlo objective).

## 6. Open (the asks, in QUESTIONS.md)

1. Is the two-layer split right: a pure kernel run by `Optimizer`, with the `StepSystem` loop as
   a teaching view, rather than every solver a `StepSystem` by construction?
2. Which categories and which native algorithms are in scope (§4.1, §4.2), against
   CONSTITUTION §1.4?
3. Where does this enter the roadmap? The maintainer's priority is the MPC path (§1.1); the
   ask left is the rung: with S76 and S31 in v0.4, or after them.
