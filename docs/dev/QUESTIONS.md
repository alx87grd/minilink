# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

Answers are recorded as **preliminary choices**, open to revision when the final architecture is
drafted, in a dated record under [docs/reviews/](../reviews/) (the latest:
[R124–R140](../reviews/2026-10-27-design-questions-rulings.md) and
[R141–R153](../reviews/2026-10-27-core-vision-decisions.md), 2026-10-27), in the plan doc each
shapes, and in TODO; the entry then leaves this file. `§N` names the workstream of
[core-architecture.md](../plans/core-architecture.md).

## The core

### One score: the three points left (an investigation for the final design)
- **Decided (2026-10-27, preliminary; [R141, R142, R150](../reviews/2026-10-27-core-vision-decisions.md)):**
  two problem classes, `PlanningProblem` and `StepPlanningProblem`; a problem's step face is
  compiled, `problem.compile(dt=)` giving `transition(x, u, k) -> (x_next, c_k)`, `N` and
  `γ = e^{−ρ dt}`; the stage cost integrated with the state in the stationary form; the RL
  discount derived from the problem. Every stepping tool reads that one face.
- **Still open, the points to settle together:**
  - **time-varying sets and costs, and the margins between samples:** they read `t`; the step face
    evaluates them at `t_k`, so a violation between two samples is not seen. Options: accept
    the sampled check (the score is of the sampled loop); check at the substeps the dense run
    already computes (R147); or a margin integrated over the period like the cost;
  - **one failure rule and the input hold:** `score_run` integrates to the horizon and checks `X`
    and `U` at every sample; the RL environment stops at the first exit and checks no `U`; value
    iteration prices the cells that leave its grid. Options: absorbing (the first exit ends the
    run and pays the price, the RL rule) or to the horizon (today's judge, every violation
    priced); the hold is the zero-order hold of `transition` (the reviewers' view) unless a
    planner states otherwise;
  - **shooting's reported score:** shooting reports its own trapezoid with a first-order hold.
    Options: lower it onto `transition` (one `J`), or keep it as the transcription's own
    objective, reported beside the judge's.
- **Reviewers (2026-10-27):** to the horizon, the RL early stop a training choice reporting the
  judge's `J`; the zero-order hold; shooting's score lowered; value iteration on a finite horizon
  (S63) in v0.4.
- **Detail:** [differentiable-problem.md](../plans/differentiable-problem.md) §9;
  [core-architecture.md](../plans/core-architecture.md) §6
- **Answer:**

### The discretized problem's cost (§6)
- **Decided (2026-10-27, preliminary; [R141](../reviews/2026-10-27-core-vision-decisions.md)):**
  `discretize(problem, dt)` returns a `StepPlanningProblem` on the plant's `SampledSystem`; "a
  sampled problem has no cost object of its own", its `c_k` produced by `transition` (R142).
- **The gap:** `StepPlanningProblem` then has two shapes: a native one with a `StepCost`
  (`c(x, u, k)`, `h(x, k)`) and a sampled one whose `cost` is nothing a user can call.
  `problem.discretize(dt).cost.c(x, u, k)` has no answer, and the class R141 justified by its
  fields has a field that is sometimes empty.
- **Options:**
  - (a) a `SampledCost(g, h, plant, dt, *, solver=, substeps=)`, a `StepCost` whose `c(x, u, k)`
    integrates `e^{−ρτ} g` over one period from `x` under the held `u` when called directly (a
    second plant evaluation only on that direct call; the compiled face still fuses it into the
    transition), so the class keeps one shape and the discretized problem prints its cost;
  - (b) `problem.compile(dt=)` the one entry in v0.3, and the explicit `discretize(problem, dt)`
    a v0.4 row that lands with its first consumer;
  - (c) as written: the sampled problem's `cost` is `None`, stated in the docstring.
- **Recommendation (agent):** (a), R142's integral stated as an object.
- **Detail:** [core-architecture.md](../plans/core-architecture.md) §6;
  [differentiable-problem.md](../plans/differentiable-problem.md) §9.1.
- **Answer:**

### One owner of the plan's time in the MPC (§8)
- **Decided (2026-10-27, preliminary; [R148](../reviews/2026-10-27-core-vision-decisions.md)):**
  `pv.solve` records in `z` the origin it solved from; the MPC's state is `(z, t_last)`,
  `n = pv.n_z + 1`; its `step` runs `z0 = pv.shift(z, t_k − t_last)`.
- **The gap:** one number, two homes (RULES 5.6): `z` holds the origin and the MPC holds
  `t_last`, kept equal by hand, on a failed solve included.
- **Options:**
  - (a) the MPC's state is `z` alone (`n = pv.n_z`); `shift(z, t) -> z` rebases the plan to the
    absolute time `t` from the origin `z` records; a failed solve keeps `z` with its old origin
    and the next tick rebases from it, no special case; the MPC still reads nothing inside `z`:

    ```python
    def step(self, x, u, k=0, params=None):      # x is z; t from the clock
        z0 = pv.shift(x, t)                      # the same plan, rebased to now
        z_new, status = pv.solve(P_k, z0=z0)
        return where(status.success, z_new, x)
    ```

  - (b) `z` holds no time: `shift(z, dt)` is relative, the MPC owns `t_last` and passes
    `t − t_last` to `policy`;
  - (c) as written.
- **Recommendation (agent):** (a); the planner already owns the origin, so the block carries
  nothing of its own.
- **Detail:** [planner-and-mpc.md](../plans/planner-and-mpc.md) §3, §6.2.
- **Answer:**

### The clock port `t` (§8; [automation-by-convention.md](../plans/automation-by-convention.md) §4.6)
- **Decided (2026-10-27, preliminary; R148):** time reaches the MPC by a standard input port
  `t`, wired by convention to the loop's clock, so the MPC "never knows `Δ`"; where the clock
  comes from in a continuous diagram is the landing change's detail.
- **Why it exists:** the delayed MPC is a pure `StepSystem`, whose `step(x, u, k)` sees the
  tick, not seconds; every equation path already takes `t` as an argument (CONSTITUTION §2, "One
  signature"), so the port is a second channel for time.
- **Options:**
  - (a) the port, as ruled: a wiring convention every time-reading step block will need (a
    reference tracker, a scheduled gain, the MPC), and a clock source in continuous diagrams;
    delay compensation is rewiring `t + Δ`;
  - (b) the `SampledSystem` wrapper, which knows `Δ`, hands its source the clock: for a pure step
    source that declares it reads seconds, the wrapper binds `t_k = t_start + k·Δ` as it binds
    `t_k` for a continuous source today, and the MPC binds `t_start` from that `t`, no port;
    delay compensation is a keyword on the MPC (`lookahead=Δ`) or a predictor block on
    `x_start`.
- **Recommendation (agent):** (b) keeps one channel for time; (a) is defensible if a clock
  source is wanted for other blocks anyway. Choose knowingly.
- **Answer:**

### The CONSTITUTION amended for the sampled loop and the draws (vision; §2, §3, §5)
- **Decided (2026-10-27, preliminary; R146, R128):** the sampled loop is a
  `SampledDiagramSystem` of the step algebra, `Computer` and `HybridDiagram` go; a disturbance
  is a source block. The amendments are deferred to the landing changes.
- **The gap:** invariant 1, invariant 3, §6's `%` sentence and §2's disturbance sentence say the
  opposite today, and AGENTS.md has every agent read the CONSTITUTION first: the first agent on
  the loop lane meets a conflict the rules say to stop on.
- **Proposed text** (four sentences, to rule before the loop lane starts; the rest of each
  paragraph unchanged):
  - **invariant 1**, replacing "The sampled loop (`Computer`, `HybridDiagram`) that couples a
    discrete clock to a continuous plant is a simulation tool, not a `System`, keeping discrete
    scheduling out of continuous dynamics `f`": "A sampled loop (a computer closed on a plant,
    `SampledDiagramSystem`) is a step diagram of that sibling algebra: a clock is applied to a
    block from outside (`%`, `discretize`), never built into continuous dynamics `f`.";
  - **invariant 3**, replacing "Hybrid and step composition (`Computer @ plant`,
    `HybridDiagram`) is a sibling algebra": "Step and sampled composition (`%`, then `@`, `>>`
    and `+` between step systems; `sampled @ continuous` the one place the two worlds meet) is a
    sibling algebra";
  - **§6, the grammar**, replacing "The sibling hybrid algebra adds exactly one operator, `%`,
    which schedules a block into a `Computer`": "The sibling step algebra adds exactly one
    operator, `%`, which puts a block on a clock (`sys % Δ` is `discretize(sys, Δ,
    solver="euler")`); its `>>`, `+` and `@` are the same grammar on step systems";
  - **§2, `PlanningProblem`**, replacing "the stochastic sibling adds distributions over starts,
    parameters and disturbances": "stochastic by its parts: a law where a number goes (the
    start, a parameter), and a disturbance a noise source block wired onto a port".
- **Answer:**

### A performance budget for the core (performance; §5, §7)
- **Today:** §7's gate says "if carrying `params` costs speed, an export option bakes them in",
  with no number, and the definition of done has no budget. The vision measured (CPU, float64,
  [its §7.5](../reviews/2026-10-27-consolidated-core-vision.md)): the one score, 1000 starts ×
  200 RK4 steps in 44 ms; the sampled loop, 100 starts × 3000 base ticks in 93 ms; the native
  MPC's closed loop, 60 ticks × 15 iterations × a 20-step horizon in 134 ms a call; a parameter
  family at 278 ms against 27 ms for the plain batch (S53).
- **Options:**
  - (a) a ninth script in the definition of done: the compiled sampled loop, the one score and
    the native MPC's loop each within 2× of the vision's measurement at the same sizes, the
    parametric program's solve within 2× of the same program with its parameters frozen, and the
    parameter family within 2× of the plain batch (S53's bar);
  - (b) no budget in v0.3; the gate stays qualitative;
  - (c) another factor, or only the parametric gate quantified.
- **Recommendation (agent):** (a).
- **Answer:**

### `dt` or `sample_period` (API and naming; §5)
- **Today:** the calls say `dt` (`discretize(sys, dt)`, `problem.compile(dt=)`,
  `discretize(problem, dt)`, `sys % dt`) and the objects say `sample_period` (R146's rename of
  `Discretization.dt`; the noise blocks, R128; `GaussianPolicy(sample_period=)`, R129). R134's
  rule is one keyword per concept.
- **Options:**
  - (a) `dt` wherever a tool integrates or steps (the calls); `sample_period` on every block that
    holds a clock (the sampled system, the noise sources, the seeded policy): two words for two
    things, an integration step and a block's clock, stated in one line of DESIGN §4;
  - (b) `sample_period` at the calls too (`discretize(sys, sample_period=)`), one word;
  - (c) `dt` on the objects too.
- **Recommendation (agent):** (a); `discretize(plant, dt, substeps=)` already separates a period
  from the step underneath it.
- **Detail:** TODO §1.1, the names row.
- **Answer:**


## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open (R139). R82 (no alias, one sweep) holds meanwhile.
- **Blocks:** the trust contract (V4); the renames that wait for the freeze (`xbar` / `ubar` →
  `x_bar` / `u_bar`, `plot_cost2go` → `plot_cost_to_go`; `NeuralPolicy` moved to v0.3, R151).
- **Today:** no alias or shim ever (R82). The frozen names are drawn at v0.9 from one rule (R139).
- **Options still open:** do the pending renames before the freeze, so none is needed after it;
  a shim of another length; a major version per rename; a snapshot test of the export list
  either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
