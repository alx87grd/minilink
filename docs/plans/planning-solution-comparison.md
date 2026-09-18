# One solution for LQR too, and a compare verb over solutions

Status: landed 2026-09-17 (steps 1–3 of §4: `LQRPlanner`, the solution contract and verbs,
`compare`; R1–R8 taken as recommended). Open: the demo and notebook rewrites (§4 step 3,
maintainer-owned, a runnable draft of `vi_pendulum_lqr.py` was shown), solver-specific views
on the records (§2.4, maintainer), `cost_to_go` as a `Field` (phase 4). Delete this file once
those land.
Depends on core-objects-4-fields.md (4.2 `QuadraticField`, 4.4 `GridField`, 4.6 `CallableField`)
for the cost-to-go plot; everything else stands alone.

## 1. What the audit found

**The `PlanningSolution` contract is complete on five planners.** Trajopt and RRT hand back
a `TrajectorySource` policy, the native trajectory and their record; DP and tabular learning a
`LookupTableController`, the interpolated table as `cost_to_go`, rollout and score opt-in;
RL the neural law, the critic as `cost_to_go` when the discounts match.
`test_planning_solution.py` replays every policy against its trajectory. MPC reads
`cmd.solution.trajectory`. Nothing to fix there.

**LQR is the hole.** `control/lqr.py` is four array-in / block-out factories. `lqr_gain`
solves the Riccati equation and throws `P` away; `trajectory_lqr` sweeps `S` and throws it
away; only `lqr_gain_schedule` returns `S`. Nine example files dig the gain or the Riccati
matrix back out by hand: `ctl.params["K"]` then `eigvals(A - B @ K)` (tutorial 03, gro501
`numpy_state_space`, gro860 `cartpole_lqr`); `solve_continuous_are` → `S` turned into a
cost-to-go and compared with `J*` (`pendulum_cost_to_go_approximation`); `K_row`, `ubar`
used to rasterize the LQR law onto the DP grid (`…vs_ppo_sb3`); `PolicyEvaluator` +
`plotting.plot_value(grid, J_lqr)` to draw an LQR cost-to-go beside VI's (`vi_pendulum_lqr.py`).

**Every plot verb sits on the planner, so a solution cannot show itself.** `plot_solution`,
`animate_solution` (base), `plot_cost2go`, `plot_policy` (DP, tabular), `plot_learning_curve`,
`plot_tree`. The solution has `__str__` only. Consequences in the examples:

| pattern | files |
| --- | --- |
| `planner.solve().trajectory` — the solution discarded in the same expression | 19 |
| a hand-rolled comparison of two or more methods on one problem | 19 |
| `closed_loop(ctl, x0, name)` / `make_pendulum()` / `simulate(ctl, x0)` fixture helpers | 7 |
| `plot_trajectory` / `plot_cost` / `animate` repeated once per method | 8 |
| `plt.subplots` overlays or grids of results (phase plane, side-by-side trees, `J*` vs `Ĵ`) | 10 |
| hand-formatted tables with a per-file `@dataclass` row (`SolveRun`, `ParityRow`, …) | 7 |
| `MonteCarloEvaluator(...).evaluate(ctl)` in a loop over `(name, ctl)` pairs | 10 |
| planner internals read for facts (`solution_node.cost`, `tree.nodes`, `result.J`, `history`) | 11 |

The reference case is `pendulum_value_iteration_vs_lqr_vs_rl.ipynb` (and its topics twin):
three laws, three `plot_policy` / `plot_control_law` calls with hand-passed `bounds`,
`vmin`, `vmax`, a `for ctl, name in (...)` loop that wires, names, simulates and plots, three
`animate` cells, three `plot_cost` cells, then `for name, ctl in ...: print(name, evaluator.evaluate(ctl))`.

## 2. Proposal

Three steps, each shippable alone. Code shown is the target reading of the notebook above.

### 2.1 `LQRPlanner` — the linear-quadratic method returns a solution

```python
lqr = LQRPlanner(problem)            # x_bar = cost.xbar, u_bar = cost.ubar, Q, R, S from the cost
solution = lqr.solve(evaluate=True)  # policy, cost_to_go, record, nominal rollout, score
print(solution)
```

- Lives in `planning/policy_synthesis/lqr.py`, subclasses `Planner`; `control/lqr.py` stays the
  array-in / block-out factory it is and the planner calls it (dependency law: planning already
  imports `control.neural`).
- Lowering at the solver boundary (RULES 4.3): `problem.cost` must be a `QuadraticCost`
  (`Q`, `R`, `S`, `xbar`, `ubar`); anything else raises "LQR needs a QuadraticCost". `A`, `B`
  from `analysis.derivatives.jacobian` at `(xbar, ubar)`, as `lqr_at_operating_point` does.
- Horizon from the problem: `tf` infinite or unset → ARE, `StateFeedbackController`, cost-to-go
  `(x − x̄)ᵀ P (x − x̄)`; finite `tf` → `lqr_gain_schedule(A, B, Q, R, S_f=cost.S, tf)`,
  `TimeVaryingStateFeedbackController`, cost-to-go `(x − x̄)ᵀ S(t) (x − x̄)`. Both are the
  `QuadraticField` of phase 4.2 (matrix or `(t, S)` schedule); until 4.2 lands, a closure like
  the RL critic.
- `RiccatiRecord(K, P, closed_loop_poles, horizon)`: `success` is "A − BK Hurwitz" for the
  infinite horizon, "schedule finite" otherwise; `__str__` prints the gain and the poles —
  what the three notebooks print by hand today.
- `evaluate=True` rolls the continuous law out through the evaluator's `"simulator"` backend
  (no control period to invent) and scores it on the problem's draws; the trajectory grid is a
  reporting choice, `dt=0.01` documented on the class.
- The LQR cost-to-go is the Riccati one: what the method produces (the contract's wording).
  The cost-to-go *of* the LQR law on the nonlinear plant stays `PolicyEvaluator`; both are
  the lesson in `vi_pendulum_lqr.py`.
- `trajectory_lqr` along a reference is a second constructor later (`LQRPlanner(problem,
  reference=traj)`), not in this slice.

### 2.2 The workflow stays explicit (ruled 2026-09-17)

The main output of a planner is a controller or a trajectory, and the demo says which:

```python
solution = planner.solve()          # policy + evidence
ctl = solution.policy               # feedback family …
cl = ctl @ plant                    # … the loop is closed in the demo, never inside a verb
cl.compute_trajectory(tf=TF)
cl.plot_trajectory()
cl.animate()

traj = solution.trajectory          # trajectory family: the plan itself
plant.plot_trajectory(traj)
replay = solution.policy >> plant   # the plan replayed open loop
```

Plotting the plan and simulating the closed loop are two different things and stay two
different lines. The solution's `trajectory` is the planner's own evidence — the plan of an
open-loop planner, the nominal rollout of a feedback law when `evaluate=True` — and is
labelled as such wherever it is drawn. No `closed_loop()` or `simulate()` verb on the solution.

What the solution draws, then, is what the planner produced:

```python
solution.plot_control_law(u_axis=0)   # the law over the problem's box and U (was three hand-passed kwargs)
solution.plot_cost_to_go(axes=(0, 1)) # the field over the state box, anchor at x_goal
solution.plot_trajectory()            # solution.trajectory, titled "plan" or "nominal rollout"
print(solution)
```

- `plot_control_law` delegates to the policy's own verb and fills `bounds` from
  `problem.X.bounding_box()` (or the state box) and `vmin` / `vmax` from `problem.U`. An
  open-loop solution raises and names `plot_trajectory`.
- `plot_cost_to_go` draws a `Field` over two state axes with the rest pinned at `x_goal` — the
  `port_map` sweep applied to a field; one function for the DP table, the Riccati form and the
  critic once they are `Field`s (phase 4.2 / 4.4 / 4.6). DP keeps `plot_cost2go` / `plot_policy`
  for the discrete tables.
- Planner verbs stay and delegate to the latest solution (`plot_solution` →
  `require_solution().plot_trajectory()`), RULES 7.1.
- `problem` on the solution (R1) is what lets these verbs run: labels from `problem.sys`, the
  box from `problem.X`, the clim from `problem.U`, the cost for a score.

### 2.3 `compare` — planner outputs side by side

```python
race = compare(VI=sol_vi, LQR=sol_lqr, PPO=sol_ppo)
print(race)                               # name | success | solver record | evaluation when present
race.plot_control_law(u_axis=0)           # one figure, one clim, one panel per law
race.plot_cost_to_go()                    # J*, (x−x̄)ᵀP(x−x̄), the critic, same box
race.plot_trajectory()                    # the plans / nominal rollouts, overlaid and labelled as such
print(race.evaluate(evaluator))         # the same table, every policy scored on one evaluator's draws
```

The table for the pendulum:

```
        success   solver                                                     evaluation
VI      yes       converged in 412 sweeps (delta=0.0913)                     —
LQR     yes       K = [[3.16 1.42]], poles -1.8±1.1j                          —
PPO     yes       PPO: 200000 plant steps in 14.2 s, mean episode cost 131.3  J over 100 trials: mean 126.9 …
```

- A frozen record over `{name: PlanningSolution}` (RULES 5.21) whose verbs are the overlays;
  the panels reuse the verbs of 2.2 with `ax=` and `show=False`. The alternative with no new
  noun is module functions over a plain dict — same code, less readable in a cell. R4 decides.
- Closed-loop comparison is **not** a compare verb. The loops are closed in the demo, one per
  law, and simulated with the Simulator as today; the three `plot_trajectory` / `animate` cells
  stay (RULES 6.14, one figure per cell). What the plotting lane may add is an overlay on the
  plant over explicit trajectories — `plant.plot_phase_plane({"VI": traj_vi, "LQR": traj_lqr})`
  — for the showcase's hand-drawn phase-plane cell; that is a `System` verb over trajectories,
  not a solution verb.
- `race.evaluate` replaces the `for name, ctl in ...: print(name, evaluator.evaluate(ctl))`
  cell; each solution's own `evaluation` fills the table when present.

## 2.4 The contract, with LQR in the family

```python
@dataclass(frozen=True)
class PlanningSolution:
    problem: PlanningProblem      # what was solved: sys, sets, cost, horizon. A frozen record;
                                  # its `sys` is a live reference, not a snapshot
    policy: System                # u = pi(x) feedback block · u = pi(t) TrajectorySource ·
                                  # u = pi(x, t) time-varying feedback (finite-horizon and trajectory LQR)
    solver: record                # one frozen dataclass per planner: `success`, one-line `__str__`,
                                  # the method's own facts and the views of them
    trajectory: Trajectory | None # the schedule (open loop) or the nominal rollout from
                                  # problem.x_start (`evaluate=True`); its own grid records dt
    evaluation: Evaluation | None # the cost on the problem's draws (`evaluate=True`)
    cost_to_go: Field | None      # the method's own J: the DP / tabular table, the Riccati form
                                  # (the linear model's J, J(x, t) on a finite horizon), the critic
```

Properties `success` (the record's), `open_loop` (`policy.m == 0`). Verbs that read only these
fields, hence one implementation for every planner: `__str__`, `plot_trajectory`, `animate`,
`plot_control_law`, `plot_cost_to_go`, `plot_cost`. Each delegates to the object that owns the
drawing — `problem.sys.plot_trajectory(trajectory)`, `policy.plot_control_law(...)` — the
same way `Planner.plot_solution` does today.

**Where the solver-specific part lives.** A fact of the solve is a field of the record; a view
of that fact is a method of the record; the planner keeps a one-line shortcut to its latest
solution. Nothing solver-specific reaches `PlanningSolution` itself.

| planner | record facts today | views (record method, planner shortcut) | evidence to add to the record |
| --- | --- | --- | --- |
| trajopt | cost, residuals, feasibility, iterations, stats | — (`iteration_history` plot, later) | `iteration_history` |
| RRT, RRT\* | reached_goal, iterations, nodes, cost, history | `plot_tree`, `animate_search`, `animate_convergence` | the `tree` |
| DP | iterations, delta, tol, converged | `plot_cost2go`, `plot_policy` (the tables), `animate_*` | the tables (`grid`, `J`, `pi`, `history`) |
| tabular | episodes, visited fraction, cost, history | `plot_learning_curve`, the table views | the `Q` table |
| RL | algorithm, timesteps, time, cost, history | `plot_learning_curve` | — |
| LQR | `K`, `P` or `(t, S)`, closed-loop poles, horizon | `plot_gain_schedule` (finite horizon) | — |

So `solution.solver.plot_tree()` and `planner.plot_tree()` draw the same figure; `print(solution)`
still prints the record's one line. A frozen record holding a reference to a 20k-node tree or
the DP tables costs nothing: the planner holds them today.

**Two caveats the contract carries.** The verbs draw the stored `trajectory`; they never
re-simulate, so a later `plant.x0 = ...` on the shared plant does not change what a solution
shows. And `compare` may hold solutions of different problems on one plant (VI on the
deterministic problem, PPO on its stochastic sibling): each solution's own `evaluation` is
comparable only when the starts agree, which is why `race.evaluate(...)` on one problem is the
yardstick.

## 2.5 The landscape: solve, evaluate, verify, compare

Four verbs act on a problem and a solution. The contract of `PlanningSolution` is solid when
each future tool finds what it needs on the solution without a new field.

**Claim versus measurement.** Everything on a `PlanningSolution` is the solver's own claim,
under the solver's own model: the DP table on its grid, the Riccati form on the linear model,
the critic's fit, the NLP's cost and residuals. `cost_to_go` is therefore *the method's
estimate*, never a measurement; the docstring says so. A measurement is what an evaluation verb
returns, and it is a separate object — `Evaluation`, or a `Field` from a policy evaluator —
never written back onto the solution. The gap between the two is the lesson
(`J*(x0)` printed next to the simulated arrival time in `double_integrator_minimum_time`).

| verb | question | takes | returns | today |
| --- | --- | --- | --- | --- |
| **solve** | what is the law? | problem | `PlanningSolution` | six planners (LQR to add) |
| **evaluate** | how good is this law on this problem? | problem + policy | `Evaluation` (samples) or a `Field` (the law's J on a grid) | `MonteCarloEvaluator`, `PolicyEvaluator`, `score_trajectory` |
| **verify** | is this solution what it claims? | solution | a residual field, a residual along the plan, a certificate | `test_planning_solution` replay; `region_of_attraction` on the loop; trajopt `feasible` |
| **compare** | which law, side by side? | named solutions | table and overlays | hand-written, 19 files |

**Evaluate takes a policy, not a solution.** That is its strength: a hand-written PD, an
SB3 policy and an LQR are scored by one yardstick. The evaluator owns the problem (one set of
draws for every law), so `evaluator.evaluate(solution)` is sugar that reads `solution.policy`
and nothing else — the solution's own `problem` may be the deterministic sibling of the
evaluator's. Two methods, one verb: sampling (`MonteCarloEvaluator`, any law, draws applied)
and the Bellman expectation equation on a grid (`PolicyEvaluator`, a `Field` of `J^π`; it
belongs beside the Monte Carlo evaluator in the docs as the second method of *evaluate*, and
its `solve()` returns a `GridField` after phase 4.4). A third method later: rollouts from every
node of a grid, the sampled `J^π` as a field, for laws the Bellman sweep cannot host.

**Verify takes the solution**, because it checks the claims:

| check | textbook form | needs from the solution | lands with |
| --- | --- | --- | --- |
| replay gap | the policy driven through the plant reproduces `trajectory` | `problem.sys`, `policy`, `trajectory` | today (tests) |
| claim at the start | `cost_to_go(x_start)` against `evaluation.mean` | `cost_to_go`, `evaluation` | `compare` table column |
| HJB residual | `min_u [g(x,u) + ∇J·f(x,u)] − ρ J(x)` on samples of `X` | `problem`, `cost_to_go` as a `Field` with `gradient` | after phase 4.1 |
| Bellman residual | the discrete twin on the DP grid (`double_integrator_minimum_time` by hand) | the record's grid, `cost_to_go` | after phase 4.4 |
| Pontryagin | costate `λ̇ = −∂H/∂x` backward along the plan, `H` minimized, transversality | `problem`, `trajectory`; the NLP multipliers as costates, on the trajopt record | later |
| optimality gap | `J^π − J*` against a reference solution's field (`policy_evaluation` notebook by hand) | two `cost_to_go` fields, or an evaluated field and a claim | `compare(reference=)` |
| closed-loop certificate | `region_of_attraction(policy @ sys)` | `policy`, `problem.sys` | today |

None of these asks for a new field. Two ask for facts on a *record* (the DP grid, the NLP
multipliers), which is where solver-specific evidence goes (2.4). The checks live in one
module, `planning/optimality.py` (working name), beside `evaluation.py`: evaluation says how
good, optimality says how far from optimal.

**Deterministic and stochastic problems.** Nothing changes on the solution. Its `trajectory`
is nominal (mean start, nominal parameters, no disturbance — the name already says so), its
`cost_to_go` is the expected cost-to-go under the problem's distributions when the method
takes them (the critic by construction, DP on a stochastic problem as the expectation, LQR
certainty-equivalent) and the nominal value otherwise; the *distribution* of the cost lives in
`Evaluation` only (`J` per trial, `value(criterion)`). A worst-case criterion is the
evaluator's reading of the same samples, and a planner that cannot optimize it says so, as the
RL planner does today. The solution never carries draws.

**Compare** reads claims and evaluations side by side and takes an evaluator for the yardstick
rather than `dt` / `n_trials` kwargs, so the evaluator's design stays in one place:
`race.evaluate(MonteCarloEvaluator(problem, dt=DT, n_trials=100, seed=1))`. A
`reference=` name later draws every cost-to-go as a gap to that solution's.

**Two solidity findings from this pass.**

- The RL solution's `policy` is the planner's live controller: `sync_controller` writes new
  weights into it after every `learn`, so an earlier solution's policy changes under it
  (`drone_ppo.ipynb` trains in three rounds). A solution is a snapshot; the RL planner should
  hand out a controller with its own copy of the weights. Same question for DP's
  `LookupTableController` if a planner is re-solved: the table is rebuilt, the old block keeps
  the old table, which is right.
- `success` means different things per method (feasible, converged, goal reached, finite
  weights, Hurwitz). The table prints the record beside it, which is the honest reading; a
  `method` property (`"value iteration"`, `"LQR"`, `"PPO"`) derived from the record gives the
  table its first column without a field.

**Later, not in scope, and the contract already carries them:** an online planner's per-tick
solutions (`solve_trajectory_from`, MPC's `cmd.solution`); a warm start from a previous
solution's `trajectory`; a time-varying `cost_to_go(x, t)` from a finite-horizon sweep; saving
a solution (a frozen record of arrays and a policy block); a `Field` difference for the
optimality gap.

## 3. Rulings needed

- **R1 `problem` on `PlanningSolution`** [core]. Add `problem: PlanningProblem` as the first
  field; six internal constructors change (dp, tabular, rrt, trajopt ×2, rl). Alternative: every
  verb of 2.2 and 2.3 takes `problem` explicitly.
- **R2 `LQRPlanner`** [core, public name]. Name (`LQRPlanner` mirrors `RRTPlanner`; or
  `LinearQuadraticPlanner`), placement, root-prelude export, and the "`QuadraticCost` only" rule.
- **R3 verb names on the solution** [public names]. `plot_cost_to_go` matches the field
  `cost_to_go`; the DP planner keeps `plot_cost2go` for its table. Or rename nothing and call
  the solution verb `plot_cost2go` too.
- **R4 `compare` returns a record or module functions act on a dict** [core].
- **R5 keep rollout opt-in** [core]. `solve(evaluate=True)` stays the way a feedback planner
  fills `trajectory` (the 2026-09-15 ruling); `plot_trajectory` on a solution without one
  raises and says so. Alternative: every planner fills the nominal rollout by default.
- **R6 ruled 2026-09-17: the planner's output is a controller or a trajectory, and the demo
  closes the loop itself** (`cl = ctl @ plant`). No solution verb simulates a closed loop;
  the nominal plan and the closed-loop run are never drawn under one name. Extra evidence
  (nominal rollout, Monte Carlo) may ride along, opt-in.
- **R7 `cost_to_go` is the method's estimate** [core, wording]. Measurements come back from
  evaluate / verify as their own objects and are never written onto the solution. Name stays
  `cost_to_go`; docstring says "the method's own estimate under its own model".
- **R8 evaluators accept a solution as sugar for its policy** [API]. `evaluate(solution)`
  reads `solution.policy` only; the evaluator's problem is the yardstick. `PolicyEvaluator`
  is documented as the second method of *evaluate* and returns a `GridField` after 4.4.

## 4. Order and verification

1. `LQRPlanner` + `RiccatiRecord` (+ `QuadraticField` from phase 4.2 if it is taken with it).
   Tests: `K` and `P` equal `lqr_gain` / `solve_continuous_are`; finite horizon equals
   `lqr_gain_schedule`; `solution.cost_to_go(x)` equals `xᵀPx`; the pendulum loop stabilizes.
2. Solution verbs (2.2) with planners delegating; `test_planning_solution.py` gains the
   asserted delegation; a smoke of each verb on every planner family.
3. `compare` (2.3) and the plant overlay for the showcase phase-plane cell; then the two
   VI-vs-LQR(-vs-PPO) notebooks and `vi_pendulum_lqr.py` read as in §2 for the planner-output
   cells, their closed-loop cells unchanged — each a maintainer-owned edit, shown as a diff first.

Baseline: the seeded DP table, the LQR gain of `cartpole_lqr.py`, and the RL record of
`test_rl_planner.py` are byte-identical before and after each step.
