# How each consumer reads the planning problem's fields (audit, 2026-10-23)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Before ruling on the one evaluation record and the terminal set
([end-state-api.md](../plans/end-state-api.md) D18, D31), an inventory of how every consumer
reads `PlanningProblem` / `StochasticPlanningProblem` (`planning/problems.py`), produced by an
independent read of the code on `dev`. Line numbers are those of 2026-10-23.

## Defaults set in `__post_init__`

- `X` defaults to the **unbounded** set, not the state box (the docstring says so); `U` to the
  input-port box; `X0` to a singleton at `x_start`; `Xf` to a singleton at `x_goal` when
  `x_goal` is given; `x_start` falls back to `sys.x0`; membership `x_start ∈ X0`, `x_goal ∈ Xf`
  is checked. `tf` is None, a positive float or `inf`, the last two words both meaning
  "infinite" in `horizon_kind()`. `infeasible_cost` unset means `None`. The stochastic class
  derives `x_start` and `X0` from `x0_distribution`; `nominal()` drops the draws; stochastic
  problems are refused by trajectory optimization, DP, LQR and RRT.

## One table per consumer (abridged; the full detail is the field-by-field read)

| Field | Trajectory optimization | DP / value iteration | LQR | RL environment and planner | Monte Carlo evaluator, `score_trajectory` | RRT | MPC | Gym bridge |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| `x_start` | shooting start; parametric binding | nominal rollout only | nominal rollout only | through `sample_x0` | the start | tree root | overwritten by the measured state each tick | `sample_x0` |
| `x_goal` | initial guess, terminal-error report | plot anchor only | ignored (operating point is `cost.xbar`) | ignored | ignored | goal bias, goal ball, best-effort target | — | ignored |
| `cost.g`, `cost.h` | Bolza objective: trapezoid + `h(x_N)` | `g dt` table; `h` seeds the iteration, infinite horizon included | `Q`, `R`, `S` only | reward `−g dt` (no `params.cost`); `−h` at a reached finite horizon | discounted trapezoid of `g`; `h` undiscounted at a reached finite horizon | not in the search | TO | `−g dt`; `−h` |
| `discount_rate` | **ignored by the objective** (reporting only) | `alpha = e^{−ρ dt}` | ignored | γ (0.99 with a warning when ρ = 0) | on `g` only | ignored | — | bound only |
| `X` | hard constraint at knots | exit priced INF, replacing `g` and the continuation; grid from `X`'s box else `sys.state.box` | ignored | exit = termination + price (discounted by γ^k); leaving the training zone = truncation | exit = cut + price, undiscounted | hard on edge states | TO | exit = terminated + price |
| `U` | hard | grid inputs, inadmissible priced INF | ignored | **ignored** (actions clipped to the port box) | ignored | hard on edge inputs | TO | ignored |
| `X0` | equality (singleton) or margin | ignored | ignored | ignored | ignored | ignored | overwritten | ignored |
| `Xf` | **hard terminal constraint** (equality for a singleton) | ignored | ignored | ignored | **ignored** | goal region (a singleton swapped for a `goal_tolerance` ball) | inherited terminal constraint on every horizon | ignored |
| `tf` | required finite | stamps `h`'s time; `solve()` runs to convergence regardless | finite ↔ infinite design | finite: episode ends with `h`; else 10 s episode, truncated, no `h` | same 10 s rule | ignored | required finite (prediction horizon) | same 10 s rule |
| `infeasible_cost` | ignored | default **1e6** | ignored | derived bound `g_max dt (1 − γ^N)/(1 − γ)` (deep RL); **1e6** (tabular) | the environment's price; **+inf** on a direct `score_trajectory` | ignored | ignored | derived bound over `sys.state.box` |
| `params.cost`, `params.system` | read | read | `.system` | **ignored** | **ignored** | ignored | TO | `.sets` only |
| `criterion` | — | — | — | refused unless `"expectation"` | **never read** (`value(criterion)` takes its own argument) | — | — | — |
| `metadata`, `params.scene` | read by nobody | | | | | | | |

## Where the consumers disagree

- **`Xf`** is a hard terminal equality in trajectory optimization (so `x_goal` alone makes every
  open-loop solve end exactly at the goal), a goal ball in RRT, and nothing anywhere else.
- **`X`** is hard (TO, RRT), an exit priced INF that erases `g` (DP), an exit that ends the
  episode with a discounted price (RL), an exit that cuts the run with an undiscounted price
  (the evaluator), or ignored (LQR). Because `X` defaults to unbounded, DP and RL silently fall
  back to `sys.state.box` for the grid and the training zone.
- **`U`** is enforced by TO, DP and RRT and ignored by LQR, RL and gym, which clip to the port box.
- **`tf`**: required finite by TO and MPC; a design switch for LQR; a time stamp for DP, whose
  `solve()` ignores it; a 10 s episode default for RL and the evaluator.
- **`discount_rate`** shapes DP, RL and the evaluator (on `g` only), and is ignored by the TO
  objective and LQR; `h` is undiscounted in the evaluator and discounted by the RL return.
- **`infeasible_cost` unset** means 1e6 (DP, tabular RL), a derived bound (deep RL, the
  evaluator, gym) or +inf (`score_trajectory`); it is charged at the exit successor (DP), at
  `x_next` (RL, the evaluator) or at the first infeasible sample (gym, `score_trajectory`).
- **`x_goal` and `cost.xbar`** are never checked against each other: LQR aims at `cost.xbar`,
  TO at `Xf`, RRT at `x_goal`, DP and RL wherever the cost is lowest.

What this means for the plan is in [end-state-api.md](../plans/end-state-api.md) D31.
