# A comparison shows each answer's nominal trajectory beside its simulation

The design of `compare`'s table and trajectory figures: the trajectory the planner wrote,
scored as written, beside the judge's simulation of the block it handed out. Ruled by the
maintainer, after a prototype and two independent reviews (a correctness review that
reproduced each defect, a design review against CONSTITUTION and RULES), then simplified
in a second pass (R2, R3, R4, the words of the table, the frame and the statistics over
draws).

## 1. The idea

Every answer is read twice, on the one problem the comparison judges:

- **the nominal trajectory**, where the planner wrote one (trajectory optimization, RRT,
  MPPI): the planner's own trajectory scored on the problem's cost, goal and constraints,
  no simulation; how good the answer is on the planner's own model;
- **the simulation**: the block the planner handed out, simulated by the judge and scored
  the same way. A feedback law runs closed loop; a plan's block is its `u(t)` as a source,
  an open-loop replay. On a stochastic problem the simulation covers every draw.

Nominal good and simulation bad reads "the plan is right; run it with feedback"
(`lqr_trajectory` makes the tracking block, which a comparison then judges as its own entry).

## 2. Rulings

- **R1.** `compare` stays the one judge: what is handed out is what is scored. No mode
  keyword, no new public function.
- **R2.** The table shows the nominal trajectory and the simulation side by side, titled
  `nominal trajectory` and `simulation`, the same columns each (J, goal, constraints). No
  length column. An answer without a nominal trajectory reads `-`. The planners' records
  are not printed: `print(comparison[name])` shows one.
- **R3.** No star and no footnote: the table states the numbers, the reader ranks them.
- **R4.** Two figures, one per reading: `plot_nominal_trajectory` draws each answer's
  nominal trajectory, `plot_simulated_trajectory` each answer's simulations. One colour per
  answer, the same in both. `plot_trajectory` is not a `Comparison` method.
- **E.** The judge's simulations are kept in the evaluations (`evaluation.trajectories`,
  every draw) and drawn from there: one simulation a solid line, several each a thin faint
  line. The nominal comparison is the deterministic problem, one simulation per answer; the
  spread is the same problem with its distributions, every simulation drawn.
- **R5.** A nominal trajectory longer than the horizon is scored on the horizon: cut at
  `t0 + T`, the state at that instant interpolated.
- **R6.** `Comparison.problem` is the judged problem, `None` for a comparison by names only;
  the nominal scores, the goal and the constraints read it.
- **R7.** On a figure matplotlib draws itself whose panels share the time axis, one legend
  sits beside the panels, gathered from every panel's labels; a figure drawn on given axes
  is left as it is. `Panel.legend` stays a bool.
- **R8.** A nominal trajectory that begins at or after the deadline `t0 + T` has nothing on
  the horizon: its half reads `starts after the deadline`, and the table still prints.
- **R9.** The table and the figures keep the order the entries were given, an entry given as
  an `Evaluation` included.
- **R10.** The table is a framed panel, the report style of the library: an `=` rule as wide
  as the table (at least the report width, 60), the title, an `=` rule, the table, a closing
  `=` rule. The title names the judged system and its horizon, and on several draws the
  criterion and their number: `Comparison, Pendulum, horizon=5, criterion=expectation, 100
  draws`; by names only, `Comparison of 2 solutions, each scored on its own problem`.
- **R11.** On several draws the simulation half shows `E[J]`, `σ(J)` and `max J` from the
  evaluation (`mean`, `std`, `worst`). A criterion other than the expectation comes first,
  its own column (`CVaR_α(J)`), and a statistic is shown once: the worst-case criterion is
  `max J` itself. σ, not the variance: it reads in the units of J, beside the mean.
- An answer's lines all carry its name: each label once in the legend, and on Plotly a
  click on the key shows or hides every line of that answer. The first row and the rows
  with limits ask for the legend.

## 3. The table

```
=========================================================================================
Comparison, Pendulum, horizon=5, criterion=expectation, 100 draws
=========================================================================================
                │ nominal trajectory          │ simulation
                │     J  goal     constraints │  E[J]    σ(J)  max J  goal     constraints
────────────────┼─────────────────────────────┼──────────────────────────────────────────
LQR             │ -                           │ 47.08   4.455  56.06  100/100  0/100
VI              │ -                           │ 39.12   1.472  42.89  100/100  100/100
TrajOpt         │  38.3  reached  ok          │ 73.65    25.6  166.3  0/100    61/100
RRT             │ 51.82  missed   ok          │ 87.72   21.58  131.2  2/100    66/100
TrajOpt_tracked │ -                           │ 38.09  0.1575  38.59  100/100  4/100
=========================================================================================
```

One run: the title without the criterion and the draws, the simulation half `J` alone, the
goal and the constraints in words.

- The numbers as `:.4g`, right-aligned.
- **goal**: `reached` / `missed`, from `reached` alone. **constraints**: `ok` / `violated`,
  from `failed` (`diverged` when the run is not finite); a run that reaches the goal
  outside the constraints reads `reached` and `violated`. The margin's value mixes units
  (rad, rad/s, N·m) and stays on `comparison.evaluations[name]`.
- Several draws: the statistics of R11; goal and constraints are counts (`183/200`: the
  runs that reach, the runs within the constraints).
- Built from the numbers, formatted last: one `cells` helper, one `line` helper.

## 4. The figures

- One row per signal component; one colour per answer, the same in both figures, fixed by
  the order of the entries.
- `plot_simulated_trajectory`: per answer, the judge's simulations from its evaluation (the
  plant's part of each): one solid, several thin and faint (width 0.6, alpha 0.12). An
  `Evaluation` entry is drawn when it keeps its trajectories; by names only, the controller
  run once from its problem's start (`controller_run_of`). Title `simulation`.
- `plot_nominal_trajectory`: per answer with one, the nominal trajectory, solid and drawn
  whole (past the horizon when it is longer). Answers without one are left out; none at all
  is refused. Title `nominal trajectory`.
- Decorations, on both, follow the signal of each row, not its position: on an `x` row the
  goal's extent at the deadline, a dark grey segment at `t0 + T` (`Xf.bounding_box()`; none
  for a set without one); on a `u` row the input limits, dotted grey lines
  (`U.bounding_box()`); on every row the horizon `t0 + T`, a dotted vertical.
- The legend's keys read at full opacity, a renderer rule (matplotlib's keys drawn opaque;
  Plotly's key of a faint trace its first sample at full opacity, in the trace's legend
  group).

## 5. The implementation

- `compare` keeps the problem (R6) and evaluates every scored entry with
  `trajectories=True`: its evaluation keeps all `n_trials` runs, the one home of the
  judge's simulations (E); memory grows with `n_trials` (the pendulum demo, 4 answers × 100
  draws, 5 s at `dt = 0.01`: 17.2 MB as the loop runs `evaluate` returns, their recorded
  signals included; 6.4 MB for the plant's `t, x, u` alone).
- The nominal trajectory is scored in `compare`'s object by `evaluate_trajectory` on
  `Trajectory(p.t, p.x, p.u)`, cut at the horizon (R5).
- The two figures share one drawing helper; each passes its lines.
- Docs: DESIGN §6's comparison sentence; `Comparison`'s docstrings; the tutorials that
  describe the table or call the figure (`advanced/02_optimal_control`,
  `core/06_compile_and_evaluate`).
- Tests: the table's halves (one run, several draws, no nominal trajectory, a diverged run,
  an `Evaluation` entry), the cut at the horizon, the decorations by signal
  (`signals=("u",)`, `t0 != 0`), the legend beside the panels and untouched with `ax=`, the
  judged problem read whatever the entries' order, the nominal figure's refusal when no
  answer has one.
