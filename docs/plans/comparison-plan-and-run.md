# A comparison shows each answer's plan beside its run

The design of `compare`'s table and trajectory figure: what the planner wrote, scored as
written, beside what the judge simulated of the block it handed out. Ruled by the
maintainer, after a prototype and two independent reviews (a correctness review that
reproduced each defect, a design review against CONSTITUTION and RULES).

## 1. The idea

Every answer is read twice, on the one problem the comparison judges:

- **the plan**, where the planner wrote one (trajectory optimization, RRT, MPPI): the
  planner's own trajectory scored on the problem's cost, goal and limits, no simulation;
  how good the answer is on the planner's own model;
- **the run**: the block the planner handed out, simulated by the judge and scored the same
  way. A feedback law runs closed loop; a plan's block is its `u(t)` as a source, an
  open-loop replay. On a stochastic problem the run covers every draw.

Plan good and run bad reads "the plan is right; run it with feedback"
(`lqr_trajectory` makes the tracking block, which a comparison then judges as its own entry).

## 2. Rulings

- **R1.** `compare` stays the one closed-loop judge: what is handed out is what is scored.
  No mode keyword, no new public function.
- **R2.** The table shows the plan and the run side by side, the same columns each (J,
  goal, limits, length). The `success` column goes; the planners' records move under the
  table, one line each.
- **R3.** A star marks the best run: the lowest criterion value (`Evaluation.objective`)
  among the answers whose every run reaches the goal within the limits
  (`success_rate == 1`); one run is the case of one draw. No answer qualifies: no star,
  no footnote.
- **R4.** The figure draws each answer's run solid and its plan dashed, one colour per answer.
- **R5.** A plan longer than the horizon is scored on the horizon: cut at `t0 + T`, the
  state at that instant interpolated. The length cell gives the plan's own length.
- **R6.** `Comparison.problem` is the judged problem, `None` for a comparison by names only;
  the plan scores, the goal and the limits read it.
- **R7.** On a figure matplotlib draws itself whose panels share the time axis, one legend
  sits beside the panels, gathered from every panel's labels; a figure drawn on given axes
  is left as it is. `Panel.legend` stays a bool.

## 3. The table

```
         │ plan, as the planner wrote it      │ run, simulated by the judge
         │     J  goal     limits  length     │      J    goal     limits    length
─────────┼────────────────────────────────────┼──────────────────────────────────────
LQR      │ no plan (a feedback law)           │  47.08    reached  violated  5 s
VI       │ no plan (a feedback law)           │  38.76 *  reached  ok        5 s
TrajOpt  │  38.3  reached  ok      5 s        │   42.2    missed   ok        5 s
RRT      │ 51.82  missed   ok      5.4 s      │  51.87    missed   ok        5 s

* best run: the lowest J that reaches the goal within the limits

records
  LQR      K(0) = [[20.117,  9.511]] over 5 s, ...; U ignored
  ...
```

- J as `:.4g`; the star in its own slot, so the numbers stay aligned.
- **goal**: `reached` / `missed`, from `reached` alone. **limits**: `ok` / `violated`,
  from `failed` (`diverged` when the run is not finite); a run that reaches the goal
  outside the limits reads `reached` and `violated`, and earns no star. The margin's value mixes units
  (rad, rad/s, N·m) and stays on `comparison.evaluations[name]`.
- No plan: the plan half reads `no plan (a feedback law)`; an `Evaluation` entry reads
  `no plan` and a blank length when it keeps no run.
- Several draws: the J header names the criterion (`E[J]`, the CVaR, `max J`), goal and
  limits are counts (`183/200`: the runs that reach, the runs within the limits), length
  blank.
- Built from the numbers, formatted last: one `cells` helper, one `line` helper.

## 4. The figure

- One row per signal component, as today; per answer, the judge's **nominal run** solid
  (`ProblemEvaluator.trajectory()`: the mean start, nominal parameters, the judge's `dt`
  and backend; for a deterministic problem the scored run) and the plan dashed.
- Decorations follow the signal of each row, not its position: on an `x` row the goal's
  extent at the deadline, a dark grey segment at `t0 + T` (`Xf.bounding_box()`; none for a
  set without one); on a `u` row the input limits, dotted grey lines
  (`U.bounding_box()`); on every row the horizon `t0 + T`, a dotted vertical.
- The legend: the answers' names and `limits`; the title `plans (dashed) and runs (solid)`.
- The plan is drawn whole: a plan past the horizon shows past the vertical.
- A comparison by names only (no judge) runs each law once, as today, through one internal
  helper (`controller_run_of`, split from `results.drawn_trajectory_of`).

## 5. The implementation

- `compare` keeps the problem (R6) and `Comparison.runs`: name → the judge's nominal run,
  the plant's part as a plain `Trajectory`, from the evaluators `score_entries` builds,
  which are then dropped: without draws the scored trial itself (kept by the evaluation,
  bit-identical to `trajectory()`, no second compile), with draws one `trajectory()` run (`None` or no entry for an
  `Evaluation` entry and by names only, where the figure runs `controller_run_of`). The
  maintainer ruled `runs`, plain data, over keeping the evaluators on the object. No draws
  are kept for the figure.
- The plan is scored in `compare`'s object by `evaluate_trajectory` on
  `Trajectory(p.t, p.x, p.u)`, cut at the horizon (R5).
- `score_run` (`planning/evaluation.py`): a run that is not finite never reaches the goal,
  `reached = (margin >= 0) & finite`; a diverged run scored on zeros could "reach" a goal
  holding the origin.
- The matplotlib renderer: the legend rule of R7, with `tight_layout(rect=...)`; DESIGN §7's
  legend sentence gains it.
- Docs: DESIGN §6's comparison sentence; `Comparison`'s docstrings; the tutorials that
  describe the old table (`advanced/02_optimal_control`, `core/06_compile_and_evaluate`).
- Tests: the table's halves and star (one run, several draws, no plan, a diverged run, an
  `Evaluation` entry), the cut at the horizon, the decorations by signal (`signals=("u",)`,
  `t0 != 0`), the legend beside the panels and untouched with `ax=`, the judged problem
  read whatever the entries' order.
