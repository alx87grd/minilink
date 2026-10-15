# The linear analysis band: the rulings (2026-10-15)

**Date:** 2026-10-15
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Before the next step of wave B (S61), the maintainer asked for a second opinion
on the pattern behind the linear analysis, frequency-domain and state-space tools: do the
analysis verbs work on linear systems only, or on any block linearized first, and is the
step response, which has a nonlinear meaning, an exception. The review found the two-level
pattern of `lqr` / `lqr_gain` and `kalman` / `kalman_gain` already governing the analysis
band (arrays in, arrays out in `analysis/linear.py`; any `System` in, linearized once at the
boundary, in the band verbs), with five departures. This file records the rulings, one
entry each, and where each lands. Code lands with S61.

---

## Rulings

- **R1 The band is linear analysis at an operating point; the step response is no
  exception.** Every analysis verb takes any `System`: an `LTISystem` is used as is, with
  the point it carries; anything else is linearized at `(x_bar, u_bar)`, the nominal state
  and inputs by default (RULES 4.3). `step_response`, `plot_step_response` and the
  `plot_step_response` shortcut answer about the linear model, in deviation variables from
  rest, and say so: the figure's title reads **"Linear Step Response"** when the system is
  an `LTISystem` (a `TransferFunction` included) and **"Linearized Step Response"** when it
  was linearized, the channel in the subtitle as today. The shortcut moves from
  `LTISystemFacades` to `DynamicSystemFacades`, beside `plot_bode`; the LTI-only carve-out
  ("a time response read off a nonlinear plant would pass for its simulation", DESIGN §3)
  leaves, since the band verb never had it and the label does the job. The step of the
  nonlinear loop is a simulation, `closed_loop(C, plant, reference=Step(...))` then
  `compute_trajectory`, and the docstrings name that path. Lives in:
  `analysis/time_response.py`, `core/facades.py`, `graphical/control/style.py`, DESIGN §3
  and the facades paragraph of §4, the frequency-response notebook's step cell.
- **R2 `step_info(time, y)` stays on arrays.** It is the band's one measurement verb, like
  `linear.margins(w, G)`, public on the analysis facade: the same call reads the linear step
  and a simulated trajectory, which is what a design-to-spec check on the nonlinear loop
  needs. The S61 sub-item "`step_info` takes a `System`" (scan analysis#4) is struck; a
  dual-arity `step_info` is the dispatch R14 rejected. The simulated `Trajectory` logging
  `y` so the nonlinear figures read an output, not a state, stays C4's (scan analysis#14).
- **R3 `controllability` and `observability` take a system; the matrices go to a named
  twin, as `lqr` / `lqr_gain`.** `controllability(sys, x_bar=None, u_bar=None, t=0.0,
  params=None, *, tol, method, eps)` and `observability(...)` read the model through
  `design_model`: an `LTISystem` as is, a plant linearized about its command port for `B`
  and, for observability, its measured `y` for `C` (`measured=True`, a plant without `y`
  refused), the pair `place` and `luenberger` already test. They return the
  `StructuralResult` (matrix, rank, `n`). On arrays, `controllability_matrix(A, B)` and
  `observability_matrix(A, C)` return the stacked Kalman matrix, the student's
  `np.linalg.matrix_rank` line left to them. Both forms are on the analysis facade; the
  root prelude keeps `controllability` and `observability` as today. The `(A, B=None)`
  dispatch and `matrix_pair` leave. Lives in: `analysis/structural.py`, `analysis/__init__.py`,
  the family-signature test's deviation ledger (two entries fewer), DESIGN §3.
- **R4 A point given with a linear model is refused by every verb.** `design_model`
  already refuses `x_bar` / `u_bar` on an `LTISystem`, which carries its point (R2 of
  2026-10-14); the analysis verbs accepted one silently, with no effect. The check moves to
  the band's one boundary, `operating_point` in `analysis/derivatives.py`, so `jacobian`,
  `linearize`, every channel verb and every design verb give the same error. Lives in:
  `analysis/derivatives.py`, `analysis/linearization.py` (the check leaves `design_model`).
- **R5 The default channel is the command port.** `siso_channel` defaults `wrt` to
  component 0 of `command_port(sys)`, the rule `design_model` and R17 already read (`u`,
  else the one input that is not `d` or `n`), not the first declared input port. The default
  `of` stays the first component of the measured output, the state's first component when
  there is no `y`. Lives in: `analysis/linearization.py`.

## Not changed, by decision

- Result shapes stay as they are: tuples from `pzmap`, `root_locus`, `step_response`,
  `modal_analysis`; the dataclasses `Margins`, `StepInfo`, `StructuralResult`.
- The arrays tier keeps its two spellings: a twin that returns the same quantity keeps the
  verb's name in `analysis.linear` (`linear.step_response`), one that returns a different
  quantity gets its own name on the facade (`lqr_gain`, `controllability_matrix`).
- `sensitivity(plant=, controller=, filter=)` keeps the loop's pieces for v0.2; reading the
  roles of one loop diagram (`sensitivity(T)`, P10's `roles()`) is a later consolidation,
  noted in TODO.
