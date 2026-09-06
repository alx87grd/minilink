# `dev-fable` pre-merge punch list — 2026-09-06

Review of `dev-fable` vs `dev-alex` (44 commits). Branch is good to merge
after the four items below. Full writeup was chat-only; this file is the
action list.

## Fix before merge

1. **Trajopt `success` does not match S09.**
   `TrajectoryOptimizationPlanner.solve` sets
   `success = solver.success or feasible`. A solver that returns
   `success=True` on a constraint-violating `z` still marks the plan
   successful. S09, the test-class docstring, and the evening log say
   success *is* feasibility. The comment in `planner.py` says the same,
   then the `or` does the opposite. DESIGN hedges with “(or the solver
   converged)”.
   - Set `success=feasible` (keep the residual fields).
   - Align DESIGN §6 and the planner comment.
   - Add a case: solver `success=True` + infeasible → plan `success=False`.
   - Files: `minilink/planning/trajectory_optimization/planner.py`,
     `DESIGN.md`, `tests/unittest/test_planning.py`
     (`TestTrajoptSuccessSemantics`).

2. **`Transcription` class docstring is dead.**
   `supports_parametric = False` sits *above* the class string, so
   `Transcription.__doc__` is `None`. Move the flag below the docstring.
   - File: `minilink/planning/trajectory_optimization/transcription.py`

3. **Nightly install omits Ipopt.**
   `.github/workflows/nightly.yml` installs
   `dev,jax,visualization,plotting,rl` and claims a full optional stack.
   `examples/demos/trajopt/trajopt_cartpole_collocation_jax.py` uses
   `optimizer_method="ipopt"` and will fail every nightly run. Add the
   `ipopt` extra (flagship already lists `cyipopt`).

4. **S02 shape checks skip step diagrams.**
   `compile()` and `compile_diagram()` call `validate_equation_shapes`;
   `compile_step_diagram()` does not. A wrong-shape `step()` / `h()`
   inside a `StepDiagramSystem` still fails late.
   - File: `minilink/core/compile/step_compiler.py`
   - Mirror the diagram loop; add a test next to
     `TestEquationShapeValidation`.

## After merge (do not block)

- Stale `disp=True` in `OptimizationResult` (`mathematical_program.py`).
- Rename `test_trajopt_solve_disp_prints_planning_report`.
- Move `TestRootPrelude` above `if __name__ == "__main__"` in
  `test_teaching_surface.py`.
- Strip leftover `configure_jax` from intro / teaching notebooks
  (`07_compile`, `06_hybrid`, showcases, `mpc.ipynb`,
  `articulated_robot_eom`).
- Add `grid_world_exact_dp.ipynb` to the `examples/README.md` teaching
  table (it is already smoked; no override).
- `validate_equation_shapes`: raise on a wrong-shaped `x0` instead of
  substituting zeros.

## Not bugs (intentional; know them)

`disp` / `solve_disp` / `step_disp` removed (no alias). Default `h` is
`y = x` when `p == n`. Auto sim grid is 1001 points. DP cleans
infeasible cells after `solve()`. Wheel excludes `symbolic/`,
`dynamics/engines/`, `c_export`.
