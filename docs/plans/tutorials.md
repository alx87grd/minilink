# Tutorials in three layers: basics, core, advanced

**Status:** draft for the maintainer's review; nothing written or deleted yet.

## Context

The 15 notebooks in `examples/tutorial/` teach the old diagram-and-simulate story. They never
teach v0.3's backbone. The audit (2026-10-31, every cell read, names checked against
`minilink/`) found:

- **The problem pipeline is never taught.** No notebook uses `problem.compile` → the problem
  evaluator (`evaluate`, `trajectory`, `program`, `pack`/`unpack`). Only `11` reaches
  `PlanningProblem` at all, and it goes through the top-level `evaluate()`.
- **Core objects are untaught:**
  - fields (`QuadraticField`, `MarginField`);
  - `EllipsoidSet` and `FieldSet`;
  - sets built directly;
  - `with_params` and "every number a parameter";
  - costs as composable objects (`+`, `w *`, `TimeCost`);
  - `X0`, `discount_rate`, `disturbances`, `realize`;
  - `SampledSystem` and `SampledDiagramSystem` by name;
  - `MathematicalProgram`, which `08` shows standalone with no link to planning.
- **Stubs:** `01_blocks`, `02_dynamics`, `05_simulation` and `09_planning` are 2–4 code cells.
  `02` promises `LTISystem` but never uses it, and `05` has an empty heading.
- **Messy:** `00` mixes student shortcuts with manual port wiring. `10_graphical` repeats
  itself, half its cells are commented out, and it carries old "9/10" numbering. `07` has a
  garbled cell and a stale `rollout` signature.
- **Gaps:** `closed_loop` (the student `@` path) appears in no tutorial. MPPI, LQR-as-planner
  and `compare` appear in no tutorial. DP, RRT and MPC appear only in `showcase_minilink`.
- **Overlap:** `showcase_minilink` duplicates `00`–`11` almost wholesale.

The maintainer's direction:
- **Layer 1, basics:** student-facing high-level shortcuts.
- **Layer 2, core:** every core object and the evaluation and planning pipeline. It's for the
  maintainer to review the API, and for advanced users.
- **Layer 3, advanced:** MPC, value iteration, planners, discrete time, hybrid loops.

Decided:
- three folders;
- replace layer by layer;
- the showcases keep their paths (README links) and are refreshed later.

Course notebooks (`examples/teaching/courses/`) are untouched: the PDFs link them by path.
Demos and tests stay paused (`docs/plans/minimal-surface.md`).

## The layers

Each notebook is short (8–15 code cells), with one idea per section and textbook naming. It
builds on reused code from the demos listed, and every cell runs top to bottom on `main`.

### The writing rule: most important first

**Every layer is written like a news article**, not as a systematic walkthrough:
- the first notebook of a layer holds what a reader most needs if they read only one;
- each later notebook goes one step deeper;
- inside a notebook, the first section is the core workflow, and later sections refine it.

Nobody is assumed to read everything. A reader who stops after the first notebook, or after
its first section, still has the main workflow.

### `examples/tutorial/basics/` — students, shortcuts only (three notebooks)

| # | Notebook | If you read only this… | Absorbs |
| --- | --- | --- | --- |
| 01 | `01_minilink.ipynb` | **The whole basic workflow, in order:** (1) write your own model, a `DynamicSystem` with `f`, `h` and `params`, in ten lines; (2) use a catalog plant the same way; (3) wire a controller to a plant with `@` (and `closed_loop` with a reference); (4) simulate, plot, animate. It ends on "you can now model, connect and simulate any system". | `00` §1, §3–5; `05`; the clean half of `10` |
| 02 | `02_analyze_and_control.ipynb` | **One step deeper:** linearize and read the poles; Bode with margins and root locus; the PID family, LQR on an operating point, observer and LQG; blocks and sources, `>>`, `feedback`, disturbance and noise ports. | `01`, `03`, `04`; `00` §6–7 (only what a student needs) |
| 03 | `03_plan_and_sample.ipynb` | **Deeper still:** a `PlanningProblem` in five lines, one planner, `compare` against a controller; a digital controller on a clock (`ctl % dt @ plant`); a glimpse of what the core and advanced layers add, with links. | `09`; the student part of `06` |

`10_graphical`'s camera and renderer options join `01_minilink`'s last section, or are
dropped.

### `examples/tutorial/core/` — the foundation and the pipeline (new)

| # | Notebook | Content | Reuse |
| --- | --- | --- | --- |
| 01 | `01_systems.ipynb` | The `System` contract (`f`, `h`, ports, `x0`, `params`); `DynamicSystem`, `StepSystem`, `%` → `SampledSystem`, `discretize`; `DiagramSystem`, `StepDiagramSystem`, `SampledDiagramSystem`; manual wiring | `00` §6–7, `06`; `core/diagram_*.py`, `hybrid/step_*.py` |
| 02 | `02_parameters.ipynb` | Every number a parameter: leaf and nested trees, live diagram views, `with_params` (sets, fields, distributions), cost `params`, the problem's tree | `compile/params_gradient.py` |
| 03 | `03_sets_fields_distributions.ipynb` | `BoxSet`, `BoxInputSet`, `SingletonSet`, `BallSet`, `EllipsoidSet`, `&`; `margin`, `contains`, `sample`, `bounding_box`, `as_cost`; `QuadraticField`, `MarginField`, `as_set`, `as_cost`, `as_terminal_cost`; `Gaussian`, `Uniform`, `Particles` | `analysis/analysis_region_of_attraction.py`, `compile/terminal_field_mpc.py` |
| 04 | `04_costs.ipynb` | `g` and `h`; `QuadraticCost.from_system`, `TimeCost`, sums and scaling, field costs, a custom `CostFunction`; `SampledCost` and `StepCost` | `value_iteration/vi_minimum_time.py`, `mpc/mppi_pendulum.py` |
| 05 | `05_problem.ipynb` | `PlanningProblem` fields (`X`, `U`, `X0`, `Xf`, `x_start`, `tf`, `discount_rate`); `goal_point`, `horizon_kind`, `replace`; stochastic problems (laws on the start and on parameters, `disturbances`, `realize`); `discretize` → `StepPlanningProblem`, the step face | `compile/grid_world_step_problem.py`, `compile/lqg_monte_carlo.py` |
| 06 | `06_compile_and_evaluate.ipynb` | `sys.compile` → evaluator (`f`, `step`, `rollout`, `jacobian`, JAX, `grad`); `problem.compile(law % dt)` → problem evaluator (`evaluate` and the `Evaluation` record, `trajectory`, Monte Carlo with `n_trials`, `compare`) | `07`; `compile/diagram_compiling.py`, `compile/digital_pid_tuning.py` |
| 07 | `07_program.ipynb` | `pe.program(wrt)`, `pack`/`unpack`; `MathematicalProgram` (`J`, `g`, `h`, bounds, `params`); `Optimizer` (`auto`, `l-bfgs`, `slsqp`, `adam`/`gd` with `iterations`, `ipopt`), `as_step_system`; a gradient through a solve | `08`; `compile/pid_autotuning_jax.py`, `optimization/optim_*.py` |
| 08 | `08_planners.ipynb` | One problem, the planner family: problem → planner → `PlanningSolution` (`policy`, `trajectory`, record); the compiled face (`pv.initial`, `solve`, `shift`, `command`) and how the MPC is built on it | `value_iteration/vi_pendulum_lqr.py`, `compile/native_mpc_tuning.py` |

### `examples/tutorial/advanced/` — tools and features

| # | Notebook | Content | Absorbs and reuse |
| --- | --- | --- | --- |
| 01 | `01_discrete_and_hybrid.ipynb` | Tick laws vs laws on a clock; multi-rate sampled loops; the two runners; discrete analysis (`linearize` → `DiscreteLTISystem`, Bode in z) | `06`; `compile/multi_rate_computer.py`, `analysis/analysis_digital_pid_poles.py` |
| 02 | `02_optimal_control.ipynb` | LQR, value iteration, trajectory optimization and RRT on one pendulum problem, compared | `09`; `vi_pendulum_lqr.py`, `trajopt/*`, `rrt/rrt_pendulum_swingup.py` |
| 03 | `03_mpc_and_mppi.ipynb` | `ModelPredictiveController`, warm start, delay, `inputs=`; MPPI; tuning an MPC by gradient | `mpc/mpc_car_minimal.py`, `mpc/mppi_pendulum.py`, `compile/native_mpc_tuning.py` |
| 04 | `04_reinforcement_learning.ipynb` | `11` trimmed: the RL planner on a stochastic problem, its policy as a block | `11` |
| 05 | `05_differentiable.ipynb` | Gradients through loops, `rollout_batch`, co-design, the double loop | `compile/differentiable_double_loop.py`, `compile/cartpole_rollout_gradients.py` |

The core and advanced tables above list topics in order of importance under the same rule. A
notebook's first section is its main idea, for example `core/06` opens on
`problem.compile(law % dt).evaluate()` before the `Evaluation` record's details. Their
counts (eight and five) are a starting point and may merge under the same rule while they're
written.

## Order of work (one layer per pull request, on `plan-minimal-surface`)

1. **Core first.** It's the maintainer's API review and the largest gap. Write `core/01`–`08`;
   delete `07_compile` and `08_optimization` when they're absorbed (into `core/06` and
   `core/07`).
2. **Basics.** Write `basics/01`–`03`, with `01_minilink` first and reviewed on its own;
   delete `00`, `01`, `02`, `03`, `04`, `05`, `09` and `10` as each is absorbed.
3. **Advanced.** Write `advanced/01`–`05`; delete `06` and `11`.
4. **Links and docs:**
   - `examples/README.md` (16 links to tutorial paths);
   - `tests/demo_checks/notebook_overrides.json` (9 entries);
   - `tests/unittest/test_repo_contract.py` and `test_demo_check_runners.py` (tutorial paths);
   - `tests/unittest/teaching_import_allowlist.txt`;
   - DESIGN's one tutorial reference (l.~1416, `11_reinforcement_learning`);
   - the README's "Tutorials" section.

   The showcases' paths and the README and docs links to them stay unchanged.

Each notebook is committed stripped (`pre-commit run nbstripout --files …`).

## API quirks the audit hit, to handle in the notebooks, not fix here

Listed as candidate TODO rows; fixing any of them is an API change for the maintainer:

- `Optimizer` defaults to `method="scipy_slsqp"`, not `"auto"`.
- Adam and gradient descent take their rate through `options={"learning_rate": …}`.
- Costs have no `with_params`; sets, fields and distributions do.
- `compare` is a module function, not an evaluator method.
- A linearized model's matrices are read through `lin.params["A"]`, since `lin.A` is a method.
- `compute_trajectory(show=False)` still prints its banner.
- `plot_bode` is a method on continuous systems only; sampled loops need the function form.
- `MPPIPlanner`, `ModelPredictiveController`, `SampledSystem` and `SampledDiagramSystem` are not
  on the root.

## Verification

- **Each notebook:** `MPLBACKEND=Agg python tests/demo_checks/run_notebook_checks.py --notebook <id>`
  passes on the branch.
- **Each layer's pull request:**
  - the notebook check for the whole layer;
  - `ruff check . && ruff format --check .`;
  - `pytest tests/unittest/test_repo_contract.py tests/unittest/test_teaching_imports.py tests/unittest/test_demo_check_runners.py`, which cover the paths and imports;
  - a link check over `examples/README.md` and the README;
  - `sphinx-build -W` if `docs/` changed.
- **The core layer is reviewed by the maintainer** notebook by notebook before the basics layer
  starts, since its content is the API review.
- **Before the last pull request:**
  - the full notebook sweep (`run_notebook_checks.py --all`);
  - a check that no remaining file links a deleted tutorial path (`git grep "examples/tutorial/[0-9]"`).
