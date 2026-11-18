# Internals: what could be cleaned behind the kept surface

A second look at the library's internals, assuming the public surface of
[consolidation-recap.md](consolidation-recap.md) stays. The question: which plumbing,
duplicates and options exist for a feature, a caller or a demo that is leaving, or for none?

**Two rulings of the maintainer frame every row:**

- **Nothing is deleted before the maintainer's pass** over the demos and the features. Every
  row is a candidate, with its size and what its removal would cost; none is a decision.
- **The value-iteration animations stay** (`animate_cost_to_go`, `animate_control_law`): they
  are used in class. They are in no table below. The sweep history they replay
  (`record_history=True`, `record.history`) is not separately removable: the animations read
  it.

## 1. How it was measured

- **Every definition counted.** All 3 837 functions, methods and classes of the kept library
  were listed with where their name appears (library, kept examples, old demos and projects,
  tests) and whether the kept surface runs them (the coverage run of minimal-surface.md §4.8).
- **Three independent readers**, one per zone (core, compile and simulation; planning,
  optimization and learning; graphics, analysis and control), each told to verify a claim by
  search before reporting it. Their 66 findings were then checked here: about fifteen against
  the code, each of them holding with one correction; the rest are as reported.
- **"No caller"** means none in the library, the tutorials, the proposed demos or the course
  notebooks. A test does not count as a caller.
- **Line counts are estimates** from the readers, to the nearest ten or so, unless a file is
  named with its size (`wc -l`; "not blank" where said).
- **Re-checked 2026-11-18,** after the plotting contract and the cut of the region of
  attraction: every name in the tables exists, the callers are re-counted by search over
  `minilink/`, `examples/`, `tests/` and `benchmarks/`, and the rows whose subject is gone
  are removed. The readers' line estimates and the "written N times" counts of §3 and §7
  were not re-measured.

## 2. What the numbers say first

| Fact | Measure |
| --- | --- |
| Code named nowhere at all | 5 definitions, 120 lines |
| Code named only by tests | 18 definitions, 300 lines |
| Departing features (experimental, path-tracking laws, boat, three-body) | referenced outside their own files only by export-table lines: they lift out with no surgery |
| Plotly | the exception: 22 library files name it |
| Validators (`require_*`) | 299 functions, 2 340 lines, 3 % of the lines; 47 never run by the kept surface and about ten exact duplicates (the count of 2026-11-12) |
| Docstrings | 13 000 lines of the library's 81 600 (65 900 not blank) |
| `if __name__ == "__main__"` blocks inside the library | 37 files, 490 lines |
| Export tables | 267 public names in 389 entries; 120 names are listed in two tables (the count of 2026-11-12) |

So the library is not carrying dead weight in the plain sense. What it carries is of five
kinds: **twins** (the same logic written twice), **reporting extras** (animations, histories,
live plots built for old demos), **the large drawing code** (a second plot library, the 3-D
renderer, the skins), **options nobody passes**, and **leftovers** of removed designs.

## 3. Twins: the same logic twice

The largest gain, and the kind that most hurts a reader: two places to read, two places to
fix, and they drift.

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| Trajectory optimization: `direct_collocation.py`, `shooting.py`, `multiple_shooting.py` | Each transcription is written once for NumPy and once for JAX. They have drifted: the NumPy shooting path adds bound rows that are always infinite on an unconstrained problem, the JAX path skips them. One program written on `(z, params)`, looped on NumPy and vectorized on JAX, replaces both. | 300 | behaviour (NumPy programs gain `params`) | M |
| Compile: the step diagram's own plan and evaluator (`plan.py`, `evaluators/discrete.py`, `compiler.py`) | A tick diagram is the sampled diagram with one rate. Its plan, evaluator and trace check are line-for-line copies of the sampled ones. | 200 | behaviour; feasibility not proven (nested boxes are flattened by one and kept whole by the other) | L |
| Value iteration on JAX: three chunked table builders | Three near-identical `jit(vmap)` builders (`build_jax_sa_chunks` and `build_jax_node_chunks` in `dp_jax.py`, `build_xnext_jax_chunks` in `discretizer.py`), and a second copy of the out-of-bound penalty of `dp.py`. | 80 | none | H |
| Value-iteration plots against the generic planning plots | The heatmap panel and the 3-D surface panel are described twice (`field_figure`, `value_surface_figure` in `dynamic_programming/plotting.py`; `cost_to_go_panel`, `cost_to_go_surface` in `planning/plotting.py`). The drawing itself and the state-label helper are one since the plotting contract. | 40 | none | H |
| The "problem leaves" tree | The parameter tree of a problem is assembled in six places; two of them differ by one argument. | 30 | none | H |
| "The problem's box, else the plant's" | Coded five times (problem, plot, grid, RRT, step problem). | 15 | none | H |
| The closed-loop score of a step problem (`planner.step_evaluation`) | Repeats `score_run` line for line. The two discount the terminal cost differently (γᴺ against e^(−ρT)), so the merge needs care. | 50 | behaviour | M |
| The Gymnasium environment's episode rules | `ProblemEnv.step` repeats the failure and termination rules of `RolloutEnvironment.step`. | 20 | none | M |
| Evaluators | `input_sequence_of` on the continuous and discrete evaluators differ by one argument; the dynamic and static evaluators repeat their constructor; `step_body` and `step_path` repeat the substep loop. | 50 | none (the last one: trace shape, unchecked) | M |
| Simulation facade | `compute_trajectory` and `compute_rollout` of a stepped system repeat the same setup and the same roles block. | 25 | none | H |
| Small exact twins | `costs.unique_term_id` = `assembly.unique_id`; `frequency.caller_stacklevel` = `core.options`; two spring coils; two Rodrigues rotations; `meshes.translation_matrix` = `kinematics.translation`; `require_mechanical_plant` (`robotic.py`, `modelbased.py`), `require_manipulator` (`robotic.py`, `arms.py`), `require_axis` (`port_map.py`, `phase_plane.py`), `require_integer_key` (`evaluator.py`, `problems.py`) each twice; `plot_bode` redoing `bode` and `margins`. | 150 | none | H |
| The two mechanical base classes | `GeneralizedMechanicalSystem` (231 lines) has two subclasses, the boat and the 3-D plane; if the boat leaves it has one, and it is `MechanicalSystem` with one extra matrix. | 200 | public base class | M |

**Not twins, checked:** the paired analysis names (`step_response`, `margins`, `root_locus`,
`poles` in two files each) are a kernel on matrices and a verb on systems, each with its own
callers. The three `plot_control_law` are three layers. The two `shapes.py` do different
jobs. The NumPy and JAX program evaluators are legitimately different.

## 4. Reporting extras built for the old demos

Animations, histories and live plots whose only callers are scripts of `examples/demos/` or
the projects. The algorithms do not need them. Whether a demo that shows one stays is the
maintainer's pass; a row here goes only with its last caller.

| Where | What | Called by | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- | --- |
| RRT: `animate_search`, `animate_convergence`, the snapshot history | The tree growing, and RRT*'s path improving. `plot_tree` and the live plot (`LiveSearchPlot`) are used by the kept surface and stay. | one old demo each (`rrt_pendulum_swingup.py`, `rrt_star_live.py`) | 300 | two public methods | M |
| Trajectory optimization: `live_plot=`, `record_history=`, the iteration record, `live_plot.py`; the evaluator's `live_plot` | `callback=` could stay as a plain pass-through to the optimizer; no example passes it to this planner. | `live_plot=`: one old demo (`trajopt_cartpole_collocation_jax.py`); the evaluator's: one old demo (`pid_autotuning_native.py`); `record_history=` on this planner: none | 230 | public keywords | M |
| Soft cost fields in a `Scene` (`workspace_fields`, `cost_density`, `GaussianField` drawing) | The kept surface uses a scene for obstacles and clearance only. | one old demo (`rrt_holonomic_obstacles.py`) | 200 | a `Scene` argument | M |
| The cost-field figure, `plot_cost_field` and its surface (`is_3d=True`) | The raster of a spatial cost and its 3-D surface. | three project scripts and one project notebook, one old demo (`mpc_car_circuit.py`); the surface by two project scripts, the notebook and that demo | 190 | none: it moves with the projects | H |
| The MPC run pictures (`mpc/viz.py`, 491 lines): the horizon and trail overlays (`mpc_animation_overlays`), the sampled-futures plot (`plot_sampled_futures`) | Pictures of a closed-loop MPC run; the planner and the loop do not need them. No tutorial and no proposed demo draws them: the three proposed racecar demos animate the lap without the horizon. | the overlays: five old demos, three project scripts and one project notebook; the plot: one old demo (`mppi_pendulum.py`) | 490 | public functions; the horizon over the car is the MPC's teaching picture | M |
| Racecar skin: `palette`, `equipment`, `scene`, `heading_arrow`, `wheel_radius`; a debug look (`frame_axes_skin`) | A skin is only ever attached as `car.skin = racecar_skin_3d` and called as `skin(plant)`; the keywords are never passed. | none | 55 | public keywords | H |

A judgment that is yours: an animation of the search tree is a strong teaching picture, as
the value sweeps are. If a course will show one, it stays and gets a demo; the point of the
row is that today nothing kept shows it.

## 5. The large drawing code

The plotting contract (one figure description, two renderers that draw it, one record) left
the library the same size: net +207 lines on about 66 000 not blank. It removed the second
copy of each plot's drawing, not the drawing code that is large on its own. Five pieces
remain large and were not touched. Each is a candidate; the column says what a user loses.

| Where | Lines (not blank) | Who uses it | What a removal would cost the user | Risk | Conf. |
| --- | --- | --- | --- | --- | --- |
| The MeshCat animation renderer (`graphical/animation/renderers/meshcat_renderer.py`) | 955 | the showcase and the plotting tutorial, five proposed demos, fourteen old demos | every 3-D animation in the browser: the UR5, the racecar and 3-D car skins, the saved `.html` clips. Matplotlib's 3-D view is the fallback, without meshes at a usable speed. A cleaning pass (22 underscore methods, the flipbook of changing geometry) is the realistic candidate, not a removal. | behaviour, for every 3-D demo | M |
| The Plotly animation renderer (`renderers/plotly_renderer.py`) | 643 | two tutorials (`basics/01`, `core/09`); no demo | `animate(renderer="plotly")`: an interactive clip in a notebook with no viewer to start. Matplotlib's inline player and MeshCat's inline page cover Jupyter and Colab. | a renderer name refused | M |
| The Plotly figure renderer (`graphical/figure_renderers/plotly.py`) | 405 | one tutorial (`core/09`), two old demos | `renderer="plotly"` on every plot: zoom, pan and hover. The description stays; only Matplotlib would draw it, and `renderer=` would have one value. | a keyword value on every `plot_*` | M |
| The Plotly live signal plot (`graphical/signals/plotly_backend.py`: a polling HTTP server behind a browser tab) | 195 | the proposed off-road game and two old games, by `plot_renderer="plotly"` | live signals beside a pygame or MeshCat game without Matplotlib's window, the pairing that is not reliable today (the two-window bug). Fixing that bug removes the reason for this file. | one keyword of `game()` | M |
| The port map and the block control-law plot (`graphical/port_map.py`: `plot_input_output_map`, `plot_control_law` on a block) | 569 | `plot_control_law`: three tutorials (on a controller, and behind `solution.plot_control_law` and `Comparison.plot_control_law`) and eleven old demos; `plot_input_output_map`: no example, one line of the systems notebook | the picture of a law `u(x)` on a plane, for any block, a neural policy included: a GRO860 and GMC714 picture. The candidate is its inside (the sweep axes and the error-space options: 360 of its lines are helpers under the two plots), read with the notebooks open; `plot_input_output_map` alone has no caller. | public methods | M |
| The racecar skin and the 3-D car skins (`graphical/catalog/racecar_skin.py`, `car_skins_3d.py`) | 1 658 | the racecar skin: three proposed demos and seven old ones; the two 3-D car skins: the two presets | the look of the flagship platform and of the two 3-D presets; `car_skin_3d` (in `skins.py`) is the plain fallback. The racecar skin holds a scene (floor tiles, walls, painted lines), a 2-D top view, a lidar, a camera and a shadow: the candidate is those extras, each a few dozen lines. | looks only | M |

If Plotly goes whole (its three pieces, 1 240 lines not blank), these go with it: the style
table (`common/plotly_style.py`), the Plotly branch of the show step and of the two records,
the renderer dispatch of the live signal plot and its handle base class with one subclass
left, and the `"plotly"` value of `renderer=` and `plot_renderer=`. **What blocks it:** the
proposed off-road game passes `plot_renderer="plotly"`, and two tutorials show
`renderer="plotly"`. Your call on Colab.

## 6. Options and paths nobody uses

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| Sampled systems: `solver="scipy"` and the host clock runner (`simulation/discrete/clock_runner.py`) | Reached only by that keyword on a sampled system; one test passes it. | 140 | a keyword value refused | M |
| The evaluator's adaptive SciPy path (`ProblemEvaluator`, `solver="scipy…"`) | Tests only. | 70 | a keyword value | M |
| `compile(flatten=True)` | Tests only; on a step diagram it raises "not supported yet". | 50 | public keyword | H |
| The `"direct"` transcription backend | Passed nowhere. | 15 | behaviour | M |
| Dubins steering for RRT (`search/dubins.py`, `DubinsSteering`) | Tests only: no demo, tutorial or project. Keep with a car demo, or it goes to the scratchpad. | 220 | public class | L |
| `Tree(nearest_method="kd_tree")` | Tests only. | 35 | none | M |
| The reward-rebuild branch of the learning planner (`reward="stage"`) | A keyword no caller passes; the environment's own option stays. | 15 | none | M |
| Custom gravity hook on the arm impedance laws (`gravity=`, signature probing) | Tests only; two of its guards can never fire. | 35 | public keyword | M |
| Law keywords: label and unit overrides on `Impedance`, `y_filt0`, integrator clamps, `autowire(strict=)` | Passed by no example, except `autowire(strict=True)` in two old demos. | 50 | public keywords | M |
| A pluggable diagram-exporter layer with one backend (`graphical/diagrams/export.py`) | Graphviz is the only exporter; the layer exists for a test. | 90 | exported names | H |
| Mesh tools: the Collada loader (`load_collada`, `collada_mesh`), `split_by_radius`, `refine` | No caller, tests included. | 200 | none | H |
| JAX setup: `configure_jax`, two backend tuples | No library caller; called by the projects, the benchmarks and the tests, always with the default. | 35 | a public name | M |
| Save and load of a grid's successor tables (`Discretizer.save`, `load`) | No caller, tests included. Useful for a long solve; keep if a course uses it. | 15 | public methods | L |

## 7. Leftovers of removed designs

Small, safe, and each one a line a reader stops on.

| Where | What | Lines |
| --- | --- | --- |
| `core/closed_loop.py` | `resolve_standard_feedback`, its record and helper: the docstring says "shared by the closed loop and the sampled loop"; neither calls it. | 65 |
| `core/roles.py` | A branch for a system with a `.computer` attribute; no class has one (verified). | 15 |
| `core/inspect.py` | The LaTeX path goes out and comes back to one class; its generic branch would recurse forever if reached. | 18 |
| `optimization/optimizer.py` | The method table is an identity copy of the presets; a method is validated three times. | 25 |
| `planning/evaluator.py` | `numbers_key` builds a cache key, and is only tested for `None`. | 10 |
| Records | Fields written and read by no library code: `frozen_at` on the value-iteration record (read by nothing); `last_program` and `last_optimizer` on the trajectory-optimization planner (read by tests and one benchmark scenario); `last_traj` on the two simulators (read by tests). | 60 |
| Fallbacks that cannot fire | About twenty `getattr(x, name, default)` on attributes every object has; six spellings of "is this a diagram"; `if problem.X is not None` (always true). | 60 |
| Forwarders | Functions that only pass their arguments on; variadic helpers every caller gives one argument; a dataclass around one dict. | 130 |
| `SampledSystem` | Three different fixes for one problem (defaults written through the view). | 8 |
| Gymnasium interface | A termination hook nothing overrides, so its branch never runs. | 15 |
| Helpers no library code calls | `require_complete_part`, `split_by_radius` (no caller at all); `signal_stack_figsize`, `camera_eye_position`, `bounding_radius`, `stiff_tires`, `resolve_standard_feedback` (tests only). | 80 |

About 490 lines, almost all at no risk.

## 8. Where the readers and the plan disagree

A reader does not know the rulings; these are its claims set against them.

| Reader's proposal | Why it does not apply as stated |
| --- | --- |
| Delete the linear approximators (no caller) | GRO860 week 8. Stays; a tutorial line would exercise it. |
| Delete value iteration on JAX (no user caller, 620 lines with its plumbing) | The tests run both backends and the speed benchmark runs it (`benchmarks/suites/dp_speed.py`), so it is exercised; three old demos call it; and it is the path to large grids. Keep, merge its three builders, give it a kept demo. |
| Drop the loop form of value iteration (`method="loop"`) | The textbook reference the docstrings point to. Stays. |
| Delete the Gaussian policies (`control/stochastic.py`, 259 lines, no caller outside tests) | Open, and yours: no library code and no example calls them (three test files do). Deleted, unless you want them as the notes' stochastic-policy object. |

## 9. Three of the largest files, by what is inside

| File | Lines | The core | The rest |
| --- | --- | --- | --- |
| `planning/evaluator.py` | 1 415 | about 400: construct, evaluate, program, realize, run, score | validation 150; sampled-loop support 70; batch-axis tree utilities 70 (they belong with params); program packing 60; failure-cost recompute 60; held plans 40; the adaptive path and the live plot (above) |
| `planning/problems.py` | 1 253 | about 500, of which 120 are the class docstring | coercers and validators 350; the stochastic-draw helpers 150; print text 60 |
| `planning/dynamic_programming/dp.py` | 1 086 | about 350: the backup, the two cost tables, the solution | policy evaluation 100 (used in teaching); the loop reference 60; JAX dispatch and plot forwarders; the table and record classes |

For the first two, the split into a subpackage (the open question in QUESTIONS) would put the
core in a file a student can read in one sitting.

## 10. House-rule debts seen in passing

- **Leading underscores** (RULES 5.8), counted as methods: 22 in the MeshCat renderer, 26 in
  the Plotly animation renderer, 9 in the Matplotlib and pygame ones, 5 in the Gymnasium
  interface; the `_jit_*` attributes of the JAX program evaluator.
- **One primitive with two names:** `Line = CustomLine`, both used at about twenty sites each.
- **The racecar skin reads an STL file at import time** (`GEOMETRY = racecar_geometry()`).
- **`core/inspect.py` reads planning-layer attributes** (`scene`, `track`) from core.

## 11. Sum and order

| Kind | Lines, about | Mostly at risk of |
| --- | --- | --- |
| Twins | 1 150 | behaviour, for the two large ones |
| Reporting extras | 1 500 | public methods and keywords |
| The large drawing code | 3 800 not blank in all; 1 240 if Plotly goes whole, the rest by cleaning | a second renderer; looks |
| Unused options and paths | 1 000 | public keywords |
| Leftovers | 490 | nothing |

The first, second, fourth and fifth rows sum to about 4 100 lines; the drawing code is of
another kind, since most of it is kept and cleaned, not removed. This is on top of the moves
of the recap, and smaller than them: the internals are in better shape than the surface
around them.

A suggested order, each step after the maintainer's pass has said what stays:

1. **The leftovers** (§7) and the small exact twins: no public name moves, the seeded
   baselines stay byte-identical, one commit per file.
2. **The reporting extras** (§4), once you say which pictures a course will show.
3. **Plotly** (§5), once you answer for Colab; the other drawing code when its file is opened.
4. **The unused options** (§6), with your pass on the public surface, since each is a keyword.
5. **The two large twins** (the transcriptions, the step plan): each is a refactor of its own
   with a baseline first, after the API review of that module.
