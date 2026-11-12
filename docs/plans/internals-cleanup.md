# Internals: what could be cleaned behind the kept surface

A second look at the library's internals, assuming the public surface of
[consolidation-recap.md](consolidation-recap.md) stays. The question: which plumbing,
duplicates and options exist for a feature, a caller or a demo that is leaving, or for none?
Nothing here is decided or done; every row is a candidate.

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
- **Line counts are estimates** from the readers, to the nearest ten or so.

## 2. What the numbers say first

| Fact | Measure |
| --- | --- |
| Code named nowhere at all | 5 definitions, 120 lines |
| Code named only by tests | 18 definitions, 300 lines |
| Departing features (experimental, path-tracking laws, boat, three-body) | referenced outside their own files only by export-table lines: they lift out with no surgery |
| Plotly | the exception: it reaches into 17 files |
| Validators (`require_*`) | 294 functions, 2 300 lines, 4 % of the code; 47 never run by the kept surface, about ten exact duplicates |
| Docstrings | 12 700 lines against 47 200 of code |
| `if __name__ == "__main__"` blocks inside the library | 38 files, 540 lines |
| Export tables | 267 public names in 389 entries; 120 names are listed in two tables |

So the library is not carrying dead weight in the plain sense. What it carries is of four
kinds: **twins** (the same logic written twice), **reporting extras** (animations, histories,
live plots built for old demos), **options nobody passes**, and **leftovers** of removed
designs. The sections follow that order of gain.

## 3. Twins: the same logic twice

The largest gain, and the kind that most hurts a reader: two places to read, two places to
fix, and they drift.

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| Trajectory optimization: `direct_collocation.py`, `shooting.py`, `multiple_shooting.py` | Each transcription is written once for NumPy and once for JAX. They have drifted: the NumPy shooting path adds bound rows that are always infinite on an unconstrained problem, the JAX path skips them. One program written on `(z, params)`, looped on NumPy and vectorized on JAX, replaces both. | 300 | behaviour (NumPy programs gain `params`) | M |
| Compile: the step diagram's own plan and evaluator (`plan.py`, `evaluators/discrete.py`, `compiler.py`) | A tick diagram is the sampled diagram with one rate. Its plan, evaluator and trace check are line-for-line copies of the sampled ones. | 200 | behaviour; feasibility not proven (nested boxes are flattened by one and kept whole by the other) | L |
| Value iteration on JAX: three chunked table builders | Three near-identical `jit(vmap)` builders, and a second copy of the out-of-bound penalty of `dp.py`. | 80 | none | H |
| Value-iteration plots against the generic planning plots | The heatmap and the 3-D surface are drawn twice (`dynamic_programming/plotting.py`, `planning/plotting.py`); the state-label helper exists three times. | 100 | none | H |
| The "problem leaves" tree | The parameter tree of a problem is assembled in six places; two of them differ by one argument. | 30 | none | H |
| "The problem's box, else the plant's" | Coded five times (problem, plot, grid, RRT, step problem). | 15 | none | H |
| The closed-loop score of a step problem (`planner.step_evaluation`) | Repeats `score_run` line for line. The two discount the terminal cost differently (γᴺ against e^(−ρT)), so the merge needs care. | 50 | behaviour | M |
| The Gymnasium environment's episode rules | `ProblemEnv.step` repeats the failure and termination rules of `RolloutEnvironment.step`. | 20 | none | M |
| Evaluators | `input_sequence_of` on the continuous and discrete evaluators differ by one argument; the dynamic and static evaluators repeat their constructor; `step_body` and `step_path` repeat the substep loop. | 50 | none (the last one: trace shape, unchecked) | M |
| Simulation facade | `compute_trajectory` and `compute_rollout` of a stepped system repeat the same setup and the same roles block. | 25 | none | H |
| Small exact twins | `costs.unique_term_id` = `assembly.unique_id` (verified identical); `frequency.caller_stacklevel` = `core.options`; two spring coils; two Rodrigues rotations; `meshes.translation_matrix` = `kinematics.translation`; `require_mechanical_plant`, `require_manipulator`, `require_axis`, `require_integer_key` each twice; `plot_bode` redoing `bode` and `margins`. | 150 | none | H |
| The two mechanical base classes | Once the boat leaves, `GeneralizedMechanicalSystem` (231 lines) has one subclass, the 3-D plane; it is `MechanicalSystem` with one extra matrix. | 200 | public base class | M |

**Not twins, checked:** the paired analysis names (`step_response`, `margins`, `root_locus`,
`poles` in two files each) are a kernel on matrices and a verb on systems, each with its own
callers. The three `plot_control_law` are three layers. The two `shapes.py` do different
jobs. The NumPy and JAX program evaluators are legitimately different.

## 4. Reporting extras built for the old demos

Animations, histories and live plots whose only callers are scripts of `examples/demos/` or
the projects. The algorithms do not need them.

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| RRT: `animate_search`, `animate_convergence`, the snapshot history | Called by two old demos only. `plot_tree` and the live plot are used and stay. | 300 | two public methods | M |
| Value iteration: `animate_cost_to_go`, `animate_control_law`, the sweep history | No caller. Also two plot "forms" nobody uses and a batch-slice path never fed. | 150 | public methods | M |
| Trajectory optimization: `callback=`, `live_plot=`, `record_history=`, the iteration record, `live_plot.py`; the evaluator's `live_plot` | Called by three old demos only. `callback=` could stay as a plain pass-through. | 230 | public keywords | M |
| Soft cost fields in a `Scene` (`workspace_fields`, `cost_density`, `GaussianField` drawing) | One old demo. The kept surface uses a scene for obstacles and clearance only. | 200 | a `Scene` argument | M |
| Two cost-field figures (`plot_cost_field_3d`, `plot_cost_field_exports`) | Called by the projects and old demos only; they move with the projects. | 190 | none | H |
| MPC: the sampled-futures plot, four overlay keywords | One old demo, the projects. | 100 | none | M |
| Racecar skin: `palette`, `equipment`, `scene`, `heading_arrow`, `wheel_radius`; a debug look | A skin is only ever called as `skin(plant)`; the keywords are never passed. | 55 | public keywords | H |

A judgment that is yours: an animation of the search tree or of the value sweeps is a strong
teaching picture. If a course will show one, that one stays and gets a demo; the point of the
row is that today nothing kept shows them.

## 5. Plotly

If the Plotly plot backends go (the animation renderer can stay), this collapses:

| What | Lines |
| --- | --- |
| The two backend files (signals, control figures) | 570 |
| The Plotly phase-plane renderer and its guards | 120 |
| Renderer dispatch in time signals, control figures, the live plot; the handle base class with one subclass left | 70 |
| Hover-text building in the analysis plots; colour and style tables | 50 |
| The `renderer=` keyword on nine plotting verbs and `plot_renderer=` on the game | 40 |

About 850 lines, and nine public keywords become unnecessary. **One blocker:** the proposed
off-road game passes `plot_renderer="plotly"`; and two course notebooks offer `"plotly"` in a
comment. Risk: behaviour for those callers. Confidence: M, your call on Colab.

## 6. Options and paths nobody uses

| Where | What | Lines | Risk | Conf. |
| --- | --- | --- | --- | --- |
| Sampled systems: `solver="scipy"` and the host clock runner (`simulation/discrete/clock_runner.py`) | Reached only by that keyword on a sampled system; one test passes it. This answers the open point of the recap. | 140 | a keyword value refused | M |
| The evaluator's adaptive SciPy path (`ProblemEvaluator`, `solver="scipy…"`) | Tests only. | 70 | a keyword value | M |
| `compile(flatten=True)` | Tests only; on a step diagram it raises "not supported yet". | 50 | public keyword | H |
| The `"direct"` transcription backend | Passed nowhere. | 15 | behaviour | M |
| Dubins steering for RRT (`search/dubins.py`, `DubinsSteering`) | Tests only. The recap kept it "for cars"; no car demo uses it. Keep with a demo, or it goes to the scratchpad. | 220 | public class | L |
| `Tree(nearest_method="kd_tree")` | Tests only. | 35 | none | M |
| The reward-rebuild branch of the learning planner (`reward="stage"`) | A keyword no caller passes; the environment's own option stays. | 15 | none | M |
| Custom gravity hook on the arm impedance laws (`gravity=`, signature probing) | Tests only; two of its guards can never fire. | 35 | public keyword | M |
| Law keywords: label and unit overrides on `Impedance`, `y_filt0`, integrator clamps, `autowire(strict=)`, `animate(native=)` | Passed by nobody. | 50 | public keywords | M |
| A pluggable diagram-exporter layer with one backend (`graphical/diagrams/export.py`) | Graphviz is the only exporter; the layer exists for a test. | 90 | exported names | H |
| Mesh tools: the Collada loader (two functions), `split_by_radius`, `refine` | No caller, tests included. | 200 | none | H |
| JAX setup: `configure_jax`, two backend tuples | Called by the projects and tests, always with the default. | 35 | a public name | M |
| Save and load of value tables and grids | Tests only. Useful for a long solve; keep if a course uses it. | 25 | public methods | L |

## 7. Leftovers of removed designs

Small, safe, and each one a line a reader stops on.

| Where | What | Lines |
| --- | --- | --- |
| `core/closed_loop.py` | `resolve_standard_feedback`, its record and helper: the docstring says "shared by the closed loop and the sampled loop"; neither calls it. | 65 |
| `core/roles.py` | A branch for a system with a `.computer` attribute; no class has one (verified). | 15 |
| `core/inspect.py` | The LaTeX path goes out and comes back to one class; its generic branch would recurse forever if reached. | 18 |
| `optimization/optimizer.py` | The method table is an identity copy of the presets; a method is validated three times. | 25 |
| `planning/evaluator.py` | `numbers_key` builds a cache key, and is only tested for `None`. | 10 |
| Records | Fields written and never read: four on the trajectory-optimization record, `frozen_at`, `last_program`, `last_optimizer`, `last_traj`, `PlotResult.payload`, `ControlFigure.extra`. | 60 |
| Fallbacks that cannot fire | About twenty `getattr(x, name, default)` on attributes every object has; six spellings of "is this a diagram"; `if problem.X is not None` (always true). | 60 |
| Forwarders | Functions that only pass their arguments on; variadic helpers every caller gives one argument; a dataclass around one dict. | 130 |
| `SampledSystem` | Three different fixes for one problem (defaults written through the view). | 8 |
| Gymnasium interface | A termination hook nothing overrides, so its branch never runs. | 15 |
| Strictly dead helpers | `require_complete_part`, `signal_stack_figsize`, `camera_eye_position`, `bounding_radius`, `stiff_tires`, others. | 80 |

About 490 lines, almost all at no risk.

## 8. Where the readers and the plan disagree

A reader does not know the rulings; these are its claims set against them.

| Reader's proposal | Why it does not apply as stated |
| --- | --- |
| Delete the region-of-attraction module (no caller) | Your ruling: reduce to core. The reader's split is the useful part: the core is about 350 lines (`region_of_attraction`, the certificate, the level search); the periphery about 410 (three plots, the simulation counter-check and its report); a `method="sos"` option that always raises. |
| Delete the linear approximators (no caller) | GRO860 week 8. Stays; a tutorial line would exercise it. |
| Delete value iteration on JAX (no user caller, 620 lines with its plumbing) | The speed benchmark runs it (`benchmarks/suites/dp_speed.py`), so it is exercised; and it is the path to large grids. Keep, merge its three builders, give it a demo. The stale `# backend="loop"` comment in a course notebook names an option that does not exist. |
| Drop the loop form of value iteration (`method="loop"`) | The textbook reference the docstrings point to. Stays. |
| Delete the Gaussian policies (`control/stochastic.py`, 259 lines, no caller outside tests) | Open: the learners no longer use them. This closes the low-confidence row of the recap toward "delete", unless you want them as the notes' stochastic-policy object. |

## 9. The three largest files, by what is inside

| File | Lines | The core | The rest |
| --- | --- | --- | --- |
| `planning/evaluator.py` | 1 409 | about 400: construct, evaluate, program, realize, run, score | validation 150; sampled-loop support 70; batch-axis tree utilities 70 (they belong with params); program packing 60; failure-cost recompute 60; held plans 40; the adaptive path and the live plot (above) |
| `planning/problems.py` | 1 248 | about 500, of which 120 are the class docstring | coercers and validators 350; the stochastic-draw helpers 150; print text 60 |
| `planning/dynamic_programming/dp.py` | 1 143 | about 350: the backup, the two cost tables, the solution | policy evaluation 100 (used in teaching); the loop reference 60; JAX dispatch and plot forwarders; the table and record classes |

For the first two, the split into a subpackage (the open question in QUESTIONS) would put the
core in a file a student can read in one sitting.

## 10. House-rule debts seen in passing

- **Leading underscores** (RULES 5.8): 31 names in the MeshCat renderer, 29 in the Plotly
  renderer, 9 in the other two, 6 in the Gymnasium interface, the `_jit_*` attributes of the JAX
  program evaluator.
- **One primitive with two names:** `Line = CustomLine`, both used at about twenty sites each.
- **The racecar skin reads an STL file at import time.**
- **`core/inspect.py` reads planning-layer attributes** (`scene`, `track`) from core.

## 11. Sum and order

| Kind | Lines, about | Mostly at risk of |
| --- | --- | --- |
| Twins | 1 200 | behaviour, for the two large ones |
| Reporting extras | 1 200 | public methods and keywords |
| Plotly | 850 | two callers |
| Unused options and paths | 1 000 | public keywords |
| Leftovers | 490 | nothing |
| **Total** | **4 700** | |

This is on top of the moves of the recap, and smaller than them: the internals are in better
shape than the surface around them.

A suggested order:

1. **The leftovers** (§7) and the small exact twins: no public name moves, the seeded
   baselines stay byte-identical, one commit per file.
2. **The reporting extras** (§4), once you say which pictures a course will show.
3. **Plotly** (§5), once you answer for Colab.
4. **The unused options** (§6), with your pass on the public surface, since each is a keyword.
5. **The two large twins** (the transcriptions, the step plan): each is a refactor of its own
   with a baseline first, after the API review of that module.
