# Core objects, phase 1: vision, constitution, rules

Status: applied 2026-09-15 (docs only). Kept as the record of what changed and why; the wording
now lives in the governance documents. DESIGN bullets for code not yet written land with phases 2 and 4.
The review this phase comes from, with the findings (F), comments (C) and rulings (Q) cited
below: docs/reviews/2026-09-15-foundations-review.md.

## 1.1 Why first

Three code steps depend on a rule: the style pass needs 5.1 to cover `margin` (today it names
only `f`, `h`, `tf`, so the `np.ndarray` hints in `sets.py` do not even break a rule); the
frozen-dataclass question needs 5.21 to say which objects are values; the `bounding_box` hook
needs 4.3 to say that a tool asks the set. And the 2026-09-12 audit's lesson holds: prose rules
drift within days unless the check lands with the rule, so Phase 2 carries one ratchet test per
new sentence.

## 1.2 Amendments the maintainer agreed to (2026-09-15), exact wording

**CONSTITUTION §2, new subsection after "Memoryless components ...":**

> ### The Mathematical Objects
>
> Around the model, the tools speak the textbook's own nouns, and a tool that takes or returns
> one of these quantities takes or returns the object, never its arrays:
>
> - `Trajectory` — the sampled evolution `(t, x, u)` every simulation and plan returns.
> - `Set` — an allowable region, `z ∈ Z(t)`, written as `margin(z, t) ≥ 0`: the state box, a
>   goal ball, free space, a level set.
> - `Distribution` — a probability law over a vector, `sample(key)` and `mean`. Sets are
>   support, distributions are probability; never one for the other.
> - `Field` — a scalar function on the model's `(x, u, t)`, the signature of `f` and `g`: a
>   Lyapunov function `V(x)`, a cost-to-go `J(x, t)`, a Q-function `Q(x, u)`, a barrier `h(x)`,
>   a clearance `d(x)`. Its sublevel and superlevel sets are `Set`s over `x`, or `InputSet`s
>   over `u` at a given `x`; its shaping is a `CostFunction`. *(agreed 2026-09-15; time and
>   action arguments added the same day)*
> - `CostFunction` — the running cost `g(x, u, t)` and the terminal cost `h(x, t)` of `J`.
> - `PlanningProblem` — a system, its sets, a cost and a horizon; the stochastic sibling adds
>   distributions over starts, parameters and disturbances.
> - `PlanningSolution` — a policy, itself a `System`, and the evidence for it.
> - `MathematicalProgram` — the one NLP shape every transcription lowers to.
>
> **Lower late.** A tool carries these objects to the solver boundary and converts to arrays
> there, once.

**RULES 2.5, appended sentence:**

> Before adding a record, name the object it already is: a `(lower, upper)` pair is a `BoxSet`,
> a reset rule is a `Distribution`, a sublevel set is a `Set`, a sampled run is a `Trajectory`.

**RULES 4.3, appended sentences:**

> A tool that needs a box asks the set (`X.bounding_box()`); it does not `isinstance` the set.
> The one `isinstance` on a set is the lowering at the solver boundary: a singleton to an
> equality residual, a box to decision bounds.

**RULES 5.1, first sentence widened:**

> Do not put type hints inside $f, h, tf$, port computations, or any other native-array
> equation path (`Set.margin`, `CostFunction.g` / `h`, `Shape.sdf`, `Distribution.sample`).

## 1.3 Further comments on the stack (new this round; each needs a yes)

**C1 — CONSTITUTION §1.4, a boundary that keeps the two nouns small.** Add a bullet:

> - It is **not** a probability or set-algebra library. Sets and distributions are the few
>   textbook objects the tools need — a box, a ball, a level set; a Gaussian, a uniform law,
>   particles — each with a two-method surface, not a general library.

Why: `distributions.py`'s own docstring already says "a duck type, not a probability library";
the constitution is where that sentence stops the first `scipy.stats`-shaped proposal.

**C2 — CONSTITUTION invariant 2, purity covers every equation path.** Today: "Equation paths
($f, h$) have no internal mutable state". Proposed: "Equation paths — $f$, $h$, and the equation
methods of sets, costs, shapes and distributions (`margin`, `g`, `h`, `sdf`, `sample`) — have no
internal mutable state, no cached side effects, and no memory between calls." Why: a
`CallableSet` around a stateful closure or a `Sampler` that advances its own generator breaks
`vmap` exactly as a stateful `f` would; the invariant should name them.

**C3 — CONSTITUTION §4.2, tools return the nouns.** "take a `System` (or a `PlanningProblem`)
and return domain objects — trajectories, plans, certificates — or matrices and figures."
Today it says "return trajectories, matrices, or figures", written before `PlanningSolution`
and `LyapunovCertificate` existed.

**C4 — RULES 2.3, the dual parameter pattern is one rule for every parametric object.** Append:
"The same rule holds for costs, sets and shapes: `params is None` resolves to the object's own
defaults; any dictionary replaces them at that level." This generalises cost-params decision 1
(2026-09-13) and is what DESIGN §4's "deferred" rows will satisfy.

**C5 — RULES 5.4, say what a symbol collision is allowed to be.** The rule says "a symbol keeps
one meaning across the codebase". The literature does not: `h` is the output map on a `System`
and the terminal cost on a `CostFunction`; `g` the running cost, the gravity vector `g(q)`, and
the NLP inequality; `C` the Coriolis and the output matrix; `B` the actuator map and the input
matrix. The 2026-09-05 review flagged it and suggested renaming `CostFunction.h`; every
notebook cost class defines `def h(self, x, t)`, so a rename is off the table during the term.
Proposed wording: "A symbol keeps one meaning within an object, and one meaning across the
codebase wherever the literature allows. The textbook overloads a few letters across objects
(`h`: output map on a `System`, terminal cost on a `CostFunction`; `g`: running cost, gravity
vector, NLP inequality; `B`, `C`: mechanical vs state-space). Those collisions are inherited,
listed in DESIGN §4, and never added to." → **question Q3 below**.

**C6 — RULES 5.21, which objects are values.** Today: "Use dataclasses for transparent domain
records (`Trajectory`, a certificate, a plan)". The cost-params plan chose plain classes for
costs (their `params` are mutated in place by notebooks); `sets.py` mixes both styles.
Proposed: "Frozen dataclasses for value objects — a `Trajectory`, a set, a shape, a certificate,
a solver record. Plain classes with `params` for parametric equation objects — systems, costs.
Never dataclasses as input/output wrappers around arrays (rule 2.5)." This settles the mixed
style in `sets.py` by rule rather than by taste.

**C7 — RULES §6, a new 6.12 "Cheap rules are tests".** "When a rule can be checked by an AST
walk or a grep, the check lands with the rule (`test_teaching_imports.py`,
`test_repo_contract.py`, the set-probe check in `test_planning.py`, the flat-demo check)." The
2026-09-12 audit said this in prose; making it a rule is what keeps 4.3 and 6.10 true.

**C8 — AGENTS.md "Ask first" list names the math objects.** "core architecture and the API of
the main tools (`System` family, the core mathematical objects — `Trajectory`, sets,
distributions, costs, `PlanningProblem`, `PlanningSolution` — diagrams, compile, `Simulator`,
planners, `Optimizer`, controllers)". Today `core/sets.py` and `core/costs.py` are core by
implication only.

**C9 — DESIGN §4 "Trajectory, sets, costs, geometry".** Add a distributions bullet (home
`core/distributions.py`, the draw convention, `support`); the `bounding_box` hook on the sets
bullet; the notation-collision list (C5); the noise-convention open item (F9). DESIGN §3
package map: `core/` row adds "distributions". DESIGN §6 stochastic paragraph: the new path.

**C10 — ROADMAP.** §6 review queue: "Open (v0.2, before `estimation/`): the disturbance
convention — per-step covariance or spectral density." §1 north-star claim 2 may name the
nouns ("a task is a `PlanningProblem`: system, sets, cost, horizon; or its stochastic sibling")
— pitch-facing, maintainer's call. §3 TRL: no change until Phase 2 lands.

**C11 — What I would not change.** The precedence ladder; the length of RULES (the 2026-09-12
audit left trimming §1–§2 to the maintainer, and nothing here needs it); AGENTS.md's workflow
(its behaviour-preserving-refactor recipe is exactly the one Phases 2–3 follow); no fifth
document.

**C12 — `Field` in the constitution's object list (F12): agreed 2026-09-15**, folded into the
1.2 wording (renamed from `StateField` the same day: with `u` and `t` in play the noun is the
field on `(x, u, t)`; `StateField` is not a student-facing name, two DESIGN §6 mentions only). It closes the triangle set ↔ field ↔ cost that the spatial band already draws
(`as_constraint`, `as_cost`), and it is the object the Lyapunov, CBF, DP and approximate-DP
chapters all stand on. Phase 4 is its promotion.

**C15 — One signature, stated once (maintainer, 2026-09-15).** The idea is spread over six
places and none says it whole: CONSTITUTION invariant 2 (purity, for `f` and `h` only),
invariant 5 and §6 "backend-native math" (one path for NumPy and JAX), RULES 1.4 / 5.2 / 5.17
(`xp`, native arrays, for `f` and `h`), RULES 2.3 (the dual `params` pattern, for systems),
DESIGN §4 "native-array equation rule" (its list: `f`, `h`, ports, sets, costs, transcriptions,
the NLP callables) and DESIGN §4 parameters ("deferred: call-time `params` on `Shape`, `Set`,
`CostFunction`"). Missing is the sentence that every object's mathematics is one stateless
method on the model's arguments, traceable in the arguments and in `params` alike. Proposed,
in CONSTITUTION §2 right after the objects list:

> **One signature.** An object's mathematics is a method on the model's arguments
> `(x, u, t; p)`, or the subset it reads: `f`, `h`, `tf` on a system; `g`, `h` on a cost;
> `margin` on a set; `value` on a field; `sdf` on a shape; `sample(key)` on a distribution. It
> is stateless, takes and returns native arrays on NumPy or JAX through one equation path,
> and traces under JAX in every argument, `params` included — so `jit`, `grad`, `vmap` and a
> parameter family reach any object the way they reach `f`. An object that does not yet read
> `params` is a gap to close, not a design.

And the rules that follow from it: RULES 5.2 "in `f` or `h`" → "in every equation path"; RULES
5.17 "Keep `f` / `h` on native arrays" → "Keep every equation path (the list of 5.1) on native
arrays"; RULES 2.3 as C4; DESIGN §4's native-array list gains fields and `Distribution.sample`,
and its "deferred" `params` rows become the tracked gap (TODO Later already carries
"parametric `Shape` / `Set` / `Cost` overrides"; Phase 5.4 here). ROADMAP §1 claim 3 may read
"the same *objects* are differentiable and compiled" instead of "the same `f`".

**C13 — What is deliberately *not* an object** (so nobody proposes it later): an operating point
(`x_bar, u_bar` arrays; a record would be a DTO), a pose or transform (4×4 arrays, `tf`), a
metric (a callable), a transition batch (arrays), a criterion (a string until CVaR earns an
object). RULES 2.5 already covers these; listing them in the review doc saves the argument.

**C14 — Demos and teaching notebooks: less is better (maintainer, 2026-09-15; wording for a
yes).** Today RULES 6.1 asks for open-and-run scripts with a one-line docstring and the story
in comments; nothing says what a demo may *contain*. The census (`examples/demos/`, 73 scripts;
`examples/tutorial/` + `examples/teaching/`, 33 notebooks):

| | Demos | Notebooks |
| --- | --- | --- |
| Clean (no helper function, no custom plot code) | 53 / 73 | 16 / 33 |
| With top-level helper functions | 11 (40 functions) | 17 (72 functions) |
| With hand-written matplotlib code | 15 (85 lines) | 9 (20 cells) |
| Cells mixing an API call with matplotlib code | — | 6 cells in 3 notebooks |
| `print(...)` statements | 121 (10 scripts with five or more) | 293 |

Worst: `rrt_holonomic_obstacles.py` (8 helpers, 252 lines), `cartpole_rollout_gradients.py`
(9 helpers), `optim_plot.py`, `neural_controller_jax.py`, `car_circuit_rl.py`,
`trajopt_holonomic_corridor.py` (218 lines); `articulated_robot_eom.ipynb` (26 helpers),
`from_value_iteration_to_ppo.ipynb` (4 mixed cells), `11_reinforcement_learning.ipynb`. Most
helpers are library gaps in disguise: the hand-written rollout and loss in the three
`compile/` demos are the differentiable closed-loop cost of the ROADMAP review queue; the
`make_*_extender` and `path_cost` helpers are `RRTOptions` and the search record; the track
`features` helpers are the composite policy block of the RL plan. Proposed wording:

> **6.1 A demo is the API, and nothing else.** *Simplicity is the ultimate sophistication*
> holds for a demo or a teaching notebook as it does for the core: the shortest sequence of
> library verbs that tells the story — build, compose, solve, evaluate, plot, animate — with
> constants at the top, a one-line title docstring and the story in short inline comments
> (5.22–5.23). Less is better. Reporting goes through the objects themselves: their `print`
> (`__str__`) and their native plots (`plot_trajectory`, `plot_solution`, `plot_control_law`,
> `plot_cost2go`, `plot_learning_curve`, `plot_tree`, `scene.plot`, ...). A report the library
> cannot give is first a library gap — a missing `__str__` or `plot_*`, in the agent's plotting
> lane — and only then a demo cell.
>
> **6.10 Flat demos.** No functions or classes in a demo or a teaching notebook except the
> model or cost it is about (its `f`, `h`, `g`) and a step the notebook's text teaches by hand
> (a hand-written RK4, a policy-gradient update). No wiring, tuning or plotting helpers, no
> configuration dictionaries feeding them, no `main()`, no `try / except ImportError` around
> plots (6.2). A loop over variants is fine; a factory is not.
>
> **6.11 Side work is quarantined.** A custom plot or a side analysis that earns its place is
> its own cell in a notebook, or its own block at the end of a script under one comment line.
> It never interleaves with the API lines, and the demo still reads with that cell removed.

`examples/README.md` "Demo style" then points at 6.1, 6.10, 6.11. The sweep of the offenders
and the native plots they need is Phase 6 (student-facing: per file, maintainer reviews).

## 1.4 Rulings (maintainer, 2026-09-15)

**Q1 — Where do bounds live? → derived `.box` property.** Ports keep `lower_bound` /
`upper_bound` arrays as the student's line; `VectorSignal` gains a read-only `box` property
(`sys.state.box`, `port.box`) returning a `BoxSet` derived on each read (RULES 5.6), so a tool
reads one set and never two arrays. `BoxSet.from_system_state(sys)` stays (used by
`rrt_car_parking.py`) and delegates to it. DESIGN §4 ports bullet records the property.

**Q2 — `InputSet` stays** as the textbook's `u ∈ U(x, t)`: actuator envelopes depend on state
(a DC motor's torque–speed curve is GRO501 material), and the cost is one short ABC.
`BoxInputSet` gains `bounding_box()` like the rest; nothing renames.

**Q3 — The `h` / `g` collision → amend 5.4** as in C5. One meaning within an object; across
objects the literature's own overloads (`h`, `g`, `B`, `C`) are inherited, listed in DESIGN §4
as a closed list, never added to. Nothing renames; notebooks untouched.

**Q4 — open: the Hamiltonian and the costate.** The maximum principle appears in
`cartpole_lqr.ipynb` (costate `λ = S x`, the Hamiltonian matrix behind `lqr_gain_schedule`),
and nowhere else: the toolbox is direct methods (collocation, shooting), DP and RL. If the
course goes further — indirect shooting, a bang-bang law read off the switching function — the
Hamiltonian `H(x, u, λ, t) = g + λᵀ f` is one method on `PlanningProblem` (`jax.grad` gives the
costate equation `λ̇ = −∂H/∂x`) and the costate is a `Trajectory` signal; if not, nothing is
missing. Affects Phase 4 only; answer on review.

**Q5 — open: sets and distributions over parameter dictionaries.** Both nouns are over vectors;
parameters are pytrees. `params_distribution={name: Distribution}` draws each leaf
independently, so a correlated posterior over `(m, l)` (identification), a joint grid of
parameter pairs (the Buckingham-π family sweeps of `rollout_batch`), or a robust set `θ ∈ Θ`
cannot be written. Not a new noun: a flatten rule (`ravel_pytree`) that lets `Gaussian`,
`Particles`, `BoxSet` take and return a dict. Lands with its first consumer (identification or
the robust class); named here so it is designed once.

## 1.5 Phase 1 done when

CONSTITUTION, RULES, AGENTS, DESIGN, ROADMAP carry the agreed wording; the review doc records
the amendments and the rulings on C1–C10 and Q1–Q3; `docs/plans/sets-and-distributions.md`
exists with Phases 2–4 adjusted to the rulings; TODO has the rows; `ruff` untouched (no Python).
`test_repo_contract.py` (link check) passes on the new documents.
