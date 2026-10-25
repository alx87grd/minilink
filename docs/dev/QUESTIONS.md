# Open questions for the maintainer

One entry per question that waits for you, ordered by the release it blocks. Answer on the
`Answer:` line or in a session; the agent then records the ruling (the plan doc it shapes and
a dated record in docs/reviews/), applies it, and deletes the entry. Rulings already taken:
[docs/reviews/](../reviews/). Known defects: [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## Blocks v0.3 (the robotics-course release, December 2026)

### Should compile-time diagram flatten default to on after the parity and speed study?
- **Blocks:** nothing until benchmarks and tests are stable; default stays `False` until you decide.
- **Today:** `compile_diagram(..., flatten=False)` inlines nested boxes with ``__`` ids
  (`inner__plant`); parity tests and `benchmarks/scenarios/nested_compile_flatten.py`
  compare nested compile, `flatten=True`, and hand-flat wiring.
- **Options:** keep default off; turn default on if flatten wins on compile and/or `f()`;
  drop the flag and always flatten at compile.
- **Recommendation:** wait for one informational benchmark run on your reference machine,
  then choose.
- **Detail:** [DESIGN.md](DESIGN.md) (compile-time flatten paragraph)
- **Answer:**

### Do you review the overnight run's 16 remaining decisions, or accept them as a block?
- **Blocks:** your merge of the v0.3 branch into `dev` (R79).
- **Today:** the run of 2026-10-24 decided 36 points without you, each with its reason, the
  alternative and the commit; its §3 measures the intended number changes. Settled since: OD-9
  confirmed, OD-10 superseded, sixteen accepted from chat (R103). Open: OD-7, OD-8, OD-12,
  OD-14, OD-17, OD-18, OD-20, OD-21, OD-23, OD-24, OD-25, OD-26, OD-27, OD-29, OD-30, OD-35;
  OD-32 and OD-33 are the two questions below.
- **Options:** accept the rest as a block; review each; accept all but the ones you name.
- **Recommendation:** the log's "please look first at": the number changes (value iteration's
  default price is now the cost bound, so the GRO860 cost-to-go colour scale tops there;
  scores moved from a held law to the continuous loop, up to 8% for a stiff PD), the input-box
  sweep (OD-18), the planners' reported trajectory (OD-27); then the rest as a block.
- **Detail:** [the run's log, §1–§3](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### Do the three demos that teach a hand-written gradient loop move onto the `Optimizer`?
- **Blocks:** the rest of the optimizer work (S74's rest, OD-32).
- **Today:** `cartpole_rollout_gradients`, `neural_controller_jax`, `params_gradient`
  (`examples/demos/compile/`) descend by hand; the first tunes an open-loop schedule, which the
  evaluator cannot tune until a source's samples are parameters. R90 kept the hand loop in two
  places on purpose.
- **Options:** convert all three; convert two, the schedule demo later; keep all three.
- **Recommendation:** none yet (converting changes what they teach).
- **Detail:** [the run's log, OD-32](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### Does LQR warn when a problem declares `U`, now that the LQR lessons declare it?
- **Blocks:** the rest of the problem-fields work (V1.2's rest, OD-33).
- **Today:** R21 says LQR ignores `U` and warns once when it is declared. The run wrote
  `U=plant.input_box` into about 90 constructions to keep their numbers (OD-18), the GRO860 LQR
  notebooks among them, so the warning would fire where nothing changed. Not implemented.
- **Options:** drop the line from the LQR lessons, then warn; keep it and warn; keep it and
  never warn (amends R21).
- **Recommendation:** none yet.
- **Detail:** [the run's log, OD-18, OD-33](../reviews/2026-10-24-v03-run-decisions.md#2-decisions)
- **Answer:**

### One mechanical family: does `DynamicBicycle` move onto the generalized mechanical base now?
- **Blocks:** one mechanical family (S32); the car family clean-up (K8).
- **Today:** the row moves its base from `DynamicSystem` to the generalized child, but its named
  ports and its subclasses' extra states do not fit `[q; v]`.
- **Options:** move it now; rename `M` to `H` only and move it when K8 reworks the cars.
- **Recommendation:** rename only; the base waits for K8.
- **Detail:** [TODO, S32](../plans/TODO.md#4-v03--a-cleaner-core-and-the-differentiable-path-then-gmc714)
- **Answer:**

### Should the failure price follow a run's cost overrides?
- **Blocks:** programs over cost weights.
- **Today:** the price keeps the problem's own weights; with no exact bound it costs 4096 calls
  of `g` at every `evaluate` and program iteration. A cache keyed on the cost's parameters
  would remove that.
- **Options:** price from the overridden weights, with the cache; keep the problem's price.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §6 item 6](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Do three demos change style?
- **Blocks:** your demo pass (K4) and the closing example (S77).
- **Today:** `pid_autotuning_jax.py` keeps two helpers, prints that carry arithmetic and a
  hand-made last figure; `diagram_shortcuts.py` labels `NoisyPendulum()` as `Pendulum()`;
  `custom_one_link_arm.py` reads module constants, not `params`, and a comment restates a line.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

## Blocks v0.4 (winter 2027)

### Do the run caches on the sampled loop go?
- **Blocks:** the sampled loop (S31, D13); RULES 2.4, no run state on a System.
- **Today:** `StepSystem.rollout` and `HybridDiagram.last_result` cache a run beside `self.traj`.
- **Recommendation:** they go: `loop.compute_trajectory()` returns the `Trajectory` and stores
  `loop.traj`; the facades read `traj`.
- **Detail:** [end-state-api.md, D13](../plans/end-state-api.md#d13-the-run-state-caches-s31-rules-24)
- **Answer:**

### Does the parametric program become one family, with hand-written parameters and live MPC ports?
- **Blocks:** the compiled MPC program binding every parameter (S76, D25); the parametric
  optimizer table (OPW-1–OPW-5).
- **Today:** it binds the measured `x0` only; collocation freezes the plant, cost and set
  parameters in closures. Names ruled (R26: `ParametricProgram`, the key `plant`).
- **Options:** two parallel types; one class always taking `p`; one family, the plain
  `MathematicalProgram` its binding.
- **Recommendation:** the family; a hand-written program may declare its own `p`; the MPC block
  takes optional live-parameter ports, unwired at nominal (`live=["plant.mu"]`). Also open:
  `bind` on both the evaluator and the program.
- **Detail:** [end-state-api.md, D25](../plans/end-state-api.md#d25-s76s-two-remaining-asks-for-v04-parametric-programmd-2);
  [parametric-program.md §2](../plans/parametric-program.md#2-the-recommendation-one-family-the-frozen-program-a-binding)
- **Answer:**

### Does the MPC block ask its planner for three verbs instead of probing it?
- **Blocks:** the MPC block's contract (T6, D27) and MPPI inside it (MP-4).
- **Today:** `validate_mpc_planner` and the dual-rate export probe with `hasattr`.
- **Recommendation:** `decision_dimension()`, `prepare_online()`,
  `warm_start_guess(z_prev, y, *, dt_mpc, k)` and `z` on the tick's record; then
  `ModelPredictiveController(planner, dt_mpc=)` wraps trajectory optimization and MPPI alike.
- **Detail:** [end-state-api.md, D27](../plans/end-state-api.md#d27-the-mpc-blocks-planner-contract-mppimd-ask-2-t6)
- **Answer:**

### What does the standard stochastic policy look like as a seeded block?
- **Blocks:** the RL follow-ups (C5, D29): the RL heads rebuilt on it, the RL numbers changing
  once.
- **Today:** your direction (2026-10-22), `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)`; its methods
  and first heads are open.
- **Recommendation:** `GaussianPolicy(mean, log_std, *, dt, seed=None)` in `control/neural.py`,
  `seed=None` the deterministic face, `log_prob` and `entropy` beside `h`, the squashed variant
  a subclass.
- **Detail:** [end-state-api.md, D29](../plans/end-state-api.md#d29-the-standard-stochastic-policy-as-a-seeded-block-future-policiesmd-5-point-4)
- **Answer:**

### Do you adopt or drop the optional `KinematicModel` delegate?
- **Blocks:** the general serial chain (C6), decided with its mechanism description.
- **Today:** C6 is design first; its July 2026 draft is in git history.
- **Recommendation:** none yet.
- **Detail:** [TODO, C6](../plans/TODO.md#5-v04--the-draws-and-the-textbook-objects)
- **Answer:**

### Which pyro demos do the courses still need?
- **Blocks:** the pyro parity rows and the migration guide (C1).
- **Today:** of pyro's 195 example scripts, 29 done, 47 partial, 83 to do, 36 dropped.
- **Recommendation:** none yet.
- **Detail:** [pyro-port-remaining.md](../plans/pyro-port-remaining.md)
- **Answer:**

### Do the planning records get solver-specific views?
- **Blocks:** the planning records before the freeze.
- **Today:** no plan doc says more than this line.
- **Recommendation:** none yet.
- **Detail:** none beyond this entry (a one-line item of ROADMAP §6).
- **Answer:**

## Blocks v0.9 / v1.0

### Where do the course material and the research projects live?
- **Blocks:** the freeze candidate (K5, then the move K6), between terms after GMC714.
- **Today:** the GRO860 notes and webpage and GRO501's `\colab{}` cite the course notebooks, which
  clone `main` unpinned; on 2026-10-22 you put pinning off to the pinned cell (V6).
- **Options:** one repo; a course repo pinned on a release; one repo with pinned course cells and
  a nightly drift run. And `projects/`: research material that stays, or course material.
- **Recommendation:** pinned cells now, a course repository at v1.0.
- **Detail:** [repository-structure.md §4](../plans/repository-structure.md#43-recommendation), [§6](../plans/repository-structure.md#6-the-asks)
- **Answer:**

### How is the examples tree laid out?
- **Blocks:** the examples moves, between terms (with K5).
- **Today:** five kinds of folder on four axes; the README table is the only map.
- **Options:** tidy today's names; two axes, chapter and course; notebook-first with paired
  scripts.
- **Recommendation:** two axes, staged between terms; to rule with it, the chapter list and a home
  for `hybrid`, `mpc`, `realtime`, `robotic` and the racecar plant folder.
- **Detail:** [repository-structure.md §3](../plans/repository-structure.md#33-recommendation)
- **Answer:**

### Does a diagram's start state become a derived property, like its live parameters?
- **Blocks:** the foundations that could break a name (S29).
- **Today:** `DiagramSystem.x0`, `n` and `state` are stored; `Simulator` reads them after a
  `refresh()`.
- **Recommendation:** the row's plan: derived properties, `x0` and `state.nominal_value` with
  one owner each.
- **Detail:** [TODO, S29](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### Does Diffrax join as an optional JAX solver?
- **Blocks:** the foundations (S27); a new optional dependency.
- **Today:** the integrators are in `core/compile/evaluators/stepping.py` (OD-11).
- **Recommendation:** none yet.
- **Detail:** [TODO, S27](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### Does one posed-geometry hook replace `tf` plus a skin, and are the drawing `Sphere` and `Box` renamed?
- **Blocks:** the foundations (S44, S30).
- **Today:** `graphical/animation/primitives.py` and `core/geometry.py` each export a `Sphere`
  and a `Box`.
- **Recommendation:** none yet.
- **Detail:** [TODO, S44 and S30](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### Which names does the v1.0 freeze cover?
- **Blocks:** the frozen surface (V3).
- **Today:** `StepSystem`, `StepDiagramSystem`, `UnitDelay`, `ModelPredictiveController` and its
  sub-facade, and the spatial names the geometry work moves, are provisional.
- **Options:** each joins the freeze, or sits behind a visibly provisional name.
- **Recommendation:** none yet.
- **Detail:** [TODO, V3](../plans/TODO.md#v09--what-could-still-break-a-name)
- **Answer:**

### From v1.0, does a rename ship one minor release behind a deprecation shim?
- **Blocks:** the trust contract (V4); amends RULES "no deprecated aliases" from v1.0.
- **Today:** no alias or shim (R82); `xbar` / `ubar` and `plot_cost2go` wait on this.
- **Recommendation:** the row's proposal: a `DeprecationWarning` shim for one minor release, and
  a snapshot test of the export list.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**

## Standing (no release depends on it)

### Is there one separator rule for wires, parameter paths, block labels and duplicate state labels?
- **Blocks:** the larger naming alignment.
- **Today:** block outputs are recorded `id:port`, the plant's input `plant.u` (OD-13), parameter
  paths `motor.v2.A`; `add_subsystem` refuses an id holding `.`, `/` or `:`.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

### How do behavior cloning and diffusion policy enter the API?
- **Blocks:** nothing (for discussion).
- **Today:** three questions: does `fit` join `solve` and the program export as the third way to
  train; is diffusion policy in scope, research lane only; is distilling MPC or MPPI into a
  neural policy the first imitation demo?
- **Recommendation:** none yet.
- **Detail:** [future-policies.md §7](../plans/future-policies.md#7-questions-for-the-maintainer-for-discussion-not-blocking)
- **Answer:**

### Does the safety filter with control barrier functions get a rung?
- **Blocks:** nothing.
- **Today:** a design draft, after the field object (A1) and the geometry (A3).
- **Recommendation:** research lane until a course asks.
- **Detail:** [cbf-safety-filter.md](../plans/cbf-safety-filter.md)
- **Answer:**

### Does the RL planner scale rewards automatically, and by default?
- **Blocks:** nothing scheduled; a new default moves every RL demo's numbers once.
- **Today:** raw costs throttle PPO through the gradient clip; scaled, it reaches LQR on every
  seed tested.
- **Recommendation:** `reward_scale="auto"`, `c = 1 / (g_max dt)` from `problem.cost_bound()`;
  the default is yours.
- **Detail:** [rl-reward-scaling.md §6](../plans/rl-reward-scaling.md#6-proposal)
- **Answer:**

## Steps waiting for your go

Workboard rows marked **[ask]** whose question is not an entry above, by release; each waits for
your go (or your review) before an agent starts.

| Step, in plain words | Id | Release | What the ask is |
| --- | --- | --- | --- |
| The default output stub builds a NumPy array under JAX | T1 | v0.3 (wave D) | the returned type of the default `h` (`np.zeros(p)`) |
| The demo sweep to the flatness rule | D1.3 | v0.3 (wave D) | your review of each diff; `showcase_jax` waits |
| The planning demos on the solution verbs | D1.4 | v0.3 (wave D) | `solution.plot_*` and `compare(...)` in place of hand-rolled comparisons |
| `Source.show_signal` | D2 | v0.3 (wave D) | remove it (the same picture is `plot_trajectory`) |
| Two dead modules | D2 | v0.3 (wave D) | remove `planning/spatial/overlays.py` (with A3) and `experimental/symbolic/mechanics/utils.py` |
| A connected input rewired silently | S66 | v0.3 (wave D) | `connect` refuses to rewire it |
| Your demo pass, and a tabular Q-learning demo | K4 | v0.3 | your pass; the demo beside value iteration (R65) |
| The noise seeds on the draws | V1 step 7 | v0.4 | one seed per random block in `problem.realize` (RN-4, RN-5, AC-7) |
| LQG as a planner | LQGPlanner | v0.4 | `LQGPlanner(problem)` beside `LQRPlanner` |
| The field object | A1 | v0.4 | `StateField` → `Field` in `core/fields.py`; the certificate's `V` a field |
| Workspace geometry | A3 | v0.4 | the `core/geometry/` package; `planning.spatial` moved, then deleted |
| SAC's actor step | S50 | v0.4 | the critic before the update (today) or after it |
| Policy iteration on the grid | S51 | v0.4 | a mode or a sibling of `DynamicProgrammingPlanner` |
| Approximate dynamic programming | C5 | v0.4 | design first; it touches the planner API |
| The general serial chain | C6 | v0.4 | one mechanism description (DH table or URDF) |
| The pyro → minilink migration guide | C1 | v0.4 | its section in README |
| Value iteration on a finite horizon | S63 | v0.4 | backward sweeps for a finite `tf`, or a warning |
| Catalog hygiene | S64 | v0.4 | the per-plant clean-ups |
| The move of the course material | K6 | v0.9 | only if K5 says so |
| The pinned Colab cell | V6 | v0.9 | a pinned release in every notebook; `showcase_jax`'s `c_export` import |
| A lighter git history | V11 | December 2026 | a force-push without notebook outputs (176 MB clone to under 25 MB) |
| The convergence leftovers | TODO §7 | before V3 | the racecar 3-D skin moved; `Sys2Gym` / `SB3Controller` to the experimental tier; `planning/search/` cut; three facade verbs |
| The public face | V10 | v1.0 | a teaching quickstart first in README; `CONTRIBUTING.md` and templates |
| The Stable-Baselines3 notebooks | S49 | v1.0 | retire two, decide two |
| The realtime band's review | — | v1.0 | frozen surface or provisional |
| Behavior cloning | — | after v1.0 | the game mode and the fitting planner |
| A plant that owns its noise ports | — | later | the example (R57), then the two noisy plants revisited |
| Simon's curve speed scaler | — | later | into the path-tracking project, or the cap `v ≤ √(a_y,max / κ)` |
