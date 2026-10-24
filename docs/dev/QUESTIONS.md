# Open questions for the maintainer

One entry per question that waits for you, ordered by the release it blocks. Answer on the
`Answer:` line or in a session; the agent then records the ruling (the plan doc it shapes and
a dated record in docs/reviews/), applies it, and deletes the entry. Rulings already taken:
[docs/reviews/](../reviews/). Known defects: [KNOWN_ISSUES.md](KNOWN_ISSUES.md).

## Blocks v0.3 (the robotics-course release, December 2026)

### Do you review the 36 decisions the overnight run took for you, or accept them as a block?
- **Blocks:** your merge of the v0.3 branch into `dev` (R79).
- **Today:** the run of 2026-10-24 decided 36 points without you, each with its reason, the
  alternative and the commit (OD-1–OD-36); its §3 measures the intended number changes. You
  have since confirmed OD-9 (`outputs` keeps its name); OD-10 is superseded (no `ev.jit`).
- **Options:** accept all 36; review each; accept all but the ones you name.
- **Recommendation:** the log's "please look first at": the number changes (value iteration's
  default price is now the cost bound, so the GRO860 cost-to-go colour scale tops there;
  scores moved from a held law to the continuous loop, up to 8% for a stiff PD), the input-box
  sweep (OD-18), the planners' reported trajectory (OD-27), `compare`'s two forms (OD-28), the
  cost term ids (OD-22); then the rest as a block. The two it left for you follow.
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

### Which one place writes gradient descent out by hand: tutorial 07 or the closing PID example?
- **Blocks:** the rule "show the mechanism once, then the tool" (RULES 6.15, R75, R90).
- **Today:** RULES 6.15 and R75 name tutorial 07, which has no descent loop; the closing PID
  example (`examples/demos/compile/pid_autotuning_jax.py`) has one. R90 left the merge for later.
- **Options:** add the loop to tutorial 07; name the closing example in RULES 6.15; keep both.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

### One mechanical family, question 1: how does computed torque drive a plant with body-frame velocities?
- **Blocks:** one mechanical family (S32, ruled R62, postponed by you on 2026-10-24).
- **Today:** `ComputedTorque` writes one torque per coordinate; the boat and the 3-D plane
  (`q̇ = N(q) v`) are refused.
- **Recommendation:** `v̇_des = Kp N(q)⁻¹ (q_d − q) + Kd (v_d − v)` (today's law when `N = I`),
  refusing a non-square `N`.
- **Detail:** [TODO, S32](../plans/TODO.md#4-v03--a-cleaner-core-and-the-differentiable-path-then-gmc714)
- **Answer:**

### One mechanical family, question 2: how does a model-based law drive an underactuated plant?
- **Blocks:** one mechanical family (S32).
- **Today:** `ComputedTorque(CartPole()) @ CartPole()` fails at wiring (two torques, one input);
  the boat and the plane are underactuated too.
- **Recommendation:** `u = B(q)⁺ τ` when `B` is not square, a solve when it is.
- **Detail:** [TODO, S32](../plans/TODO.md#4-v03--a-cleaner-core-and-the-differentiable-path-then-gmc714);
  [KNOWN_ISSUES.md](KNOWN_ISSUES.md)
- **Answer:**

### One mechanical family, question 3: does `DynamicBicycle` move onto the generalized mechanical base now?
- **Blocks:** one mechanical family (S32); the car family clean-up (K8).
- **Today:** the row moves its base from `DynamicSystem` to the generalized child, but its named
  ports and its subclasses' extra states do not fit `[q; v]`.
- **Options:** move it now; rename `M` to `H` only and move it when K8 reworks the cars.
- **Recommendation:** rename only; the base waits for K8.
- **Detail:** [TODO, S32](../plans/TODO.md#4-v03--a-cleaner-core-and-the-differentiable-path-then-gmc714)
- **Answer:**

### Do the problem's draws reach the noise inside an evaluated loop now, or does a warning stand in until v0.4?
- **Blocks:** Monte Carlo numbers that mean what they say; the per-block seeds (RN-4, v0.4).
- **Today:** `evaluate(..., n_trials, key)` draws starts and plant parameters per trial, but a
  noise block inside the loop keeps its seed, so every trial plays the same signal, with no
  warning; the problem's `disturbances` ports are held at nominal, with one
  (`minilink/planning/evaluator.py`). The randomness rulings ask for the warning until RN-4
  (RD-D19).
- **Options:** (a) the warning now, RN-4 in v0.4; (b) connect the problem's key to
  `System.realize` now (RN-4 in v0.3), the Monte Carlo numbers changing once.
- **Recommendation:** (a): RD-D19 already rules the warning; only (b) is new.
- **Detail:** [randomness.md §1](../plans/randomness.md#1-what-the-code-still-lacks),
  [§8](../plans/randomness.md#8-rulings); [the run's log, §6 item 1](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Should a diagram's `params=None` read live parameters or a snapshot?
- **Blocks:** the compiled evaluator's contract (DESIGN §5).
- **Today:** compiled without `bind_params`, `params=None` reads the live parameters on NumPy and
  the first call's values on JAX; DESIGN §5 says "the parameters compiled in". A partial dict
  reads the live parameters of the blocks it leaves out.
- **Options:** live on both backends; a snapshot at compile on both (`bind_params=True` today).
- **Recommendation:** none yet.
- **Detail:** [the run's log, §12](../reviews/2026-10-24-v03-run-decisions.md#12-the-docs-brought-to-the-branch-the-landed-plans-and-stale-audits-retired); [KNOWN_ISSUES.md](KNOWN_ISSUES.md)
- **Answer:**

### Should a run that blows up score worse than a run that leaves the sets?
- **Blocks:** the scoring formula (R31, R32).
- **Today:** with the price on, a diverged run costs `price`, a finite run that leaves `X`
  `cost + price`, so diverging scores better; its gradient is NaN.
- **Options:** charge a diverged run `cost_bound + price`; or put `isfinite(cost)` in `finite`
  and keep the finite prefix's cost.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §6 item 2](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Does the tabular learner step with the grid's solver?
- **Blocks:** GRO860's claim that value iteration on the same grid is what the learner approaches.
- **Today:** its environment steps with RK4, its grid defaults to Euler (R8); the claim holds
  only with `solver="euler"`. Changing the default changes the course results.
- **Options:** the grid's solver by default; RK4, with `solver="euler"` where it is taught.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §6 item 3](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Does the Gym bridge stop one step earlier, though the fix touches a GRO860 notebook?
- **Blocks:** the gymnasium bridge's horizon.
- **Today:** on an infinite horizon `Sys2Gym` runs one step too many (11 steps for `tf = 1`,
  `dt = 0.1`), unlike the RL environment; the Gymnasium notebook's own class mirrors the line.
- **Options:** fix the bridge and the notebook; the bridge only.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §6 item 4](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
- **Answer:**

### Should the same cost added twice be refused?
- **Blocks:** the cost algebra's parameters.
- **Today:** `q + 10 * q` shares its parameters between both terms; refusing it refuses
  `cost + cost` too.
- **Options:** refuse; keep the sharing.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §6 item 5](../reviews/2026-10-24-v03-run-decisions.md#6-the-review-pass-after-the-run)
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

### Do `closed_loop` and `feedback` join a band facade?
- **Blocks:** the band facades (a public name).
- **Today:** on the root prelude, on no band facade, as before the run.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

### Should a chain `A >> B >> C` carry every name?
- **Blocks:** only a chained diagram's display name.
- **Today:** it is named `A >> B` (checked on the branch), as before the run.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

### Do three demos change style?
- **Blocks:** your demo pass (K4) and the closing example (S77).
- **Today:** `pid_autotuning_jax.py` keeps two helpers, prints that carry arithmetic and a
  hand-made last figure; `diagram_shortcuts.py` labels `NoisyPendulum()` as `Pendulum()`;
  `custom_one_link_arm.py` reads module constants, not `params`, and a comment restates a line.
- **Recommendation:** none yet.
- **Detail:** [the run's log, §7](../reviews/2026-10-24-v03-run-decisions.md#7-the-second-review-pass)
- **Answer:**

### Does the dependency law gain three exceptions, or do three imports move?
- **Blocks:** DESIGN's dependency law.
- **Today:** no exception covers `planning/dynamic_programming/lqr.py` → `control`,
  `core/discrete/` and `System.__mod__` → `simulation.discrete`, or `graphical/diagrams/` →
  `blocks.routing` and `simulation.discrete`.
- **Options:** write the exceptions with their reasons; move the code.
- **Recommendation:** none yet.
- **Detail:** [DESIGN, "Dependency law"](DESIGN.md#dependency-law)
- **Answer:**

## Blocks v0.4 (winter 2027)

### Do the run caches on the sampled loop go?
- **Blocks:** the sampled loop (S31, D13); RULES 2.4, no run state on a System.
- **Today:** `StepSystem.rollout` and `HybridDiagram.last_result` cache a run beside `self.traj`.
- **Recommendation:** they go: `loop.compute_trajectory()` returns the `Trajectory` and stores
  `loop.traj`; the facades read `traj`.
- **Detail:** [end-state-api.md, D13](../plans/end-state-api.md#d13-the-run-state-caches-s31-rules-24)
- **Answer:**

### Does a sampled controller with memory and draws need a channel of its own?
- **Blocks:** the sampled loop (S31, D14).
- **Today:** MPPI's warm start and a diffusion policy's chunk have no channel; the RL collectors
  thread their own keys.
- **Recommendation:** no: a controller with memory is a `StepSystem` with state; its draws come
  from its own `params["seed"]` and the tick, as the noise block's do.
- **Detail:** [end-state-api.md, D14](../plans/end-state-api.md#d14-controller-memory-and-the-draws-on-the-tick-s31-future-policiesmd-5-points-2-4)
- **Answer:**

### Does `problem.compile(policy)` pick the continuous or the sampled run by the policy's kind?
- **Blocks:** the sampled face of the compiled problem (V1 step 7b, D15).
- **Today:** v0.3 refuses a `StepSystem` controller (`require_continuous`).
- **Recommendation:** a continuous law runs as a diagram, a `Computer` on the sampled kernel's
  JAX face; one `compile` and one `evaluate` for both.
- **Detail:** [end-state-api.md, D15](../plans/end-state-api.md#d15-the-compiled-problem-dispatches-on-the-policys-kind-s31-v1-step-7b)
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

### Is the path-integral planner named `MPPIPlanner` or `PathIntegralPlanner`?
- **Blocks:** the MPPI planner (S72, D26).
- **Today:** working names `PathIntegralPlanner` / `PathIntegralRecord`; prototypes in
  `experimental/mppi/`.
- **Recommendation:** `MPPIPlanner` and `MPPIRecord` (the `LQRPlanner` precedent).
- **Detail:** [end-state-api.md, D26](../plans/end-state-api.md#d26-mppis-name-mppimd-ask-1); [mppi.md §10](../plans/mppi.md#10-asks-for-the-maintainer)
- **Answer:**

### Does the MPC block ask its planner for three verbs instead of probing it?
- **Blocks:** the MPC block's contract (T6, D27) and MPPI inside it (MP-4).
- **Today:** `validate_mpc_planner` and the dual-rate export probe with `hasattr`.
- **Recommendation:** `decision_dimension()`, `prepare_online()`,
  `warm_start_guess(z_prev, y, *, dt_mpc, k)` and `z` on the tick's record; then
  `ModelPredictiveController(planner, dt_mpc=)` wraps trajectory optimization and MPPI alike.
- **Detail:** [end-state-api.md, D27](../plans/end-state-api.md#d27-the-mpc-blocks-planner-contract-mppimd-ask-2-t6)
- **Answer:**

### How many plant draws does MPPI take per control sample by default?
- **Blocks:** MPPI's stochastic branch (S72, D28).
- **Today:** not built; the plan weighs 4 against 1.
- **Recommendation:** `n_plant_samples=1`, the bias stated in the docstring.
- **Detail:** [end-state-api.md, D28](../plans/end-state-api.md#d28-the-stochastic-default-of-mppi-mppimd-ask-3)
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

### Where does the RL rollout environment live once MPPI reads it?
- **Blocks:** MPPI (S72, D30).
- **Today:** `RolloutEnvironment` is in `learning/reinforcement_learning/`. D30's reason ("a
  caller of the sampled kernel") reads against R13 (the RL environment depends on the step model
  only).
- **Options:** it stays; it moves to `planning/environment.py`.
- **Recommendation:** it stays; its reason restated against R13.
- **Detail:** [end-state-api.md, D30](../plans/end-state-api.md#d30-where-rolloutenvironment-lives-mppimd-ask-5)
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

### Are the rest of pyro's game demos ported to the realtime simulator, or dropped?
- **Blocks:** the parity rows (C1); the behavior-cloning game mode would reuse them.
- **Today:** the table marks them "Drop (use `simulation/realtime/` or skip)".
- **Recommendation:** none yet.
- **Detail:** [pyro-port-remaining.md](../plans/pyro-port-remaining.md)
- **Answer:**

### Is the parity table's "hybrid events / mixed-rate sim: Drop" row still right?
- **Blocks:** the parity rows (C1).
- **Today:** the code has the hybrid simulator and `Computer` (`simulation/discrete/`);
  event-driven switching stays out of scope (ROADMAP §7).
- **Recommendation:** none yet.
- **Detail:** [the run's log, §12](../reviews/2026-10-24-v03-run-decisions.md#12-the-docs-brought-to-the-branch-the-landed-plans-and-stale-audits-retired)
- **Answer:**

### Are Lyapunov certificates a GRO860 checklist row of their own, or an analysis tool the lecture uses?
- **Blocks:** the field object (A1), which makes the certificate's `V` a field.
- **Today:** `region_of_attraction` returns a provisional `LyapunovCertificate` (ROADMAP §3).
- **Recommendation:** none yet.
- **Detail:** [fields.md](../plans/fields.md)
- **Answer:**

### Do the planning records get solver-specific views?
- **Blocks:** the planning records before the freeze.
- **Today:** no plan doc says more than this line.
- **Recommendation:** none yet.
- **Detail:** none beyond this entry (a one-line item of ROADMAP §6).
- **Answer:**

### Is `PlanningProblem.metadata` documented or retired?
- **Blocks:** the problem's constructor before the freeze.
- **Today:** a read-only mapping that no library tool reads (`minilink/planning/problems.py`);
  left as is in session 4.
- **Recommendation:** none yet.
- **Detail:** [TODO §7](../plans/TODO.md#7-later-ideas)
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

### Does the pinned install cell use `pip install minilink==<tag>` or `git clone -b v<tag>`?
- **Blocks:** the pinned Colab cell (V6).
- **Today:** the course notebooks import from the examples tree by relative path.
- **Options:** `pip install` (fast, no examples tree); `git clone -b` (keeps those imports).
- **Recommendation:** none yet.
- **Detail:** [repository-structure.md §6, ask 2](../plans/repository-structure.md#6-the-asks)
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

### Does minilink raise its own error family?
- **Blocks:** the trust contract (S70) and the wiring-time checks (S66).
- **Today:** plain `ValueError` / `TypeError`.
- **Recommendation:** the row's proposal: `WiringError`, `ShapeError`, `SolverError` subclassing
  today's errors, so no caller breaks.
- **Detail:** [TODO, S70](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**

### Does minilink get a Zenodo DOI, and is a JOSS paper wanted?
- **Blocks:** v1.0's release hygiene (V2), and the RULES 6.9 carve-out JOSS would need.
- **Today:** on PyPI since 0.1.0.
- **Recommendation:** none yet.
- **Detail:** [TODO, V2](../plans/TODO.md#v10--a-colleague-can-build-a-course-on-it)
- **Answer:**

## Standing (no release depends on it)

### Do you place the proposed sentence in CONSTITUTION §1, or drop it?
- **Blocks:** nothing; the shape it states is ruled (R1).
- **Today:** "One model and one differentiable path from it to a cost, solved by one optimizer;
  every approach that controls a vehicle or a robot is a tool on that path: …".
- **Recommendation:** none (yours to place or drop).
- **Detail:** [end-state-api.md §4](../plans/end-state-api.md#4-a-sentence-for-constitution-1-maintainer-owned)
- **Answer:**

### Does CONSTITUTION invariant 1 still call the sampled-loop seam an open decision?
- **Blocks:** nothing.
- **Today:** it says closing the `Computer` / `HybridDiagram` seam "is an open decision of the
  roadmap"; R13 ruled the sampled loop a tool, and no such item is open.
- **Recommendation:** none yet.
- **Detail:** [CONSTITUTION, the invariants](CONSTITUTION.md#the-five-contract-invariants)
- **Answer:**

### Does an undeclared feedthrough raise or warn at compile?
- **Blocks:** wiring mistakes failing at wiring time (S66).
- **Today:** an output that reads an undeclared input is handed zero, so a loop solves the wrong
  fixed point silently ([KNOWN_ISSUES.md](KNOWN_ISSUES.md)).
- **Options:** raise; warn.
- **Recommendation:** none yet.
- **Detail:** [TODO, S66](../plans/TODO.md#3-wave-d--textbook-code-minimal-demos-consolidation)
- **Answer:**

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
