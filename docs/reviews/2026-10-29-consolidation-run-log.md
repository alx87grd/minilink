# The consolidation run (2026-10-29): the log

**Audience:** Prof. Alexandre Girard (Maintainer), for the next conversation.
**Context:** the run that implements [the consolidation plan](../plans/consolidation-after-the-runs.md)
(rulings R168–R179 in [the record](2026-10-29-consolidation-rulings.md)), with the amendments the
maintainer gave in the session that started it (§1). Where a review would have been asked, the
decision is taken as the maintainer would, written in §2 to be raised with him, and the run
continues.

**How to read it.** §1 is the amendments of the session and the summary. §2 lists every decision
taken without the maintainer (C-1…), each with the step, the choice, the reason and the
alternative. §3 is the ledger of intended number changes. §4 is the textbook-check statement of
every math diff. §5 is what was not done. §6 is the progress record, one line per pushed row.

## 1. The amendments of the session, and the summary

The maintainer's answers when the plan was checked (2026-10-29), each overriding the plan where
they differ:

1. **No refusal of `x >> (loop)`** (plan §5): composition stays simple; `>>` into a loop is
   always legal ("feed the loop's reference"); the docs warn that `@` binds before `>>`.
2. **The cost bound** (plan §4.2): one formula owned by the problem, `problem.cost_bound(tf,
   dt=None)`: the integral for the continuous judge, the sum at a tool's `dt` for every tool
   that steps on a grid (the face reads the problem's at its own `dt`).
3. **The failure flag** (plan §4.2): `penalize_failure: bool` (default off) and `failure_cost:
   float | None` (`None`: the cost bound) replace `infeasible_cost`; `cost_bound` is always
   computed; the word `price` goes.
4. **The plant's step** (plan §5): one rule, the continuous simulator's, no exception; `dt=` at
   the runs, `substeps=` at the compiles; every demo, notebook and baseline loop gets an
   explicit step so its numbers do not move.
5. **The MPC** (plan §6): `ModelPredictiveController(planner, delay=, warm_start=, …)` keeps both
   switches, independent; underneath it always builds the one three-block architecture (solve,
   command, memory), `delay=True` adds a visible `UnitDelay`; the default becomes
   `delay=False, warm_start=True`; `lookahead` goes; every demo sets the flags that keep its
   behaviour.
6. **The default start** (plan §7): the start region's bounding-box centre when finite and
   inside, else `sys.x0` when inside, else a refusal asking for `x_start`.
7. **The cart-pole RL demo** (plan §4.1): its reliability is not re-measured and it is not
   retuned; the maintainer handles it.

## 2. Decisions taken without the maintainer

- **C-1 (step 1) The trajectory-optimization planner's offline key.** The plan said "built at
  construction, rebuilt by the user, or a key on the parts' identity". The key is the problem's
  parts by identity (plant, cost, `X`, `U`, `X0`, `Xf`, held with the program so no identity is
  reused), its horizon `tf` and its numbers' digest: a part replaced or a new horizon builds a
  new program (the test "a new horizon before a solve is a new program" keeps passing); an edit
  inside a part needs a new planner. Alternative: rebuild every solve (a JAX re-trace per solve).
- **C-2 (step 1) A transcription holds the plant it compiled** (`Transcription.plant_evaluator`,
  by the plant's identity and the backend), so a held planner's repeat solve compiles nothing
  (the reconstruction read the plant through the cache before). Alternative: pass the evaluator
  through every transcription method.
- **C-3 (step 1) The compile-once tests.** The cache's own cases go (a rebuilt loop reuses, a
  copy shares nothing, the eviction, the fingerprint, the port bound in the key, the run
  records no edit); the edit cases stay as the proof that a convenience call sees every edit
  (a function swapped, a helper edited, a class method replaced, an embedded model's numbers);
  the repeats are now a held tool's (a `Simulator`, a compiled sampled loop), and one test
  pins that a convenience call compiles each time.

- **C-4 (step 2) The spatial lane's numbers.** The workspace fields' trees are their parts':
  `ClearanceField.params = {"scene": scene.params}`, `CorridorMarginField.params = {"track":
  track.params}` (`{"path": {"waypoints"}, "half_width"}`); a `Scene` is `{"obstacles": …,
  "workspace_fields": …}`, each shape and each density its own (`Sphere {"center", "radius"}`,
  `Box {"lower", "upper"}`, `Union` by position, `Inflated {"base", "radius"}`, `GaussianField
  {"center", "amplitude", "sigma"}`), each with `with_params`. The body's probes are placed by
  the plant's own kinematics on the plant's numbers (one owner: the plant), so the field no
  longer hands its tree to the plant's `tf`. Alternative: the field's tree holding a copy of
  the plant's numbers too (a second home).
- **C-5 (step 2) No shared class-level `params = {}`.** The base classes (`CostFunction`,
  `StepCost`, `Set`, `InputSet`, `Field`, `Distribution`, `Shape`, `ReferencePath`,
  `WorkspaceField`, `Shaping`) read `params` through `core/params.py`'s `own_params()`: each
  instance's own dict, a new `{}` read from an instance that holds none, nothing written on
  read. Alternative: a read-only mapping that refuses a write; it is not a JAX pytree node,
  so a tree holding one (a field cost's `{"field": {}}`) would fail to trace.
- **C-6 (step 2) The numbers kept out of `params`, each with its structural reason** (the
  audit, `tools/audit_numbers.py` over every public system plus the core objects by hand):
  the drawn geometry of a skin read in `tf` (`DoublePendulum`/`Acrobot.l2` drawn, the cart's
  box, the cars' tracks and widths: rendering, never the dynamics); a summing junction's signs
  (`Sum.signs`, the wiring); a neural policy's action box (`u_mid`, `u_half`, the plant's input
  bounds; as leaves the RL algorithms would train them); a clock's period (it decides the
  plan); an LTI model's operating point (`xbar`, `ubar`, `ybar`, never in `f`); a
  `SampledCost`'s period, rate and origin (the face passes the problem's own leaves). Moved
  into `params`: the fields' (under `"field"`), the shapings' thresholds and scales (under
  `"shaping"`, the factories now return small `Shaping` objects; a plain function still works),
  `QuadraticField`'s schedule times `t`, `TrajectorySource`'s times (`{"t", "values"}`),
  `TimeCost.eps`, an LTI block's matrices (`{"A", "B", "C", "D"}`, also `TransferFunction`,
  `Lead`, `LowPassFilter`: the realization's matrices; the coefficients stay for display),
  `LookupPolicy`'s table and levels (`{"u_table", "levels"}`, now a traceable multilinear or
  nearest lookup; it equals SciPy's to the bit on a 2-D grid), the shapes', paths' and
  densities' numbers (C-4), a model-based law's model (C-7).
- **C-7 (step 2) A model-based law's model numbers** (the TODO §1.1 row, landed here):
  `ComputedTorque`, `SlidingMode`, `JointImpedance`, `TaskImpedance`, `TaskKinematic`,
  `TaskKinematicNullspace` hold `params["model"]`, a deep copy of the plant's numbers taken at
  construction, and call the model hooks with it. **Behaviour change:** an edit of
  `plant.params` after the law is built no longer reaches the law's model (it did, through the
  shared object); a mismatch is `law.params["model"]["m"] = …`. DESIGN's "Embedded-model params
  rule" is rewritten. **To raise:** the deep copy rather than an alias of the plant's dict (an
  alias would make "the model's numbers" and "the plant's numbers" one edit again).

- **C-8 (step 3) The method's numbers through the optimizer.** The native laws hold
  `params` (`GradientDescentLaw {"learning_rate"}`, `AdamLaw {"learning_rate", "b1", "b2",
  "eps"}` and `"max_grad_norm"` when clipped, the clip's presence structure);
  `Optimizer.params` is the program's tree with `"method"`, and `Optimizer.solve(params=)`
  splits `"method"` off for the law (an argument of the jitted run, so a new learning rate
  never recompiles). A host method (SLSQP, trust-constr, Ipopt, L-BFGS) has `"method": {}`:
  its options are its stopping rule, kept as constructor options. The trajectory-optimization
  planner's tree is the program's leaves, `"method"` and `"seed": None`; `planner.solve(params)`
  and `pv.solve(params)` split the method's numbers off before the problem copy.
- **C-9 (step 3) The RL planner's tree is a report.** `planner.params = {"method":
  algorithm.method_params, "seed": seed}`: the discount and the algorithm's numbers (PPO's
  `gae_lambda`, `clip_range`, `ent_coef`, `vf_coef`; SAC's `tau`) and its Adam's under
  `"optimizer"`. Their home stays the algorithm's attributes, read when the update is first
  traced (set them before learning); no `solve(params=)` rebinds them. **To raise:** a
  training run with a tree argument (every update reading the numbers as arguments) is
  possible, and a larger change of the four algorithms.
- **C-10 (step 3) MPPI's tree** gains `cost`, `discount_rate` and `X`'s numbers (when `X` has
  some); the environment's step merges a given cost and discount into the face's tree and reads
  a given `X` (the input box the samples are clipped to stays structure, read at build). The
  failure price is the problem's at build (a moved weight does not move it; §4 rewrites it).
  An MPPI MPC on a traced loop (`problem.compile(mpc % dt, backend="jax")`) fails because the
  face's `initial(params)` is host-only; that is plan §6's traced first guess, landed there.

## 3. Ledger of intended number changes

| Row | What moved | Why |
| --- | --- | --- |

## 4. Textbook check

Every math diff was read beside `planning/dynamic_programming/dp.py`.

- Step 1: no equation line changed.
- Step 2: the fields' margins and costs, the shapes' `sdf`, the paths' distance and arc
  lengths, the densities, the LTI matrices, `TimeCost`, `TrajectorySource` and the new
  `LookupPolicy.action` were read beside `dp.py`: bindings (`params`, then `xp`), the
  parameters one per line, the equations as named lines with their textbook comments, a
  name returned. `arc_lengths` (ℓᵢ and sᵢ) is a generic building block shared by four
  methods (RULES 7.8).

## 5. Not done

- The cart-pole RL demo's reliability (the maintainer's, §1 item 7).

## 6. Progress

| Step | Commit | Row |
| --- | --- | --- |
| 1 | `dd7822d4` | the compile cache out (R168) |
