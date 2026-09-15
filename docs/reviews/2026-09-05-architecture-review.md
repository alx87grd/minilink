# Minilink architecture and vision review — September 2026

Point-in-time external audit ahead of the v0.1 teaching release.
Branch `dev-alex` @ `5c41f83`. Reviewed 2026-09-05.

This is an **audit record**, not a fourth backlog home. Items worth acting on
should be moved by the maintainer into [ROADMAP.md](../../ROADMAP.md) (maturity
and milestone decisions) or [docs/plans/TODO.md](../plans/TODO.md) (operational
work). Nothing here was applied to the code.

---

## 0. Verdict in one page

**The foundation is sound. The surface is too large for one maintainer, and the
documented vision has quietly drifted ahead of — and in a few places behind —
the code.**

The core object model (`System` → ports → `DiagramSystem` → compile → simulate)
is a good design, defensibly close to what Drake and python-control converged
on, with a composition grammar (`@`, `>>`, `+`) that is genuinely nicer for
teaching than either. The control band reads like a textbook. The parameter
contract, the `Trajectory` type, the signal/port metadata model, and the
compile-vs-reference parity tests are all better than typical for a personal
research toolbox. `ruff` is clean, 884 tests pass, coverage is 77%, examples
have zero import drift, and there are only 5 `TODO` markers in 46 600 lines.
That is not a codebase full of AI slop.

The problems are problems of **scale and closure**, not of taste:

1. **The library is 34% larger than python-control with 45% of its relative test
   weight** — and python-control is maintained by a multi-institution team.
   320 public classes and 269 public module-level functions are more public API
   than one person can freeze, document, and support through a student cohort.
2. **Three headline claims are not delivered by the code today**: teaching-first
   imports (used in 2 of 555 example imports), "differentiate through the same
   `f`" (61% of catalog plants fail to compile under JAX), and "a diagram can be
   used anywhere a system can" (`HybridDiagram` — the MPC path — is not a
   `System` at all).
3. **Two scope reversals in three months** (discrete time was declared out of
   scope in June, then shipped as ~11% of the library; `__init__` re-export
   barrels were rejected in June, then adopted and documented as the canonical
   course API in August) have left ROADMAP §7, DESIGN §2/§3, and the example
   corpus describing three different products.
4. **The default simulation grid is wrong by two orders of magnitude** — every
   `compute_trajectory(tf=10)` on a stock plant returns 100 001 samples, and on
   the JAX path that silently selects fixed-step RK4 with 400 000 RHS
   evaluations instead of SciPy's 200.

None of these are deep architectural errors. All four are fixable in days to a
few weeks. The genuinely *structural* questions — the evaluator/solver split,
the hybrid stack's position outside the type system, and the two geometry
vocabularies — are discussed in §4 with recommendations that mostly amount to
**subtraction**.

**Recommended posture for v0.1: freeze less, delete more, and make the teaching
surface actually be the surface the examples use.** The single highest-leverage
week of work is §8 P0.

---

## 1. Method

Everything below is reproducible. Environment: conda env `minilink`
(`/opt/anaconda3/envs/minilink/bin/python`, Python 3.13), repo root on
`PYTHONPATH`, `MPLBACKEND=Agg`.

| What | How |
| --- | --- |
| Read the contract docs | `README.md`, `DESIGN.md` (900 lines), `ROADMAP.md`, `AGENTS.md`, `docs/plans/*` |
| Read the core | `core/system.py`, `diagram.py`, `wiring.py`, `composition.py`, `facades.py`, `feedback.py`, `signals.py`, `trajectory.py`, `costs.py`, `sets.py`, `geometry.py`, `hybrid_diagram.py`, `backends.py`, all of `core/compile/`, `simulation/`, `control/`, `blocks/`, `dynamics/abstraction/` |
| Ran the suite | `pytest` → 884 passed, 3 skipped, exit 0 |
| Measured coverage | `coverage run --source=minilink -m pytest` → 77% (20 458 stmts, 4 685 missed) |
| Lint gate | `ruff check .` clean; `ruff format --check .` → 420 files formatted |
| API-surface census | AST walk of all 224 library modules |
| Dead-API detection | every public module-level name, searched across `minilink/`, `tests/`, `examples/`, `benchmarks/`, `docs/` |
| Evaluator usage census | all 88 evaluator methods, referenced-from analysis |
| Catalog backend matrix | instantiated all 49 catalog classes, `compile(backend=…)` for both backends |
| Import-style census | AST walk of 100 example scripts + 24 notebooks |
| Doc/code drift | markdown file paths and all `:mod:`/`:class:`/`:meth:` refs resolved against the AST |
| Behavioural probes | README quick-start, README custom-plant example, operator semantics, default time grid, `x0` propagation |
| Landscape | Drake, python-control (0.10.2 source, locally), bdsim, Archimedes, CasADi/acados, Diffrax/JAX ecosystem |

Where a claim below is quantitative, the command that produced it is in
[Appendix A](#appendix-a--reproducing-the-measurements).

---

## 2. The vision — is the claim coherent?

### 2.1 What minilink says it is

From DESIGN §*Product identity & scope*:

> A Python/JAX block-diagram toolbox for modeling, simulating, controlling,
> optimizing, and learning with dynamical systems — equations that read like
> textbook math. **Distinct edge:** one object model for plants, controllers,
> diagrams, and NN/ID blocks … without splitting a "sim stack" from a "learning
> stack."

**This is a good claim.** It is specific, it names a real gap in the landscape,
and it is falsifiable. It is also, as written, *almost* true — which is why the
places it is not true matter so much.

### 2.2 Where it sits in the landscape

I checked the four closest neighbours against their current docs/source.

| Peer | What they own | Where minilink genuinely differs |
| --- | --- | --- |
| **Drake** (`systems::framework`) | Multibody/contact, deep MathProg, event scheduling. `System` is stateless; **`Context` holds time, state *and parameters***; scalar-type conversion clones `System<double>` → `System<AutoDiffXd>` | Minilink keeps `params` on the object with a call-time override — python-control's model, not Drake's. Simpler and better for teaching; the cost is that "stateless" is a half-truth (§4.2) |
| **python-control 0.10.2** | Classical LTI, `nlsys(updfcn, outfcn, …, params=dict)`, `interconnect()` by signal name, `input_output_response(sys, timepts, U, x0, params)`, `control.optimal` | Minilink's `f(x,u,t,params)` signature and params policy are the same idea. Minilink adds: real ports with labels/units/bounds, operator composition, animation, JAX. python-control is 34 807 LOC library + 27 255 LOC tests |
| **bdsim** (Corke) | Code-first block diagrams, wires carry any Python type, `bd.connect()`, ~60 blocks, continuous + sampled | Closest *teaching* peer. Minilink's blocks band is 15 blocks (§6.5) but its plants are far deeper and it has an optimize/learn story bdsim does not |
| **Archimedes** (2025–, targeting 1.0 mid-2026) | CasADi symbolic compile, C codegen, HIL. Explicitly *causal state-space*, explicitly rejects acausal and graphical modelling. Keeps unstable features in a **separate `experimental` module**; defines 1.0 as "6+ months without breaking API changes in core" | The most direct competitor for the "Python toolkit for real control engineering" niche. Their stable/experimental *namespace* split is the pattern minilink should copy (§4.7) |

**Conclusion: the positioning is defensible and the gap is real.** Nobody else
offers textbook-readable nonlinear plants + operator composition + animation +
trajopt/DP/RRT + optional JAX autodiff in one object model. DESIGN's comparison
table is honest and mostly accurate.

Two corrections to that table:

- The **python-control row says "Interop"**. There is no interop code anywhere
  in the repo (`grep -rn "import control"` → nothing). This matters, see §8 P2-12.
- The **Pyro row** claims successorship, and the parity audit is 50 KB of
  detailed tracking — but that audit contains stale rows (§6.7).

### 2.3 The vision is moving faster than the docs

This is the finding I would most want a maintainer to sit with.

| Decision | Recorded as | Current code |
| --- | --- | --- |
| Discrete time | June 2026: *"discrete time is explicitly OUT of scope (continuous-time only) — don't propose discrete blocks, ZOH, RNN dynamics, or sample-time machinery"* | `StepSystem`, `StepDiagramSystem`, `StepExecutionPlan`, `step_compiler`, 6 step evaluator classes, `Computer`, `StepSchedule` (multi-rate integer divisors), `ZOHHold`, `HybridDiagram`, `HybridSimulator`, `analysis/discretize`. ≈ 2 750 LOC core + 900 realtime + 1 349 MPC ≈ **11% of the library**, **15% of the tests** |
| `__init__` re-export barrels | June 2026: *"explicitly rejected `__init__.py` barrel re-exports even for catalog/blocks"* | `minilink/__init__.py`, `catalog/`, `control/`, `analysis/`, `blocks/` all ship lazy `_EXPORTS` barrels; DESIGN §2 documents them as *"canonical course / script API"*; ROADMAP §5 marks the decision **done** |

Both reversals may well be right. The problem is that **the artefacts of the old
decision were never removed and the artefacts of the new one were never
adopted**: the barrels exist but nothing imports through them (§3.1), and
ROADMAP §7's out-of-scope list still reads as though hybrid were a small
experiment.

> **Recommendation V-1.** Before freezing v0.1, do one pass over ROADMAP §7 and
> DESIGN §*Product identity* and rewrite them to describe the product that
> exists, including the hybrid path as a first-class (if subsidiary) capability
> with its own TRL row. A scope list that a reader can falsify by `grep` costs
> more credibility than it buys.

---

## 3. Where the code diverges from the vision

Five claims from `README.md` / `DESIGN.md`, tested.

### 3.1 "Teaching-first imports" — used in 2 of 555 example imports

DESIGN §2 defines three layers: root prelude → band facades → defining module,
and says band facades are the *"canonical course / script API"*.

Across all 100 example scripts and 24 notebooks:

| Layer | Uses |
| --- | --- |
| Root prelude (`from minilink import …`) | **1** |
| Band facade (`from minilink.catalog import …`, `minilink.control`, `minilink.analysis`) | **1** |
| Defining module (`minilink.X.Y`) | 314 |
| Deep (4+ levels) | 239 |

`minilink.catalog` — a 156-line re-export package whose entire purpose is short
teaching imports — is used **zero times**, against 93 deep
`minilink.dynamics.catalog.*` imports. The flagship marketing notebook
`showcase_minilink.ipynb` opens with
`from minilink.dynamics.catalog.pendulum.pendulum import Pendulum`. The teaching
MPC notebook needs **16 distinct deep imports** to set up one problem.

This is the sharpest vision/reality gap in the repo, and the cheapest to close.
It is also a decision that was made twice in opposite directions (§2.3), so it
needs a maintainer ruling, not a mechanical fix:

> **Recommendation V-2 (decide, then enforce).**
> **Option A — adopt.** Rewrite example/notebook imports to the documented
> layers (mechanical, ~1 day, low risk, testable by an AST check in CI). The
> barrels then earn their 300-odd lines and students get a 3-line preamble.
> **Option B — abandon.** Delete `minilink/catalog/` and the band `_EXPORTS`
> blocks, return to the June "import from the defining module" rule, and rewrite
> DESIGN §2 to say so.
> Either is fine. The current state — documented, shipped, unused — is the only
> one that is not.

I lean **A** for a student release: `from minilink.catalog import CartPole` is
the line you want on a lecture slide. But add a CI check, or it will drift again.

### 3.2 "Differentiate and `jit` through the same `f`" — 61% of the catalog cannot

I instantiated all 49 default-constructible catalog classes and compiled each
with both backends:

| Backend | Compiles |
| --- | --- |
| `numpy` | **49 / 49** |
| `jax` | **19 / 49** |

The 30 failures include `CartPole`, `KinematicBicycle`, `KinematicCar`,
`DynamicBicycle`, `TwoLinkManipulator`, `ThreeLinkManipulator3D`, `Drone2D`,
`Boat2D`, `Rocket`, `MountainCar`, `VanderPol`, `Lorenz`, `ThreeBodyProblem`,
`QuarterCarOnRoughTerrain`, and both propulsion cars.

Classified by source style:

| Style | Modules | Classes |
| --- | --- | --- |
| Dual-backend (`xp = array_module(…)`) | 7 | 20 |
| JAX-only (`require_jax_numpy()`) | 2 (`cartpole.py`, `jax_vehicles.py`) | 19 |
| NumPy-only (bare `np`) | 11 | 28 |

The consequence is exactly the thing DESIGN says minilink avoids: a student who
learns `KinematicBicycle` in the modelling lecture must switch to a *different
class* (`BicycleKin`, in a different module, with different ports) to do
trajectory optimization. **The sim stack / opt stack split was not avoided; it
was pushed down into the catalog.**

The good news: the fix is mechanical. I verified it on `KinematicBicycle` —
adding `xp = array_module(x, u)` and changing `np.` → `xp.` in `f` makes it
JAX-traceable with no other change:

```
jax f: [0.9553365  0.29552022 0.05016734]
```

> **Recommendation V-3.** Sweep the 11 NumPy-only catalog modules to the `xp`
> idiom (AGENTS "The `xp` idiom" already mandates it). Then delete the JAX twins
> that become redundant — starting with `JaxCartPole`, and re-examining how much
> of the 1 146-line `jax_vehicles.py` ladder survives once `steering.py` and
> `dynamic_bicycle.py` are traceable (§4.5). Add a contract test that asserts
> every catalog plant compiles under both backends; that test is what keeps the
> headline claim true.

### 3.3 "A diagram can be used anywhere a system can" — except the hybrid one

```
HybridDiagram is a System subclass?  False
has compile?  False        has f?  False
has __matmul__ / __add__?  False False
has n / inputs / outputs?  False False False
```

`HybridDiagram` is a plain `@dataclass`. So is `Computer` (also not a `System`).
The most advanced workflow in the library — sampled MPC on a continuous plant,
the flagship of the "control" pillar — produces an object that **cannot be
nested in a diagram, compiled, linearized, or composed**, and that hand-copies
`compute_trajectory` / `plot_trajectory` / `animate` / `plot_diagram` from
`SharedSystemFacades` (already noted in TODO §2).

It also breaks the operator grammar's type closure:

| Expression | Result type |
| --- | --- |
| `a + b`, `a >> b`, `ctl @ plant` | `DiagramSystem` (a `System`) ✅ |
| `block % dt` | `Computer` (**not** a `System`) |
| `Computer @ plant`, `mpc @ plant` | `HybridDiagram` (**not** a `System`) |

So `@` sometimes returns a `System` and sometimes does not, depending on the
left operand — and `%`, which sits in the same syntactic space, is not a
composition operator at all but a "build a runtime" operator.

This is the one finding I would call a genuine **structural** issue rather than
a cleanup. Options in §4.1.

### 3.4 "Models are stateless" — mostly true, with four exceptions

README: *"Systems represent equations and interfaces. They do not hide the
evolving simulation state internally."* That specific claim is **true and well
enforced** — `f`/`h`/port computes are pure, and the `params is None →
self.params` contract has **zero** violations of the banned `params or
self.params` idiom across 46 600 lines. That is real discipline.

But the object does carry mutable state beyond equations:

| Attribute | Kind |
| --- | --- |
| `params`, `x0`, `solver_info`, `skin`, `camera_*` | model defaults — fine, and documented |
| `self.traj`, `StepSystem.rollout`, `HybridDiagram.last_result` | **caches of simulation output stored on the model** |
| `_composition_entry` / `_composition_output` | hidden wiring bookkeeping written by the shortcut operators |
| `Computer.x` | genuinely stateful runtime (documented, correct) |

The caches are documented as conveniences and library code never reads them —
that's the right guard rail. But `sys.traj` reads like a model attribute, and
the naming is already inconsistent (`traj` vs `last_result` vs `rollout`), which
TODO §2 flags. Compare Drake, where *everything* mutable — time, state,
**parameters** — lives in a `Context` and the `System` is genuinely immutable.

Minilink's choice (params on the object, overridable per call) matches
python-control and is **better for teaching** — `plant.params["l"] = 5.0` is the
line you want in lecture. Keep it. Just stop calling it "stateless" without
qualification and pick one cache name.

### 3.5 "`graphical/` owns plots" — plotting has eight homes

DESIGN §3 assigns rendering to `graphical/` and §7 says *"Facades delegate to
`graphical/`."* In fact matplotlib code lives in:

`graphical/signals`, `graphical/phase_plane`, `graphical/diagrams`,
`graphical/animation`, `graphical/port_map` · `analysis/frequency.py` ·
`planning/spatial/plotting.py` (721 LOC) · `planning/search/plotting.py` (430) ·
`planning/search/live_plot.py` · `planning/policy_synthesis/plotting.py` (407) ·
`planning/trajectory_optimization/live_plot.py` · `control/mpc/controller.py`
(`init_debug_figure` / `update_debug_figure`) · `control/mpc/viz.py` ·
`blocks/sources.py` (`show_signal`, 80 lines of bespoke axes work, with its own
`TODO` saying it should be folded into `graphical`).

Domain plots living next to their domain is a defensible pattern. But then
`port_map.py` (the *control* law plotter) should not be the one exception living
in `graphical/`, and DESIGN should say what the rule actually is.

> **Recommendation V-4.** Write the rule down: `graphical/` owns *renderers and
> generic signal/diagram/animation plotting*; a tool band may own plots of its
> own result objects. Then move `port_map.py` next to `control/` (or accept it
> and say why), delete `blocks/sources.show_signal` (a `Source` is a static leaf
> — `compute_trajectory()` + `plot_trajectory()` already does exactly this), and
> move the MPC debug figure out of the controller class.

---

## 4. Foundational architecture

The load-bearing decisions, graded.

### 4.1 `System` / `DiagramSystem` — **right, keep**

Base shell with `n=0`, ports as first-class objects carrying dim/labels/units/
bounds/nominal, output ports declaring feedthrough `dependencies`, diagram
stacking subsystem state in insertion order, algebraic-loop detection by DFS
over port dependency edges, wiring validated at `connect()` time. `signals.py`
infers port dimension from whichever metadata you supplied and raises on
conflicts. This is a solid, conventional, well-executed framework core — richer
than python-control's signal naming, simpler than Drake's Context machinery.

The one thing to fix here is **`x0` staleness** (§4.2).

**The hybrid question.** Two coherent resolutions to §3.3:

- **(a) Promote.** Make `HybridDiagram` a `System` whose state is
  `[plant_x; computer_x]`, whose `f` is the plant flow with held inputs (well
  defined *between* ticks), and whose tick is a periodic discrete update handled
  by the simulator. This is the standard hybrid-automaton formulation and
  precisely what Drake does (`DeclarePeriodicDiscreteUpdate` on a system that
  owns both continuous and discrete state). It preserves DESIGN's rule that
  *leaf* `f` always means `dx` — the merge happens only at the composite. Cost:
  a real design cycle, event scheduling in `Simulator`, and it reopens a
  decision DESIGN froze deliberately.
- **(b) Rename and be honest.** Accept that hybrid is an *orchestration* concept
  like `Simulator`, not a diagram. Rename `HybridDiagram` → `HybridLoop` (or
  `HybridRuntime`), stop hand-copying `System` facades onto it, and change `%`
  from an operator to a method (`block.on_schedule(dt)`) so the operator algebra
  stays closed over `System`. Cost: a rename and a doc paragraph.

> **Recommendation A-1.** Do **(b) for v0.1** — it is a day of work, it removes
> a class of student confusion ("why can't I put my MPC loop inside a bigger
> diagram?"), and it makes the honest statement that the composition grammar is
> closed over `System`. Record **(a)** as the open v1.0 question. Do not do (a)
> before the teaching release.

### 4.2 `x0` is a stale snapshot while `params` is a live view — **fix**

```
plant.x0[0] = 1.5          # after wiring
diagram.x0                 # -> [0. 0.]      stale
diagram.refresh()
diagram.x0                 # -> [1.5 0.]
```

`DiagramSystem.params` is a live property assembled from subsystem references on
every access (excellent). `x0`, `n`, `state`, `state_index` are cached
attributes rebuilt only by `compute_state_properties()` / `refresh()`. Two
different aggregation semantics for the same kind of derived data, on the same
object.

Simulation is *correct* — `Simulator.__init__` calls `self.sys.refresh()` before
reading `sys.x0` — so this is latent, not a live bug. But it violates AGENTS
rule 7 ("Derived, not cached: computable quantities are read-only properties,
never stale cached attrs"), and `x0` is the single most-touched teaching knob in
the library. A student who prints `diagram.x0` sees a lie.

> **Recommendation A-2.** Make `DiagramSystem.x0` (and `n`, `state`) derived
> properties in the same shape as `params`. That also lets `refresh()` shrink to
> the thing it should be — a subsystem-level hook — and removes the "recompile
> after structural changes" footgun from the diagram layer.

### 4.3 The evaluator is doing the solver's job — **the biggest cleanup**

`core/compile/evaluators/` defines **88 methods**. Roughly forty of them are
integration helpers, and their names are a five-axis cross product:

`{rk4, euler} × {step, integrate} × {zoh, linear, ivp} × {fast, trace} × {bound, parametric}`

Referenced-from analysis across the whole repo (library, tests, examples,
benchmarks, docs):

| Category | Count |
| --- | --- |
| Methods defined on evaluator classes | 88 |
| **Never referenced outside the evaluators package** | **50** (28 of them public) |
| Referenced only from tests, never from library or examples | 10 |
| Carry essentially all traffic | `outputs` (324), `f` (226), `step` (52), `rollout` (21), `f_p` (17), `integrate_zoh` (12) |

DESIGN already concedes this by splitting the API into a "frozen subset" and a
"stable-internal" grid whose "names may still change before v1.0". That is a
docs-level workaround for a design that grew a dimension too many.

Worse, it duplicates a layer that already exists. `simulation/solvers/` has a
clean `SolverBackend` ABC with four backends — but `RK4SolverBackend.integrate`
is nine lines that call `evaluator.rk4_integrate_ivp(...)`. **The actual RK4 is
in the evaluator; the "solver backend" is a label.** This is exactly the
inversion the repo's own orchestrator/backend rule warns against: the base
backend should expose only the abstract operation, and orchestration/convenience
should sit above it — here, the *primitive* layer absorbed the integrators and
the *backend* layer became ceremony.

There is a legitimate reason for some of it: on JAX the rollout loop must be
inside `jit` (`lax.scan`), so it cannot live in a NumPy `for` loop upstairs. But
that argues for **one** traced-rollout primitive the solver composes, not forty
named variants mirrored on the NumPy side.

> **Recommendation A-3 (staged, safe).**
> 1. **Delete now:** the 24 dynamically-registered `_jit` alias methods (6 classes)
>    (`register_jit_aliases`). They exist only so that
>    `test_f_jit_alias_identity` can assert `f_jit is f`. They are aliases, which
>    AGENTS rule 10 bans outright ("Pre-1.0 no-alias rule"), and DESIGN documents
>    them as "optional". Zero call sites in library, examples, or benchmarks.
> 2. **Delete next:** the 28 unreferenced public integration methods
>    (`*_ivp_p`, `*_ivp_trace`, `*_ivp_trace_p`, `euler_*_trace*`,
>    `rk4_step_ivp*`, `rollout_p`, `step_block`, `f_scipy`, `f_ivp_scipy`, …).
>    Keep the frozen subset DESIGN already names.
> 3. **Then decide the layering.** Target: evaluators expose the *pure maps*
>    (`f`, `outputs`, `step`, plus `_p` and `_trace` variants) and **one**
>    scannable step primitive; `simulation/solvers/` owns every integrator,
>    including a JAX solver that builds its own `lax.scan` from `f_trace`. That
>    restores the orchestrator/backend split you asked for elsewhere in the repo
>    and collapses ~1 000 lines of mirrored NumPy/JAX integration code.
>
> Steps 1–2 are pure deletion behind the documented "stable-internal" caveat and
> can land this week. Step 3 is the one real refactor I would recommend before
> v1.0 — and it should wait until after the teaching release.

### 4.4 Two mechanical bases, two names for the same matrix — **unify or drop**

| | `MechanicalSystem` | `GeneralizedMechanicalSystem` |
| --- | --- | --- |
| Inertia | `H(q)` | `M(q)` |
| Velocity map | implicit `qdot = v` | explicit `N(q)` |
| State split | `x2q` / `q2x` | `x2qv` / `qv2x` |
| Extra ports | `q`, `dq` | none |
| Users | ~15 catalog classes + `Manipulator` (5 arms) + `JaxMechanicalSystem` | **2** (`Boat2D`, `Plane3D`) |

`GeneralizedMechanicalSystem` is strictly the more general formulation
(`MechanicalSystem` is it with `N = I`, `pos = dof`), yet they are siblings with
different names for the same physical quantity and different accessors. A
student reading two catalog plants sees `H` in one and `M` in the other for the
inertia matrix. `Boat2D` and `Plane3D` also silently lose the `q` / `dq` ports
that `closed_loop_qdq` and the robotic controllers rely on.

> **Recommendation A-4.** Pick one. Cheapest honest fix: make
> `MechanicalSystem` inherit from `GeneralizedMechanicalSystem` with `N = I`,
> and alias the matrix name once with a clear docstring note (`H` for
> manipulators, per Craig/Siciliano; `M` in vehicle/marine literature). If that
> is too much churn, the alternative is to delete
> `GeneralizedMechanicalSystem` and let its two users define `f` directly —
> 154 lines and one concept removed for two plants.

### 4.5 The vehicle family is three overlapping families — **consolidate**

| Module | LOC | Classes |
| --- | --- | --- |
| `vehicles/jax_vehicles.py` | 1 146 | 15 (9 ladder rungs + 6 `*Ports` twins) |
| `vehicles/steering.py` | 368 | 7 |
| `vehicles/dynamic_bicycle.py` | 436 | 2 |
| `vehicles/car_profile.py` | 707 | 2 (parameter envelopes) |
| `vehicles/propulsion.py`, `suspension.py`, `mountain_car.py` | 449 | 4 |
| **total** | **3 106** | **30** |

`vehicles/` alone is **6.7% of the library** and 30 of 49 catalog plants' worth
of surface. It contains near-duplicates across backends: `Holonomic` ↔
`HolonomicMobileRobot`, `BicycleKin` ↔ `KinematicBicycle`, `BicycleDyn` ↔
`DynamicBicycle`.

The six `*Ports` twins are pure boilerplate — each clears `self.inputs`, re-adds
named ports, and repacks them into the stacked `u` before delegating to
`super().f`:

```python
class BicycleDynPorts(BicycleDyn):
    def __init__(self):
        super().__init__(); self.inputs = {}
        self.add_input_port("w_rear", …); self.add_input_port("delta", …)
    def f(self, x, u, t=0.0, params=None):
        w_rear, delta = self.get_port_values_from_u(u, "w_rear", "delta")
        return super().f(x, jnp.array([w_rear[0], delta[0]]), t, params)
```

That is a **missing framework feature** wearing a class costume: "the same plant,
with `u` split into named sub-ports." Six classes exist because `System` has no
way to say that declaratively.

> **Recommendation A-5.** (i) Replace the `*Ports` twins with a constructor flag
> or a framework-level "split `u` into named ports" declaration — the plant math
> should never be subclassed to rename a port. (Note: `DynamicBicycle`'s named
> `w_rear`/`delta` ports are a deliberate prior decision and must be preserved
> — this recommendation is about *how* they are declared, not whether.)
> (ii) After the `xp` sweep (V-3), re-derive the ladder: several JAX rungs
> become redundant with the NumPy plants. (iii) Consider whether the full
> 9-rung fidelity ladder belongs in the shipped catalog at all, or in
> `examples/projects/` where the path-tracking research that motivated it lives.

### 4.6 Two geometry vocabularies with colliding names — **rename**

```python
from minilink.core.geometry import Sphere, Box                     # SDF solids, sdf(p) < 0 inside
from minilink.graphical.animation.primitives import Sphere, Box    # drawing primitives
```

Both are importable, both are public, both are core-adjacent, and they mean
completely different things. There is a third shape vocabulary in
`graphical/catalog/shapes.py` (`spring_between`, `link_pose_3d`, …) and a fourth
in `planning/spatial/collision.py` (`disc`, `car_outline`, `point_probe`). A car
therefore has a `car_skin_2d` (drawing), a `car_outline` (collision), and no
shared source of truth for its shape.

This is a real foundational duplication — and the naming collision is the part
that will actually bite a student in a notebook.

> **Recommendation A-6.** At minimum, rename so no two importable public types
> share a name: `graphical.animation.primitives.Sphere/Box` → `SphereGlyph`/
> `BoxGlyph` (or the SDF side → `SphereSolid`/`BoxSolid`). The deeper unification
> (one shape → both an SDF and a glyph) is a v1.0 project, not a v0.1 one; note
> it and move on.

Related naming collisions worth one documented paragraph rather than a refactor,
since they are all inherited from the literature:

| Symbol | Meaning A | Meaning B |
| --- | --- | --- |
| `h` | `System.h` output map | `CostFunction.h` terminal cost |
| `g` | `CostFunction.g` running cost | `MechanicalSystem.g(q)` gravity; `MathematicalProgram` `g ≥ 0` |
| `C` | `MechanicalSystem.C` Coriolis | `StateSpaceSystem.C` output matrix |
| `B` | `MechanicalSystem.B` actuator map | `StateSpaceSystem.B` input matrix |

`CostFunction.h` is the one I would actually rename (it already has a
`terminal_cost()` wrapper; the hook could be `phi` or `terminal`) — the others
are unavoidable and just need a "notation collisions" note in DESIGN.

### 4.7 Stable and provisional share one namespace — **adopt the Archimedes pattern**

Today the tier boundary exists only as a table in `README.md`. Everything —
frozen `core/` and TRL-1 `symbolic/` alike — is importable from the same
namespace with no signal at the import site. A student who finds
`minilink.planning.spatial.overlays` in autocomplete has no way to know it is
0%-covered dead code.

Archimedes (shipping into the same niche, same 1.0 timeline) solves this by
putting unstable features in a separate `experimental` module with no stability
guarantee, and defining 1.0 as "6+ months without breaking API changes in core."

> **Recommendation A-7.** You do not need to move packages. Two cheaper steps
> get most of the value: (i) a one-line tier banner at the top of every
> provisional module's docstring, generated from a single registry so it cannot
> drift; (ii) a contract test that asserts the *teaching surface* — root prelude
> + band facades — contains **only** stable-tier names. That makes the promise
> to students machine-checked instead of prose.

---

## 5. Behavioural defects found by running the code

These are small, concrete, and all of them are on the first-hour student path.

### 5.1 Every default simulation returns 100 001 samples — **fix before release**

```
>>> (ImpedanceController() @ Pendulum()).compute_trajectory(tf=10.0)
n_pts=100001, dt=0.0001 (auto dt)
solver: 'scipy'   integration_stats: {'nfev': 200, …}
```

The output grid comes from `solver_info["smallest_time_constant"] * 0.1`. That
constant is hard-coded to `0.001` on `System` and **overridden by exactly three
places in the entire library** (`oscillators.py`, `three_body.py`,
`ancf_tire_jax.py`). So essentially every plant reports on a 0.1 ms grid.

For SciPy this is *pure output sampling* — the integrator took 200 RHS
evaluations and then densely interpolated 100 001 points. Cost: 0.22 s to
simulate, 0.79 s to plot, 0.56 s to reconstruct internal signals, for a
2-state pendulum.

On the JAX path it is worse, because the same constant feeds an unrelated
policy: `select_solver` picks `rk4_fixedsteps` when `n_pts >= 10_000`. So:

```
>>> Pendulum().compute_trajectory(tf=10.0, compile_backend="jax")
solver: 'rk4_fixedsteps' (auto-selected)
integration_stats: {'nfev': 400000, …}
```

**400 000 RHS evaluations instead of 200**, for a smooth pendulum, chosen
silently by a default nobody set.

The root cause is a conflation the peers all avoid: python-control takes an
explicit `timepts`; Drake publishes at a chosen period independent of the
integrator. Here, *reporting resolution* and *integration step* are the same
number.

> **Recommendation B-1.** Default the **output grid** to a fixed, human-scaled
> count (`n_steps=500` or so) for adaptive solvers, and keep `dt` as the
> fixed-step-solver knob. Decouple `select_solver`'s auto-RK4 trigger from
> `n_pts` — base it on the requested solver or on `discontinuous_behavior`, not
> on how many points the user wants plotted. Then give `MechanicalSystem` a
> `smallest_time_constant` derived from the plant's own scales so the fixed-step
> path is sane too. This is one of the highest-value hours available.

### 5.2 The README's own custom-plant example cannot be composed

The "Models that read like the textbook" example builds:

```python
super().__init__(n=2, input_dim=1, expose_state=True)   # no output_dim
```

which yields `p == 0` and no `y` port. Simulation works. But the headline
composition feature does not:

```
>>> ProportionalController() @ MassSpringDamper()
ValueError: Cannot wire closed-loop feedback: ctl declares measurement 'y'
dim 1, but sys 'y' output has dim None
```

Two problems: the README teaches a plant shape that cannot be used with `@`, and
the error message leaks `None` instead of saying *"plant has no `y` output port;
pass `output_dim=…` or wire ports explicitly."*

> **Recommendation B-2.** Add `output_dim=2` to the README example (or drop
> `expose_state` and show `output_dim`), and fix
> `_feedback_mismatch_message` to name the missing port rather than printing a
> `None` dimension. Ten minutes; it is the first thing a student will hit.

### 5.3 Library facades print by default

`compute_trajectory(..., verbose=True)` is the default, so a bare
`diagram.compute_trajectory(tf=10)` emits a 20-line banner. AGENTS rule 9 says
*"Libraries are silent: no `print` except explicit `verbose=`."* The flag is
explicit, so this is arguably within the letter of the rule — but the default
value inverts its intent, and 180 `print` calls sit outside `__main__` guards
(52 in `sim_reporting.py`, 42 in `planning/…/planner.py`, 18 in `optimizer.py`).

For teaching, a short one-line report is genuinely useful. A 20-line framed
panel on every call is not.

> **Recommendation B-3.** Keep `verbose=True` but make the default report **one
> line** (`solver, dt, n_pts, wall time`), with the framed panel behind
> `verbose="full"` or `verbose=2`. Also unify the flag name — today it is
> `verbose` / `disp` / `solve_disp` / `step_disp` depending on the band (already
> in TODO §2).

### 5.4 `nbstripout` has been a no-op since the July restructure

```yaml
files: ^examples/notebooks/.*\.ipynb$      # this directory no longer exists
```

Notebooks live under `examples/learn/`, `examples/tooling/notebooks/`, and
`examples/projects/`. AGENTS.md states outputs are stripped by pre-commit; they
are not. One notebook (`examples/learn/teaching/mpc.ipynb`, currently modified)
already carries stored outputs.

> **Recommendation B-4.** `files: ^examples/.*\.ipynb$`. One line.

### 5.5 The package is not on PyPI

`pypi.org/pypi/minilink/json` → **404**. Install is `git clone` + `PYTHONPATH`,
and the top troubleshooting entry in `install.md` is `No module named 'minilink'`
— i.e. the install method's own failure mode is the most common support request.

`pyproject.toml` is already fully configured (hatchling + hatch-vcs, wheel
targets, extras). The name is currently free.

> **Recommendation B-5.** Publish `0.1.0rc1` to PyPI before the cohort starts,
> if only to claim the name. `pip install minilink` in a Colab cell is worth
> more to a student release than any feature on the ROADMAP priority list.

---

## 6. Health metrics

### 6.1 Size

| Component | LOC | Files |
| --- | ---: | ---: |
| `minilink/` (library) | **46 600** | 224 |
| `tests/` | 16 155 | 49 |
| `benchmarks/` | 7 598 | 39 |
| `examples/*.py` | 12 311 | 100 |
| `examples/*.ipynb` (code lines) | 4 913 | 24 |
| **Python total** | **~87 600** | **436** |
| Markdown docs | 346 KB / ~41 600 words | 22 |

| Public API | Count |
| --- | ---: |
| Public classes | **320** |
| Public module-level functions | **269** |
| Root prelude `__all__` | 10 |
| `catalog` / `control` / `analysis` / `blocks` facades | 50 / 13 / 10 / 18 |

**Reference point.** python-control 0.10.2 (multi-institution team, 15 years):
34 807 LOC library, 27 255 LOC tests — a test/library ratio of **0.78**.
Minilink: 46 600 / 16 155 = **0.35**. Minilink's library is 34% larger with 45%
of the relative test weight, maintained by one person.

That is the central fact of this review. It is not an argument that the code is
bad; it is an argument that **the next phase should be subtraction**.

### 6.2 Quality gates — all green

- `pytest`: **884 passed, 3 skipped**, exit 0.
- Full demo sweep (`run_all_demos.py --timeout 60`, headless):
  **60 passed, 0 failed, 3 skipped** (the 3 are interactive realtime sessions,
  correctly excluded). Every runnable script under `examples/demos/` works
  today — including both `c_export` demos, which pytest never touches.
- `ruff check .`: clean. `ruff format --check .`: 420 files formatted.
- Compile-vs-reference parity: tested (`test_compile.py` asserts compiled `f`
  matches the recursive `diagram.f()` reference, and `f_trace`/`rk4_step_trace`/
  `integrate_zoh_trace` parity). This is the right invariant to defend and it is
  defended.
- Examples: **zero import drift** across 124 files. Every `from minilink…import`
  resolves. (Two apparent failures were submodule-import false positives.)
- All `:mod:` / `:class:` / `:meth:` references in the markdown docs resolve
  against the AST. Only 5 `TODO`/`FIXME` markers in 46 600 lines.

### 6.3 Coverage — 77%, with three modules at zero

| Module | Stmts | Cover |
| --- | ---: | ---: |
| `interfaces/c_export.py` | 295 | **0%** |
| `symbolic/mechanics/utils.py` | 31 | **0%** |
| `planning/spatial/overlays.py` | 27 | **0%** |
| `simulation/realtime/pygame_input.py` | 121 | 12% |
| `graphical/…/pygame_renderer.py` | 211 | 23% |
| `graphical/…/meshcat_renderer.py` | 303 | 26% |
| `graphical/…/matplotlib_renderer.py` | 285 | 39% |
| `dynamics/catalog/aerial/plane.py` | 307 | 42% |
| `planning/search/plotting.py` | 188 | 47% |
| **TOTAL** | **20 458** | **77%** |

Renderers being lightly covered is normal. `matplotlib_renderer.py` at 39% is
less normal — it is the renderer every student sees.

### 6.4 Dead and unreachable code

- **75** public module-level names are never referenced outside their own file.
  (Some are legitimate implementation classes reached through a factory —
  `DiscretizedRK4DynamicSystem` via `discretize()` is a false positive. The list
  needs judgement, not blind deletion.) High-confidence subset below.
- **28** public evaluator methods + **12** `_jit` aliases: never called anywhere
  (§4.3).
- `interfaces/c_export.py`: 461 lines, **0% covered by pytest** and **not in the
  flagship demo manifest**, so nothing in CI exercises it. Its two demo scripts
  *do* pass in the local full sweep, so the code works — the problem is
  governance, not correctness: a real experimental JAX→C transpiler is sitting
  in a band DESIGN calls a placeholder, with no TRL row, riding the v0.1 freeze
  implicitly. TODO §2 already raises this.
- `planning/spatial/overlays.py`: imported by nothing.
- `symbolic/mechanics/utils.py`: six inertia helpers, no callers.

### 6.5 Where the mass actually is

| Package | LOC | Share | Comment |
| --- | ---: | ---: | --- |
| `core/` | 9 454 | 20% | 1 608 of it is `jax_evaluators.py` alone |
| `dynamics/` | 9 013 | 19% | `vehicles/` is 3 106 of it (§4.5) |
| `planning/` | 8 811 | 19% | includes ~1 550 LOC of plotting in three modules |
| `graphical/` | 7 879 | 17% | four renderers (2 309 LOC) |
| `simulation/` | 3 266 | 7% | |
| `control/` | 2 631 | 6% | 1 349 of it is `mpc/` |
| everything else | 5 546 | 12% | `analysis/` 1 246, `optimization/` 1 244, `symbolic/` 1 082, `blocks/` 891, `interfaces/` 819, `catalog/` 156 |

Two observations. **`blocks/` is 891 LOC and 15 blocks** — thin for a
block-diagram teaching library (bdsim ships ~60). There is no `Sine`, `Ramp`,
`Chirp`, `Delay`, `Derivative`, `Switch`, `Product`, or `RateLimiter`. For a
course that draws block diagrams on the board, a `Sine` source is more
load-bearing than a ninth bicycle fidelity rung.

**Three separate "compile a callable graph to a backend" subsystems** exist:
`core/compile/evaluators/`, `optimization/evaluators/`, and
`planning/trajectory_optimization/parametric_evaluator.py`. I measured the last
two: **54% of the smaller file's lines are identical** to the other. TODO §2
already flags the placement; the duplication number makes the case.

### 6.6 Style rules: stated vs enforced

| AGENTS rule | Status |
| --- | --- |
| `params is None`, never `params or self.params` | ✅ **0** violations |
| Native-array equation paths | ✅ broadly held (the JAX gap in §3.2 is a *`np` vs `xp`* issue, not a conversion-in-equations issue) |
| Libraries are silent | ⚠️ default `verbose=True` prints a 20-line panel (§5.3) |
| Derived, not cached | ⚠️ `x0`/`n`/`state` on diagrams are cached (§4.2) |
| Pre-1.0 no-alias rule | ❌ `register_jit_aliases` creates 24 alias methods whose only consumer is their own test |
| No leading-underscore method names on system/facade/simulator classes | ❌ **43** on true `System` subclasses (`_ctl_regulation`, `_u_in`×7, `_trig`, `_rnea`, `_aba`, `_compute_u_ff`, …); 145 if planners/evaluators are included |
| `get_port_values_from_u` as the port accessor | ⚠️ **13** uses library-wide; the entire `control/` band uses raw index arithmetic instead (`r = u[:ref_dim]; pos = u[ref_dim:ref_dim+n]`) |
| Docs are contract | ⚠️ mostly excellent, with the drift in §6.7 |

The underscore rule is the one I would either **enforce or delete**. 43
violations on the exact classes the rule names means the rule is not real. My
honest read: the rule as written is too broad — `UR5Manipulator._rnea` is a
genuine internal, and a leading underscore is the standard Python signal.
Consider narrowing it to "no underscore on *facade or contract* methods" and
letting equation-internal helpers keep theirs.

The `get_port_values_from_u` gap matters more than it looks. The framework's
central abstraction is **named ports**, and its own flagship library band does
not use the named-port accessor — it recomputes offsets by hand in every control
law. `r, pos, rate = self.get_port_values_from_u(u, "r", "pos", "rate")` is both
safer and more textbook-readable than three slice expressions with `ref_dim + n`
arithmetic.

### 6.7 Documentation health

346 KB of markdown, ~41 600 words, across four authoritative documents
(README 18 KB, DESIGN 57 KB, ROADMAP 8 KB, AGENTS 13 KB) and 13 plan docs
(200 KB+, of which `pyro-port-remaining.md` is 50 KB and
`test-benchmark-consolidation.md` is 48 KB and marked "Partial").

The discipline is genuinely high — every Sphinx-style cross-reference resolves,
the doc map in AGENTS.md is real, and the plans index states the rules. But the
volume is itself a maintenance liability, and drift has started:

- `docs/plans/pyro-port-remaining.md` marks
  `MechanicalSystemWithPositionInputs` and
  `GeneralizedMechanicalSystemWithPositionInputs` as **Done** with a minilink
  home. Neither class exists — and DESIGN explicitly rejects them ("no
  `WithPositionInputs` inheritance branches"). Same doc references
  `minilink/analysis/analysis_linearize.py`, `minilink/control/linear.py`,
  `minilink/control/pid.py` — none exist.
- `DESIGN.md:223–224` contains a duplicated line.
- `control-block-contract.md` (32 KB) is marked "Landed — awaiting DESIGN
  architectural review"; the plans README says landed contracts move into DESIGN
  and **finished plan docs get deleted**.

> **Recommendation D-1.** Before v0.1: (i) delete or archive landed plan docs
> (`control-block-contract.md` at minimum); (ii) re-audit the "Done" rows in
> `pyro-port-remaining.md` against the AST — a parity audit that is wrong is
> worse than no parity audit, because it drives ROADMAP §3 criterion 1;
> (iii) consider whether a 50 KB parity matrix is still the right artefact, or
> whether the remaining work is now small enough to be 15 lines in TODO.md.

### 6.8 CI reaches 10 of 100 example scripts

`flagship_manifest.json` lists **10** demos, all under `examples/demos/`.
`run_all_demos.py` sweeps `examples/demos/` only (63 files) and is not in CI —
though when I ran it manually, all 60 runnable scripts passed, so the sweep is
worth promoting rather than fixing. `examples/projects/` — **7 173 LOC across 33
files**, the largest example bucket — has no automated check at all beyond
notebook smoke.

Those projects (`pathtracking/mpc_v1`, `bicycle_los`, `bicycle_los_v2`,
`car_trajopt`, …) are research scenarios, not teaching material. They will rot
silently as the API moves, and every one that rots is a support ticket or a
misleading example for a student who wanders into it.

> **Recommendation D-2.** Either put `examples/projects/` under the demo sweep
> (nightly, not per-PR) or move it out of the teaching repo into a
> `minilink-experiments` sibling. Given the v0.1 goal, I would move it out: it
> is 7 200 lines of maintenance you do not need in front of students.

---

## 7. What is genuinely good — keep these

Worth stating explicitly, because a review of this shape reads harsher than the
codebase deserves.

1. **The `System` contract and the port model.** Ports with dim inference,
   labels, units, bounds, nominal values, and declared feedthrough dependencies
   are richer than python-control's signal naming and simpler than Drake's.
   Algebraic-loop detection at wiring time is the right call.
2. **The composition grammar.** `ctl @ plant`, `src >> plant`, `.autowire()` is
   the nicest teaching syntax in this space — measurably better than
   `ct.interconnect(...)` or `DiagramBuilder.AddSystem(...)`. The conservative
   autowire rules (never insert a `Mux`, never overwrite, fail loudly on
   ambiguity under `strict`) are well judged.
3. **`Trajectory`.** Frozen dataclass, validated at construction, `MappingProxy`
   signals, `with_signal` returning a new object. Exactly right.
4. **The params contract.** `params is None → self.params`, nested by subsystem
   id, live-view property, JAX pytree in the parametric tier, zero violations of
   the banned idiom. This is the kind of invariant most codebases state and
   then break.
5. **The control band's readability.** `output.py`, `state.py`, `lqr.py` read
   like a textbook — short, no ceremony, `__main__` hello-worlds that actually
   demonstrate something. This is the product's differentiator and it is real.
6. **Compile-vs-reference parity as a defended invariant.** Having a slow
   recursive reference path and testing the fast path against it is the single
   most valuable test in the repo.
7. **The `feedback_profile` declaration.** Read-only context that unlocks `@`
   resolution, `plot_control_law`, and `PolicyEvaluator` without changing how a
   block computes, resolved by duck typing rather than `isinstance`. Elegant,
   and the right amount of magic.
8. **Discipline artefacts.** 5 TODOs in 46 600 lines, ruff clean, all doc
   cross-references resolving, zero import drift in 124 example files, an honest
   "Discontinuous closed loops — known issues" section in DESIGN that documents
   a real numerical trap instead of hiding it. That last one especially — most
   libraries would not write it down.

---

## 8. Prioritized plan

### P0 — before any student sees it (days)

| # | Action | Why | Effort |
| --- | --- | --- | --- |
| 1 | Fix the default output grid; decouple auto-RK4 from `n_pts` (§5.1) | 100 001 samples / 400 000 RHS evals per default run | hours |
| 2 | Fix the README custom-plant example + the `@` "dim None" message (§5.2) | first-hour failure straight from the README | minutes |
| 3 | **Decide** the import layering; then enforce it in every example + a CI check (§3.1) | the documented teaching API is used twice in 554 imports | 1 day |
| 4 | Publish `0.1.0rc1` to PyPI (§5.5) | `pip install minilink` in a Colab cell; claim the name | hours |
| 5 | Fix the `nbstripout` glob (§5.4) | hook has been a no-op since July | minutes |
| 6 | One-line default simulation report (§5.3) | 20-line banner on every call | hours |

### P1 — subtraction pass (2–3 weeks)

| # | Action | Removes | Ref |
| --- | --- | --- | --- |
| 7 | Delete the 24 `_jit` aliases and the 28 unreferenced evaluator methods | ~400 LOC + a banned pattern | §4.3 |
| 8 | `xp` sweep of the 11 NumPy-only catalog modules + a both-backends contract test | makes the headline claim true; unlocks deleting JAX twins | §3.2 |
| 9 | Delete `planning/spatial/overlays.py` + `symbolic/mechanics/utils.py` (dead, 0% covered); give `interfaces/c_export.py` an honest TRL row **and** a CI entry, or quarantine it | ~60 LOC deleted; 461 LOC stop riding the freeze implicitly | §6.4 |
| 9b | Promote `run_all_demos.py` to a nightly CI job | it already passes 60/60; CI only checks 10 | §6.8 |
| 10 | Decide the fate of `symbolic/` (1 082 LOC) and `dynamics/engines/` (864 LOC) | TRL-1 code carrying tests, benchmarks, and 4 sandbox demos | §6.5 |
| 11 | Rename the colliding geometry types | `Box`/`Sphere` mean two things | §4.6 |
| 12 | Make `DiagramSystem.x0`/`n`/`state` derived properties | removes a latent teaching trap and a rule violation | §4.2 |
| 13 | Move `examples/projects/` out, or into a nightly sweep | 7 173 unverified LOC | §6.8 |
| 14 | Adopt `get_port_values_from_u` across `control/` | the framework's own abstraction, unused by its flagship band | §6.6 |
| 15 | Rename `HybridDiagram` → `HybridLoop`; make `%` a method | closes the operator grammar over `System` | §4.1 |

### P2 — decisions to make, not code to write

| # | Question | My recommendation |
| --- | --- | --- |
| 16 | Frequency/MIMO analysis (ROADMAP priority **1**) | **Don't build it.** Add a ~50-line `LTISystem ↔ control.ss()` bridge and point courses at python-control for Nyquist/margin/rlocus. DESIGN already claims "Interop" with python-control; delivering that claim is cheaper than reimplementing `margin()` and strictly better for students, who should learn the standard tool. This deletes a whole ROADMAP priority. |
| 17 | Vehicle family consolidation (§4.5) | Collapse after the `xp` sweep; move the fidelity ladder to `projects/` unless a course uses it |
| 18 | Two mechanical bases (§4.4) | Unify or drop `GeneralizedMechanicalSystem` (2 users) |
| 19 | Evaluator/solver layering (§4.3 step 3) | Real refactor. **After** the teaching release |
| 20 | Hybrid as a `System` (§4.1a) | The open v1.0 question. Not now |
| 21 | DP's three backends (`loop`/`numpy`/`jax`, ~1 400 LOC, Bellman backup written three times) | Move `loop` into a teaching notebook where a visible double loop is *more* pedagogically valuable than a library backend; keep `numpy` + `jax` |
| 22 | Doc consolidation (§6.7) | Delete landed plan docs; re-audit the pyro parity "Done" rows |
| 23 | Blocks band depth (§6.5) | Add `Sine`/`Ramp`/`Chirp`/`Delay`/`Switch` before adding another plant |

### What I would *not* do before v0.1

- The evaluator/solver refactor (large blast radius, no student-visible benefit).
- Promoting `HybridDiagram` to a `System` (real design cycle).
- Estimation (Luenberger/Kalman) and `identification/fitting.py` — ROADMAP
  priority 2. Nice to have; nothing in a first course breaks without them, and
  every new module widens a surface that is already too wide.
- Any new catalog plants.

---

## 9. Open questions only you can answer

1. **Which course is v0.1 actually for?** The answer changes everything above.
   A modelling/simulation course needs `blocks/` depth and rock-solid
   plot/animate. A nonlinear-control course needs the control band and
   analysis. An optimal-control course needs trajopt + DP. A robotics course
   needs manipulators + impedance. Right now the library is sized for all four,
   and the release criteria in ROADMAP §3 ("representative closed-loop demo per
   major plant family") reflect that. **Picking one course as the v0.1 target
   would let you defer roughly half the ROADMAP.**
2. **Is pyro parity still the right north star?** ROADMAP §1 makes it the
   milestone, and the 50 KB audit tracks 104 library symbols and 195 demos. But
   parity with a predecessor is a *migration* goal, not a *product* goal, and
   35 of those demos are already marked Drop. If the students are new, parity
   buys nothing; if you are migrating your own course materials, it buys
   exactly as much as the materials you actually reuse. Consider replacing
   "pyro parity" with "the N demos I will use in GRO### this semester."
3. **Do you want the JAX path to be a *feature* or the *foundation*?** Today it
   is a feature that 39% of the catalog supports. If it is the foundation
   (which the "distinct edge" claim implies), then the both-backends contract
   test in P1-8 becomes non-negotiable and the NumPy-only plants are bugs. If it
   is a feature, say so in README and stop the twins from multiplying.
4. **Who maintains this in 12 months?** 87 600 lines of Python and 41 600 words
   of prose is a team-sized artefact. Every recommendation above is really the
   same recommendation: shrink the surface you have promised to keep working, so
   the parts students touch can be excellent.

---

## Appendix A — reproducing the measurements

All from repo root with `PYTHONPATH=.` and the `minilink` conda env.

```bash
# suite + lint
MPLBACKEND=Agg python -m pytest -q
ruff check . && ruff format --check .

# full demo sweep  (60 passed, 0 failed, 3 skipped)
MPLBACKEND=Agg SDL_VIDEODRIVER=dummy \
  python tests/demo_checks/run_all_demos.py --timeout 60 --continue-on-error

# coverage (installed into a scratch dir, not the project env)
python -m pip install --target /tmp/pylibs coverage
PYTHONPATH=.:/tmp/pylibs MPLBACKEND=Agg python -m coverage run --source=minilink -m pytest -q
PYTHONPATH=.:/tmp/pylibs python -m coverage report --sort=cover | head -30

# catalog backend matrix  (49 numpy / 19 jax)
python - <<'PY'
import inspect, warnings; warnings.filterwarnings("ignore")
import minilink.catalog as C
ok = {"numpy": [], "jax": []}; fail = {"numpy": [], "jax": []}
for n in sorted(C.__all__):
    cls = getattr(C, n)
    if not inspect.isclass(cls): continue
    try: s = cls()
    except Exception: continue
    for b in ("numpy", "jax"):
        try: s.compile(backend=b); ok[b].append(n)
        except Exception: fail[b].append(n)
print({b: (len(ok[b]), len(fail[b])) for b in ok}); print(fail["jax"])
PY

# example import-layer census  (1 root / 1 band / 314 module / 239 deep)
python - <<'PY'
import re, json, pathlib, collections
by = collections.Counter()
def scan(t):
    for m in re.finditer(r"from (minilink[\w.]*) import", t):
        d = len(m.group(1).split("."))
        by["root" if d == 1 else "band" if d == 2 else "module" if d == 3 else "deep"] += 1
for p in pathlib.Path("examples").rglob("*.py"): scan(p.read_text(errors="ignore"))
for p in pathlib.Path("examples").rglob("*.ipynb"):
    j = json.loads(p.read_text())
    for c in j.get("cells", []):
        if c.get("cell_type") == "code": scan("".join(c.get("source", [])))
print(by)
PY

# default grid / solver selection
python -c "
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
Pendulum().compute_trajectory(tf=10.0)                          # n_pts=100001, scipy, nfev=200
Pendulum().compute_trajectory(tf=10.0, compile_backend='jax')   # rk4_fixedsteps, nfev=400000
"

# README custom-plant example is not composable
python -c "
import numpy as np
from minilink import DynamicSystem
from minilink.control.output import ProportionalController
class MSD(DynamicSystem):
    def __init__(self): super().__init__(n=2, input_dim=1, expose_state=True)
    def f(self, x, u, t=0, params=None): return np.array([x[1], u[0]-4*x[0]-0.3*x[1]])
print(ProportionalController() @ MSD())
"

# hybrid is outside the type system
python -c "
from minilink.core.system import System
from minilink.core.hybrid_diagram import HybridDiagram
from minilink.simulation.computer import Computer
print(issubclass(HybridDiagram, System), issubclass(Computer, System))
"

# nbstripout hook matches nothing
grep -n 'files:' .pre-commit-config.yaml && ls examples/notebooks
```

The evaluator-usage census, the dead-public-name scan, the style-rule AST
audits, and the doc/code drift check are longer scripts; they are straight AST
walks over `minilink/` cross-referenced against text search over `minilink/`,
`tests/`, `examples/`, `benchmarks/`, and `docs/`, and are described inline in
§4.3, §6.4, and §6.6.
