# Minilink — second opinion, combined verdict, and an agent-sized roadmap

Independent review (Fable 5.1) of branch `dev-alex` @ `5c41f83`, 2026-09-05,
written after — but not derived from — the Opus 5 review in
[`2026-09-05-architecture-review.md`](2026-09-05-architecture-review.md).

Part A is my own analysis, built from probes the first review did not run.
Part B is a critical reading of the Opus report: what it got right, where I
weight things differently, what it missed. Part C is the combined verdict.
Part D is the roadmap, cut into steps a coding agent can finish in about an
hour each, with "done when" criteria. Nothing was applied to the code.

---

## Part A — Independent analysis

### A.0 What I did differently

The first review measured the *static* shape of the codebase (API census,
coverage, dead code, import style). I spent my time on the *dynamic* behaviour
a student or a researcher actually meets, and on the vision as you restated it
— pyro successor, JAX-compiled multilevel API, textbook-readable teaching
notebooks, and a research substrate for you and your graduate students.

| Probe | Why |
| --- | --- |
| Your site, Scholar, and pyro's `system.py` | To review against the actual courses (GRO860 optimal control & RL, GMC714 robot modelling & control, GRO640) and the actual research (Buckingham-π / dimensionless policies, value iteration, low-adhesion vehicles) |
| Fresh `venv` with **only** `numpy scipy matplotlib` | Is the "core needs nothing else" claim true end to end? |
| `vmap` / `grad` / batched rollouts through compiled evaluators | Is the JAX research foundation real, or a demo? |
| Trajopt on the canonical problems, all backends, with and without float64 | Does the optimize pillar work by default? |
| Deliberately wrong student code (bad `f` shape, forgotten `super().__init__`, unconnected feedback, wrong port direction) | Silent failure is the enemy of teaching |
| NumPy-path timings, JAX compile/retrace timings | What does the Basic tier feel like? |
| Notebook curriculum, `Sys2Gym` bridge, animation pipeline, planning/optimization contracts, branch hygiene, Sphinx site | The parts the first review skimmed |

### A.1 The good news first — three vision claims I could verify are true

**1. The Basic tier is real.** In a fresh virtual environment containing only
`numpy 2.5.2`, `scipy 1.18.1`, `matplotlib 3.11.1` (no graphviz, no JAX):
closed-loop simulation, `plot_trajectory`, `plot_phase_plane`, `animate`,
`linearize` + `lqr`, and value iteration all run. `plot_diagram` and the
notebook `_repr_svg_` degrade silently instead of crashing. This is exactly
the property pyro had and exactly what a Colab-first teaching library needs.
Protect it with a test (D-S11).

**2. The JAX research foundation is real and fast.** Through one compiled
closed-loop diagram, with no library changes:

| Operation | Result |
| --- | --- |
| `jax.vmap` over 1 000 initial states via `f_trace` | works, `(1000, 2)` |
| `jax.vmap` over 100 values of `params["sys"]["m"]` via `f_trace_p` | works, `(100, 2)` |
| `jax.grad` of a loss w.r.t. a physical parameter | works |
| batched rollout, 50 initial states × 1 000 RK4 steps, via `rk4_integrate_ivp_trace` | works, `(50, 1001, 2)` |
| jitted 10 000-step RK4 rollout | 74 ms first call, **13.5 ms** thereafter |
| compile + first call of a second identical diagram | 29 ms (retrace is cheap) |

This is the substrate your dimensionless-policy research needs — vmapping the
same `f` over a parameter family is the Buckingham-π experiment in one line.
It works today at the evaluator level. It just has no facade (A.7).

**3. The composition grammar really is the teaching win.** `lqr @ plant`
resolves the state-feedback profile and wires `x ↔ x` automatically, so the
`closed_loop()` helper in `pendulum_swing_up_vi_vs_lqr.ipynb` that wires the
diagram by hand is ceremony the library already removes. The two-pass
parameter pedagogy in `00_core.ipynb` (plain locals first, `params` dict
second) is genuinely good teaching design.

### A.2 Silent failures on the first-hour path — the most important thing I found

A teaching library's cardinal sin is accepting wrong code quietly. I tried the
four mistakes every student makes in their first hour:

| Student mistake | What minilink does today |
| --- | --- |
| `f` returns shape `(1,)` for a 2-state system | **Nothing.** `scipy`, `euler`, and `rk4_fixedsteps` all run to completion and return garbage (`x_final = [4.44, 5.44]` for a system that should decay). `compile(backend="numpy")` does not check either. The JAX path raises — but with a message about "in-place array mutation", which is not the problem. |
| `h` returns the wrong shape | **Nothing**, through simulation and plotting. |
| Wires a controller but forgets the feedback connection | **Nothing.** The controller runs on its port's nominal value; the plant runs open-loop; no warning. |
| Forgets `super().__init__()` in a subclass | `AttributeError: 'NoSuper' object has no attribute 'solver_info'` |

The wiring layer, by contrast, is excellent: dimension mismatches and wrong
port direction fail at `connect()` time with messages that name the available
ports. The *equation* layer just has no equivalent check. One probe call of
`f` and each port compute at `compile()` time — `x0`, nominal `u`, `t=0`,
compare `.shape` to the declaration — closes the first two rows on both
backends in an hour (D-S02). A targeted warning for controllers whose declared
measurement port is unconnected closes the third (D-S03). A two-line guard
closes the fourth (D-S06).

I rate this above everything in the first review except the default time
grid, because it is the difference between a library that teaches and a
library that has to be taught around.

### A.3 The optimize pillar is float32 by default, and that is why it "fails"

On the canonical problems, with the library's own defaults:

| Problem | Backend | Optimizer | `success` | Note |
| --- | --- | --- | --- | --- |
| pendulum swing-up, feasible (`|u| ≤ 20`, `tf = 3`) | jax | SLSQP | **False** | "Iteration limit reached" — yet the plan re-simulates to within 0.007 of the goal |
| same | numpy | SLSQP | True | |
| same | jax | IPOPT | **False** | "Restoration phase failed" |
| cart-pole (the `trajopt_cartpole_collocation_jax.py` problem) | jax | SLSQP | **False** | |
| same, after `configure_jax(enable_x64=True)` | jax | SLSQP | **True** | 0.9 s |
| pendulum, after `enable_x64=True` | jax | SLSQP / IPOPT | **True / True** | 0.3 s |

The JAX transcriptions run in float32 unless the *caller* enables x64. The DP
planner enables it for itself (`dp.py`, `discretizer.py` call
`configure_jax(enable_x64=True)`); the trajectory-optimization planner and
the `Optimizer` do not. The demo works only because the demo script calls
`configure_jax` — as do 34 other example and test files. A student who builds
their own problem from the intro notebook gets `success=False` and a working
plan, and has no way to know which to believe.

Two fixes, both small: make float64 the library policy at JAX-evaluator
construction (D-S05), and make `SolveMetadata.success` mean *"the returned
plan satisfies the transcription's defects to tolerance"* rather than echoing
the solver's status string (D-S10). The second one matters independently:
IPOPT's "stopped at a point that was converged" is reported as failure today.

### A.4 Numerics: the JAX integrators should not be hand-rolled forever

The first review documented the evaluator's 40-method integration grid and
proposed moving integrators into `simulation/solvers/`. I would go one step
further: **for the JAX path, adopt Diffrax as the integrator rather than
maintaining an in-house RK4/Euler family.** Diffrax provides adaptive
step-size control, dense output, event handling, and — critically for the
learn pillar — proper reverse-mode adjoints through the solve, all under
`jit`/`vmap`, and its vector-field signature `f(t, y, args)` maps onto
`f_trace_p` in a five-line wrapper. `dynax` (the closest research-side peer
I found) is built exactly this way: Equinox modules for systems, Diffrax for
integration, SciPy for fitting.

That does not mean deleting `rk4_step` — a fixed-step RK4 is the right thing
to *teach*, and MPC/hybrid ZOH stepping legitimately wants a fixed step. It
means the *default* JAX ODE solve should be `diffrax.Tsit5` with a PID
controller, exposed as one `SolverBackend` (D-S27), and the parametric
integrate-helper grid can then shrink to the handful of primitives the hybrid
path uses.

Related: `Sys2Gym.step` integrates with a pure-Python forward-Euler call to
`self.sys.f` at `dt=0.05`. For a 6-state drone at 200 000 PPO timesteps that
is acceptable; it is also the least accurate integrator in the library on the
one path where the policy is *trained* on the dynamics. The compiled
evaluator's `integrate_zoh` (RK4, optionally JAX) is a drop-in (D-S33).

### A.5 Teaching surface — what the curriculum tells me

The eleven intro notebooks plus seven teaching notebooks are a real
curriculum, and the ordering (core → blocks → dynamics → control → analysis →
simulation → hybrid → compile → optimization → planning → graphical) is
sensible. Three observations:

- **Every notebook opens with a 5-level import** and the MPC lesson needs 16
  of them. The first review quantified this (2 of 555 imports use the
  documented layers); I only add that the fix is the single largest
  readability improvement available per hour of work.
- **The Sphinx site is autodoc-only.** That is fine for v0.1 — README plus
  notebooks are the real docs — but the landing page should say so and link
  the notebooks first, not the API tree.
- **The Colab cell is `git clone` + `sys.path.insert`.** It works. `pip
  install minilink` would be one line and would also make the name yours
  (it is unclaimed on PyPI as of today).

### A.6 Architecture — where I agree, and two places I read differently

I independently reached the same conclusions as the first review on: the
default output grid (100 001 samples; 400 000 RHS evaluations on the JAX
path), `x0` being a stale snapshot while `params` is live, the two
importable `Box`/`Sphere` types, the two mechanical bases, the vehicle
family's overlap, the unused teaching-import layer, the evaluator method
grid, and the hybrid stack living outside the `System` hierarchy. I verified
each with my own probes rather than re-reading the numbers.

Two places I read differently:

**NumPy "compile" is not a speedup — and that's fine.** The compiled NumPy
diagram evaluator runs `f` in 12.7 µs; the recursive reference `diagram.f()`
runs in 13.7 µs. On the NumPy path `compile()` buys topology sorting and a
uniform API, not performance; the README's "compiled execution" claim is a
JAX claim. Say so plainly. It also means the Basic tier's ceiling is Python
call overhead (~7 µs per leaf `f`), which is entirely adequate for teaching —
the 2-link arm with joint impedance simulates 1 001 points in milliseconds.

**`examples/projects/` should stay in the repo.** The first review suggested
a sibling repository. For a solo maintainer that is friction with no upside;
the goal is only that those 7 200 lines are *not part of the release
contract*. Excluding them from the wheel, the demo sweep, and the README is
sufficient (D-S18, D-S20).

### A.7 Research fit — what your papers ask of the library

Your recent work (dimensionless policies via Buckingham π; zero-shot RL
transfer across physical parameters; multi-vehicle transfer learning) has a
precise software signature: **the same `f`, evaluated over a family of
parameter vectors, feeding a policy-synthesis or policy-evaluation loop.**
Minilink already has the two halves — `f_trace_p(x, u, t, params)` with a
pytree of parameters, and `vmap` — but no object joins them:

- there is no batched rollout facade (`rollout_batch(x0s, params_batch)`),
- `PolicyEvaluator` and `DynamicProgrammingPlanner` take one problem with one
  parameter set,
- `Trajectory` has no batch axis, so a family of rollouts has nowhere to live.

Adding the facade (D-S26) is an hour and turns a five-line vmap incantation
into the research primitive your students would use daily. Adding an
iLQR/DDP planner built from `jacfwd` of `f_trace` (D-S36) would be the
cleanest possible demonstration of "research-grade algorithm assembled from
minilink parts" — and it is a GRO860 topic.

### A.8 Repository hygiene

37 local branches, of which **31 are already merged into `main`**; 78 remote
branches, of which **42 are `cursor/*` agent branches**. None of this affects
users, but it makes `git branch` useless as a map of live work and it is a
five-minute cleanup (D-S21).

---

## Part B — Reading the Opus review

The first review is careful, its numbers reproduce, and its verdict — sound
foundation, oversized surface, docs drifting — is one I share. Where I would
adjust:

| Opus finding | My view |
| --- | --- |
| Default grid = 100 001 samples; JAX auto-selects fixed-step RK4 (§5.1) | **Agree, P0.** Independently reproduced. |
| Teaching imports used in 2 of 555 example imports (§3.1) | **Agree, P0.** The most readability per hour available. |
| 61% of catalog fails under JAX; `xp` sweep is mechanical (§3.2) | **Agree, P1.** Verified the two-line fix myself. |
| `HybridDiagram` not a `System`; rename for v0.1, promote for v1.0 (§3.3, §4.1) | **Agree with the recommendation**, but I weight it lower for a teaching release — hybrid/MPC is not first-semester material. |
| Evaluator absorbed the solver's job; 50 methods unreferenced (§4.3) | **Agree on the diagnosis.** On the remedy I prefer *outsourcing* the JAX integrators to Diffrax (A.4) over an in-house re-layering. |
| "Two scope reversals in three months" (§2.3) | **Overweighted.** The reversals happened, but the current code and DESIGN are internally consistent; only ROADMAP §7 and the parity audit lag. This is a docs-sync task, not a governance finding. |
| `control/` uses index slicing instead of `get_port_values_from_u` (§6.6) | **Disagree on priority.** `r = u[:n]; y = u[n:]` is exactly what a textbook shows; the helper is safer but not obviously more readable. Style note, not a plan item. |
| Move `examples/projects/` to a sibling repo (§6.8) | **Disagree.** Keep in-tree, exclude from the release contract (A.6). |
| python-control bridge instead of building frequency tools (§8 P2-16) | **Strongly agree.** Cheapest deletion of a ROADMAP priority available. |
| Plotting has eight homes (§3.5) | **Agree it's inconsistent; low priority.** Domain plots next to domains is defensible; just write the rule down. |
| Two geometry vocabularies, two mechanical bases (§4.4–4.6) | **Agree, P2** — after the teaching release. |

What the first review missed, all of which I would rank in the top ten:

1. Silent acceptance of wrong-shape `f`/`h` on the NumPy path (A.2).
2. Silent open-loop simulation when feedback is unconnected (A.2).
3. JAX float32 default breaking trajopt reliability; DP enables x64, trajopt
   does not (A.3).
4. `SolveMetadata.success` echoing solver status instead of checking defects
   (A.3).
5. The Basic tier verified in a genuinely clean environment (A.1) — a
   positive the release should advertise and a test should protect.
6. The JAX research substrate verified to `vmap`/`grad` end to end (A.1) —
   a positive the README undersells.
7. `Sys2Gym` integrating with a Python Euler loop (A.4).
8. The Diffrax option (A.4).
9. Research-facing gaps: batched rollout facade, parameter-family policy
   evaluation, iLQR (A.7).
10. Branch hygiene (A.8).

---

## Part C — Combined verdict

**What I think of minilink, plainly.** It is a serious piece of work and a
better foundation than pyro by every measure that matters for your two
audiences: real ports with metadata, arbitrary diagrams, a compile path that
genuinely delivers `jit`/`vmap`/`grad`, a planning/optimization layer with
clean declarative contracts, and a control band that reads like a textbook.
The core abstractions are the same ones Drake and python-control converged
on, expressed with a composition grammar nicer than either. I found no
structural mistake that would make me want to restart any layer.

The risk is not quality; it is **surface area versus one maintainer**, and —
new from my pass — **silent failure on the beginner path**. A library that
returns garbage for a wrong-shape `f`, runs open-loop when you forget a wire,
and reports a working trajectory as a failed optimization will generate
support load out of proportion to its actual defects. Those three are cheap.

The strategy both reviews converge on:

1. **Make the beginner path fail loudly and default sanely** (D Phase 0).
2. **Make the teaching surface an executable contract** — one name list,
   tests that examples import only through it, a Basic-tier smoke in a
   clean environment (D Phase 1).
3. **Make the JAX claim true across the catalog and give research a
   facade** (D Phase 2).
4. **Subtract**: dead evaluator API, dead modules, merged branches; exclude
   quarantine and projects from the release contract without deleting them
   (D Phases 1–2).
5. **Defer the real refactors** — evaluator/solver layering, hybrid as a
   `System`, geometry unification — until after a cohort has used v0.1
   (D Phase 3).
6. **Replace "pyro parity" with "one course, end to end, from `pip
   install`, in Colab."**

---

## Part D — Roadmap in agent-sized steps

Each step is scoped to roughly one hour for a coding agent working in this
repo with the existing test suite as a safety net. Every step is independent
unless a prerequisite is listed. "Done when" is the acceptance test. Steps
marked **[human]** need a decision or credentials from you.

Conventions: run `ruff check . && ruff format --check .` and the relevant
`pytest tests/unittest/test_<domain>.py` at the end of every step; update
DESIGN/README only where a step changes a public contract.

### Phase 0 — First-hour safety and sane defaults (10 steps, ~1 day)

**S01 — Decouple the output grid from the integration step.**
Touch: `simulation/simulator.py` (`select_time_vector`, `select_solver`),
`simulation/time_grid.py`, `simulation/static_simulator.py`, tests in
`test_simulation.py` that assert `100001`.
Change: when neither `n_steps` nor `dt` is given, default to `n_steps = 1001`
for adaptive solvers; keep `smallest_time_constant`-derived `dt` only for
`euler`/`rk4_fixedsteps`/`euler_fixedsteps`. Make the JAX auto-RK4 rule depend
on the *requested* solver or `discontinuous_behavior`, never on `n_pts`.
Done when: `Pendulum().compute_trajectory(tf=10)` returns 1 001 samples on
both backends; `compile_backend="jax"` default still picks `scipy` with
`nfev≈200`; suite green.

**S02 — Validate `f`/`h`/port shapes at compile time.**
Touch: `core/compile/compiler.py` (leaf + diagram entry), `numpy_evaluators.py`
constructors, `jax_evaluators.py` (run the shape probe *before*
`check_jax_compatible`).
Change: probe `f(x0, u_nom, 0.0, params)` and every `port.compute(...)`;
raise `ValueError("f() of 'Name' returned shape (1,); expected (2,) for
n=2")` (and the port analogue). Skip the probe for `n == 0` dynamics.
Done when: a `DynamicSystem(n=2)` whose `f` returns `(1,)` raises the new
message from `compute_trajectory`, `compile("numpy")`, and `compile("jax")`;
a new test in `test_compile.py` covers `f` and a port compute; suite green.

**S03 — Warn when a controller's measurement port is unconnected.**
Touch: `core/compile/compiler.py` or `simulation/simulator.py` (one place),
`core/feedback.py` (`feedback_ports` already resolves the measurement role).
Change: at diagram compile, for each subsystem with a resolvable declaration
whose `measurement` input has `connections[sid][port] is None`, emit one
`UserWarning`: `"ctl: measurement port 'y' is unconnected — the controller
runs on its nominal value (open loop). Connect plant→ctl or set
allow_unconnected=True."` Add `allow_unconnected: bool = False` to
`DiagramSystem.compile` / `Simulator`.
Done when: `DiagramSystem` with an `ImpedanceController` lacking feedback
warns once; `ctl @ plant` does not; existing demos pass without new warnings
(fix any that intentionally leave a measurement unconnected by passing the
flag); test in `test_diagrams.py`.

**S04 — Fix the README custom-plant example and the `@` "dim None" message.**
Touch: `README.md` (add `output_dim=2` to `MassSpringDamper`), `core/
composition.py` (`_feedback_mismatch_message`).
Change: when the plant lacks the expected output port, say `"plant 'X' has no
'y' output port; pass output_dim=… to DynamicSystem or wire ports explicitly
with add_subsystem/connect"`.
Done when: `ProportionalController() @ MassSpringDamper()` from the README
either works or raises the new message; test in `test_diagrams.py`.

**S05 — Make float64 the JAX policy.**
Touch: `core/backends.py` (`require_jax_numpy` or a new `ensure_jax_x64()`),
`core/compile/evaluators/jax_evaluators.py` (all constructors),
`optimization/evaluators/jax_evaluator.py`, `planning/trajectory_optimization/
parametric_evaluator.py`; docs line in DESIGN §*NumPy and JAX*.
Change: enable `jax_enable_x64` on first JAX-evaluator construction unless
`MINILINK_JAX_X64=0`; DP's existing calls become no-ops.
Done when: the A.3 pendulum/cart-pole SLSQP problems report `success=True`
on `compile_backend="jax"` without the caller touching `configure_jax`; the
34 example/test call sites keep working; test in `test_jax_planning.py`.

**S06 — Guard against a forgotten `super().__init__()`.**
Touch: `core/facades.py` (`compile`, `compute_trajectory`, `plot_*`,
`animate`) via one small helper.
Change: if `not hasattr(self, "inputs")`: raise `TypeError(f"{type(self).
__name__}.__init__ must call super().__init__(n=…) before use")`.
Done when: the A.2 `NoSuper` case raises the new message; test in
`test_core.py`.

**S07 — One-line default simulation report.**
Touch: `simulation/sim_reporting.py`, `Simulator`, `StaticSimulator`,
`HybridSimulator` (same helper).
Change: `verbose=True` prints one line (`solver, n_pts, dt, wall time,
success`); the framed panel moves behind `verbose="full"`. Keep `verbose=
False` silent.
Done when: `compute_trajectory()` prints one line; `verbose="full"` prints the
panel; tests that grep report text updated.

**S08 — Fix the `nbstripout` hook and strip stored outputs.**
Touch: `.pre-commit-config.yaml` (`files: ^examples/.*\.ipynb$`),
`examples/learn/teaching/mpc.ipynb`.
Done when: `pre-commit run nbstripout --all-files` changes nothing after the
strip; the one notebook with outputs is clean.

**S09 — Trajopt `success` means "defects satisfied". [prereq S05]**
Touch: `planning/trajectory_optimization/planner.py` (result assembly),
`planning/results.py` (`SolveMetadata` — add `max_defect`, `max_violation`),
transcription classes (expose a `defects(z)` / `constraint_violation(z)`).
Change: `success = solver_ok or (max_defect ≤ tol_defect and
max_violation ≤ tol_g)`; report both numbers in the solve summary.
Done when: an IPOPT "converged to acceptable point" plan that satisfies
defects reports `success=True`; an infeasible plan reports `False` with the
defect norm; tests in `test_planning.py`.

**S10 — Prepare the PyPI release. [human for credentials]**
Touch: `pyproject.toml` (verify metadata), `README.md` install section,
`install.md` (`pip install minilink` first, clone second), Colab cells in
`examples/learn/**` (replace clone with `pip install minilink`).
Agent does: `python -m build`, `twine check dist/*`, a `0.1.0rc1` tag plan,
and a `publish.yml` workflow using trusted publishing. You do: create the
PyPI project and approve the first upload.
Done when: `pip install dist/minilink-*.whl` into a clean venv runs the README
quick start.

### Phase 1 — The teaching surface as an executable contract (11 steps)

**S11 — Teaching-surface registry + Basic-tier smoke test.**
Touch: new `minilink/teaching.py` (a single tuple of `(module, name)` for the
frozen surface; no re-exports — the band facades already exist), new
`tests/unittest/test_teaching_surface.py`.
Tests: every registered name imports and has a docstring; every name lives
under a stable band path (`core`, `simulation`, `blocks`, `dynamics/catalog`,
`control` minus `mpc`, `analysis`); and a subprocess smoke that blocks
`graphviz`/`jax`/`meshcat`/`plotly`/`pygame`/`sympy` imports and runs
sim + plot + phase plane + animate + linearize + lqr + VI (the A.1 probe).
Done when: the test passes and fails if a name is removed or a hard import of
an optional package sneaks into the Basic path.

**S12 — Import-layer CI check for `examples/learn/`. [prereq S11]**
Touch: `tests/unittest/test_public_imports.py` (extend), an allowlist file.
Change: AST-walk every notebook and script under `examples/learn/`; assert
each `from minilink… import` is a root-prelude, band-facade, or
allowlisted deep import; start with the current deep imports allowlisted so
the test is green, then shrink the allowlist in S13–S15.
Done when: the test runs in CI and the allowlist is the only thing keeping it
green.

**S13 — Add the missing band facades.**
Touch: `simulation/__init__.py` (`Simulator`, `StaticSimulator`), new
`planning/__init__.py` exports (`PlanningProblem`, `TrajectoryOptimization
Planner`, `DynamicProgrammingPlanner`, `StateSpaceGrid`, `RRTPlanner`,
`RRTStarPlanner`), `core/__init__.py` (`Trajectory`, `DiagramSystem`, costs,
sets) — same lazy `_EXPORTS` + `__getattr__` pattern as `control/`.
Done when: `from minilink.planning import PlanningProblem` works; existing
deep imports still work; `test_public_imports.py` covers the new names.

**S14 — Rewrite `examples/learn/intro/*.ipynb` imports. [prereq S13]**
Mechanical: `minilink.dynamics.catalog.pendulum.pendulum` → `minilink.catalog`,
`minilink.control.impedance` → `minilink.control`, etc. One step for
`00`–`05`, one for `06`–`10` + both showcases.
Done when: notebook smoke (`run_notebook_checks.py`) passes; the S12 allowlist
shrinks accordingly.

**S15 — Rewrite `examples/learn/teaching/*` and `examples/demos/*` imports.
[prereq S13]** Four steps by folder: teaching notebooks; `demos/{control,
diagrams,blocks,plots,statespace,analysis}`; `demos/{planning,trajopt,
optimization}`; `demos/{mpc,hybrid,step,robotic,animation,realtime}`.
Done when: `run_all_demos.py` passes 60/60; S12 allowlist is empty for
`learn/` and minimal for `demos/`.

**S16 — Delete the `_jit` aliases.**
Touch: `core/compile/evaluators/tiers.py` (`register_jit_aliases`,
`_TRACE_TIER_SUFFIXES`), six call sites in `jax_evaluators.py`,
`test_compile.py::test_f_jit_alias_identity`, DESIGN §5 alias sentence.
Done when: `grep -rn "_jit" minilink/` finds only internal `_f_jit_fn`-style
closures; suite green.

**S17 — Delete the 28 unreferenced public evaluator methods.**
Touch: `jax_evaluators.py`, `numpy_evaluators.py`, DESIGN §5 "frozen subset"
table (remove the "stable-internal grid" paragraph).
List: `euler_integrate_ivp_p`, `euler_integrate_ivp_trace`,
`euler_integrate_ivp_trace_p`, `euler_integrate_zoh_p`,
`euler_integrate_zoh_trace_p`, `euler_step_ivp_p`, `euler_step_ivp_trace`,
`euler_step_ivp_trace_p`, `euler_step_trace`, `euler_step_trace_p`,
`f_ivp_scipy`, `f_scipy`, `integrate_zoh_p`, `outputs_trace_p`,
`rk4_integrate_ivp_p`, `rk4_integrate_ivp_trace`,
`rk4_integrate_ivp_trace_p`, `rk4_integrate_linear_trace`,
`rk4_integrate_linear_trace_p`, `rk4_integrate_zoh_trace_p`, `rk4_step_ivp`,
`rk4_step_ivp_p`, `rk4_step_ivp_trace`, `rk4_step_ivp_trace_p`,
`rk4_step_trace_p`, `rollout_p`, `step_block`, `step_trace_p`.
Done when: suite green; the census script in the first review's Appendix A
reports zero public unreferenced evaluator methods.

**S18 — Delete dead modules; exclude quarantine and projects from the wheel.**
Touch: delete `planning/spatial/overlays.py`, `symbolic/mechanics/utils.py`;
`pyproject.toml` `[tool.hatch.build.targets.wheel]` exclude `minilink/
symbolic/**`, `minilink/dynamics/engines/**`; `examples/README.md` note that
`projects/` and `sandbox/` are outside the release contract.
Done when: `python -m build` produces a wheel without those paths; suite
green (their tests still run from the repo).

**S19 — Give `c_export` an honest home.**
Touch: `tests/demo_checks/flagship_manifest.json` (add both `c_export`
demos, `requires: ["jax"]`), `ROADMAP.md` TRL table (new row "C export
(interfaces/c_export)", TRL 2), DESIGN §3 interfaces row.
Done when: the regression CI job runs the two demos.

**S20 — Nightly full demo sweep.**
Touch: `.github/workflows/nightly.yml` running `run_all_demos.py --timeout
120 --continue-on-error` and `run_notebook_checks.py` on a schedule.
Done when: the workflow file validates and runs on `workflow_dispatch`.

**S21 — Branch hygiene. [human confirms the list]**
Agent produces the list of the 31 local branches merged into `main` and the
42 remote `cursor/*` branches with last-commit dates and a `git branch -d` /
`git push origin --delete` script; you run it.
Done when: `git branch` shows only live work.

### Phase 2 — Make the JAX claim true; give research a facade (12 steps)

**S22a–h — `xp` sweep, one module per step.**
Order (by teaching value): `vehicles/steering.py`; `pendulum/cartpole.py`
(then `JaxCartPole` becomes redundant); `aerial/drone.py`;
`manipulators/arms.py` (may need two steps — `_trig` helpers); `marine/
boat.py`; `mass_spring_damper/linear.py`; `vehicles/dynamic_bicycle.py`;
`aerial/rocket.py` + `vehicles/mountain_car.py` + `vehicles/suspension.py` +
`equations/oscillators.py` + `vehicles/propulsion.py`.
Pattern: `xp = array_module(x, u)` after params unpack; `np.` → `xp.` in `f`,
`h`, port computes, `H`/`C`/`g`; no in-place writes; keep `np` for
constructor metadata.
Done when: `Cls().compile(backend="jax")` succeeds and `f` matches NumPy on
three random points; `test_dynamics_catalog.py` parametrized parity test.

**S23 — Catalog both-backends contract test. [prereq S22a]**
Touch: `tests/unittest/test_dynamics_catalog.py`.
Change: parametrize over `minilink.catalog.__all__`; assert `compile("numpy")`
and `compile("jax")` both succeed and agree; `xfail` list = the modules not
yet swept, shrinking to empty by S22h.
Done when: the test exists and the xfail list is the only red.

**S24 — Retire `JaxCartPole`. [prereq S22b, S23]**
Touch: delete the class; update `catalog/__init__.py`, demos/notebooks that
import it (the trajopt demos), DESIGN mention.
Done when: `grep -rn JaxCartPole` is empty; trajopt demos run on `CartPole`.

**S25 — Replace the six `*Ports` vehicle twins with a constructor option.**
Touch: `vehicles/jax_vehicles.py`.
Change: `BicycleDyn(named_ports=True)` (and rungs) declares the named input
ports and packs `u` inside `f` once; delete `BicycleAccPorts`,
`BicycleDynPorts`, `BicycleDynRatePorts`, `BicycleDynTauRatePorts`,
`BicycleDynServoPorts`, `BicycleDynEnginePorts`; update the hybrid/MPC demos.
Note: `DynamicBicycle`'s named `w_rear`/`delta` ports are a settled decision
and stay as they are.
Done when: hybrid/MPC demos and `test_hybrid.py` pass.

**S26 — Batched rollout facade for research. [prereq S05]**
Touch: `jax_evaluators.py` (`rollout_batch(x0s, u_sequences=None, t0=0.0,
dt=…, n_steps=…, params=None)` built on `jax.vmap` of the existing trace
rollout; `params` may be a pytree with a leading batch axis), a new demo
`examples/demos/identification/rollout_param_family.py` (vmap over `m`, `l`
— the Buckingham-π experiment), a test.
Done when: 100 initial states × 100 parameter sets roll out in one call and
match a loop of single rollouts.

**S27 — Optional Diffrax solver backend.**
Touch: new `simulation/solvers/diffrax_ivp.py` (`DiffraxSolverBackend`
implementing `integrate` / `integrate_forced` via `diffeqsolve(ODETerm(lambda
t, y, args: evaluator.f_trace_p(...)), Tsit5(), stepsize_controller=
PIDController(...), saveat=SaveAt(ts=times))`), `simulator.py` preset
`"diffrax"`, `pyproject` extra `jax = [..., "diffrax"]`.
Done when: `compute_trajectory(solver="diffrax", compile_backend="jax")`
matches `scipy` to `rtol 1e-5` on the pendulum; skipped when diffrax is
absent.

**S28 — Decide the vehicle ladder. [human]**
Agent produces a one-page table: each of the 9 rungs + 7 `steering.py`
classes, its demos/notebooks, and whether an equivalent now exists on the
other backend after S22. You choose which survive in `catalog/` and which
move to `examples/projects/`.

### Phase 3 — Foundations, after a cohort has used v0.1 (9 steps)

**S29 — `DiagramSystem.x0`, `n`, `state` as derived properties.**
Touch: `core/wiring.py` (`compute_state_properties` → properties;
`refresh()` shrinks), `Simulator` (drop the pre-read `refresh()`).
Done when: setting `plant.x0` after wiring is visible on `diagram.x0`
immediately; suite green.

**S30 — Rename the graphical `Sphere`/`Box` glyphs.**
Touch: `graphical/animation/primitives.py` (`Sphere` → `SphereGlyph`, `Box`
→ `BoxGlyph`), all catalog skins and demos (≈25 files, mechanical).
Done when: `grep -rn "primitives import.*\bBox\b"` is empty; suite green.

**S31 — `HybridDiagram` → `HybridLoop`; `%` → `on_schedule()`.**
Touch: `core/hybrid_diagram.py`, `core/hybrid_composition.py`,
`simulation/computer.py`, `System.__mod__` (remove), demos, DESIGN §4.
Done when: hybrid demos and `test_hybrid.py` pass with the new names; no
alias left behind.

**S32 — Unify the mechanical bases (two steps).**
(a) `MechanicalSystem(GeneralizedMechanicalSystem)` with `N = I`,
`pos = dof`, and `H` as the documented name; (b) port `Boat2D`/`Plane3D`
onto the unified base and give them `q`/`dq` ports.
Done when: `test_mechanical_robotics.py` and catalog checks pass.

**S33 — `Sys2Gym.step` on the compiled evaluator.**
Touch: `interfaces/gymnasium.py` (compile once in `__init__`; step via
`integrate_zoh`; `backend=` kwarg), PPO notebook unchanged.
Done when: the drone PPO notebook trains to the same qualitative policy;
`test_interfaces_gymnasium.py` parity test against the old Euler path at
small `dt`.

**S34 — python-control bridge.**
Touch: new `interfaces/python_control.py` (`to_control(lti) -> control.
StateSpace`, `from_control(ss) -> LTISystem`), a demo `examples/demos/
analysis/classical_with_python_control.py` (bode/margin/rlocus/nyquist on a
linearized cart-pole), `pyproject` extra `control = ["control"]`, ROADMAP
priority 1 replaced by this bridge.
Done when: the demo runs and the round trip preserves `A, B, C, D`.

**S35 — Docs sync.**
Touch: DESIGN §*Continuous-time core* (duplicated line 223–224), ROADMAP §1
(north star → "one course end to end from `pip install`"), §7 (hybrid
described as it exists), `docs/plans/pyro-port-remaining.md` (stale "Done"
rows for `*WithPositionInputs`; three nonexistent paths), delete
`docs/plans/control-block-contract.md` (landed), `docs/plans/README.md`
index rows, Sphinx `index.rst` (link notebooks before the API tree).
Done when: the first review's doc-drift script reports zero missing paths.

**S36 — iLQR planner from parts (research showcase, optional).**
Touch: new `planning/trajectory_optimization/ilqr.py` (`ILQRPlanner
(Planner)`: `jacfwd` of `f_trace` for `A_k, B_k`, quadratic cost expansions,
backward/forward passes under `jit`), a demo on the cart-pole, a test against
the collocation solution.
Done when: cart-pole swing-up converges in under a second on JAX and the plan
matches direct collocation to plotting accuracy.

**S37 — Evaluator/solver re-layering. [prereq S27, multi-step, defer]**
The deep refactor from the first review's §4.3 — only worth starting once
S27 has shown the JAX default solve can live outside the evaluator.

### Suggested sequencing

- **Week 1:** S01–S10 (Phase 0). Ship `0.1.0rc1`.
- **Weeks 2–3:** S11–S21 (Phase 1). The teaching surface becomes a test.
- **Weeks 4–6:** S22–S28 (Phase 2). The JAX claim becomes true; research
  gets `rollout_batch`.
- **After one semester:** Phase 3, in whatever order the cohort's questions
  suggest.

The first ten steps are one agent-day and remove every silent failure and
bad default a student can hit in their first hour. If only those ten happen
before the course starts, the release is already in better shape than the
current ROADMAP's five priorities would leave it.

---

## Part E — Converged plan (post-interview, 2026-09-05)

Recorded after the maintainer interview. Where an answer changed Part D, this
section wins. Branch: `dev-fable`.

### E.1 Decisions

| Question | Ruling |
| --- | --- |
| Target course | **GRO860 — optimal control & RL.** Term is **already running (Fall 2026)**: students use the git clone today, so v0.1 is a mid-term hardening release, and names the GRO860 notebooks already import must keep working through the term. |
| v0.1 milestone | Every GRO860 topic runs end to end on the teaching surface, from `pip install minilink`, in Colab: **value iteration / DP on a grid · LQR + linearization · trajectory optimization (collocation/shooting) · RL via `Sys2Gym` + SB3 (PPO)**. |
| North star | Course-first for v0.1; **pyro parity moves to v0.2**. |
| Import style | **Band facades everywhere in student-facing material** (`from minilink.catalog import …`, `minilink.control`, `minilink.analysis`, `minilink.planning`, `minilink.simulation`), enforced by a CI test. Deep imports stay valid for library code and research. |
| Release scope | **Teaching wheel; research from git.** `symbolic/`, `dynamics/engines/`, `interfaces/c_export.py`, `examples/projects/`, `examples/sandbox/` stay in the repo, outside the wheel and outside the release contract. |
| Default grid | Fixed output count (`n_steps = 1001`) for adaptive solvers; `dt` from the time constant only for fixed-step solvers; auto-RK4 no longer keyed on point count. |
| Simulation report | **Keep the framed panel** (pyro style) as the `verbose=True` default; only unify the flag names (`verbose` everywhere). Step S07 reduced to that. |
| Unconnected inputs | **Stays silent by design** (unconnected = nominal, Simulink-ground semantics). Document loudly in `00_core`; no warning. Step S03 dropped. |
| JAX precision | **Library-wide float64 default**, `MINILINK_JAX_X64=0` opts out. |
| Hybrid / MPC | **Not a GRO860 topic.** Stays provisional; no rename before the term; the MPC lesson keeps shipping but is off the v0.1 checklist. `HybridLoop` question deferred to v1.0. |
| Diffrax | Noted as a future improvement; **not short-term**. |
| Vehicle plants | GMC714 teaches a modelling ladder, so a **small teaching ladder stays in `catalog/`**; specialized research variants move to `examples/projects/`. |
| Docs | **Full consolidation**, before any Python change. |
| Install / PyPI | **Conda stays the recommended local install; Colab keeps the git-clone cell.** PyPI publication wanted eventually as a third option — not a v0.1 gate, not Phase 0. S10 → v0.2. |
| Frequency tools | **Decision postponed** (original intent: minimal NumPy-only `pzmap`/`nyquist`/`margin`/`ss2tf`; python-control bridge possible later). Not part of this consolidation; S34 withdrawn from the plan. |
| AGENTS rules | Two-lane entry gate is a **soft rule**. The no-leading-underscore rule is a style preference and **stays as written** (the 43 `_method` names on System subclasses are a later cleanup, not a rule change). |

### E.2 What the census says about the vehicle ladder

Used in `examples/learn` + `examples/demos`: `BicycleDynRate` (5 files),
`HolonomicMobileRobot` (3), `Holonomic` (2), `KinematicCar` (2),
`DynamicBicycleCar3D` (2), `DynamicBicycle` (1). **Zero** teaching uses:
`KinematicBicycle`, `BicycleKin`, `BicycleAcc`, `BicycleDyn`,
`BicycleDynTauRate`, `BicycleDynServo`, `BicycleDynEngine`,
`ConstantSpeedKinematicCar`, `DynamicHolonomicMobileRobot`,
`HolonomicMobileRobot3D`, `UdeSRacecar`, `CarProfile`.

Teaching ladder (**confirmed**): **holonomic point → kinematic bicycle
(car skin) → dynamic bicycle with linear tires → one actuated rung
(`BicycleDynRate`)** — four rungs, each dual-backend after the `xp` sweep.
Everything else moves under `examples/projects/pathtracking/` with the
scenarios that use it. DP keeps all three backends by design — `loop` is the
readable teaching reference, `numpy` the object-level core, `jax` the
accelerated research tier — they are a ladder, not duplication.

### E.3 Revised phases

**Phase D — docs as plan of record (7 steps, before any code).**

| Step | Doc | Change |
| --- | --- | --- |
| D1 | `ROADMAP.md` | Rewrite: §1 north star = GRO860 end to end (v0.1), pyro parity (v0.2); two-lane rule (teaching surface = contract, research lane free); phases 0–3 as the milestone plan; TRL table kept, add rows for C export and realtime; §7 out-of-scope rewritten to describe hybrid as it exists. |
| D2 | `README.md` | Student-first: `pip install minilink` + Colab cell first; the fixed custom-plant example; band-facade imports throughout; API-stability table replaced by "teaching surface / research lane"; call-chains section trimmed. |
| D3 | `DESIGN.md` | Trim to contracts: fold the landed control-block decision record in (remove the `TODO: User Architectural Review` marker), delete the duplicated line, add the notation-collision note, add the wheel-scope rule, state the NumPy-compile-is-not-a-speedup fact, state float64 policy, state the "unconnected = nominal" rule as a contract. |
| D4 | `AGENTS.md` | Import rule for student-facing code; wheel/release scope; two-lane entry rule ("nothing enters the teaching surface without demo + both-backend test + docstring"); narrow the leading-underscore rule to facade/contract methods. |
| D5 | `docs/plans/TODO.md` | Re-base on Phases 0–3; drop rows made moot (S03, hybrid rename now). |
| D6 | `docs/plans/` | Delete `control-block-contract.md` (Implemented) and `test-benchmark-consolidation.md` (Complete); fold `vehicle-abstraction.md` into the ladder decision and delete; move `mpc-tuning.md` next to `examples/projects/mpc/`; shrink `pyro-port-remaining.md` to open rows and fix the stale `*WithPositionInputs` "Done" rows and the three nonexistent paths; mark `phase4-fidelity-maps`, `optimizer-parametric-wiring`, `standard-planning-problems`, `neural-blocks-collection`, `articulated-mechanism` as **Later / research lane**; update the plans index. |
| D7 | `install.md`, `examples/README.md`, `docs/index.rst` | pip-first install; notebooks linked before the API tree; `projects/`/`sandbox/` labelled outside the release contract. |

**Phase 0 — first-hour safety (9 steps, one agent-day).**
S01 default grid · S02 shape validation · S04 README example + `@` message ·
S05 float64 policy · S06 `super().__init__` guard · S07 unify verbose flag
names (panel stays) · S08 nbstripout · S09 trajopt `success` = defects
satisfied. (S03 dropped by decision; S10 PyPI moved to v0.2.) Because the term is
running, Phase D is compressed to one to two days and Phase 0 starts the
moment the docs are merged.

**Phase 1 — teaching contract + the GRO860 path (14 steps).**
S11 teaching-surface registry + Basic-tier clean-env smoke · S12 import-layer
CI check · S13 band facades for `simulation`/`planning`/`core` · S14–S15
rewrite imports in `learn/` then `demos/` · **S33 promoted:** `Sys2Gym.step` on
the compiled evaluator (`integrate_zoh`, optional JAX) · **new S38:** DP
honesty — `DynamicProgrammingOptions.final_time` reads `problem.tf`,
`SolveMetadata.success` reports convergence, and the VI notebooks use
`vi_ctl @ plant` instead of hand wiring · S16 delete `_jit` aliases · S17
delete 28 unreferenced evaluator methods · S18 wheel excludes quarantine +
projects · S19 `c_export` TRL row (research lane) · S20 nightly demo sweep ·
S21 branch hygiene.

**Phase 2 — the JAX claim, and the research facade (10 steps).**
S22a–h `xp` sweep (order: `steering`, `cartpole`, `drone`, `arms`, `boat`,
`linear`, `dynamic_bicycle`, the rest) · S23 catalog both-backends contract
test · S24 retire `JaxCartPole` · **S25 revised:** carve the four-rung
teaching ladder, `named_ports=` flag replaces the `*Ports` twins, the other
rungs + `car_profile` move to `examples/projects/` · S26 `rollout_batch`
research facade (parameter-family sweeps).

**Phase 3 — after the term.**
S29 derived `x0` · S30 geometry glyph rename · S31 `HybridLoop` (v1.0
question) · S32 mechanical bases · frequency tools (native or bridge —
postponed decision; S34 withdrawn) · S10 PyPI option · S36 iLQR (idea) ·
S27 Diffrax (later) · S37 evaluator re-layering (later).

### E.4 Resolved in the second interview round

1. GRO860 is running now (Fall 2026) — v0.1 is a mid-term release.
2. The framed simulation panel stays as the default report.
3. The four-rung vehicle teaching ladder in E.2 is confirmed.
4. The DP `loop` backend stays in the library as the documented reference.
5. (Unchallenged assumption) The MPC teaching notebook keeps shipping under
   `learn/teaching/`, labelled provisional and off the v0.1 checklist.
