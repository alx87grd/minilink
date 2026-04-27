# Minilink package layout (target)

Structural **north star** for the repository: where new code lives and how packages depend on each other. Ordinary PRs move the tree toward this document; depth, contracts, and TRL live in [DESIGN.md](DESIGN.md).

**See also:** [pluggable_implementations.md](pluggable_implementations.md) (ABC + implementation file naming for optimizers, future solvers, renderers).

---

## Canonical tree (full target layout)

Illustrative filenames; exact modules evolve in PRs. **`compiler.py`** is the **orchestrator** (build / select execution plan, dispatch backend, return an evaluator). There is **no `compile/compilers/`** subpackage — the name collides with the orchestrator; each **`evaluators/*.py`** backend module owns **construction + lowering** for that backend (or small private helpers next to `compiler.py`).

**Naming:** Prefer **long, explicit evaluator modules** under `evaluators/`, e.g. **`numpy_evaluator.py`**, **`jax_evaluator.py`** (not bare `numpy.py` / `jax.py`), for clarity and stable imports.

**`jax_utils`:** Target **`compile/jax_utils.py`** (JAX compilation / evaluator–adjacent). **`tools/`** is for **external** bridges (Gym, URDF, engines), not internal array-backend sugar.

```text
minilink/
├── __init__.py
│
├── core/                           # modeling language — no ODE/IVP or compile *evaluators* here
│   ├── __init__.py
│   ├── system.py                   # System, DynamicSystem, StaticSystem, ports, VectorSignal
│   ├── diagram.py                  # DiagramSystem
│   ├── trajectory.py               # Trajectory
│   ├── costs.py                    # CostFunction hierarchy (target; may still live under planning/ until moved)
│   ├── sets.py                     # Set, InputSet, boxes, … (target)
│   └── blocks/                     # diagram primitives, not full plants
│       ├── __init__.py
│       ├── basic.py
│       └── sources.py
│
├── symbolic/                       # optional SymPy extra
│   ├── __init__.py
│   └── manipulator/                # example subpackage: derivation / export (names TBD)
│       └── ...
│
├── dynamics/                       # analytic dynamics only
│   ├── __init__.py
│   ├── abstractions/               # inheritance ladder: MechanicalSystem, StateSpace, …
│   │   ├── __init__.py
│   │   └── ...
│   └── catalog/                    # named plants by domain
│       ├── __init__.py
│       ├── pendulum/
│       ├── vehicles/
│       ├── msd/
│       └── equations/              # direct DynamicSystem ODEs, no extra abstraction
│
├── physics/                        # engine-backed (JAX contact MVP today)
│   ├── __init__.py
│   ├── engine_jax.py
│   └── system.py                   # e.g. PhysicsWorldSystem
│
├── compile/                        # diagram → execution plan → evaluator
│   ├── __init__.py
│   ├── compiler.py                 # ORCHESTRATOR: plan build, dispatch, return evaluator
│   ├── execution_plan.py           # ExecutionPlan, scheduling / flat graph semantics
│   ├── jax_utils.py                # JAX helpers shared by evaluators/JAX path (not tools/)
│   ├── evaluator_timing.py         # throughput / profiling of evaluator & f(x,u,t) (was benchmark/f_speed.py)
│   └── evaluators/                 # PRODUCT: ABC + per-backend construction
│       ├── __init__.py
│       ├── evaluator.py            # DynamicsEvaluator ABC (from today’s compile/evaluator.py)
│       ├── numpy_evaluator.py      # NumPy evaluator + NumPy lowering
│       └── jax_evaluator.py        # JAX evaluator + JAX lowering
│
├── simulation/                     # time integration given an evaluator
│   ├── __init__.py
│   ├── simulator.py
│   ├── input_interpolation.py
│   ├── integration_timing.py       # end-to-end sim / stepper benchmarks (was benchmark/simulation_speed.py)
│   ├── scenarios/                  # stress builders & scenario graphs (was benchmark/scenario/)
│   │   ├── __init__.py
│   │   └── ...
│   └── solvers/
│       ├── __init__.py
│       ├── solver.py               # IVP / fixed-step ABC
│       ├── scipy_ivp.py
│       ├── euler.py
│       └── rk4_fixed.py
│
├── estimation/                     # observers, filters, sensor models
│   └── __init__.py
│
├── control/
│   ├── __init__.py
│   └── ...
│
├── planning/
│   ├── __init__.py
│   ├── planner.py
│   ├── problems.py                 # PlanningProblem; imports core.costs / core.sets (target)
│   ├── search/
│   │   └── ...
│   ├── trajectory_optimization/
│   │   └── ...
│   └── policy_synthesis/
│       └── ...
│
├── optimization/
│   ├── __init__.py
│   ├── mathematical_program.py
│   └── optimizers/
│       ├── __init__.py
│       ├── optimizer.py
│       └── scipy_minimize.py
│
├── analysis/                       # diagnostics & numerics (not “how to plot”)
│   └── __init__.py
│
├── graphical/                      # presentation: plot, animate, render
│   ├── __init__.py
│   ├── plotting.py
│   ├── animation.py
│   ├── environment.py
│   ├── primitives.py
│   ├── matplotlib_style.py
│   ├── graphe.py
│   └── renderers/
│       ├── __init__.py
│       ├── renderer.py             # AnimationRenderer ABC (contract module; was base.py)
│       ├── timing.py               # AnimationFrameSchedule / shared playback sampling
│       ├── matplotlib_renderer.py
│       ├── meshcat_renderer.py
│       └── pygame_renderer.py
│
└── tools/                          # Gym, pygame interactive bridge, URDF, MuJoCo, … — external only
    └── __init__.py
```

### `compile/` — roles

| Piece | Role |
|--------|------|
| **`compiler.py`** | **Orchestrator** — public entry: build or accept `ExecutionPlan`, choose backend (`"numpy"` \| `"jax"` \| …), delegate to the matching **`evaluators/*_evaluator.py`** builder, return a concrete **evaluator**. |
| **`execution_plan.py`** | Data structures and helpers for the flat / ordered execution schedule (not the orchestrator). |
| **`evaluators/`** | **Evaluator domain** — `evaluator.py` (ABC); **`numpy_evaluator.py`**, **`jax_evaluator.py`** hold concrete classes **and** backend-specific lowering / assembly (no **`compilers/`** folder). |
| **`jax_utils.py`** | Shared JAX utilities for the compile/JAX-evaluator path (preferred over top-level `jax_utils.py` or `tools/`). |
| **`evaluator_timing.py`** | Benchmarks / throughput for compiled dynamics (`f`, evaluator calls); replaces **`benchmark/f_speed.py`**. |

### Retiring **`benchmark/`**

- **Remove** the top-level **`minilink/benchmark/`** package from the target layout.
- **`benchmark/f_speed.py`** → **`compile/evaluator_timing.py`**.
- **`benchmark/simulation_speed.py`** → **`simulation/integration_timing.py`**.
- **`benchmark/scenario/`** → **`simulation/scenarios/`** (stress systems & builders; **`compile/evaluator_timing.py`** may import from here when scenarios are shared).

**Typical dependency flow:** `core` ← (`dynamics`, `physics`, `estimation`, `control`, `planning`, …); `symbolic` feeds `dynamics`; `compile` consumes `core`; `simulation` consumes `compile` + `core`; `analysis` consumes trajectories / linearizations; `graphical` consumes data produced anywhere.

### `compile/` vs `simulation/` (vocabulary)

- **`compile/`** — *Given a model and a diagram schedule, what **closed-form callable** do I run?* The **evaluator** exposes `f`, `h`, outputs, and simulator hooks. **`compiler.py`** dispatches to **`evaluators/*_evaluator.py`**.
- **`simulation/`** — *Given an evaluator, how do I **advance time** and produce a `Trajectory`?* **`solvers/`** are pluggable IVP / fixed-step backends; **`simulator.py`** orchestrates the loop, interpolation, and backend choice.

**Today vs target:** On disk, files may still sit at `compile/evaluator.py`, `numpy_evaluator.py`, `jax_evaluator.py` (package root) and `simulation/solver_backends.py` until migrated. [pluggable_implementations.md](pluggable_implementations.md) should stay aligned when updated.

---

## Principles

1. **Dependency direction:** Generic types (`Trajectory`, costs, sets, ports) stay in **`core/`** so nothing imports **`planning/`** just to score or constrain trajectories.
2. **Language vs solvers:** *What* the system is → **`core/`**, **`dynamics/`**, **`physics/`**. *How* you integrate, optimize, or compile → **`simulation/`**, **`optimization/`**, **`compile/`**, **`planning/`**.
3. **Pluggable backends** (optimizers, solvers, renderers): role-named contract module (`optimizer.py`, `solver.py`, **`graphical/renderers/renderer.py`**) + one implementation file per mechanism — see [pluggable_implementations.md](pluggable_implementations.md).
4. **Inheritance ladders** (`MechanicalSystem`, …): **`dynamics/abstractions/`** only — not the same pattern as backends.
5. **Optional heavy deps:** SymPy (`symbolic/`), Gym (`tools/`), optional renderers (`graphical/`), future engines (`physics/`, `tools/`) → **`pyproject` extras**.

---

## Package map

| Package | Role |
|---------|------|
| **`core/`** | Diagram algebra, ports, `System` hierarchy, `Trajectory`, **`costs.py`**, **`sets.py`**, **`blocks/`**. |
| **`symbolic/`** | CAS; must not be required for `import minilink.dynamics`. |
| **`dynamics/`** | **`abstractions/`** + **`catalog/`**. |
| **`physics/`** | Engine-backed worlds; peer of `dynamics/`. |
| **`compile/`** | **`compiler.py`**, **`execution_plan.py`**, **`evaluators/`**, **`jax_utils.py`**, **`evaluator_timing.py`**. |
| **`simulation/`** | **`simulator.py`**, **`solvers/`**, **`input_interpolation.py`**, **`integration_timing.py`**, **`scenarios/`**. |
| **`estimation/`** | Observers, filters, sensor models. |
| **`control/`** | Feedback laws, RL policy wrappers as `StaticSystem`. |
| **`planning/`** | `PlanningProblem`, `planner.py`, families (`search/`, `trajectory_optimization/`, `policy_synthesis/`, …). |
| **`optimization/`** | `MathematicalProgram`, `optimizers/`. |
| **`analysis/`** | Diagnostics as computations (arrays, structured results). |
| **`graphical/`** | Plotting, animation, **`renderers/renderer.py`** + backends. |
| **`tools/`** | External ecosystem bridges — not internal JAX compile helpers. |

---

## `analysis/` vs `graphical/`

- **`analysis/`** — eigenvalues, phase fields, Bode data, etc.
- **`graphical/`** — how figures and animations are produced; can consume **`analysis/`** outputs.

If a routine is mostly plotting with no reusable math, it belongs in **`graphical/`**.

---

## Plot backends & diagram export — defer

Multi-backend **`plotters/`** (e.g. Matplotlib + Plotly) and multi-export **`diagram_exporters/`** are **deferred** until a second backend is real. Keep **`plotting.py`** and **`graphe.py`** as single-stack until then.

---

## `estimation/` vs `planning/` vs `core/`

- **`planning/`** uses **costs** and **sets** from **`core/`** for trajectory constraints.
- **`estimation/`** owns state reconstruction from measurements; avoid duplicating filters under **`planning/`**.

---

## `control/` vs `planning/` (RL)

- **Training / synthesis** → `planning/policy_synthesis/` (or a dedicated RL family later).
- **Deployed policy as a block** → `control/`.

---

## Pyro alignment (conceptual)

- Pyro **`dynamic/*`** → **`core/blocks`** + **`dynamics/abstractions`** + **`dynamics/catalog`**.
- Pyro **`analysis/*`** → **`analysis/`** + **`graphical/`** + **`simulation/`** + **`core/costs`** / **`core/sets`**.
- Pyro **`planning/*`**, **`control/*`**, **`tools/*`** → same-named packages here.

---

## Target vs repository today (migration checklist)

The tree above is **target**, not necessarily current `main`.

**Transitional / to remove**

- **`mechanics/`** — fold into **`dynamics/abstractions/`** + **`symbolic/`**.
- **`benchmark/`** — split per “Retiring benchmark” above.
- **Top-level `minilink/jax_utils.py`** — supersede with **`compile/jax_utils.py`** (brief re-export OK).
- **`planning/costs.py`**, **`planning/sets.py`** → **`core/costs.py`**, **`core/sets.py`**.

**Compile migration**

- Move **`compile/evaluator.py`**, **`numpy_evaluator.py`**, **`jax_evaluator.py`** into **`compile/evaluators/`** (keep **`numpy_evaluator.py`** / **`jax_evaluator.py`** names).

**Graphical**

- **`renderers/base.py`** → **`renderers/renderer.py`**.

**Dual pygame**

- **`pygame_renderer.py`** — drawing / animation.
- **`tools/`** (future) — interactive game loop / joystick bridge.

**`simulation/scenarios/`**

- If **`compile/evaluator_timing`** needs compile-only graphs later, optional **`compile/scenarios/`** — not required initially.

**`core/` note**

- “No solvers in core” means **no IVP stack** and **no evaluators** — not “no NumPy.” **`costs`** / **`sets`** may use NumPy.

**Repo root**

- **`examples/`**, **`tests/`** — outside **`minilink/`**; not shown in the tree.

**Placeholders**

- **`estimation/`**, **`analysis/`**, **`tools/`** — add when first modules land.

---

## Documentation maintenance

Update **DESIGN.md** §2 (package map) when the on-disk tree converges. Use **ROADMAP.md** for sequencing work, not for duplicating this full tree.

---

## Why this layout

- Clear split: analytic **`dynamics/`** vs engine **`physics/`** vs CAS **`symbolic/`**.
- **`compiler.py`** is the only “compiler” orchestration name; no **`compilers/`** folder.
- Timing lives next to what it measures: **`evaluator_timing`** vs **`integration_timing`**.
- **`estimation/`** and **`analysis/`** avoid overloading **`control/`** or **`graphical/`**.
