# minilink

`minilink` is a Python-native block-diagram framework for modeling,
compiling, simulating, optimizing, and visualizing dynamical systems.

![diagram](https://github.com/user-attachments/assets/b5c2c740-ae0b-42ab-afba-e90f2dd92a26)

Colab demo: https://drive.google.com/file/d/1eMrC_8h1iZbq6lMvk4e68M6YysupJ7dg/view?usp=sharing

## What It Is

- **Readable dynamical systems**: write equations as `f(x, u, t, params)` and
  outputs as `h(x, u, t, params)`.
- **Named-port diagrams**: compose MIMO blocks with explicit input and output
  ports.
- **Compiled evaluators**: turn leaves and diagrams into flat NumPy or JAX
  `DynamicsEvaluator` objects through `ExecutionPlan`.
- **Simulation and trajectories**: use `Simulator` or `System.compute_trajectory`
  to produce the canonical `Trajectory(t, x, u, signals)`.
- **Optimization and planning**: describe finite-dimensional NLPs with pure
  `MathematicalProgram` objects; trajectory optimization transcribes planning
  problems into that same optimization layer.
- **Visualization**: plot, animate, render, and inspect diagrams through the
  optional graphical layer.

## Quick Start

```python
from minilink.core.blocks.basic import Integrator
from minilink.core.blocks.sources import Step
from minilink.core.diagram import DiagramSystem
from minilink.graphical.plotting import plot_trajectory
from minilink.simulation.simulator import Simulator

diagram = DiagramSystem()
diagram.add_subsystem(Integrator(), "plant")
diagram.add_subsystem(Step(), "source")
diagram.connect("source", "y", "plant", "u")

traj = Simulator(diagram, tf=10.0).solve()
plot_trajectory(diagram, traj)
```

## Install

Core install:

```bash
pip install minilink
```

Optional extras:

```bash
pip install "minilink[jax]"            # JAX evaluators, JIT, autodiff
pip install "minilink[visualization]"  # meshcat / pygame renderers
pip install "minilink[symbolic]"       # SymPy mechanics helpers
pip install "minilink[ipopt]"          # Ipopt optimizer adapter
```

## Current Design Rules

Short summary—details and edge cases are in [DESIGN.md](DESIGN.md).

- **NumPy baseline, JAX optional**: choose backends explicitly (`compile_backend`,
  evaluator backends); vocabulary in `minilink.compile.backend_policy`.
- **Native-array equation paths** (`System`, ports, sets/costs, programs):
  preserve the active backend; Python floats and forced NumPy conversion belong at
  boundaries (evaluators, solvers, plotting, sampling helpers).
- **Prefer one readable class** for simple algebra; optional `Jax<X>` twins only
  when one implementation cannot stay traceable for both backends.
- **`params is None`** uses object defaults; any other `params` value overrides—
  never treat empty dicts as falsy for “use defaults.”

## Main Packages

- `minilink.core`: `System`, `DiagramSystem`, ports, `Trajectory`, sets, costs,
  and basic blocks.
- `minilink.compile`: `ExecutionPlan`, backend policy, NumPy/JAX dynamics
  evaluators.
- `minilink.simulation`: `Simulator`, solver backends, forced-input utilities,
  simulation benchmarks.
- `minilink.optimization`: pure `MathematicalProgram`, program evaluators, and
  optimizer method presets such as `scipy_slsqp`, `scipy_trust_constr`, and
  `ipopt`.
- `minilink.planning`: deterministic planning problems, initial guesses, search,
  policy synthesis, and trajectory optimization.
- `minilink.dynamics`: reusable plant abstractions and catalog models.
- `minilink.graphical`: plotting, animation, graph display, and renderer
  backends.

## Documentation

- [DESIGN.md](DESIGN.md): current architecture and public contracts.
- [ROADMAP.md](ROADMAP.md): active priorities and subsystem maturity.
- [agent.md](agent.md): maintainer and AI-agent contribution rules.

## Examples And Benchmarks

- `examples/scripts/`: runnable demos for diagrams, compilation, animation, and
  JAX physics.
- `tests/manual/`: one-off smoke tests and exploratory checks.
- `tests/benchmark/`: flat benchmark runners using subsystem-local benchmark
  helpers such as `minilink.simulation.benchmark` and
  `minilink.optimization.benchmark`.
