# Minilink Flow Map

Minimal call chains for debugging. User workflows: [README.md](README.md).
Contracts: [DESIGN.md](DESIGN.md).

## Package roles

| Package | Owns |
| --- | --- |
| `core` | `System`, `DiagramSystem`, ports, `Trajectory`, sets, costs |
| `compile` | `ExecutionPlan`, `DynamicsEvaluator` |
| `simulation` | `Simulator`, solvers, time grids |
| `graphical` | plots, diagrams, animation (`Animator` + renderers) |
| `planning` | `PlanningProblem`, planners, transcriptions |
| `optimization` | `MathematicalProgram`, `Optimizer` |

## Main chains

```text
Model:     subclass System → f/h (+ ports or DynamicSystem options)

Compose:   + / >> / @ / autowire  →  DiagramSystem
           or add_subsystem + connect (+ connect_new_output_port)

Simulate:  compute_trajectory*  →  Simulator  →  compile  →  solve  →  Trajectory
           compute_forced*      →  Simulator.solve_forced
           (* stores sys.traj)

Compile:   sys.compile(backend)  →  DynamicsEvaluator

Plot:      plot_trajectory*  →  graphical.signals  →  PlotResult
           plot_phase_plane* →  graphical.phase_plane
           plot_diagram      →  graphical.diagrams (Graphviz/Mermaid)

Animate:   animate* / render / game  →  Animator  →  renderer backend
           planner.plot_solution / animate_solution  →  problem.sys.*

Trajopt:   PlanningProblem + Transcription + TrajectoryOptimizationPlanner
           → transcribe → MathematicalProgram → Optimizer → Trajectory

NLP:       MathematicalProgram → Optimizer → OptimizationResult
```

## Notes

- `Trajectory` is numeric only (`t`, `x`, `u`, optional `signals`); labels stay on `System`.
- Diagram internal signals in plots: `"subsystem_id:port_id"`.
- `DiagramSystem.connection_verbose` defaults to `True`; set `False` to quiet wiring.
- Shortcuts do not merge two diagrams with `+` or nest diagrams with `>>`.
- `compute_*` returns `Trajectory`; `plot_*` returns `PlotResult`; `show=False` skips display.
