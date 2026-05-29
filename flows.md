# Minilink Flow Map

This file maps the main user-facing flows to the objects and functions that
actually execute them. It is intentionally implementation-facing: when a call
path feels indirect, start here before jumping through modules.

For the recommended user-facing workflow and import paths, see
[README.md](README.md). Architecture contracts live in [DESIGN.md](DESIGN.md).

## Package Boundaries

```text
minilink.core
  Owns System, DiagramSystem, ports, state metadata, Trajectory, sets, costs.

minilink.compile
  Turns a System or DiagramSystem into a DynamicsEvaluator.

minilink.simulation
  Owns time grids, solver selection, and Trajectory generation.

minilink.graphical
  Owns views of core objects for plotting, diagram display, and animation.
  Subpackages:
    common/       environment, styles, PlotResult
    signals/      time-signal plotting
    phase_plane/  phase-plane plotting
    diagrams/     diagram topology export/display
    animation/    primitives, Animator, renderer lifecycle

minilink.planning
  Owns planning problem orchestration and delegates plotting/animation back
  to the problem system.
```

Core data objects:

- `System`: equations, ports, state/input/output metadata, graphical hooks, and
  convenience facades.
- `DiagramSystem`: a `System` composed from named subsystems and port
  connections.
- `DynamicsEvaluator`: compiled executable view of either a leaf system or a
  diagram.
- `Trajectory`: numeric sampled result, shaped as `t`, `x`, `u`, and optional
  named `signals`.
- `PlotResult`: graphical result wrapper returned by plot calls.
- `SignalPlotSpec`, `PhasePlaneSpec`, `DiagramTopology`: backend-neutral views
  built from core objects before rendering/export.

## 1. Leaf System Definition

Typical user code:

```python
class Plant(DynamicSystem):
    def f(self, x, u, t, params=None):
        ...

    def h(self, x, u, t, params=None):
        ...
```

Flow:

```text
User subclass
  -> System / DynamicSystem / StaticSystem __init__()
  -> add_input_port(), add_output_port(), state metadata setup
  -> user implements f(), h(), or output-port compute functions
  -> optional graphical hooks:
       get_kinematic_geometry()
       get_kinematic_transforms(x, u, t)
       get_dynamic_geometry(x, u, t)
       get_camera_transform(x, u, t)
```

Important object ownership:

- `System` remains the source of truth for dimensions, labels, units, bounds,
  defaults, and params.
- Graphical modules read from `System`; they do not store model data back into
  `System` except through the ordinary `System.traj` simulation cache.

## 2. Diagram Construction

Manual wiring:

```text
DiagramSystem()
  -> add_subsystem(system, sys_id)
  -> add_input_port() / add_output_port()
  -> connect(source_sys_id, source_port_id, target_sys_id, target_port_id)
  -> connect_new_output_port(...)
  -> refresh()
```

Shortcut wiring:

```text
sys_a + sys_b
  -> minilink.core.composition.add_systems()
  -> DiagramSystem with independent subsystems

sys_a >> sys_b
  -> minilink.core.composition.series()
  -> DiagramSystem with output-to-input wiring

controller @ plant
  -> minilink.core.composition.closed_loop()
  -> DiagramSystem with r/y/u closed-loop convention

diagram.autowire(strict=False)
  -> minilink.core.composition.autowire()
  -> fills unconnected inputs when exactly one safe source matches
```

Notes:

- `DiagramSystem.connection_verbose` defaults to `True` and prints each
  `connect()` call. Set it to `False` for quiet wiring in scripts.
- Shortcuts do not merge two existing diagrams with `+` or connect into a
  nested `DiagramSystem` with `>>`.

Objects produced:

- A `DiagramSystem` containing `subsystems`, `connections`, diagram boundary
  ports, and `state_index` slices into the flattened state vector.
- The diagram is still a `System`, so all simulation, plotting, compilation,
  and animation facades work through the same object-level methods.

## 3. Compilation

User call:

```python
evaluator = sys.compile(backend="numpy")
```

Leaf system flow:

```text
System.compile()
  -> minilink.compile.compiler.compile(system, backend)
  -> NumpyLeafEvaluator(system)
     or JaxLeafEvaluator(system)
  -> DynamicsEvaluator
```

Diagram flow:

```text
DiagramSystem.compile()
  -> minilink.compile.compiler.compile(diagram, backend)
  -> compile_diagram(diagram, backend)
  -> check_algebraic_loops(diagram)
  -> _build_execution_plan_from_order(diagram, port_execution_order)
  -> ExecutionPlan
  -> NumpyDiagramEvaluator(plan, diagram)
     or JaxDiagramEvaluator(plan, diagram)
  -> DynamicsEvaluator
```

Runtime evaluator responsibilities:

- Leaf evaluators wrap one system's `f`, `h`, params, and nominal input data.
- Diagram evaluators execute the `ExecutionPlan`: external inputs, nominal
  values, internal port signals, subsystem state slices, boundary outputs, and
  state derivatives.

## 4. Standard Simulation

Primary user call:

```python
traj = sys.compute_trajectory(tf=10.0, dt=0.01)
```

Lower-level equivalent:

```python
from minilink.simulation.simulator import Simulator
traj = Simulator(sys, tf=10.0, dt=0.01).solve()
```

Flow:

```text
System.compute_trajectory(...)
  -> Simulator(sys, t0, tf, n_steps, dt, solver, compile_backend)
      -> sys.refresh()
      -> select_time_vector(...)
      -> _resolve_and_build_evaluator(sys, compile_backend)
          -> sys.compile(...)
      -> select_solver(sys, solver)
      -> _select_backend("scipy" | "euler" | "rk4")
  -> Simulator.solve()
      -> solver_backend.solve(evaluator, time grid, x0, nominal/default u)
      -> Trajectory(t, x, u, signals)
  -> sys.traj = traj
  -> optional show=True:
      -> minilink.graphical.signals.plot_time_signals(sys, traj)
  -> return Trajectory
```

Main objects:

- `Simulator` owns the time grid, solver backend, and compiled evaluator for
  one rollout.
- `Trajectory` is numeric-only sampled data. Labels, units, state names, and
  port metadata stay on `System`.

## 5. Forced-Input Simulation

User call:

```python
traj = sys.compute_forced(u, input_port_id="r", tf=5.0)
```

Flow:

```text
System.compute_forced(u, input_port_id, ...)
  -> Simulator(...)
  -> Simulator.solve_forced(u, input_port_id)
      -> validate full-system or single-port forced input
      -> sample callable/array input over Simulator.times
      -> solver_backend.solve(evaluator, time grid, x0, forced u)
      -> Trajectory(t, x, u, signals)
  -> sys.traj = traj
  -> optional show=True:
      -> minilink.graphical.signals.plot_time_signals(sys, traj)
  -> return Trajectory
```

## 6. Time-Signal Plotting

User calls:

```python
result = sys.plot_trajectory(signals=("x", "u"), backend="matplotlib")
result = sys.plot_trajectory(traj, signals=("plant:y",), backend="plotly")
```

Flow:

```text
System.plot_trajectory(traj=None, signals, backend, show)
  -> if traj argument is provided:
       use it
     elif self.traj exists:
       use self.traj
     else:
       compute_trajectory(show=False)
  -> minilink.graphical.signals.plot_time_signals(sys, traj, signals, backend)
      -> build_signal_plot_spec(sys, traj, signals)
          -> for "x":
               use traj.x + sys.state labels/units
             for "u":
               use traj.u + sys input labels/units
             for extra trajectory signal:
               use traj.get_signal(name)
             for diagram internal output "subsystem:port":
               reconstruct_internal_signals(traj) if needed
          -> SignalTrace for each scalar component
          -> SignalPlotSpec(title, t, traces)
      -> if backend == "matplotlib":
           render_matplotlib_signal_plot(spec, show)
         elif backend == "plotly":
           render_plotly_signal_plot(spec, show)
      -> PlotResult
```

No intermediate sampled container exists here. `Trajectory` is the sampled data
source; `SignalPlotSpec` is the only backend-neutral plotting view.

Live time-signal flow:

```text
open_time_signal_plot(sys, traj, signals, backend)
  -> build_signal_plot_spec(...)
  -> open_matplotlib_signal_plot(...) or open_plotly_signal_plot(...)
  -> LivePlotHandle

LivePlotHandle.update(next_traj, title=None)
  -> rebuild SignalPlotSpec through the captured spec_builder
  -> update existing backend lines/traces
```

## 7. Phase-Plane Plotting

User call:

```python
result = sys.plot_phase_plane(traj=None, x_axis=0, y_axis=1)
```

Flow:

```text
System.plot_phase_plane(traj=None, ...)
  -> if traj is None:
       use self.traj if available, otherwise no overlay
  -> minilink.graphical.phase_plane.plot_phase_plane(sys, traj, ...)
      -> normalize backend
      -> build_phase_plane_spec(sys, traj, ...)
          -> validate state dimension and selected axes
          -> choose x_ref and u defaults
          -> derive bounds from state metadata or trajectory overlay
          -> build grid X, Y
          -> evaluate sys.f(x_grid, u, t, params) at each grid point
          -> optional PhasePlaneTrajectoryOverlay from traj.x
          -> PhasePlaneSpec
      -> render_phase_plane_matplotlib(spec, streamplot, show)
      -> PlotResult
```

Current backend status:

- `backend="matplotlib"` is implemented.
- Other phase-plane backends intentionally raise a clear `ValueError`.

## 8. Diagram Display And Export

Object-level display:

```python
graph = sys.get_diagram()
graph = sys.plot_diagram(show_inline=False, show_pdf=False)
```

Flow:

```text
System.get_diagram() or DiagramSystem.get_diagram()
  -> minilink.graphical.diagrams.get_diagram(sys_or_diagram)
      -> export_diagram_topology(sys_or_diagram, backend="graphviz")
          -> build_diagram_topology(sys_or_diagram)
              -> DiagramTopology(nodes, edges)
          -> _resolve_topology_exporter("graphviz")
              -> GraphvizTopologyExporter from diagrams.dot
          -> graphviz.Digraph
  -> return graphviz.Digraph or None if graphviz is unavailable
```

Display flow:

```text
System.plot_diagram(...)
  -> minilink.graphical.diagrams.plot_diagram(sys_or_diagram, ...)
      -> get_diagram(sys_or_diagram)
      -> _render_diagram_graph(graph, show_inline, show_pdf, filename)
          -> inline display in notebook-capable environments
          -> optional graph.render(filename, view=show_pdf)
      -> return graph
```

Text export flow:

```text
export_diagram_topology(sys_or_diagram, backend="mermaid")
  -> build_diagram_topology(sys_or_diagram)
  -> MermaidTopologyExporter from diagrams.mermaid
  -> Mermaid flowchart source string
```

Naming rule:

- Public diagram calls use `diagram`: `get_diagram`, `plot_diagram`,
  `export_diagram_topology`.
- `graphviz` remains only the backend key and dependency name. The internal
  module for that backend is `minilink.graphical.diagrams.dot`.

## 9. Single-Frame Rendering

User call:

```python
result = sys.render(x, u, t, renderer="matplotlib")
```

Flow:

```text
System.render(x, u, t, is_3d, renderer)
  -> Animator(sys)
  -> Animator.show(x, u, t, is_3d, renderer)
      -> _make_renderer(renderer)
          -> MatplotlibRenderer | MeshcatRenderer | PlotlyRenderer | PygameRenderer
      -> sys.get_kinematic_geometry()
          -> list[Primitive]
      -> Animator._prepare_transforms(x, u, t, primitives)
          -> sys.get_kinematic_transforms(x, u, t)
          -> sys.get_camera_transform(x, u, t)
      -> renderer.open_scene(...)
      -> renderer.draw_frame(primitives, transforms, t, camera)
      -> renderer.present(block=True)
      -> renderer.close_scene()
      -> backend-specific result
```

Main ownership boundary:

- `System` owns what should be drawn: primitives, transforms, and camera.
- `Animator` owns playback orchestration.
- Renderer classes own backend lifecycle and drawing state.

## 10. Trajectory Animation

User call:

```python
animation = sys.animate(traj=None, renderer="matplotlib", html=None)
```

Flow:

```text
System.animate(traj=None, ...)
  -> if traj argument is provided:
       use it
     elif self.traj exists:
       use self.traj
     else:
       compute_trajectory()
  -> resolve html with prefers_inline_animation() if html is None
  -> Animator(sys)
  -> Animator.animate_simulation(traj, ...)
      -> _make_renderer(renderer)
      -> sys.get_kinematic_geometry()
      -> trajectory_frame_schedule(traj, time_factor_video)
      -> for each animation frame:
           sim_index_for_frame(...)
           traj.x[:, sim_idx], traj.u[:, sim_idx], traj.t[sim_idx]
           _prepare_transforms(...)
      -> if html:
           renderer.render_inline_animation(primitives, frames, schedule)
         elif save:
           renderer.export_animation(...)
         elif native:
           renderer.play_native(primitives, frames, schedule)
         else:
           renderer.open_scene()
           loop draw_frame() + present()
           renderer.close_scene()
      -> animation object, HTML object, or None depending on backend/output mode
```

Renderer lifecycle stays class-based here because animation backends maintain
state across many frames and sometimes own native playback engines.

## 11. Interactive Game Loop

User call:

```python
sys.game(renderer="pygame", dt=1 / 30)
```

Flow:

```text
System.game(...)
  -> Animator(sys)
  -> Animator.game(...)
      -> _make_renderer(renderer)
      -> _require_interactive_renderer(...)
      -> import pygame
      -> sys.get_kinematic_geometry()
      -> initialize x, u, t
      -> renderer.open_scene(...)
      -> while not quit:
           pygame events + keyboard state
           _u_from_keyboard(keys, m=sys.m)
           Euler integrate sys.f(x, u, t) with dynamics_substeps
           _prepare_transforms(x, u, t, primitives)
           renderer.draw_frame(...)
           renderer.present(...)
           renderer.poll_events()
      -> renderer.close_scene()
      -> pygame.quit()
```

Current limitation:

- Input is pygame-keyboard only.
- Integration is Euler in the animator loop.
- Future live input and integration backends should sit beside renderers, not
  inside `System`.

## 12. Trajectory Optimization

User call:

```python
planner = TrajectoryOptimizationPlanner(problem, transcription=..., options=...)
traj = planner.compute_solution()
```

Flow:

```text
TrajectoryOptimizationPlanner.compute_solution()
  -> transcription.transcribe(problem, compile_backend=...)
      -> MathematicalProgram
  -> pack_initial_guess(problem, guess)
  -> Optimizer(program, z0, method=..., compile_backend=...)
  -> Optimizer.solve(callback=...)
  -> transcription.unpack_solution(problem, result)
  -> Trajectory
  -> planner.last_result / iteration_history
```

Plotting and animation reuse the problem system's facades:

```text
planner.plot_solution(signals, backend)
  -> problem.sys.plot_trajectory(last_result, ...)

planner.animate_solution(**kwargs)
  -> problem.sys.animate(last_result, ...)
```

Live callback during optimization:

```text
LiveTrajectoryPlotCallback(iteration)
  -> open_time_signal_plot(problem.sys, iteration.trajectory, ...)
  -> LivePlotHandle.update(...)
```

## 13. Base Planner Delegation

All concrete planners inherit from `minilink.planning.planner.Planner`. After
`compute_solution()`, result plotting and animation delegate to the problem
system's normal facades:

```text
Planner.plot_solution(signals, backend)
  -> require_result()
  -> problem.sys.plot_trajectory(last_result, signals, backend)

Planner.animate_solution(**kwargs)
  -> require_result()
  -> problem.sys.animate(last_result, **kwargs)
```

Search and policy-synthesis planners follow the same delegation pattern once they
store a trajectory or policy result.

## 14. Optional Show Flags And Return Types

Common pattern:

```text
compute_* methods
  -> return Trajectory
  -> show=True may additionally plot, but plotting result is not returned

plot_* methods
  -> return PlotResult or backend render object

animate/render/game methods
  -> return backend-specific display/playback object or None
```

Concrete examples:

- `compute_trajectory(show=False)` returns `Trajectory`.
- `compute_trajectory(show=True)` still returns `Trajectory`.
- `plot_trajectory(show=False)` returns `PlotResult` without displaying.
- `plot_phase_plane(show=False)` returns `PlotResult` without displaying.
- `plot_diagram(show_inline=False, show_pdf=False)` returns the diagram object
  without displaying or writing a file.

