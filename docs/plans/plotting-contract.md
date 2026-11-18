# One plotting contract

**Status:** ruled (2026-11-18). The design of 2026-11-15 (an audit of the code and two reviews,
one independent), the census corrected against the code (2026-11-17), and the maintainer's
rulings of 2026-11-18, taken one at a time (§7). The run of §8 implements it.
**Where it sits:** infrastructure, any release (CONSTITUTION §6, the infrastructure exemption);
it replaces the "One plotting contract" row of [TODO.md](TODO.md) and the keyword half of
"One plot vocabulary" (S67).
**Lane:** agent-managed (the plotting interfaces); the public names below are ruled.
**Guiding rule (the maintainer, 2026-11-18):** the long-term shape over compatibility. The
course material is safe on `main`, which receives this work after the term; the teaching
notebooks that need a change are listed for the maintainer when it lands.

---

## 1. The goal

1. **The library never depends on a renderer.** A tool computes arrays and core objects; a plot
   function builds a description of the figure; a renderer draws the description. Matplotlib
   and Plotly are imported only inside their renderer modules.
2. **Matplotlib and Plotly draw everything.** A new figure kind is written once, as a
   description; each renderer draws every layer kind.
3. **One user API.** Every `plot_*` takes `renderer=`, `show=`, `ax=` and returns a
   `PlotResult`; every animation takes `renderer=`, `show=`, `file_name=` and returns an
   `AnimationResult`. A student who has drawn a Bode plot knows how to draw a cost-to-go.
4. **Plots compose.** A second plot draws into the first one's panel (a trajectory on a
   cost-to-go, a region of attraction on a phase plane), on both renderers.
5. **`show=True` means shown now, where the user is.** In a script, a notebook, a function or
   a long cell, every figure appears at the line that draws it, in code order.

The import test of goal 1 fails today in 16 files (45 import statements under `minilink/`):
37 sites in the 12 drawn-directly files, and eight in `graphical/phase_plane/phase_plane.py`,
`graphical/port_map.py`, `graphical/common/environment.py` and `graphical/diagrams/dot.py`.

## 2. What exists (measured on `dev-consolidation`)

75 `plot_*` and `animate*` functions, about 12 000 lines of drawing code, in three tiers:

| Tier | Plots | Description | Renderers |
| --- | --- | --- | --- |
| **A. Described** | Bode, pole-zero, root locus, Nyquist, step response | `ControlFigure` | matplotlib, plotly |
| | time signals (`plot_trajectory`, `plot_rollout`, `plot_cost`), live signal plots | `SignalPlotSpec` | matplotlib, plotly |
| | phase plane | `PhasePlaneSpec` | matplotlib, plotly |
| | input-output map, control law of a block | `PortMapSpec` | matplotlib |
| **B. Drawn directly** | DP cost-to-go and control law, the planning solution's plots, `Comparison`, region of attraction, scene / track / cost field, search tree, MPC futures, learning curves, gain schedule | none | matplotlib |
| **C. Own pipelines** | system animations | scene primitives, frames | matplotlib, plotly, meshcat, pygame |
| | DP and search animations, search live plot | none | matplotlib |
| | block diagrams | graphviz | graphviz |

The keywords and returns of the drawn-directly tier, as the code stands:

| Functions | `renderer=` | `ax=` | `show=` | Returns |
| --- | --- | --- | --- | --- |
| DP `plot_cost_to_go`, `plot_control_law`, `plot_value`, `plot_field`, `plot_control_law_field` | no | yes | yes | `(fig, ax)`, a list for several slices |
| `plot_scene`, `plot_track`, `plot_cost_field`, `plot_cost_field_3d`, `plot_tree`, `plot_sampled_futures` | no | yes | yes | `(fig, ax)` |
| the two `plot_learning_curve`, the three `plot_gain_schedule` | no | yes | yes | the axes |
| `animate_search`, `animate_convergence` | no | yes | yes | the animation |
| `plot_region_of_attraction` | no | yes | yes | `PlotResult` |
| `plot_value_3d`, the three DP `animate*` | no | no | yes | `(fig, ax)`, the animation |
| `PlanningSolution.plot_cost_to_go`, `plot_control_law` and the planner mirrors | yes (`require_matplotlib`) | yes | yes | `PlotResult` |
| `Comparison` plots | no | no | yes | `PlotResult` |
| `animate` on a system | yes | no | yes | the renderer's object, or `None` (§4.2) |
| `plot_diagram` | no | no | yes | the Graphviz object, or `None` |

The show itself is 14 `plt.show` calls in 11 files, each with its own blocking rule.

Two measurements that size the work: tier B's drawing is about 170 matplotlib calls over 10
files, of a narrow vocabulary (`plot`, `scatter`, `pcolormesh` / `imshow`, `contour(f)`,
`fill` / `add_patch`, `plot_surface`, `quiver` / `streamplot`, labels, limits, legend, colour
bar); the rest of its 3 000 lines is data preparation, which stays. The tests assert on
matplotlib artists (65 to 91 assertions in 12 files); they move to the description over time.

## 3. The design

```
tool (arrays, core objects)  →  plot function  →  FigureSpec  →  render(spec, renderer=)  →  PlotResult
                                 builds layers      plain data      matplotlib | plotly          figure, axes, spec
```

### 3.1 One description: figure → panels → layers

`FigureSpec`, `Panel`, the layers and `render`, in `minilink/graphical/figure.py`, public in
`minilink.graphical`, not in the top-level namespace (D2). `ControlFigure` becomes `FigureSpec`.

A `FigureSpec` is a title and a grid of panels (`n_cols`, shared axes, an optional shared colour
bar). A `Panel` holds the axes labels, limits, scales, `equal_aspect`, `legend`, `is_3d`, and an
ordered list of layers:

| Layer | Draws | Used by |
| --- | --- | --- |
| `Trace` | a polyline or a point set (`mode`), NaN breaks a line; dash, arrows | time plots, Bode, poles and zeros, trajectories, learning curves, the search tree |
| `RefLine` | a horizontal or vertical reference | margins, set-points, limits |
| `Note` | text anchored in the panel | margins, step figures |
| `Mesh` | values on a regular 2-D mesh: heatmap, level sets or 3-D surface (`draw=`) | cost-to-go, control law, port map, cost field, region of attraction, clearance |
| `Fill` | a closed polygon | obstacles, a corridor, a region |
| `Arrows` | a vector field, quiver or stream | phase plane |

`Mesh` is one name or three (`Heatmap`, `Contour`, `Surface`) by the spike's measure (§8, step
2): one name if the fields mostly coincide, three if `draw=` would switch half of them off.

**The format stays small by rule.** A field enters a layer only when a plot in the library
needs it and both renderers honour it (or ignore it without changing the figure's meaning).
No pass-through of renderer keywords: a renderer-specific finish is the caller's own call on
`result.axes` or `result.figure`. Per-plot style keywords nobody passes are removed (§7, ruling 4).

### 3.2 One renderer call

```python
render(spec, *, renderer="matplotlib", show=True, ax=None) -> PlotResult
```

A renderer is one module with a draw function per layer kind, the panel and figure setup, and
the shared show step (§4.1). The renderers sit in a name-to-module table read lazily. Both
renderers draw all six layer kinds (ruling 8): `Mesh` as `go.Heatmap`, `go.Contour`,
`go.Surface`; `Fill` as a filled `Scatter`; `Arrows` as segments with heads, and the stream
style ported from the phase plane's Plotly code.

### 3.3 The plot record

```python
@dataclass
class PlotResult:
    renderer: str
    figure: Any       # the drawn figure: matplotlib Figure, plotly Figure, graphviz Digraph
    axes: Any         # matplotlib: the Axes of a one-panel figure, a list in panel order
                      # for several; None on plotly and graphviz
    spec: FigureSpec  # what was drawn, overlays included; None on graphviz
```

`payload` goes (D4). `PlotResult` does not unpack as a pair: callers read `result.figure` and
`result.axes`, and the scripts that unpack `fig, ax = …` change (ruling 1). A multi-slice call
returns a list of records; a live plot returns its handle. The record never displays itself
(`_ipython_display_` shows nothing): the figure was shown when it was drawn (§4.1).

### 3.4 Composition: `ax=` takes a matplotlib `Axes` or a `PlotResult` (D5)

```python
fig, ax = plt.subplots()                         # the caller's own axes
scene.plot(ax=ax)                                # draws as today, labels and title included

result = sol.plot_cost_to_go(show=False, renderer="plotly")
plant.plot_phase_plane(traj, ax=result)          # adds its layers to result's panel
```

- **Into an `Axes`:** the plot draws in full (a caller passes `title=""` to keep its own).
- **Into a `PlotResult`:** the plot adds its layers only; the host keeps its title, labels,
  limits and scales; the layers are appended to `result.spec` and drawn on its figure; the
  renderer follows the host (a disagreeing `renderer=` is refused).
- **Refused, by name:** a multi-panel plot into a one-panel host and the reverse; a 3-D layer
  into a 2-D panel; a second colour-scaled `Mesh` in a panel with a colour bar; a Plotly figure
  the user built by hand (no description to append to).
- **Legends:** an overlay's entry that repeats a host label is not added twice.
- Every call but the last passes `show=False`; the docstring of `ax=` says so.
- Inside the library, the internal `ax=` calls become panel assembly (`Comparison` builds one
  `FigureSpec` of `n × m` panels). The host's overlay keywords (`trajectory=`,
  `trajectories=`) stay as shortcuts that add the same layers.

## 4. Showing, animations and diagrams

### 4.1 Where a figure is shown

One resolver in `graphical/common/environment.py`, `resolve_show(show, renderer)`, used by
every plot, animation and diagram (ruling 6c). It replaces `prefers_inline_animation`,
`is_inline_capable` and the direct `"google.colab" in sys.modules` checks. `show=` takes
`True`, `False`, `"inline"` or `"window"`; `True` picks the place:

| `show=True` | script / IPython | Jupyter | Colab |
| --- | --- | --- | --- |
| matplotlib | window (blocks in a script) | inline if the backend is inline, else its window or widget | inline |
| plotly | browser tab | inline | inline |
| meshcat | browser viewer | inline (self-contained page) | inline |
| pygame | window | window | refused by name |
| graphviz | the vector PDF in the system viewer | inline SVG | inline SVG |

- Only matplotlib reads matplotlib's backend; the other renderers decide on the environment.
- `"inline"` where nothing displays it, and `"window"` where there is no window (Colab, a
  server), raise by name.
- **Shown at call time, in code order** (ruling 6e (i)): in a notebook every figure, matplotlib
  included, is displayed at the line that draws it (`display`, then closed for matplotlib), and
  the returned record does not display again. A change made after a shown call needs
  `show=False` first, as in a script.
- One show step holds the blocking rule (a bare script blocks on the last window).

### 4.2 Animations

Every animation (`animate` on a system, a sampled loop and a planner; the DP and search
animations) takes `renderer=`, `show=`, `file_name=` and returns an `AnimationResult`
(ruling 6):

```python
@dataclass
class AnimationResult:
    renderer: str
    figure: Any       # matplotlib Figure | plotly Figure | meshcat Visualizer | None (pygame)
    animation: Any    # FuncAnimation | meshcat Animation | None (plotly: frames in figure)

    def save(self, file_name): ...   # .gif on matplotlib, .html on meshcat and plotly
```

- Shown at call time where `show` resolves (§4.1), the record never displays again; every path
  returns a record, `show=False` included.
- `html=` goes for `show="inline"`; `save=True` goes: `file_name=` writes the file, defaulting
  to `None`, no file (ruling 6b).
- `native=` goes (ruling 6d): an animation always plays a clip (matplotlib `FuncAnimation`,
  Plotly frames, the meshcat keyframes); pygame, which has no clip, always plays live. The
  frame-by-frame loop stays inside the live paths (`game()`, `RealtimeSimulator`). Meshcat's
  clip draws changing geometry (arrows, a horizon, sampled trajectories, trails) as a flipbook
  of pages switched by a `visible` track, and keyframes the world slide of a following camera.
  Measured 2026-11-18 (this container, meshcat clip build and the per-frame draw cost):

  | Scene | Pages | Clip build, size | Frame by frame vs. its interval |
  | --- | --- | --- | --- |
  | formula car, contact force arrows | 2 676 | 4.7 s, 13.8 MB | 54 ms vs 30 ms |
  | MPC horizon (`mpc_racecar_kinematic.py`) | 468 | 2.4 s, 7.5 MB | 43 ms vs 30 ms |
  | MPPI samples (`mppi_racecar.py`) | 3 768 | 7.2 s, 19.5 MB | 61 ms vs 30 ms |

  The clips played correctly in headless Chromium (screenshots at two times); the per-frame
  loop runs slower than real time on all three.
- `animate_modal` takes the same keywords and returns an `AnimationResult`; the poles and
  modes are read from the modal analysis, and its callers change.
- The live paths (`game`, the realtime simulator, live plots) are sessions, not results.

### 4.3 Block diagrams (ruling 6e)

- `plot_diagram(sys, *, file_name=None, show=True, expand=True)`, every keyword keyword-only,
  returns a `PlotResult(renderer="graphviz", figure=<Digraph>)`, shown at call
  time: inline SVG in a notebook (it was shown only as a cell's last expression), the vector
  PDF in the system viewer elsewhere. `show="pdf"` and the matplotlib PNG window go;
  `"window"` names the viewer.
- The viewer's file is one PDF per diagram in a fixed temp folder
  (`<tmp>/minilink/<name>.pdf`), overwritten; no `.gv` source beside it.
- `file_name=` writes exactly that file in the format of its extension (`.pdf`, `.svg`,
  `.png`; PDF without one). It wrote the DOT source under the given name and the PDF beside it.
- Without the Graphviz binary: a warning and a record holding the `Digraph`.

## 5. What this plan leaves out

- A theme or semantic-colour system: the colours stay in the plots (D7).
- Playing a list of descriptions as an animation; an `AnimationResult.spec` until a caller
  needs it.
- A public "build your own figure" tutorial.
- Any change to what a plot shows: each figure keeps its data, labels and limits.

## 6. Housekeeping found by the audit

- S67 "One plot vocabulary" keeps its rows outside this contract (the output-port selection
  of `plot_trajectory`, the axis-label formatter and unit convention, the silent renderers,
  the phase-plane window warning, the region plot's `sys.x0`).
- QUESTIONS, "What the check of the core notebooks found in the API", item 15 (`Comparison`
  without `renderer`, `ax`, `title`) is answered by this contract; item 11 (`plot_diagram`'s
  positional `show`) is ruled: keyword-only (§4.3).
- Building the animation frames dominates an animation's cost (22 s on the Stanley lap, 40 s on
  the kinematic MPC, 120 s on MPPI, before either playback); a TODO row profiles it.

## 7. Rulings (2026-11-18)

1. Callers read `result.figure` and `result.axes`; `PlotResult` does not unpack.
2. **D3.** `figure` is the drawn figure, `spec` the description, `axes` the `Axes` for one
   panel and a list for several. **D4.** `payload` is removed.
3. **D1.** The whole design in one run, behind the spike's gate (§8, step 2).
4. **D2.** `FigureSpec`, `Panel`, the layers and `render`, public in `minilink.graphical`.
5. **D5.** `ax=` takes a matplotlib `Axes` or a `PlotResult`.
6. The per-plot style keywords with no caller go; `figsize` and `cmap` stay;
   `waypoint_color="none"` becomes `show_waypoints=False`.
7. `plot_cost_field` samples its costs; `FieldGrid` and `sample_field_costs` go.
8. **6.** Animations return an `AnimationResult`. **6b.** `show="inline"` replaces `html=`;
   `file_name=` replaces `save=`. **6c.** The resolver of §4.1. **6d.** `native=` goes.
   **6e.** The diagram of §4.3; every figure shown at call time, matplotlib included.
9. **D6.** The time signals and the phase plane merge onto `FigureSpec` in the run, behind
   the stop rule of §8, step 5.
10. Both renderers draw all six layers, the stream arrows included.
11. `animate_modal` returns an `AnimationResult` as every animation; `plot_diagram`'s
    keywords are keyword-only. Public signatures change freely: every caller is updated.
12. The run is on the branch `plotting-contract`, with a pull request into
    `dev-consolidation`.

## 8. The run

One branch, one commit per item, the gates at the end. **Baseline first:** a seeded record of
every figure's matplotlib artists (kind, count, data arrays, labels, limits) and of every
animation's frames, per family, run twice and compared byte for byte; each behaviour-preserving
step reproduces it. A deliberate regrouping (the search tree's edges as one broken line)
compares the data drawn, not the artist count, and says so in its commit.

1. **The record and the show.** `PlotResult` of §3.3; `resolve_show` and one show step (§4.1);
   call-time display in notebooks; `require_matplotlib` and `plot_result_of` into
   `graphical/common/`; the drawn-directly functions take `renderer=` and return a record;
   their callers in `examples/` and `tests/` read the record; the phase plane's renderers out
   of its description module; one contract test over every public `plot_*` (keywords, record).
2. **Animations and diagrams.** `AnimationResult`; `show=`, `file_name=`; `html=`, `save=`,
   `native=` removed and their callers changed; the DP and search animations; `plot_diagram`
   as §4.3. A notebook test (nbclient) checks call-time display: a diagram, an animation and a
   plot in one cell, in code order.
3. **The spike and its gate.** `Mesh` in `FigureSpec`; `plot_cost_to_go` drawn through it on
   both renderers; a trajectory overlaid with `ax=result` on both. The run continues only if:
   the matplotlib figure draws the same data as the baseline; render time at most 20 % above
   today's (median of 5); `Mesh` needs 12 fields or fewer; each renderer gains 150 lines or
   fewer for it. Otherwise it stops after step 2 with the spike's report.
4. **The families onto `FigureSpec`**, one commit series each: the mesh family (DP, the
   planning solution, `Comparison`, the port map); the region of attraction; the learning
   curves and the gain schedule; the search tree; the MPC futures; the spatial plots (rulings
   6 and 7). Each takes the contract, loses its matplotlib import and gets its Plotly test.
   After this step the import test of §1 passes, `environment.py`'s backend read allowed.
5. **The merge**, last: the time signals and the phase plane onto `FigureSpec`; the live handle
   updates its `Trace` layers. Stops and reports, keeping steps 1 to 4, if a figure record
   moves, a live-plot test fails, or a live update is more than 20 % slower (median of 5).
6. **Docs:** DESIGN (the plotting contract, the call chains), README where it shows a plot's
   return, the TRL ledger; the TODO rows leave.
7. **Review and gates:** a review pass; two independent reviewers (correctness, architecture);
   ruff; the full `pytest -n auto`; the demo-check bridge on its own; the affected demos; the
   notebook check on `examples/`; the docs build.

**Stops and asks:** a baseline that moves in a behaviour-preserving step; a gate of step 3 or 5;
a public name not covered by §7; a speed loss against `dev` beyond noise that cannot be fixed.
Every other open choice takes the recommended option and is logged below.

**For the maintainer at the end:** the teaching notebooks in `alx87grd/teaching` that unpack a
plot's return, pass `html=`, `save=`, `native=` or `show="pdf"`, or adjust a figure after a
shown call.

### Decisions taken during the run

(none yet)
