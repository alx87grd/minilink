# One plotting contract

**Status:** final plan (2026-11-18). The design of 2026-11-15 (an audit of the code and two
reviews, one independent), the census corrected against the code (2026-11-17), the
maintainer's rulings of 2026-11-18 taken one at a time, and two independent reviews of the
ruled plan (architecture; correctness and risk), whose findings the maintainer accepted (§7).
The run of §8 implements it.
**Where it sits:** infrastructure, any release (CONSTITUTION §6, the infrastructure exemption);
it replaces the "One plotting contract" row of [TODO.md](TODO.md) and the keyword half of
"One plot vocabulary" (S67).
**Lane:** agent-managed (the plotting interfaces); the public names below are ruled.
**Guiding rule (the maintainer):** the long-term shape over compatibility. Public signatures
change freely and every caller is updated. The course material is safe on `main`, which
receives this work after the term; the teaching notebooks that need a change are listed for
the maintainer when it lands.

---

## 1. The goal

1. **The library never depends on a renderer.** A tool computes arrays and core objects; a plot
   function builds a description of the figure; a renderer draws the description. Matplotlib
   and Plotly are imported only inside the renderer modules under `graphical/`.
2. **Matplotlib and Plotly draw every plot.** A new figure kind is written once, as a
   description; each renderer draws every layer kind.
3. **One user API.** Every `plot_*` takes `renderer=`, `show=`, `ax=` and returns a
   `PlotResult`; every animation takes `renderer=`, `show=` and returns an `AnimationResult`;
   both records `.save(file_name)` and `.show()`. A student who has drawn a Bode plot knows how
   to draw, overlay and save a cost-to-go.
4. **Plots compose.** A second plot draws into the first one's panels (a trajectory on a
   cost-to-go, two runs on one trajectory figure, two Bode plots), on both renderers.
5. **`show=True` means shown now, where the user is.** In a script, a notebook, a function or a
   long cell, every figure appears at the line that draws it, in code order; a headless run
   shows nothing.

**The import test.** No `import matplotlib` or `import plotly` under `minilink/` outside the
renderer modules of `graphical/`; the matplotlib-only modules (§4.4) are named in its
allow-list. It fails today in 16 files (45 import statements): 37 sites in the 12
drawn-directly files, and eight in `graphical/phase_plane/phase_plane.py`,
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
| **C. Own pipelines** | system animations, one frame (`sys.render`) | scene primitives, frames | matplotlib, plotly, meshcat, pygame |
| | DP and search animations, search and trajectory-optimization live plots | none | matplotlib |
| | block diagrams | graphviz | graphviz |

The keywords and returns of the drawn-directly tier, as the code stands:

| Functions | `renderer=` | `ax=` | `show=` | Returns |
| --- | --- | --- | --- | --- |
| DP `plot_cost_to_go`, `plot_control_law`, `plot_value`, `plot_field`, `plot_control_law_field` | no | yes | yes | `(fig, ax)`, a list for several slices |
| `plot_scene`, `plot_track`, `plot_cost_field`, `plot_cost_field_3d`, `plot_tree`, `plot_sampled_futures` | no | yes | yes | `(fig, ax)` |
| `plot_cost_field_exports` | no | no | yes | files in `save_dir`, `**plot_kwargs` |
| the two `plot_learning_curve`, the three `plot_gain_schedule` | no | yes | yes | the axes |
| `animate_search`, `animate_convergence` | no | yes | yes | the animation |
| `plot_region_of_attraction` | no | yes | yes | `PlotResult` |
| `plot_value_3d`, the three DP `animate*` | no | no | yes | `(fig, ax)`, the animation |
| `PlanningSolution.plot_cost_to_go`, `plot_control_law` and the planner mirrors | yes (`require_matplotlib`) | yes | yes | `PlotResult` |
| `Discretizer.plot_cost_to_go` | no | via `**kwargs` | via `**kwargs` | as the DP plot |
| `Comparison` plots | no | no | yes | `PlotResult` |
| `animate` on a system | yes | no | yes | the renderer's object, or `None` (§4.2) |
| `sys.render(x, u, t)` | yes | no | no | what the renderer's `present` returns |
| `plot_diagram` | no | no | yes | the Graphviz object, or `None` |

The show itself is 14 `plt.show` calls in 11 files, each with its own blocking rule; every one
is silent under the Agg backend, which the tests and the demo sweep rely on.

Two measurements that size the work: tier B's drawing is about 170 matplotlib calls over 10
files, of a narrow vocabulary (`plot`, `scatter`, `pcolormesh` / `imshow`, `contour(f)`,
`fill` / `add_patch`, `plot_surface`, `quiver` / `streamplot`, labels, limits, legend, colour
bar); the rest of its 3 000 lines is data preparation, which stays. The tests assert on
matplotlib artists (65 to 91 assertions in 12 files); they move to the description over time.

The callers the breaking changes reach (measured 2026-11-18 over `minilink/`, `examples/`,
`tests/`, `docs/`, `benchmarks/`; `experimental/` and `docs/make_assets.py` included in the
run's scope):

| Change | Hits |
| --- | --- |
| a plot's or an animation's return unpacked (`a, b = …`) | 25 |
| `.payload` | 1 |
| `html=` | 38 |
| `native=` | 34, and 4 in `experimental/` |
| `save=True/False` with `file_name=` | 12 |
| `show="pdf"` | 4, and 2 in `experimental/` |
| `animate_modal` | 20 (3 tests unpack `poles, modes`) |
| `sample_field_costs`, `FieldGrid` | 39 |
| `waypoint_color` | 5 |
| `prefers_inline_animation`, `is_inline_capable`, `"google.colab" in sys.modules` | 32 |
| `plot_diagram()` in `examples/demos` and `examples/projects` | about 36 |

## 3. The design

```
tool (arrays, core objects)  →  plot function  →  FigureSpec  →  draw(spec, renderer=)  →  PlotResult
                                 builds layers      plain data     matplotlib | plotly          figure, axes, spec
```

Each `plot_x(...)` is `draw(x_figure(...), renderer=, show=, ax=)`: an internal builder that
returns the description, and the one drawing call. `Comparison` and the user's grids assemble
panels from builders' descriptions.

### 3.1 One description: figure → panels → layers

`FigureSpec`, `Panel`, the layers and `draw`, in `minilink/graphical/figure.py`, public in
`minilink.graphical`, not in the top-level namespace. `ControlFigure` becomes `FigureSpec`.
The renderers live beside the animation renderers: `graphical/figure_renderers/matplotlib.py`
and `plotly.py`.

A `FigureSpec` is a title and a grid of panels (`n_cols`, shared axes, an optional shared colour
bar). A `Panel` holds the axes labels, limits, scales, `equal_aspect`, `legend`, `is_3d`, and an
ordered list of layers:

| Layer | Draws | Used by |
| --- | --- | --- |
| `Trace` | a polyline or a point set (`mode`), NaN breaks a line; dash, arrows; `fill=True` closes and fills it | time plots, Bode, poles and zeros, trajectories, learning curves, the search tree, obstacles, corridors |
| `RefLine` | a horizontal or vertical reference | margins, set-points, limits |
| `Note` | text anchored in the panel | margins, step figures |
| `Heatmap` | values on a regular 2-D mesh as colours | cost-to-go, control law, port map, cost field |
| `Contour` | level sets of values on a mesh, filled or not | region of attraction, clearance, the basin |
| `Surface` | values on a mesh as a 3-D surface | the cost-to-go in 3-D, the cost field in 3-D |
| `Arrows` | a vector field, quiver or stream | phase plane |

Seven names. A `Trace` with no colour takes the renderer's colour cycle, so two overlaid runs of
one plot are told apart. The description objects are frozen: composition builds a new
`FigureSpec`, it never mutates one.

**The format stays small by rule.** A field enters a layer only when a plot in the library
needs it and both renderers honour it (or ignore it without changing the figure's meaning).
No pass-through of renderer keywords: a renderer-specific finish is the caller's own call on
`result.axes` or `result.figure`. Per-plot style keywords nobody passes are removed (§7).

### 3.2 One drawing call

```python
draw(spec, *, renderer="matplotlib", show=None, ax=None) -> PlotResult
```

A renderer is one module with a draw function per layer kind, the panel and figure setup, and
the shared show step (§4.1). The renderers sit in a name-to-module table read lazily. Both
renderers draw every layer kind: `Heatmap`, `Contour`, `Surface` as `go.Heatmap`,
`go.Contour`, `go.Surface`; a filled `Trace` as `Scatter(fill="toself")`; `Arrows` as segments
with heads, and the stream style ported from the phase plane's Plotly code.

`renderer=None` draws nothing and returns the record with its `spec` (`figure` and `axes`
`None`): tests assert on it, and grids assemble from it.

### 3.3 The plot record

```python
@dataclass
class PlotResult:
    renderer: str | None
    figure: Any       # the drawn figure: matplotlib Figure, plotly Figure, graphviz Digraph
    axes: Any         # matplotlib: the Axes of a one-panel figure, a list in panel order
                      # for several; None on plotly and graphviz
    spec: FigureSpec  # what was drawn, overlays included; None on graphviz

    def show(self): ...              # where resolve_show puts it (§4.1)
    def save(self, file_name): ...   # the extension picks the format
```

- `payload` goes. `PlotResult` does not unpack as a pair: callers read `result.figure` and
  `result.axes`. A multi-slice call returns a list of records; a live plot its handle.
- `save`: matplotlib `savefig` (`.png`, `.pdf`, `.svg`, …); plotly `.html`, or an image
  through kaleido; graphviz the format of the extension (`.pdf`, `.svg`, `.png`). No
  `file_name=` keyword on any plot or animation: saving is one way, on the record.
- `show()` displays a record made with `show=False` (and adjusted), on any renderer.
- The record never displays itself as a cell's last expression (`_ipython_display_` shows
  nothing): the figure was shown when it was drawn.

### 3.4 Composition: `ax=` takes matplotlib `Axes` or a `PlotResult`

```python
result = sol.plot_cost_to_go(show=False, renderer="plotly")
sol.plot_trajectory(ax=result)                   # the run on the cost-to-go, on plotly

first = sys.plot_trajectory(traj_a, show=False)
sys.plot_trajectory(traj_b, ax=first)            # two runs, panel by panel

fig, (left, right) = plt.subplots(1, 2)          # the caller's own axes
scene.plot(ax=left)
planner.plot_tree(ax=right)
```

- **One rule for both hosts:** a plot adds its layers to the host's panels, and sets a panel's
  title, labels, limits and scales only where the host has none. A fresh `Axes` has none, so
  it draws in full; a `PlotResult` or a used `Axes` keeps what it has. The `title=""` idiom
  goes.
- **Panel by panel:** a host with the same panel count takes the layers panel by panel; `ax=`
  takes a list of `Axes` for a multi-panel plot. A count that differs is refused by name.
- **Refused, by name:** a 3-D layer into a 2-D panel; a second colour bar in a panel that has
  one; a Plotly figure the user built by hand (no description to append to); a `renderer=`
  that disagrees with the host.
- **Legends:** an overlay's entry that repeats a host label is not added twice.
- **`show=None`, the default:** shows unless `ax=` is given. A plot drawn into a host does not
  show the half-built figure; the caller shows the host (`result.show()`, or the next plot
  without `ax=`).
- The record's `spec` after an overlay is the host's with the overlay's layers: a new frozen
  `FigureSpec`, drawn in place on the host's figure.
- The host's overlay keywords (`trajectory=`, `trajectories=`) stay as shortcuts that add the
  same layers.

## 4. Showing, animations and diagrams

### 4.1 Where a figure is shown

One resolver in `graphical/common/environment.py`, `resolve_show(show, renderer)`, used by
every plot, animation and diagram. It replaces `prefers_inline_animation`, `is_inline_capable`
and the direct `"google.colab" in sys.modules` checks, and is tested as a pure table
(`show`, renderer, environment, backend, headless) → place, every cell enumerated.
`show=` takes `None`, `True`, `False`, `"inline"` or `"window"`:

| `show=True` | script / IPython | Jupyter | Colab |
| --- | --- | --- | --- |
| matplotlib | window (blocks in a script) | inline backend: inline; widget backend: its widget | inline |
| plotly | browser tab | inline | inline |
| meshcat | browser viewer | inline (self-contained page) | inline |
| pygame | window | window | refused by name |
| graphviz | `"window"`: the vector PDF in the system viewer | inline SVG | inline SVG |

- **Headless shows nothing.** `MINILINK_HEADLESS=1`, the Agg backend in a script or a kernel,
  or Linux with no `DISPLAY` / `WAYLAND_DISPLAY`: `show=True` builds the figure and shows
  nothing, on every renderer; an explicit `"inline"` or `"window"` raises by name. The tests
  and the demo sweep set `MINILINK_HEADLESS=1`.
- Only matplotlib reads matplotlib's backend; the other renderers decide on the environment.
  `detect_env` reads Spyder's shell as IPython.
- `"inline"` where nothing displays it, and `"window"` where there is no window, raise by name.
- **Shown at call time, in code order.** In a notebook with the inline backend, a figure is
  displayed at the line that draws it (`display`, then closed for matplotlib); the returned
  record does not display again. Widget backends keep their own display and are not closed.
  A change made after a shown call needs `show=False` first, as in a script.
- **Blocking:** in a script each matplotlib window blocks, as today; nothing blocks elsewhere.
  An interactive animation keeps a module-level reference until its window closes, so it does
  not stop when the call returns inside a function.

### 4.2 Animations

Every animation (`animate` on a system, a sampled loop and a planner; `animate_modal`; the DP
and search animations) takes `renderer=`, `show=` and returns an `AnimationResult`;
`animate_modal(mode="all")` returns a list of them, as a multi-slice plot does.

```python
@dataclass
class AnimationResult:
    renderer: str
    figure: Any       # matplotlib Figure | plotly Figure | meshcat Visualizer | None (pygame)
    animation: Any    # FuncAnimation | meshcat Animation | None (plotly: frames in figure)

    def show(self): ...
    def save(self, file_name): ...   # matplotlib .gif / .mp4; meshcat and plotly .html
```

- Shown at call time where `show` resolves (§4.1); the record never displays again; every
  path returns a record, `show=False` included. Meshcat starts its viewer server only when
  the record is shown or saved.
- `sys.animate(traj, renderer="meshcat", show=False).save("lap.html")` writes the
  self-contained page of the meshcat clip.
- `html=` goes for `show="inline"`; `save=` and `file_name=` go for `.save()`.
- `native=` goes: an animation always plays a clip (matplotlib `FuncAnimation`, Plotly frames,
  the meshcat keyframes); pygame, which has no clip, always plays live. The frame-by-frame
  loop stays inside the live paths (`game()`, `RealtimeSimulator`). Meshcat's clip draws
  changing geometry (arrows, a horizon, sampled trajectories, trails) as a flipbook of pages
  switched by a `visible` track, and keyframes the world slide of a following camera.
  Measured 2026-11-18 (this container, meshcat clip build and the per-frame draw cost):

  | Scene | Pages | Clip build, size | Frame by frame vs. its interval |
  | --- | --- | --- | --- |
  | formula car, contact force arrows | 2 676 | 4.7 s, 13.8 MB | 54 ms vs 30 ms |
  | MPC horizon (`mpc_racecar_kinematic.py`) | 468 | 2.4 s, 7.5 MB | 43 ms vs 30 ms |
  | MPPI samples (`mppi_racecar.py`) | 3 768 | 7.2 s, 19.5 MB | 61 ms vs 30 ms |

  The clips played correctly in headless Chromium (screenshots at two times); the per-frame
  loop runs slower than real time on all three.
- `sys.render(x, u, t)`, one frame, takes `renderer=` and `show=` and returns an
  `AnimationResult` of one frame (`animation` `None`).
- The live paths (`game`, the realtime simulator, `Evaluator.live_plot`, the live search and
  trajectory-optimization plots) are sessions, not results; they take `renderer=` and `ax=`
  where they draw into a figure. Gymnasium's `render()` follows the Gymnasium API and is out of
  scope.

### 4.3 Block diagrams

- `plot_diagram(sys, *, show=None, expand=True)` returns a
  `PlotResult(renderer="graphviz", figure=<Digraph>)`, shown at call time: inline SVG in a
  notebook (it was shown only as a cell's last expression), the vector PDF in the system
  viewer elsewhere. `show="pdf"` and the matplotlib PNG window go; `"window"` names the viewer.
- `.save("loop.svg")` writes exactly that file in the format of its extension (`.pdf`,
  `.svg`, `.png`). `file_name=` wrote the DOT source under the given name and the PDF beside
  it; it goes.
- The viewer's file is in a per-user temp folder, named by the diagram's name cleaned to a
  portable file name plus a short hash of its DOT source; no `.gv` beside it.
- Without the Graphviz binary: a warning and a record holding the `Digraph`.

### 4.4 The matplotlib-only modules

The DP and search animations, the search live plot, the trajectory-optimization live plot and
the MPC futures' colour map draw with matplotlib and are not descriptions (playing a list of
descriptions is not planned). They move under `graphical/` as named matplotlib-only modules,
fed by the planners' data, and are the import test's allow-list.

## 5. What this plan leaves out

- A theme or semantic-colour system: the colours stay in the plots.
- Playing a list of descriptions as an animation; an `AnimationResult.spec` until a caller
  needs it.
- Any change to what a plot shows: each figure keeps its data, labels and limits.

## 6. Housekeeping found by the audit

- S67 "One plot vocabulary" keeps its rows outside this contract (the output-port selection
  of `plot_trajectory`, the axis-label formatter and unit convention, the silent renderers,
  the phase-plane window warning, the region plot's `sys.x0`).
- QUESTIONS, "What the check of the core notebooks found in the API", item 15 (`Comparison`
  without `renderer`, `ax`, `title`) is answered by this contract.
- RULES 6.14 (one figure per notebook cell) gives "Jupyter displays one figure per cell" as its
  reason; with call-time display that reason no longer holds, and the rule's wording is
  revisited with the docs of the run (the rule itself is the maintainer's).
- Building the animation frames dominates an animation's cost (22 s on the Stanley lap, 40 s on
  the kinematic MPC, 120 s on MPPI, before either playback); a TODO row profiles it.

## 7. Rulings (2026-11-18)

**On the design (D1 to D7 of the proposal):**

- **R1. Scope:** the whole design in one run, behind the spike's gate (§8, step 1).
- **R2. Names and home:** `FigureSpec`, `Panel`, the layers and the drawing call, public in
  `minilink.graphical`, not in the top-level namespace.
- **R3. The record:** `figure` is the drawn figure, `spec` the description, `axes` the `Axes`
  for one panel and a list for several. Callers read fields; `PlotResult` does not unpack.
- **R4.** `payload` is removed.
- **R5. Composition:** `ax=` takes matplotlib `Axes` or a `PlotResult`.
- **R6. Style:** the per-plot colours stay; the style keywords with no caller go; `figsize`
  and `cmap` stay; `waypoint_color="none"` becomes `show_waypoints=False`.
- **R7.** `plot_cost_field` samples its costs; `FieldGrid` and `sample_field_costs` go.
- **R8. The merge:** the time signals onto `FigureSpec` in the run, behind the stop rule of §8,
  step 6; the phase plane moves with the families of step 5.
- **R9. Plotly:** both renderers draw every layer, the stream arrows included.

**On showing, animations and diagrams:**

- **R10.** Animations return an `AnimationResult`, `animate_modal` included.
- **R11.** `show="inline"` replaces `html=`; `native=` goes (meshcat clips draw changing
  geometry).
- **R12.** One resolver (§4.1), with a headless rule and `MINILINK_HEADLESS=1`.
- **R13.** Every figure shown at the line that draws it; matplotlib displayed and closed with
  the inline backend, widget backends left to themselves.
- **R14.** The diagram: the vector PDF in the system viewer outside notebooks, inline SVG
  inside; keyword-only; `show="pdf"` goes.

**After the two independent reviews (the maintainer accepted them):**

- **R15. One way to save:** `.save(file_name)` on both records; no `file_name=` or `save=`
  keyword on any plot, animation or diagram. `.show()` on both records.
- **R16.** `show=None` by default: show unless `ax=` is given.
- **R17.** `renderer=None` returns the description only.
- **R18. Names:** the drawing call is `draw` (`render` is a system's one-frame verb); the mesh
  layers are `Heatmap`, `Contour`, `Surface` (`Mesh` is the graphics word for a triangle
  mesh); `Fill` folds into `Trace(fill=True)`: seven layer names.
- **R19.** `axes` keeps R3's shape.
- **R20.** Composition rules of §3.4 (one rule for both hosts, panel by panel, a list of `Axes`
  for a multi-panel plot); colour cycle for traces with no colour.
- **R21.** The run's order of §8: the spike first, `renderer=` added to a family in the commit
  that makes Plotly draw it.
- **R22.** A tutorial notebook on plotting and animation closes the run (§8, step 8).
- **R23.** The run is on the branch `plotting-contract`, with a pull request into
  `dev-consolidation`.

## 8. The run

One branch, one commit per item, the gates at the end.

**Environment first.** The full environment is installed and checked before any code moves:
`pip install -e ".[full,dev]"` plus the Graphviz binary, `ipympl`, `kaleido` (Plotly image
export) and the headless SDL driver for pygame; `import` of jax, plotly, meshcat, pygame,
graphviz, gymnasium, sympy, nbclient checked. A skipped Plotly, meshcat or pygame test counts
as a failed gate, not a pass.

**Baseline.** Two records per figure, captured from the plotting tests and the affected demos
with the environment pinned (`override_env("script")`, `MINILINK_HEADLESS=1`), figures found
by `result.figure` or by diffing `plt.get_fignums()`, never only the current figure:

- the **data record** (each drawn array, labels, titles, limits, scales, colour limits, per
  panel): byte-identical after every step: this is the gate;
- the **artist record** (matplotlib artist kinds and counts): informational, since moving to
  the description legitimately changes artists (`imshow` and `pcolormesh`, patches and fills).

Run twice and compared byte for byte before any change.

1. **The spike and its gate.** `Heatmap` in `FigureSpec`; `plot_cost_to_go` drawn through it
   on both renderers; a trajectory overlaid with `ax=result` on both. The run continues only
   if: the data record is unchanged; building the spec plus `fig.canvas.draw()` is at most
   20 % slower than today's drawing (data preparation timed apart, median of 5); `Heatmap`
   needs 12 fields or fewer; each renderer gains 150 lines or fewer for it. Otherwise it stops
   with the spike's report.
2. **The record and the show.** `PlotResult` of §3.3 (`show`, `save`); `resolve_show`, the
   headless rule and one show step (§4.1); call-time display; `require_matplotlib` and
   `plot_result_of` into `graphical/common/`; the drawn-directly functions return a record,
   their callers in `examples/`, `experimental/`, `docs/` and `tests/` read it; the phase
   plane's renderers out of its description module; `MINILINK_HEADLESS=1` in the test and
   demo runners; one contract test over every public `plot_*` (keywords, record).
3. **Animations and diagrams.** `AnimationResult` (`show`, `save`); `html=`, `save=`,
   `file_name=`, `native=` removed and their callers changed; `animate_modal`, `sys.render`,
   the DP and search animations; `plot_diagram` as §4.3. A notebook test (nbclient, a kernel
   started without `MPLBACKEND=Agg`) checks call-time display: a diagram, an animation and a
   plot in one cell, in code order, each once.
4. **The draw call.** `FigureSpec`, `draw`, both renderers, `ax=` and the composition rules of
   §3.4, `renderer=None`, the colour cycle.
5. **The families onto `FigureSpec`**, one commit series each, each taking `renderer=` and its
   Plotly test in the same commit: the mesh family (DP, the planning solution, `Comparison`,
   `Discretizer`, the port map); the region of attraction; the phase plane; the learning curves
   and the gain schedule; the search tree; the MPC futures; the spatial plots (R6, R7,
   `plot_cost_field_exports`). The matplotlib-only modules move under `graphical/` (§4.4).
   After this step the import test passes.
6. **The merge**, last: the time signals onto `FigureSpec`; the live handle updates its `Trace`
   layers. Stops and reports, keeping steps 1 to 5, if the data record moves, a live-plot test
   fails, or a live update is more than 20 % slower (median of 5).
7. **Docs:** DESIGN (the plotting contract, the call chains), README where it shows a plot's
   return or a save, `examples/README.md`, the TRL ledger; RULES 6.14's reason flagged to the
   maintainer; the TODO rows leave.
8. **The tutorial:** `examples/tutorial/core/09_plots_and_animations.ipynb`, the one place to
   see and test the system: a plot and its record; saving; overlaying (`ax=`, panel by panel,
   on both renderers); `show=` and where a figure goes in a notebook, a console, a script and
   Colab, with the table of §4.1; headless runs; animations on matplotlib, plotly and meshcat,
   saved to `.gif` and `.html`; the block diagram; how the description and the renderers fit.
   It runs on Colab and in the notebook check.
9. **Review and gates:** a review pass; two independent reviewers (correctness,
   architecture); ruff; the full `pytest -n auto`; the demo-check bridge on its own; the
   affected demos; the notebook check on `examples/`; the docs build. Then the pull request.

**Stops and asks:** a data record that moves in a behaviour-preserving step; a gate of step 1
or 6; a public name not covered by §7; a speed loss against `dev` beyond noise that cannot be
fixed. Every other open choice takes the recommended option and is logged below.

**For the maintainer at the end:** the teaching notebooks in `alx87grd/teaching` that unpack a
plot's return, pass `html=`, `save=`, `native=`, `file_name=` or `show="pdf"`, or adjust a
figure after a shown call.

### Resuming the run

The run is expected to stop and restart (credits, a reclaimed container). Nothing lives only
in a session:

- **Small commits, pushed at once.** Every commit is one small item that passes ruff and its
  narrow tests, and is pushed to `origin/plotting-contract` right after. A half-done item is
  never left uncommitted at a break: it is finished, or reverted.
- **The progress log below** is updated in the same commit as the work it records: the step,
  the item, its commit, and the next action. A restart reads it first.
- **The baseline tooling and records live on the branch**, in `docs/plans/plotting-run/`
  (`baseline.py`, `baseline.json`), and leave before the pull request is merged.
- **A restart:** `git fetch && git checkout plotting-contract && git pull --ff-only`; install
  the environment (below); read the progress log; re-run `baseline.py --check`; continue at
  the next action.
- **The environment** (a fresh container):
  ```bash
  apt-get install -y graphviz coinor-libipopt-dev liblapack-dev libblas-dev libmumps-seq-dev gfortran pkg-config
  pip install -e ".[full,dev]" ipympl anywidget kaleido cyipopt
  ```
  pygame runs headless with `SDL_VIDEODRIVER=dummy`.

### Progress log

| Step | Item | Commit | State | Next action |
| --- | --- | --- | --- | --- |
| 0 | Plan and environment | a7e0c1f3 | done | baseline tooling |
| 0 | Baseline: 80 tests and demos, about 190 figures, two runs byte-identical (`--dist loadfile`, realtime tests left out), 3.5 min a check | e284a250 | done | step 1, the spike |
| 1 | Spike: `graphical/figure.py` (`FigureSpec`, `Panel`, `Trace`, `RefLine`, `Note`, `Heatmap`, `draw`), `figure_renderers/matplotlib.py` and `plotly.py`; the DP heatmaps and their trajectory overlay drawn through it. **Gate passed:** data record unchanged (80 tests); 158 vs 161 ms and 158 vs 159 ms (plot plus `canvas.draw`, median of 7, alternated, this container); `Heatmap` 10 fields; `draw_heatmap` 20 lines (matplotlib), 18 (plotly); overlay through `ax=result` on both renderers | 2a20a599 | done | step 2, the record and the show |
| 2a | `resolve_show` (the table of §4.1, headless, refusals by name) and `show_figure`, the one show step (inline displayed at the call and closed for matplotlib; a window blocks in a script); the 30 show sites go through it (live figures keep their open window); `MINILINK_HEADLESS=1` in the test conftest, the demo and notebook runners and the CI workflows; `tests/unittest/test_show.py`, the table test (57 cases). Baseline unchanged; 1 300 plotting tests pass | eabb011e | done | 2b: `PlotResult` with `show`/`save`, `payload` out |
| 2b | `PlotResult(renderer, figure, axes, spec)` with `show()` and `save(file_name)` (matplotlib any format, Plotly `.html` or an image through kaleido, Graphviz by extension); `payload` removed; `plot_result_of` and `require_matplotlib` in `graphical/common/plot_result.py`; the drawn-directly plots (DP, spatial, search tree, MPC futures, learning curves, gain schedule, `plot_value_3d`) return a record; their callers in `examples/` (scripts and two notebooks), `experimental/` and `tests/` read `.figure` / `.axes`. Baseline recaptured on the pre-run code (a7e0c1f3) with figures credited to the test that made them: 78 tests, 0 moved | this commit | done | 2c: the contract test over every public `plot_*` |

### Decisions taken during the run

- Colours in a description are CSS names or hex codes, which both renderers read; the
  matplotlib shorthand (`"k"`, `"g"`) is not used in builders.
- When `ax=` is a `PlotResult`, the renderer follows the host; a `renderer=` that names
  another library than the host's (other than the default) is refused.
- The spike's code stays: it is step 4's base (no scratch branch).
- `MINILINK_HEADLESS=1` silences every `show`, explicit `"inline"` and `"window"` included:
  it is the switch for automated runs. Without it, an explicit place that cannot be shown
  is refused by name.
- Pygame refuses `show=True` where it has no window (Colab, a server), rather than staying
  silent: it only draws in a window of its own; its off-screen driver
  (`SDL_VIDEODRIVER=dummy`) counts as a window.
- The `show=None` default ("show unless `ax=` is given") enters each function with its
  family's move onto the description (step 5).
- The live figures (`block=False` in the signal, port-map and phase-plane backends) keep
  their open, non-blocking window: they are updated after they are shown.
