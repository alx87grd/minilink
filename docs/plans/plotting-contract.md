# One plotting contract

**Status:** design proposal (2026-11-15), consolidated from an audit of the code and two
reviews of the design, one of them independent; nothing built. The maintainer decides the open
choices of §7, one at a time, before any code moves.
**Where it sits:** infrastructure, any release (CONSTITUTION §6, the infrastructure exemption);
it replaces the two plotting rows of [TODO.md](TODO.md) ("One plotting contract" in the
cross-release section and "One plot vocabulary", S67) once ruled.
**Lane:** agent-managed (the plotting interfaces), except the public names of §7, which are the
maintainer's.

---

## 1. The goal

1. **The library never depends on a renderer.** A tool computes arrays and core objects; a plot
   function builds a description of the figure; a renderer draws the description. Matplotlib
   and Plotly are imported only inside their renderer modules.
2. **Matplotlib draws everything; Plotly draws what it is asked to.** A new figure kind is
   written once, as a description. Plotly support for it is a renderer-side addition, never a
   change to the tool or to the plot function.
3. **One user API.** Every `plot_*` takes the same three keywords (`renderer=`, `show=`, `ax=`)
   and returns the same record. A student who has drawn a Bode plot knows how to draw a
   cost-to-go.
4. **Plots compose.** A second plot can draw into the first one's panel (a trajectory on a
   cost-to-go, a region of attraction on a phase plane), on both renderers.

Goal 1 has a test that can be written today and fails today: no `import matplotlib` or
`import plotly` outside the renderer modules. It fails in 12 library files (37 import sites),
all of them in the "drawn directly" tier below.

## 2. What exists (measured on `dev-consolidation`, 2026-11-15)

76 `plot_*` and `animate*` functions; about 12 000 lines across the drawing code. Three tiers:

| Tier | Plots | Description object | Renderers |
| --- | --- | --- | --- |
| **A. Described** | Bode, pole-zero, root locus, Nyquist, step response | `ControlFigure` (panels of traces, reference lines, notes) | matplotlib, plotly |
| | time signals (`plot_trajectory`, `plot_rollout`, `plot_cost`) and the live signal plots built on them | `SignalPlotSpec` | matplotlib, plotly |
| | phase plane | `PhasePlaneSpec` | matplotlib, plotly |
| | input-output map, control law of a block | `PortMapSpec` | matplotlib only |
| **B. Drawn directly** | DP cost-to-go and control law (heatmap, 3-D surface), the planning solution's plots, `Comparison`, region of attraction, spatial scene / track / cost field, search tree, MPC sampled futures, RL learning curves, gain schedule | none: the function calls matplotlib | matplotlib only |
| **C. Own pipelines** | system animations (`animate`, `animate_modal`) | scene primitives (`graphical/animation/primitives.py`) | matplotlib, plotly, meshcat, pygame |
| | DP and search animations, the search live plot | none | matplotlib only |
| | block diagrams | graphviz graph | graphviz |

What is already good: tier A is the design the goal asks for. `ControlFigure` (80 lines) carries
five figure kinds through two renderers of about 200 lines each; the system animations do the
same with scene primitives on four renderers.

What is not:

- **Four description schemas for one idea.** `ControlFigure`, `SignalPlotSpec`, `PhasePlaneSpec`
  and `PortMapSpec` each describe "axes with things drawn on them", each with its own renderer
  code. A fifth figure kind today means a fifth schema or a tier B function.
- **Tier B is a third of the plots and all of planning.** They return `(fig, ax)`, a `PlotResult`
  wrapping `(fig, ax)`, or a list of either. Plotly cannot draw any of them without rewriting the
  function.
- **The keywords are mixed.** `renderer=` in about 35 files; `ax=` in 17; the control plots and
  the time signals have no `ax=`, most of tier B has no `renderer=`. `show=` is everywhere but finished in 13 places, each with its own
  `plt.show(...)` call and blocking rule.
- **`PlotResult` carries a duplicate and misses the description.** `payload` repeats
  `(figure, axes)` on matplotlib and `figure` on plotly; `axes` is `None` on plotly; the
  description the figure was drawn from is dropped.
- **Composition is ad hoc.** Overlays are keywords of their host (`trajectory=` on the
  cost-to-go, `trajectories=` and `basin=` on the region of attraction, `traj` on the phase
  plane). `ax=` works on matplotlib only: on tier B, the port map and the phase plane's
  matplotlib renderer. The library passes it 22 times internally (the `Comparison` grids, the
  overlays). `is_3d=True` does not use it: the surface opens a second figure, and the record
  then holds the heatmap's axes from one function and the surface's from another.
- **Two returns fall outside the record.** A multi-slice call returns a list of results, and a
  live plot returns its update handle.

Two measurements that size the work:

- **Tier B's drawing vocabulary is narrow.** Its about 3 000 lines are mostly data preparation
  (sampling a field on a mesh, projecting a tree). The matplotlib drawing itself is about 170
  calls over 10 files, and only these kinds: `plot`, `scatter`, `pcolormesh` / `imshow`,
  `contour` / `contourf`, `fill` / `add_patch`, `plot_surface`, `quiver` / `streamplot`, plus
  labels, limits, legend, colour bar, grid and aspect. Moving a plot onto a description
  replaces those calls and keeps the data preparation.
- **Users own their matplotlib axes.** Across the examples and the teaching notebooks, about
  29 calls in 13 files pass `ax=`, almost all `scene.plot(ax=ax)` and `track.plot(ax=ax)` onto
  axes the script made with `plt.subplots` and then draws on with raw matplotlib; about 16 to
  34 calls (two counts, two patterns) unpack `fig, ax = …`. Plotly is asked for on
  `plot_trajectory`, `animate` and the realtime games only. So drawing into a caller's
  matplotlib `Axes` is the composition path in use, and it stays first class.
- **The tests assert on matplotlib artists**: 65 to 91 assertions (two counts) in 12 files
  (`ax.lines`, `ax.collections`, titles, legends). Each family's migration re-runs them; they
  move to assertions on the description over time.

## 3. The proposal: one description, renderers behind it

```
tool (arrays, core objects)  →  plot function  →  description  →  render(spec, renderer=)  →  PlotResult
                                 builds layers      plain data      matplotlib | plotly          figure, axes, spec
```

### 3.1 One description: figure → panels → layers

`ControlFigure` grows into the one schema. Its name is decision D2: every renderer in the code
already calls its argument `spec`, and `Figure` is also the name of matplotlib's and Plotly's
own figure class, so this document writes `FigureSpec` and `result.spec`.

A `FigureSpec` is a title and a grid of panels (`n_cols`, shared axes, an optional shared colour
bar). A `Panel` holds the axes labels, limits, scales, `equal_aspect`, `legend`, `is_3d`, and an
ordered list of layers:

| Layer | Draws | Used by |
| --- | --- | --- |
| `Trace` (as today) | a polyline or a point set (`mode`), NaN breaks a line; dash, arrows | every time plot, Bode, poles and zeros, trajectories, learning curves, the search tree |
| `RefLine` (as today) | a horizontal or vertical reference | margins, set-points, limits |
| `Note` (as today) | text anchored in the panel | margins, step figures |
| `Mesh` | values on a regular 2-D mesh, drawn as a heatmap, as level sets (filled or not) or as a 3-D surface (`draw=`) | cost-to-go, control law, port map, cost field, region of attraction, clearance, the basin |
| `Fill` | a closed polygon | obstacles, a corridor, a region |
| `Arrows` | a vector field (quiver or stream) | phase plane |

Six layer kinds, three of them `ControlFigure`'s own, unchanged: the schema grows by three
names. Whether `Mesh` is one layer with a `draw=` field or three (`Heatmap`, `Contour`,
`Surface`) is settled by the spike: one name if their fields mostly coincide, three if `draw=`
would switch half of them off. The spatial scene's shapes are `Fill` polygons sampled from the
core geometry, so a renderer never learns what an obstacle is.

**Fields the direct plots use today that the schema must take or drop**, family by family
(read in the code):

- *Mesh as heatmap:* smooth shading on a grid's own nodes against cell images with an extent;
  transparency and drawing order (the scene's density under its obstacles); a power or log
  colour scale with relabelled ticks (the cost field); colour limits; the colour bar on or
  off, its label; per panel or shared.
- *Mesh as level sets:* explicit levels, colour, dash, a legend label.
- *Mesh as surface:* the camera, the z label and limits.
- *Trace:* transparency, drawing order, a colour taken from a colour map by index (the MPC
  futures), a `z` array in a 3-D panel.
- *Panel and figure:* a panel title beside the figure title (`Comparison`), the figure size.
- *Live:* the signal handle updates a fixed set of traces; the search live plot removes and
  redraws a varying number, so it needs its own update or stays as it is.

Each is kept only if a caller uses it (the rule below); a matplotlib-only finish such as a
wireframe stays the caller's own call on `result.axes`.

**The schema stays small by rule.** It is a closed vocabulary, not a wrapper of matplotlib:

- a field enters a layer only when a plot in the library needs it and both renderers can honour
  it (or ignore it without changing the figure's meaning, as `hover` is on matplotlib);
- there is no pass-through of renderer keywords. A user who wants a matplotlib-only finish
  takes `result.axes` and calls matplotlib on it;
- the per-plot style options go at migration time when nobody passes them. Of the fifteen
  checked on the spatial and search plots, eleven (`obstacle_color`, `tree_color`,
  `vmax_percentile`, …) have no caller in the examples, tests, benchmarks or teaching
  notebooks.

### 3.2 One renderer call, one finishing step

```python
render(spec, *, renderer="matplotlib", show=True, ax=None) -> PlotResult
```

A renderer is one module with a draw function per layer kind (`draw_mesh(ax, layer)`), the
panel and figure setup, and one `finish(show)` holding the blocking rule now copied in 13
places. The renderers sit in a name-to-module table read lazily, so a third renderer is one
module and one table entry. A layer a renderer cannot draw raises by name
(`"plotly does not draw Arrows(style='stream'); use renderer='matplotlib'"`): being
matplotlib-only is a fact of the renderer, not of the plot function.

### 3.3 One record

```python
@dataclass
class PlotResult:
    renderer: str
    figure: Any       # the renderer's figure (matplotlib Figure, plotly Figure), as today
    axes: Any         # matplotlib's Axes, one per panel; None on plotly, as today
    spec: FigureSpec  # what was drawn, overlays included: re-render it, test it
```

`payload` goes. No new handle type: on matplotlib `axes` is what a caller already knows; on
Plotly the record itself is what the next plot draws into (§3.4). A multi-slice call keeps
returning a list of records, and a live plot its handle.

### 3.4 Composition: `ax=` takes a matplotlib `Axes` or a `PlotResult`

```python
fig, ax = plt.subplots()                         # the caller's own axes, as scripts do today
scene.plot(ax=ax)
ax.plot(x, y)                                    # raw matplotlib beside it

result = planner.plot_cost_to_go(show=False)     # or: one plot into another, either renderer
plant.plot_phase_plane(traj, ax=result)
```

- **Into a matplotlib `Axes`:** the plot draws as it does today, labels, limits and title
  included (a caller passes `title=""` to keep its own). This is the path the scripts use and
  it does not change.
- **Into a `PlotResult`:** the plot adds its layers only; the host keeps its title, labels,
  limits and scales. The layers are appended to the host's `spec` and drawn in place on its
  figure, so `result.spec` always says what the figure shows, and the renderer follows the
  host (a `renderer=` that disagrees is refused).
- **Refused, by name:** a multi-panel plot into a one-panel host and the reverse (two runs on
  one `plot_trajectory` figure match and are allowed); a 3-D layer into a 2-D panel; a second
  colour-scaled `Mesh` in a panel that already has a colour bar, unless the overlay asks for
  none.
- **Legends:** an overlay's entry that repeats a host label is not added twice.
- **`show`:** every call but the last passes `show=False`; a figure already displayed in a
  notebook does not receive later layers. The docstring of `ax=` says so.
- Inside the library, the internal `ax=` calls become panel assembly on the description
  (`Comparison` builds one `FigureSpec` of `n × m` panels), with no renderer object passed
  around.

The host's overlay keywords (`trajectory=`, `trajectories=`) stay as shortcuts that add the
same layers. On Plotly, a figure built by a helper that owns the whole figure (the phase
plane's stream arrows) cannot host an overlay; the renderer refuses it by name.

### 3.5 Live plots and animations

- **Live signal plots** already work on their description: a handle is rendered once and then
  `handle.update(spec)` redraws the lines in place. They stay on `SignalPlotSpec` and move only
  if the optional merge of step 4 is taken; the search live plot stays as it is.
- **System animations** keep their scene primitives: they are already renderer-independent and
  carry 3-D meshes, cameras and frame schedules a 2-D description should not grow. Only their
  signature is aligned: `animate*(…, renderer=, show=, file_name=) -> PlotResult`.
- **DP and search animations** stay matplotlib functions. Playing a list of descriptions is
  not planned: no use asks for it.
- **Block diagrams** stay on graphviz; `plot_diagram` keeps its own return.

## 4. Options

| Option | What it is | Cost | What the goal gets |
| --- | --- | --- | --- |
| **A. Signatures only** (the current TODO row) | every function takes `renderer=`, `ax=`, `show=` and returns `PlotResult`; tier B stays matplotlib code and refuses `"plotly"` by name | small: about 40 functions, mostly signatures and returns | one API; goal 1 not met (the 37 import sites stay); composition on matplotlib only |
| **B. One description, in one pass** (§3) | the four schemas merge; tier B moves onto it; one renderer call | the merge (about 1 000 lines touched) plus about 170 drawing calls replaced over 10 files | everything in §1, after one long branch |
| **C. One description, family by family** | a spike first (§5); then each tier B family moves onto the description and takes the contract in the same commit; the merge of the described plots last and optional | B's cost, spread; every step ships alone and can stop | everything in §1 at the end; each family complete when it lands |

## 5. Recommendation

**Option C, behind a spike.** The design is right for the goal and tier A proves it works at
small scale. What is not yet measured is whether one schema stays small once heatmaps,
contours and 3-D surfaces enter it. So measure before committing:

0. **Spike (not merged).** On a scratch branch: add `Mesh` (heatmap and surface) to
   `ControlFigure`, draw `plot_cost_to_go` through it on matplotlib and Plotly, and overlay a
   trajectory through `ax=result` on both. Report the lines added per renderer, whether the
   matplotlib figure draws the same data as today's, the render time against today's, and
   whether `Mesh` wants one name or three. The maintainer rules D1 on those numbers.
1. **The record and the finish.** `PlotResult` as in §3.3; every `plot_*` returns one (the
   scripts that unpack `fig, ax = …` change, about 16 to 34 call sites); one `finish(show)`.
   Small, and it does not wait for the rest.
2. **The mesh family.** `ControlFigure` becomes `FigureSpec` with `Mesh`; one `render` call
   with `ax=` as in §3.4. Onto it: the cost-to-go and control-law plots (DP, the planning
   solution, `Comparison`) and the port map, which is the same figure.
3. **The other direct plots, one family per commit series, in order of teaching use:** the
   region of attraction; the learning curves and the gain schedule; the search tree; the MPC
   futures; the spatial plots (`FieldGrid` goes, as the TODO row already says). Each family
   takes `renderer=`, `show=`, `ax=`, loses its matplotlib import, and trims the style options
   nobody passes. The work can stop after any family; what has moved is complete. After this
   step the import test of §1 passes.
4. **Optional, last: the time signals and the phase plane onto `FigureSpec`.** They already
   meet goal 1 on their own descriptions, so merging them buys one schema instead of three and
   costs the riskiest change (the live handles, the stream arrows). Until then they take
   `ax=` through their own renderers. Decided after step 3, on what the schema looks like.
5. **Plotly where it is cheap.** A heatmap, level sets, a polygon and a surface are one Plotly
   trace each; stream arrows wait for a real use.

Each step is behaviour-preserving on matplotlib: its baseline is the figure's artists (kind,
count, data arrays, labels) captured before and compared after. Where the drawing is
deliberately regrouped (the search tree's edges as one broken line: 0.44 s against 0.70 s for
5 000 edges, median of 3 on the Agg canvas), the baseline compares the data drawn, not the
artist count.

**Done when:** the import test of §1 passes; one parametrized contract test calls every public
`plot_*` with `show=False` and checks the keywords, the `PlotResult` and a non-empty `spec`;
the two TODO rows are gone.

## 6. What this plan deliberately leaves out

- A theme or semantic-colour system (decision D7): the colours stay where they are.
- Plotly for every figure: only the layers a demo or a course asks for.
- A public "build your own figure" tutorial: the schema can be public (D2) without being taught.
- Any change to what a plot shows. Migration keeps each figure as it is drawn today.

## 7. Decisions for the maintainer (to take one at a time)

- **D1. Scope.** Option C behind the spike as recommended; B in one pass; or A alone (cheaper,
  planning stays matplotlib, goal 1 not met).
- **D2. Names and home.** The description's name (`FigureSpec` with `result.spec`, or `Figure`
  with `result.description`), `Panel` and the layer names (six, or eight if the spike splits
  `Mesh`); the module (`minilink.graphical.figure`); whether they are public. Recommendation: `FigureSpec`, public
  in `minilink.graphical`, not in the top-level namespace.
- **D3. The record's fields.** As §3.3 (`figure` stays the renderer's object, about 120 reads
  in examples, tests and the teaching notebooks keep working), or `figure` becomes the
  description and the renderer's object is renamed. Recommendation: §3.3.
- **D4. `payload`.** Remove with step 1, or keep as an alias for one release. Recommendation:
  remove; one test reads it and nothing else does
  ([internals-cleanup.md](internals-cleanup.md) already lists it as written and never read).
- **D5. Composition.** `ax=` taking a matplotlib `Axes` or a `PlotResult` (§3.4), or a separate
  keyword for the second. Recommendation: `ax=` alone.
- **D6. The merge of step 4.** Decide after step 3 (recommended), commit to it now, or drop it.
  The DP and search animations stay as they are either way.
- **D7. Style.** Keep the per-plot colours (recommended), or one theme read from semantic
  colour names.

## 8. Housekeeping found by the audit

- TODO's "One plot vocabulary" row (S67) still lists `backend` among the plot keywords; RULES 2.7
  names it `renderer`. The row merges into this plan once ruled.
- `PlotResult._ipython_display_` already keeps a bare plot call in a notebook cell clean; the
  new record keeps it.
