# One plotting contract

**Status:** read-only audit and proposal (2026-11-15); nothing built. The maintainer decides the
open choices of §6, one at a time, before any code moves.
**Where it sits:** infrastructure, any release (CONSTITUTION §6, the infrastructure exemption);
it replaces the two plotting rows of [TODO.md](TODO.md) ("One plotting contract" in the
cross-release section and "One plot vocabulary", S67) once ruled.
**Lane:** agent-managed (the plotting interfaces), except the public names of §6, which are the
maintainer's.

---

## 1. The goal

1. **The library never depends on a renderer.** A tool computes arrays and core objects; a plot
   builds a description of the figure; a renderer draws the description. Matplotlib and Plotly
   are imported only inside their renderer.
2. **Matplotlib draws everything; Plotly draws what it is asked to.** A new figure kind is
   written once, as a description. Plotly support for it is a renderer-side addition, never a
   change to the tool or to the plot function.
3. **One user API.** Every `plot_*` takes the same three keywords (`renderer=`, `show=`, `ax=`)
   and returns the same record. A student who has drawn a Bode plot knows how to draw a
   cost-to-go.
4. **Plots compose.** A second plot can draw into the first one's panel (a trajectory on a
   cost-to-go, a region of attraction on a phase plane), on both renderers.

## 2. What exists (measured on `dev-consolidation`, 2026-11-15)

76 `plot_*` and `animate*` functions; about 12 000 lines across the drawing code. Three tiers:

| Tier | Plots | Description object | Renderers |
| --- | --- | --- | --- |
| **A. Described** | Bode, pole-zero, root locus, Nyquist, step response | `ControlFigure` (panels of traces, reference lines, notes) | matplotlib, plotly |
| | time signals (`plot_trajectory`, `plot_rollout`, `plot_cost`) | `SignalPlotSpec` | matplotlib, plotly |
| | phase plane | `PhasePlaneSpec` | matplotlib, plotly |
| | input-output map, control law of a block | `PortMapSpec` | matplotlib only |
| **B. Drawn directly** | DP cost-to-go and control law (heatmap, 3-D surface), the planning solution's plots, `Comparison`, region of attraction, spatial scene / track / cost field, search tree, MPC sampled futures, RL learning curves, gain schedule | none: the function calls matplotlib | matplotlib only |
| **C. Own pipelines** | animations (`animate`, `animate_modal`, DP and search animations) | scene primitives (`graphical/animation/primitives.py`) for the system animations; none for the DP and search animations | matplotlib, plotly, meshcat, pygame |
| | live plots (realtime simulator, trajectory optimization, tree search) | live handles fed by iteration records | matplotlib, plotly |
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
- **The keywords are mixed.** `renderer=` on 35 files; `ax=` on 17; tier A has no `ax=`, most of
  tier B has no `renderer=`. `show=` is everywhere but finished in 13 places, each with its own
  `plt.show(...)` call and blocking rule.
- **`PlotResult` carries a duplicate and misses the description.** `payload` repeats
  `(figure, axes)` on matplotlib and `figure` on plotly; `axes` is `None` on plotly; the
  description the figure was drawn from is dropped, so it cannot be re-rendered or extended.
- **Composition is ad hoc.** Overlays are keywords of their host (`trajectory=` on the
  cost-to-go, `trajectories=` and `basin=` on the region of attraction, `traj` on the phase
  plane). `ax=` works only on matplotlib and only on tier B.
- **Callers barely use the escape hatches.** Across the examples, tests and the teaching
  notebooks: 5 calls unpack `fig, ax = plot_…()` (4 of them the cost-to-go), 5 pass `ax=`
  (gain schedule, learning curve), about 120 read `result.figure` or `result.axes`, and Plotly is
  asked for on `plot_trajectory`, `animate` and the realtime games only. The migration cost is in
  the library, not in user code.

## 3. The proposal: one description, renderers behind it

```
tool (arrays, core objects)  →  plot function  →  Figure  →  render(figure, renderer=)  →  PlotResult
                                 builds layers      data      matplotlib | plotly            figure, axes, description
```

### 3.1 One description: `Figure` → `Panel` → layers

`ControlFigure` grows into the one schema (the name and module are decision D2). A figure is a
title and a grid of panels; a panel is axes labels, limits, scales, and an ordered list of
layers. The layers the 76 functions need, counted from tier A and B:

| Layer | Draws | Used by |
| --- | --- | --- |
| `Line` | a polyline, optional markers, dash, arrows | every time plot, Bode, Nyquist, trajectories, learning curves, search tree edges |
| `Markers` | a point set | poles and zeros, start and goal, samples |
| `RefLine` | a horizontal or vertical reference | margins, set-points, limits |
| `Note` | text anchored in the panel | margins, step figures |
| `Heatmap` | values on a regular 2-D mesh, a colour scale and bar | cost-to-go, control law, port map, cost field |
| `Contour` | level sets of a mesh | region of attraction, clearance, `V = c` |
| `Arrows` | a vector field (quiver or stream) | phase plane |
| `Fill` | a closed polygon or a band | obstacles, a corridor, a region, a basin |
| `Surface` | a 3-D surface over a mesh | `is_3d=True` cost-to-go and port map |

Nine layer kinds, each a frozen dataclass of arrays and a few style fields, in about 150 lines.
The spatial scene's shapes (circles, boxes) are `Fill` polygons sampled from the core geometry,
so the renderer never learns about obstacles.

### 3.2 One renderer call, one finishing step

```python
render(figure, *, renderer="matplotlib", show=True, ax=None) -> PlotResult
```

Each renderer is a dict from layer type to a small draw function (`draw_heatmap(ax, layer)`),
plus the panel and figure setup and one `finish(show)` that holds the blocking rule now copied
in 13 places. Adding Plotly for a new figure kind means adding the draw functions for the
layers it uses that Plotly lacks; a layer a renderer cannot draw raises
`ValueError("plotly does not draw Surface; use renderer='matplotlib'")` by name, so a
matplotlib-only figure is a fact of the renderer, not of the plot function.

### 3.3 One record

```python
@dataclass
class PlotResult:
    renderer: str
    figure: Any             # the renderer's figure (matplotlib Figure, plotly Figure), as today
    axes: Any               # the renderer's axes, one per panel (plotly: the panel's row and column)
    description: Figure     # what was drawn: re-render it, extend it, test it
```

`payload` goes. Tests assert on the description (`result.description.panels[0].layers`) instead
of on matplotlib artists, which makes them renderer-independent and faster. The field names are
decision D3.

### 3.4 Composition

```python
result = planner.plot_cost_to_go(show=False)
plant.plot_phase_plane(traj, ax=result)       # draws into the same panel, on either renderer
```

`ax=` accepts a matplotlib axes (as today) or a `PlotResult` (or one of its panels): the second
plot builds its own figure, its layers are appended to the target panel, and the target is
re-rendered. On matplotlib the drawing goes straight into the existing axes; on Plotly the
figure is redrawn from the merged description. The host's overlay keywords (`trajectory=`,
`trajectories=`) can stay as shortcuts that build the same layers.

### 3.5 What stays outside

- **System animations** keep their scene primitives: they are already renderer-independent and
  carry 3-D meshes, cameras and frame schedules that a 2-D figure description should not grow.
  Their contract is only aligned: `animate*(…, renderer=, show=, file_name=) -> PlotResult`,
  the animation object on `canvas`.
- **DP and search animations** become a `Figure` per frame plus a frame list, drawn by the
  matplotlib renderer's animation helper; Plotly frames come later if asked.
- **Live plots** keep their update handles (a live plot redraws in place many times a second;
  rebuilding a description per frame is the wrong trade). They are aligned on `renderer=` only.
- **Block diagrams** stay on graphviz; `plot_diagram` keeps its own return.

## 4. Options

| Option | What it is | Cost | What the goal gets |
| --- | --- | --- | --- |
| **A. Signatures only** (the current TODO row) | every function takes `renderer=`, `ax=`, `show=` and returns `PlotResult`; tier B stays matplotlib code and refuses `"plotly"` by name | small: about 40 functions, mostly signatures and returns | one API; no renderer independence for planning; composition through matplotlib `ax=` only |
| **B. One description for every static plot** (§3) | the four schemas merge into one; tier B is rewritten onto it; one renderer call; composition through the description | large: tier B is about 3 000 lines; the merge about 1 000; done family by family | everything in §1 |
| **C. B in stages, A first** | A's signatures and record land first (users see the final API at once); then the schemas merge; then tier B moves onto the description family by family, each step behaviour-preserving | B's cost, spread; each stage ships alone | A's API immediately, B's architecture at the end, no step that breaks a course notebook |

## 5. Recommendation

**Option C.** The user API (keywords and record) is the part students learn, so it lands first and
never moves again; the description behind it is the part that makes Plotly and future renderers
cheap, and it can migrate one family at a time without a flag day. Tier A proves the design
already works: the plan generalizes `ControlFigure` instead of inventing a new layer.

Stages, each one commit series with its own seeded baseline (the plot's description or its
matplotlib artists, compared before and after):

1. **The record and the keywords.** `PlotResult` as in §3.3 (with `payload` kept as a deprecated
   alias for one release, decision D4); every `plot_*` takes `renderer=`, `show=`, `ax=` and
   returns a `PlotResult`; one `finish(show)`; a matplotlib-only plot refuses `"plotly"` by
   name. Contract tests over every public `plot_*` (one parametrized test).
2. **One description.** `ControlFigure` becomes `Figure` with the layers of §3.1;
   `SignalPlotSpec`, `PhasePlaneSpec` and `PortMapSpec` become builders of a `Figure`; one
   matplotlib and one Plotly renderer replace the four pairs.
3. **Planning onto the description.** The cost-to-go and control-law heatmaps (DP, the planning
   solution, `Comparison`), then the region of attraction, then the spatial plots (`FieldGrid`
   goes, as the TODO row already says), the search tree, the MPC futures, the learning curves,
   the gain schedule.
4. **Composition.** `ax=` accepts a `PlotResult`; the overlay keywords become shortcuts over it.
5. **Plotly where it is cheap.** `Heatmap`, `Contour` and `Fill` on Plotly (each a few lines);
   `Surface` and the animations only when a course or a demo asks.

## 6. Decisions for the maintainer (to take one at a time)

- **D1. Scope.** Option C as recommended, or A alone (cheaper, planning stays matplotlib).
- **D2. Names and home.** `Figure`, `Panel` and the layer names; where they live
  (`minilink.graphical.figure`) and whether they are public, so a user can build a custom
  figure and render it with the same call. Recommendation: public, few names, in
  `minilink.graphical`.
- **D3. The record's fields.** Keep `figure` as the renderer's object and add `description`
  (§3.3), or make `figure` the description and name the renderer's object `canvas` (this
  changes what `result.figure` means in about 120 reads across examples, tests and the teaching
  notebooks). Recommendation: keep `figure`, add `description`: no reader breaks.
- **D4. `payload`.** Remove now, or keep as an alias for one release.
- **D5. Composition keyword.** `ax=` taking a `PlotResult` too (one keyword, already known from
  matplotlib), or a separate `on=`. Recommendation: `ax=`.
- **D6. Animations.** Aligned only (§3.5), or DP and search animations moved onto per-frame
  descriptions in stage 3.
- **D7. Style.** One colour and font theme per renderer, read from the description's semantic
  colours (`"state"`, `"input"`, `"reference"`) instead of hex strings in each plot, or keep the
  per-plot colours.

## 7. Housekeeping found by the audit

- TODO's "One plot vocabulary" row (S67) still lists `backend` among the plot keywords; RULES 2.7
  names it `renderer`. The row merges into this plan's stage 1 once ruled.
- `PlotResult._ipython_display_` already keeps a bare plot call in a notebook cell clean; the
  new record keeps it.
