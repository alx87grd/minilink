# Plotting, simplified: axes only in `ax=`, matplotlib the reference

**Status:** ruled and built (2026-11-18). Two independent reviewers read the plan against
the code (design; correctness and risk); both said to build it with changes, and the
maintainer ruled the four points they raised, one at a time (§2, rulings 4 to 7). It leaves
when the maintainer has looked at the result.
**Where it sits:** infrastructure, the plotting lane (agent-managed; the public keywords below
are ruled). It follows the plotting contract (DESIGN §7) and narrows it.
**Why:** the contract is uniform, and heavier than a student needs. Composition through a
plot's record carries a rulebook; two renderers are promised equal. The goal is the smallest
contract a student can hold in mind, with nothing that closes the door on a second renderer.

## 1. What a student writes

Unchanged:

```python
result = G.plot_bode()              # draws and shows; returns a PlotResult
result.figure, result.axes          # matplotlib's figure and axes
result.save("bode.png")
G.plot_bode(renderer="plotly")      # the same description, drawn by Plotly
```

Changed, composition:

```python
# Before: a plot's record as the host; the two descriptions are merged
drawn = scene.plot(show=False)
track.plot(ax=drawn)
drawn.show()

# After: matplotlib axes as the host; the plot draws on them
drawn = scene.plot(show=False)
track.plot(ax=drawn.axes)
drawn.show()
```

## 2. The rulings

1. **`show=False` is strict.** The figure is not shown, in a script, a console and a notebook
   alike; `result.show()` shows it. `show=` is a yes or a no, `True` by default: a plot
   shows the figure it makes, and on axes it is given it only draws. A figure a
   plot made is shown by its record, so the record is what to keep, not only its axes;
   `plt.show()` is for a figure the caller made.
2. **`ax=` takes matplotlib axes only:** one `Axes`, or a list of them for a plot of several
   panels. A `PlotResult` is refused by name, the message saying to pass `result.axes`.
   On the caller's axes a plot sets a title or a label only where the axes has none; its
   scale, ticks, aspect and grid are applied as on a figure of its own.
3. **Matplotlib is the reference renderer.** It draws every plot. Plotly draws a figure of
   its own as closely as it can, with no promise that every layer field looks the same, and
   no composition. `renderer="plotly"` stays on every plot; whether Plotly stays at all is
   decided in the maintainer's pass over the features. `renderer=None` keeps its meaning
   (the description alone); the environment never picks the renderer.
4. **Colours are a plot's own.** Two plots of one kind on shared axes share their colours
   (two trajectories, two Bode plots, two phase-plane runs). No recolouring and no `color=`
   keyword: no course overlays two plots of one kind (41 `plot_trajectory` calls in the
   teaching notebooks, none on shared axes). Answers are compared with `compare(...)`,
   which draws each in its own colour, or with the caller's own lines on `result.axes`. If
   a course overlays two plots of one kind, a `label=` keyword on the plots that fix a
   colour would be the remedy (it names the curve in the legend and frees its colour); no
   plot takes it today.
5. **A plot never shrinks the view of axes that already show something.** On empty axes it
   sets its limits, unless the caller set its own; on axes that show something, or that
   share an axis with axes that do, it widens the view to hold its limits too. The order of
   two plots that set limits crops neither; a plot without limits of its own leaves a fixed
   view as it is.
6. **A record matplotlib did not draw has no axes, and says so.** On a Plotly figure, a
   Graphviz diagram or a description alone, `result.axes` raises by name when it is read, and is refused by name as
   `ax=`: it no longer reads as "no axes given", which drew a separate figure in silence.
7. **Axes refuse what they cannot take, by name:** a 2-D plot on 3-D axes (as a 3-D plot on
   2-D axes already was) and a second colour bar on axes that show one.

## 3. What left

The merge of two descriptions and everything built for it, 216 lines and 122 lines of tests:

| Where | What |
| --- | --- |
| `graphical/figure.py` | `composed`, `both_limits`, `both_ticks`, `stacked_notes`, `clear_of`, `distinct_colors`, `is_recolored`, `colors_of`, `require_drawn_host`, `require_same_renderer`, `require_composable`; the `PlotResult` branch of `draw` |
| `graphical/figure_renderers/matplotlib.py` | `draw_into`, `draw_changes` |
| `graphical/figure_renderers/plotly.py` | `draw_into`, `draw_changes`, `legend_names` |

`overlay(host, layers_by_panel)` stays: the planning plots build one description from several
with it (`planning/plotting.py`), as plain data before any figure exists.

## 4. What it costs

- Two plots of one kind on shared axes come out in the same colours, with the same legend
  labels (ruling 4).
- Two notes at one anchor overlap: the margin boxes of two Bode plots on shared axes.
- A second plot's ticks replace the first's (the phase ticks of a second Bode plot).
- A record's `spec` describes what its own call drew: a figure drawn on by several plots has
  no single description, so `draw(result.spec, renderer=...)` redraws one plot, not the
  composition.
- No overlay on a Plotly figure.

## 5. What stays renderer-neutral

- Every plot builds a `FigureSpec`, which names no drawing library.
- `draw(spec, renderer=...)` picks a renderer module by name; a new renderer is one module
  with one function that draws a description.
- The record: `figure`, `spec`, `show()`, `save()`.
- Composition as data, on any renderer, exists: build the plots with `renderer=None`,
  `overlay` one's layers on the other's description, and draw once.

Matplotlib-specific: `ax=` and `result.axes`.

## 6. What was touched

1. **Library:** `draw` refuses a record in `ax=`; the functions of §3 left; the matplotlib
   renderer holds the view rule and the two refusals; a record matplotlib did not draw
   carries `NoAxes`.
2. **Callers:** `examples/projects/mpc/mpc_spatial.py` and `mpc_spatial_stack.ipynb` pass
   `result.axes`. Four notebook cells hid a figure with `show=False` and then called
   `plt.show()`, which shows nothing in a notebook: the showcase's planner comparison, two
   cells of `mpc_spatial_stack.ipynb`, one of `car_trajopt.ipynb`. Each now ends on its
   record's `.show()`.
3. **The tutorial** `examples/tutorial/core/09_plots_and_animations.ipynb`: its section on
   `ax=` draws a track on a scene, and its Plotly overlay is gone.
4. **Tests:** the overlay tests of `test_figure.py` on a record left; the axes rules, the
   refusals and `NoAxes` have theirs.
5. **Docs:** DESIGN §7 (the record, composition, the two renderers), README, ROADMAP §2 and
   the maturity ledger, the Plotly row of TODO §3.
6. **Teaching** (`alx87grd/teaching`, branch `plotting-contract`): the cells of
   `optimal_control/racecar_toward_mpc.ipynb` that passed a record.

## 7. The drawing verbs made uniform (ruled 2026-11-19)

The maintainer asked for an interface with no inconsistency left. Every drawing verb
(`plot*`, `animate*`, `render`) now:

- writes its keywords out (eight forwarders took `**kwargs`), its own first, then the
  standard ones it takes, keyword-only, in one order and with one default each:
  `renderer="matplotlib"`, `show=True`, `ax=None`;
- reads `show` as a yes or a no (`None`, "show unless `ax=` is given", is gone: a plot on
  given axes only draws);
- returns one record: a `plot_*` is annotated `-> PlotResult`, and a plot of several slices
  is one figure of panels side by side, where it returned a list;
- names one idea one way: `grid_shape=` on the scene and cost-field plots, `u_axis=` on the
  control-law animation, `title=` on every control plot and on `plot_trajectory`.

`tests/unittest/test_plot_contract.py` holds each of these on the source.

