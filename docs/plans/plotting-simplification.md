# Plotting, simplified: axes only in `ax=`, matplotlib the reference

**Status:** a plan the maintainer agreed to in outline (2026-11-18), waiting for two
independent reviews before it is built.
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
   alike; `result.show()` shows it. `show=None`, the default, shows unless `ax=` is given.
   Nothing changes here: this is the contract as it stands.
2. **`ax=` takes matplotlib axes only:** one `Axes`, or a list of them for a plot of several
   panels. A `PlotResult` is refused by name, the message saying to pass `result.axes`.
   On the caller's axes a plot sets a title, a label or a limit only where the axes has none,
   and leaves the showing to the caller unless asked.
3. **Matplotlib is the reference renderer.** It draws every plot. Plotly draws a single
   figure as well as it can, with no promise that every layer field looks the same, and no
   composition. `renderer="plotly"` stays on every plot; whether Plotly stays at all is
   decided in the maintainer's pass over the features.

## 3. What leaves

The merge of two descriptions and everything built for it, about 225 lines and their tests:

| Where | What |
| --- | --- |
| `graphical/figure.py` | `composed`, `both_limits`, `both_ticks`, `stacked_notes`, `clear_of`, `distinct_colors`, `is_recolored`, `colors_of`, `has_colorbar`, `require_drawn_host`, `require_same_renderer`, `require_composable`; the `PlotResult` branch of `draw` |
| `graphical/figure_renderers/matplotlib.py` | `draw_into`, `draw_changes` |
| `graphical/figure_renderers/plotly.py` | `draw_into`, `draw_changes`, `legend_names` |

`overlay(host, layers_by_panel)` stays if the planning plots still build one description from
several (`planning/plotting.py`); it works on plain data before any figure exists.

## 4. What it costs

- Two overlaid plots that fix their own colour (two Bode plots, two pole-zero maps) come out
  in one colour. Two trajectories still differ: their lines take matplotlib's colour cycle.
- An overlay does not widen the host's limits: a second system's pole outside the first
  plot's view stays out of view until the caller sets the limits.
- No overlay on a Plotly figure.

Open, the maintainer's: a `color=` keyword on the control plots (Bode, pole-zero, root locus,
Nyquist, step), so an overlay is told apart in one line. Added only if a course overlays them.

## 5. What stays renderer-neutral

- Every plot builds a `FigureSpec`, which names no drawing library.
- `draw(spec, renderer=...)` picks a renderer module by name; a new renderer is one module
  with one function that draws a description.
- The record: `figure`, `spec`, `show()`, `save()`. `draw(result.spec, renderer=...)` redraws
  a plot elsewhere.

Matplotlib-specific: `ax=` and `result.axes`. If composition is wanted on another renderer
later, two additive routes exist, neither changing what is written on matplotlib: `ax=` also
accepting that renderer's own target, or descriptions combined as data before one draw.

## 6. The steps

1. **Library:** `draw` refuses a `PlotResult` in `ax=`; the functions of §3 leave; the two
   renderers lose `draw_into`.
2. **Callers:** the calls in `examples/` and `experimental/` that pass a record pass
   `result.axes`.
3. **The tutorial** `examples/tutorial/core/09_plots_and_animations.ipynb`: its section "One
   plot into another" on axes.
4. **Tests:** the overlay tests on a record leave or are rewritten on axes; the contract
   tests (`test_plot_contract.py`) unchanged.
5. **Docs:** the composition paragraph of DESIGN §7 and its sentence on the two renderers;
   the breaking-change entry of ROADMAP §2; this plan deleted when it lands.
6. **Teaching** (`alx87grd/teaching`, branch `plotting-contract`): the cells of
   `optimal_control/racecar_toward_mpc.ipynb` that pass a record.
7. **Gates:** ruff; the full `pytest -n auto`; the demo-check bridge; the full demo sweep;
   the notebook check; the docs build; the teaching notebooks.

Done when a record passed to `ax=` is refused by name, every example and tutorial runs, and
DESIGN §7 states the contract of §2.
