# Control-analysis plots — Bode, pole-zero, root locus, Nyquist (draft, 2026-09-07)

Status: **draft v1 for the maintainer's rulings** (§7). Nothing implemented.
Lane: teaching surface (`analysis/frequency.py` + a small plotting layer in
`graphical/`). Closes the ROADMAP §6 "frequency analysis" question with the
NumPy-only route: minilink already carries `bode`, `pzmap` and
`transfer_function` on its own linearization, so the missing tools are two
algorithms (root locus, Nyquist) and one plotting layer that draws every
control plot the MATLAB way on **both** matplotlib and plotly.

## 1. What a student writes

```python
plant = Pendulum()
plant.plot_bode(x_bar)                          # Bode Diagram, margins annotated
plant.plot_pzmap(x_bar)                         # Pole-Zero Map
plant.plot_root_locus(x_bar)                    # Root Locus of the loop gain K
plant.plot_nyquist(x_bar)                       # Nyquist Diagram, -1 marked
plant.plot_step_response(x_bar)                 # Step Response of the channel

plant.plot_root_locus(x_bar, of=("y", 1), wrt="u", backend="plotly")  # Colab
gains, roots = plant.root_locus(x_bar)          # the data behind the plot
gm, pm, w_gc, w_pc = plant.margins(x_bar)       # gain and phase margins
```

Every tool keeps the family pattern
`tool(sys, x_bar, u_bar, t, params, *, of, wrt, method, eps, <options>, backend, show)`,
has a data function and a `plot_` function in `analysis/frequency.py`, and is
a method on `DynamicSystem`. `LTISystem` and `TransferFunction` go through
the same path (their linearization is themselves), so
`plant.transfer_function(x_bar).plot_root_locus()` reads naturally.

## 2. Which tools

| tool | today | plan |
| --- | --- | --- |
| `bode` / `plot_bode` | data + matplotlib plot, five-decade grid, channel in the y label | restyle (§3), margins annotation, MATLAB frequency range, plotly |
| `pzmap` / `plot_pzmap` | data + matplotlib plot, two colours, legend | restyle: one colour, `x` / `o`, equal aspect, optional damping grid, plotly |
| `root_locus` / `plot_root_locus` | missing | **new** — the one tool courses cannot do without |
| `nyquist` / `plot_nyquist` | missing | new — almost free once the complex response is shared |
| `margins` | missing | new — gain / phase margin with crossover frequencies, drawn on the Bode plot |
| `step_response` / `plot_step_response` | missing | new, small — the linearized channel through the existing simulator |
| Nichols chart | missing | out of scope (rare in the courses) |
| phase plane on plotly | `backend="plotly"` raises | done in the same pass: the rule is every graphical tool, both backends |

Recommended first pass: root locus, margins on Bode, Nyquist, the restyle of
Bode and pole-zero. Step response and the damping grid are a second, cheap
pass. So "just the root locus" is the minimum, but Nyquist and margins cost
about an hour each on the same code and belong to the same textbook chapter.

## 3. The MATLAB look, as rules

Applied by one style module so both backends agree:

- **Titles** are the MATLAB names: "Bode Diagram", "Pole-Zero Map", "Root
  Locus", "Nyquist Diagram", "Step Response"; the channel goes in a subtitle
  line "From: u[0]  To: y[1]", not in the y label (the two existing tests
  that look for the channel in a label move to the title).
- **Axis labels:** "Magnitude (dB)", "Phase (deg)", "Frequency (rad/s)";
  "Real Axis (seconds⁻¹)", "Imaginary Axis (seconds⁻¹)"; "Time (seconds)",
  "Amplitude".
- **Frequency grid:** log axis with major and minor grid lines; phase ticks
  at multiples of 45° or 90°; the automatic range runs one decade below the
  slowest pole or zero to one decade above the fastest (today it is fixed at
  two decades each side, so a pendulum spans 1e-2 to 1e3).
- **Colours:** MATLAB blue `#0072BD` for the system, orange `#D95319` for a
  second one; poles and zeros share the system colour, `x` for poles, hollow
  `o` for zeros, no legend for a single system.
- **Reference lines:** thin grey real and imaginary axes; margins as dashed
  vertical lines at the crossover frequencies with a text box
  "Gm = 12.3 dB (at 4.5 rad/s), Pm = 48° (at 1.2 rad/s)"; the critical point
  `-1` as a red `+` on the Nyquist plot; arrows on the Nyquist contour.
- **Shape:** equal aspect and 10 % padding for pole-zero, root locus and
  Nyquist; `FIGSIZE_BASE` and `FONT_SIZE` from the existing style modules.
- **Plotly extras** the notebook gains for free: hover text with frequency
  and gain on Bode, with gain `K`, damping ratio and natural frequency on the
  root locus, the `plotly_white` template and `PLOTLY_FIG_WIDTH` already used
  by the signal plots.

## 4. Design

**One figure spec, two renderers.** The signal plots already follow
"build a frozen spec, then render with the chosen backend"
(`graphical/signals/time_signals.py`). Control plots are all the same
material: one or two panels of line traces, markers, reference lines and text
annotations. So a small declarative spec covers all of them:

```python
@dataclass(frozen=True)
class Panel:                     # one axes
    traces: tuple[Trace, ...]    # x, y, style ("line" | "markers"), colour, hover text
    lines: tuple[RefLine, ...]   # vertical / horizontal dashed references
    notes: tuple[Note, ...]      # text boxes (margins, gain at cursor)
    x_label: str; y_label: str; x_log: bool = False; equal_aspect: bool = False

@dataclass(frozen=True)
class ControlFigure:
    title: str; subtitle: str; panels: tuple[Panel, ...]; share_x: bool = False
```

`graphical/control/` holds `figure_spec.py`, `matplotlib_backend.py`,
`plotly_backend.py` and `style.py` (the §3 rules as constants and two helper
functions). `analysis/frequency.py` builds the spec from the data functions
and calls `render_control_figure(spec, backend=..., show=...)`, which returns
the existing `PlotResult`. No per-plot renderer, so adding the Nichols chart
later would be a spec builder only. The phase plane reuses the plotly
renderer for its vector field (`plotly.figure_factory.create_quiver`) and
trajectory overlay.

**Data functions**, all NumPy on the SISO channel of the linearization:

- `frequency_response(sys, …, w=None, n=200) -> (w, G)` with `G` complex —
  the one evaluation behind `bode`, `nyquist` and `margins`; `bode` keeps
  returning `(w, magnitude_db, phase_deg)`.
- `margins(sys, …) -> Margins(gm_db, pm_deg, w_gc, w_pc)`: crossings of
  0 dB and −180° found by interpolation on the response; `inf` when a
  crossing does not exist, as MATLAB reports.
- `root_locus(sys, …, gains=None) -> (gains, roots)`: closed-loop poles of
  `1 + K·num(s)/den(s) = 0`, roots of `den + K·num` over a log-spaced gain
  grid (K from 0 to the value where the branches reach the asymptotes at
  ten times the largest pole radius), refined where a branch moves fast,
  columns matched between consecutive gains by nearest neighbour so each
  column is one continuous branch. Poles at `K = 0` and zeros at `K → ∞`
  are drawn as markers.
- `nyquist(sys, …, w=None) -> (w, G)`: positive frequencies from
  `frequency_response`; the plot mirrors the conjugate branch. Poles on the
  imaginary axis are detected and reported (the indented contour is a later
  refinement).
- `step_response(sys, …, tf=None) -> Trajectory`: the channel's
  `TransferFunction` under a unit step through `compute_forced`; `tf`
  defaults to five times the slowest time constant; `StepInfo` (rise time,
  settling time, overshoot, steady-state value) computed from the trajectory
  and annotated on the plot.

**Facades** on `DynamicSystemFacades`: `plot_root_locus`, `plot_nyquist`,
`plot_step_response`, `root_locus`, `nyquist`, `margins`, `step_response`,
two-line delegations like the existing ones. Root exports and the analysis
band gain the same names; the teaching-surface registry lists them.

## 5. Steps (about ten agent-hours)

1. **Figure spec and renderers** (2.5 h): `graphical/control/`, the style
   rules, both backends; `plot_bode` and `plot_pzmap` ported onto the spec
   with the §3 look (titles, labels, colours, equal aspect, frequency range).
   Tests: both backends return a `PlotResult` with a figure; the plotly figure
   has the expected trace count and axis types; labels and titles.
2. **Frequency response and margins** (1.5 h): `frequency_response`,
   `margins`, margin annotations on `plot_bode(margins=True)`, default on.
   Tests: margins of a known second-order loop, `inf` cases.
3. **Root locus** (2.5 h): data function, branch matching, `plot_root_locus`
   with hover text on plotly and an optional damping grid. Tests: double
   integrator with a lead compensator (breakaway point known), conjugate
   symmetry, branch count equals the pole count, ends at zeros or asymptotes.
4. **Nyquist** (1.5 h): data function, mirrored plot with arrows and the
   critical point; imaginary-axis poles reported. Tests: encirclement count
   on a stable and an unstable loop.
5. **Phase plane on plotly** (1 h): quiver plus trajectory overlay; the
   "not implemented" branch goes.
6. **Step response** (1 h, second pass): data function, `StepInfo`,
   plot with annotations.
7. **Student material and docs** (1.5 h, `[ask]`): `analysis_bode.py`
   becomes `analysis_frequency.py` with Bode, margins and Nyquist; new
   `analysis_root_locus.py`; `04_analysis.ipynb` shows one plot with
   `backend="plotly"`; README analysis line; DESIGN tools row; ROADMAP §6 and
   the TODO row closed with the NumPy-only decision; teaching-surface
   registry extended.

## 6. Out of scope

Nichols chart, multi-system overlays in one figure (the spec allows a second
trace colour, the API for it comes later), the indented Nyquist contour for
imaginary-axis poles, discrete-time versions of the plots (no discrete
`LTISystem` yet), `sgrid` on by default.

## 7. Rulings needed

1. **NumPy-only or python-control bridge** for the new tools. Recommended:
   NumPy-only, the ROADMAP's original intent; the bridge would add a second
   system model for two hundred lines of algorithms.
2. **First-pass scope.** Root locus + margins + Nyquist + the restyle
   (recommended), or root locus only.
3. **Channel label placement.** MATLAB "From / To" subtitle (recommended)
   or the current y-label form.
4. **Margins on Bode by default** (recommended: on, the plot reads like a
   textbook figure) or opt-in.
5. **Damping grid** (`sgrid`) as an option on pole-zero and root locus
   (recommended, off by default) or not at all.
6. **Backend default.** Keep `backend="matplotlib"` everywhere and let
   notebooks pass `backend="plotly"` (recommended), or pick plotly
   automatically inside Colab and Jupyter.
7. **Step response in the first pass** or the second.
