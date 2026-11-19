"""The figure description and its two renderers: every layer, the record, and composition."""

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
import pytest  # noqa: E402

from minilink.graphical import (  # noqa: E402
    Arrows,
    Contour,
    FigureSpec,
    Heatmap,
    Note,
    Panel,
    PlotResult,
    RefLine,
    Surface,
    Trace,
    draw,
)
from minilink.graphical.figure import overlay, side_by_side  # noqa: E402

pytest.importorskip("plotly")
from plotly.colors import sample_colorscale  # noqa: E402

RENDERERS = ("matplotlib", "plotly")

X = np.linspace(-1.0, 1.0, 11)
Y = np.linspace(-2.0, 2.0, 21)
Z = np.add.outer(Y**2, X**2)  # z[j, i] at (x[i], y[j])
GX, GY = np.meshgrid(X, Y)


def one_panel(*layers, **panel):
    return FigureSpec(panels=(Panel(layers=layers, **panel),))


LAYERS = {
    "trace": one_panel(Trace(X, X**2, label="x²")),
    "markers": one_panel(Trace(X, X, mode="markers", marker="o-open")),
    "fill": one_panel(Trace([0, 1, 1, 0], [0, 0, 1, 1], fill=True, color="red")),
    "reference and note": one_panel(RefLine("y", 0.5), Note("hello")),
    "heatmap": one_panel(Heatmap(X, Y, Z, colorbar="z")),
    "contour lines": one_panel(Contour(X, Y, Z, levels=(1.0, 2.0), color="black")),
    "contour band": one_panel(
        Contour(X, Y, Z, levels=(0.5, 1.5), filled=True, color="#cfe3f7", label="in")
    ),
    "surface": one_panel(Surface(X, Y, Z), is_3d=True),
    "quiver": one_panel(Arrows(GX, GY, -GY, GX)),
    "stream": one_panel(Arrows(GX, GY, -GY, GX, style="stream")),
}


@pytest.mark.parametrize("renderer", RENDERERS)
@pytest.mark.parametrize("name", LAYERS)
def test_every_layer_draws_on_each_renderer(name, renderer):
    result = draw(LAYERS[name], renderer=renderer, show=False)
    assert isinstance(result, PlotResult)
    assert result.renderer == renderer
    assert result.spec is LAYERS[name]
    assert result.figure is not None
    if renderer == "plotly":
        assert len(result.figure.data) >= 1 or name == "reference and note"
    else:
        assert result.axes.figure is result.figure
    plt.close("all")


@pytest.mark.parametrize("renderer", RENDERERS)
def test_a_tick_offset_labels_the_colour_bar_by_the_shifted_values(renderer):
    spec = one_panel(Heatmap(X, Y, Z, vmin=0.0, vmax=4.0, tick_offset=10.0))
    result = draw(spec, renderer=renderer, show=False)
    if renderer == "plotly":
        labels = list(result.figure.data[0].colorbar.ticktext)
    else:
        labels = [t.get_text() for t in result.figure.axes[1].get_yticklabels()]
    assert labels[0] == "10.0"
    plt.close("all")


def test_renderer_none_keeps_the_description_alone():
    spec = LAYERS["heatmap"]
    result = draw(spec, renderer=None)
    assert result.renderer is None and result.figure is None and result.spec is spec
    with pytest.raises(ValueError, match="description only"):
        result.save("nothing.png")


def test_an_unknown_renderer_is_refused_by_name():
    with pytest.raises(ValueError, match="renderer must be one of"):
        draw(LAYERS["trace"], renderer="bokeh")


def test_fresh_axes_take_the_plots_title_and_labels():
    fig, ax = plt.subplots()
    draw(one_panel(Trace(X, X), title="run", x_label="x"), ax=ax, show=False)
    assert ax.get_title() == "run" and ax.get_xlabel() == "x"
    plt.close(fig)


def test_a_two_panel_plot_draws_into_a_list_of_axes():
    fig, axes = plt.subplots(1, 2)
    spec = FigureSpec(
        panels=(Panel(layers=(Trace(X, X),)), Panel(layers=(Trace(X, -X),)))
    )
    result = draw(spec, ax=list(axes), show=False)
    assert result.axes == list(axes)
    assert len(axes[0].lines) == 1 and len(axes[1].lines) == 1
    plt.close(fig)


def test_a_repeated_legend_label_shows_once():
    spec = one_panel(Trace(X, X, label="run"), Trace(X, -X, label="run"))
    result = draw(spec, show=False)
    assert [t.get_text() for t in result.axes.get_legend().get_texts()] == ["run"]
    plt.close("all")


@pytest.mark.parametrize("renderer", RENDERERS)
def test_a_panel_without_a_legend_draws_none(renderer):
    spec = one_panel(
        Trace(X, X, label="run"), Trace(X, -X, label="other"), legend=False
    )
    result = draw(spec, renderer=renderer, show=False)
    if renderer == "matplotlib":
        assert result.axes.get_legend() is None
    else:
        assert result.figure.layout.showlegend is False
    plt.close("all")


def test_traces_without_a_colour_take_the_colour_cycle():
    result = draw(one_panel(Trace(X, X), Trace(X, -X)), show=False)
    first, second = result.axes.lines
    assert first.get_color() != second.get_color()
    plt.close("all")


@pytest.mark.parametrize(
    "renderer, suffix", [("matplotlib", ".png"), ("plotly", ".html")]
)
def test_a_record_saves_in_the_format_of_its_extension(renderer, suffix, tmp_path):
    result = draw(LAYERS["heatmap"], renderer=renderer, show=False)
    written = result.save(tmp_path / f"figure{suffix}")
    assert written.exists() and written.stat().st_size > 0
    plt.close("all")


@pytest.mark.parametrize(
    "renderer, suffix", [("matplotlib", ".png"), ("plotly", ".html")]
)
def test_a_record_saved_without_an_extension_returns_the_file_written(
    renderer, suffix, tmp_path
):
    result = draw(LAYERS["trace"], renderer=renderer, show=False)
    written = result.save(tmp_path / "figure")
    assert written == tmp_path / f"figure{suffix}" and written.exists()
    plt.close("all")


def test_saving_leaves_the_global_font_settings_alone(tmp_path):
    draw(LAYERS["trace"], show=False).save(tmp_path / "figure.pdf")
    for key in ("pdf.fonttype", "ps.fonttype"):
        assert matplotlib.rcParams[key] == matplotlib.rcParamsDefault[key]
    plt.close("all")


def test_plotly_direction_arrows_sit_on_their_own_panel():
    spec = FigureSpec(
        panels=(
            Panel(layers=(Trace(X, X),)),
            Panel(layers=(Trace(X, -X, arrows=True),)),
        ),
        n_cols=2,
    )
    result = draw(spec, renderer="plotly", show=False)
    arrows = result.figure.layout.annotations
    assert arrows and all(
        (a.xref, a.yref, a.axref, a.ayref) == ("x2", "y2", "x2", "y2") for a in arrows
    )


def test_drawing_into_user_axes_keeps_the_users_figure_title():
    fig, ax = plt.subplots()
    fig.suptitle("mine")
    draw(FigureSpec(panels=(Panel(layers=(Trace(X, X),)),), title="plot"), ax=ax)
    assert fig.get_suptitle() == "mine"
    plt.close(fig)


def test_plotly_draws_a_reference_line_before_the_panels_first_trace():
    spec = one_panel(RefLine("y", 0.5), RefLine("x", 0.3), Trace(X, X))
    assert len(draw(spec, renderer="plotly", show=False).figure.layout.shapes) == 2


@pytest.mark.parametrize("renderer", RENDERERS)
def test_a_star_marker_draws_in_3d(renderer):
    star = Trace([0.0], [0.0], [0.0], mode="markers", marker="*")
    draw(FigureSpec(panels=(Panel(layers=(star,), is_3d=True),)), renderer=renderer)
    plt.close("all")


@pytest.mark.parametrize("renderer", RENDERERS)
def test_direction_arrows_skip_a_break_in_the_line(renderer):
    x = np.r_[np.linspace(0.0, 1.0, 30), np.nan, np.linspace(2.0, 3.0, 30)]
    draw(one_panel(Trace(x, x, arrows=True)), renderer=renderer, show=False)
    plt.close("all")


def test_plotly_keeps_an_equal_aspect_after_a_3d_panel():
    spec = FigureSpec(
        panels=(
            Panel(layers=(Surface(X, Y, Z),), is_3d=True),
            Panel(layers=(Trace(X, X),), equal_aspect=True),
        ),
        n_cols=2,
    )
    layout = draw(spec, renderer="plotly", show=False).figure.layout
    assert layout.yaxis.scaleanchor == "x"


def test_a_colour_bars_offset_ticks_stay_inside_its_range():
    spec = one_panel(Heatmap(X, Y, Z, colorbar="J", tick_offset=10.0))
    result = draw(spec, show=False)
    bar = result.figure.axes[-1]
    low, high = bar.get_ylim()
    assert low == pytest.approx(np.min(Z)) and high == pytest.approx(np.max(Z))
    plt.close("all")


def test_a_figure_without_a_panel_is_refused_by_name():
    with pytest.raises(ValueError, match="at least one panel"):
        draw(FigureSpec(panels=()), show=False)


def test_the_renderers_name_is_read_in_any_case():
    assert draw(LAYERS["trace"], renderer="Plotly", show=False).renderer == "plotly"


def test_plotly_draws_a_heatmaps_gamma_on_its_colour_scale():
    plain = draw(one_panel(Heatmap(X, Y, Z)), renderer="plotly", show=False)
    warped = draw(one_panel(Heatmap(X, Y, Z, gamma=0.5)), renderer="plotly", show=False)
    scale = warped.figure.data[0].colorscale

    # The colour at a quarter of the range is the map's at 0.25^0.5 = 0.5
    stops = [stop for stop, _ in scale]
    quarter = scale[stops.index(0.25)][1]
    assert quarter == sample_colorscale("viridis", [0.5])[0]
    assert scale != plain.figure.data[0].colorscale


def test_a_plot_in_a_grid_of_user_axes_leaves_the_figures_title_alone():
    fig, (left, right) = plt.subplots(1, 2)
    draw(FigureSpec(panels=(Panel(layers=(Trace(X, X),)),), title="plot"), ax=left)
    assert fig.get_suptitle() == ""
    plt.close(fig)


def test_a_plot_draws_on_a_drawn_plots_axes_and_keeps_its_title_and_labels():
    host = draw(one_panel(Heatmap(X, Y, Z), title="J", x_label="x"), show=False)
    run = one_panel(Trace(X, X, color="black"), title="run", x_label="other")
    both = draw(run, ax=host.axes)
    assert both.figure is host.figure and len(host.axes.lines) == 1
    assert host.axes.get_title() == "J" and host.axes.get_xlabel() == "x"
    plt.close("all")


def test_a_plot_widens_the_view_of_axes_that_show_something_and_never_shrinks_it():
    host = draw(
        one_panel(Trace(X, X), x_lim=(-1.0, 1.0), y_lim=(-1.0, 1.0)), show=False
    )
    draw(one_panel(Trace(5 * X, X), x_lim=(-5.0, 0.5)), ax=host.axes)
    assert host.axes.get_xlim() == (-5.0, 1.0) and host.axes.get_ylim() == (-1.0, 1.0)

    # The caller's own data stays in view
    fig, ax = plt.subplots()
    ax.plot([0.0, 100.0], [0.0, 1.0])
    draw(one_panel(Trace(X, X), x_lim=(-1.0, 0.5)), ax=ax)
    low, high = ax.get_xlim()
    assert low <= -1.0 and high >= 100.0
    plt.close("all")


def test_empty_axes_keep_the_limits_the_caller_set():
    fig, ax = plt.subplots()
    ax.set_xlim(-9.0, 9.0)
    draw(one_panel(Trace(X, X), x_lim=(-1.0, 1.0)), ax=ax)
    assert ax.get_xlim() == (-9.0, 9.0)
    plt.close(fig)


@pytest.mark.parametrize(
    "host, plot, message",
    [
        (LAYERS["trace"], FigureSpec(panels=(Panel(), Panel())), "panel"),
        (LAYERS["trace"], LAYERS["surface"], "3-D plot draws on 3-D axes"),
        (LAYERS["surface"], LAYERS["trace"], "2-D plot does not draw on 3-D axes"),
        (LAYERS["heatmap"], LAYERS["heatmap"], "colour bar"),
    ],
)
def test_axes_refuse_a_plot_they_cannot_take_by_name(host, plot, message):
    drawn = draw(host, show=False)
    with pytest.raises(ValueError, match=message):
        draw(plot, ax=drawn.axes)
    plt.close("all")


def test_a_record_is_refused_as_the_place_to_draw_by_name():
    host = draw(LAYERS["trace"], show=False)
    with pytest.raises(ValueError, match="pass ax=result.axes"):
        draw(LAYERS["trace"], ax=host)
    plt.close("all")


@pytest.mark.parametrize(
    "renderer, message", [("plotly", "plotly"), (None, "description")]
)
def test_a_record_matplotlib_did_not_draw_has_no_axes_and_says_so(renderer, message):
    """Its ``axes`` name the reason when read, and when passed as ``ax=``."""
    host = draw(LAYERS["trace"], renderer=renderer, show=False)
    with pytest.raises(AttributeError, match=message):
        host.axes.set_xlim(0.0, 1.0)
    with pytest.raises(ValueError, match=message):
        draw(LAYERS["trace"], ax=host.axes)
    plt.close("all")


def test_plotly_takes_no_axes():
    fig, ax = plt.subplots()
    with pytest.raises(ValueError, match="takes no ax="):
        draw(LAYERS["trace"], renderer="plotly", ax=ax, show=False)
    plt.close(fig)


def test_axes_that_share_an_axis_widen_together():
    """A plot's limits are not dropped on an empty panel whose shared axis already shows another plot."""
    fig, (left, right) = plt.subplots(1, 2, sharey=True)
    draw(one_panel(Trace(X, X), y_lim=(-1.0, 1.0)), ax=left)
    draw(one_panel(Trace(X, 5 * X), y_lim=(-5.0, 5.0)), ax=right)
    assert right.get_ylim() == (-5.0, 5.0) and left.get_ylim() == (-5.0, 5.0)
    plt.close(fig)


def test_axes_that_show_only_a_patch_keep_their_own_view():
    fig, ax = plt.subplots()
    ax.add_patch(plt.Rectangle((10.0, 10.0), 5.0, 5.0))
    draw(one_panel(Trace([10.0, 15.0], [10.0, 15.0]), x_lim=(10.0, 15.0)), ax=ax)
    assert ax.get_xlim()[0] > 5.0
    plt.close(fig)


def test_a_plot_of_several_panels_draws_nothing_when_one_axes_refuses_it():
    fig, (flat, deep) = plt.subplots(1, 2)
    spec = FigureSpec(
        panels=(
            Panel(layers=(Trace(X, X),)),
            Panel(layers=(Surface(X, Y, Z),), is_3d=True),
        )
    )
    with pytest.raises(ValueError, match="3-D plot draws on 3-D axes"):
        draw(spec, ax=[flat, deep])
    assert not flat.lines
    plt.close(fig)


def test_a_record_in_a_list_of_axes_is_refused_by_name():
    first = draw(LAYERS["trace"], show=False)
    with pytest.raises(ValueError, match="pass ax=result.axes"):
        draw(LAYERS["trace"], ax=[first])
    plt.close("all")


@pytest.mark.parametrize("renderer", ["plotly", None])
def test_a_record_without_axes_copies_and_names_itself_when_indexed(renderer):
    import copy

    result = draw(LAYERS["trace"], renderer=renderer, show=False)
    assert copy.deepcopy(result).renderer == renderer
    with pytest.raises(TypeError, match="axes"):
        result.axes[0]
    with pytest.raises(TypeError, match="axes"):
        list(result.axes)


def test_overlay_adds_layers_to_a_description_drawn_once_on_any_renderer():
    """Several plots as one description: built as data, drawn by one call."""
    under = draw(LAYERS["heatmap"], renderer=None).spec
    over = draw(one_panel(Trace(X, X, color="black")), renderer=None).spec
    both = overlay(under, [panel.layers for panel in over.panels])
    assert [type(layer).__name__ for layer in both.panels[0].layers] == [
        "Heatmap",
        "Trace",
    ]
    assert len(draw(both, show=False).axes.lines) == 1
    assert len(draw(both, renderer="plotly", show=False).figure.data) == 2
    plt.close("all")


def test_a_plot_on_given_axes_only_draws_whatever_show_says(monkeypatch):
    """The figure of given axes is its owner's to show: ``show=True`` shows nothing there."""
    import minilink.graphical.figure_renderers.matplotlib as renderer

    shown = []
    monkeypatch.setattr(renderer, "show_figure", lambda *args: shown.append(args))
    fig, ax = plt.subplots()
    draw(LAYERS["trace"], ax=ax, show=True)
    assert shown == []
    draw(LAYERS["trace"], show=True)
    assert len(shown) == 1
    plt.close("all")


def test_side_by_side_puts_panels_in_one_row():
    spec = side_by_side([LAYERS["trace"].panels[0], LAYERS["heatmap"].panels[0]], "two")
    result = draw(spec, show=False)
    assert spec.n_cols == 2 and spec.title == "two" and len(result.axes) == 2
    plt.close("all")
