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

pytest.importorskip("plotly")

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
def test_every_layer_draws_on_both_renderers(name, renderer):
    result = draw(LAYERS[name], renderer=renderer, show=False)
    assert isinstance(result, PlotResult)
    assert result.renderer == renderer
    assert result.spec is LAYERS[name]
    assert result.figure is not None
    if renderer == "plotly":
        assert len(result.figure.data) >= 1 or name == "reference and note"
    else:
        assert result.axes is not None
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


@pytest.mark.parametrize("renderer", RENDERERS)
def test_an_overlay_adds_its_layers_panel_by_panel(renderer):
    host = draw(LAYERS["heatmap"], renderer=renderer, show=False)
    run = one_panel(Trace(X, X, color="black", label="run"))
    both = draw(run, ax=host, show=False)
    assert both.figure is host.figure
    assert [type(layer).__name__ for layer in both.spec.panels[0].layers] == [
        "Heatmap",
        "Trace",
    ]
    plt.close("all")


def test_an_overlay_keeps_the_hosts_title_and_labels():
    host = draw(one_panel(Heatmap(X, Y, Z), title="J", x_label="x"), show=False)
    draw(one_panel(Trace(X, X), title="run", x_label="other"), ax=host, show=False)
    assert host.axes.get_title() == "J"
    assert host.axes.get_xlabel() == "x"
    plt.close("all")


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


@pytest.mark.parametrize(
    "host, overlay, message",
    [
        (LAYERS["trace"], FigureSpec(panels=(Panel(), Panel())), "panel"),
        (LAYERS["trace"], LAYERS["surface"], "3-D"),
        (LAYERS["surface"], LAYERS["trace"], "2-D plot does not draw into a 3-D"),
        (LAYERS["heatmap"], LAYERS["heatmap"], "colour bar"),
    ],
)
def test_an_overlay_its_host_cannot_take_is_refused_by_name(host, overlay, message):
    drawn = draw(host, show=False)
    with pytest.raises(ValueError, match=message):
        draw(overlay, ax=drawn, show=False)
    plt.close("all")


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


@pytest.mark.parametrize("renderer", RENDERERS)
def test_an_overlay_line_in_the_hosts_colour_takes_a_free_one(renderer):
    host = draw(one_panel(Trace(X, X, color="#1f77b4")), renderer=renderer, show=False)
    both = draw(one_panel(Trace(X, -X, color="#1f77b4")), ax=host, show=False)
    first, second = both.spec.panels[0].layers
    assert first.color == "#1f77b4" and second.color not in ("#1f77b4", None)
    plt.close("all")


def test_each_clashing_colour_takes_its_own_free_colour():
    """Two host colours repeated by the overlay move to two distinct free colours; a colourless layer clashes with none."""
    host = draw(
        one_panel(
            Trace(X, X, color="#1f77b4"),
            Trace(X, 2 * X, color="#d62728"),
            Heatmap(X, Y, Z, colorbar=None),
            Trace(X, 3 * X),
        ),
        show=False,
    )
    both = draw(
        one_panel(
            Trace(X, -X, color="#1f77b4"),
            Trace(X, -2 * X, color="#d62728"),
            Trace(X, -3 * X, color="#1f77b4"),
            Trace(X, -4 * X),
        ),
        ax=host,
        show=False,
    )
    _, _, _, _, blue, red, blue_again, plain = both.spec.panels[0].layers
    used = {"#1f77b4", "#d62728"}
    assert blue.color not in used and red.color not in used
    assert blue.color != red.color and blue_again.color == blue.color
    assert plain.color is None
    plt.close("all")


def test_a_description_only_host_is_refused_by_name():
    host = draw(LAYERS["trace"], renderer=None)
    with pytest.raises(ValueError, match="draw the host first"):
        draw(LAYERS["trace"], ax=host, show=False)


def test_plotly_adds_no_legend_entry_the_host_shows():
    host = draw(one_panel(Trace(X, X, label="run")), renderer="plotly", show=False)
    both = draw(one_panel(Trace(X, -X, label="run")), ax=host, show=False)
    assert [trace.showlegend for trace in both.figure.data] == [True, False]


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
