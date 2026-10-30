"""Time-signal plots in Plotly: one subplot row per trace, drawn once or kept live."""

from __future__ import annotations

import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer

from minilink.graphical.common.environment import detect_env
from minilink.graphical.common.plotly_style import (
    PLOTLY_FIG_WIDTH,
    PLOTLY_SIGNAL_MARGIN,
    PLOTLY_SIGNAL_MIN_HEIGHT,
    PLOTLY_SIGNAL_ROW_HEIGHT,
    PLOTLY_TEMPLATE,
)
from minilink.graphical.signals.signal_colors import plotly_color
from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    PlotResult,
    SignalPlotSpec,
    coerce_signal_plot_spec,
    require_same_trace_count,
)

###############################################################################
# Public API
###############################################################################


def render_plotly_signal_plot(
    spec: SignalPlotSpec,
    *,
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Render a one-shot time-signal plot with Plotly."""
    fig = create_figure(spec, **kwargs)
    if show:
        fig.show()

    return PlotResult(backend="plotly", payload=fig, figure=fig)


def open_plotly_signal_plot(
    spec: SignalPlotSpec,
    *,
    spec_builder=None,
    show: bool = True,
    **kwargs,
) -> LivePlotHandle:
    """Open a live Plotly time-signal plot."""
    fig = create_figure(spec, **kwargs)

    # Not shown: the figure alone, updated in place
    if not show:
        return PlotlyLivePlotHandle(fig, spec_builder=spec_builder)

    # In a notebook: a FigureWidget under the cell, updated in place
    env = detect_env()
    if env in ("jupyter", "colab"):
        go, _ = require_plotly()
        try:
            fw = go.FigureWidget(fig)
        except (TypeError, ValueError, ImportError):
            # Plotly 6+ may require ``anywidget``; fall back to localhost polling.
            polling = PlotlyPollingServer()
            polling.start(fig)
            return PlotlyLivePlotHandle(
                fig,
                spec_builder=spec_builder,
                polling_server=polling,
            )

        try:
            from IPython.display import display

            display(fw)
        except ImportError:
            fw.show()

        return PlotlyLivePlotHandle(fw, spec_builder=spec_builder)

    # Elsewhere: a browser page on localhost polls the latest figure
    polling = PlotlyPollingServer()
    polling.start(fig)

    return PlotlyLivePlotHandle(
        fig,
        spec_builder=spec_builder,
        polling_server=polling,
    )


class PlotlyLivePlotHandle(LivePlotHandle):
    """Live Plotly trace updater (Figure, FigureWidget, or polling server)."""

    def __init__(
        self,
        fig,
        *,
        spec_builder=None,
        polling_server: PlotlyPollingServer | None = None,
    ):
        self.fig = fig
        self.spec_builder = spec_builder
        self._polling_server = polling_server

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        """Redraw each trace from a new trajectory or plot spec, with the same traces."""
        spec = coerce_signal_plot_spec(
            traj_or_spec,
            self.spec_builder,
            title=title,
        )

        # In place: the same traces take the new samples, in one batched redraw
        # when the figure has one (a FigureWidget)
        require_same_trace_count(spec, self.fig.data)
        batch = getattr(self.fig, "batch_update", None)
        if callable(batch):
            with self.fig.batch_update():
                redraw_traces(self.fig, spec, title)
        else:
            redraw_traces(self.fig, spec, title)

        # Through the polling page: push the updated figure
        if self._polling_server is not None:
            self._polling_server.push_figure(self.fig)

    def close(self) -> None:
        """Stop the polling server, when there is one."""
        if self._polling_server is not None:
            self._polling_server.shutdown()
            self._polling_server = None


###############################################################################
# Internal machinery
###############################################################################


def create_figure(spec: SignalPlotSpec, **kwargs):
    """One subplot row per trace on a shared time axis; the other ``kwargs`` update the layout."""
    go, make_subplots = require_plotly()

    # The size: the caller's, or one row height per trace above a minimum height
    kwargs = dict(kwargs)
    width = kwargs.pop("width", PLOTLY_FIG_WIDTH)
    height = kwargs.pop(
        "height",
        max(PLOTLY_SIGNAL_MIN_HEIGHT, PLOTLY_SIGNAL_ROW_HEIGHT * len(spec.traces)),
    )

    # Forwarded from live/open paths for Matplotlib; not valid Plotly layout keys.
    kwargs.pop("pause", None)
    kwargs.pop("block", None)

    # One row per trace, its y axis titled with the trace's label and unit
    fig = make_subplots(
        rows=len(spec.traces),
        cols=1,
        shared_xaxes=True,
        vertical_spacing=0.03,
    )
    for row, trace in enumerate(spec.traces, start=1):
        fig.add_trace(
            go.Scatter(
                x=spec.t,
                y=trace.values,
                mode="lines",
                name=trace.label,
                line={
                    "color": plotly_color(trace.color),
                    "width": trace.linewidth,
                },
                opacity=trace.alpha,
            ),
            row=row,
            col=1,
        )
        ylabel = ylabel_with_unit(trace.label, trace.unit)
        fig.update_yaxes(title_text=ylabel, row=row, col=1)

    # The time axis under the bottom row, then the layout, the caller's keys last
    fig.update_xaxes(title_text=spec.abscissa_label, row=len(spec.traces), col=1)
    fig.update_layout(
        title=spec.title,
        width=width,
        height=height,
        showlegend=False,
        margin=dict(PLOTLY_SIGNAL_MARGIN),
        template=PLOTLY_TEMPLATE,
        hovermode="x unified",
    )
    if kwargs:
        fig.update_layout(**kwargs)

    return fig


def require_plotly():
    """Refuse to draw without Plotly; return ``(go, make_subplots)``."""
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:
        raise ImportError(
            "Plotly signal plots require Plotly. Install with: pip install plotly",
        ) from exc

    return go, make_subplots


def ylabel_with_unit(label: str, unit: str) -> str:
    """The y-axis title ``label [unit]``; a unit already in brackets keeps them, as in the matplotlib backend."""
    if not unit:
        return label

    unit = str(unit).strip()
    if unit.startswith("[") and unit.endswith("]"):
        return f"{label} {unit}"

    return f"{label} [{unit}]"


class PlotlyPollingServer:
    """Serves the latest figure JSON on localhost for live updates in a browser."""

    def __init__(self):
        self._holder = {"lock": threading.Lock(), "json": None}
        self._httpd: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self, initial_fig) -> None:
        """Serve ``initial_fig`` on localhost and open its page in the browser."""
        # The first figure, as JSON
        with self._holder["lock"]:
            self._holder["json"] = json.loads(initial_fig.to_json())

        # The server, on a port the system picks, in a daemon thread
        self._httpd = PlotlyPollHTTPServer(
            ("127.0.0.1", 0),
            PlotlyPollHandler,
            self._holder,
        )
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        assert self._httpd is not None

        # The page, which polls the figure
        port = self._httpd.server_address[1]
        webbrowser.open(f"http://127.0.0.1:{port}/")

    def push_figure(self, fig) -> None:
        """Serve ``fig`` from now on; the page picks it up at its next poll."""
        with self._holder["lock"]:
            self._holder["json"] = json.loads(fig.to_json())

    def shutdown(self) -> None:
        """Stop the server and join its thread; a second call does nothing."""
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


class PlotlyPollHTTPServer(HTTPServer):
    """A localhost server that holds the figure JSON its handlers serve."""

    allow_reuse_address = True

    def __init__(self, server_address, handler_cls, plotly_json_holder):
        self.plotly_json_holder = plotly_json_holder
        super().__init__(server_address, handler_cls)


# Minimal page: polls full figure JSON so the browser view stays in sync with Python.
# A plain ``go.Figure`` opened with ``show()`` is a one-shot snapshot; mutations do not
# reach an external browser tab.
_PLOTLY_POLL_HTML = """<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8"/>
<title>minilink live plot</title>
<script src="https://cdn.plot.ly/plotly-2.35.2.min.js"></script>
</head>
<body style="margin:0">
<div id="plot" style="width:100%;height:100vh"></div>
<script>
let ready = false;
const el = document.getElementById("plot");
async function poll() {
  const r = await fetch("/data");
  if (!r.ok) return;
  const fig = await r.json();
  if (!ready) {
    Plotly.newPlot(el, fig.data, fig.layout, {responsive: true});
    ready = true;
  } else {
    Plotly.react(el, fig.data, fig.layout);
  }
}
setInterval(poll, 200);
poll();
</script>
</body>
</html>
"""


class PlotlyPollHandler(BaseHTTPRequestHandler):
    """Serves the page at ``/`` and the latest figure JSON at ``/data``."""

    def do_GET(self):
        # The page
        if self.path == "/":
            body = _PLOTLY_POLL_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        # The latest figure; 503 while there is none yet
        if self.path == "/data":
            holder = self.server.plotly_json_holder
            with holder["lock"]:
                payload = holder["json"]
            if payload is None:
                self.send_error(503)
                return
            raw = json.dumps(payload, separators=(",", ":")).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(raw)))
            self.end_headers()
            self.wfile.write(raw)
            return

        self.send_error(404)

    def log_message(self, fmt, *args):
        """Keep the per-request log off the console."""
        return


def redraw_traces(fig, spec, title):
    """Move each trace of ``fig`` to the spec's samples and label; retitle it when ``title`` is given."""
    for trace_obj, trace in zip(fig.data, spec.traces, strict=True):
        trace_obj.x = spec.t
        trace_obj.y = trace.values
        trace_obj.name = trace.label
    if title is not None:
        fig.update_layout(title=title)
