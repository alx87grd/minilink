"""Plotly backend for sampled time-signal plots."""

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
from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    PlotResult,
    SignalPlotSpec,
    _coerce_signal_plot_spec,
)

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


class _PlotlyPollHTTPServer(HTTPServer):
    allow_reuse_address = True

    def __init__(self, server_address, handler_cls, plotly_json_holder):
        self.plotly_json_holder = plotly_json_holder
        super().__init__(server_address, handler_cls)


class _PlotlyPollHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            body = _PLOTLY_POLL_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
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
        return


class _PlotlyPollingServer:
    """Serves the latest figure JSON on localhost for live updates in a browser."""

    def __init__(self):
        self._holder = {"lock": threading.Lock(), "json": None}
        self._httpd: HTTPServer | None = None
        self._thread: threading.Thread | None = None

    def start(self, initial_fig) -> None:
        with self._holder["lock"]:
            self._holder["json"] = json.loads(initial_fig.to_json())
        self._httpd = _PlotlyPollHTTPServer(
            ("127.0.0.1", 0),
            _PlotlyPollHandler,
            self._holder,
        )
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._thread.start()
        assert self._httpd is not None
        port = self._httpd.server_address[1]
        webbrowser.open(f"http://127.0.0.1:{port}/")

    def push_figure(self, fig) -> None:
        with self._holder["lock"]:
            self._holder["json"] = json.loads(fig.to_json())

    def shutdown(self) -> None:
        if self._httpd is not None:
            self._httpd.shutdown()
            self._httpd = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None


def render_plotly_signal_plot(
    spec: SignalPlotSpec,
    *,
    show: bool = True,
    **kwargs,
) -> PlotResult:
    """Render a one-shot time-signal plot with Plotly."""
    fig = _create_figure(spec, **kwargs)
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
    create_kw = dict(kwargs)
    if not show:
        fig = _create_figure(spec, **create_kw)
        return PlotlyLivePlotHandle(
            fig,
            spec_builder=spec_builder,
            create_kw=create_kw,
        )

    env = detect_env()
    if env in ("jupyter", "colab"):
        go, _ = _import_plotly()
        fig = _create_figure(spec, **create_kw)
        try:
            fw = go.FigureWidget(fig)
        except (TypeError, ValueError, ImportError):
            # Plotly 6+ may require ``anywidget``; fall back to localhost polling.
            polling = _PlotlyPollingServer()
            polling.start(fig)
            return PlotlyLivePlotHandle(
                fig,
                spec_builder=spec_builder,
                create_kw=create_kw,
                polling_server=polling,
            )
        try:
            from IPython.display import display

            display(fw)
        except ImportError:
            fw.show()
        return PlotlyLivePlotHandle(
            fw,
            spec_builder=spec_builder,
            create_kw=create_kw,
        )

    initial_fig = _create_figure(spec, **create_kw)
    polling = _PlotlyPollingServer()
    polling.start(initial_fig)
    return PlotlyLivePlotHandle(
        initial_fig,
        spec_builder=spec_builder,
        create_kw=create_kw,
        polling_server=polling,
    )


class PlotlyLivePlotHandle(LivePlotHandle):
    """Live Plotly trace updater (Figure, FigureWidget, or polling server)."""

    def __init__(
        self,
        fig,
        *,
        spec_builder=None,
        create_kw=None,
        polling_server: _PlotlyPollingServer | None = None,
    ):
        self.fig = fig
        self.spec_builder = spec_builder
        self._create_kw = dict(create_kw) if create_kw is not None else {}
        self._polling_server = polling_server

    def update(self, traj_or_spec, *, title: str | None = None) -> None:
        spec = _coerce_signal_plot_spec(
            traj_or_spec,
            self.spec_builder,
            title=title,
        )

        if self._polling_server is not None:
            new_fig = _create_figure(spec, **self._create_kw)
            self.fig = new_fig
            self._polling_server.push_figure(new_fig)
            return

        if len(spec.traces) != len(self.fig.data):
            raise ValueError(
                "Live plot update changed the number of traces; open a new plot.",
            )

        batch = getattr(self.fig, "batch_update", None)
        if callable(batch):
            with self.fig.batch_update():
                for trace_obj, trace in zip(self.fig.data, spec.traces, strict=True):
                    trace_obj.x = spec.t
                    trace_obj.y = trace.values
                    trace_obj.name = trace.label
                if title is not None:
                    self.fig.update_layout(title=title)
        else:
            for trace_obj, trace in zip(self.fig.data, spec.traces, strict=True):
                trace_obj.x = spec.t
                trace_obj.y = trace.values
                trace_obj.name = trace.label
            if title is not None:
                self.fig.update_layout(title=title)

    def close(self) -> None:
        if self._polling_server is not None:
            self._polling_server.shutdown()
            self._polling_server = None


def _create_figure(spec: SignalPlotSpec, **kwargs):
    go, make_subplots = _import_plotly()

    kwargs = dict(kwargs)
    width = kwargs.pop("width", PLOTLY_FIG_WIDTH)
    height = kwargs.pop(
        "height",
        max(PLOTLY_SIGNAL_MIN_HEIGHT, PLOTLY_SIGNAL_ROW_HEIGHT * len(spec.traces)),
    )
    # Forwarded from live/open paths for Matplotlib; not valid Plotly layout keys.
    kwargs.pop("pause", None)
    kwargs.pop("block", None)

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
                line={"color": _plotly_color(trace.color)},
            ),
            row=row,
            col=1,
        )
        ylabel = f"{trace.label} [{trace.unit}]" if trace.unit else trace.label
        fig.update_yaxes(title_text=ylabel, row=row, col=1)

    fig.update_xaxes(title_text="Time [s]", row=len(spec.traces), col=1)
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


def _import_plotly():
    try:
        import plotly.graph_objects as go
        from plotly.subplots import make_subplots
    except ImportError as exc:
        raise ImportError(
            "Plotly signal plots require Plotly. Install with: "
            "pip install 'minilink[plotting]'",
        ) from exc
    return go, make_subplots


def _plotly_color(color: str) -> str:
    colors = {
        "blue": "#0000ff",
        "tab:blue": "#1f77b4",
        "tab:red": "#d62728",
        "tab:green": "#2ca02c",
    }
    return colors.get(color, color)
