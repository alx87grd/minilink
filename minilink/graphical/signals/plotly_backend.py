"""Time-signal plots in Plotly: one subplot row per trace, drawn once or kept live."""

from __future__ import annotations

import json
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, HTTPServer

from minilink.graphical.common.environment import detect_env, is_headless
from minilink.graphical.figure import draw
from minilink.graphical.signals.time_signals import (
    LivePlotHandle,
    figure_of_update,
    require_same_trace_count,
)

###############################################################################
# Public API
###############################################################################


def open_plotly_signal_plot(figure, *, figure_of=None, show=True):
    """Draw ``figure`` with Plotly and keep it live: a widget in a notebook, a polled page elsewhere."""
    fig = draw(figure, renderer="plotly", show=False).figure

    # Not shown: the figure alone, updated in place
    if not show or is_headless():
        return PlotlyLivePlotHandle(fig, figure_of=figure_of)

    # In a notebook: a FigureWidget under the cell, updated in place
    env = detect_env()
    if env in ("jupyter", "colab"):
        # Lazy import: Plotly is optional, loaded with the first live figure
        import plotly.graph_objects as go

        try:
            fw = go.FigureWidget(fig)
        except (TypeError, ValueError, ImportError):
            # Plotly 6+ may require ``anywidget``; fall back to localhost polling.
            polling = PlotlyPollingServer()
            polling.start(fig)
            return PlotlyLivePlotHandle(
                fig, figure_of=figure_of, polling_server=polling
            )

        try:
            from IPython.display import display

            display(fw)
        except ImportError:
            fw.show()

        return PlotlyLivePlotHandle(fw, figure_of=figure_of)

    # Elsewhere: a browser page on localhost polls the latest figure
    polling = PlotlyPollingServer()
    polling.start(fig)

    return PlotlyLivePlotHandle(fig, figure_of=figure_of, polling_server=polling)


class PlotlyLivePlotHandle(LivePlotHandle):
    """Live Plotly trace updater (Figure, FigureWidget, or polling server)."""

    def __init__(
        self,
        fig,
        *,
        figure_of=None,
        polling_server: PlotlyPollingServer | None = None,
    ):
        self.fig = fig
        self.figure_of = figure_of
        self._polling_server = polling_server

    def update(self, traj_or_figure, *, title: str | None = None) -> None:
        """Redraw each trace from a new trajectory or description, with the same panels."""
        spec = figure_of_update(traj_or_figure, self.figure_of, title=title)

        # In place: the same traces take the new samples, in one batched redraw
        # when the figure has one (a FigureWidget)
        require_same_trace_count(spec, len(self.fig.data))
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
    """Move each trace of ``fig`` to its panel's samples and label; retitle it when ``title`` is given."""
    for trace_obj, panel in zip(fig.data, spec.panels, strict=True):
        trace = panel.layers[0]
        trace_obj.x = trace.x
        trace_obj.y = trace.y
        trace_obj.name = trace.label
    if title is not None:
        fig.update_layout(title=title)
