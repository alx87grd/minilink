#!/usr/bin/env python3
"""
Minimal diagram editor: JSON topology in memory, SVG preview via Graphviz.

Run from the repository root::

    python examples/diagram_editor_mvp/server.py

Then open http://127.0.0.1:8765 in a browser. **Click an output port, then an
input port** on the Graphviz SVG to add a connection (inline SVG, not an ``<img>``).

Requires the ``graphviz`` Python package and system Graphviz (``dot``), same as
``minilink.graphical.graphe``.
"""

from __future__ import annotations

import json
import threading
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from textwrap import dedent
from urllib.parse import urlparse

from minilink.core.diagram_topology import (
    DiagramTopology,
    build_diagram_from_topology,
    default_mvp_catalog,
    mvp_port_signature,
    topology_loads,
)

_HOST = "127.0.0.1"
_PORT = 8765

_INDEX_HTML = dedent(
    """\
    <!DOCTYPE html>
    <html lang="en">
    <head>
      <meta charset="utf-8" />
      <meta name="viewport" content="width=device-width, initial-scale=1" />
      <title>minilink diagram MVP</title>
      <style>
        :root { font-family: system-ui, sans-serif; background: #1a1b26; color: #c0caf5; }
        a { color: #7aa2f7; }
        main { max-width: 960px; margin: 1rem auto; padding: 0 1rem; }
        h1 { font-weight: 600; font-size: 1.25rem; }
        section { margin-bottom: 1.5rem; border: 1px solid #3b4261; border-radius: 8px; padding: 1rem; background: #16161e; }
        label { display: block; margin: 0.35rem 0 0.1rem; font-size: 0.85rem; color: #a9b1d6; }
        input, select, button { font: inherit; padding: 0.35rem 0.5rem; border-radius: 4px; border: 1px solid #565f89; background: #24283b; color: #c0caf5; }
        button { cursor: pointer; background: #414868; border-color: #7aa2f7; }
        button.secondary { background: #24283b; border-color: #565f89; }
        .row { display: flex; flex-wrap: wrap; gap: 0.5rem; align-items: flex-end; margin-top: 0.5rem; }
        .row > div { flex: 1; min-width: 120px; }
        pre { background: #11111b; padding: 0.75rem; overflow: auto; font-size: 0.8rem; border-radius: 6px; border: 1px solid #3b4261; }
        #svgWrap { width: 100%; min-height: 200px; border: 1px dashed #414868; border-radius: 6px; background: #fff; overflow: auto; }
        #svgWrap svg { display: block; max-width: 100%; height: auto; }
        #svgWrap a { cursor: crosshair; }
        ul.edges { list-style: none; padding: 0; margin: 0; }
        ul.edges li { display: flex; justify-content: space-between; align-items: center; padding: 0.35rem 0; border-bottom: 1px solid #2a2e3f; font-size: 0.9rem; }
        .muted { color: #565f89; font-size: 0.8rem; }
      </style>
    </head>
    <body>
      <main>
        <h1>Diagram topology MVP</h1>
        <p class="muted">Click an <strong>output</strong> port (right column), then an <strong>input</strong> port (left column) on another block.</p>
        <p id="status" class="muted" style="margin-top:0.5rem;"></p>
        <section>
          <h2 style="margin-top:0;font-size:1rem;">Add block</h2>
          <div class="row">
            <div><label>id</label><input id="nid" placeholder="e.g. plant" /></div>
            <div><label>kind</label><select id="nkind"></select></div>
            <div><button type="button" id="btnAdd">Add node</button></div>
          </div>
          <p class="muted">Params use defaults; edit JSON below for <code>Kp</code>, <code>x0</code>, etc.</p>
        </section>
        <section>
          <h2 style="margin-top:0;font-size:1rem;">Edges</h2>
          <ul class="edges" id="edgeList"></ul>
        </section>
        <section>
          <h2 style="margin-top:0;font-size:1rem;">Diagram (click to wire)</h2>
          <div class="row" style="margin-bottom:0.5rem;">
            <button type="button" class="secondary" id="btnClearPick">Clear port pick</button>
          </div>
          <div id="svgWrap"></div>
        </section>
        <section>
          <h2 style="margin-top:0;font-size:1rem;">JSON</h2>
          <p class="muted">Save/load this file outside the browser; &quot;Apply JSON&quot; parses and refreshes.</p>
          <textarea id="json" rows="16" style="width:100%;font-family:ui-monospace,monospace;font-size:0.8rem;background:#11111b;color:#c0caf5;border:1px solid #3b4261;border-radius:6px;padding:0.5rem;"></textarea>
          <div class="row" style="margin-top:0.75rem;">
            <button type="button" id="btnApply">Apply JSON</button>
            <button type="button" class="secondary" id="btnReload">Reload from server</button>
          </div>
        </section>
      </main>
      <script>
    const state = { spec: null, catalog: null };
    let wireSrc = null;
    function setStatus(t) {
      document.getElementById('status').textContent = t;
    }
    function portLists(kind, direction) {
      const sig = state.catalog[kind];
      if (!sig) return [];
      return [...sig[direction]];
    }
    function parsePortHref(h) {
      if (!h || h[0] !== '#') return null;
      const raw = h.slice(1);
      const q = raw.indexOf('?');
      if (q < 0 || raw.slice(0, 2) !== 'ml') return null;
      const params = new URLSearchParams(raw.slice(q + 1));
      const dir = params.get('dir');
      const sys = decodeURIComponent(params.get('sys') || '');
      const port = decodeURIComponent(params.get('port') || '');
      if (!dir || !sys || !port) return null;
      return { dir, sys, port };
    }
    function onSvgWrapClick(e) {
      const a = e.target.closest('a');
      if (!a) return;
      const h = a.getAttribute('href') || a.getAttributeNS('http://www.w3.org/1999/xlink', 'href');
      const p = parsePortHref(h);
      if (!p) return;
      e.preventDefault();
      e.stopPropagation();
      const node = state.spec.nodes.find((x) => x.id === p.sys);
      if (!node) {
        setStatus('Unknown block in diagram');
        return;
      }
      if (p.dir === 'out') {
        const outs = portLists(node.kind, 'outputs');
        if (!outs.includes(p.port)) {
          setStatus('That cell is not an output for this block');
          return;
        }
        wireSrc = { sys: p.sys, port: p.port };
        setStatus('Picked output ' + p.sys + ':' + p.port + ' — click an input port');
        return;
      }
      if (p.dir === 'in') {
        const ins = portLists(node.kind, 'inputs');
        if (!ins.includes(p.port)) {
          setStatus('That cell is not an input for this block');
          return;
        }
        if (!wireSrc) {
          setStatus('Click an output port first');
          return;
        }
        if (wireSrc.sys === p.sys) {
          setStatus('Pick an input on a different block');
          return;
        }
        const edge = {
          source_sys: wireSrc.sys,
          source_port: wireSrc.port,
          target_sys: p.sys,
          target_port: p.port,
        };
        const dup = state.spec.edges.some(
          (x) => x.target_sys === edge.target_sys && x.target_port === edge.target_port,
        );
        if (dup) {
          setStatus('That input already has a connection');
          wireSrc = null;
          return;
        }
        state.spec.edges.push(edge);
        wireSrc = null;
        setStatus(
          'Connected ' + edge.source_sys + ':' + edge.source_port + ' → ' + edge.target_sys + ':' + edge.target_port,
        );
        push();
      }
    }
    function kindSelect() {
      const sel = document.getElementById('nkind');
      sel.innerHTML = '';
      for (const k of Object.keys(state.catalog).sort()) {
        const o = document.createElement('option');
        o.value = k;
        o.textContent = k;
        sel.appendChild(o);
      }
    }
    function renderEdges() {
      const ul = document.getElementById('edgeList');
      ul.innerHTML = '';
      state.spec.edges.forEach((e, idx) => {
        const li = document.createElement('li');
        const lab = document.createElement('span');
        lab.textContent = e.source_sys + ':' + e.source_port + ' → ' + e.target_sys + ':' + e.target_port;
        const btn = document.createElement('button');
        btn.type = 'button';
        btn.className = 'secondary';
        btn.textContent = 'Remove';
        btn.onclick = () => {
          state.spec.edges = state.spec.edges.filter((_, j) => j !== idx);
          push();
        };
        li.appendChild(lab);
        li.appendChild(btn);
        ul.appendChild(li);
      });
    }
    async function loadInlineSvg() {
      const host = document.getElementById('svgWrap');
      const r = await fetch('/api/graph.svg?_=' + Date.now());
      const txt = await r.text();
      host.innerHTML = txt;
    }
    async function pull() {
      const r = await fetch('/api/spec');
      state.spec = await r.json();
      document.getElementById('json').value = JSON.stringify(state.spec, null, 2);
      if (!state.catalog) {
        const c = await fetch('/api/catalog');
        state.catalog = await c.json();
        kindSelect();
      }
      renderEdges();
      await loadInlineSvg();
      setStatus(
        wireSrc
          ? 'Picked output ' + wireSrc.sys + ':' + wireSrc.port + ' — click an input'
          : 'Click an output port, then an input port',
      );
    }
    async function push() {
      document.getElementById('json').value = JSON.stringify(state.spec, null, 2);
      const r = await fetch('/api/spec', {
        method: 'POST',
        body: JSON.stringify(state.spec),
        headers: { 'Content-Type': 'application/json' },
      });
      const j = await r.json();
      if (!j.ok) {
        alert(j.error || 'save failed');
        return;
      }
      renderEdges();
      await loadInlineSvg();
    }
    document.getElementById('svgWrap').addEventListener('click', onSvgWrapClick, true);
    document.getElementById('btnClearPick').onclick = () => {
      wireSrc = null;
      setStatus('Cleared — click an output port, then an input');
    };
    document.getElementById('btnAdd').onclick = () => {
      const id = document.getElementById('nid').value.trim();
      const kind = document.getElementById('nkind').value;
      if (!id) {
        alert('id required');
        return;
      }
      if (state.spec.nodes.some((n) => n.id === id)) {
        alert('duplicate id');
        return;
      }
      state.spec.nodes.push({ id, kind, params: {} });
      document.getElementById('nid').value = '';
      push();
    };
    document.getElementById('btnApply').onclick = async () => {
      try {
        state.spec = JSON.parse(document.getElementById('json').value);
        await push();
      } catch (e) {
        alert('invalid JSON: ' + e);
      }
    };
    document.getElementById('btnReload').onclick = pull;
    pull();
      </script>
    </body>
    </html>
    """
).encode("utf-8")

_lock = threading.Lock()


def _initial_state() -> str:
    from minilink.core.diagram_topology import topology_dumps

    return topology_dumps(
        DiagramTopology.from_json_dict(
            {
                "version": 1,
                "name": "Demo",
                "nodes": [
                    {
                        "id": "step",
                        "kind": "step",
                        "params": {
                            "initial_value": [0.0],
                            "final_value": [20.0],
                            "step_time": 10.0,
                        },
                    },
                    {"id": "c1", "kind": "prop_controller", "params": {"Kp": 10.0}},
                    {"id": "c2", "kind": "prop_controller", "params": {"Kp": 10.0}},
                    {
                        "id": "i1",
                        "kind": "integrator",
                        "params": {"x0": [20.0], "state_label": "v"},
                    },
                    {
                        "id": "i2",
                        "kind": "integrator",
                        "params": {"x0": [20.0], "state_label": "x"},
                    },
                ],
                "edges": [
                    {
                        "source_sys": "i1",
                        "source_port": "y",
                        "target_sys": "i2",
                        "target_port": "u",
                    },
                    {
                        "source_sys": "c2",
                        "source_port": "u",
                        "target_sys": "i1",
                        "target_port": "u",
                    },
                    {
                        "source_sys": "i1",
                        "source_port": "y",
                        "target_sys": "c2",
                        "target_port": "y",
                    },
                    {
                        "source_sys": "c1",
                        "source_port": "u",
                        "target_sys": "c2",
                        "target_port": "ref",
                    },
                    {
                        "source_sys": "i2",
                        "source_port": "y",
                        "target_sys": "c1",
                        "target_port": "y",
                    },
                    {
                        "source_sys": "step",
                        "source_port": "y",
                        "target_sys": "c1",
                        "target_port": "ref",
                    },
                ],
            }
        )
    )


_state_holder = [_initial_state()]


def _diagram_from_state() -> DiagramTopology:
    with _lock:
        return topology_loads(_state_holder[0])


def _svg_bytes() -> bytes:
    top = _diagram_from_state()
    diagram = build_diagram_from_topology(top, default_mvp_catalog())
    graphe = diagram.get_graphe(port_links=True)
    if graphe is None:
        return b"<!-- graphviz unavailable -->"
    return graphe.pipe(format="svg")


class _Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt: str, *args: object) -> None:
        # Quieter default for local dev
        print(f"[diagram_editor] {fmt % args}")

    def _send(self, code: int, body: bytes, content_type: str) -> None:
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path in ("/", "/index.html"):
            self._send(HTTPStatus.OK, _INDEX_HTML, "text/html; charset=utf-8")
            return
        if parsed.path == "/api/spec":
            with _lock:
                body = _state_holder[0].encode("utf-8")
            self._send(HTTPStatus.OK, body, "application/json; charset=utf-8")
            return
        if parsed.path == "/api/catalog":
            body = json.dumps(mvp_port_signature(), sort_keys=True).encode("utf-8")
            self._send(HTTPStatus.OK, body, "application/json; charset=utf-8")
            return
        if parsed.path == "/api/graph.svg":
            try:
                svg = _svg_bytes()
            except Exception as exc:
                msg = f"<!-- render error: {exc} -->".encode("utf-8")
                self._send(HTTPStatus.OK, msg, "image/svg+xml; charset=utf-8")
                return
            self._send(HTTPStatus.OK, svg, "image/svg+xml; charset=utf-8")
            return
        self._send(HTTPStatus.NOT_FOUND, b"not found", "text/plain; charset=utf-8")

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path != "/api/spec":
            self._send(HTTPStatus.NOT_FOUND, b"not found", "text/plain; charset=utf-8")
            return
        length = int(self.headers.get("Content-Length", "0"))
        raw = self.rfile.read(length)
        try:
            data = json.loads(raw.decode("utf-8"))
            DiagramTopology.from_json_dict(data)
        except (json.JSONDecodeError, ValueError, KeyError) as exc:
            self._send(
                HTTPStatus.BAD_REQUEST,
                json.dumps({"ok": False, "error": str(exc)}).encode("utf-8"),
                "application/json; charset=utf-8",
            )
            return
        with _lock:
            _state_holder[0] = json.dumps(data, indent=2, sort_keys=True)
        self._send(
            HTTPStatus.OK,
            json.dumps({"ok": True}).encode("utf-8"),
            "application/json; charset=utf-8",
        )


def main() -> None:
    server = ThreadingHTTPServer((_HOST, _PORT), _Handler)
    print(f"Diagram editor MVP: http://{_HOST}:{_PORT}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("shutdown")


if __name__ == "__main__":
    main()
