"""
Parallel diagram editor: FastAPI serves the built Vite + React Flow UI and JSON API.

Does not import or modify minilink core beyond ``diagram_topology`` helpers used
for validation and catalog metadata.

Run (after ``npm run build`` in this directory)::

    cd examples/diagram_editor_web
    uvicorn server:app --host 127.0.0.1 --port 8765

Dev with hot reload (two terminals): start this server on 8765, then
``npm run dev`` (Vite proxies ``/api`` to 8765).
"""

from __future__ import annotations

import json
import threading
from pathlib import Path

from fastapi import FastAPI, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.staticfiles import StaticFiles

from minilink.core.diagram_topology import DiagramTopology, mvp_port_signature

_ROOT = Path(__file__).resolve().parent
_DIST = _ROOT / "dist"

_lock = threading.Lock()

_DEFAULT_SPEC = {
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
        {"source_sys": "i1", "source_port": "y", "target_sys": "i2", "target_port": "u"},
        {"source_sys": "c2", "source_port": "u", "target_sys": "i1", "target_port": "u"},
        {"source_sys": "i1", "source_port": "y", "target_sys": "c2", "target_port": "y"},
        {"source_sys": "c1", "source_port": "u", "target_sys": "c2", "target_port": "ref"},
        {"source_sys": "i2", "source_port": "y", "target_sys": "c1", "target_port": "y"},
        {"source_sys": "step", "source_port": "y", "target_sys": "c1", "target_port": "ref"},
    ],
}

_state: dict[str, object] = {
    "spec": json.dumps(_DEFAULT_SPEC, sort_keys=True),
    "layout": "{}",
}


def _get_spec_dict() -> dict:
    with _lock:
        return json.loads(str(_state["spec"]))


def _get_layout_dict() -> dict:
    with _lock:
        return json.loads(str(_state["layout"]))


app = FastAPI(title="minilink diagram editor")
app.add_middleware(
    CORSMiddleware,
    allow_origins=[
        "http://127.0.0.1:5173",
        "http://localhost:5173",
        "http://127.0.0.1:8765",
        "http://localhost:8765",
    ],
    allow_methods=["*"],
    allow_headers=["*"],
)


@app.get("/api/catalog")
def api_catalog() -> dict:
    """Port names per MVP block kind (matches :func:`mvp_port_signature`)."""
    sig = mvp_port_signature()
    return {k: {"inputs": list(v["inputs"]), "outputs": list(v["outputs"])} for k, v in sig.items()}


@app.get("/api/state")
def api_state_get() -> dict:
    return {"spec": _get_spec_dict(), "layout": _get_layout_dict()}


@app.post("/api/state")
def api_state_post(body: dict) -> dict:
    if "spec" not in body:
        raise HTTPException(status_code=400, detail="missing spec")
    try:
        DiagramTopology.from_json_dict(body["spec"])
    except (KeyError, TypeError, ValueError) as exc:
        raise HTTPException(status_code=400, detail=str(exc)) from exc
    layout = body.get("layout") or {}
    if not isinstance(layout, dict):
        raise HTTPException(status_code=400, detail="layout must be an object")
    dumped = json.dumps(body["spec"], sort_keys=True)
    layout_dumped = json.dumps(layout, sort_keys=True)
    with _lock:
        _state["spec"] = dumped
        _state["layout"] = layout_dumped
    return {"ok": True}


if (_DIST / "index.html").is_file():
    from fastapi.responses import FileResponse

    @app.get("/")
    def _index() -> FileResponse:
        return FileResponse(_DIST / "index.html")

    app.mount("/assets", StaticFiles(directory=_DIST / "assets"), name="assets")
