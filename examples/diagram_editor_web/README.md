# Parallel diagram editor (web)

Interactive block wiring using **[React Flow](https://reactflow.dev/)** (`@xyflow/react`): drag from an **output** handle (right) to an **input** handle (left). This lives entirely under `examples/` and does not change minilink core.

The Python server only validates and stores JSON using `minilink.core.diagram_topology` (same `spec` shape as the topology helpers: `version`, `name`, `nodes`, `edges`). Optional `layout` holds `{ "block_id": { "x", "y" } }` for node positions.

## Why React Flow

Graphviz is excellent for static layout and PDF/SVG export; it is a poor fit for hit-tested ports and live edge editing. React Flow is MIT-licensed, widely used for node editors, and gives first-class support for custom nodes, handles, pan/zoom, and edge CRUD.

## Setup

```bash
# from repository root
pip install -e ".[diagram-editor]"

cd examples/diagram_editor_web
npm install
npm run build
uvicorn server:app --host 127.0.0.1 --port 8765
```

Open `http://127.0.0.1:8765` (API + static UI on one port after build).

## Dev (hot reload)

Terminal A:

```bash
cd examples/diagram_editor_web
uvicorn server:app --host 127.0.0.1 --port 8765
```

Terminal B:

```bash
cd examples/diagram_editor_web
npm run dev
```

Open the Vite URL (usually `http://127.0.0.1:5173`); `/api` is proxied to the FastAPI server.
