import {
  Background,
  Controls,
  MiniMap,
  ReactFlow,
  ReactFlowProvider,
  addEdge,
  useEdgesState,
  useNodesState,
  useReactFlow,
  MarkerType,
} from "@xyflow/react";
import "@xyflow/react/dist/style.css";
import { useCallback, useEffect, useMemo, useState } from "react";
import { BlockNode } from "./BlockNode";

const nodeTypes = { block: BlockNode };

function handleToPort(handleId) {
  if (!handleId) return "";
  if (handleId.startsWith("in_")) return handleId.slice(3);
  if (handleId.startsWith("out_")) return handleId.slice(4);
  return handleId;
}

function specToFlow(spec, catalog, layout) {
  const nodes = spec.nodes.map((n, i) => {
    const sig = catalog[n.kind] || { inputs: [], outputs: [] };
    const pos = layout[n.id] || {
      x: 40 + (i % 3) * 260,
      y: 40 + Math.floor(i / 3) * 160,
    };
    return {
      id: n.id,
      type: "block",
      position: pos,
      data: {
        id: n.id,
        kind: n.kind,
        inputs: [...sig.inputs],
        outputs: [...sig.outputs],
      },
    };
  });
  const edges = spec.edges.map((e, idx) => ({
    id: `e-${idx}-${e.source_sys}-${e.source_port}-${e.target_sys}-${e.target_port}`,
    source: e.source_sys,
    target: e.target_sys,
    sourceHandle: `out_${e.source_port}`,
    targetHandle: `in_${e.target_port}`,
    markerEnd: { type: MarkerType.ArrowClosed, color: "#7aa2f7" },
  }));
  return { nodes, edges };
}

function layoutFromNodes(rf) {
  const positions = {};
  rf.getNodes().forEach((n) => {
    positions[n.id] = { x: n.position.x, y: n.position.y };
  });
  return positions;
}

function EditorCanvas() {
  const [catalog, setCatalog] = useState(null);
  const [spec, setSpec] = useState(null);
  const [layout, setLayout] = useState({});
  const [nodes, setNodes, onNodesChange] = useNodesState([]);
  const [edges, setEdges, onEdgesChange] = useEdgesState([]);
  const rf = useReactFlow();

  const reload = useCallback(async () => {
    const [cRes, sRes] = await Promise.all([
      fetch("/api/catalog"),
      fetch("/api/state"),
    ]);
    const c = await cRes.json();
    const body = await sRes.json();
    setCatalog(c);
    setSpec(body.spec);
    const lay = body.layout || {};
    setLayout(lay);
    const { nodes: n, edges: e } = specToFlow(body.spec, c, lay);
    setNodes(n);
    setEdges(e);
  }, [setEdges, setNodes]);

  useEffect(() => {
    reload();
  }, [reload]);

  const persist = useCallback(async (nextSpec, nextLayout) => {
    const r = await fetch("/api/state", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ spec: nextSpec, layout: nextLayout }),
    });
    if (!r.ok) {
      const err = await r.json().catch(() => ({}));
      window.alert(err.detail || "Save failed");
      await reload();
    }
  }, [reload]);

  const onConnect = useCallback(
    (params) => {
      if (params.source === params.target) return;
      const newEdge = {
        source_sys: params.source,
        source_port: handleToPort(params.sourceHandle),
        target_sys: params.target,
        target_port: handleToPort(params.targetHandle),
      };
      setEdges((eds) =>
        addEdge(
          {
            ...params,
            markerEnd: { type: MarkerType.ArrowClosed, color: "#7aa2f7" },
          },
          eds,
        ),
      );
      setSpec((prev) => {
        if (!prev) return prev;
        const filtered = prev.edges.filter(
          (x) =>
            !(
              x.target_sys === newEdge.target_sys &&
              x.target_port === newEdge.target_port
            ),
        );
        const next = { ...prev, edges: [...filtered, newEdge] };
        const positions = layoutFromNodes(rf);
        setLayout(positions);
        persist(next, positions);
        return next;
      });
    },
    [persist, rf, setEdges],
  );

  const onEdgesDelete = useCallback(
    (deleted) => {
      setSpec((prev) => {
        if (!prev) return prev;
        const nextEdges = prev.edges.filter(
          (e) =>
            !deleted.some(
              (d) =>
                d.source === e.source_sys &&
                handleToPort(d.sourceHandle) === e.source_port &&
                d.target === e.target_sys &&
                handleToPort(d.targetHandle) === e.target_port,
            ),
        );
        const next = { ...prev, edges: nextEdges };
        const positions = layoutFromNodes(rf);
        setLayout(positions);
        persist(next, positions);
        return next;
      });
    },
    [persist, rf],
  );

  const onNodeDragStop = useCallback(() => {
    const positions = layoutFromNodes(rf);
    setLayout(positions);
    setSpec((cur) => {
      if (cur) persist(cur, positions);
      return cur;
    });
  }, [persist, rf]);

  const addBlock = useCallback(
    (id, kind) => {
      if (!id || !spec || !catalog) return;
      if (spec.nodes.some((n) => n.id === id)) {
        window.alert("Duplicate id");
        return;
      }
      const next = {
        ...spec,
        nodes: [...spec.nodes, { id, kind, params: {} }],
      };
      const positions = layoutFromNodes(rf);
      const nextLayout = {
        ...positions,
        [id]: { x: 100 + Math.random() * 140, y: 100 + Math.random() * 100 },
      };
      setSpec(next);
      setLayout(nextLayout);
      const { nodes: n, edges: e } = specToFlow(next, catalog, nextLayout);
      setNodes(n);
      setEdges(e);
      persist(next, nextLayout);
    },
    [catalog, persist, rf, setEdges, setNodes, spec],
  );

  const ready = catalog && spec;

  const defaultEdgeOptions = useMemo(
    () => ({ markerEnd: { type: MarkerType.ArrowClosed, color: "#7aa2f7" } }),
    [],
  );

  const kindKeys = catalog ? Object.keys(catalog).sort() : [];

  return (
    <div style={{ height: "100%", display: "flex", flexDirection: "column" }}>
      <div className="toolbar">
        <span style={{ opacity: 0.85 }}>
          Drag from an output (right) to an input (left). Delete edges with Backspace.
        </span>
        <input id="new-id" placeholder="new block id" />
        <select id="new-kind" defaultValue={kindKeys[0] || ""}>
          {kindKeys.map((k) => (
            <option key={k} value={k}>
              {k}
            </option>
          ))}
        </select>
        <button
          type="button"
          onClick={() => {
            const idEl = document.getElementById("new-id");
            const kindEl = document.getElementById("new-kind");
            addBlock(idEl.value.trim(), kindEl.value);
            idEl.value = "";
          }}
        >
          Add block
        </button>
        <button type="button" onClick={() => reload()}>
          Reload
        </button>
      </div>
      <div style={{ flex: 1, minHeight: 0 }}>
        {ready ? (
          <ReactFlow
            nodes={nodes}
            edges={edges}
            onNodesChange={onNodesChange}
            onEdgesChange={onEdgesChange}
            onConnect={onConnect}
            onEdgesDelete={onEdgesDelete}
            onNodeDragStop={onNodeDragStop}
            nodeTypes={nodeTypes}
            fitView
            snapToGrid
            defaultEdgeOptions={defaultEdgeOptions}
          >
            <Background />
            <Controls />
            <MiniMap />
          </ReactFlow>
        ) : (
          <div style={{ padding: 16 }}>Loading…</div>
        )}
      </div>
    </div>
  );
}

export default function App() {
  return (
    <ReactFlowProvider>
      <EditorCanvas />
    </ReactFlowProvider>
  );
}
