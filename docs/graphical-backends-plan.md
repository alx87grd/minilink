# Graphical backends plan: signals and topology

This document is an **implementation plan** for introducing small, swappable
backend interfaces for (1) time-series / trajectory plotting and (2) block-diagram
topology rendering—analogous in spirit to `AnimationRenderer` for kinematic
animation, but **intentionally thinner** than the `compile` / `simulation`
stacks.

Canonical design contracts remain in [DESIGN.md](../DESIGN.md); when public
behavior or package layout changes because of this work, update DESIGN (and
[ROADMAP.md](../ROADMAP.md) if maturity claims shift).

---

## 0. Goals and non-goals

### Goals

- **Signal plotting:** stable entry points (`plot_trajectory`, `plot_signals`)
  delegate to a **`SignalPlotBackend`** contract. **Matplotlib** remains the
  default implementation and default dependency story unless the project
  explicitly decides otherwise later.
- **Diagram topology:** a **neutral graph description** (or thin adapter) feeds
  **`DiagramTopologyExporter`** (or equivalent) implementations. **Graphviz**
  remains the default for **port-accurate** block diagrams.
- **Optional backends** (e.g. Plotly for interactive signals, Mermaid for
  documentation-style diagrams) install behind **optional extras** and use
  **lazy imports**, consistent with optional graphical stacks elsewhere.

### Non-goals (initial phases)

- Replacing matplotlib as a core dependency in the first iteration (the default
  backend can stay matplotlib-backed).
- Pixel-perfect equivalence between Graphviz and Mermaid (Mermaid is a
  **simplified / documentation** path; document limits).
- A single unified “graphics kernel” owning animation + signals + topology in one
  mega-class.

---

## 1. Public API and backward compatibility

- Preserve existing **call shapes** where possible: `plot_trajectory(sys, traj,
  *, plot="xu")`, `plot_signals(...)`, `plot_graphviz(...)`, and `System` /
  `DiagramSystem` graphe conveniences.
- Add an optional **`backend`** argument (string dispatch, e.g.
  `backend="matplotlib"`) **or** allow passing a backend instance for tests and
  advanced use—pick one primary style and document it.
- **Default behavior** must match current behavior so existing tests, examples,
  and notebooks stay green unless a deliberate breaking release is chosen.

---

## 2. Phase A — Import and global side-effect hygiene

**Problem:** `plotting.py` currently applies matplotlib global state and prints
at import time, which complicates optional backends and testing.

**Tasks**

- Move **`plt.ion()`**, **`matplotlib.rcParams` mutations**, and **stdout
  prints** off the unconditional import path.
- Apply interactive defaults **inside** the matplotlib backend when plotting (or
  behind an explicit `init_matplotlib_interactive()` used only by demos).

**Acceptance criteria**

- Importing `minilink.graphical.plotting` (or the façade module chosen in Phase
  B) does not print or mutate global matplotlib state unless documented.

---

## 3. Phase B — Signal plotting backends

### 3.1 Contract

- Add a small **`SignalPlotBackend`** (ABC or `typing.Protocol`) in a dedicated
  module, for example:
  - `minilink/graphical/signal_plotting.py`, or
  - `minilink/graphical/plotting/backends/base.py` if a subpackage is preferred.
- Methods (names illustrative):
  - `plot_trajectory(sys, traj, *, plot: str, **kwargs) -> PlotResult`
  - `plot_signals(sys, traj, signals, **kwargs) -> PlotResult`
- **`PlotResult`:** a minimal `@dataclass` or typed dict describing what was
  produced (`kind`, `figure`, `axes`, etc.). Plotly returns a different payload;
  do **not** force every backend to return `matplotlib.figure.Figure`.

### 3.2 Default implementation

- Implement **`MatplotlibSignalBackend`** by **refactoring** existing code from
  `plotting.py` without behavior changes.
- Keep **`matplotlib_style.py`** and **`environment.py`** as dependencies of
  the matplotlib backend only.

### 3.3 Registry

- Small registry: mapping **backend name → factory or class**, with a clear
  `ValueError` for unknown names.

### 3.4 Facade

- Public `plot_trajectory` / `plot_signals` become thin dispatch to the
  selected backend (default: matplotlib).

**Acceptance criteria**

- All existing plotting tests pass unchanged.
- At least one test constructs `MatplotlibSignalBackend` directly (smoke).

---

## 4. Phase C — Topology model and diagram exporters

### 4.1 Neutral topology model

- Introduce dataclasses (or similar) describing **nodes**, **ports**, and
  **edges** independent of Graphviz, for example in
  `minilink/graphical/topology_graph.py`.
- Provide a builder from **`DiagramSystem`** (and optionally from standalone
  `System`) so exporters do not duplicate wiring logic.

### 4.2 Exporter contract

- **`DiagramTopologyExporter`** (or separate small classes per backend) with
  backend-specific return types documented explicitly:
  - Graphviz: `graphviz.Digraph` (or render pipeline today).
  - Mermaid: `str` (source only).
  - Future web JSON: dict, etc.

### 4.3 Migrate Graphviz path

- Move graph construction from `graphe.py` toward **`GraphvizTopologyExporter`**
  consuming the neutral model.
- Keep **`get_system_graphe` / `get_diagram_graphe`** as thin wrappers during
  transition; deprecate in docstrings only if superseded by a single
  `export_diagram(...)` façade.

### 4.4 Display façade

- `plot_graphviz` remains for the Graphviz object **or** grows a sibling
  `show_diagram(..., backend="graphviz")` that dispatches.

**Acceptance criteria**

- Existing graphe / graphviz workflows produce the same artifacts for the
  default path (modulo intentional fixes).
- Unit tests on the neutral model: small synthetic diagram → expected structure.

---

## 5. Phase D — Optional second backends

### Signals (example: Plotly)

- Add **`plotly`** under a new or existing optional extra in `pyproject.toml`
  (e.g. extend `visualization` or add `plotting = ["plotly"]`).
- Implement **`PlotlySignalBackend`** with **lazy** `import plotly` inside methods.
- Tests marked `@pytest.mark.optional` (pattern already in `pyproject.toml`).

### Diagrams (example: Mermaid)

- Implement **`MermaidTopologyExporter`**: `TopologyGraph -> str`.
- Document **semantic limits** (ports, layout, edge routing) vs Graphviz.

**Acceptance criteria**

- CI default run does not require optional extras.
- One smoke test per optional backend behind the optional marker.

---

## 6. Phase E — Documentation

- **[DESIGN.md](../DESIGN.md)** §6 (graphics): document the two seams
  (`SignalPlotBackend`, topology exporters), default backends, and return-type
  policy.
- **[README.md](../README.md)**: one-line mention when a second backend ships
  (example snippet).
- **This file:** update phase checkboxes or status when phases complete (optional
  maintenance).

---

## 7. Testing strategy

| Layer | Tests |
|--------|--------|
| Topology model | Pure unit tests on node/edge counts and port ids |
| Graphviz exporter | Stable structural assertions on DOT or graph attributes (avoid brittle layout coordinates) |
| Matplotlib signal backend | Existing trajectory tests; import hygiene test |
| Optional backends | `@pytest.mark.optional`; minimal “runs without error” |

---

## 8. Risks and mitigations

| Risk | Mitigation |
|------|------------|
| Large single PR | Land Phase A, then B, then C, then D separately |
| Return type proliferation | Document `PlotResult` and per-exporter returns; avoid pretending all backends return `Figure` |
| Import cycles | Keep `core` free of backend imports; build topology from diagram APIs in `graphical` or via thin methods on `DiagramSystem` with lazy imports if needed |
| Mermaid vs Graphviz confusion | Document Mermaid as **overview / docs**, not a full port-pin replacement |

---

## 9. Target package layout (end state, illustrative)

```text
minilink/graphical/
  environment.py
  matplotlib_style.py
  plotting.py                 # thin façade and/or re-exports (optional)
  signal_plotting.py          # protocol + registry + public plot_* dispatch
  signal_backends/
    matplotlib_backend.py
    plotly_backend.py         # optional extra
  topology_graph.py           # neutral model + builder(s)
  diagram_export.py           # registry + export_diagram façade (optional)
  diagram_backends/
    graphviz_exporter.py
    mermaid_exporter.py
  graphe.py                   # HTML helpers + legacy wrappers; shrink over time
  animation.py
  renderers/
    ...
```

Exact filenames may change; keep **one obvious user import path** (either
retain `plotting.plot_trajectory` or re-export from `minilink.graphical` when
the public API freeze allows).

---

## 10. Execution order (summary)

1. **Phase A** — matplotlib import / print hygiene in plotting.
2. **Phase B** — `SignalPlotBackend` + matplotlib backend + dispatch.
3. **Phase C** — neutral topology model + Graphviz exporter refactor.
4. **Phase D** — optional Plotly + Mermaid (or prioritized subset).
5. **Phase E** — DESIGN / README updates aligned with shipped behavior.

---

## 11. Relation to existing patterns

- **`AnimationRenderer`** ([renderers/renderer.py](../minilink/graphical/renderers/renderer.py)):
  reference pattern for **optional hooks** (`play_native`, export, inline) and
  backend selection in `animation.py`.
- **`compile/evaluators/`**, **`simulation/solvers/`**: deeper stacks justified by
  correctness and orchestration; **do not** mirror that depth for plotting—keep
  interfaces **small** and serializers **thin**.
