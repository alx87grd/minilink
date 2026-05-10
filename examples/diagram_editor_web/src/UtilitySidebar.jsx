import { useEffect, useState } from "react";
import { draftFromNode, paramsFromDraft } from "./paramUtils.js";

export function UtilitySidebar({
  spec,
  catalog,
  selectedId,
  onClearSelection,
  onApplyParams,
  onRenameDiagram,
  onDeleteBlock,
  onFitView,
}) {
  const node = spec?.nodes?.find((n) => n.id === selectedId) ?? null;
  const [diagramName, setDiagramName] = useState(spec?.name ?? "");
  const [draft, setDraft] = useState({});

  useEffect(() => {
    setDiagramName(spec?.name ?? "");
  }, [spec?.name]);

  useEffect(() => {
    setDraft(draftFromNode(node));
  }, [node]);

  const portInfo = node && catalog ? catalog[node.kind] : null;

  return (
    <aside className="utility-sidebar">
      <div className="sidebar-section">
        <h3 className="sidebar-heading">Diagram</h3>
        <label className="sidebar-label" htmlFor="diagram-name">
          Name
        </label>
        <input
          id="diagram-name"
          className="sidebar-input"
          value={diagramName}
          onChange={(e) => setDiagramName(e.target.value)}
        />
        <button
          type="button"
          className="sidebar-btn"
          onClick={() => onRenameDiagram(diagramName.trim() || "Diagram")}
        >
          Apply diagram name
        </button>
      </div>

      <div className="sidebar-section">
        <h3 className="sidebar-heading">Canvas</h3>
        <button type="button" className="sidebar-btn" onClick={onFitView}>
          Fit view
        </button>
      </div>

      <div className="sidebar-section">
        <h3 className="sidebar-heading">Selection</h3>
        {!node && (
          <p className="sidebar-muted">Click a block to edit parameters or delete it.</p>
        )}
        {node && (
          <>
            <p className="sidebar-meta">
              <strong>{node.id}</strong>
              <br />
              <span className="sidebar-muted">{node.kind}</span>
            </p>
            {portInfo && (
              <p className="sidebar-muted sidebar-ports">
                In: {portInfo.inputs.join(", ") || "—"}
                <br />
                Out: {portInfo.outputs.join(", ") || "—"}
              </p>
            )}

            {(node.kind === "integrator" ||
              node.kind === "prop_controller" ||
              node.kind === "step") && (
              <div className="sidebar-form">
                {node.kind === "integrator" && (
                  <>
                    <label className="sidebar-label">k</label>
                    <input
                      className="sidebar-input"
                      value={draft.k ?? ""}
                      onChange={(e) => setDraft((d) => ({ ...d, k: e.target.value }))}
                    />
                    <label className="sidebar-label">x₀ (first state)</label>
                    <input
                      className="sidebar-input"
                      value={draft.x0 ?? ""}
                      onChange={(e) => setDraft((d) => ({ ...d, x0: e.target.value }))}
                    />
                    <label className="sidebar-label">State label</label>
                    <input
                      className="sidebar-input"
                      value={draft.state_label ?? ""}
                      onChange={(e) =>
                        setDraft((d) => ({ ...d, state_label: e.target.value }))
                      }
                    />
                  </>
                )}
                {node.kind === "prop_controller" && (
                  <>
                    <label className="sidebar-label">Kp</label>
                    <input
                      className="sidebar-input"
                      value={draft.Kp ?? ""}
                      onChange={(e) => setDraft((d) => ({ ...d, Kp: e.target.value }))}
                    />
                  </>
                )}
                {node.kind === "step" && (
                  <>
                    <label className="sidebar-label">initial_value[0]</label>
                    <input
                      className="sidebar-input"
                      value={draft.initial_value ?? ""}
                      onChange={(e) =>
                        setDraft((d) => ({ ...d, initial_value: e.target.value }))
                      }
                    />
                    <label className="sidebar-label">final_value[0]</label>
                    <input
                      className="sidebar-input"
                      value={draft.final_value ?? ""}
                      onChange={(e) =>
                        setDraft((d) => ({ ...d, final_value: e.target.value }))
                      }
                    />
                    <label className="sidebar-label">step_time</label>
                    <input
                      className="sidebar-input"
                      value={draft.step_time ?? ""}
                      onChange={(e) =>
                        setDraft((d) => ({ ...d, step_time: e.target.value }))
                      }
                    />
                  </>
                )}
                <button
                  type="button"
                  className="sidebar-btn sidebar-btn-primary"
                  onClick={() => {
                    const nextParams = paramsFromDraft(node.kind, draft);
                    for (const v of Object.values(nextParams)) {
                      if (typeof v === "number" && !Number.isFinite(v)) {
                        window.alert("All numeric parameters must be finite numbers.");
                        return;
                      }
                      if (Array.isArray(v) && v.some((x) => !Number.isFinite(Number(x)))) {
                        window.alert("All numeric parameters must be finite numbers.");
                        return;
                      }
                    }
                    onApplyParams(node.id, nextParams);
                  }}
                >
                  Apply block parameters
                </button>
              </div>
            )}
            {node.kind !== "integrator" &&
              node.kind !== "prop_controller" &&
              node.kind !== "step" && (
                <p className="sidebar-muted">
                  No parameter form for this kind; edit JSON or extend the editor.
                </p>
              )}

            <button
              type="button"
              className="sidebar-btn sidebar-btn-danger"
              onClick={() => onDeleteBlock(node.id)}
            >
              Delete block…
            </button>
            <button type="button" className="sidebar-btn" onClick={onClearSelection}>
              Clear selection
            </button>
          </>
        )}
      </div>
    </aside>
  );
}
