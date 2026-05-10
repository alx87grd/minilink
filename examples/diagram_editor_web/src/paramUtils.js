/**
 * MVP block params: defaults and round-trip with the JSON ``spec`` shape
 * expected by ``minilink.core.diagram_topology`` factories.
 */

export function mergeDefaultParams(kind, params) {
  const p = params && typeof params === "object" ? { ...params } : {};
  if (kind === "integrator") {
    return {
      k: p.k != null ? Number(p.k) : 1,
      x0: Array.isArray(p.x0) && p.x0.length ? [Number(p.x0[0])] : [0],
      state_label: p.state_label != null ? String(p.state_label) : "x",
    };
  }
  if (kind === "prop_controller") {
    return {
      Kp: p.Kp != null ? Number(p.Kp) : 10,
    };
  }
  if (kind === "step") {
    const iv = Array.isArray(p.initial_value) ? p.initial_value : [0];
    const fv = Array.isArray(p.final_value) ? p.final_value : [0];
    return {
      initial_value: [Number(iv[0] ?? 0)],
      final_value: [Number(fv[0] ?? 0)],
      step_time: p.step_time != null ? Number(p.step_time) : 1,
    };
  }
  return { ...p };
}

/** Flat object for controlled form fields */
export function draftFromNode(node) {
  if (!node) return {};
  const p = mergeDefaultParams(node.kind, node.params);
  if (node.kind === "integrator") {
    return {
      k: String(p.k),
      x0: String(p.x0[0]),
      state_label: String(p.state_label),
    };
  }
  if (node.kind === "prop_controller") {
    return { Kp: String(p.Kp) };
  }
  if (node.kind === "step") {
    return {
      initial_value: String(p.initial_value[0]),
      final_value: String(p.final_value[0]),
      step_time: String(p.step_time),
    };
  }
  return {};
}

export function paramsFromDraft(kind, draft) {
  if (kind === "integrator") {
    return {
      k: Number(draft.k),
      x0: [Number(draft.x0)],
      state_label: String(draft.state_label || "x"),
    };
  }
  if (kind === "prop_controller") {
    return { Kp: Number(draft.Kp) };
  }
  if (kind === "step") {
    return {
      initial_value: [Number(draft.initial_value)],
      final_value: [Number(draft.final_value)],
      step_time: Number(draft.step_time),
    };
  }
  return {};
}
