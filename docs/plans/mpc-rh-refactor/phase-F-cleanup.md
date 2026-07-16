# Phase F — `control/mpc` move + hygiene

**Status:** done  
**Depends on:** E0–E8 complete  
**Next after F:** [planning-ui-simplification.md](../planning-ui-simplification.md)

## Goal

Move the product MPC System family from `planning/mpc/` → `control/mpc/`,
fuse internals, hard-cut old imports, and gate with before/after
`ModelPredictiveController` smoke (trajectory + timing). No vision contract
changes. Pipeline B and planning-UI ceremony are out of scope.

## Layout (locked)

```text
minilink/control/mpc/
  __init__.py
  controller.py      # fused runtime product
  utilities.py       # warm-start + nominal (pure functions)
  viz.py             # plans_from_rollout + animation_overlays
```

**Fuse into `controller.py`:** factory/mixin, algebraic + warm-start Systems,
`Command`, latch, export helpers, broadcast leaf.

**`utilities.py` public:** `mpc_warm_start_guess`, `mpc_default_computer_x0`;
nominal build/eval as needed. Private: `_shift_plan_trajectory`, `_clamp_tau`.
Demote `warm_start_guess_from_prev_plan` from public `__all__`.

**Delete:** entire `planning/mpc/` (incl. parametric shims) — hard cut.

**DESIGN exception:** `control/mpc` may import `planning.trajectory_optimization`.

## Checklist

| Step | Work |
| --- | --- |
| F.0 | Expand this card |
| F.5a | Capture `f_mpc_parity` baseline (pre-move) |
| F.1 | Create `control/mpc` (`controller` / `utilities` / `viz`) |
| F.2 | Retarget imports; rename stateful/stateless tests |
| F.3 | Delete `planning/mpc` |
| F.4 | DESIGN / vision / ROADMAP / README / benchmarks README |
| F.5b | Compare parity + full smoke |

## Gate

```bash
# Pre-move (once):
MPLBACKEND=Agg python benchmarks/run_f_mpc_parity.py --capture

# Post-move:
MPLBACKEND=Agg python benchmarks/run_f_mpc_parity.py
ruff check . && ruff format --check .
pytest tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_*.py \
       tests/unittest/test_planning_architecture.py -q
MPLBACKEND=Agg python examples/scripts/hybrid/demo_mpc_hybrid_minimal.py
MPLBACKEND=Agg python examples/scripts/hybrid/demo_mpc_hybrid_dual_rate.py
MPLBACKEND=Agg python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py
```

Compare: trajectory ~1e-4; timing factor 2.0.

## Non-goals

- Pipeline B / online scene bind
- Planning UI constructor ceremony
- Vision / planner ABC changes

## Exit

`control/mpc` landed; `planning/mpc` gone; parity baseline compares green;
docs match. Next: UI plan.
