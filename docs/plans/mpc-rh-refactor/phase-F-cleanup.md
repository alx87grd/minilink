# Phase F — Post-refactor cleanup

**Status:** stub — **ready to expand** ([E8](phase-E8.md) green)  
**Depends on:** E0–E8 complete  
**Next after F:** [planning-ui-simplification.md](../planning-ui-simplification.md)
(constructor ceremony). Pipeline B is out of this closing sequence.

## Goal

One hygiene pass after the MPC/RH refactor is **functionally done**. Prune merge
debt, sync docs with the landed product API, reduce demo duplication, and
**land the package layout below** — **without** changing contracts in
[vision.md](vision.md).

**Constructor / planner API UX** is **not** Phase F — see
[planning UI simplification](../planning-ui-simplification.md) (UI-1+).

---

## Parked layout (expand into F.1…)

Captured from product review (July 2026). Turn into numbered tasks when
expanding this card; include E8 files (`nominal.py`, `broadcast_block.py`).

### Package home

Move the product MPC System family from `planning/mpc/` → **`control/mpc/`**.

- DESIGN §3: diagram **control laws** live in `control/`; `planning/` owns
  trajopt/search tools (`TrajectoryOptimizationPlanner`, parametric NLP stay
  under `planning/trajectory_optimization/`).
- Document a narrow DESIGN exception: `control/mpc` may import
  `planning.trajectory_optimization` (RH controller wraps a traj-family planner).
- Pre-1.0 hard cut: no lasting `planning.mpc` re-export; retarget all call sites
  in the same change.

### Target file structure (fewer files)

```text
minilink/control/mpc/
  __init__.py       # public API only
  controller.py     # fused product module (see below)
  warm_start.py     # guess / computer x0 helpers
  viz.py            # mpc_plans_from_rollout + mpc_animation_overlays
```

**Fuse into `controller.py`** (today’s scatter under `planning/mpc/`):

- `model_predictive_controller.py` — factory, mixin, `compute_command`, `@`, debug
- `controller.py` + `step_block.py` — algebraic / warm-start System backends
- `tick_latch.py`, `command.py`, `_block_common.py` (incl. dual-rate export)
- `broadcast_block.py` — dual-rate broadcast leaf (option A shared latch)

**Keep separate:** `warm_start.py` (demos + latch), `viz.py` (graphical /
hybrid), `nominal.py` (cache / interp helpers — or fold if tiny).

**Delete:** `planning/mpc/parametric_*.py` shims; then remove `planning/mpc/` entirely.

### Public import after F

```python
from minilink.control.mpc import ModelPredictiveController, Command, ...
```

### Also in F (hygiene categories)

| Category | Note |
| --- | --- |
| Test filenames | Rename `test_mpc_stateful/stateless_*.py` when retargeting imports |
| Docs | DESIGN package map, README import paths, mark old brainstorm docs historical |
| Demos | Dedup only when touching a file; preserve user-tuned constants (AGENTS.md) |

---

## When to expand this card

1. Re-scan demos, tests, and plan docs against [vision.md](vision.md) (E8 done).
2. Turn **Parked layout** into numbered tasks (F.1 package move, F.2 fuse,
   F.3 retarget/delete, F.4 docs, F.5 gate) in a short expansion PR.

Until expanded, treat the parked layout as **intent**, not an active work order.

---

## Split of responsibility

| Phase F (hygiene + layout) | [planning-ui-simplification.md](../planning-ui-simplification.md) |
| --- | --- |
| Move MPC → `control/mpc`, fuse files, delete shims | Flat planner constructor kwargs |
| Doc and export alignment | Simulator-style defaults, tier-2 advanced path |
| Optional demo dedup helpers | README / DESIGN constructor examples |
| Test naming tidy-up | RRT / DP family alignment (UI-5 / UI-6) |

---

## Sequencing

```mermaid
flowchart LR
  E8[E8 done]
  Fexpand[Expand phase-F card]
  Fwork[Phase F control/mpc]
  UI[Planning UI ceremony]

  E8 --> Fexpand --> Fwork --> UI
```

**Order locked:** F before UI so teaching demos / README land on
`minilink.control.mpc` once. Pipeline B is not in this graph.

---

## Gate (phase exit)

```bash
ruff check . && ruff format --check .
pytest tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_*.py tests/unittest/test_planning_architecture.py -q

export MPLBACKEND=Agg SDL_VIDEODRIVER=dummy
python examples/scripts/hybrid/demo_mpc_hybrid_minimal.py
python examples/scripts/hybrid/demo_mpc_hybrid_dual_rate.py
python examples/scripts/mpc/demo_dynamic_bicycle_rate_mpc_straight_line.py
```

---

## Non-goals

- Vision or planner ABC contract changes
- Pipeline B / online scene bind (later feature)
- Physics-hiding presets or merging problem → planner → controller stages

---

## Exit

F expanded and executed (including `control/mpc` layout); docs and exports
match product API. Then schedule [planning-ui-simplification.md](../planning-ui-simplification.md).
