# Phase E4 — Merge parametric MPC into TOP

**Status:** stub (expand before coding)  
**Depends on:** [E3](phase-E3.md) green · contracts: [vision.md](vision.md)

## Goal

One trajopt class with `compile_mode="batch"|"parametric"`; `MPCPlanner` thin
alias. Sync DESIGN §6 / ROADMAP maturity notes.

## Exit

No separate MPC NLP engine; demos import TOP or alias only.

## Gate

```bash
pytest tests/unittest/test_mpc_planner.py \
       tests/unittest/test_jax_direct_collocation.py \
       tests/unittest/test_planning_architecture.py \
       tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_hybrid_*.py
```

**Smoke:** hybrid minimal + one trajopt demo + straight-line.

## Outline (expand when starting)

| Step | Work |
| --- | --- |
| E4.1 | `compile_mode` on TOP options |
| E4.2 | Move parametric prepare/bind/`solve_trajectory_from` into TOP |
| E4.3 | Shared collocation + `transcribe_parametric` (or façade) |
| E4.4 | `MPCPlanner` → factory/alias |
| E4.5 | Update `examples/scripts/mpc/*` + hybrid MPC demos |
| E4.6 | DESIGN / ROADMAP |

Next: [phase-E5.md](phase-E5.md).
