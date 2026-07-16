# Phase E8 — Broadcast + dual-rate export

**Status:** done  
**Depends on:** [E7](phase-E7.md) · contracts: [vision.md](vision.md)

## Goal

High-rate nominal sampling from the latched plan (no NLP), with an explicit
slow post-process interpolator; advanced dual-rate `Computer` packaging for
sim; default `export_to_computer` / `mpc @ plant` stay `u_ff` ZOH.

## Shared API (deploy = sim)

| Method | Rate | Work |
| --- | --- | --- |
| `compute_command(y, t=…)` | `dt_mpc` | NLP + latch **only** (no interpolator) |
| `generate_nominal_interpolator(*, derivatives=True)` | once / replan | Build `NominalCache` (opt-in) |
| `get_nominal_u/x/u_dot/x_dot(t)` | free / `dt_broadcast` | Eval cache → one `ndarray` each |

### Deploy sketch (ROS-agnostic)

```python
def on_replan(y, t_wall):
    mpc.compute_command(y, t=t_wall)
    mpc.generate_nominal_interpolator(derivatives=True)

def on_broadcast(t_wall):
    u = mpc.get_nominal_u(t_wall)
```

### Sim packaging (advanced)

```python
computer = mpc.dual_rate_computer(dt_broadcast=0.01)
hybrid = computer @ plant   # u_nom; replan = mpc.dt_mpc
```

Default: `mpc @ plant` / `export_to_computer()` → single-rate `u_ff` ZOH.

### Option A — shared latch (no port edge)

`replan` and `broadcast` are **not** wired by ports. After each NLP,
`after_solve` calls `generate_nominal_interpolator`; broadcast calls
`get_nominal_*` on the same MPC. Diagram plots correctly show no signal
between the leaves. That is intentional packaging around the deploy API
(zero-lag same-tick apply), not a Computer signal-graph bug. A later
port-carried flat plan (option B) would add an honest edge and one
`dt_broadcast` lag under parallel ticks.

## Locked decisions

- Hold: clamp \(\tau\) to \([0, T]\)
- FD rates only inside `generate_nominal_interpolator` when `derivatives=True`
- `dual_rate_computer(dt_broadcast)` only; replan from `mpc.dt_mpc`
- No ctor `dt_ctl`; no auto-interpolator in `compute_command`
- Dual-rate Computer = **option A** (shared latch / side-channel), not a port edge

## Checklist

| Step | Work |
| --- | --- |
| E8.0 | Expand this card; sync vision Broadcast section |
| E8.1 | `nominal.py` + mixin getters + latch `t_solve` |
| E8.2 | Broadcast leaf + `dual_rate_computer` |
| E8.3 | DESIGN / phases / demo + tests |

## Gate

```bash
ruff check . && ruff format --check .
pytest tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_export_computer.py \
       tests/unittest/test_mpc_*.py -q
MPLBACKEND=Agg python examples/scripts/mpc/demo_mpc_minimal.py
MPLBACKEND=Agg python examples/scripts/mpc/demo_mpc_dual_rate.py
```

## Exit

Four getters + explicit interpolator; dual-rate sim packaging; deploy-shaped test.

Next: [phase-F-cleanup.md](phase-F-cleanup.md), then planning UI.
(Pipeline B is a later feature, not in this closing sequence.)
