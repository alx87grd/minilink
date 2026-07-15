# Phase E8 — Broadcast + dual-rate export (+ other backends)

**Status:** stub (contract locked; expand before coding)  
**Depends on:** earlier phases green · contracts: [vision.md](vision.md)

## Goal

High-rate nominal broadcast from the latched plan, independent of the replan
tick; dual-rate `Computer` export for sim; optional fill of reserved
`x_dot`/`u_dot`; smoke `solve_trajectory_from` on batch TOP / RRT.

## Contract (locked)

### Two independent deploy surfaces

| Method | Period | NLP? | Output |
| --- | --- | --- | --- |
| `compute_command(y, …)` | \(\Delta t_{\mathrm{mpc}}\) | yes | `Command` + latch plan |
| `get_nominal(t, *, include_derivatives=False)` | free / \(\Delta t_{\mathrm{ctl}}\) | **no** | \(u(t), x(t)\) [, rates] from **last** plan |

Absolute \(t\); \(\tau = t - t_{\mathrm{solve}}\). Real nodes / Rust: two timers.

### Dual-rate Computer export (sim)

E2 single-rate: `export_to_computer()` / `mpc @ plant` applies `u_ff` ZOH.

E8 multi-rate:

```python
computer = mpc.export_to_computer(dt_ctl=0.01)
hybrid = computer @ plant
```

| Leaf | Fire | Role |
| --- | --- | --- |
| Replan | \(\Delta t_{\mathrm{mpc}}\) | latch / `compute_command` path |
| Broadcast | \(\Delta t_{\mathrm{ctl}}\) | `u_nom`, `x_nom` [, `du_nom`, `dx_nom`] via `get_nominal(t)` |

`ModelPredictiveController(..., dt_mpc=…, dt_ctl=None)` — `dt_ctl is None`
keeps E2 single-rate; set `dt_ctl` for multi-rate schedule (two divisors).

Hold / edge policy outside \([t_{\mathrm{solve}}, t_{\mathrm{solve}}+T]\) and derivative
convention: specify when coding.

## Exit / gate

Targeted new tests; one multi-rate hybrid or hand-loop broadcast demo.
Smoke TOP / RRT `solve_trajectory_from` as listed in overview.
