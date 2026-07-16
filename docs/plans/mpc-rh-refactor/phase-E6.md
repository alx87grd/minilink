# Phase E6 — Observability polish

**Status:** done  
**Depends on:** [E5](phase-E5.md) · contracts: [vision.md](vision.md) · R9 background:
[../mpc-controller-architecture.md](../mpc-controller-architecture.md)

## Goal

ROS-agnostic telemetry polish on the existing E2
`ModelPredictiveController` product loop. Not a new control mode: deploy
(`compute_command`) stays E2; high-rate `get_nominal` / dual-rate stay E8;
package move to `control/mpc` stays [F](phase-F-cleanup.md).

## Contract

```python
cmd = mpc.compute_command(y)           # E2 deploy tick
meta = mpc.get_solve_metadata()        # last SolveMetadata (or None)
meta = cmd.metadata                    # alias → cmd.plan.metadata
mpc.reset()                            # clear deploy k, last_command, latch
```

Free tools unchanged: `mpc_animation_overlays`, `mpc_plans_from_rollout`.

### Deploy-node wrap (no ROS2 package)

External nodes / scripts clock the replan period and call Minilink only:

```python
cmd = mpc.compute_command(y, t=t_wall)
publish_u(cmd.u_ff)
publish_flat(cmd.plan_flat)
log(mpc.get_solve_metadata())
```

Library does not own middleware logging or ROS packages.

## Checklist

| Step | Work |
| --- | --- |
| E6.0 | Expand this card; sync phases / folder README |
| E6.1 | `get_solve_metadata()` on mixin; `Command.metadata` property |
| E6.2 | Facade `reset()` (deploy counter + latch + `last_command`) |
| E6.3 | DESIGN / README: deploy-node wrap + telemetry hooks |
| E6.4 | Tests + ruff; mark done |

## Explicitly defer

| Item | Where |
| --- | --- |
| `get_nominal` / `include_derivatives` | [E8](phase-E8.md) |
| `export_to_computer(dt_ctl=…)` multi-rate | E8 |
| Live `update_animation(handles)` second system | skip (post-hoc overlays cover demos) |
| Heavy static `plot_*` library | out of scope |
| Package move to `control/mpc` | [F](phase-F-cleanup.md) after E8 |
| Scene parametric `params` | [E7](phase-E7.md) |

## Constraints

- No API break to overlays / plan-history tools.
- No ROS2 package; no inventing deploy beyond documenting the wrap.
- Thin mixin methods so F fuse stays mechanical.

## Gate

```bash
ruff check . && ruff format --check .
pytest tests/unittest/test_model_predictive_controller.py \
       tests/unittest/test_mpc_*.py -q
```

## Exit

Named telemetry + reset on the product controller; deploy-node wrap documented.

Next: [phase-E7.md](phase-E7.md).
