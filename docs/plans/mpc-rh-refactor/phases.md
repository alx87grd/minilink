# Phases — E0–E8 overview

Contracts live in [vision.md](vision.md). This file owns **order, gates, and
PR slicing**. Expand the active phase card before coding.

Do not start the next phase until that phase’s gate is green unless noted.

Branch: **`dev-mpc-v2`** (hybrid + MPC controllers present; same full stack as
`dev-alex` when the vision locked).

## Baseline inventory (must keep working)

**Hybrid MPC demos** — primary closed-loop regression:

| Demo | Role |
| --- | --- |
| `examples/scripts/hybrid/demo_mpc_hybrid_minimal.py` | Flagship → target `rhc @ plant` |
| `examples/scripts/hybrid/demo_mpc_hybrid_track_lap.py` | Richer hybrid + scene |

**MPC hand-loop demos** — keep green:

| Demo | Role |
| --- | --- |
| `demo_dynamic_bicycle_rate_mpc_straight_line.py` | Minimal compile-once + hand loop |
| `demo_dynamic_bicycle_rate_mpc_closed_loop_lap.py` | Full closed-loop lap |
| Obstacle / stadium / wide variants | Smoke optional per phase |
| `demo_mpc_spatial_scene_guide.py` | Teaching / scene API |
| `demo_dynamic_bicycle_rate_mpc_straight_line_trajopt.py` | Per-tick trajopt reference |

**Tests** — gate whenever touching `Planner` / MPC / hybrid:

| Area | Tests |
| --- | --- |
| MPC NLP | `tests/unittest/test_mpc_planner.py` |
| MPC blocks / hybrid | `test_mpc_stateless_controller.py`, `test_mpc_stateful_controller.py`, `test_mpc_export_computer.py`, `test_mpc_hybrid_*.py` |
| JAX collocation | `tests/unittest/test_jax_direct_collocation.py` |
| Planning contracts | `tests/unittest/test_planning_architecture.py` |
| RRT / DP | `test_rrt.py`, `test_dynamic_programming.py` |

**Always before push:** `ruff check .` && `ruff format --check .`, plus the
phase’s pytest list.

## Dependency graph

```mermaid
flowchart LR
  E0[E0 Foundation]
  E1[E1 Online surface]
  E2[E2 RH facade]
  E3[E3 Demo migration]
  E4[E4 TOP merge]
  E6[E6 Polish]
  E7[E7 Scene params]
  E8[E8 Broadcast backends]

  E0 --> E1 --> E2 --> E3 --> E4 --> E6 --> E7 --> E8
```

E2 includes `compute_command` **and** `export_to_computer` / `__matmul__`;
E3 migrates hybrid minimal to `rhc @ plant`. E5 (PoC leaf retirement) sits
after demos move — see [phase-E5.md](phase-E5.md).

Default conflict rule: **hard error** if both `problem.horizon` and
`options.tf` are set and disagree.

## Phase index

| Phase | Goal | Card |
| --- | --- | --- |
| **E0** | Types, `horizon`, `solve` rename — no demo behavior change | [phase-E0.md](phase-E0.md) **(full)** |
| **E1** | `MPCPlanner.solve_trajectory_from` → `TrajectoryPlan` | [phase-E1.md](phase-E1.md) |
| **E2** | `RecedingHorizonController` tick + export / `@` | [phase-E2.md](phase-E2.md) |
| **E3** | Migrate flagship demos to RH | [phase-E3.md](phase-E3.md) |
| **E4** | Merge parametric MPC into TOP | [phase-E4.md](phase-E4.md) |
| **E5** | Retire PoC controller leaves | [phase-E5.md](phase-E5.md) |
| **E6** | Observability polish | [phase-E6.md](phase-E6.md) |
| **E7** | Parametric scene (pipeline B) | [phase-E7.md](phase-E7.md) |
| **E8** | Broadcast + other backends | [phase-E8.md](phase-E8.md) |

## Suggested PR slicing

| PR | Contents | Gate focus |
| --- | --- | --- |
| PR-A | E0 | planning + MPC + hybrid tests |
| PR-B | E1 + E2 | MPC + RH + export tests |
| PR-C | E3 | hybrid parity + smoke demos |
| PR-D | E4 | MPC + trajopt + hybrid + RH |
| PR-E | E5 | after all demos on RH |
| PR-F+ | E6–E8 | phase gates |

## Start here

1. **E0.1–E0.3** — `TrajectoryPlan` / `SolveMetadata` + `PlanningProblem.horizon` + resolve helper ([phase-E0.md](phase-E0.md)).
2. **E0.5** — `compute_solution` → `solve` rename (mechanical blast early).
3. **E1** — `solve_trajectory_from` on `MPCPlanner`.
4. **E2** — `RecedingHorizonController` with `compute_command` + `__matmul__`.
5. **E3.1** — migrate `demo_mpc_hybrid_minimal.py` to `rhc @ plant`.

Defer E4 until E2–E3 green. Retire PoC leaves (E5) after demos moved.
