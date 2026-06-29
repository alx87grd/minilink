# Phase 7 — Collision Reuse

Master plan: [`../01_master_overview.md`](../01_master_overview.md). Core math:
[`../03_kinematics_core_math.md`](../03_kinematics_core_math.md).

Converge `RobotBody` / `PlanarRigidBody.body_poses`
([`planning/spatial/robot.py`](../../minilink/planning/spatial/robot.py)) onto the
**shared `tf` world dict** (vision doc § *Collision reuse*).

## Goal

One forward-kinematics pass feeds both the rendered chassis and the clearance
probes: collision `Shape`s key to the **same** frame names that `tf` returns, both
consuming world poses through `core/kinematics.py` (`apply_transform` relocated
here and renamed `apply`; call sites updated). No parent/child tree needed.

## Automated gate

Spatial tests green; collision results unchanged vs current `body_poses`.

## User Review 7

Optional.
