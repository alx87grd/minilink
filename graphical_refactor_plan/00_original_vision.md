# Original Vision (read-only)

The ground-truth vision document is the **canonical read-only file at the repo
root** — do not duplicate or edit it for implementation tracking:

[`../kinematic-contract-upgrade_394627b5.plan.md`](../kinematic-contract-upgrade_394627b5.plan.md)

It defines the locked architecture: the three upgraded hooks (`tf`,
`get_kinematic_geometry`, `get_dynamic_geometry`), string-keyed world frames,
`primitive.local_transform`, the skin tiers + Option-B swap mechanism, the dynamic
layer, drawables (`System` / `Scene` / `SceneHistory`), module placement
(`core/kinematics.py`), the camera system (D3b), and the locked decisions D1–D4.

Refer to it for the *why*; this folder holds the *how*:

- [`01_master_overview.md`](01_master_overview.md) — phases, gates, success criteria.
- [`02_demo_use_cases.md`](02_demo_use_cases.md) — destination user API.
- [`03_kinematics_core_math.md`](03_kinematics_core_math.md) — reusable core math.
- [`phases/`](phases/) — one detailed plan per phase.
