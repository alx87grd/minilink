---
name: kinematic-contract-implementation
overview: "Master plan for the kinematic contract upgrade. Ground truth: 00_original_vision.md. Parallel _v2 pipeline, PNG pixel-parity gates, user review checkpoints. Demo ergonomics + per-phase plans live in sibling files."
todos:
  - id: p0-baseline
    content: "Phase 0: PNG baseline script (12 plants x 3 samples = 36 PNGs) + pytest harness on old pipeline"
    status: completed
  - id: p1-foundation
    content: "Phase 1: Additive foundation - core/kinematics, Arrow/TorqueArrow primitives, visualization, camera, public graphical/catalog (shapes + skins), local_transform"
    status: pending
  - id: p2-v2-pipeline
    content: "Phase 2: Parallel v2 pipeline - _v2 hooks (defaults {}), Animator2, animate_v2; old pipeline untouched"
    status: pending
  - id: p3a-vehicles
    content: "Phase 3a: Vehicle catalog _v2 hooks + pixel parity gate + user review"
    status: pending
  - id: p3b-pendulum
    content: "Phase 3b: Pendulum family _v2 + pixel parity gate + user review"
    status: pending
  - id: p3c-manipulators
    content: "Phase 3c: Manipulators _v2 + pixel parity gate + user review"
    status: pending
  - id: p3d-rest
    content: "Phase 3d: Remaining catalog + diagram _v2 + pixel parity gate + user review"
    status: pending
  - id: p4-demos
    content: "Phase 4: Demo subclass _v2 hooks + MPC/trajopt parity + user review"
    status: pending
  - id: p5-cutover
    content: "Phase 5: Cutover - delete old, rename _v2, base {} default, retire DynamicBicycleCar3D (skin=car_skin_3d), hacks, docs"
    status: pending
  - id: p6-overlays
    content: "Phase 6: Scene/SceneHistory/Replay + animate(overlays) + MPC demo cleanup (user architectural review)"
    status: pending
  - id: p7-collision
    content: "Phase 7: Collision RobotBody convergence onto shared tf dict"
    status: pending
isProject: false
---

# Kinematic Contract — Master Plan Overview

## Ground truth

The **vision, architecture, and locked decisions** live in
[`00_original_vision.md`](00_original_vision.md).

This document is the **implementation playbook**: how we get there safely, what
each phase delivers, and the gates between them. Companion files:

- [`02_demo_use_cases.md`](02_demo_use_cases.md) — destination user API.
- [`03_kinematics_core_math.md`](03_kinematics_core_math.md) — reusable core math.
- [`phases/`](phases/) — one detailed plan per phase.

Do not duplicate the full design rationale here — refer to the vision doc for
frame vocabulary, module placement, camera layers, and collision notes.

---

## Goal (one paragraph)

Replace index-aligned `get_kinematic_geometry` / `get_kinematic_transforms` lists
with **string-keyed frames** (`tf`) and **frame-keyed geometry dicts**, revive
`get_dynamic_geometry`, retire `T[3,3]` side-channels and column-norm arrow hacks,
and compose plants with scenes, MPC histories, and replay ghosts at
`animate(overlays=[...])` — with **pixel-identical** matplotlib output to today's
rendering.

---

## Execution principles

1. **Parallel v2 pipeline** — build new alongside old; compare `animate()` vs
   `animate_v2()` until cutover.
2. **Old defaults frozen until Phase 5** — base `System` debug-point skin stays on
   old hooks through Phases 0–4; only `_v2` hooks use `{}`.
3. **PNG pixel parity** — primary sign-off measure (matplotlib Agg, fixed DPI).
4. **Validation gate + user review** after every phase; no phase N+1 until both
   pass.
5. **Temporary `_v2` suffix** — dev-branch only; grep-clean at Phase 5 cutover.

```mermaid
flowchart TD
  Vision["Vision doc — ground truth"] --> P0
  P0["Phase 0: PNG baselines"] --> R0["User Review 0"]
  R0 --> P1["Phase 1: Foundation"]
  P1 --> R1["User Review 1"]
  R1 --> P2["Phase 2: v2 pipeline"]
  P2 --> R2["User Review 2"]
  R2 --> P3["Phase 3: Catalog _v2 batches"]
  P3 --> R3["User Review 3"]
  R3 --> P4["Phase 4: Demo _v2"]
  P4 --> R4["User Review 4"]
  R4 --> P5["Phase 5: Cutover"]
  P5 --> R5["User Review 5 — merge"]
  R5 --> P6["Phase 6: Overlays"]
  P6 --> R6["User Review 6"]
  R6 --> P7["Phase 7: Collision"]
```

---

## Success criteria

| When | Criterion |
| --- | --- |
| Phases 0–4 | Old pipeline unchanged; all pytest green; PNG baselines match |
| Phases 3–4 | `render_v2()` pixel-identical to `render()` per migrated plant |
| Phase 5 | Zero `_v2`; final API; base `{}` default; hacks deleted; docs synced |
| Phase 6 | MPC demos use overlays; same pixels, less boilerplate |
| Phase 7 | `RobotBody` shares `tf` frame dict with renderer |

---

## Phase map

| Phase | Scope | Automated gate | User review | Detail |
| --- | --- | --- | --- | --- |
| **0** | PNG baseline harness | Baseline pytest green | Spot-check 3–4 PNGs | [phase0](phases/phase0_baselines.md) |
| **1** | Additive foundation | Full pytest; baselines unchanged | Skim module APIs | [phase1](phases/phase1_foundation.md) |
| **2** | `Animator2` + `_v2` hooks | Old baselines + empty v2 smoke | `animate()` still identical | [phase2](phases/phase2_v2_pipeline.md) |
| **3a–e** | Catalog `_v2` (5 batches) | Per-batch pixel parity | Side-by-side old vs v2 | [phase3](phases/phase3_catalog_migration.md) |
| **4** | Demo `_v2` (subclasses) | Demo PNG parity | 1 MPC + 1 trajopt demo | [phase4](phases/phase4_demos.md) |
| **5** | Cutover + docs | Full pytest; baselines match | Full demo sweep | [phase5](phases/phase5_cutover.md) |
| **6** | Scene / SceneHistory / Replay | Overlay parity | Architectural review | [phase6](phases/phase6_overlays.md) |
| **7** | Collision on `tf` | Spatial tests | Optional | [phase7](phases/phase7_collision.md) |

---

## Locked decisions (from vision doc)

| ID | Decision | When applied |
| --- | --- | --- |
| D1 | `tf` native-array, JAX-traceable | `_v2` from Phase 3; final at Phase 5 |
| D2 | Default viz `{}` | **Phase 5 cutover only** on final hooks |
| D3a | `animate(overlays=[...])` | Phase 6 |
| D3b | Camera hints + resolver + callable override | Animator2 Phase 2; final Phase 5 |
| D4 | No permanent shims | `_v2` dev suffix until Phase 5 |

**Skin swap (LOCKED — Option B):** the animator only ever calls
`get_kinematic_geometry() -> dict`. Skins are **pure functions** `(plant) -> dict`
in `graphical.catalog`. The base method delegates to an opt-in `skin` attribute;
swapping a look is one assignment (`car.skin = car_skin_3d`). `skin :
get_kinematic_geometry :: params : f`. Reject `sys.get_kinematic_geometry = fn`
(instance-assigned functions get no `self`). `DynamicBicycleCar3D` is retired.

---

## Risk mitigations

| Risk | Mitigation |
| --- | --- |
| Base-class default breaks baselines | `{}` only at Phase 5 on final hooks |
| Silent visual drift | ~36 PNG baselines + old-vs-v2 per batch |
| Monolithic breakage | Parallel pipeline through Phase 4 |
| Demo boilerplate lingers | Phase 6 explicitly replaces subclasses; Phase 4 parity first |
| Review fatigue | One user review per phase/batch, not per file |

> **Intentional visual changes are exempt from pixel parity.** The Phase 0
> baselines bake in some pre-existing rendering defaults that are *known wrong*
> (notably suboptimal default camera scales on a few plants). Pixel parity proves
> only "v2 == old"; if a later phase deliberately fixes such a default, that is an
> allowed visual change — regenerate the affected baselines and **validate
> visually** (user sign-off) rather than gating on parity against the old PNGs.

---

## References

- **Vision:** [`00_original_vision.md`](00_original_vision.md)
- **Current contract:** [`minilink/core/system.py`](../minilink/core/system.py),
  [`minilink/graphical/animation/animator.py`](../minilink/graphical/animation/animator.py)

---

## Current status

**Phase 0 complete** (branch `refactor-v4`): 36 committed PNG baselines +
`manifest.json` + `test_kinematic_regression` (passing, ruff clean) — awaiting
**User Review 0** before Phase 1. Phases 1–7 pending.
