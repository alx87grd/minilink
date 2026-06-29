# Phase 0 — PNG Baseline Harness

Master plan: [`../01_master_overview.md`](../01_master_overview.md).

**No API changes.** Revert unrelated WIP in `demo_holonomic_corridor.py` before
starting.

## Deliverables

- [`scripts/generate_kinematic_baselines.py`](../../scripts/generate_kinematic_baselines.py)
- [`tests/fixtures/kinematic_baseline/`](../../tests/fixtures/kinematic_baseline/) — ~36 PNGs + `manifest.json`
- [`tests/unittest/test_kinematic_regression.py`](../../tests/unittest/test_kinematic_regression.py)

## Scope

**12 plants × 3 samples** (`x0`, mid-trajectory, interesting state), rendered with
matplotlib **Agg** at a **fixed DPI** for deterministic pixels:

`DynamicBicycle`, `DynamicBicycleCar3D`, `KinematicBicycle`,
`HolonomicMobileRobot`, `Pendulum`, `CartPole`, `TwoLinkManipulator`,
`FiveLinkPlanarManipulator`, `Drone2D`, `SimpleIntegrator`, `SingleMass`,
`JaxDynamicBicycleRateInputs`

> **Naming note:** there is no NumPy `DynamicBicycleRateInputs` — the rate-input
> plant the MPC demos use is `JaxDynamicBicycleRateInputs` (JAX-only). The harness
> renders it when JAX is installed and **skips it gracefully** otherwise (the
> regression test skips that entry too). The same rename applies to the Phase 3a
> batch and demo use case 7.

The three samples per plant are deterministic states — `x0` plus two fixed
perturbations (`rest`, `pose_a`, `pose_b`) — not simulator output: rendering reads
only `get_kinematic_*` / `get_camera_transform` (pure trig, never `f`), so
perturbing `x0` poses joints/headings/steer without any simulator coupling or
instability.

The harness saves the reference PNGs once (committed), then `test_kinematic_regression`
re-renders and compares pixel-for-pixel (decoded image arrays) against the
committed baselines, using the exact `(x, u, t)` recorded in `manifest.json`.

## Automated gate

```bash
python scripts/generate_kinematic_baselines.py
pytest tests/unittest/test_kinematic_regression.py
```

## Status — DONE (awaiting User Review 0)

Implemented on branch `refactor-v4`:

- `scripts/generate_kinematic_baselines.py` — Agg, fixed DPI (figsize 6×4.5 ×
  100 DPI = 600×450 px), plant registry + deterministic `sample_states`.
- `tests/fixtures/kinematic_baseline/` — **36 PNGs** + `manifest.json`
  (records module/class/`x`/`u`/`t`/sha256 per sample).
- `tests/unittest/test_kinematic_regression.py` — re-renders each manifest state
  and asserts pixel-identical; ruff clean; passes in `dev-h26`.

## User Review 0 — PASSED

Baselines spot-checked and approved as the source of truth.

> **Known-imperfect baselines (approved):** a few plants bake in **suboptimal
> default camera scales** (geometry too small / off-center). This is a
> pre-existing rendering-default issue, captured *as-is* so the v2 pipeline can be
> proven byte-for-byte equivalent first. A later phase may deliberately fix these
> default scales — that is an **allowed visual change**: regenerate the affected
> baselines and **validate visually** (user sign-off) instead of asserting pixel
> parity against the old, wrong-scale PNGs. Pixel parity gates only guarantee
> "v2 == old"; it is expected (and fine) for an intentional camera-default fix to
> break parity on exactly those plants.
