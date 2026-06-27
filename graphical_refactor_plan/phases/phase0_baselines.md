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
`DynamicBicycleRateInputs`

The harness saves the reference PNGs once (committed), then `test_kinematic_regression`
re-renders and compares pixel-for-pixel against the committed baselines.

## Automated gate

```bash
python scripts/generate_kinematic_baselines.py
pytest tests/unittest/test_kinematic_regression.py
```

## User Review 0

Confirm the baseline PNGs look correct (spot-check 3–4). These images are the
**source of truth** for every later pixel-parity comparison.
