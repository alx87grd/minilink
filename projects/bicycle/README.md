# Bicycle vehicle project

Project-specific dynamic-bicycle work isolated from the `minilink` core library.
This code was separated out of PR #36 (`dev-sim`); the reusable, generic pieces
of that PR were merged into `minilink` core instead (see below).

This folder is **not** part of the shipped `minilink` package. It depends on
`minilink` but lives outside it so experimental vehicle models, controllers, and
simulation scripts stay separate from the maintained core.

## Layout

- `models/` — bicycle dynamic models (`DynamicBicycleMagicForces`,
  `DynamicBicycleRearWheelDrive`, `DynamicBicycleRearWheelDriveEngine`), tire
  models (`TireModel`, `LinearTire`, `Pacejka`), and engine/CVT experiments.
- `control/` — vehicle-specific blocks: `BicycleMeasurement`, motor/throttle and
  steering maps, and the longitudinal `AccelerationMeasurement` estimator.
- `simulations/` — runnable simulation scripts (lateral, longitudinal,
  position-cascade, trajectory) plus `vehicule_helper.py`.
- `benchmark/` — manual simulator-speed benchmark for the bicycle models.

## Generic features promoted to `minilink` core

The following generic blocks from the original PR now live in core and are
imported from there by this project:

- `minilink.control.pid` — `PID`, `Sum`
- `minilink.control.measurement` — `Measurement`
- `minilink.control.references` — `ConstantReference`
- `minilink.graphical.signals` — signal-vs-signal plotting
  (`plot_data_signals`, `build_data_plot_spec`) and `System.plot_data`

## Running

Run scripts as modules from the repository root so the `projects` package
resolves:

```bash
python -m projects.bicycle.simulations.lateral.simul_rear_wheel_drive_steering_PID
```

Animations need a display; on headless machines set `MPLBACKEND=Agg` and rely on
the `compute_trajectory` / `plot_*` paths (the `animate()` calls require a GUI).
