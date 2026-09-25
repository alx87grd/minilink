"""Online estimators (placeholder — content planned, home decided).

Blocks that infer hidden quantities *online* from measured signals, running
inside a diagram in time — hidden **states** (Luenberger, Kalman, EKF) or
hidden **parameters** (recursive least squares, adaptive laws). Port shape:
``(u, y) -> estimate``.

Planned modules (ROADMAP.md §5, step P4; the factory names are settled in
docs/plans/gro501-classical-control.md, P4):

- ``luenberger.py`` — pole-placement observers (+ design factory)
- ``kalman.py`` — steady-state Kalman filter (+ design factory)
- ``ekf.py`` — extended Kalman filter (uses ``analysis/`` linearization)
- ``recursive.py`` — online parameter estimators (RLS, gradient laws)

Placement rule: online, in-the-loop inference lives here; *offline* fitting
over logged data is a verb and lives in ``identification/``. Design factories
take plain arrays in and return this package's blocks out.
"""
