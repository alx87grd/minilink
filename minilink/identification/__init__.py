"""Model fitting from logged :class:`~minilink.core.trajectory.Trajectory` data (planned, step C4).

Placement rule: batch fitting over data lives here; estimators that run
*online inside a diagram* live in ``estimation/``. The design is step C4 of
docs/plans/TODO.md (ROADMAP.md §5); the equation-error prototype is
``examples/demos/compile/params_gradient.py``.
"""
