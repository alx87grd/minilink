"""Model fitting from data (placeholder — content planned, home decided).

Offline verbs over logged :class:`~minilink.core.trajectory.Trajectory` data
that return fitted parameters or systems. One verb for physical parameters
and neural-network weights alike — both ride the parametric evaluator tier
(``f_p`` / ``jacobian_f_params``; see
``examples/scripts/diagrams/demo_params_gradient.py`` for the equation-error
prototype).

Planned modules (see ROADMAP.md §5):

- ``fitting.py`` — equation-error and prediction-error fits
- experiment design helpers (PRBS/chirp inputs live in ``blocks/sources``)

Placement rule: batch fitting over data lives here; estimators that run
*online inside a diagram* live in ``estimation/``.
"""
