"""System analysis tools.

Verbs that *characterize* a system; they return data, ``LTISystem`` models,
or plots — never user-facing system classes (factories are fine).

Teaching imports::

    from minilink.analysis import bode, jacobian, modal_analysis
    from minilink.analysis.linearize import linearize
    from minilink.analysis.discretize import discretize

Every verb reads ``tool(<what>, x_bar=None, u_bar=None, t=0.0, params=None, *,
method="auto", eps=1e-6, ...)``: ``jacobian(sys, "f", "x")`` is ∂f/∂x, the
channel tools (``bode``, ``pzmap``, ``transfer_function``) take ``of=`` /
``wrt=`` keywords, and the same verbs are methods on every ``System``.

Symbols whose **name matches a submodule** (``linearize``, ``discretize``) are
imported from that submodule — not re-exported on the package attribute.

Implemented modules:

- ``derivatives.py`` — ``jacobian(sys, of, wrt)`` at an operating point
- ``linearize.py`` — equilibrium linearization → matrices or ``LTISystem``
- ``structural.py`` — controllability / observability
- ``equilibria.py`` — trim points and root-finding on ``f``
- ``modal.py`` — ``modal_analysis`` (poles, modes) and ``animate_modal``
- ``linear.py`` — the linear-algebra core on one channel ``(A, b, c, d)``:
  poles, zeros, frequency response, margins, root locus, step response
- ``frequency.py`` — one-channel Bode, pole-zero, Nyquist, margins, root
  locus, transfer function and their plots (matplotlib or plotly)
- ``time_response.py`` — step response and ``StepInfo``
- ``discretize.py`` — continuous→step plant wrappers

Planned additions (see ROADMAP.md teaching-release priorities):

- Nichols chart; multi-system overlays in one figure

Placement rule: if it *characterizes* an existing system, it belongs here;
if it *is* a block you wire into a diagram, it belongs in a library package.
"""

from __future__ import annotations

from minilink.core.facade import lazy_facade

# Only names that do not collide with submodule filenames.
_EXPORTS: dict[str, tuple[str, str]] = {
    "LyapunovCertificate": ("minilink.analysis.lyapunov", "LyapunovCertificate"),
    "StructuralResult": ("minilink.analysis.structural", "StructuralResult"),
    "VerificationReport": ("minilink.analysis.lyapunov", "VerificationReport"),
    "animate_modal": ("minilink.analysis.modal", "animate_modal"),
    "bode": ("minilink.analysis.frequency", "bode"),
    "controllability": ("minilink.analysis.structural", "controllability"),
    "find_equilibrium": ("minilink.analysis.equilibria", "find_equilibrium"),
    "frequency_response": ("minilink.analysis.frequency", "frequency_response"),
    "jacobian": ("minilink.analysis.derivatives", "jacobian"),
    "margins": ("minilink.analysis.frequency", "margins"),
    "modal_analysis": ("minilink.analysis.modal", "modal_analysis"),
    "nyquist": ("minilink.analysis.frequency", "nyquist"),
    "observability": ("minilink.analysis.structural", "observability"),
    "region_of_attraction": (
        "minilink.analysis.lyapunov",
        "region_of_attraction",
    ),
    "plot_region_of_attraction": (
        "minilink.analysis.lyapunov",
        "plot_region_of_attraction",
    ),
    "plot_bode": ("minilink.analysis.frequency", "plot_bode"),
    "plot_nyquist": ("minilink.analysis.frequency", "plot_nyquist"),
    "plot_pzmap": ("minilink.analysis.frequency", "plot_pzmap"),
    "plot_root_locus": ("minilink.analysis.frequency", "plot_root_locus"),
    "plot_step_response": ("minilink.analysis.time_response", "plot_step_response"),
    "pzmap": ("minilink.analysis.frequency", "pzmap"),
    "root_locus": ("minilink.analysis.frequency", "root_locus"),
    "step_info": ("minilink.analysis.time_response", "step_info"),
    "step_response": ("minilink.analysis.time_response", "step_response"),
    "transfer_function": ("minilink.analysis.frequency", "transfer_function"),
}

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
