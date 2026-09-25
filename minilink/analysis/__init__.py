"""Analysis verbs on a ``System``: derivatives, linearization, structure, equilibria, modes, frequency and time responses, Lyapunov certificates.

Every tool reads ``tool(sys, x_bar, u_bar, t, params, *, method="auto", eps)`` and the
same verbs are methods on every ``System``.
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
