"""The regression gate of minilink (not shipped, not correctness tests).

``run_regression_check.py`` runs the suites (``suites``, ``scenarios``) against the
committed ``baselines``; the measuring helpers (``f_evaluators``, ``simulation``,
``optimization``, ``trajopt``, ``dynamic_programming``), shared plumbing (``common``,
``baseline``, ``host_profiles``) and synthetic fixtures (``systems``) serve it.
"""
