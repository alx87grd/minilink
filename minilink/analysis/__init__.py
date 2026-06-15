"""System analysis tools.

Verbs that *characterize* a system; they return data, ``LTISystem`` models,
or plots — never user-facing system classes (factories are fine).

Implemented modules:

- ``linearize.py`` — equilibrium linearization → matrices or ``LTISystem``
- ``structural.py`` — controllability / observability
- ``equilibria.py`` — trim points and root-finding on ``f``
- ``modal.py`` — ``modal_analysis`` (poles, modes) and ``animate_modal``

Planned modules (see ROADMAP.md §5):

- ``frequency.py`` — Bode, Nyquist, gain/phase margins, pole-zero
- ``time_response.py`` — step/impulse sugar over the simulator

Placement rule: if it *characterizes* an existing system, it belongs here;
if it *is* a block you wire into a diagram, it belongs in a library package.
"""
