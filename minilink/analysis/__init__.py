"""System analysis tools (placeholder — content planned, home decided).

Verbs that *characterize* a system; they return data, ``LTISystem`` models,
or plots — never user-facing system classes (factories are fine).

Planned modules (see ROADMAP.md §5 and the use-case projection):

- ``linearize.py`` — equilibrium linearization → ``LTISystem``
- ``frequency.py`` — Bode, Nyquist, gain/phase margins, pole-zero
- ``time_response.py`` — step/impulse sugar over the simulator
- ``structural.py`` — controllability/observability, modal analysis
- ``equilibria.py`` — trim points and root-finding on ``f``

Placement rule: if it *characterizes* an existing system, it belongs here;
if it *is* a block you wire into a diagram, it belongs in a library package.
"""
