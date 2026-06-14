"""Bridges to external ecosystems (placeholder — content planned, home decided).

Wrappers that let minilink systems talk to other frameworks, and external
models enter minilink as plants.

Planned modules (see ROADMAP.md §5):

- ``gymnasium.py`` — expose a diagram as an RL environment (policies come
  back as ``control/`` blocks)
- ``torch.py`` / ``flax.py`` — NN model wrappers
- cosimulation / FMI, multibody-description import

Placement rule: anything whose job is "talk to another ecosystem" lives
here; homegrown plants — whatever their implementation technology — live in
``dynamics/``.
"""
