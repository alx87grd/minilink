"""Bridges to external ecosystems.

Wrappers that let minilink systems talk to other frameworks, and external
models enter minilink as plants.

Available modules:

- ``gymnasium.py`` — :class:`~minilink.interfaces.gymnasium.Sys2Gym` exposes a
  system + cost as an RL environment (``reward = -g * dt``); trained policies
  come back as :class:`~minilink.interfaces.gymnasium.SB3Controller` feedback
  blocks. Requires the optional ``gymnasium`` dependency (``pip install
  minilink[rl]``); training itself stays external (e.g. stable-baselines3).

Planned modules (see ROADMAP.md Later):

- ``torch.py`` / ``flax.py`` — NN model wrappers
- cosimulation / FMI, multibody-description import — live adapters implement
  the :mod:`minilink.simulation.realtime` contracts
  (:class:`~minilink.simulation.realtime.io.RealtimeInput` /
  :class:`~minilink.simulation.realtime.io.RealtimeOutput`)

Placement rule: anything whose job is "talk to another ecosystem" lives
here; homegrown plants — whatever their implementation technology — live in
``dynamics/``.
"""
