"""Experimental tier — research lane, TRL < 3, repo-only (ROADMAP §2).

The import path states the maturity: nothing in the library imports these
modules, they are excluded from the published wheel, and their APIs may change
without notice.

- ``symbolic/`` — SymPy equations of motion (Lagrange / Kane) and export
- ``engines/`` — hand-rolled JAX contact worlds and the ANCF tire
- ``c_export`` — JAX → C transpiler for controller leaves
- ``ppo_jax`` — PPO reinforcement learning in pure JAX on a compiled plant
"""
