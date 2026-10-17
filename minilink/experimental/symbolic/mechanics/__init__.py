"""
Symbolic multibody mechanics (SymPy).

Prototype derivation and export; API and performance are not frozen.

Import convention: use the defining submodules, for example
``from minilink.dynamics.abstraction.mechanical import MechanicalSystem`` (NumPy) or
``JaxMechanicalSystem`` (JAX), and ``from minilink.experimental.symbolic.mechanics.model import MechanicalModel``,
``from minilink.experimental.symbolic.mechanics.symbolic_system import MechanicalSystem`` (symbolic EoM; distinct from the numeric class).

Dependencies: SymPy (``pip install sympy``). For ``to_minilink(..., backend="jax")``,
also install JAX (``pip install jax jaxlib``).
"""
