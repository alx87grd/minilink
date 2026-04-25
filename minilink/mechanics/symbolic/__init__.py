"""
Symbolic multibody mechanics (SymPy).

**TRL 1** — prototype derivation and export; API and performance not frozen.

Import convention: use the defining submodules, for example
``from minilink.mechanics.mechanical import MechanicalSystem`` (NumPy) or
``JaxMechanicalSystem`` (JAX), and ``from minilink.mechanics.symbolic.model import MechanicalModel``,
``from minilink.mechanics.symbolic.symbolic_system import MechanicalSystem`` (symbolic EoM; distinct from the numeric class).

Dependencies: SymPy (``pip install minilink[symbolic]``). For ``to_minilink(..., backend="jax")``,
also install JAX (``pip install minilink[jax]`` or ``jax`` + ``jaxlib``).
"""
