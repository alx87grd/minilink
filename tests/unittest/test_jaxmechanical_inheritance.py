"""Regression tests for the JaxMechanicalSystem / MechanicalSystem subclass relationship."""

import pytest

from minilink.dynamics.abstraction.mechanical import (
    JaxMechanicalSystem,
    MechanicalSystem,
)


def test_jax_mechanical_subclasses_numpy_mechanical():
    assert issubclass(JaxMechanicalSystem, MechanicalSystem)


def test_jax_mechanical_init_matches_numpy_layout():
    a = MechanicalSystem(dof=3, actuators=2)
    b = JaxMechanicalSystem(dof=3, actuators=2)
    assert (a.n, a.m, a.p) == (b.n, b.m, b.p)
    assert a.state.labels == b.state.labels
    assert a.state.units == b.state.units
    assert a.inputs["u"].labels == b.inputs["u"].labels
    assert a.inputs["u"].units == b.inputs["u"].units


def test_jax_mechanical_constructor_does_not_require_jax():
    # Constructor must work without invoking JAX (lazy-import policy).
    sys = JaxMechanicalSystem(dof=2)
    assert sys.dof == 2
    assert sys.name.endswith("Mechanical System")


def test_jax_cartpole_still_subclasses_jax_mechanical():
    pytest.importorskip("jax")
    from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole

    assert issubclass(JaxCartPole, JaxMechanicalSystem)
