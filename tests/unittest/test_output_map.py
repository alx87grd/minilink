"""The output map ``h`` is the ``"y"`` port: on every block and catalog plant, and on their compiled evaluators."""

import inspect
import warnings

import numpy as np
import pytest

import minilink.blocks as blocks
import minilink.catalog as catalog
import minilink.control as control
from minilink import PID, Pendulum
from minilink.core.system import System

# The blocks whose constructors need arguments, built once each
BUILT = {
    "Gain": lambda: blocks.Gain([[2.0, 0.0], [0.0, 3.0]]),
    "MLP": lambda: blocks.MLP(2, 3, hidden=(4,)),
    "TransferFunction": lambda: blocks.TransferFunction([1.0], [1.0, 2.0]),
    "Lead": lambda: blocks.Lead(),
    "StateFeedback": lambda: control.StateFeedback(np.array([[1.0, 2.0]])),
}


def systems():
    """Every block and catalog plant that builds without arguments, and the ones of :data:`BUILT`."""
    found = {}
    for module in (catalog, blocks, control):
        for name in module.__all__:
            cls = getattr(module, name)
            if not (inspect.isclass(cls) and issubclass(cls, System)):
                continue
            build = BUILT.get(name, cls)
            try:
                with warnings.catch_warnings():
                    warnings.simplefilter("ignore")
                    found[name] = build()
            except TypeError:
                continue

    return found


SYSTEMS = systems()


@pytest.mark.parametrize("name", sorted(SYSTEMS))
def test_h_is_the_y_port_and_empty_without_one(name):
    sys = SYSTEMS[name]
    rng = np.random.default_rng(0)
    x = rng.normal(size=sys.n)
    u = rng.normal(size=sys.m)

    y = np.asarray(sys.h(x, u, 0.3))

    if "y" not in sys.outputs:
        assert y.size == 0
        return
    y_port = np.asarray(sys.outputs["y"].compute(x, u, 0.3))
    assert y.shape == y_port.shape
    np.testing.assert_allclose(y, y_port, rtol=1e-12, atol=1e-12)


def test_the_compiled_h_is_the_systems():
    plant = Pendulum()
    ev = plant.compile()
    x = np.array([0.3, -0.2])
    u = np.array([0.5])

    np.testing.assert_allclose(ev.h(x, u), plant.h(x, u), rtol=1e-12)
    np.testing.assert_allclose(ev.h(x), plant.h(x, np.zeros(1)), rtol=1e-12)


def test_the_compiled_h_of_a_diagram_is_its_y_port_or_empty():
    plant = Pendulum()
    loop = PID() @ plant
    ev = loop.compile()
    x = np.zeros(loop.n)

    assert "y" not in loop.outputs or ev.h(x).shape == (loop.p,)
    assert np.asarray(PID().compile().h(np.zeros(PID().n))).size == 0


def test_the_compiled_h_of_a_sampled_plant_is_its_y_port():
    from minilink import discretize

    view = discretize(Pendulum(), 0.01)
    ev = view.compile()
    x = np.array([0.3, -0.2])

    np.testing.assert_allclose(ev.h(x, np.zeros(1), 0), view.h(x, np.zeros(1), 0))
