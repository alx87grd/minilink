"""The state observer: the innovation form, its poles, its place in a loop, and what it is not."""

import numpy as np
import pytest

from minilink.control.design import place_gain
from minilink.control.state import StateFeedbackController
from minilink.core.diagram import DiagramSystem
from minilink.core.feedback import roles
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.estimation.observer import StateObserver

# the double integrator, position measured
A = np.array([[0.0, 1.0], [0.0, 0.0]])
B = np.array([[0.0], [1.0]])
C = np.array([[1.0, 0.0]])
POLES = [-2.0, -3.0]
L = place_gain(A.T, C.T, POLES).T  # the dual placement, written out here


def observer():
    return StateObserver(A, B, C, L)


def test_ports_params_and_labels():
    obs = observer()
    assert list(obs.inputs) == ["u", "y"] and list(obs.outputs) == ["x"]
    assert (obs.inputs["u"].dim, obs.inputs["y"].dim, obs.outputs["x"].dim) == (1, 1, 2)
    assert obs.id == "observer" and obs.n == 2
    assert obs.state.labels == ["x̂[0]", "x̂[1]"] == obs.outputs["x"].labels
    np.testing.assert_allclose(obs.params["L"], L)
    np.testing.assert_allclose(obs.params["ybar"], 0.0)
    x_hat = np.array([0.3, -0.2])
    np.testing.assert_allclose(obs.estimate(x_hat, np.zeros(2)), x_hat)
    np.testing.assert_allclose(np.sort(obs.poles.real), np.sort(POLES))


def test_the_innovation_form():
    obs = observer()
    x_hat, u, y = np.array([0.3, -0.2]), np.array([0.5]), np.array([1.0])
    y_hat = C @ x_hat
    expected = A @ x_hat + B @ u + L @ (y - y_hat)
    np.testing.assert_allclose(obs.f(x_hat, np.concatenate([u, y])), expected)


def test_the_operating_point_is_an_equilibrium_of_the_estimate():
    xbar, ubar = np.array([0.7, 0.0]), np.array([0.0])
    obs = StateObserver(A, B, C, L, xbar=xbar, ubar=ubar, ybar=C @ xbar + 0.1)
    np.testing.assert_allclose(obs.params["ybar"], C @ xbar + 0.1)
    dx_hat = obs.f(xbar, np.concatenate([ubar, C @ xbar + 0.1]))
    np.testing.assert_allclose(dx_hat, 0.0, atol=1e-12)
    # a shape that describes no plant is refused
    with pytest.raises(ValueError, match="L must be"):
        StateObserver(A, B, C, np.ones((3, 1)))


def test_the_estimate_converges_on_the_plant():
    plant = LTISystem(A, B, C)
    obs = observer()
    loop = DiagramSystem()
    loop.add_subsystem(plant, "plant")
    loop.add_subsystem(obs, "observer")
    loop.connect("plant", "y", "observer", "y")  # both read the nominal u = 0
    plant.x0 = np.array([1.0, 0.5])
    loop.refresh()
    traj = loop.compute_trajectory(tf=6.0)
    x_plant = traj.x[slice(*loop.state_index["plant"]), -1]
    x_hat = traj.x[slice(*loop.state_index["observer"]), -1]
    np.testing.assert_allclose(x_hat, x_plant, atol=1e-3)


def test_traces_under_jax():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    obs = observer()
    u = jnp.array([0.5, 1.0])
    J = jax.jacobian(lambda x: obs.f(x, u))(jnp.array([0.3, -0.2]))
    np.testing.assert_allclose(np.asarray(J), A - L @ C, atol=1e-12)


def test_a_box_with_a_gain_is_a_controller_and_the_roles_see_the_observer():
    plant = LTISystem(A, B, C)
    obs, gain = observer(), StateFeedbackController(np.array([[2.0, 3.0]]))
    box = DiagramSystem()
    box.add_subsystem(obs, "observer")
    box.add_subsystem(gain, "gain")
    box.connect("observer", "x", "gain", "x")
    box.connect("gain", "u", "observer", "u")
    box.expose("r", "y", "u")
    loop = box @ plant
    who = roles(loop)
    assert who.plant is plant and who.controller is box and who.observer is obs
    plant.x0 = np.array([1.0, 0.0])
    loop.refresh()
    traj = loop.compute_trajectory(tf=12.0)
    np.testing.assert_allclose(traj.x[:, -1], 0.0, atol=1e-3)
    # the separation principle: the loop's poles are the gain's and the observer's
    closed = np.sort(np.linalg.eigvals(loop.linearize().A()).real)
    np.testing.assert_allclose(closed, [-3.0, -2.0, -2.0, -1.0], atol=1e-6)


def test_an_observer_is_not_a_controller():
    from minilink.core.system import StepSystem

    plant = LTISystem(A, B, C)
    obs = observer()
    with pytest.raises(ValueError, match="an observer, not a controller"):
        obs @ plant
    with pytest.raises(ValueError, match="an observer, not a controller"):
        obs.plot_control_law()

    class SampledObserver(StepSystem):  # a discrete filter has the same ports
        def __init__(self):
            super().__init__(n=2)
            self.add_input_port("u", dim=1)
            self.add_input_port("y", dim=1)
            self.add_output_port("x", dim=2, function=self.compute_state)

    with pytest.raises(ValueError, match="an observer, not a controller"):
        SampledObserver() % 0.01


def test_print_shows_the_gain_and_the_poles():
    text = str(observer())
    assert (
        "L = " in text and "poles of A − L C" in text and "-2" in text and "-3" in text
    )
