"""The state observer: the innovation form, its poles, its place in a loop, and what it is not."""

import numpy as np
import pytest
from scipy.linalg import solve_continuous_are

from minilink.analysis.linearization import linearize
from minilink.control.design import lqg, lqr_gain, place_gain
from minilink.control.state import StateFeedbackController
from minilink.core.diagram import DiagramSystem
from minilink.core.feedback import roles
from minilink.dynamics.abstraction.state_space import LTISystem
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.estimation.design import kalman, kalman_gain, luenberger, luenberger_gain
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


# The designs: the duals of place and lqr


def test_luenberger_gain_places_the_error_poles():
    L_design = luenberger_gain(A, C, POLES)
    np.testing.assert_allclose(np.poly(A - L_design @ C), np.poly(POLES), atol=1e-9)
    np.testing.assert_allclose(L_design, place_gain(A.T, C.T, POLES).T)
    with pytest.raises(ValueError, match="not observable"):
        luenberger_gain(
            A, np.array([[0.0, 1.0]]), POLES
        )  # velocity alone hides position


def test_kalman_gain_is_the_dual_of_lqr():
    Q_n, R_n = np.diag([0.1, 0.3]), np.array([[0.2]])
    L_design = kalman_gain(A, C, Q_n, R_n)
    np.testing.assert_allclose(L_design, lqr_gain(A.T, C.T, Q_n, R_n).T)
    assert np.all(np.linalg.eigvals(A - L_design @ C).real < 0)
    # the hand calculation of the GRO501 notebook
    P = solve_continuous_are(A.T, C.T, Q_n, R_n)
    np.testing.assert_allclose(L_design, P @ C.T @ np.linalg.inv(R_n))


def test_the_three_forms_give_one_observer():
    plant = Pendulum()  # y is the state, C = I
    x_bar = np.array([0.3, 0.0])
    Q_n, R_n = np.diag([0.01, 0.1]), 1e-3 * np.eye(2)
    model = linearize(plant, x_bar)
    on_arrays = kalman(
        model.A(), model.B(), model.C(), Q_n, R_n, x_bar=x_bar, u_bar=model.ubar
    )
    on_model = kalman(model, Q_n, R_n)
    on_plant = kalman(plant, Q_n, R_n, x_bar=x_bar)
    for obs in (on_model, on_plant):
        np.testing.assert_allclose(obs.params["L"], on_arrays.params["L"])
        np.testing.assert_allclose(obs.params["xbar"], x_bar)
        np.testing.assert_allclose(obs.params["ybar"], plant.h(x_bar, model.ubar))
        assert obs.state.labels == ["theta_hat", "dtheta_hat"]
        assert obs.name == "Kalman Filter" and obs.id == "observer"
    np.testing.assert_allclose(
        kalman(plant, Q_n=Q_n, R_n=R_n, x_bar=x_bar).params["L"], on_plant.params["L"]
    )
    with pytest.raises(TypeError, match=r"kalman\(sys, Q_n, R_n\)"):
        kalman(plant, x_bar, Q_n, R_n)


def test_luenberger_on_the_plant_and_the_hat():
    obs = luenberger(LTISystem(A, B, C), POLES)
    np.testing.assert_allclose(np.sort(obs.poles.real), np.sort(POLES))
    assert obs.name == "Luenberger Observer" and obs.state.labels == [
        "x̂[0]",
        "x̂[1]",
    ]
    one_letter = LTISystem(A, B, C)
    one_letter.state.labels = ["q", "v"]
    assert luenberger(one_letter, POLES).state.labels == ["q\u0302", "v\u0302"]


# lqg: an LQR gain on a Kalman estimate, one controller box


def test_lqg_is_the_observer_and_the_gain_in_one_box():
    Q, R = np.diag([1.0, 0.1]), np.array([[0.5]])
    Q_n, R_n = np.diag([0.1, 0.3]), np.array([[0.2]])
    box = lqg(A, B, C, Q, R, Q_n, R_n)
    assert list(box.subsystems) == ["observer", "gain"]
    assert list(box.inputs) == ["r", "y"] and list(box.outputs) == ["u"]
    np.testing.assert_allclose(box.subsystems["gain"].params["K"], lqr_gain(A, B, Q, R))
    np.testing.assert_allclose(
        box.subsystems["observer"].params["L"], kalman_gain(A, C, Q_n, R_n)
    )
    plant = LTISystem(A, B, C)
    loop = box @ plant
    who = roles(loop)
    assert who.controller is box and who.observer is box.subsystems["observer"]
    # the separation principle: the loop's poles are the gain's and the observer's
    K, L_design = (
        box.subsystems["gain"].params["K"],
        box.subsystems["observer"].params["L"],
    )
    expected = np.concatenate(
        [np.linalg.eigvals(A - B @ K), np.linalg.eigvals(A - L_design @ C)]
    )
    closed = np.linalg.eigvals(loop.linearize().A())
    np.testing.assert_allclose(
        np.sort_complex(closed), np.sort_complex(expected), atol=1e-8
    )
    # the same box from the model and from the plant
    model = linearize(plant)
    np.testing.assert_allclose(
        lqg(model, Q, R, Q_n, R_n).subsystems["gain"].params["K"], K
    )
    np.testing.assert_allclose(
        lqg(plant, Q=Q, R=R, Q_n=Q_n, R_n=R_n).subsystems["observer"].params["L"],
        L_design,
    )


def test_lqg_holds_the_cart_pole_upright_from_a_disturbed_start():
    from minilink.dynamics.catalog.pendulum.cartpole import CartPole

    plant = CartPole()
    x_up = np.array([0.0, np.pi, 0.0, 0.0])
    box = lqg(
        plant,
        Q=np.diag([1.0, 10.0, 1.0, 1.0]),
        R=np.array([[0.1]]),
        Q_n=1e-2 * np.eye(4),
        R_n=1e-3 * np.eye(4),
        x_bar=x_up,
    )
    np.testing.assert_allclose(box.subsystems["observer"].params["xbar"], x_up)
    assert box.subsystems["observer"].state.labels == [
        "x̂",
        "theta_hat",
        "dx_hat",
        "dtheta_hat",
    ]
    loop = box @ plant
    plant.x0 = np.array([-0.5, np.pi + 0.2, 0.0, 0.0])
    loop.refresh()
    traj = loop.compute_trajectory(tf=8.0)
    plant_state = traj.x[slice(*loop.state_index["plant"]), -1]
    np.testing.assert_allclose(plant_state, x_up, atol=1e-2)
