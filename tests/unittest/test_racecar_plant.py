"""Contract of the racecar plant: limits, symmetries, no-slip agreement, both backends."""

import numpy as np
import pytest

from minilink import KinematicBicycle
from minilink.catalog import UdeSRacecar, UdeSRacecarDyn
from minilink.dynamics.catalog.vehicles.racecar import stiff_tires

jax = pytest.importorskip("jax", reason="the JAX parity checks need jax")
jnp = jax.numpy


def cruise_state(car, speed, delta=0.0, power=0.0):
    """State of the car rolling at ``speed`` with its rear wheel matched."""
    x = np.zeros(car.n)
    x[3] = speed
    x[6] = speed / car.params["r_r"]
    x[7] = delta
    x[8] = power

    return x


def test_kinematic_scale_matches_the_public_params():
    """The kinematic car is the bicycle at ``PUBLIC_RACECAR_PARAMS`` lengths."""
    car = UdeSRacecar()

    assert car.n == 3
    assert car.params["a"] == pytest.approx(car.params["b"])
    assert car.params["length"] == pytest.approx(car.params["a"] + car.params["b"])
    assert list(car.inputs["u"].labels) == ["speed", "steering"]


def test_ports_and_state_layout():
    """Nine states, two bounded commands, and the sensor ports the demos read."""
    car = UdeSRacecarDyn()

    assert car.n == 9
    assert list(car.inputs) == ["P_cmd", "delta_cmd"]
    assert car.inputs["P_cmd"].upper_bound == pytest.approx(car.params["P_max"])
    assert car.inputs["delta_cmd"].lower_bound == pytest.approx(
        -car.params["delta_max"]
    )
    assert set(car.outputs) == {"y", "speed", "slip", "grip", "imu", "power"}
    assert car.h(car.x0, np.zeros(car.m)).shape == (9,)


def test_solver_hint_is_under_the_fastest_mode():
    """The hint is shorter than the fastest time constant of the linearized plant.

    Measured on the Jacobian itself rather than on a restatement of the formula that
    produced the hint, so that a mode the estimate forgets shows up here.
    """
    car = UdeSRacecarDyn(named_ports=False)
    hint = car.solver_info["smallest_time_constant"]

    # at rest: the slip denominators sit on their floor, so the tire modes are stiffest
    A = np.asarray(jax.jacfwd(car.f)(jnp.zeros(car.n), jnp.zeros(2)))
    fastest = 1.0 / np.abs(np.linalg.eigvals(A).real).max()

    assert 0.0 < hint <= fastest
    assert hint < car.params["engine_tau"]


def test_rigid_tires_reproduce_the_kinematic_bicycle():
    """With no slip left, the rear axle follows the textbook bicycle path."""
    car = UdeSRacecarDyn()
    car.params = stiff_tires(car.params)
    speed, delta, tf, n = 3.0, 0.15, 2.0, 401
    car.x0 = cruise_state(car, speed, delta)
    L = car.params["a"] + car.params["b"]
    car.x0[0] = car.params["b"]  # both models start with their rear axle at the origin
    car.x0[4] = car.params["b"] * speed * np.tan(delta) / L
    car.x0[5] = speed * np.tan(delta) / L

    u = np.vstack([np.full(n, 8.0), np.full(n, delta)])
    traj = car.compute_forced(u=u, tf=tf, n_steps=n, verbose=False)

    kinematic = KinematicBicycle()
    kinematic.params = {"a": car.params["a"], "b": car.params["b"], "length": 0.34}
    kinematic.x0 = np.zeros(3)
    kin_traj = kinematic.compute_forced(
        u=np.vstack([traj.x[3], np.full(n, delta)]), tf=tf, n_steps=n, verbose=False
    )

    # the plant carries its pose at the centre of gravity; the bicycle at the rear axle
    b = car.params["b"]
    x_rear = traj.x[0] - b * np.cos(traj.x[2])
    y_rear = traj.x[1] - b * np.sin(traj.x[2])
    assert np.max(np.abs(traj.x[2] - kin_traj.x[2])) < 0.02
    assert np.max(np.abs(x_rear - kin_traj.x[0])) < 0.02
    assert np.max(np.abs(y_rear - kin_traj.x[1])) < 0.02


def test_steady_cornering_matches_the_understeer_formula():
    """``delta = L/R + K_us a_y/g`` holds at small lateral acceleration."""
    car = UdeSRacecarDyn()
    p = car.params
    L = p["a"] + p["b"]
    K_us = 1.0 / p["c_alpha_f"] - 1.0 / p["c_alpha_r"]
    speed, delta, n = 3.0, 0.06, 1201
    car.x0 = cruise_state(car, speed, delta, power=7.0)

    u = np.vstack([np.full(n, 7.0), np.full(n, delta)])
    traj = car.compute_forced(u=u, tf=6.0, n_steps=n, verbose=False)

    vx, yaw_rate = traj.x[3, -1], traj.x[5, -1]
    radius = vx / yaw_rate
    predicted = (L + K_us * vx**2 / p["gravity"]) / delta
    assert radius == pytest.approx(predicted, rel=0.06)


def test_understeer_gradient_is_the_stiffness_difference():
    """The measured gradient is ``1/c_alpha_f - 1/c_alpha_r`` within the brush curvature."""
    car = UdeSRacecarDyn()
    p = car.params
    L = p["a"] + p["b"]
    speed, n = 3.0, 1201

    extra, lateral = [], []
    for delta in (0.03, 0.06):
        car.x0 = cruise_state(car, speed, delta, power=7.0)
        u = np.vstack([np.full(n, 7.0), np.full(n, delta)])
        traj = car.compute_forced(u=u, tf=6.0, n_steps=n, verbose=False)
        vx, yaw_rate = traj.x[3, -1], traj.x[5, -1]
        extra.append(delta - L * yaw_rate / vx)
        lateral.append(vx * yaw_rate / p["gravity"])

    K_us = (extra[1] - extra[0]) / (lateral[1] - lateral[0])
    assert K_us == pytest.approx(1.0 / p["c_alpha_f"] - 1.0 / p["c_alpha_r"], rel=0.25)


def test_top_speed_is_set_by_the_power_and_the_drag():
    """Full power on a straight settles at a steady speed, with the power budget closed."""
    car = UdeSRacecarDyn()
    p = car.params
    n = 2001
    car.x0 = np.zeros(car.n)

    u = np.vstack([np.full(n, p["P_max"]), np.zeros(n)])
    traj = car.compute_forced(u=u, tf=25.0, n_steps=n, verbose=False)

    top_speed = traj.x[3, -1]
    assert 9.0 < top_speed < 12.0
    assert abs(car.f(traj.x[:, -1], np.array([p["P_max"], 0.0]))[3]) < 0.05
    P, P_wheel = car.power(traj.x[:, -1], np.zeros(2))
    assert P == pytest.approx(p["P_max"], rel=1e-3)
    assert P_wheel <= p["P_max"] + 1e-9


def test_launch_is_traction_limited_and_spins_the_rear_wheel():
    """From rest, full power slips the rear tire instead of pushing the car harder."""
    car = UdeSRacecarDyn()
    p = car.params
    n = 401
    car.x0 = np.zeros(car.n)

    u = np.vstack([np.full(n, p["P_max"]), np.zeros(n)])
    traj = car.compute_forced(u=u, tf=1.0, n_steps=n, verbose=False)

    slips = np.array([car.slips(traj.x[:, k], np.zeros(2)) for k in range(n)])
    imu = np.array([car.imu(traj.x[:, k], np.zeros(2)) for k in range(n)])
    grip_limit = p["mu"] * p["gravity"] * p["a"] / (p["a"] + p["b"])
    assert slips[:, 0].max() > 0.1
    assert imu[:, 0].max() < grip_limit * 1.02
    assert traj.x[6, 10] * p["r_r"] > traj.x[3, 10]


def test_torque_is_capped_at_standstill():
    """At zero wheel rate the drive delivers ``tau_sat``, not an infinite torque."""
    car = UdeSRacecarDyn()
    p = car.params
    x = np.zeros(car.n)
    x[8] = 1.0e4

    w_rear_dot = car.f(x, np.array([p["P_max"], 0.0]))[6]

    assert w_rear_dot == pytest.approx(p["tau_sat"] / p["Jw_rear"], rel=1e-6)


def test_steering_is_rate_limited_and_stops_at_the_end_stop():
    """A full-lock command is reached at the servo rate, and the stop holds it."""
    car = UdeSRacecarDyn()
    p = car.params
    n = 601
    car.x0 = np.zeros(car.n)

    u = np.vstack([np.zeros(n), np.full(n, 2.0)])
    traj = car.compute_forced(u=u, tf=1.5, n_steps=n, verbose=False)

    rate = np.diff(traj.x[7]) / (traj.t[1] - traj.t[0])
    assert rate.max() <= p["steer_rate_max"] * 1.001
    assert traj.x[7, -1] == pytest.approx(p["delta_max"], rel=1e-3)

    beyond = np.zeros(car.n)
    beyond[7] = 1.5 * p["delta_max"]
    assert car.f(beyond, np.array([0.0, 2.0]))[7] <= 0.0


def test_mirror_symmetry():
    """The car built on a flat floor turns right exactly as it turns left."""
    car = UdeSRacecarDyn()
    mirror = np.array([1.0, -1.0, -1.0, 1.0, -1.0, -1.0, 1.0, -1.0, 1.0])
    x = np.array([1.0, 0.4, 0.2, 4.0, -0.3, 0.8, 85.0, 0.1, 30.0])
    u = np.array([40.0, 0.2])

    dx = car.f(x, u)
    dx_mirrored = car.f(mirror * x, np.array([u[0], -u[1]]))

    assert dx_mirrored == pytest.approx(mirror * dx, rel=1e-9, abs=1e-12)


def test_coasting_never_gains_energy():
    """With no power in, the kinetic energy of car and wheel only falls."""
    car = UdeSRacecarDyn()
    p = car.params
    n = 801
    car.x0 = cruise_state(car, 6.0, delta=0.05)

    u = np.zeros((2, n))
    traj = car.compute_forced(u=u, tf=8.0, n_steps=n, verbose=False)

    energy = (
        0.5 * p["mass"] * (traj.x[3] ** 2 + traj.x[4] ** 2)
        + 0.5 * p["inertia"] * traj.x[5] ** 2
        + 0.5 * p["Jw_rear"] * traj.x[6] ** 2
    )
    assert np.max(np.diff(energy)) < 1.0e-6


def test_numpy_and_jax_agree_on_the_equations_and_the_ports():
    """One equation path, two backends."""
    car = UdeSRacecarDyn()
    x = np.array([1.0, 0.4, 0.2, 4.0, -0.3, 0.8, 85.0, 0.1, 30.0])
    u = np.array([40.0, 0.2])

    on_numpy = np.concatenate(
        [
            car.f(x, u),
            car.h(x, u),
            car.slips(x, u),
            car.grip(x, u),
            car.imu(x, u),
            car.power(x, u),
        ]
    )
    x_jax, u_jax = jnp.asarray(x), jnp.asarray(u)
    on_jax = np.concatenate(
        [
            np.asarray(car.f(x_jax, u_jax)),
            np.asarray(car.h(x_jax, u_jax)),
            np.asarray(car.slips(x_jax, u_jax)),
            np.asarray(car.grip(x_jax, u_jax)),
            np.asarray(car.imu(x_jax, u_jax)),
            np.asarray(car.power(x_jax, u_jax)),
        ]
    )

    assert on_jax == pytest.approx(on_numpy, rel=1e-6, abs=1e-9)


def test_jacobian_is_finite_at_rest():
    """The car parked at the origin still has a usable linearization."""
    car = UdeSRacecarDyn()

    A = np.asarray(jax.jacfwd(car.f)(jnp.zeros(car.n), jnp.zeros(car.m)))
    B = np.asarray(jax.jacfwd(car.f, argnums=1)(jnp.zeros(car.n), jnp.zeros(car.m)))

    assert np.all(np.isfinite(A))
    assert np.all(np.isfinite(B))
    assert B[8, 0] == pytest.approx(1.0 / car.params["engine_tau"])
