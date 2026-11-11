"""Contract of the 3-D car: its attitude block, its corners, its reductions to rung 4."""

import numpy as np
import pytest

from minilink.core.backends import configure_jax
from minilink.dynamics.catalog.vehicles.drivetrain_bicycle import DrivetrainBicycle
from minilink.dynamics.catalog.vehicles.drivetrain_bicycle import (
    DrivetrainBicycle as Rung4,
)
from minilink.dynamics.catalog.vehicles.suspension_car import (
    FormulaCar3D,
    OffRoadCar3D,
    SpringDamper,
    SuspensionCar3D,
    attitude,
    euler_rate_map,
    euler_rate_map_derivative,
)
from minilink.dynamics.catalog.vehicles.terrain import Bumps, Slope, Terrain
from minilink.dynamics.catalog.vehicles.tires import BrushTire
from minilink.graphical.catalog import (
    car_skin_3d,
    formula_car_skin_3d,
    offroad_car_skin_3d,
)


def cruising(car, v, **states):
    """The car rolling straight at ``v`` at its ride height, the rear wheels matched."""
    x = car.x0.copy()
    x[3] = v
    x[6] = v / car.params["r_r"]
    for index, value in states.items():
        x[int(index[1:])] = value
    return x


# The attitude block


def test_the_rate_map_derivative_is_the_time_derivative_of_the_map():
    """``Ė [p, q, r]`` against a central difference of ``E`` along the rates."""
    rng = np.random.default_rng(5)
    for _ in range(20):
        phi, theta = rng.uniform(-1.2, 1.2, 2)
        p, q, r = rng.standard_normal(3)
        rates = np.array([p, q, r])
        h = 1.0e-6

        E_plus = euler_rate_map(phi + h * p, theta + h * q)
        E_minus = euler_rate_map(phi - h * p, theta - h * q)
        finite_difference = (E_plus - E_minus) @ rates / (2.0 * h)

        np.testing.assert_allclose(
            euler_rate_map_derivative(phi, theta, p, q, r),
            finite_difference,
            atol=1e-8,
        )


def test_the_rate_map_gives_the_body_rates_of_the_attitude():
    """``R^T Ṙ`` is the skew matrix of ``omega = E [p, q, r]``."""
    phi, theta, psi = 0.3, -0.2, 1.1
    rates = np.array([0.4, -0.7, 0.9])
    h = 1.0e-6

    R = attitude(phi, theta, psi)
    R_dot = (
        attitude(*(np.array([phi, theta, psi]) + h * rates))
        - attitude(*(np.array([phi, theta, psi]) - h * rates))
    ) / (2.0 * h)
    W = R.T @ R_dot
    omega = euler_rate_map(phi, theta) @ rates

    np.testing.assert_allclose([W[2, 1], W[0, 2], W[1, 0]], omega, atol=1e-8)


def test_a_free_spinning_body_keeps_its_energy_and_world_momentum():
    """Off the ground, no force but gravity: ``½ ωᵀ I ω`` and ``R I ω`` hold."""
    car = SuspensionCar3D()
    p = car.params
    for wheel in car.wheels:
        p["suspension"][wheel]["F_eps"] = 1.0e-9  # No residual load off the ground
    car.x0 = car.x0.copy()
    car.x0[9] = 50.0  # High above the ground: no load
    car.x0[13:15] = [0.8, -0.5]
    car.x0[5] = 0.6

    traj = car.compute_trajectory(tf=2.0, u=np.zeros(2), solver="scipy_max")

    I = np.diag([p["Ixx"], p["Iyy"], p["inertia"]])

    def energy_and_momentum(x):
        omega = euler_rate_map(x[10], x[11]) @ np.array([x[13], x[14], x[5]])
        return 0.5 * omega @ I @ omega, attitude(x[10], x[11], x[2]) @ I @ omega

    e0, h0 = energy_and_momentum(traj.x[:, 0])
    e1, h1 = energy_and_momentum(traj.x[:, -1])
    assert e1 == pytest.approx(e0, rel=1e-5)
    np.testing.assert_allclose(h1, h0, rtol=1e-5, atol=1e-6)


# The corners


def test_at_rest_on_flat_ground_the_four_loads_carry_the_weight():
    car = SuspensionCar3D()
    x = car.x0.copy()

    loads = car.loads(x, np.zeros(2))
    dx = car.f(x, np.zeros(2))

    # Within the smooth floor's residual, F_eps² / (4 Fz) per corner
    weight = car.params["mass"] * car.params["gravity"]
    assert loads.sum() == pytest.approx(weight, rel=1e-4)
    np.testing.assert_allclose(dx, 0.0, atol=1e-3)


def test_a_dropped_car_settles_at_its_ride_height():
    car = SuspensionCar3D()
    x0 = car.x0.copy()
    x0[9] += 0.05

    traj = car.compute_trajectory(tf=3.0, x0=x0, u=np.zeros(2), backend="jax")

    assert traj.x[9, -1] == pytest.approx(car.params["h_cg"], abs=1e-3)
    assert np.max(np.abs(traj.x[10:12, -1])) < 1e-6


def test_a_car_above_the_ground_carries_no_load():
    car = SuspensionCar3D()
    x = car.x0.copy()
    x[9] += 1.0

    loads = car.loads(x, np.zeros(2))

    assert np.all(loads < 1.0)


def test_the_spring_carries_its_static_load_at_ride_height():
    spring = SpringDamper(k=1.0e4, c=0.0, F_eps=1.0)

    assert spring.load(0.5, 0.0, 3000.0, 0.5) == pytest.approx(3000.0, rel=1e-6)
    assert spring.load(0.48, 0.0, 3000.0, 0.5) == pytest.approx(3200.0, rel=1e-6)
    assert spring.load(5.0, 0.0, 3000.0, 0.5) == pytest.approx(0.0, abs=1e-3)


# The car on flat ground


def test_the_first_nine_states_reduce_to_the_drivetrain_bicycle_on_a_narrow_track():
    """A steady turn on a narrow track with stiff springs: rung 4's yaw rate."""
    car = SuspensionCar3D()
    car.params["track"] = 0.3
    for wheel in car.wheels:
        car.params["suspension"][wheel]["k"] = 3.3e5
        car.params["suspension"][wheel]["c"] = 6.6e3
    bicycle = DrivetrainBicycle()
    u = np.array([8000.0, 0.03])
    x4 = np.zeros(9)
    x4[3] = 10.0
    x4[6] = 10.0 / 0.3

    run_3d = car.compute_trajectory(tf=8.0, x0=cruising(car, 10.0), u=u, backend="jax")
    run_4 = bicycle.compute_trajectory(tf=8.0, x0=x4, u=u, backend="jax")

    assert run_3d.x[5, -1] == pytest.approx(run_4.x[5, -1], rel=0.02)
    assert run_3d.x[3, -1] == pytest.approx(run_4.x[3, -1], rel=0.01)


def test_a_mirrored_turn_is_the_mirror_of_the_turn():
    car = SuspensionCar3D()
    x = cruising(car, 8.0, x4=0.3, x5=0.2, x10=0.02, x13=0.1)
    mirror = np.array([1, -1, -1, 1, -1, -1, 1, -1, 1, 1, -1, 1, 1, -1, 1], float)

    dx = car.f(x, np.array([5000.0, 0.1]))
    dx_mirrored = car.f(mirror * x, np.array([5000.0, -0.1]))

    np.testing.assert_allclose(dx_mirrored, mirror * dx, rtol=1e-9, atol=1e-9)


def test_a_coasting_car_never_gains_energy():
    car = SuspensionCar3D()
    p = car.params
    x0 = cruising(car, 12.0, x7=0.05, x10=0.02)

    traj = car.compute_trajectory(tf=3.0, x0=x0, u=np.array([0.0, 0.05]), backend="jax")

    def energy(x):
        kinetic = 0.5 * p["mass"] * (x[3] ** 2 + x[4] ** 2 + x[12] ** 2)
        omega = euler_rate_map(x[10], x[11]) @ np.array([x[13], x[14], x[5]])
        rotation = 0.5 * omega @ np.diag([p["Ixx"], p["Iyy"], p["inertia"]]) @ omega
        wheel = 0.5 * p["Jw_rear"] * x[6] ** 2
        return kinetic + rotation + wheel + p["mass"] * p["gravity"] * x[9]

    energies = np.array([energy(traj.x[:, k]) for k in range(0, traj.t.size, 50)])
    # The springs store a little energy: allow their share, a few joules
    assert np.all(np.diff(energies) < 5.0)
    assert energies[-1] < energies[0]


def test_braking_pitches_the_nose_down_and_a_left_turn_rolls_to_the_right():
    car = SuspensionCar3D()

    braking = car.compute_trajectory(
        tf=1.0, x0=cruising(car, 15.0), u=np.array([-1.0e5, 0.0]), backend="jax"
    )
    turning = car.compute_trajectory(
        tf=2.0, x0=cruising(car, 12.0), u=np.array([10000.0, 0.08]), backend="jax"
    )

    assert np.max(braking.x[11]) > 1e-3  # Pitch: nose down is positive θ about y
    assert turning.x[10, -1] > 1e-3  # Roll: the body leans out of a left turn


@pytest.mark.jax
def test_numpy_and_jax_agree_and_the_car_differentiates():
    jax = pytest.importorskip("jax")
    configure_jax(enable_x64=True)
    jnp = jax.numpy
    car = SuspensionCar3D(wheel_angles=True)
    x = np.concatenate([cruising(car, 6.0, x4=0.2, x5=0.1, x10=0.03)[:15], [1.0, 2.0]])
    u = np.array([3000.0, 0.1])

    on_numpy = car.f(x, u)
    on_jax = np.asarray(jax.jit(car.f)(jnp.asarray(x), jnp.asarray(u)))
    jacobian = np.asarray(jax.jacfwd(car.f)(jnp.asarray(car.x0), jnp.asarray(u)))
    dmu = jax.grad(
        lambda mu: car.f(
            jnp.asarray(x),
            jnp.asarray(u),
            0.0,
            {**car.params, "terrain": {**car.params["terrain"], "friction": mu}},
        )[4]
    )(1.0)

    np.testing.assert_allclose(on_jax, on_numpy, rtol=1e-10, atol=1e-10)
    assert np.all(np.isfinite(jacobian))
    assert np.isfinite(float(dmu))


def test_the_parts_are_held_and_a_whole_tree_is_routed_to_them():
    car = SuspensionCar3D(
        tire={
            "fl": BrushTire(),
            "fr": BrushTire(),
            "rl": BrushTire(c_alpha=10.0),
            "rr": BrushTire(c_alpha=10.0),
        }
    )

    for wheel in car.wheels:
        assert car.params["tire"][wheel] is car.tire[wheel].params
        assert car.params["suspension"][wheel] is car.suspension[wheel].params
    assert car.params["terrain"] is car.terrain.params
    assert car.params["tire"]["rl"]["c_alpha"] == 10.0


# The car on the terrains


def test_on_a_frictionless_slope_the_car_slides_at_g_sin_beta():
    beta = 0.1
    slope = Slope()
    slope.params["slope"] = beta
    car = SuspensionCar3D(terrain=slope)
    car.params["terrain"]["friction"] = 1.0e-3
    x0 = car.x0.copy()

    dx = car.f(x0, np.zeros(2))

    # Sliding back down the slope, along x, at g sin β (the body at rest on its springs)
    assert dx[3] == pytest.approx(-car.params["gravity"] * np.sin(beta), rel=0.05)


def test_on_bumps_the_body_heave_stays_bounded():
    bumps = Bumps()
    bumps.params["amplitude"] = 0.04
    car = SuspensionCar3D(terrain=bumps)
    x0 = cruising(car, 8.0)

    traj = car.compute_trajectory(
        tf=4.0, x0=x0, u=np.array([6000.0, 0.0]), backend="jax"
    )

    assert np.all(np.isfinite(traj.x))
    assert np.max(np.abs(traj.x[9] - car.params["h_cg"])) < 0.2
    assert np.max(np.abs(traj.x[10:12])) < 0.2


def test_a_patch_of_ice_lowers_only_the_grip_of_the_wheels_on_it():
    class IceOnTheLeft(Terrain):
        """Half the grip where y > 0."""

        def friction(self, x, y, t=0.0, params=None):
            return np.where(y > 0.0, 0.5, 1.0)

    car = SuspensionCar3D(terrain=IceOnTheLeft())
    x = cruising(car, 6.0, x4=3.0)  # Sliding sideways: every tire at its limit

    grip = car.grip(x, np.zeros(2))
    forces = car.contact_forces(x)
    lateral = np.abs(np.asarray(forces["Fy"]))

    assert lateral[0] < 0.6 * lateral[1]  # Front left on the ice, front right not
    assert lateral[2] < 0.6 * lateral[3]
    assert np.all(np.isfinite(grip))


def test_a_turn_too_fast_for_the_track_lifts_an_inner_wheel():
    car = SuspensionCar3D()
    car.params["h_cg"] = 1.0
    car.params["track"] = 1.0
    car.x0[9] = 1.0
    for wheel in car.wheels:
        car.params["tire"][wheel]["mu"] = 1.5
    fast = car.compute_trajectory(
        tf=3.0,
        x0=cruising(car, 18.0, x9=1.0),
        u=np.array([20000.0, 0.15]),
        backend="jax",
    )
    slow = car.compute_trajectory(
        tf=3.0, x0=cruising(car, 5.0, x9=1.0), u=np.array([2000.0, 0.15]), backend="jax"
    )

    min_load_fast = min(
        car.rollover(fast.x[:, k], np.zeros(2))[0] for k in range(0, fast.t.size, 20)
    )
    min_load_slow = min(
        car.rollover(slow.x[:, k], np.zeros(2))[0] for k in range(0, slow.t.size, 20)
    )
    assert min_load_fast < 50.0
    assert min_load_slow > 500.0


def test_the_wheel_frames_sit_at_the_hubs_and_the_contact_vectors_draw():
    car = SuspensionCar3D(wheel_angles=True)
    car.show_contact_vectors = True
    x = car.x0.copy()

    frames = car.tf(x, np.zeros(2))
    arrows = car.get_dynamic_geometry(x, np.zeros(2))

    for wheel in car.wheels:
        hub = frames[f"wheel_{wheel}"][:3, 3]
        assert hub[2] == pytest.approx(car.params["r_f"], abs=1e-9)
    assert len(arrows["world"]) == 8


# The presets and their looks


@pytest.mark.parametrize("preset", [FormulaCar3D, OffRoadCar3D])
def test_a_preset_rests_at_its_ride_height_and_its_look_fits_its_frames(preset):
    car = preset(wheel_angles=True)

    loads = car.loads(car.x0, np.zeros(2))
    look = car.skin(car)
    frames = car.tf(car.x0, np.zeros(2))

    assert np.sum(loads) == pytest.approx(car.params["mass"] * 9.81, rel=1e-3)
    assert set(look) - {"world"} <= set(frames)
    assert {"body", "wheel_fl", "wheel_fr", "wheel_rl", "wheel_rr"} <= set(look)


@pytest.mark.parametrize("preset", [FormulaCar3D, OffRoadCar3D])
def test_a_preset_drives_off_and_keeps_its_four_wheels_down(preset):
    # Straight: full power from rest spins the rear tires, and a steered launch spins the car
    car = preset()
    configure_jax()

    traj = car.compute_trajectory(
        tf=3.0, u=np.array([0.3 * car.params["P_max"], 0.0]), backend="jax"
    )

    assert traj.x[3, -1] > 10.0
    assert np.all(np.isfinite(traj.x))
    assert min(car.rollover(traj.x[:, -1], np.zeros(2))[0], 1e9) > 200.0


def test_a_tire_given_to_a_preset_replaces_its_slicks():
    car = FormulaCar3D(tire=BrushTire(mu=0.3))

    assert car.params["tire"]["fl"]["mu"] == 0.3


@pytest.mark.parametrize(
    "skin", [car_skin_3d, formula_car_skin_3d, offroad_car_skin_3d]
)
def test_a_3d_look_fits_the_3d_car_and_the_bicycle(skin):
    for car in (SuspensionCar3D(wheel_angles=True), Rung4(wheel_angles=True)):
        car.skin = skin

        look = car.skin(car)
        frames = car.tf(car.x0, np.zeros(2))

        assert set(look) - {"world"} <= set(frames)


def test_the_plain_car_has_a_look_and_one_track():
    car = SuspensionCar3D()

    car.track = 1.4

    assert car.params["track"] == 1.4
    assert "wheel_rl" in car.skin(car)


def test_the_front_axle_frame_sits_on_the_ground_between_its_contacts():
    car = SuspensionCar3D(terrain=Slope())
    x = car.x0.copy()
    x[0] = -10.0
    x[9] = car.terrain.height(-10.0, 0.0) + car.params["h_cg"]
    x[11] = -0.1

    frames = car.tf(x, np.zeros(2))
    corners = car.corner_geometry(x)
    axle = frames["axle_front"][:3, 3]

    assert np.allclose(axle, 0.5 * (corners["C"][0] + corners["C"][1]))
    assert axle[2] == pytest.approx(car.terrain.height(axle[0], axle[1]), abs=1e-3)


def test_a_terrain_given_to_two_cars_is_copied_for_each():
    bumps = Bumps()
    car_1 = SuspensionCar3D(terrain=bumps)
    car_2 = SuspensionCar3D(terrain=bumps)

    car_1.params["terrain"]["amplitude"] = 0.3

    assert car_2.params["terrain"]["amplitude"] == 0.05


def test_coasting_over_rolling_hills_the_car_follows_the_ground_and_loses_energy():
    hills = Bumps()
    hills.params["amplitude"] = 1.0  # [m]
    hills.params["wavelength"] = 25.0  # [m], grades up to 14°
    car = OffRoadCar3D(terrain=hills)
    configure_jax()
    mass = car.params["mass"]
    gravity = car.params["gravity"]
    x0 = cruising(car, 8.0, x0=3.0, x1=4.0, x2=0.6)
    x0[9] = hills.height(3.0, 4.0) + car.params["h_cg"]

    traj = car.compute_trajectory(tf=10.0, x0=x0, u=np.zeros(2), backend="jax")
    x = traj.x
    ride = x[9] - hills.height(x[0], x[1])
    energy = 0.5 * mass * (x[3] ** 2 + x[4] ** 2 + x[12] ** 2) + mass * gravity * x[9]

    # Over 2 m of relief: on its springs all along, never gaining energy
    assert np.ptp(hills.height(x[0], x[1])) > 1.5
    assert np.max(np.abs(ride - car.params["h_cg"])) < 0.15
    assert np.max(energy - energy[0]) < 1.0
