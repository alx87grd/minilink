"""The project folders' own car models and helpers: ``car_trajopt`` and ``pathtracking``.

The catalog's contracts are in ``test_dynamics_catalog.py``; this file checks the code
under ``examples/projects/``, which builds on the catalog.
"""

import unittest

import numpy as np
import pytest

from examples.projects.car_trajopt.vehicles.extras import (
    ConstantSpeedKinematicCar,
    DynamicHolonomicMobileRobot,
    HolonomicMobileRobot3D,
)
from examples.projects.car_trajopt.vehicles.ladder import (
    BicycleDynEngine,
    BicycleDynServo,
)
from examples.projects.pathtracking.common.vehicle import (
    LinearTire,
    Pacejka,
    RearDriveBicycle,
    create_vehicle,
)
from minilink.core.backends import configure_jax
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import BicycleDynRate
from minilink.graphical.animation.primitives import Arrow
from tests.unittest.graphics_contract_helpers import (
    geometry_smoke,
    resolved_primitive_count,
)

# The car_trajopt project's extras: the reference values, the arrows, the params


def test_the_dynamic_holonomic_robot_integrates_its_acceleration():
    np.testing.assert_allclose(
        DynamicHolonomicMobileRobot().f(
            np.array([1.0, 2.0, 3.0, 4.0]), np.array([5.0, 6.0])
        ),
        [3.0, 4.0, 5.0, 6.0],
    )


@pytest.mark.parametrize(
    "make",
    [DynamicHolonomicMobileRobot, HolonomicMobileRobot3D],
    ids=lambda m: m.__name__,
)
def test_the_holonomic_extras_draw_their_command_arrow(make):
    system = make()
    assert resolved_primitive_count(system, Arrow) == 1
    geometry_smoke(system)


def test_the_constant_speed_car_reads_the_params_it_is_given():
    car = ConstantSpeedKinematicCar()
    x = np.array([0.3, -0.2, 0.4])
    u = np.array([0.25])
    params = {**car.params, "a": 2.5}
    stored = ConstantSpeedKinematicCar()
    stored.params = params

    given = car.tf(x, u, 0.0, params=params)
    from_stored = stored.tf(x, u, 0.0)
    default = car.tf(x, u, 0.0)

    for frame in given:
        np.testing.assert_array_equal(
            np.asarray(given[frame]), np.asarray(from_stored[frame])
        )
    assert any(
        not np.array_equal(np.asarray(given[frame]), np.asarray(default[frame]))
        for frame in given
    )


def test_the_constant_speed_car_drives_the_bicycle_at_the_speed_given():
    car = ConstantSpeedKinematicCar()
    u = np.array([0.2])
    np.testing.assert_array_equal(car.bicycle_input(u), [2.0, 0.2])
    fast = {**car.params, "speed": 5.0}
    np.testing.assert_array_equal(car.bicycle_input(u, fast), [5.0, 0.2])


# The car_trajopt project's ladder: the servo and engine rungs, the car profiles


@pytest.mark.jax
class TestBicycleDynServo(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynServo(named_ports=True)
        self.uy = BicycleDynServo()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertEqual(self.uy.n, 9)
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)

    def test_f_matches_named_servo_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 4.0, 0.1, 0.0])
        u = np.array([200.0, 0.05])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)

    def test_f_matches_named_servo_inputs_with_large_steer_command(self):
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 33.0, 0.0, 0.0])
        u = np.array([0.0, 1.0])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)

    def test_zero_torque_cruise_actuators_near_equilibrium(self):
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0 / 0.3, 0.0, 0.0])
        u = np.array([0.0, 0.0])
        dx = np.asarray(self.named.f(x, u))
        np.testing.assert_allclose(dx[6:8], np.zeros(2), atol=0.5)

    def test_params_override_Ca(self):
        sys = BicycleDynServo()
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.5, 0.0, 10.0 / 0.3, 0.05, 0.0])
        u = np.array([0.0, 0.0])
        params = {**sys.params, "Ca": 0.5 * sys.params["Ca"]}
        dx_nom = np.asarray(sys.f(x, u))
        dx_soft = np.asarray(sys.f(x, u, params=params))
        self.assertFalse(np.allclose(dx_nom, dx_soft))


@pytest.mark.jax
class TestBicycleDynEngine(unittest.TestCase):
    def setUp(self):
        configure_jax(enable_x64=True)
        self.named = BicycleDynEngine(named_ports=True)
        self.uy = BicycleDynEngine()

    def test_standard_ports(self):
        self.assertIn("u", self.uy.inputs)
        self.assertEqual(self.uy.inputs["u"].dim, 2)
        self.assertEqual(self.uy.n, 9)
        self.assertEqual(self.uy.inputs["u"].labels, ["P_cmd", "delta_cmd"])
        self.assertIn("y", self.uy.outputs)
        self.assertEqual(self.uy.outputs["y"].dim, self.uy.n)
        self.assertNotIn("torque_tau", self.uy.params)
        self.assertNotIn("transmission_ratio", self.uy.params)
        self.assertNotIn("engine_power_peak", self.uy.params)

    def test_f_matches_named_engine_inputs(self):
        x = np.array([1.0, 2.0, 0.1, 3.0, 0.2, 0.05, 10.0, 0.1, 5000.0])
        u = np.array([8000.0, 0.05])
        dx_named = np.asarray(self.named.f(x, u))
        dx_uy = np.asarray(self.uy.f(x, u))
        np.testing.assert_allclose(dx_named, dx_uy, rtol=1e-09, atol=1e-09)

    def test_unsaturated_torque_is_power_over_omega(self):
        sys = BicycleDynEngine()
        w = 50.0
        P = 5000.0
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, w, 0.0, P])
        u = np.array([P, 0.0])
        # Mid-speed unsaturated: τ ≈ P/ω ≪ tau_sat
        self.assertLess(abs(P / w), sys.params["tau_sat"])
        dx = np.asarray(sys.f(x, u))
        # P_dot ≈ 0 when P_cmd = P
        np.testing.assert_allclose(dx[8], 0.0, atol=1e-09)

    def test_stall_torque_saturates_near_zero_omega(self):
        sys = BicycleDynEngine()
        tau_sat = sys.params["tau_sat"]
        P = 1e6
        x = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, P])
        u = np.array([P, 0.0])
        dx = np.asarray(sys.f(x, u))
        # ω=0, P>0 → τ = +τ_sat; brake≈0 → ω̇ = τ_sat / Jw
        np.testing.assert_allclose(dx[6], tau_sat / sys.params["Jw_rear"], rtol=1e-06)

    def test_params_override_engine_brake(self):
        sys = BicycleDynEngine()
        x = np.array([0.0, 0.0, 0.0, 10.0, 0.0, 0.0, 10.0 / 0.3, 0.0, 0.0])
        u = np.array([0.0, 0.0])
        params = {**sys.params, "bw_engine": 10.0 * sys.params["bw_engine"]}
        dx_nom = np.asarray(sys.f(x, u))
        dx_heavy = np.asarray(sys.f(x, u, params=params))
        self.assertFalse(np.allclose(dx_nom[6], dx_heavy[6]))


class TestCarProfile(unittest.TestCase):
    def test_registered_profiles(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            CAR_PROFILES,
            get_car_profile,
            list_car_profiles,
        )

        self.assertEqual(list_car_profiles(), ("passenger_car", "racecar", "udes_1_5"))
        for name in list_car_profiles():
            self.assertIs(get_car_profile(name), CAR_PROFILES[name])

    def test_racecar_matches_demo_vehicle_geometry(self):
        from examples.projects.car_trajopt.vehicles.car_profile import racecar_profile

        profile = racecar_profile()
        self.assertEqual(profile.mass, 700.0)
        self.assertEqual(profile.a, 1.2)
        self.assertEqual(profile.b, 1.0)
        self.assertEqual(profile.r_r, 0.34)
        self.assertEqual(profile.engine_power_peak, 100000.0)
        self.assertEqual(profile.v_nom, 10.0)
        self.assertLess(profile.v_nom, profile.limits.vx_max)
        self.assertAlmostEqual(profile.limits.delta_max, np.pi / 4.0, places=2)
        self.assertEqual(profile.limits.delta_dot_max, 3.0)
        self.assertEqual(profile.limits.w_rear_dot_max, 41.0)
        self.assertEqual(profile.limits.tau_rear_max, 3400.0)
        self.assertEqual(profile.limits.tau_rear_min, -3400.0)

    def test_propulsion_limits_from_power_at_nominal(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            get_car_profile,
            list_car_profiles,
        )

        for name in list_car_profiles():
            profile = get_car_profile(name)
            tau = profile.propulsion_torque_nominal()
            wdot = profile.propulsion_wheel_accel_nominal()
            if abs(tau) >= 100.0:
                expected_tau = round(tau, -1)
            else:
                expected_tau = round(tau, 1)
            self.assertEqual(profile.limits.tau_rear_max, expected_tau)
            self.assertEqual(profile.limits.tau_rear_min, -expected_tau)
            self.assertEqual(profile.limits.w_rear_dot_max, round(wdot))
            self.assertAlmostEqual(
                profile.limits.v_dot_max,
                round(profile.propulsion_longitudinal_accel_nominal(), 1),
            )
            self.assertEqual(profile.limits.a_long_max, profile.limits.v_dot_max)
            self.assertEqual(profile.limits.delta_dot_max, profile.steer_rate_max)
            self.assertLessEqual(profile.v_nom, profile.limits.vx_max)

    def test_actuator_limits_exceed_traction_reference(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            get_car_profile,
            list_car_profiles,
        )

        for name in list_car_profiles():
            profile = get_car_profile(name)
            if profile.propulsion_torque_nominal() > profile.traction_torque_max():
                self.assertGreater(
                    profile.limits.tau_rear_max,
                    profile.traction_torque_max(),
                    msg=name,
                )
                self.assertGreater(
                    profile.limits.w_rear_dot_max,
                    round(profile.traction_wheel_accel_reference()),
                    msg=name,
                )
                self.assertGreater(profile.actuator_traction_headroom(), 1.0, msg=name)

    def test_udes_matches_kinematic_racecar_geometry(self):
        from examples.projects.car_trajopt.vehicles.car_profile import udes_1_5_profile
        from minilink.catalog import UdeSRacecar

        udes = UdeSRacecar()
        profile = udes_1_5_profile()
        self.assertAlmostEqual(profile.a, udes.params["a"])
        self.assertAlmostEqual(profile.b, udes.params["b"])
        self.assertAlmostEqual(profile.length, udes.params["length"])

    def test_apply_car_profile_rate_plant(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            apply_car_profile,
            passenger_car_profile,
        )

        sys = BicycleDynRate()
        apply_car_profile(sys, passenger_car_profile())
        profile = passenger_car_profile()
        self.assertAlmostEqual(sys.params["mass"], profile.mass)
        self.assertAlmostEqual(sys.params["a"] + sys.params["b"], profile.a + profile.b)
        self.assertAlmostEqual(sys.params["Ca"], profile.Ca)
        self.assertAlmostEqual(sys.params["a"], profile.a)
        self.assertAlmostEqual(sys.params["b"], profile.b)
        self.assertFalse(hasattr(sys, "tire_model_f"))
        self.assertAlmostEqual(sys.state.upper_bound[6], profile.limits.w_rear_max)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.limits.w_rear_dot_max
        )
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[1], profile.limits.delta_dot_max
        )

    def test_apply_car_profile_servo_plant(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            apply_car_profile,
            racecar_profile,
        )

        sys = BicycleDynServo()
        apply_car_profile(sys, racecar_profile())
        profile = racecar_profile()
        self.assertAlmostEqual(sys.params["steering_tau"], profile.steering_tau)
        self.assertAlmostEqual(sys.params["torque_tau"], 0.05)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.limits.tau_rear_max
        )
        self.assertAlmostEqual(sys.inputs["u"].upper_bound[1], profile.limits.delta_max)

    def test_apply_car_profile_engine_plant(self):
        from examples.projects.car_trajopt.vehicles.car_profile import (
            apply_car_profile,
            racecar_profile,
        )

        sys = BicycleDynEngine()
        apply_car_profile(sys, racecar_profile())
        profile = racecar_profile()
        self.assertAlmostEqual(sys.params["engine_tau"], profile.engine_tau)
        self.assertAlmostEqual(sys.params["steering_tau"], profile.steering_tau)
        self.assertAlmostEqual(sys.params["tau_sat"], profile.tau_sat)
        self.assertAlmostEqual(sys.params["bw_engine"], profile.bw_engine)
        self.assertAlmostEqual(sys.params["tau_fric"], profile.tau_fric)
        self.assertNotIn("engine_power_peak", sys.params)
        self.assertNotIn("transmission_ratio", sys.params)
        self.assertAlmostEqual(
            sys.inputs["u"].upper_bound[0], profile.engine_power_peak
        )
        self.assertAlmostEqual(
            sys.inputs["u"].lower_bound[0], -profile.engine_power_peak
        )
        self.assertAlmostEqual(sys.inputs["u"].upper_bound[1], profile.limits.delta_max)
        self.assertAlmostEqual(sys.state.upper_bound[8], profile.engine_power_peak)
        self.assertAlmostEqual(sys.state.lower_bound[8], -profile.engine_power_peak)


# The pathtracking project's vehicle: the rolling resistance, the tires, the inputs


class TestPathtrackingVehicle(unittest.TestCase):
    def test_the_front_rolling_resistance_opposes_the_front_wheel_spin(self):
        vehicle = create_vehicle()
        vehicle.rolling_friction_constant = 0.015
        v = np.array(
            [0.0, 0.0, 0.0, 2.0, -2.0]
        )  # The rear spins forward, the front back
        u_in = np.array([0.0, 0.0])
        Fx_front, _, _, _ = vehicle.compute_tire_physics(v, u_in)
        Fz_f = vehicle.mass * vehicle.gravity * vehicle.b / vehicle.L
        d = vehicle.generalized_d(np.zeros(3), v, u_in)
        tau_front = vehicle.r_f * Fx_front + vehicle.bw_front * v[4] - 0.015 * Fz_f
        self.assertAlmostEqual(d[4], tau_front, places=9)

    def test_the_lateral_weight_is_one_without_longitudinal_slip(self):
        # G_yκ(κ = 0) = 1 whatever the curvature E (Pacejka 2006, §4.3.2)
        tire = Pacejka()
        tire.r_Ey1 = 0.5  # E = r_Ey1 + r_Ey2 d_fz = −0.5
        for alpha in (0.0, 0.1, -0.2):
            self.assertEqual(tire.Gyk(0.0, alpha), 1.0)
        self.assertLess(tire.Gyk(0.3, 0.1), 1.0)

    def test_the_rear_drive_bicycle_leaves_the_callers_input_alone(self):
        vehicle = RearDriveBicycle()
        vehicle.tire_model_f = LinearTire()
        vehicle.tire_model_r = LinearTire()
        x = np.zeros(8)
        x[3] = 5.0
        x[6] = 5.0 / vehicle.r_r
        x[7] = 5.0 / vehicle.r_f
        u = np.array([10.0, 2.0])  # A steer command past the limit
        dx = vehicle.f(x, u)
        np.testing.assert_array_equal(u, [10.0, 2.0])
        clipped = np.array([10.0, vehicle.max_steer])
        np.testing.assert_array_equal(dx, vehicle.f(x, clipped))

    def test_the_friction_circle_caps_each_contact_of_an_array(self):
        # One contact inside the circle, one beyond it: each as its own scalar call
        alpha = np.array([0.01, 0.5])
        kappa = np.array([0.005, 0.9])
        Fz = np.array([3000.0, 3000.0])
        for tire in (LinearTire(), Pacejka(combined_slip_mode="c")):
            with self.subTest(tire=type(tire).__name__):
                Fx, Fy = tire.slip2forces(alpha, kappa, Fz)
                for i in range(2):
                    Fx_i, Fy_i = tire.slip2forces(alpha[i], kappa[i], Fz[i])
                    self.assertEqual((Fx[i], Fy[i]), (Fx_i, Fy_i))
                self.assertLessEqual(
                    np.hypot(Fx[1], Fy[1]), tire.mu * Fz[1] * (1 + 1e-12)
                )


# The pathtracking project's path helpers: the point ahead along the path, both copies

PATH_MODULES = (
    "examples.projects.pathtracking.bicycle_los.los",
    "examples.projects.pathtracking.bicycle_los_v2.path_tracking",
)


@pytest.mark.parametrize("module", PATH_MODULES)
def test_the_point_ahead_starts_on_the_next_segment_after_a_degenerate_one(module):
    import importlib

    ahead = importlib.import_module(module).point_ahead_along_path
    for path in (
        [(0.0, 0.0), (0.0, 0.0), (4.0, 0.0), (4.0, 3.0)],
        [(0.0, 0.0), (0.0, 0.0), (0.0, 0.0), (4.0, 0.0), (4.0, 3.0)],
    ):
        ax, ay, i, lam, heading = ahead(path, 0, 0.7, 1.0, closed=False)
        assert (ax, ay, heading) == (1.0, 0.0, 0.0)
        assert i == len(path) - 3


@pytest.mark.parametrize("module", PATH_MODULES)
def test_an_open_path_stops_at_its_end_with_its_last_heading(module):
    import importlib

    ahead = importlib.import_module(module).point_ahead_along_path
    square = [(0.0, 0.0), (4.0, 0.0), (4.0, 3.0), (0.0, 3.0)]
    reached = ahead(square, 1, 0.2, 9.0, closed=False)
    assert reached == (0.0, 3.0, 2, 1.0, np.pi)
    assert [type(value) for value in reached] == [float, float, int, float, float]
    clamped = ahead(square, 3, 0.5, 1.0, closed=False)  # No closing segment
    assert clamped[:3] == (1.0, 3.0, 2)
    with pytest.raises(ValueError, match="non-negative"):
        ahead(square, 1, 0.2, -1.0, closed=False)
    wrapped = ahead(square, 0, 0.5, -1.0, closed=True)  # A loop wraps a negative ds
    assert [type(value) for value in wrapped] == [float, float, int, float, float]


def test_a_closed_rounded_rectangle_does_not_repeat_its_first_waypoint():
    from examples.projects.pathtracking.common.path_generator import (
        rounded_rectangle_path,
    )

    closed = rounded_rectangle_path(40.0, 20.0, 7.0, nseg=2, narc=4, closed=True)
    open_loop = rounded_rectangle_path(40.0, 20.0, 7.0, nseg=2, narc=4, closed=False)
    # The closing segment returns to the first waypoint: no zero-length segment
    assert np.hypot(*(closed[-1] - closed[0])) > 0.1
    # The open path runs the whole loop, back onto its first waypoint
    np.testing.assert_array_equal(open_loop[-1], open_loop[0])
    np.testing.assert_array_equal(open_loop[:-1], closed)


# The pathtracking project's signal blocks: one xp body each, so they trace under jit


@pytest.mark.jax
def test_the_pathtracking_signal_blocks_trace_under_jit():
    jax = pytest.importorskip("jax")
    configure_jax(enable_x64=True)
    from examples.projects.pathtracking.common.allocation import Allocation
    from examples.projects.pathtracking.mpc_v1.nominal_refs import NominalRefs
    from examples.projects.pathtracking.mpc_v1.plant_to_mpc import PlantToMpcState

    rng = np.random.default_rng(0)
    allocation = Allocation(0.34, 40000.0, transmission_ratio=2.0)
    nominal = NominalRefs()
    to_mpc = PlantToMpcState()
    cases = [
        (allocation.h_throttle, np.concatenate([[3000.0, 0.1], rng.normal(size=10)])),
        (
            allocation.h_throttle,
            np.concatenate([[-50.0, 0.1], 40.0 + rng.normal(size=10)]),
        ),
        (allocation.h_delta, np.concatenate([[3000.0, 0.1], rng.normal(size=10)])),
        (nominal.h_heading_ref, rng.normal(size=8)),
        (nominal.h_vx_ref, rng.normal(size=8)),
        (nominal.h_delta_ff, rng.normal(size=8)),
        (to_mpc.h_y_mpc, rng.normal(size=10)),
    ]
    for h, u in cases:
        expected = h(None, u)
        traced = jax.jit(lambda u, h=h: h(None, u))(u)
        np.testing.assert_array_equal(np.asarray(traced), expected)
