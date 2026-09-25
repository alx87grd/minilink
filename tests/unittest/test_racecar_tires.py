"""Contract of the brush tire: shape of the saturation, friction ellipse, both backends."""

import numpy as np
import pytest

from minilink.dynamics.catalog.vehicles.racecar import PUBLIC_RACECAR_PARAMS
from minilink.dynamics.catalog.vehicles.tires import (
    axle_slips,
    brush_saturation,
    brush_tire_forces,
    smooth_sign,
)

PARAMS = dict(PUBLIC_RACECAR_PARAMS)
L = PARAMS["a"] + PARAMS["b"]
FZ_F = PARAMS["mass"] * PARAMS["gravity"] * PARAMS["b"] / L
FZ_R = PARAMS["mass"] * PARAMS["gravity"] * PARAMS["a"] / L


def test_saturation_is_one_at_zero_slip():
    """A tire at rest against its bristles is the linear tire: g(0) = 1."""
    assert brush_saturation(0.0) == pytest.approx(1.0)


def test_saturation_is_c1_at_the_sliding_threshold():
    """Value and slope match at z = 3, where the contact patch finishes sliding."""
    z = 3.0
    step = 1.0e-6

    left = brush_saturation(z - step)
    right = brush_saturation(z + step)
    slope_left = (brush_saturation(z - step) - brush_saturation(z - 2 * step)) / step
    slope_right = (brush_saturation(z + 2 * step) - brush_saturation(z + step)) / step

    assert left == pytest.approx(right, abs=1e-6)
    assert brush_saturation(z) == pytest.approx(1.0 / 3.0)
    assert slope_left == pytest.approx(slope_right, abs=1e-5)
    assert slope_left == pytest.approx(-1.0 / 9.0, abs=1e-4)


def test_saturation_never_lets_the_force_pass_the_limit():
    """``z g(z) <= 1``: the normalized force is bounded by the friction limit."""
    z = np.linspace(0.0, 50.0, 2001)

    force = z * brush_saturation(z)

    assert force.max() <= 1.0 + 1e-12
    assert force[-1] == pytest.approx(1.0)


def test_pure_lateral_force_saturates_at_mu_fz():
    """A front tire at a huge slip angle carries exactly ``mu Fz``."""
    _, Fy_f, _, _ = brush_tire_forces(0.0, 1.5, 0.0, 5.0, PARAMS)

    assert Fy_f == pytest.approx(PARAMS["mu"] * FZ_F, rel=1e-3)


def test_combined_slip_stays_inside_the_friction_ellipse():
    """Traction and cornering share one circle at the driven axle."""
    kappa = np.linspace(-2.0, 2.0, 41)
    alpha = np.linspace(-1.0, 1.0, 41)
    K, A = np.meshgrid(kappa, alpha)

    _, _, Fx_r, Fy_r = brush_tire_forces(K, 0.0 * K, A, 5.0, PARAMS)

    assert np.max(np.hypot(Fx_r, Fy_r)) <= PARAMS["mu"] * FZ_R * (1.0 + 1e-9)


def test_small_slip_is_the_linear_tire():
    """Below saturation the axle stiffnesses are ``c_alpha Fz`` and ``c_kappa Fz``."""
    kappa, alpha = 1.0e-4, 1.0e-4

    _, Fy_f, Fx_r, Fy_r = brush_tire_forces(kappa, alpha, alpha, 5.0, PARAMS)

    assert Fy_f == pytest.approx(PARAMS["c_alpha_f"] * FZ_F * alpha, rel=1e-3)
    assert Fy_r == pytest.approx(PARAMS["c_alpha_r"] * FZ_R * alpha, rel=1e-3)
    assert Fx_r == pytest.approx(PARAMS["c_kappa"] * FZ_R * kappa, rel=1e-3)


def test_force_signs_follow_the_slips():
    """A tire pushes the car the way its slip says, and resists its rolling."""
    _, Fy_f, Fx_r, Fy_r = brush_tire_forces(0.05, 0.05, 0.05, 5.0, PARAMS)
    Fx_f_forward, _, _, _ = brush_tire_forces(0.0, 0.0, 0.0, 5.0, PARAMS)
    Fx_f_backward, _, _, _ = brush_tire_forces(0.0, 0.0, 0.0, -5.0, PARAMS)

    assert Fy_f > 0.0 and Fy_r > 0.0 and Fx_r > 0.0
    assert Fx_f_forward < 0.0 < Fx_f_backward


def test_slips_vanish_when_the_wheel_rolls_free_and_straight():
    """Rolling straight at the matched wheel rate is the no-slip condition."""
    vx = 4.0

    kappa_r, alpha_f, alpha_r = axle_slips(
        vx, 0.0, 0.0, vx / PARAMS["r_r"], 0.0, PARAMS
    )

    assert kappa_r == pytest.approx(0.0)
    assert alpha_f == pytest.approx(0.0)
    assert alpha_r == pytest.approx(0.0)


def test_slips_have_the_expected_signs():
    """Overspeeding the wheel drives; sliding to the right makes a left-pushing force."""
    vx = 4.0

    kappa_fast, _, _ = axle_slips(vx, 0.0, 0.0, 1.2 * vx / PARAMS["r_r"], 0.0, PARAMS)
    _, alpha_f, alpha_r = axle_slips(vx, -0.4, 0.0, vx / PARAMS["r_r"], 0.0, PARAMS)

    assert kappa_fast > 0.0
    assert alpha_f > 0.0 and alpha_r > 0.0


def test_smooth_sign_is_bounded_and_odd():
    """The creep sign saturates at one and changes sign with the speed."""
    v = np.linspace(-1.0, 1.0, 101)

    s = smooth_sign(v, PARAMS["v_c"])

    assert np.all(np.abs(s) <= 1.0)
    assert smooth_sign(0.0, PARAMS["v_c"]) == pytest.approx(0.0)
    assert s[0] == pytest.approx(-s[-1])


@pytest.mark.jax
def test_numpy_and_jax_agree():
    """The same lines give the same numbers on both backends."""
    jax = pytest.importorskip("jax")
    jnp = jax.numpy
    args = (0.08, 0.12, 0.05, 3.0)

    on_numpy = np.array(brush_tire_forces(*args, PARAMS))
    on_jax = np.array(
        brush_tire_forces(*[jnp.asarray(value) for value in args], PARAMS)
    )

    assert on_jax == pytest.approx(on_numpy, rel=1e-6)


@pytest.mark.jax
def test_gradients_are_finite_at_zero_slip():
    """``z_floor`` keeps the slip demand differentiable where every slip is zero."""
    jax = pytest.importorskip("jax")
    jnp = jax.numpy
    grad = jax.jacfwd(
        lambda slips: jnp.array(
            brush_tire_forces(slips[0], slips[1], slips[2], 3.0, PARAMS)
        )
    )

    jacobian = np.array(grad(jnp.zeros(3)))

    assert np.all(np.isfinite(jacobian))
    assert jacobian[1, 1] == pytest.approx(PARAMS["c_alpha_f"] * FZ_F, rel=1e-6)
