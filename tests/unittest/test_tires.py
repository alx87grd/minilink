"""Contract of the tire laws: the slope at zero slip, the friction limit, both backends."""

import numpy as np
import pytest

from minilink.core.backends import configure_jax
from minilink.dynamics.catalog.vehicles.tires import (
    BrushTire,
    LinearSaturation,
    LinearTire,
    MagicFormulaTire,
    smooth_floor,
    tire_slip,
    wheel_slip,
)

MU = 0.9
FZ = 1200.0
C_KAPPA = 10.0
C_ALPHA = 13.0

LAWS = [LinearTire, BrushTire, MagicFormulaTire]
SATURATING = [BrushTire, MagicFormulaTire]


def make(law):
    """The law on this module's numbers."""
    return law(mu=MU, c_alpha=C_ALPHA, c_kappa=C_KAPPA)


def forces(law, kappa, alpha, Fz=FZ):
    """The law's forces on this module's numbers."""
    Fx, Fy = make(law).forces(kappa, alpha, Fz)

    return np.asarray(Fx), np.asarray(Fy)


@pytest.mark.parametrize("law", LAWS, ids=lambda law: law.__name__)
def test_the_slope_at_zero_slip_is_the_stiffness(law):
    """``dF/d(slip) = c Fz`` at zero slip, in both directions."""
    slip = 1.0e-7

    Fx, _ = forces(law, slip, 0.0)
    _, Fy = forces(law, 0.0, slip)

    assert Fx / slip == pytest.approx(C_KAPPA * FZ, rel=1e-6)
    assert Fy / slip == pytest.approx(C_ALPHA * FZ, rel=1e-6)


@pytest.mark.parametrize("law", SATURATING, ids=lambda law: law.__name__)
def test_the_force_never_passes_the_friction_limit(law):
    """``|F| <= mu Fz``, for a pure slip and for combined slips alike."""
    kappa = np.linspace(-3.0, 3.0, 121)
    alpha = np.linspace(-1.5, 1.5, 121)
    K, A = np.meshgrid(kappa, alpha)

    Fx, Fy = forces(law, K, A)
    Fx_pure, _ = forces(law, kappa, 0.0 * kappa)
    _, Fy_pure = forces(law, 0.0 * alpha, alpha)

    limit = MU * FZ * (1.0 + 1e-12)
    assert np.max(np.hypot(Fx, Fy)) <= limit
    assert np.max(np.abs(Fx_pure)) <= limit
    assert np.max(np.abs(Fy_pure)) <= limit


def test_the_three_laws_agree_at_small_slip():
    """Up to a slip demand of 0.3 the laws differ by less than 5 % of ``mu Fz``."""
    z = np.linspace(0.0, 0.3, 61)
    alpha = z * MU / C_ALPHA

    lateral = [forces(law, 0.0 * alpha, alpha)[1] for law in LAWS]

    for i in range(3):
        for j in range(i + 1, 3):
            assert np.max(np.abs(lateral[i] - lateral[j])) < 0.05 * MU * FZ


def test_the_magic_formula_peaks_at_the_friction_limit():
    """The curve rises to ``mu Fz`` and falls past it, towards ``sin(C pi / 2)`` of it."""
    tire = make(MagicFormulaTire)
    alpha = np.linspace(0.0, 3.0, 30001)

    _, Fy = forces(MagicFormulaTire, 0.0 * alpha, alpha)

    assert np.max(Fy) == pytest.approx(MU * FZ, rel=1e-6)
    assert Fy[-1] < 0.99 * np.max(Fy)
    assert Fy[-1] > np.sin(tire.params["C"] * np.pi / 2.0) * MU * FZ


def test_a_force_points_along_the_slip_demand():
    """Combined slip: the force is the linear force scaled down, its direction kept."""
    kappa, alpha = 0.2, 0.15

    for law in SATURATING:
        Fx, Fy = forces(law, kappa, alpha)
        assert Fy / Fx == pytest.approx((C_ALPHA * alpha) / (C_KAPPA * kappa))


def saturated_linear_forces(vx, vy, w, R, Fz, Ca, Ck, mu, v_min_epsilon):
    """The saturated linear tire written out at one contact: the law's reference."""
    kappa, alpha = tire_slip(vx, vy, w, R, v_min_epsilon)
    Fx = Ck * kappa
    Fy = Ca * alpha
    F_max = mu * Fz
    F_total = np.sqrt(Fx**2 + Fy**2)
    ratio = np.where(F_total > F_max, F_max / np.maximum(F_total, 1e-12), 1.0)

    return Fx * ratio, Fy * ratio


def test_linear_saturation_is_its_reference_bit_for_bit():
    """The law gives the reference's bits: linear inside the circle, on it beyond."""
    rng = np.random.default_rng(3)
    vx = 10.0 * rng.standard_normal(500)
    vy = 2.0 * rng.standard_normal(500)
    w = 30.0 * rng.standard_normal(500)
    R, Fz, Ca, Ck, mu, v_eps = 0.3, 7357.5, 60000.0, 100000.0, 1.0, 0.1

    tire = LinearSaturation(mu=mu, Ca=Ca, Ck=Ck, v_eps=v_eps)
    kappa, alpha = tire.slip(vx, vy, w, R)
    Fx, Fy = tire.forces(kappa, alpha, Fz)
    Fx_ref, Fy_ref = saturated_linear_forces(vx, vy, w, R, Fz, Ca, Ck, mu, v_eps)

    np.testing.assert_array_equal(Fx, Fx_ref)
    np.testing.assert_array_equal(Fy, Fy_ref)


def test_the_slips_vanish_when_the_wheel_rolls_free_and_straight():
    """Rolling straight at the matched rate: no slip for either slip function."""
    vx, R = 4.0, 0.3

    for slip in (tire_slip, wheel_slip):
        kappa, alpha = slip(vx, 0.0, vx / R, R, 0.1)
        assert alpha == pytest.approx(0.0)
        assert kappa == pytest.approx(0.0)


def test_the_smooth_slip_is_finite_and_smooth_at_rest():
    """At standstill the rolling speed sits on its floor ``v_min_epsilon``."""
    kappa, alpha = wheel_slip(0.0, -0.05, 1.0, 0.3, 0.1)

    assert alpha == pytest.approx(np.arctan(0.5))
    assert kappa == pytest.approx(3.0)


def test_the_smooth_floor_lifts_off_smoothly():
    """``max(F, 0)`` away from zero, ``F_eps / 2`` at zero."""
    F_eps = 1.0

    assert smooth_floor(1.0e3, F_eps) == pytest.approx(1.0e3)
    assert smooth_floor(-1.0e3, F_eps) == pytest.approx(0.0, abs=1e-3)
    assert smooth_floor(0.0, F_eps) == pytest.approx(0.5 * F_eps)


@pytest.mark.jax
@pytest.mark.parametrize("law", LAWS + [LinearSaturation], ids=lambda law: law.__name__)
def test_numpy_and_jax_agree(law):
    """The same lines give the same numbers on both backends."""
    jax = pytest.importorskip("jax")
    configure_jax(enable_x64=True)
    jnp = jax.numpy
    tire = law()
    args = (0.08, 0.12, FZ, 0.8)

    on_numpy = np.array(tire.forces(*args))
    on_jax = np.array(tire.forces(*[jnp.asarray(value) for value in args]))

    assert on_jax == pytest.approx(on_numpy, rel=1e-12)


@pytest.mark.jax
@pytest.mark.parametrize("law", LAWS, ids=lambda law: law.__name__)
def test_gradients_are_finite_at_zero_slip(law):
    """The floor of the slip demand keeps every law differentiable at zero slip."""
    jax = pytest.importorskip("jax")
    configure_jax(enable_x64=True)
    jnp = jax.numpy
    tire = make(law)
    jacobian_of = jax.jacfwd(
        lambda slips: jnp.array(tire.forces(slips[0], slips[1], FZ))
    )

    jacobian = np.array(jacobian_of(jnp.zeros(2)))

    assert np.all(np.isfinite(jacobian))
    assert jacobian[0, 0] == pytest.approx(C_KAPPA * FZ, rel=1e-6)
    assert jacobian[1, 1] == pytest.approx(C_ALPHA * FZ, rel=1e-6)


@pytest.mark.parametrize("law", LAWS + [LinearSaturation], ids=lambda law: law.__name__)
def test_a_tire_owns_its_numbers_and_reads_the_tree_it_is_given(law):
    """``params=None`` reads the tire's own dict; a tree given is read instead."""
    tire = law()
    given = {**tire.params, "mu": 0.5}
    kappa, alpha = tire.slip(5.0, 0.4, 5.0 / 0.3 + 2.0, 0.3)

    own = np.array(tire.forces(kappa, alpha, FZ))
    from_tree = np.array(tire.forces(kappa, alpha, FZ, params=given))
    on_a_wet_road = np.array(tire.forces(kappa, alpha, FZ, friction=0.5))

    if law is not LinearTire:  # The linear tire has no friction limit
        assert not np.allclose(own, from_tree)
    np.testing.assert_array_equal(from_tree, on_a_wet_road)


@pytest.mark.parametrize("law", LAWS + [LinearSaturation], ids=lambda law: law.__name__)
def test_a_tire_params_assignment_keeps_its_dict(law):
    """Assigning a tire's params refills its own dict, so a car holding it stays current."""
    tire = law()
    held = tire.params

    tire.params = {**tire.params, "mu": 0.7}

    assert tire.params is held
    assert held["mu"] == 0.7
