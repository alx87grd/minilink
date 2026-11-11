"""Tires: the slip of a wheel, and the laws that turn it into a force."""

from minilink.core.backends import array_module

###############################################################################
# Public API
###############################################################################


class LinearTire:
    """Linear tire: the force grows with the slip, with no friction limit.

    The stiffnesses are per unit load, so the force is linear in the load ``Fz``.
    It is the small-slip limit of every other law of this module.
    """

    def __init__(self):
        # Parameters: none of its own; the car hands over the stiffnesses
        self.params = {}

    def forces(self, kappa, alpha, Fz, mu, c_kappa, c_alpha, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        mu : float
            Friction coefficient [-] (no friction limit here: unused).
        c_kappa, c_alpha : float
            Longitudinal [-] and cornering [1/rad] stiffness per unit load.
        params : dict, optional
            The law's own numbers (none).
        """
        Fx = c_kappa * Fz * kappa
        Fy = c_alpha * Fz * alpha

        return Fx, Fy


class BrushTire:
    """Fiala brush tire: the friction limit, reached smoothly.

    The brush model integrates the bristle deflection over the contact patch: the
    force follows the linear tire at small slip and reaches ``mu Fz`` once the whole
    patch slides. Traction and cornering share one friction ellipse.

    References
    ----------
    Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., ch. 3; Fiala, VDI Zeitschrift 96 (1954).
    """

    def __init__(self):
        # Parameters: the floor that keeps the slip demand differentiable at zero slip
        self.params = {"z_floor": 1.0e-6}

    def forces(self, kappa, alpha, Fz, mu, c_kappa, c_alpha, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        mu : float
            Friction coefficient [-].
        c_kappa, c_alpha : float
            Longitudinal [-] and cornering [1/rad] stiffness per unit load.
        params : dict, optional
            Needs ``z_floor``.
        """
        params = self.params if params is None else params
        xp = array_module(kappa, alpha, Fz)

        z_floor = params["z_floor"]

        # Slip demand: the linear force over the friction limit, per direction
        s_x = c_kappa * kappa / mu
        s_y = c_alpha * alpha / mu
        z = xp.sqrt(s_x**2 + s_y**2 + z_floor**2)

        # F = μ Fz shape(z) along the demand, with shape(z) = z g(z) for the brush
        g = brush_saturation(z)
        Fx = mu * Fz * s_x * g
        Fy = mu * Fz * s_y * g

        return Fx, Fy


class MagicFormulaTire:
    """Magic formula tire (Pacejka): a peak at the friction limit, then a fall.

    The curve is taken along the slip demand ``z``, with ``B = c / (C mu)``, so its
    slope at zero slip is the stiffness ``c`` and its peak is ``mu Fz``. The shape
    ``mf_C`` sets the fall past the peak, to ``sin(mf_C pi / 2)`` of it at large
    slip, and the curvature ``mf_E`` where the peak sits.

    References
    ----------
    Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., ch. 4.
    """

    def __init__(self):
        # Parameters: the shape and the curvature of the curve, and the slip floor
        self.params = {"mf_C": 1.3, "mf_E": -0.5, "z_floor": 1.0e-6}

    def forces(self, kappa, alpha, Fz, mu, c_kappa, c_alpha, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        mu : float
            Friction coefficient [-].
        c_kappa, c_alpha : float
            Longitudinal [-] and cornering [1/rad] stiffness per unit load.
        params : dict, optional
            Needs ``mf_C``, ``mf_E`` and ``z_floor``.
        """
        params = self.params if params is None else params
        xp = array_module(kappa, alpha, Fz)

        C = params["mf_C"]
        E = params["mf_E"]
        z_floor = params["z_floor"]

        # Slip demand: the linear force over the friction limit, per direction
        s_x = c_kappa * kappa / mu
        s_y = c_alpha * alpha / mu
        z = xp.sqrt(s_x**2 + s_y**2 + z_floor**2)

        # shape(z) = sin(C atan(b − E (b − atan b))), with b = B slip = z / C
        b = z / C
        shape = xp.sin(C * xp.arctan(b - E * (b - xp.arctan(b))))

        # F = μ Fz shape(z) along the demand
        Fx = mu * Fz * s_x * shape / z
        Fy = mu * Fz * s_y * shape / z

        return Fx, Fy


class LinearSaturation:
    """Linear tire scaled back onto its friction circle ``|F| <= mu Fz``.

    The stiffnesses here are absolute: ``c_kappa`` receives ``Ck`` [N] and
    ``c_alpha`` receives ``Ca`` [N/rad], shared by both axles, and the force is not
    multiplied by ``Fz``. The law of ``DynamicBicycle`` and ``BicycleDynRate``
    with their default tire.
    """

    def __init__(self):
        # Parameters: none of its own; the car hands over ``Ca``, ``Ck`` and ``mu``
        self.params = {}

    def forces(self, kappa, alpha, Fz, mu, c_kappa, c_alpha, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        mu : float
            Friction coefficient [-].
        c_kappa, c_alpha : float
            Absolute longitudinal [N] and cornering [N/rad] stiffness.
        params : dict, optional
            The law's own numbers (none).
        """
        xp = array_module(kappa, alpha)

        # Linear slip forces: F_x = C_κ κ and F_y = C_α α
        Fx = c_kappa * kappa
        Fy = c_alpha * alpha

        # Scaled back onto the friction circle |F| ≤ μ F_z (10⁻¹² guards the division)
        F_max = mu * Fz
        F_total = xp.sqrt(Fx**2 + Fy**2)
        ratio = xp.where(F_total > F_max, F_max / xp.maximum(F_total, 1e-12), 1.0)
        Fx_sat = Fx * ratio
        Fy_sat = Fy * ratio

        return Fx_sat, Fy_sat


# The slip of one wheel, from its contact velocity in its own frame


def tire_slip(vx, vy, w, R, v_min_epsilon):
    """Slip angle ``alpha`` [rad] and slip ratio ``kappa`` [-] at one contact.

    The rolling speed is floored as ``|vx| + v_min_epsilon``.
    """
    xp = array_module(vx, vy, w)

    # The regularized speed |vx| + ε keeps both slips finite at rest
    vx_adj = xp.abs(vx) + v_min_epsilon

    # α = −atan(vy / (|vx| + ε)) and κ = (ω R − vx) / (|vx| + ε)
    alpha = -xp.arctan(vy / vx_adj)
    kappa = (w * R - vx) / vx_adj

    return alpha, kappa


def wheel_slip(vx, vy, w, R, v_min_epsilon):
    """Slip angle ``alpha`` [rad] and slip ratio ``kappa`` [-] at one contact.

    The rolling speed is floored as ``sqrt(vx^2 + v_min_epsilon^2)``, smooth at rest.

    Parameters
    ----------
    vx, vy : float or array
        Contact-point velocity in the wheel frame [m/s].
    w : float or array
        Wheel angular rate [rad/s], positive when rolling forward.
    R : float
        Wheel radius [m].
    v_min_epsilon : float
        Floor of the rolling speed [m/s].

    Returns
    -------
    alpha : float or array
        Slip angle [rad]: positive when the tire pushes the car to the left.
    kappa : float or array
        Slip ratio [-]: positive when the tire drives the car forward.
    """
    xp = array_module(vx, vy, w)

    # The rolling speed |vx|, floored to √(vx² + ε²) near rest
    v_roll = xp.sqrt(vx**2 + v_min_epsilon**2)

    # α = −atan(vy / |vx|) and κ = (ω R − vx) / |vx|
    alpha = -xp.arctan(vy / v_roll)
    kappa = (w * R - vx) / v_roll

    return alpha, kappa


def axle_slips(vx, vy, yaw_rate, w_rear, delta, params):
    """Slip ratio of the driven rear axle and slip angle of each axle.

    Parameters
    ----------
    vx, vy : float
        Body-frame velocity of the centre of gravity [m/s].
    yaw_rate : float
        Body yaw rate [rad/s].
    w_rear : float
        Rear wheel angular rate [rad/s], positive when rolling forward.
    delta : float
        Front steer angle [rad].
    params : dict
        Needs ``a``, ``b`` [m], ``r_r`` [m] and ``v_min_epsilon`` [m/s].

    Returns
    -------
    kappa_r : float
        Rear slip ratio [-]: positive when the tire drives the car forward.
    alpha_f, alpha_r : float
        Slip angles [rad]: positive when the tire pushes the car to the left.

    Notes
    -----
    Every denominator is floored by ``v_min_epsilon`` (:func:`wheel_slip`), so the
    slips stay bounded and differentiable at standstill instead of blowing up as
    ``1 / vx``.
    """
    xp = array_module(vx, vy, yaw_rate, w_rear, delta)

    a = params["a"]
    b = params["b"]
    r_r = params["r_r"]
    v_eps = params["v_min_epsilon"]

    c_d = xp.cos(delta)
    s_d = xp.sin(delta)

    # Contact-point velocities, the front pair rotated into the steered wheel frame:
    # [vx_f; vy_f] = R(δ)ᵀ [vx; vy + a ψ̇]
    vy_axle_f = vy + a * yaw_rate
    vx_f = vx * c_d + vy_axle_f * s_d
    vy_f = -vx * s_d + vy_axle_f * c_d
    vy_r = vy - b * yaw_rate

    # The slips of each wheel; the front one rolls free, so only its angle counts
    alpha_f, _ = wheel_slip(vx_f, vy_f, 0.0, r_r, v_eps)
    alpha_r, kappa_r = wheel_slip(vx, vy_r, w_rear, r_r, v_eps)

    return kappa_r, alpha_f, alpha_r


# The transitional law at one contact, its slip computed inside


def linear_tire_forces(vx, vy, w, R, Fz, Ca, Ck, mu, v_min_epsilon):
    """Linear-slip tire forces with friction-circle saturation (pyro ``LinearTire``).

    ``Fx = Ck kappa`` and ``Fy = Ca alpha``, scaled back onto ``|F| <= mu Fz``.
    """
    xp = array_module(vx, vy, w)

    alpha, kappa = tire_slip(vx, vy, w, R, v_min_epsilon)

    # Linear slip forces: F_x = C_κ κ and F_y = C_α α
    Fx = Ck * kappa
    Fy = Ca * alpha

    # Scaled back onto the friction circle |F| ≤ μ F_z (10⁻¹² guards the division)
    F_max = mu * Fz
    F_total = xp.sqrt(Fx**2 + Fy**2)
    ratio = xp.where(F_total > F_max, F_max / xp.maximum(F_total, 1e-12), 1.0)
    Fx_sat = Fx * ratio
    Fy_sat = Fy * ratio

    return Fx_sat, Fy_sat


# The friction an axle uses


def friction_use(Fx_f, Fy_f, Fx_r, Fy_r, params):
    """Used fraction ``(phi_f, phi_r)`` of each axle's friction ellipse [-].

    ``1`` means the axle is sliding: it has no force left for anything else.
    """
    xp = array_module(Fx_f, Fy_f, Fx_r, Fy_r)

    mass = params["mass"]
    gravity = params["gravity"]
    a = params["a"]
    b = params["b"]
    mu = params["mu"]

    # Static axle loads
    Fz_f = mass * gravity * b / (a + b)
    Fz_r = mass * gravity * a / (a + b)

    # φ = √(Fx² + Fy²) / (μ Fz), the radius in the normalized force plane (the front's
    # Fx is rolling resistance only)
    phi_f = xp.sqrt(Fx_f**2 + Fy_f**2) / (mu * Fz_f)
    phi_r = xp.sqrt(Fx_r**2 + Fy_r**2) / (mu * Fz_r)

    return phi_f, phi_r


# The building blocks of the laws


def smooth_sign(v, v_c):
    """Sign of ``v`` softened over the creep speed ``v_c``, for a resistance at rest."""
    xp = array_module(v)

    # sign(v) ≈ tanh(v / v_c): ±1 beyond a few v_c, and differentiable at v = 0
    s = xp.tanh(v / v_c)

    return s


def smooth_floor(F, F_eps):
    """``max(F, 0)`` softened over ``F_eps``: a normal load that lifts off smoothly.

    Parameters
    ----------
    F : float or array
        The load before the floor [N]: negative when the wheel would pull the ground.
    F_eps : float
        The width of the softened corner [N].
    """
    xp = array_module(F)

    # max(F, 0) ≈ ½ (F + √(F² + F_eps²)): F well above F_eps, 0 well below, smooth at 0
    F_floor = 0.5 * (F + xp.sqrt(F**2 + F_eps**2))

    return F_floor


def brush_saturation(z):
    """Saturation factor ``g(z) = phi(z) / z`` applied to the linear tire force.

    The brush model integrates the bristle deflection over the contact patch. While part
    of the patch still grips, the force is ``phi(z) = 1 - (1 - z/3)^3`` of the friction
    limit; past ``z = 3`` the whole patch slides and the force stays at the limit, so the
    factor decays as ``1 / z``.

    Parameters
    ----------
    z : float or array
        Slip demand [-]: the linear-slip force divided by the friction limit ``mu Fz``.

    Returns
    -------
    g : float or array
        Factor in ``(0, 1]``. ``g(0) = 1`` (linear tire) and ``z g(z) <= 1`` always, so
        the force never exceeds ``mu Fz``. The two branches match in value and slope at
        ``z = 3`` (both give ``1/3`` and ``-1/9``).
    """
    xp = array_module(z)

    # g = φ(z) / z, with φ(z) = 1 − (1 − z/3)³ while the patch grips and 1 once it all
    # slides; the grip branch is expanded to a polynomial (no 0/0 at z = 0), and the
    # slide branch floored at z = 3, since both branches are evaluated
    g = xp.where(z < 3.0, 1.0 - z / 3.0 + z**2 / 27.0, 1.0 / xp.maximum(z, 3.0))

    return g
