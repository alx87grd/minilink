"""Fiala brush tires: the axle slips, their saturated forces and the friction ellipse."""

from minilink.core.backends import array_module

###############################################################################
# Public API
###############################################################################


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
    Every denominator is floored by ``v_min_epsilon``, so the slips stay bounded and
    differentiable at standstill instead of blowing up as ``1 / vx``.
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

    # κ = (r ω − vx) / |vx| and α = −arctan(vy / |vx|): the slip velocity over the
    # rolling speed, with |vx| floored to √(vx² + ε²) near rest
    kappa_r = (r_r * w_rear - vx) / xp.sqrt(vx**2 + v_eps**2)
    alpha_f = -xp.arctan(vy_f / xp.sqrt(vx_f**2 + v_eps**2))
    alpha_r = -xp.arctan(vy_r / xp.sqrt(vx**2 + v_eps**2))

    return kappa_r, alpha_f, alpha_r


def brush_tire_forces(kappa_r, alpha_f, alpha_r, vx, params):
    """Axle forces ``(Fx_f, Fy_f, Fx_r, Fy_r)`` [N], each in its own wheel frame.

    Static axle loads carry the weight. The free-rolling front axle makes a cornering
    force and pays rolling resistance; the driven rear axle shares one friction ellipse
    between its traction force and its cornering force, so asking for both at once costs
    grip in the other direction.

    ``C_rr`` is charged to the front axle only: what the rear wheels lose to rolling is
    carried by the drivetrain terms of the plant (``bw_drive``, ``tau_fric``), which
    resist the wheel rate itself rather than the contact patch.

    Parameters
    ----------
    kappa_r, alpha_f, alpha_r : float
        Slips from :func:`axle_slips`.
    vx : float
        Body-frame forward speed [m/s], for the direction of the rolling resistance.
    params : dict
        Needs ``mass``, ``gravity``, ``a``, ``b``, ``mu``, ``c_alpha_f``, ``c_alpha_r``,
        ``c_kappa``, ``C_rr``, ``v_c`` and ``z_floor``.

    Returns
    -------
    Fx_f, Fy_f, Fx_r, Fy_r : float
        Longitudinal and lateral force of the front and of the rear axle [N].

    Notes
    -----
    Axle stiffnesses are given per unit load (``c_alpha`` in 1/rad, ``c_kappa`` in [-]),
    the usual way to write a tire whose stiffness scales with what it carries. The
    understeer gradient of the car is then ``1/c_alpha_f - 1/c_alpha_r`` rad per g,
    independent of the mass split.

    References
    ----------
    Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., ch. 3; Fiala, VDI Zeitschrift 96 (1954).
    """
    xp = array_module(kappa_r, alpha_f, alpha_r, vx)

    mass = params["mass"]
    gravity = params["gravity"]
    a = params["a"]
    b = params["b"]
    mu = params["mu"]
    c_alpha_f = params["c_alpha_f"]
    c_alpha_r = params["c_alpha_r"]
    c_kappa = params["c_kappa"]
    C_rr = params["C_rr"]
    v_c = params["v_c"]
    z_floor = params["z_floor"]

    # Static axle loads, and the stiffnesses they carry
    Fz_f = mass * gravity * b / (a + b)
    Fz_r = mass * gravity * a / (a + b)
    C_alpha_f = c_alpha_f * Fz_f
    C_alpha_r = c_alpha_r * Fz_r
    C_kappa = c_kappa * Fz_r

    # Slip demand z = √(s_x² + s_y² + z_floor²), with s = C · slip / (μ Fz) the
    # linear-slip force over the friction limit: traction and cornering share one
    # ellipse at the rear, and the floor keeps the root differentiable at zero slip
    z_f = xp.sqrt((C_alpha_f * alpha_f / (mu * Fz_f)) ** 2 + z_floor**2)
    s_x = C_kappa * kappa_r / (mu * Fz_r)
    s_y = C_alpha_r * alpha_r / (mu * Fz_r)
    z_r = xp.sqrt(s_x**2 + s_y**2 + z_floor**2)

    # F = C · slip · g(z), the linear tire force saturated by the brush model; the
    # free-rolling front carries only the rolling resistance −C_rr Fz_f sign(vx)
    Fx_f = -C_rr * Fz_f * smooth_sign(vx, v_c)
    Fy_f = C_alpha_f * alpha_f * brush_saturation(z_f)
    Fx_r = C_kappa * kappa_r * brush_saturation(z_r)
    Fy_r = C_alpha_r * alpha_r * brush_saturation(z_r)

    return Fx_f, Fy_f, Fx_r, Fy_r


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


# The building blocks of the brush law


def smooth_sign(v, v_c):
    """Sign of ``v`` softened over the creep speed ``v_c``, for a resistance at rest."""
    xp = array_module(v)

    # sign(v) ≈ tanh(v / v_c): ±1 beyond a few v_c, and differentiable at v = 0
    s = xp.tanh(v / v_c)

    return s


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
