"""Tires: the slip of a wheel, and the laws that turn it into a force."""

from minilink.core.backends import array_module
from minilink.core.params import own_params

#: The floor of the slip demand: keeps √(s_x² + s_y²) differentiable at zero slip
SLIP_DEMAND_FLOOR = 1.0e-6

###############################################################################
# Public API
###############################################################################


class LinearTire:
    """Linear tire: the force grows with the slip, with no friction limit.

    A tire is a part of a car: its law, its slip and its numbers. The stiffnesses
    are per unit load, so the force is linear in the load ``Fz``; it is the
    small-slip limit of every other law of this module. ``mu`` bounds nothing here:
    it serves the used-grip output only.
    """

    params = own_params()

    def __init__(self, mu=1.0, c_alpha=8.2, c_kappa=13.6, v_eps=0.5):
        # Parameters: the grip on the reference surface, the stiffnesses per unit
        # load and the floor of the slip denominators
        self.params = {"mu": mu, "c_alpha": c_alpha, "c_kappa": c_kappa, "v_eps": v_eps}

    def slip(self, vx, vy, w, R, params=None):
        """Slip ratio ``kappa`` [-] and slip angle ``alpha`` [rad] (:func:`wheel_slip`)."""
        params = self.params if params is None else params

        v_eps = params["v_eps"]

        kappa, alpha = wheel_slip(vx, vy, w, R, v_eps)

        return kappa, alpha

    def stiffnesses(self, Fz, params=None):
        """Small-slip stiffnesses ``(C_kappa, C_alpha)`` [N, N/rad] under the load ``Fz``."""
        params = self.params if params is None else params

        c_kappa = params["c_kappa"]
        c_alpha = params["c_alpha"]

        C_kappa = c_kappa * Fz
        C_alpha = c_alpha * Fz

        return C_kappa, C_alpha

    def forces(self, kappa, alpha, Fz, friction=1.0, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        friction : float or array
            The road's friction factor under the wheel [-], 1 on the reference
            surface (unused: no friction limit).
        params : dict, optional
            Needs ``c_alpha``, ``c_kappa``.
        """
        params = self.params if params is None else params

        c_alpha = params["c_alpha"]
        c_kappa = params["c_kappa"]

        Fx = c_kappa * Fz * kappa
        Fy = c_alpha * Fz * alpha

        return Fx, Fy

    def friction_use(self, Fx, Fy, Fz, friction=1.0, params=None):
        """Used fraction of the friction ellipse [-] (:func:`used_grip`)."""
        params = self.params if params is None else params

        mu = friction * params["mu"]

        phi = used_grip(Fx, Fy, Fz, mu)

        return phi


class BrushTire:
    """Fiala brush tire: the friction limit, reached smoothly.

    The brush model integrates the bristle deflection over the contact patch: the
    force follows the linear tire at small slip and reaches ``mu Fz`` once the whole
    patch slides. Traction and cornering share one friction ellipse. On a bicycle
    model each axle's tire is an effective one: its stiffnesses lump the axle's two
    tires and its steering compliance.

    References
    ----------
    Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., ch. 3; Fiala, VDI Zeitschrift 96 (1954).
    """

    params = own_params()

    def __init__(self, mu=1.0, c_alpha=8.2, c_kappa=13.6, v_eps=0.5):
        # Parameters: the grip on the reference surface, the stiffnesses per unit
        # load and the floor of the slip denominators
        self.params = {"mu": mu, "c_alpha": c_alpha, "c_kappa": c_kappa, "v_eps": v_eps}

    def slip(self, vx, vy, w, R, params=None):
        """Slip ratio ``kappa`` [-] and slip angle ``alpha`` [rad] (:func:`wheel_slip`)."""
        params = self.params if params is None else params

        v_eps = params["v_eps"]

        kappa, alpha = wheel_slip(vx, vy, w, R, v_eps)

        return kappa, alpha

    def stiffnesses(self, Fz, params=None):
        """Small-slip stiffnesses ``(C_kappa, C_alpha)`` [N, N/rad] under the load ``Fz``."""
        params = self.params if params is None else params

        c_kappa = params["c_kappa"]
        c_alpha = params["c_alpha"]

        C_kappa = c_kappa * Fz
        C_alpha = c_alpha * Fz

        return C_kappa, C_alpha

    def forces(self, kappa, alpha, Fz, friction=1.0, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        friction : float or array
            The road's friction factor under the wheel [-], 1 on the reference
            surface: the grip is ``friction mu``.
        params : dict, optional
            Needs ``mu``, ``c_alpha``, ``c_kappa``.
        """
        params = self.params if params is None else params
        xp = array_module(kappa, alpha, Fz)

        mu = friction * params["mu"]
        c_alpha = params["c_alpha"]
        c_kappa = params["c_kappa"]

        # Slip demand: the linear force over the friction limit, per direction
        s_x = c_kappa * kappa / mu
        s_y = c_alpha * alpha / mu
        z = xp.sqrt(s_x**2 + s_y**2 + SLIP_DEMAND_FLOOR**2)

        # F = μ Fz shape(z) along the demand, with shape(z) = z g(z) for the brush
        g = brush_saturation(z)
        Fx = mu * Fz * s_x * g
        Fy = mu * Fz * s_y * g

        return Fx, Fy

    def friction_use(self, Fx, Fy, Fz, friction=1.0, params=None):
        """Used fraction of the friction ellipse [-] (:func:`used_grip`)."""
        params = self.params if params is None else params

        mu = friction * params["mu"]

        phi = used_grip(Fx, Fy, Fz, mu)

        return phi


class MagicFormulaTire:
    """Magic formula tire (Pacejka): a peak at the friction limit, then a fall.

    The curve is taken along the slip demand ``z``, with ``B = c / (C mu)``, so its
    slope at zero slip is the stiffness ``c`` and its peak is ``mu Fz``. The shape
    ``C`` sets the fall past the peak, to ``sin(C pi / 2)`` of it at large slip, and
    the curvature ``E`` where the peak sits.

    References
    ----------
    Pacejka, *Tyre and Vehicle Dynamics*, 3rd ed., ch. 4.
    """

    params = own_params()

    def __init__(self, mu=1.0, c_alpha=8.2, c_kappa=13.6, v_eps=0.5, C=1.3, E=-0.5):
        # Parameters: the grip, the stiffnesses per unit load, the slip floor, and
        # the shape and the curvature of the curve
        self.params = {
            "mu": mu,
            "c_alpha": c_alpha,
            "c_kappa": c_kappa,
            "v_eps": v_eps,
            "C": C,
            "E": E,
        }

    def slip(self, vx, vy, w, R, params=None):
        """Slip ratio ``kappa`` [-] and slip angle ``alpha`` [rad] (:func:`wheel_slip`)."""
        params = self.params if params is None else params

        v_eps = params["v_eps"]

        kappa, alpha = wheel_slip(vx, vy, w, R, v_eps)

        return kappa, alpha

    def stiffnesses(self, Fz, params=None):
        """Small-slip stiffnesses ``(C_kappa, C_alpha)`` [N, N/rad] under the load ``Fz``."""
        params = self.params if params is None else params

        c_kappa = params["c_kappa"]
        c_alpha = params["c_alpha"]

        C_kappa = c_kappa * Fz
        C_alpha = c_alpha * Fz

        return C_kappa, C_alpha

    def forces(self, kappa, alpha, Fz, friction=1.0, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        friction : float or array
            The road's friction factor under the wheel [-], 1 on the reference
            surface: the grip is ``friction mu``.
        params : dict, optional
            Needs ``mu``, ``c_alpha``, ``c_kappa``, ``C``, ``E``.
        """
        params = self.params if params is None else params
        xp = array_module(kappa, alpha, Fz)

        mu = friction * params["mu"]
        c_alpha = params["c_alpha"]
        c_kappa = params["c_kappa"]
        C = params["C"]
        E = params["E"]

        # Slip demand: the linear force over the friction limit, per direction
        s_x = c_kappa * kappa / mu
        s_y = c_alpha * alpha / mu
        z = xp.sqrt(s_x**2 + s_y**2 + SLIP_DEMAND_FLOOR**2)

        # shape(z) = sin(C atan(b − E (b − atan b))), with b = B slip = z / C
        b = z / C
        shape = xp.sin(C * xp.arctan(b - E * (b - xp.arctan(b))))

        # F = μ Fz shape(z) along the demand
        Fx = mu * Fz * s_x * shape / z
        Fy = mu * Fz * s_y * shape / z

        return Fx, Fy

    def friction_use(self, Fx, Fy, Fz, friction=1.0, params=None):
        """Used fraction of the friction ellipse [-] (:func:`used_grip`)."""
        params = self.params if params is None else params

        mu = friction * params["mu"]

        phi = used_grip(Fx, Fy, Fz, mu)

        return phi


class LinearSaturation:
    """Linear tire scaled back onto its friction circle ``|F| <= mu Fz``.

    Its stiffnesses are absolute, ``Ca`` [N/rad] and ``Ck`` [N], and its slip is
    floored as ``|vx| + v_eps`` (:func:`tire_slip`). The law of the dynamic bicycle
    with its default tire.
    """

    params = own_params()

    def __init__(self, mu=1.0, Ca=60000.0, Ck=100000.0, v_eps=0.1):
        # Parameters: the grip, the absolute stiffnesses and the slip floor
        self.params = {"mu": mu, "Ca": Ca, "Ck": Ck, "v_eps": v_eps}

    def slip(self, vx, vy, w, R, params=None):
        """Slip ratio ``kappa`` [-] and slip angle ``alpha`` [rad] (:func:`tire_slip`)."""
        params = self.params if params is None else params

        v_eps = params["v_eps"]

        kappa, alpha = tire_slip(vx, vy, w, R, v_eps)

        return kappa, alpha

    def stiffnesses(self, Fz, params=None):
        """Small-slip stiffnesses ``(C_kappa, C_alpha)`` [N, N/rad]: absolute, whatever ``Fz``."""
        params = self.params if params is None else params

        Ck = params["Ck"]
        Ca = params["Ca"]

        return Ck, Ca

    def forces(self, kappa, alpha, Fz, friction=1.0, params=None):
        """Longitudinal and lateral force ``(Fx, Fy)`` [N] of one wheel, in its frame.

        Parameters
        ----------
        kappa, alpha : float or array
            Slip ratio [-] and slip angle [rad] of the wheel.
        Fz : float or array
            Normal load [N].
        friction : float or array
            The road's friction factor under the wheel [-], 1 on the reference
            surface: the grip is ``friction mu``.
        params : dict, optional
            Needs ``mu``, ``Ca``, ``Ck``.
        """
        params = self.params if params is None else params
        xp = array_module(kappa, alpha)

        mu = friction * params["mu"]
        Ca = params["Ca"]
        Ck = params["Ck"]

        # The linear slip forces
        Fx = Ck * kappa
        Fy = Ca * alpha

        # Scaled back onto the friction circle |F| ≤ μ F_z (10⁻¹² guards the division)
        F_max = mu * Fz
        F_total = xp.sqrt(Fx**2 + Fy**2)
        ratio = xp.where(F_total > F_max, F_max / xp.maximum(F_total, 1e-12), 1.0)
        Fx_sat = Fx * ratio
        Fy_sat = Fy * ratio

        return Fx_sat, Fy_sat

    def friction_use(self, Fx, Fy, Fz, friction=1.0, params=None):
        """Used fraction of the friction circle [-] (:func:`used_grip`)."""
        params = self.params if params is None else params

        mu = friction * params["mu"]

        phi = used_grip(Fx, Fy, Fz, mu)

        return phi


# The slip of one wheel, from its contact velocity in its own frame


def tire_slip(vx, vy, w, R, v_min_epsilon):
    """Slip ratio ``kappa`` [-] and slip angle ``alpha`` [rad] at one contact.

    The rolling speed is floored as ``|vx| + v_min_epsilon``.
    """
    xp = array_module(vx, vy, w)

    # The regularized speed |vx| + ε keeps both slips finite at rest
    vx_adj = xp.abs(vx) + v_min_epsilon

    # α = −atan(vy / (|vx| + ε)) and κ = (ω R − vx) / (|vx| + ε)
    alpha = -xp.arctan(vy / vx_adj)
    kappa = (w * R - vx) / vx_adj

    return kappa, alpha


def wheel_slip(vx, vy, w, R, v_min_epsilon):
    """Slip ratio ``kappa`` [-] and slip angle ``alpha`` [rad] at one contact.

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
    kappa : float or array
        Slip ratio [-]: positive when the tire drives the car forward.
    alpha : float or array
        Slip angle [rad]: positive when the tire pushes the car to the left.
    """
    xp = array_module(vx, vy, w)

    # The rolling speed |vx|, floored to √(vx² + ε²) near rest
    v_roll = xp.sqrt(vx**2 + v_min_epsilon**2)

    # α = −atan(vy / |vx|) and κ = (ω R − vx) / |vx|
    alpha = -xp.arctan(vy / v_roll)
    kappa = (w * R - vx) / v_roll

    return kappa, alpha


# The building blocks of the laws


def used_grip(Fx, Fy, Fz, mu):
    """Used fraction ``phi`` of the friction ellipse [-]: ``1`` means the tire slides."""
    xp = array_module(Fx, Fy, Fz)

    # φ = √(Fx² + Fy²) / (μ Fz), the radius in the normalized force plane
    phi = xp.sqrt(Fx**2 + Fy**2) / (mu * Fz)

    return phi


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
