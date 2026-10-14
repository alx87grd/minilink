"""Boat parking in a current: the plant, the dock, the cost and the cascade law the prototypes share."""

import numpy as np

from minilink import Boat2DWithCurrent, CostFunction, lqr_gain
from minilink.core.backends import array_module
from minilink.core.system import System

THRUST_MAX = np.array([10000.0, 1000.0])  # [N] surge and sway thrust limits
CURRENT = np.array([-1.0, -1.0])  # [m/s] world-frame current, flowing to the south-west
XY_RANGE = 40.0  # [m] half-width of the state box round the dock
# The start: downstream of the dock, at rest, bow along +x
X_START = np.array([-25.0, -10.0, 0.0, 0.0, 0.0, 0.0])
# The dock: the origin, bow into the current. A thrust vector l_t aft of the c.g. can
# hold any point, but only with the relative flow near the bow or stern axis: every
# other heading asks for a sway force whose yaw moment the boat cannot cancel.
X_DOCK = np.array([0.0, 0.0, np.arctan2(-CURRENT[1], -CURRENT[0]), 0.0, 0.0, 0.0])

# The cost shared by every planner: position error at a 10 m scale, heading error on
# the circle, the body velocities and the normalized thrust, scaled so a parking run
# costs of the order of a hundred. Heading and velocities weigh more than position:
# a learned law with light weights there wanders round the dock instead of settling.
Q = np.diag([0.01, 0.01, 1.0, 0.1, 0.1, 0.5])
R = np.diag([0.1, 0.1])


class ParkingBoat(Boat2DWithCurrent):
    """The catalog boat with thrusts normalized to [-1, 1] and a parking-sized state box."""

    def __init__(self, current=CURRENT):
        super().__init__()
        self.name = "Parking Boat"
        self.params["current_velocity"] = np.asarray(current, dtype=float)
        self.params["thrust_max"] = THRUST_MAX
        self.inputs["u"].lower_bound = np.array([-1.0, -1.0])
        self.inputs["u"].upper_bound = np.array([+1.0, +1.0])
        self.inputs["u"].units = ["-", "-"]
        self.state.lower_bound = np.array(
            [-XY_RANGE, -XY_RANGE, -4 * np.pi, -5, -5, -2]
        )
        self.state.upper_bound = -self.state.lower_bound
        self.x0 = X_START.copy()
        self.camera_follow_frame = None
        self.camera_scale = XY_RANGE

    def damping(self, relative_velocity, params=None):
        params = self.params if params is None else params
        damping_coef = params["damping_coef"]
        rho, Afc, Alc, loa = params["rho"], params["Afc"], params["Alc"], params["loa"]
        Cx_max, Cy_max = params["Cx_max"], params["Cy_max"]
        Cm_max, N_max = params["Cm_max"], params["N_max"]
        xp = array_module(relative_velocity)
        vx, vy, r = relative_velocity

        # The catalog's quadratic drag with the attack angle eliminated: V² cos α |cos α|
        # is vx |vx|, V² sin α |sin α| is −vy |vy| and V² sin 2α is −2 vx vy, so the
        # same forces are polynomials in the relative velocity, smooth at rest where
        # the attack angle (an arctan2 at the origin) has no gradient.
        fx = 0.5 * rho * Afc * Cx_max * vx * xp.abs(vx)
        fy = 0.5 * rho * Alc * Cy_max * vy * xp.abs(vy)
        mz = (
            rho * Alc * loa * Cm_max * vx * vy + N_max * rho * Alc * loa * xp.abs(r) * r
        )
        d = damping_coef * relative_velocity + xp.array([fx, fy, mz])

        return d

    def generalized_force(self, q, v, u, t=0.0, params=None):
        params = self.params if params is None else params
        thrust_max = params["thrust_max"]
        B = self.B(q, params)

        # the normalized command scaled to newtons, then placed by the thrust geometry
        tau = B @ (thrust_max * u)

        return tau

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        return super().get_dynamic_geometry(x, self.params["thrust_max"] * u, t, params)


def dock_thrust(boat, x_dock=X_DOCK):
    """The normalized thrust that holds the dock against the current, by least squares."""
    q, v = boat.x2qv(x_dock)
    thrust_max = boat.params["thrust_max"]
    B = boat.B(q)
    d = boat.d(q, v)

    # B diag(T_max) u = d(q, 0): the current's drag at rest, balanced by the thrusters
    u_dock = np.linalg.lstsq(B * thrust_max, d, rcond=None)[0]

    return u_dock


def wrap_angle(theta, xp):
    """The angle brought back to [-π, π)."""
    return (theta + xp.pi) % (2.0 * xp.pi) - xp.pi


class ParkingCost(CostFunction):
    """Quadratic cost about the dock, with the heading error measured on the circle.

    ``g = eᵀ Q e + (u − u_dock)ᵀ R (u − u_dock) + w (1 − exp(−|e_p|² / 2 ρ²))`` for the
    pose error ``e = x − x_dock`` with its heading component wrapped to ``[−π, π)``:
    periodic, so a law that reads the heading as a point on the circle sees the same
    cost, and with a slope everywhere but at the wrap, so a boat parked stern first
    is not at a stationary point. The last term is a docking well of weight ``w``
    and radius ``ρ`` [m], a bounded reward for the last metres that a learned law
    needs and a planner does not (``well=0`` by default). The terminal cost ``h`` is
    ``terminal`` times the state part of ``g``.
    """

    def __init__(self, x_dock, u_dock, Q=Q, R=R, terminal=0.0, well=0.0, radius=2.0):
        self.params = {
            "Q": np.asarray(Q, dtype=float),
            "R": np.asarray(R, dtype=float),
            "x_dock": np.asarray(x_dock, dtype=float),
            "u_dock": np.asarray(u_dock, dtype=float),
            "terminal": float(terminal),
            "well": float(well),
            "radius": float(radius),
        }

    def g(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        Q, R, x_dock, u_dock = (
            params["Q"],
            params["R"],
            params["x_dock"],
            params["u_dock"],
        )
        w, rho = params["well"], params["radius"]
        xp = array_module(x)
        e = x - x_dock
        e = xp.array([e[0], e[1], wrap_angle(e[2], xp), e[3], e[4], e[5]])
        du = u - u_dock

        g = (
            e @ Q @ e
            + du @ R @ du
            + w * (1.0 - xp.exp(-(e[:2] @ e[:2]) / (2.0 * rho**2)))
        )

        return g

    def h(self, x, t=0.0, params=None):
        params = self.params if params is None else params
        Q, x_dock, terminal = params["Q"], params["x_dock"], params["terminal"]
        xp = array_module(x)
        e = x - x_dock
        e = xp.array([e[0], e[1], wrap_angle(e[2], xp), e[3], e[4], e[5]])

        h = terminal * (e @ Q @ e)

        return h


def parking_cost(boat, x_dock=X_DOCK, terminal=0.0, well=0.0):
    """The parking cost about the dock and the thrust that holds it."""
    return ParkingCost(x_dock, dock_thrust(boat, x_dock), terminal=terminal, well=well)


POLICY_HIDDEN = (64, 64)  # the learned law's network
POLICY_WEIGHTS = (
    "examples/experimental/boat/parking_ppo_policy.npz"  # written by parking_ppo.py
)


def dock_features(x):
    """Policy features: the dock-relative position in hull lengths, the heading error
    as a point on the circle, the body velocities in units of 2 m/s."""
    xp = array_module(x)
    e = x - X_DOCK
    scale, speed = 10.0 / XY_RANGE, 2.0  # [1/m], [m/s]

    z = xp.array(
        [
            scale * e[0],
            scale * e[1],
            xp.cos(e[2]),
            xp.sin(e[2]),
            x[3] / speed,
            x[4] / speed,
            x[5],
        ]
    )

    return z


class CascadeParkingController(System):
    """Cascade parking law: a position loop over an LQR velocity loop.

    The outer loop turns the world-frame pose error into a body-frame velocity
    reference, ``v_ref = N(q)ᵀ K_p e``, saturated at ``v_max``. Far from the dock
    (beyond ``d_switch``) the heading error is the bearing of the dock minus the
    course over ground, heading plus sideslip, with the sideslip faded out below
    the speed ``v_blend``; the sway reference is dropped there, so the boat closes
    in bow first. Within ``d_switch`` the reference is the dock pose, its heading
    biased by ``K_lat`` [rad/m] toward the lateral offset, up to ``bias_max``. The inner
    loop is the LQR gain of the velocity subsystem linearized at the dock,
    ``u = K (v_ref − v)``, with no weight on sway (``Q_v[1] = 0``): a stern thruster
    cannot push sideways without yawing, and a sway loop only fights the yaw loop.
    Ports: the plant state ``y`` (dimension 6) in, the normalized thrust ``u`` out.
    """

    def __init__(
        self,
        boat,
        x_dock=X_DOCK,
        K_p=(0.2, 0.2, 1.0),
        d_switch=5.0,
        v_max=(2.0, 0.5, 0.5),
        Q_v=(1.0, 0.0, 5.0),
        R_v=(1.0, 0.1),
        v_blend=0.5,
        K_lat=0.5,
        bias_max=1.0,
    ):
        super().__init__()
        self.name = "Cascade Parking Controller"

        # Inner loop: LQR on the velocity rows of the dock linearization
        lin = boat.linearize(x_dock, dock_thrust(boat, x_dock))
        A_v, B_v = lin.A()[3:, 3:], lin.B()[3:, :]
        K = lqr_gain(A_v, B_v, np.diag(Q_v), np.diag(R_v))

        self.params = {
            "K": K,
            "K_p": np.diag(np.asarray(K_p, dtype=float)),
            "x_dock": np.asarray(x_dock, dtype=float),
            "d_switch": float(d_switch),
            "v_max": np.asarray(v_max, dtype=float),
            "v_blend": float(v_blend),
            "K_lat": float(K_lat),
            "bias_max": float(bias_max),
        }
        self.add_input_port("y", dim=6)
        self.add_output_port("u", dim=2, function=self.ctl, dependencies=("y",))

    def ctl(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        K, K_p, x_dock = params["K"], params["K_p"], params["x_dock"]
        d_switch, v_max, v_blend = (
            params["d_switch"],
            params["v_max"],
            params["v_blend"],
        )
        K_lat, bias_max = params["K_lat"], params["bias_max"]
        xp = array_module(u)
        q, v = u[:3], u[3:]
        q_dock = x_dock[:3]

        # Pose error: when far, the course over ground (heading plus sideslip) steered
        # onto the dock's bearing, since a hull that slides sideways can point at the dock
        # and still orbit it; the sideslip fades out below ``v_blend``, where the course
        # of a boat is noise and its heading is what the thruster steers. When close,
        # the heading steered onto the dock's, biased toward the lateral offset (a line
        # of sight), the bias reversed when the dock lies astern
        e_xy = q_dock[:2] - q[:2]
        distance = xp.hypot(e_xy[0], e_xy[1])
        far = distance > d_switch
        course = q[2] + xp.arctan2(v[1], v[0] + v_blend)
        e_course = wrap_angle(xp.arctan2(e_xy[1], e_xy[0]) - course, xp)
        c_d, s_d = xp.cos(q_dock[2]), xp.sin(q_dock[2])
        e_along, e_lateral = (
            c_d * e_xy[0] + s_d * e_xy[1],
            -s_d * e_xy[0] + c_d * e_xy[1],
        )
        astern = xp.tanh(2.0 * e_along + 1.0)  # −1 when the dock is well behind
        bias = xp.clip(K_lat * e_lateral * astern, -bias_max, bias_max)
        e_theta = xp.where(far, e_course, wrap_angle(q_dock[2] + bias - q[2], xp))
        e = xp.array([e_xy[0], e_xy[1], e_theta])

        # Outer loop: a world-frame rate, mapped to a saturated body-frame velocity
        # reference. Far from the dock the sway reference is dropped: the stern thruster
        # cannot push sideways without yawing, so the boat closes in like a car, bow first.
        c, s = xp.cos(q[2]), xp.sin(q[2])
        # fmt: off
        N = xp.array([
            [  c,  -s, 0.0],
            [  s,   c, 0.0],
            [0.0, 0.0, 1.0],
        ])
        # fmt: on
        v_ref = N.T @ (K_p @ e)
        v_ref = xp.array([v_ref[0], xp.where(far, 0.0, v_ref[1]), v_ref[2]])
        v_ref = xp.clip(v_ref, -v_max, v_max)

        # Inner loop: LQR on the velocity error, inside the thrust limits
        u_cmd = xp.clip(K @ (v_ref - v), -1.0, 1.0)

        return u_cmd
