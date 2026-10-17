"""Engine bicycle: a rear-wheel-drive car on slip tires, with engine and steering lag."""

import numpy as np

from minilink.core.kinematics import SE2
from minilink.core.system import DynamicSystem, System
from minilink.graphical.animation.primitives import Arrow, CustomLine

###############################################################################
# Public API
###############################################################################


def create_vehicle(
    X=0.0, Y=0.0, theta=0.0, vx=0.0, vy=0.0, yawrate=0.0, tire_slip_mode=None
):
    """Build an ``EngineBicycle`` with the demo nominal parameters."""
    vehicle = EngineBicycle()

    # Wheels: radii, outlines, bearing friction and spin inertias
    vehicle.r_f = 0.3429
    vehicle.r_r = 0.3429
    vehicle.wheel_len_rear = vehicle.r_r * 2
    vehicle.wheel_width_rear = 0.2794
    vehicle.wheel_len_front = vehicle.r_f * 2
    vehicle.wheel_width_front = 0.2286
    vehicle.bw_rear = 0.0
    vehicle.bw_front = 0.0
    vehicle.Jw_rear = 1.6
    vehicle.Jw_front = 1.3

    # Chassis: CG-to-axle distances, wheelbase, mass, yaw inertia, gravity and drag
    vehicle.a = 1.16
    vehicle.b = 0.95
    vehicle.L = vehicle.a + vehicle.b
    vehicle.mass = 698.8
    vehicle.inertia = 700.0
    vehicle.gravity = 9.81
    vehicle.rho = 1.225
    vehicle.CdA = 0.0

    # Tires: the magic formula on both axles
    vehicle.tire_model_f = Pacejka(combined_slip_mode=tire_slip_mode)
    vehicle.tire_model_r = Pacejka(combined_slip_mode=tire_slip_mode)

    # Engine and steering: power, losses, time constants and steer limits
    vehicle.engine_power_peak = 48470.5
    vehicle.transmission_ratio = 1.0
    vehicle.engine_dry_resistance = 8.0
    vehicle.engine_rolling_resistance = 0.025
    vehicle.engine_tau = 0.25
    vehicle.steering_tau = 0.15
    vehicle.max_steer = np.pi / 4.0
    vehicle.min_steer = -np.pi / 4.0

    # Initial state: the given pose and body velocities, both wheels rolling at vx
    vehicle.x0 = np.array(
        [
            X,
            Y,
            theta,
            vx,
            vy,
            yawrate,
            vx / vehicle.r_r,
            vx / vehicle.r_f,
            0.0,
            0.0,
        ],
        dtype=float,
    )

    return vehicle


# The bicycle ladder: a rear force input, then the wheel spin states, then the
# engine and steering lag


class ForceInputBicycle(DynamicSystem):
    """Planar dynamic bicycle base with rear longitudinal force input.

    State ``x = [x, y, theta, vx, vy, yawrate]``: the world pose and the
    body-frame velocities. Inputs ``f_rear`` [N] and ``delta`` [rad].
    """

    def __init__(self, n=6):
        super().__init__(n=n)
        self.name = "Force-input bicycle"

        # Signal labels and units
        self.state.labels = ["x", "y", "theta", "vx", "vy", "yawrate"]
        self.state.units = ["m", "m", "rad", "m/s", "m/s", "rad/s"]

        # Ports: the rear longitudinal force and the steer command in, the state out
        self.add_input_port("f_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["f_rear"].labels = ["f_rear"]
        self.inputs["f_rear"].units = ["N"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.add_output_port("y", dim=n, function=self.h, dependencies=())

        # Parameters: steer limits [rad], CG-to-axle distances a and b, wheelbase L
        # and wheel radii [m]
        self.max_steer = 1.571
        self.min_steer = -1.571

        self.a = 1.0
        self.b = 1.0
        self.L = self.a + self.b
        self.r_f = 0.3
        self.r_r = 0.3

        # Graphics: the wheel outlines [m]
        self.wheel_len_rear = self.r_r * 2
        self.wheel_width_rear = 0.2
        self.wheel_len_front = self.r_f * 2
        self.wheel_width_front = 0.2

        # Parameters: mass [kg], yaw inertia [kg m²], gravity [m/s²], air density
        # [kg/m³], drag area CdA [m²], and the tire model of each axle
        self.mass = 1500.0
        self.inertia = 2500.0
        self.gravity = 9.81
        self.rho = 1.225
        self.CdA = 0.3 * 2.2

        self.tire_model_f = TireModel()
        self.tire_model_r = TireModel()

        # Graphics: a camera that follows the body
        self.camera_follow_frame = "body"
        self.camera_scale = 15.0

    def x2q(self, x):
        """The pose ``q`` and the body-frame velocities ``v`` of the state."""
        q = x[0:3]
        v = x[3:6]

        return q, v

    def q2x(self, dq, dv):
        """The state derivative from the pose and velocity derivatives."""
        dx = np.concatenate([dq, dv])

        return dx

    def M_mat(self, q):
        """Mass matrix of the planar rigid body."""
        mass = self.mass
        inertia = self.inertia

        # M = diag(m, m, I_z)
        M = np.diag(np.array([mass, mass, inertia], dtype=float))

        return M

    def C_mat(self, q, v):
        """Coriolis matrix: the yaw rate couples the body-frame velocities."""
        mass = self.mass

        yawrate = v[2]

        C = np.zeros((3, 3), dtype=float)
        C[1, 0] = mass * yawrate
        C[0, 1] = -mass * yawrate

        return C

    def N_mat(self, q):
        """World-frame velocity kinematics ``q̇ = N(q) v``."""
        theta = q[2]
        c = np.cos(theta)
        s = np.sin(theta)

        # fmt: off
        N = np.array([
            [  c,  -s, 0.0],
            [  s,   c, 0.0],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        # fmt: on

        return N

    def compute_wheel_velocities(self, v_body, u_inputs):
        """Each wheel's velocity in its own frame, and its spin rate."""
        a = self.a
        b = self.b
        r_f = self.r_f
        r_r = self.r_r

        vx = v_body[0]
        vy = v_body[1]
        yawrate = v_body[2]
        delta = u_inputs[1]

        # Contact-point velocities in the body frame
        vx_f_b = vx
        vy_f_b = vy + a * yawrate
        vx_r_b = vx
        vy_r_b = vy - b * yawrate

        # The same velocities in each wheel frame: the front one turned by the steer δ
        c_d = np.cos(delta)
        s_d = np.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        # Both wheels roll without slip: ω = v_x / r
        w_r = vx_r / r_r
        w_f = vx_f / r_f

        return vx_f, vy_f, w_f, vx_r, vy_r, w_r

    def compute_tire_physics(self, v_body, u_inputs):
        """Tire forces ``(Fx_f, Fy_f, Fx_r, Fy_r)``, each in its wheel frame."""
        a = self.a
        b = self.b
        L = self.L
        r_f = self.r_f
        r_r = self.r_r
        mass = self.mass
        gravity = self.gravity
        tire_f = self.tire_model_f
        tire_r = self.tire_model_r

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs
        )

        # Static normal-load split between the axles: F_zf = m g b / L, F_zr = m g a / L
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        # Slip forces at each contact
        Fx_f, Fy_f = tire_f.vel2forces(vx_f, vy_f, w_f, r_f, Fz_f)
        Fx_r, Fy_r = tire_r.vel2forces(vx_r, vy_r, w_r, r_r, Fz_r)

        # The rear longitudinal force is the input, not the tire's
        Fx_r = u_inputs[0]

        return Fx_f, Fy_f, Fx_r, Fy_r

    def tire_forces_body_frame_from_forces(self, Fx_f, Fy_f, Fx_r, Fy_r, u_in):
        """The wheel-frame tire forces turned into the body frame."""
        delta = float(u_in[1])
        c_d = np.cos(delta)
        s_d = np.sin(delta)

        # The front pair turned by the steer δ; the rear wheel's frame is the body's
        Fx_f_b = Fx_f * c_d - Fy_f * s_d
        Fy_f_b = Fx_f * s_d + Fy_f * c_d
        Fx_r_b = Fx_r
        Fy_r_b = Fy_r

        return Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b

    def tire_forces_body_frame(self, v_body, u_in):
        """Tire forces ``(Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b)`` in the body frame."""
        Fx_front, Fy_front, Fx_rear, Fy_rear = self.compute_tire_physics(v_body, u_in)
        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame_from_forces(
            Fx_front, Fy_front, Fx_rear, Fy_rear, u_in
        )

        return Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b

    def generalized_d(self, q, v, u_in):
        """Generalized damping: minus the tire and drag wrench on the body."""
        a = self.a
        b = self.b
        rho = self.rho
        CdA = self.CdA

        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame(v, u_in)
        vx = v[0]

        # Resultant wrench: ΣF_x less the drag ½ ρ CdA vx |vx|, ΣF_y and the yaw
        # moment ΣM_z = a F_yf − b F_yr
        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = a * Fy_f_b - b * Fy_r_b
        F_aero = 0.5 * rho * CdA * vx * abs(vx)
        Sum_Fx -= F_aero
        F_ext = np.array([Sum_Fx, Sum_Fy, Sum_Mz], dtype=float)

        d = -F_ext

        return d

    def f(self, x, u, t=0.0, params=None):
        min_steer = self.min_steer
        max_steer = self.max_steer

        q, v = self.x2q(x)
        f_rear, delta = self.get_port_values_from_u(u, "f_rear", "delta")

        # The applied inputs: the rear force, and the steer held inside its limits
        u_in = np.array([f_rear[0], delta[0]])
        u_in[1] = np.clip(u_in[1], min_steer, max_steer)

        M = self.M_mat(q)
        C = self.C_mat(q, v)
        d_vec = self.generalized_d(q, v, u_in)

        # Rigid-body EoM in the body frame: M v̇ = −C v − d, and q̇ = N(q) v
        dv = np.linalg.solve(M, -C @ v - d_vec)
        N = self.N_mat(q)
        dq = N @ v

        dx = self.q2x(dq, dv)

        return dx

    def h(self, x, u, t=0.0, params=None):
        return x.copy()

    def tf(self, x, u, t=0.0, params=None):
        a = self.a
        b = self.b

        X = float(x[0])
        Y = float(x[1])
        theta = float(x[2])
        u_in = self.delayed_actuators(x, u)
        delta = float(u_in[1])

        # The body in the world, and each axle in the body (the front one steered by δ)
        W_T_B = SE2(X, Y, theta)
        B_T_R = SE2(-b, 0.0, 0.0)
        B_T_F = SE2(a, 0.0, delta)

        return {
            "body": W_T_B,
            "rear_wheel": W_T_B @ B_T_R,
            "front_wheel": W_T_B @ B_T_F,
        }

    def get_kinematic_geometry(self):
        """The chassis line between the axles, and the two wheel outlines."""
        a = self.a
        b = self.b
        wheel_len_rear = self.wheel_len_rear
        wheel_width_rear = self.wheel_width_rear
        wheel_len_front = self.wheel_len_front
        wheel_width_front = self.wheel_width_front

        chassis = [
            CustomLine(
                np.array([[-b, 0.0, 0.0], [a, 0.0, 0.0]]),
                color="black",
                linewidth=2,
            )
        ]
        rear_wheel = [
            CustomLine(
                wheel_rectangle_pts(wheel_len_rear, wheel_width_rear),
                color="black",
                linewidth=1,
            )
        ]
        front_wheel = [
            CustomLine(
                wheel_rectangle_pts(wheel_len_front, wheel_width_front),
                color="black",
                linewidth=1,
            )
        ]

        return {"body": chassis, "rear_wheel": rear_wheel, "front_wheel": front_wheel}

    def get_dynamic_geometry(self, x, u, t=0, params=None):
        """Velocity (blue) and tire force (red) arrows at each axle."""
        b = self.b
        a = self.a

        v_r_loc, v_f_loc, F_r_loc, F_f_loc = self.contact_fields(x, u)

        return {
            "body": [
                Arrow(
                    base=(-b, 0.0),
                    vector=v_r_loc,
                    scale=0.2,
                    color="blue",
                    linewidth=1.25,
                ),
                Arrow(
                    base=(a, 0.0),
                    vector=v_f_loc,
                    scale=0.2,
                    color="blue",
                    linewidth=1.25,
                ),
                Arrow(
                    base=(-b, 0.0),
                    vector=F_r_loc,
                    scale=0.001,
                    color="red",
                    linewidth=1.25,
                ),
                Arrow(
                    base=(a, 0.0),
                    vector=F_f_loc,
                    scale=0.001,
                    color="red",
                    linewidth=1.25,
                ),
            ]
        }

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def delayed_actuators(self, x, u):
        """The inputs the actuators apply: the commands themselves, with no lag."""
        return u

    def contact_fields(self, x, u):
        """Body-frame contact velocities and tire forces at each axle."""
        a = self.a
        b = self.b

        _, v = self.x2q(x)
        u_in = self.delayed_actuators(x, u)
        vx = v[0]
        vy = v[1]
        yawrate = v[2]

        # Contact velocities at the front and rear axles
        v_f_loc = np.array([vx, vy + a * yawrate])
        v_r_loc = np.array([vx, vy - b * yawrate])

        # Tire forces in the body frame
        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame(v, u_in)
        F_f_loc = np.array([Fx_f_b, Fy_f_b])
        F_r_loc = np.array([Fx_r_b, Fy_r_b])

        return v_r_loc, v_f_loc, F_r_loc, F_f_loc


class RearDriveBicycle(ForceInputBicycle):
    """Dynamic bicycle with rear and front wheel spin states."""

    def __init__(self, n=8):
        super().__init__(n)
        self.name = "Rear-drive bicycle"

        # Signal labels and units: the body's six, then the two wheel spin rates
        self.state.labels = [
            "X",
            "Y",
            "theta",
            "vx",
            "vy",
            "yawrate",
            "w_rear",
            "w_front",
        ]
        self.state.units = [
            "m",
            "m",
            "rad",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad/s",
        ]

        # Ports: the rear wheel torque and the steer command, in place of the base's
        self.inputs = {}
        self.add_input_port("tau_rear", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["tau_rear"].labels = ["tau_rear"]
        self.inputs["tau_rear"].units = ["Nm"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.outputs = {}
        self.add_output_port("y", dim=n, function=self.h, dependencies=())

        # Parameters: wheel bearing friction, rolling-resistance coefficient, and
        # wheel spin inertias [kg m²]
        self.bw_rear = 0.0
        self.bw_front = 0.0
        self.rolling_friction_constant = 0.0
        self.Jw_rear = 1.0
        self.Jw_front = 1.0

    def x2q(self, x):
        """The pose ``q`` and the velocities ``v``, the wheel spin rates included."""
        q = x[0:3]
        v = x[3:8]

        return q, v

    def M_mat(self, q):
        """Mass matrix of the body and the two wheels."""
        mass = self.mass
        inertia = self.inertia
        Jw_rear = self.Jw_rear
        Jw_front = self.Jw_front

        # M = diag(m, m, I_z, J_wr, J_wf)
        M = np.diag([mass, mass, inertia, Jw_rear, Jw_front])

        return M

    def C_mat(self, q, v):
        """Coriolis matrix: the yaw rate couples the body-frame velocities."""
        mass = self.mass

        yawrate = v[2]

        C = np.zeros((5, 5))
        C[0, 1] = -mass * yawrate
        C[1, 0] = mass * yawrate

        return C

    def N_mat(self, q):
        """Velocity kinematics ``q̇ = N(q) v``; the wheel spins move no pose."""
        theta = q[2]
        c = np.cos(theta)
        s = np.sin(theta)

        # fmt: off
        N = np.array([
            [  c,  -s, 0.0, 0.0, 0.0],
            [  s,   c, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0],
        ])
        # fmt: on

        return N

    def g(self, q):
        # No gravity in the plane
        g = np.zeros(5)

        return g

    def compute_wheel_velocities(self, v_body, u_inputs):
        """Each wheel's velocity in its own frame; the spin rates are states."""
        a = self.a
        b = self.b

        vx = v_body[0]
        vy = v_body[1]
        yawrate = v_body[2]
        w_rear = v_body[3]
        w_front = v_body[4]
        delta = u_inputs[1]

        # Contact-point velocities in the body frame
        vx_f_b = vx
        vy_f_b = vy + a * yawrate
        vx_r_b = vx
        vy_r_b = vy - b * yawrate

        # The same velocities in each wheel frame: the front one turned by the steer δ
        c_d = np.cos(delta)
        s_d = np.sin(delta)
        vx_f = c_d * vx_f_b + s_d * vy_f_b
        vy_f = -s_d * vx_f_b + c_d * vy_f_b
        vx_r = vx_r_b
        vy_r = vy_r_b

        return vx_f, vy_f, w_front, vx_r, vy_r, w_rear

    def compute_tire_physics(self, v_body, u_inputs):
        """The four tire forces, each in its own wheel frame."""
        a = self.a
        b = self.b
        L = self.L
        r_f = self.r_f
        r_r = self.r_r
        mass = self.mass
        gravity = self.gravity
        tire_f = self.tire_model_f
        tire_r = self.tire_model_r

        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(
            v_body, u_inputs
        )

        # Static normal-load split between the axles: F_zf = m g b / L, F_zr = m g a / L
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        # Slip forces at each contact; the undriven front wheel takes no traction
        Fx_front, Fy_front = tire_f.vel2forces(vx_f, vy_f, w_f, r_f, Fz_f)
        Fx_front = 0.0
        Fx_rear, Fy_rear = tire_r.vel2forces(vx_r, vy_r, w_r, r_r, Fz_r)

        return Fx_front, Fy_front, Fx_rear, Fy_rear

    def generalized_d(self, q, v, u_in):
        """Generalized damping: minus the body wrench and the wheels' load torques."""
        a = self.a
        b = self.b
        L = self.L
        r_f = self.r_f
        r_r = self.r_r
        mass = self.mass
        gravity = self.gravity
        rho = self.rho
        CdA = self.CdA
        bw_rear = self.bw_rear
        bw_front = self.bw_front
        c_rr = self.rolling_friction_constant

        w_rear = v[3]
        w_front = v[4]
        Fx_front, Fy_front, Fx_rear, Fy_rear = self.compute_tire_physics(v, u_in)
        Fx_f_b, Fy_f_b, Fx_r_b, Fy_r_b = self.tire_forces_body_frame_from_forces(
            Fx_front, Fy_front, Fx_rear, Fy_rear, u_in
        )
        vx = v[0]

        # Resultant wrench on the body: ΣF_x less the drag ½ ρ CdA vx |vx|, ΣF_y and
        # the yaw moment ΣM_z = a F_yf − b F_yr
        Sum_Fx = Fx_f_b + Fx_r_b
        Sum_Fy = Fy_f_b + Fy_r_b
        Sum_Mz = a * Fy_f_b - b * Fy_r_b
        F_aero = 0.5 * rho * CdA * vx * abs(vx)
        Sum_Fx -= F_aero

        # Static normal-load split between the axles: F_zf = m g b / L, F_zr = m g a / L
        Fz_f = mass * gravity * (b / L)
        Fz_r = mass * gravity * (a / L)

        # Load torque on each wheel: τ_r = r_r F_xr + b_wr ω_r + c_rr F_zr sign(ω_r),
        # τ_f = r_f F_xf + b_wf ω_f + c_rr F_zf sign(ω_f)
        Tau_load_rear = r_r * Fx_rear + bw_rear * w_rear + c_rr * Fz_r * np.sign(w_rear)
        Tau_load_front = (
            r_f * Fx_front + bw_front * w_front + c_rr * Fz_f * np.sign(w_front)
        )

        # The external forces: the body wrench, and the load torques against each spin
        Q_ext = np.array([Sum_Fx, Sum_Fy, Sum_Mz, -Tau_load_rear, -Tau_load_front])

        d = -Q_ext

        return d

    def u2e(self, u):
        """The generalized input: the rear torque drives the rear wheel spin alone."""
        tau_rear, delta = u

        e = np.array([0.0, 0.0, 0.0, tau_rear, 0.0])

        return e

    def B(self, q, u):
        B = np.eye(5)

        return B

    def accelerations(self, q, v, u, t=0.0):
        """The velocity derivative ``v̇`` from the equations of motion."""
        M = self.M_mat(q)
        C = self.C_mat(q, v)
        g = self.g(q)
        d = self.generalized_d(q, v, u)
        B = self.B(q, u)
        e = self.u2e(u)

        # M v̇ = B e − C v − g − d
        dv = np.linalg.inv(M) @ (B @ e - C @ v - g - d)

        return dv

    def f(self, x, u, t=0.0, params=None):
        min_steer = self.min_steer
        max_steer = self.max_steer

        # The steer command held inside its limits, in the caller's u itself
        u[1] = np.clip(u[1], min_steer, max_steer)

        q, v = self.x2q(x)

        # v̇ from the equations of motion, and q̇ = N(q) v
        dv = self.accelerations(q, v, u, t)
        N = self.N_mat(q)
        dq = N @ v

        dx = self.q2x(dq, dv)

        return dx


class EngineBicycle(RearDriveBicycle):
    """Rear-wheel-drive bicycle with throttle, engine lag, and steering lag.

    State ``x = [X, Y, theta, vx, vy, yawrate, w_rear, w_front,
    torque_engine, delta_act]``. Inputs ``throttle`` (normalized) and ``delta``
    (steer command) [rad].
    """

    def __init__(self, n=10):
        super().__init__(n=n)
        self.name = "Engine bicycle"

        # Signal labels and units: the rear-drive bicycle's eight, then the engine
        # torque and the actual steer the lags hold
        self.state.labels = [
            "X",
            "Y",
            "theta",
            "vx",
            "vy",
            "yawrate",
            "w_rear",
            "w_front",
            "torque_engine",
            "delta_act",
        ]
        self.state.units = [
            "m",
            "m",
            "rad",
            "m/s",
            "m/s",
            "rad/s",
            "rad/s",
            "rad/s",
            "Nm",
            "rad",
        ]

        # Ports: the throttle and the steer command in; the state and the rear
        # tire's forces and slips out
        self.inputs = {}
        self.add_input_port("throttle", nominal_value=np.array([0.0]))
        self.add_input_port("delta", nominal_value=np.array([0.0]))
        self.inputs["throttle"].labels = ["throttle"]
        self.inputs["throttle"].units = ["normalized"]
        self.inputs["delta"].labels = ["delta"]
        self.inputs["delta"].units = ["rad"]

        self.outputs = {}
        self.add_output_port("y", dim=n, function=self.h, dependencies=())
        self.add_output_port(
            "rear_tire_data",
            dim=4,
            function=self.rear_tire_forces_and_slip,
            dependencies="all",
        )

        # Parameters: engine peak power [W], transmission ratio, the engine's dry,
        # rolling and viscous resistances, the engine and steering time constants
        # [s], and the steer rate limit [rad/s]
        self.engine_power_peak = 40000.0
        self.transmission_ratio = 1.0
        self.engine_dry_resistance = 8.0
        self.engine_rolling_resistance = 0.025
        self.engine_viscous_resistance = 0.008
        self.engine_tau = 0.25
        self.steering_tau = 0.15
        self.steering_rate_max = 10.0

    def q2x(self, dq, dv, d_tau_engine=0.0, d_delta_act=0.0):
        """The state derivative, the two lag states' rates last."""
        dx = np.concatenate([dq, dv, [d_tau_engine, d_delta_act]])

        return dx

    def engine_torque_from_throttle(self, v, throttle):
        """The rear wheel torque a throttle asks of the engine, less its resistances."""
        engine_power_peak = self.engine_power_peak
        transmission_ratio = self.transmission_ratio
        engine_dry_resistance = self.engine_dry_resistance
        engine_rolling_resistance = self.engine_rolling_resistance
        engine_viscous_resistance = self.engine_viscous_resistance

        w_rear = v[3]
        throttle = np.clip(throttle, 0.0, 1.0)

        # Engine speed through the transmission, the wheel rate kept off zero:
        # ω_e = max(ω_r, 10⁻⁶) i
        w_rear_num = max(w_rear, 1e-6)
        w_engine = w_rear_num * transmission_ratio

        # Power-limited torque: τ_max = P_peak / ω_e, clipped to [0, P_peak]
        available_torque = engine_power_peak / w_engine
        available_torque = np.clip(available_torque, 0.0, engine_power_peak)

        # τ_r = throttle τ_max, less the dry, rolling and viscous losses
        # c_dry sign(ω_e) + c_roll ω_e + (c_visc ω_e)² once the wheel turns
        tau_rear = throttle * available_torque
        if w_rear_num > 1e-6:
            tau_rear = (
                tau_rear
                - engine_dry_resistance * np.sign(w_engine)
                - engine_rolling_resistance * w_engine
                - (engine_viscous_resistance * w_engine) ** 2
            )

        return tau_rear

    def f(self, x, u, t=0.0, params=None):
        engine_tau = self.engine_tau
        steering_tau = self.steering_tau
        steering_rate_max = self.steering_rate_max
        min_steer = self.min_steer
        max_steer = self.max_steer

        q, v = self.x2q(x)
        throttle, delta_cmd = self.get_port_values_from_u(u, "throttle", "delta")
        throttle = throttle[0]
        delta_cmd = delta_cmd[0]

        # Engine lag toward the throttle's torque:
        # τ̇_e = (τ_cmd − τ_e) / max(T_e, 10⁻⁶)
        tau_cmd = self.engine_torque_from_throttle(v, throttle)
        torque_engine, delta_act = self.delayed_actuators(x, u)
        T_engine = max(engine_tau, 1e-6)
        d_tau_engine = (tau_cmd - torque_engine) / T_engine

        # Steering lag toward the command held inside the steer limits:
        # δ̇_act = clip((δ_cmd − δ_act) / max(T_s, 10⁻⁶), ±δ̇_max)
        delta_cmd = float(np.clip(delta_cmd, min_steer, max_steer))
        delta_act_used = float(np.clip(delta_act, min_steer, max_steer))
        T_steering = max(steering_tau, 1e-6)
        d_delta_act = (delta_cmd - delta_act) / T_steering
        d_delta_act = float(np.clip(d_delta_act, -steering_rate_max, steering_rate_max))

        # The steer stops at its limits: no rate further out
        if delta_act >= max_steer and d_delta_act > 0.0:
            d_delta_act = 0.0

        if delta_act <= min_steer and d_delta_act < 0.0:
            d_delta_act = 0.0

        # The body and the wheels, driven by the engine torque and the held steer:
        # v̇ from the equations of motion, and q̇ = N(q) v
        u_drive = np.array([torque_engine, delta_act_used], dtype=float)
        dv = self.accelerations(q, v, u_drive, t)
        N = self.N_mat(q)
        dq = N @ v

        dx = np.concatenate([dq, dv, np.array([d_tau_engine, d_delta_act])])

        return dx

    def rear_tire_forces_and_slip(self, x, u, t=0.0, params=None):
        """The rear tire's ``[Fx, kappa, Fy, alpha]``: its forces and its slips."""
        a = self.a
        L = self.L
        r_r = self.r_r
        mass = self.mass
        gravity = self.gravity
        tire_r = self.tire_model_r

        _, v = self.x2q(x)
        torque_engine, delta_act = self.delayed_actuators(x, u)
        u_drive = np.array([torque_engine, delta_act], dtype=float)
        vx_f, vy_f, w_f, vx_r, vy_r, w_r = self.compute_wheel_velocities(v, u_drive)

        # The rear axle's static normal load: F_zr = m g a / L
        Fz_r = mass * gravity * (a / L)

        # The rear slips, then the forces they make
        alpha, kappa = tire_r.vel2slip(vx_r, vy_r, w_r, r_r)
        Fx, Fy = tire_r.slip2forces(alpha, kappa, Fz_r)

        rear_tire_data = np.array([Fx, kappa, Fy, alpha], dtype=float)

        return rear_tire_data

    ###########################################################################
    # Internal machinery
    ###########################################################################

    def delayed_actuators(self, x, u):
        """The engine torque and the steer the lag states hold, not the commands."""
        torque_engine = float(x[8])
        delta_act = float(x[9])

        u_drive = np.array([torque_engine, delta_act], dtype=float)

        return u_drive


class VehicleMeasurement(System):
    """Split the 10-D plant output into scalar ports used by the cascade."""

    def __init__(self, name: str = "Vehicle measurements"):
        super().__init__(0)
        self.name = name

        # Ports: the plant output in; the heading, the speed, the yaw rate and the
        # rear wheel rate out
        self.add_input_port("y", nominal_value=np.zeros(10))
        self.add_output_port("theta", dim=1, function=self.h_theta, dependencies=("y",))
        self.add_output_port("vx", dim=1, function=self.h_vx, dependencies=("y",))
        self.add_output_port(
            "yawrate", dim=1, function=self.h_yawrate, dependencies=("y",)
        )
        self.add_output_port(
            "w_rear", dim=1, function=self.h_w_rear, dependencies=("y",)
        )

    def h_theta(self, x, u, t=0.0, params=None):
        theta = np.array([u[2]], dtype=float)

        return theta

    def h_vx(self, x, u, t=0.0, params=None):
        vx = np.array([u[3]], dtype=float)

        return vx

    def h_yawrate(self, x, u, t=0.0, params=None):
        yawrate = np.array([u[5]], dtype=float)

        return yawrate

    def h_w_rear(self, x, u, t=0.0, params=None):
        w_rear = np.array([u[6]], dtype=float)

        return w_rear


# Tire models


class TireModel:
    """Base strategy for tire-road interaction."""

    def __init__(self):
        # Slip regularization speed [m/s]: keeps the slips finite at standstill
        self.v_min_epsilon = 0.1

    def vel2slip(self, vx, vy, w, R):
        """The lateral slip angle ``alpha`` and the longitudinal slip ratio ``kappa``."""
        v_min_epsilon = self.v_min_epsilon

        wr = w * R

        # α = −atan2(v_y, max(|v_x|, ε)),
        # κ = (ω R − v_x) / max(|v_x|, |ω R|, ε)
        denom = np.maximum(np.maximum(np.abs(vx), np.abs(wr)), v_min_epsilon)
        alpha = -np.arctan2(vy, np.maximum(np.abs(vx), v_min_epsilon))
        kappa = (wr - vx) / denom

        return alpha, kappa

    def slip2forces(self, alpha, kappa, Fz):
        """Convert slip values to tire forces."""
        raise NotImplementedError

    def vel2forces(self, vx, vy, w, R, Fz):
        """Compute tire forces directly from wheel velocities."""
        alpha, kappa = self.vel2slip(vx, vy, w, R)
        return self.slip2forces(alpha, kappa, Fz)


class LinearTire(TireModel):
    """Linear slip tire with friction-circle saturation."""

    def __init__(self, Ca=60000, Ck=100000, mu=1.0):
        super().__init__()

        # Parameters: cornering stiffness [N/rad], longitudinal stiffness [N], friction
        self.Ca = Ca
        self.Ck = Ck
        self.mu = mu

    def slip2forces(self, alpha, kappa, Fz):
        """Linear slip forces, scaled back onto the friction circle beyond it."""
        Ck = self.Ck
        Ca = self.Ca
        mu = self.mu

        # F_x = C_κ κ, F_y = C_α α
        Fx = Ck * kappa
        Fy = Ca * alpha

        # Friction circle: √(F_x² + F_y²) ≤ μ F_z
        F_max = mu * Fz
        F_total = np.sqrt(Fx**2 + Fy**2)
        if F_total > F_max:
            ratio = F_max / F_total
            Fx *= ratio
            Fy *= ratio

        return Fx, Fy


class Pacejka(TireModel):
    """Pacejka tire model."""

    def __init__(
        self,
        Bx=10.0,
        Cx=1.3,
        Dx=1.0,
        Ex=0.97,
        By=10.0,
        Cy=1.3,
        Dy=1.0,
        Ey=0.97,
        combined_slip_mode=None,
    ):
        super().__init__()
        self.combined_slip_mode = combined_slip_mode

        # Parameters: the magic-formula stiffness B, shape C, peak D and curvature E,
        # longitudinal (x) and lateral (y)
        self.Bx = Bx
        self.Cx = Cx
        self.Dx = Dx
        self.Ex = Ex
        self.By = By
        self.Cy = Cy
        self.Dy = Dy
        self.Ey = Ey

        # Parameters: the combined-slip weighting coefficients
        self.r_Bx1 = 1.0
        self.r_Bx2 = 1.0
        self.r_Cx1 = 1.0
        self.r_Ex1 = 1.0
        self.r_Ex2 = 1.0

        self.r_By1 = 1.0
        self.r_By2 = 1.0
        self.r_By3 = 1.0
        self.r_Cy1 = 1.0
        self.r_Ey1 = 1.0
        self.r_Ey2 = 1.0

        # Parameters: the load increment and the scaling factors of the weighting,
        # and the friction coefficient of the friction circle
        self.d_fz = -1.0
        self.lambda_ya = 1.0
        self.lambda_yk = 1.0
        self.mu = 1.0

    def slip2forces(self, alpha, kappa, Fz):
        """The magic-formula force of each direction, then the combined-slip mode."""
        Bx = self.Bx
        Cx = self.Cx
        Dx = self.Dx
        Ex = self.Ex
        By = self.By
        Cy = self.Cy
        Dy = self.Dy
        Ey = self.Ey
        mode = self.combined_slip_mode

        def magic_formula(x, B, C, D, E, fz):
            # F = D F_z sin(C atan(B x − E (B x − atan(B x))))
            D_scaled = D * fz
            F = D_scaled * np.sin(C * np.arctan(B * x - E * (B * x - np.arctan(B * x))))
            return F

        # Pure slip: F_x0 from the slip ratio κ, F_y0 from the slip angle α
        Fx = magic_formula(kappa, Bx, Cx, Dx, Ex, Fz)
        Fy = magic_formula(alpha, By, Cy, Dy, Ey, Fz)

        Fx, Fy = self.combined_slip(Fx, Fy, kappa, alpha, Fz, mode=mode)

        return Fx, Fy

    def combined_slip(self, Fx_0, Fy_0, kappa, alpha, Fz, mode=None):
        """Optional combined-slip weighting."""
        mu = self.mu

        Fx = Fx_0
        Fy = Fy_0

        if mode == "w":
            # Weighting: F_x = G_xα F_x0, F_y = G_yκ F_y0
            G_xa = self.Gxa(kappa, alpha)
            Fx *= G_xa
            G_yk = self.Gyk(kappa, alpha)
            Fy *= G_yk
        elif mode == "c":
            # Friction circle: √(F_x² + F_y²) ≤ μ F_z
            F_max = mu * Fz
            F_total = np.sqrt(Fx**2 + Fy**2)
            if F_total > F_max:
                ratio = F_max / F_total
                Fx *= ratio
                Fy *= ratio

        return Fx, Fy

    def Gxa(self, k, a):
        """Combined-slip weight ``G_xα`` of the longitudinal force."""
        r_Bx1 = self.r_Bx1
        r_Bx2 = self.r_Bx2
        r_Cx1 = self.r_Cx1
        r_Ex1 = self.r_Ex1
        r_Ex2 = self.r_Ex2
        d_fz = self.d_fz
        lambda_ya = self.lambda_ya

        B = r_Bx1 * np.cos(np.arctan(r_Bx2 * k)) * lambda_ya
        C = r_Cx1
        E = r_Ex1 + r_Ex2 * d_fz

        # G_xα = cos(C atan(B tan α − E (B tan α − atan(B tan α))))
        ratio = np.cos(
            C
            * np.arctan(B * np.tan(a) - E * (B * np.tan(a) - np.arctan(B * np.tan(a))))
        )

        return ratio

    def Gyk(self, k, a):
        """Combined-slip weight ``G_yκ`` of the lateral force."""
        r_By1 = self.r_By1
        r_By2 = self.r_By2
        r_By3 = self.r_By3
        r_Cy1 = self.r_Cy1
        r_Ey1 = self.r_Ey1
        r_Ey2 = self.r_Ey2
        d_fz = self.d_fz
        lambda_yk = self.lambda_yk

        B = r_By1 * np.cos(np.arctan(r_By2 * (np.tan(a) - r_By3))) * lambda_yk
        C = r_Cy1
        E = r_Ey1 + r_Ey2 * d_fz

        # G_yκ = cos(C atan(B κ − E (B κ − atan(B κ))))
        ratio = np.cos(C * np.arctan(B * k - E * (B * k - np.arctan(B * k))))

        return ratio


###############################################################################
# Internal machinery
###############################################################################


def wheel_rectangle_pts(wl: float, ww: float) -> np.ndarray:
    """Closed polyline in wheel frame (x forward, y lateral)."""
    h = wl / 2
    w = ww / 2

    # fmt: off
    pts = np.array([
        [ h,  w, 0.0],
        [ h, -w, 0.0],
        [-h, -w, 0.0],
        [-h,  w, 0.0],
        [ h,  w, 0.0],
    ])
    # fmt: on

    return pts
