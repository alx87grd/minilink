import numpy as np

from minilink.core.system import DynamicSystem


class PID(DynamicSystem):
    """Generic scalar PID controller.

    Inputs
    ------
    ref
        Reference signal.
    meas
        Measured signal.

    Output
    ------
    cmd
        PID command.

    State
    -----
    x = [int_e, meas_filt]

    Notes
    -----
    Error is:

        e = ref - meas

    PID law is:

        cmd = Kp * e + Ki * int_e - Kd * d_meas_filt

    where the derivative is computed on the filtered measurement, not directly
    on the error. This avoids derivative kick when the reference changes.
    """

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        tau: float = 0.05,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        name: str = "PID",
    ):
        # 2 states:
        # x[0] = integral of error
        # x[1] = filtered measurement
        #
        # 2 scalar inputs:
        # ref, meas
        #
        # 1 scalar output:
        # cmd
        super().__init__(2, 2, 1)

        self.name = name

        self.params = {
            "Kp": float(Kp),
            "Ki": float(Ki),
            "Kd": float(Kd),
            "tau": float(tau),
            "cmd_min": float(cmd_min),
            "cmd_max": float(cmd_max),
            "i_min": float(i_min),
            "i_max": float(i_max),
        }

        self.state.labels = ["int_e", "meas_filt"]
        self.state.units = ["", ""]

        self.x0 = np.array([0.0, 0.0], dtype=float)

        self.t_hist = []
        self.ref_hist = []
        self.meas_hist = []

        self.inputs = {}

        self.add_input_port(
            1,
            "ref",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            1,
            "meas",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}

        self.add_output_port(
            1,
            "cmd",
            function=self.h_cmd,
            dependencies=["ref", "meas"],
        )

        self.add_output_port(
            2,
            "x",
            function=self.compute_state,
            dependencies=[],
        )

    def _signals(self, u):
        ref = float(u[0])
        meas = float(u[1])
        e = ref - meas
        return ref, meas, e

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params

        int_e = float(x[0])
        meas_filt = float(x[1])

        _, meas, e = self._signals(u)

        tau = max(float(p["tau"]), 1e-9)

        # Integrator with simple clamp protection.
        d_int_e = e

        if int_e >= p["i_max"] and e > 0.0:
            d_int_e = 0.0
        elif int_e <= p["i_min"] and e < 0.0:
            d_int_e = 0.0

        # First-order measurement filter.
        d_meas_filt = (meas - meas_filt) / tau

        return np.array(
            [
                d_int_e,
                d_meas_filt,
            ],
            dtype=float,
        )

    def h_cmd(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params

        int_e = float(x[0])
        meas_filt = float(x[1])

        ref, meas, e = self._signals(u)

        tau = max(float(p["tau"]), 1e-9)

        # Derivative-on-measurement.
        d_meas_filt = (meas - meas_filt) / tau

        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_meas_filt

        # cmd += ref

        cmd = np.clip(
            cmd,
            p["cmd_min"],
            p["cmd_max"],
        )
        # DEBUG
        # print(f"acc_cmd={cmd}, e={e}")

        # Log signals
        self.t_hist.append(float(t))
        self.ref_hist.append(ref)
        self.meas_hist.append(meas)

        return np.array([cmd], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
