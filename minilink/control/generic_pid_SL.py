import matplotlib.pyplot as plt
import numpy as np

from minilink.core.system import DynamicSystem, System


class Sum(System):
    """SUM."""

    def __init__(self, max=None, min=None, name: str = "Sum"):
        super().__init__(0)

        self.name = name

        self.max = max
        self.min = min

        self.inputs = {}
        self.add_input_port(
            "1",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            "2",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}
        self.add_output_port(
            "result",
            dim=1,
            function=self.h_meas,
            dependencies=["1", "2"],
        )

    def h_meas(self, x, u, t=0.0, params=None):
        result = u[0] + u[1]

        if self.max is not None and self.min is not None:
            result = np.clip(result, max=self.max, min=self.min)

        return result

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []


class PID(DynamicSystem):
    """PID Generic"""

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        tau: float = 0.1,
        meas0: float = 0.0,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        name: str = "PID",
    ):
        super().__init__(2)
        self.name = name

        self.params = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "tau": tau,
            "cmd_min": cmd_min,
            "cmd_max": cmd_max,
            "i_min": i_min,
            "i_max": i_max,
        }
        self.state.labels = ["int_e", "meas_filt"]
        self.x0 = np.array([0.0, meas0], dtype=float)

        self.inputs = {}
        self.add_input_port("ref", nominal_value=np.array([0.0]))
        self.add_input_port("meas", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(
            "cmd",
            dim=1,
            function=self.h_w,
            dependencies=["ref", "meas"],
        )
        # self.add_output_port(1, "x", function=self.compute_state, dependencies=[])
        self.add_output_port(
            "logs",
            dim=2,
            function=self.data_signal,
            dependencies=["ref", "meas"],
        )

        self.add_output_port(
            "pid_int_value",
            dim=3,
            function=self.int_vars,
            dependencies=[],
        )

    def data_signal(self, x, u, t=0.0, params=None):
        ref = float(u[0])
        meas = float(u[1])
        return np.array([ref, meas], dtype=float)

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas = float(u[0]), float(u[1])

        e = ref - meas
        tau = max(p["tau"], 1e-3)
        # Dirty derivative on measurement
        d_meas_filt = (meas - meas_filt) / tau

        # Integrator with simple clamp protection.
        d_int_e = e

        # e' = ref' - meas'
        # Si ref' ~= 0; il faut que ref change lentement.
        # e' = 0 - meas' = -meas'
        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_meas_filt
        stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
        stop_lo = (cmd <= p["cmd_min"]) and (e < 0)

        d_int_e = 0.0 if (stop_hi or stop_lo) else e

        if int_e >= p["i_max"] and e > 0.0:
            d_int_e = 0.0
        elif int_e <= p["i_min"] and e < 0.0:
            d_int_e = 0.0

        return np.array([d_int_e, d_meas_filt], dtype=float)

    def h_w(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas = float(u[0]), float(u[1])

        e = ref - meas

        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau

        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt

        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([cmd], dtype=float)

    def int_vars(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas = float(u[0]), float(u[1])

        e = ref - meas

        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau

        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt

        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([e, d_filt, int_e], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
