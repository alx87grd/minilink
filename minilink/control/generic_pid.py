import numpy as np

from minilink.core.system import DynamicSystem


class PID(DynamicSystem):
    """PID Generic"""

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        tau: float = 0.1,
        meas0: float = 0.0,
        name: str = "PID",
    ):
        super().__init__(2)
        self.name = name

        self.params = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "cmd_min": cmd_min,
            "cmd_max": cmd_max,
            "i_min": i_min,
            "i_max": i_max,
            "tau": tau,
        }
        self.state.labels = ["int_e", "meas_filt"]
        self.x0 = np.array([0.0, meas0], dtype=float)

        self.inputs = {}
        self.add_input_port("ref", nominal_value=np.array([0.0]))
        self.add_input_port("meas", nominal_value=np.array([0.0]))
        self.add_input_port("feedfoward", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(
            "cmd",
            dim=1,
            function=self.h_w,
            dependencies=["ref", "meas", "feedfoward"],
        )
        # self.add_output_port(1, "x", function=self.compute_state, dependencies=[])
        self.add_output_port(
            "logs",
            dim=2,
            function=self.data_signal,
            dependencies=["ref", "meas", "feedfoward"],
        )

        self.add_output_port(
            "pid_int_value",
            dim=4,
            function=self.int_vars,
            dependencies=["ref", "meas", "feedfoward"],
        )

    def data_signal(self, x, u, t=0.0, params=None):
        ref = float(u[0])
        meas = float(u[1])
        return np.array([ref, meas], dtype=float)

    def calculate_error(self, ref: float, meas: float, u) -> float:
        e = ref - meas
        return float(e)

    def compute_cmd(
        self, e: float, int_e: float, d_filt: float, p: dict, feedfoward: float = 0.0
    ) -> float:
        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_filt
        cmd += feedfoward
        return cmd

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas, feedfoward = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)
        tau = max(p["tau"], 1e-3)
        # Dirty derivative on measurement
        d_meas_filt = (meas - meas_filt) / tau

        # Integrator with simple clamp protection.
        d_int_e = e

        # e' = ref' - meas'
        # Si ref' ~= 0; il faut que ref change lentement.
        # e' = 0 - meas' = -meas'
        cmd = self.compute_cmd(e, int_e, d_meas_filt, p=p, feedfoward=feedfoward)
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
        ref, meas, feedfoward = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)

        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau

        cmd = self.compute_cmd(e, int_e, d_filt, p=p, feedfoward=feedfoward)

        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([cmd], dtype=float)

    def int_vars(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e, meas_filt = float(x[0]), float(x[1])
        ref, meas, feedfoward = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)

        tau = max(p["tau"], 1e-3)
        d_filt = (meas - meas_filt) / tau

        cmd = self.compute_cmd(e, int_e, d_filt, p=p, feedfoward=feedfoward)

        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([e, d_filt, int_e, cmd], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, x, u, t):
        return []

class PIDMeas(DynamicSystem):
    """PID Generic"""

    def __init__(
        self,
        Kp: float = 1.0,
        Ki: float = 0.0,
        Kd: float = 0.0,
        cmd_min: float = -np.inf,
        cmd_max: float = np.inf,
        i_min: float = -np.inf,
        i_max: float = np.inf,
        name: str = "PID",
    ):
        super().__init__(1)
        self.name = name

        self.params = {
            "Kp": Kp,
            "Ki": Ki,
            "Kd": Kd,
            "cmd_min": cmd_min,
            "cmd_max": cmd_max,
            "i_min": i_min,
            "i_max": i_max,
        }
        self.state.labels = ["int_e"]
        self.x0 = np.array([0.0], dtype=float)

        self.inputs = {}
        self.add_input_port("ref", nominal_value=np.array([0.0]))
        self.add_input_port("meas", nominal_value=np.array([0.0]))
        self.add_input_port("d_meas", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(
            "cmd",
            dim=1,
            function=self.h_w,
            dependencies=["ref", "meas", "d_meas"],
        )
        # self.add_output_port(1, "x", function=self.compute_state, dependencies=[])
        self.add_output_port(
            "logs",
            dim=3,
            function=self.data_signal,
            dependencies=["ref", "meas", "d_meas"],
        )

        self.add_output_port(
            "pid_int_value",
            dim=4,
            function=self.int_vars,
            dependencies=["ref", "meas", "d_meas"],
        )

    def data_signal(self, x, u, t=0.0, params=None):
        ref = float(u[0])
        meas = float(u[1])
        d_meas = float(u[2])
        return np.array([ref, meas, d_meas], dtype=float)

    def calculate_error(self, ref: float, meas: float, u) -> float:
        e = ref - meas
        return float(e)

    def compute_cmd(
        self, e: float, int_e: float, d_meas: float, p: dict
    ) -> float:
        cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * d_meas
        return cmd

    def f(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e = float(x[0])
        ref, meas, d_meas = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)

        # Integrator with simple clamp protection.
        d_int_e = e

        cmd = self.compute_cmd(e, int_e, d_meas, p=p)
        stop_hi = (cmd >= p["cmd_max"]) and (e > 0)
        stop_lo = (cmd <= p["cmd_min"]) and (e < 0)

        d_int_e = 0.0 if (stop_hi or stop_lo) else e

        if int_e >= p["i_max"] and e > 0.0:
            d_int_e = 0.0
        elif int_e <= p["i_min"] and e < 0.0:
            d_int_e = 0.0

        return np.array([d_int_e], dtype=float)

    def h_w(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e = float(x[0])
        ref, meas, d_meas = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)

        cmd = self.compute_cmd(e, int_e, d_meas, p=p)

        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([cmd], dtype=float)

    def int_vars(self, x, u, t=0.0, params=None):
        p = self.params if params is None else params
        int_e = float(x[0])
        ref, meas, d_meas = float(u[0]), float(u[1]), float(u[2])

        e = self.calculate_error(ref, meas, u)


        cmd = self.compute_cmd(e, int_e, d_meas, p=p)

        cmd = np.clip(cmd, p["cmd_min"], p["cmd_max"])

        return np.array([e, d_meas, int_e, cmd], dtype=float)

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, x, u, t):
        return []
