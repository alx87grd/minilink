import matplotlib.pyplot as plt
import numpy as np

from minilink.core.system import DynamicSystem, System


class Sum(System):
    """SUM."""

    def __init__(self, max=None, min=None, name: str = "Sum"):
        super().__init__(0, 2, 1)

        self.name = name

        self.max = max
        self.min = min

        self.inputs = {}
        self.add_input_port(
            1,
            "1",
            nominal_value=np.array([0.0]),
        )

        self.add_input_port(
            1,
            "2",
            nominal_value=np.array([0.0]),
        )

        self.outputs = {}
        self.add_output_port(
            1,
            "result",
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


# class PID(DynamicSystem):
#     """Generic scalar PID controller with numerical measurement derivative.

#     Inputs
#     ------
#     ref
#         Reference signal.
#     meas
#         Measured signal.

#     Output
#     ------
#     cmd
#         PID command.

#     State
#     -----
#     x = [int_e]

#     Notes
#     -----
#     Error is:

#         e = ref - meas

#     PID law is:

#         cmd = Kp * e + Ki * int_e - Kd * meas_dot

#     where meas_dot is computed numerically from the measured signal:

#         meas_dot = (meas - meas_prev) / (t - t_prev)

#     The derivative is computed on the measurement, not directly on the error.
#     This avoids derivative kick when the reference changes.
#     """

#     def __init__(
#         self,
#         Kp: float = 1.0,
#         Ki: float = 0.0,
#         Kd: float = 0.0,
#         cmd_min: float = -np.inf,
#         cmd_max: float = np.inf,
#         i_min: float = -np.inf,
#         i_max: float = np.inf,
#         name: str = "PID",
#     ):
#         # 1 state:
#         # x[0] = integral of error
#         #
#         # 2 scalar inputs:
#         # ref, meas
#         #
#         # 1 scalar output:
#         # cmd
#         super().__init__(1, 2, 1)

#         self.name = name

#         self.params = {
#             "Kp": float(Kp),
#             "Ki": float(Ki),
#             "Kd": float(Kd),
#             "cmd_min": float(cmd_min),
#             "cmd_max": float(cmd_max),
#             "i_min": float(i_min),
#             "i_max": float(i_max),
#         }

#         self.state.labels = ["int_e"]
#         self.state.units = [""]

#         self.x0 = np.array([0.0], dtype=float)

#         self.t_hist = []
#         self.ref_hist = []
#         self.meas_hist = []

#         # TODO: ENLEVERRRR
#         # For numerical derivative
#         self._prev_t = None
#         self._prev_meas = None
#         self._meas_dot = 0.0

#         self.inputs = {}

#         self.add_input_port(
#             1,
#             "ref",
#             nominal_value=np.array([0.0]),
#         )

#         self.add_input_port(
#             1,
#             "meas",
#             nominal_value=np.array([0.0]),
#         )

#         self.outputs = {}

#         self.add_output_port(
#             1,
#             "cmd",
#             function=self.h_cmd,
#             dependencies=["ref", "meas"],
#         )

#         self.add_output_port(
#             1,
#             "x",
#             function=self.compute_state,
#             dependencies=[],
#         )

#     def _signals(self, u):
#         ref = float(u[0])
#         meas = float(u[1])
#         e = ref - meas
#         return ref, meas, e

#     def f(self, x, u, t=0.0, params=None):
#         p = self.params if params is None else params

#         int_e = float(x[0])

#         _, _, e = self._signals(u)

#         # Integrator with simple clamp protection.
#         d_int_e = e

#         if int_e >= p["i_max"] and e > 0.0:
#             d_int_e = 0.0
#         elif int_e <= p["i_min"] and e < 0.0:
#             d_int_e = 0.0

#         return np.array(
#             [
#                 d_int_e,
#             ],
#             dtype=float,
#         )

#     def h_cmd(self, x, u, t=0.0, params=None):
#         p = self.params if params is None else params

#         int_e = float(x[0])

#         ref, meas, e = self._signals(u)

#         # TODO: FAIRE DERIVER AUTREMENT VOIR PID PYRO POUR NE PAS AVOIR D'ETATS
#         # Numerical derivative of measurement.
#         if self._prev_t is None or self._prev_meas is None:
#             meas_dot = 0.0
#         else:
#             dt = float(t) - self._prev_t

#             if dt > 1e-12:
#                 meas_dot = (meas - self._prev_meas) / dt
#             else:
#                 # Avoid division by zero if h_cmd is called multiple times
#                 # at the same simulation time.
#                 meas_dot = self._meas_dot

#         self._meas_dot = meas_dot
#         self._prev_t = float(t)
#         self._prev_meas = meas

#         # PID command with derivative-on-measurement.
#         cmd = p["Kp"] * e + p["Ki"] * int_e - p["Kd"] * meas_dot

#         cmd = np.clip(
#             cmd,
#             p["cmd_min"],
#             p["cmd_max"],
#         )

#         # Log signals
#         self.t_hist.append(float(t))
#         self.ref_hist.append(ref)
#         self.meas_hist.append(meas)

#         return np.array([cmd], dtype=float)

#     def reset_derivative(self):
#         """Reset the stored values used for the numerical derivative."""
#         self._prev_t = None
#         self._prev_meas = None
#         self._meas_dot = 0.0

#     def get_kinematic_geometry(self):
#         return []

#     def get_kinematic_transforms(self, _x, _u, _t):
#         return []


# def pid_result(pid: PID):
#     t = np.array(pid.t_hist)
#     ref = np.array(pid.ref_hist)
#     meas = np.array(pid.meas_hist)

#     idx = np.argsort(t)
#     t = t[idx]
#     ref = ref[idx]
#     meas = meas[idx]

#     t_unique, unique_idx = np.unique(t, return_index=True)
#     ref = ref[unique_idx]
#     meas = meas[unique_idx]
#     t = t_unique

#     plt.figure()
#     plt.plot(t, ref, label="Reference speed")
#     plt.plot(t, meas, label="Measured speed")
#     plt.xlabel("Time [s]")
#     plt.ylabel("Speed [m/s]")
#     plt.title("Speed tracking")
#     plt.legend()
#     plt.grid(True)
#     plt.show()


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
        super().__init__(2, 2, 1)
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
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(1, "meas", nominal_value=np.array([0.0]))

        self.outputs = {}
        self.add_output_port(
            1,
            "cmd",
            function=self.h_w,
            dependencies=["ref", "meas"],
        )
        # self.add_output_port(1, "x", function=self.compute_state, dependencies=[])
        self.add_output_port(
            2,
            "logs",
            function=self.data_signal,
            dependencies=["ref", "meas"],
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
        d_meas_filt = (meas - meas_filt) / tau

        # Integrator with simple clamp protection.
        d_int_e = e

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

    def get_kinematic_geometry(self):
        return []

    def get_kinematic_transforms(self, _x, _u, _t):
        return []
