"""Multi-rate filter + controller via :class:`~minilink.simulation.computer.Computer`.

Fast measurement filter (100 Hz) and slow P controller (10 Hz) on a discrete
plant, all clocked on a 100 Hz base tick. The controller output **holds** between
fires; the filter runs every tick.

Run from the repo root::

    python examples/scripts/step/demo_computer_multi_rate.py
"""

import numpy as np

from minilink.control.output import ProportionalController
from minilink.core.backends import array_module
from minilink.core.diagram import StepDiagramSystem
from minilink.core.system import StepSystem
from minilink.simulation.computer import Computer, StepSchedule

DT_BASE = 0.01  # 100 Hz base tick
N_STEPS = 160


class DiscreteLowPass(StepSystem):
    """First-order low-pass: ``x_{k+1} = α x_k + (1-α) u_k``, ``y = x``."""

    def __init__(self, alpha=0.25):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        alpha = params["alpha"]
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


class DiscretePlant(StepSystem):
    """Simple integrator plant ``x_{k+1} = x_k + u_k``."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        x = xp.asarray(x, dtype=float).reshape(1)
        u = xp.asarray(u, dtype=float).reshape(1)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        xp = array_module(x)
        return xp.asarray(x, dtype=float).reshape(1).copy()


def reference(k):
    """Step reference on the boundary input."""
    return np.array([1.0 if k < 60 else 0.0])


def build_diagram():
    diagram = StepDiagramSystem()
    diagram.add_subsystem(DiscreteLowPass(alpha=0.3), "filter")
    diagram.add_subsystem(ProportionalController(0.35), "ctl")
    diagram.add_subsystem(DiscretePlant(), "plant")

    diagram.add_input_port("r")
    diagram.connect("input", "r", "ctl", "r")
    diagram.connect("plant", "y", "filter", "u")
    diagram.connect("filter", "y", "ctl", "y")
    diagram.connect("ctl", "u", "plant", "u")

    diagram.connect_new_output_port("plant", "y", "y")
    diagram.connect_new_output_port("filter", "y", "y_f")
    diagram.connect_new_output_port("ctl", "u", "u_cmd")
    return diagram


def run_computer(diagram, schedule, n_steps):
    computer = Computer(diagram, schedule)
    computer.compile()
    computer.reset()

    k_axis = []
    r_hist = []
    y_hist = []
    y_f_hist = []
    u_hist = []
    ctl_fire = []

    ctl_div = computer.divisor("ctl")
    for _ in range(n_steps):
        k = computer.k
        r = reference(k)
        outs = computer.tick(r)

        k_axis.append(k)
        r_hist.append(float(r[0]))
        y_hist.append(float(outs["y"][0]))
        y_f_hist.append(float(outs["y_f"][0]))
        u_hist.append(float(outs["u_cmd"][0]))
        if k % ctl_div == 0:
            ctl_fire.append(k)

    return {
        "k": np.array(k_axis, dtype=float),
        "r": np.array(r_hist),
        "y": np.array(y_hist),
        "y_f": np.array(y_f_hist),
        "u": np.array(u_hist),
        "ctl_fire": np.array(ctl_fire, dtype=float),
    }


def plot_results(data, schedule):
    import matplotlib.pyplot as plt

    k = data["k"]

    fig, axes = plt.subplots(2, 1, figsize=(9, 6), sharex=True)

    ax = axes[0]
    ax.step(k, data["r"], where="post", label="r (reference)", color="C0")
    ax.plot(k, data["y"], label="y plant (raw)", color="C1", alpha=0.85)
    ax.plot(k, data["y_f"], label="y filter (100 Hz)", color="C2", linewidth=1.5)
    for k_fire in data["ctl_fire"]:
        ax.axvline(k_fire, color="0.75", linewidth=0.6, linestyle=":")
    ax.set_ylabel("Output")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)
    ax.set_title(
        "Computer multi-rate: filter @ 100 Hz, controller @ 10 Hz "
        f"(dt_base={schedule.dt_base} s; dotted = controller fire)"
    )

    ax = axes[1]
    ax.step(k, data["u"], where="post", label="u_cmd (holds @ 10 Hz)", color="C3")
    for k_fire in data["ctl_fire"]:
        ax.axvline(k_fire, color="0.75", linewidth=0.6, linestyle=":")
    ax.set_xlabel("Step k")
    ax.set_ylabel("Control")
    ax.legend(loc="upper right")
    ax.grid(True, alpha=0.3)

    sec = axes[0].secondary_xaxis(
        "top",
        functions=(lambda x: x * schedule.dt_base, lambda t: t / schedule.dt_base),
    )
    sec.set_xlabel("Time [s] (metadata from dt_base)")

    fig.tight_layout()
    plt.show()


diagram = build_diagram()
diagram.plot_diagram()

schedule = StepSchedule.from_rates(
    dt_base=DT_BASE,
    rates_hz={"filter": 100.0, "ctl": 10.0, "plant": 100.0},
)
data = run_computer(diagram, schedule, N_STEPS)
plot_results(data, schedule)
