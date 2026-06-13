"""Manual demo: exercise every block and tool added in the pyro→minilink
tool migration. Run from the repo root::

    python examples/scripts/blocks/demo_signal_blocks.py

Sections:
1. Routing blocks (Sum, Gain, Mux, Demux) — algebraic checks.
2. Nonlinear blocks (Saturation, DeadZone, Relay) — input/output curves.
3. Filter blocks (LowPass, Notch, Washout) — frequency response.
4. TrajectorySource — replay a stored signal.
5. Control laws (ProportionalController, PIDController, LinearFeedbackController).
6. Analysis + LQR (linearize, controllability/observability, equilibria, lqr).
"""

import matplotlib.pyplot as plt
import numpy as np

from minilink.analysis.equilibria import find_equilibrium
from minilink.analysis.linearize import linearize
from minilink.analysis.structural import controllability, observability
from minilink.blocks.filters import LowPassFilter, NotchFilter, Washout
from minilink.blocks.nonlinear import DeadZone, Relay, Saturation
from minilink.blocks.routing import Demux, Gain, Mux, Sum
from minilink.blocks.sources import TrajectorySource
from minilink.control.linear import PIDController, ProportionalController
from minilink.control.lqr import lqr
from minilink.core.diagram import DiagramSystem
from minilink.dynamics.catalog.equations.integrators import DoubleIntegrator
from minilink.dynamics.catalog.pendulum.pendulum import InvertedPendulum, Pendulum

# 1. Routing -----------------------------------------------------------------
print("\n=== 1. Routing blocks ===")
print("Sum [5,2] (1,-1):", Sum().outputs["y"].compute(None, np.array([5.0, 2.0])))
print(
    "Gain diag[2,3] @ [1,1]:",
    Gain([2.0, 3.0]).outputs["y"].compute(None, np.array([1.0, 1.0])),
)
print(
    "Mux([1,2],[9]):", Mux((2, 1)).outputs["y"].compute(None, np.array([1.0, 2.0, 9.0]))
)
demux = Demux((2, 1))
u = np.array([1.0, 2.0, 9.0])
print(
    "Demux out0/out1:",
    demux.outputs["out0"].compute(None, u),
    demux.outputs["out1"].compute(None, u),
)


# 2. Nonlinear input/output curves -------------------------------------------
grid = np.linspace(-3.0, 3.0, 400)
blocks = {
    "Saturation(-1, 1)": Saturation(-1.0, 1.0),
    "DeadZone(0.5)": DeadZone(0.5),
    "Relay(1.0)": Relay(1.0),
}
fig1, axes = plt.subplots(1, 3, figsize=(11, 3.2))
for ax, (label, block) in zip(axes, blocks.items()):
    y = [block.compute(None, np.array([v]))[0] for v in grid]
    ax.plot(grid, grid, "k--", alpha=0.3, label="identity")
    ax.plot(grid, y, "b", linewidth=2)
    ax.set_title(label)
    ax.set_xlabel("input")
    ax.grid(alpha=0.3)
axes[0].set_ylabel("output")
fig1.suptitle("2. Nonlinear blocks")
fig1.tight_layout()


# 3. Filter frequency response -----------------------------------------------
def magnitude(lti, freqs_hz):
    A, B, C, D = lti.A(), lti.B(), lti.C(), lti.D()
    n = A.shape[0]
    mags = []
    for f in freqs_hz:
        s = 2j * np.pi * f
        H = C @ np.linalg.solve(s * np.eye(n) - A, B) + D
        mags.append(abs(H[0, 0]))
    return np.array(mags)


freqs = np.logspace(-2, 2, 400)
fig2, ax2 = plt.subplots(figsize=(7, 3.5))
for name, filt in [
    ("LowPass 1 Hz", LowPassFilter(cutoff_hz=1.0)),
    ("Notch 1 Hz", NotchFilter(notch_hz=1.0, quality=5.0)),
    ("Washout 1 Hz", Washout(cutoff_hz=1.0)),
]:
    ax2.semilogx(
        freqs, 20.0 * np.log10(magnitude(filt, freqs)), linewidth=2, label=name
    )
ax2.axvline(1.0, color="k", linestyle=":", alpha=0.4)
ax2.set_title("3. Filter blocks — magnitude response")
ax2.set_xlabel("frequency [Hz]")
ax2.set_ylabel("|H| [dB]")
ax2.set_ylim(-40, 5)
ax2.grid(alpha=0.3, which="both")
ax2.legend()
fig2.tight_layout()


# 4. TrajectorySource --------------------------------------------------------
t_samples = np.linspace(0.0, 10.0, 11)
src = TrajectorySource(t_samples, np.sin(t_samples))
t_fine = np.linspace(-1.0, 11.0, 500)
replayed = [src.h(np.array([]), np.array([]), ti)[0] for ti in t_fine]
fig3, ax3 = plt.subplots(figsize=(7, 3.0))
ax3.plot(t_samples, np.sin(t_samples), "ro", label="samples")
ax3.plot(t_fine, replayed, "b", label="replayed (clamped outside)")
ax3.set_title("4. TrajectorySource")
ax3.set_xlabel("t [s]")
ax3.grid(alpha=0.3)
ax3.legend()
fig3.tight_layout()


# 5. Control laws ------------------------------------------------------------
print("\n=== 5. Control laws ===")

# ProportionalController works with the @ operator (output feedback on y).
# Pendulum's output y = [theta, dtheta], so the gain is (1, 2) and r defaults to 0.
prop_plant = Pendulum()
prop_plant.x0 = np.array([2.0, 0.0])
prop = ProportionalController([[200.0, 20.0]])
prop_loop = prop @ prop_plant
prop_loop.compute_trajectory(tf=8.0, n_steps=801)
prop_loop.plot_trajectory()

# PIDController removes steady-state error on a double integrator.
pid = PIDController()
pid.params.update({"Kp": 5.0, "Ki": 1.0, "Kd": 4.0})
pid.inputs["r"].nominal_value = np.array([1.0])  # setpoint
pid_loop = DiagramSystem()
pid_loop.add_subsystem(pid, "pid")
pid_loop.add_subsystem(DoubleIntegrator(), "plant")
pid_loop.connect("plant", "x", "pid", "y")
pid_loop.connect("pid", "u", "plant", "u")
pid_loop.compute_trajectory(tf=40.0, n_steps=2001)
print("PID final position (target 1.0):", round(float(pid_loop.traj.x[1, -1]), 4))
pid_loop.plot_trajectory()


# 6. Analysis + LQR ----------------------------------------------------------
print("\n=== 6. Analysis + LQR ===")
plant = InvertedPendulum()
lti = linearize(plant, x_bar=[0.0, 0.0])
print("A =\n", np.round(lti.A(), 4))
print("controllable:", controllability(lti.A(), lti.B()).is_full_rank)
print("observable:", observability(lti.A(), lti.C()).is_full_rank)
print(
    "hanging equilibrium of Pendulum:",
    np.round(find_equilibrium(Pendulum(), [0.3, 0.0]), 4),
)

controller = lqr(
    lti.A(), lti.B(), Q=np.diag([10.0, 1.0]), R=[[1.0]], xbar=[0.0, 0.0], ubar=[0.0]
)
lqr_loop = DiagramSystem()
lqr_loop.add_subsystem(controller, "lqr")
lqr_loop.add_subsystem(plant, "plant")
lqr_loop.connect("plant", "x", "lqr", "x")
lqr_loop.connect("lqr", "u", "plant", "u")
plant.x0 = np.array([0.4, 0.0])
lqr_loop.compute_trajectory(tf=8.0, n_steps=801)
print("LQR final state (target [0,0]):", np.round(lqr_loop.traj.x[:, -1], 4))
lqr_loop.plot_trajectory()

plt.show()
