"""Plot a first-order low-pass engine torque time response.

This is a standalone matplotlib script. It does not require Minilink.
It shows how the actual engine torque tau_engine follows a commanded
engine torque tau_cmd with first-order dynamics:

    d_tau_engine/dt = (tau_cmd - tau_engine) / tau_engine_tau

Run:
    python engine_lowpass_response.py
"""

import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------
# Engine response parameters
# ---------------------------------------------------------------------
TAU_ENGINE = 0.25  # First-order time constant [s]
TAU_CMD = 260.0  # Commanded engine torque after throttle step [Nm]
TAU_INITIAL = 0.0  # Initial actual engine torque [Nm]

TF = 2.0  # Simulation final time [s]
DT = 0.001  # Simulation time step [s]


# ---------------------------------------------------------------------
# Simulate first-order low-pass response
# ---------------------------------------------------------------------
t = np.arange(0.0, TF + DT, DT)
tau_engine = np.zeros_like(t)
tau_cmd = np.full_like(t, TAU_CMD)

tau_engine[0] = TAU_INITIAL

for k in range(len(t) - 1):
    d_tau = (tau_cmd[k] - tau_engine[k]) / TAU_ENGINE
    tau_engine[k + 1] = tau_engine[k] + DT * d_tau


# Analytical solution for comparison
# tau(t) = tau_cmd + (tau_initial - tau_cmd) * exp(-t / tau)
tau_analytical = TAU_CMD + (TAU_INITIAL - TAU_CMD) * np.exp(-t / TAU_ENGINE)


# ---------------------------------------------------------------------
# Plot response
# ---------------------------------------------------------------------
plt.figure(figsize=(8, 4.5))
plt.plot(t, tau_cmd, "k--", linewidth=1.5, label="Commanded torque")
plt.plot(t, tau_engine, "b-", linewidth=2.0, label="Simulated engine torque")
plt.plot(t, tau_analytical, "r:", linewidth=2.0, label="Analytical response")

# Mark 1, 2, and 3 time constants
for n in [1, 2, 3]:
    tx = n * TAU_ENGINE
    if tx <= TF:
        ty = TAU_CMD * (1.0 - np.exp(-n))
        plt.axvline(tx, color="gray", linestyle=":", linewidth=0.8)
        plt.plot(tx, ty, "ko", markersize=4)
        plt.text(tx, ty, f"  {n}τ", va="bottom")

plt.grid(True, alpha=0.35)
plt.xlabel("Time [s]")
plt.ylabel("Engine torque [Nm]")
plt.title("First-order engine torque response")
plt.legend()
plt.tight_layout()
plt.show()
