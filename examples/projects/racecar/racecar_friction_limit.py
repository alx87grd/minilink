"""Friction limit of the 1/10 racecar: the grip ceiling, then a rear-limited spin."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import DiagramSystem, TrajectorySource
from minilink.catalog import UdeSRacecarDyn

MU_VALUES = (1.0, 0.7, 0.5)
V_TURN_IN = 7.0  # [m/s] speed at turn-in
P_COAST = 15.0  # [W] held during the ramp: less than the drag, so the car slows
STEER_RATE = 0.10  # [rad/s]
TF_RAMP = 6.0  # [s]
N_RAMP = 1201

STEER_RATE_SLOW = 0.02  # [rad/s] slow enough that the yaw rate keeps up with the steer
TF_SLOW = 25.0  # [s]
N_SLOW = 2501

MU_SPIN = 0.5
SPIN_SPEEDS = (3.0, 4.0, 5.0)  # [m/s]
P_HOLD = (7.0, 11.0, 17.0)  # [W] power that holds each speed (notes/parameters.md)
DELTA_CORNER = 0.12  # [rad]
T_STEP = 1.5  # [s] the throttle goes wide open mid-corner
TF_SPIN = 3.0  # [s]
N_SPIN = 601


# Turn-in ramp: the steer angle grows until the tires give up.
ramps = []
for mu in MU_VALUES:
    car = UdeSRacecarDyn()
    car.params["mu"] = mu
    car.x0 = np.array(
        [
            0.0,
            0.0,
            0.0,
            V_TURN_IN,
            0.0,
            0.0,
            V_TURN_IN / car.params["r_r"],
            0.0,
            P_COAST,
        ]
    )

    ramp = DiagramSystem()
    ramp.add_subsystem(TrajectorySource([0.0, TF_RAMP], [[P_COAST, P_COAST]]), "power")
    ramp.add_subsystem(
        TrajectorySource([0.0, TF_RAMP], [[0.0, STEER_RATE * TF_RAMP]]), "steer"
    )
    ramp.add_subsystem(car, "car")
    ramp.connect("power", "y", "car", "P_cmd")
    ramp.connect("steer", "y", "car", "delta_cmd")
    ramps.append(ramp.compute_trajectory(tf=TF_RAMP, n_steps=N_RAMP, verbose=False))

# On the least grippy floor the axles saturate and the slip angles run away.
ramp.plot_trajectory(ramps[-1], signals=("car:slip", "car:grip", "car:imu"), show=False)

# Same corner, wide open throttle halfway through: the rear spends its grip driving.
spins = []
for speed, power in zip(SPIN_SPEEDS, P_HOLD):
    car = UdeSRacecarDyn()
    car.params["mu"] = MU_SPIN
    car.x0 = np.array(
        [0.0, 0.0, 0.0, speed, 0.0, 0.0, speed / car.params["r_r"], DELTA_CORNER, power]
    )

    spin = DiagramSystem()
    spin.add_subsystem(
        TrajectorySource(
            [0.0, T_STEP, TF_SPIN],
            [[power, car.params["P_max"], car.params["P_max"]]],
            interpolation="previous",
        ),
        "power",
    )
    spin.add_subsystem(
        TrajectorySource([0.0, TF_SPIN], [[DELTA_CORNER, DELTA_CORNER]]), "steer"
    )
    spin.add_subsystem(car, "car")
    spin.connect("power", "y", "car", "P_cmd")
    spin.connect("steer", "y", "car", "delta_cmd")
    spins.append(spin.compute_trajectory(tf=TF_SPIN, n_steps=N_SPIN, verbose=False))

# The rear ellipse fills, the rear slip angle takes off and the car spins.
spin.plot_trajectory(spins[-1], signals=("x", "car:slip", "car:grip"), show=False)

# The same corner entered five times more slowly: the steer angle, the yaw rate and the
# sideslip stay in step, which is the condition the understeer gradient is defined under.
car = UdeSRacecarDyn()
car.x0 = np.array(
    [0.0, 0.0, 0.0, V_TURN_IN, 0.0, 0.0, V_TURN_IN / car.params["r_r"], 0.0, P_COAST]
)

quasi_steady = DiagramSystem()
quasi_steady.add_subsystem(
    TrajectorySource([0.0, TF_SLOW], [[P_COAST, P_COAST]]), "power"
)
quasi_steady.add_subsystem(
    TrajectorySource([0.0, TF_SLOW], [[0.0, STEER_RATE_SLOW * TF_SLOW]]), "steer"
)
quasi_steady.add_subsystem(car, "car")
quasi_steady.connect("power", "y", "car", "P_cmd")
quasi_steady.connect("steer", "y", "car", "delta_cmd")
slow_ramp = quasi_steady.compute_trajectory(tf=TF_SLOW, n_steps=N_SLOW, verbose=False)


# --- side analysis: grip ceiling, friction circle and understeer gradient ---
L = car.params["a"] + car.params["b"]
Fz_r = car.params["mass"] * car.params["gravity"] * car.params["a"] / L
fig, axes = plt.subplots(1, 3, figsize=(13.5, 4.2))

for mu, traj in zip(MU_VALUES, ramps):
    car.params["mu"] = mu
    # the plant's own accelerometer: the tire forces over the mass, not v * yaw_rate,
    # which is only the centripetal part of it
    axes[0].plot(traj.x[7], car.imu(traj.x, np.zeros(2))[1] / 9.81, label=f"mu = {mu}")
    axes[0].axhline(mu, color="gray", lw=0.6, ls=":")
axes[0].set_xlabel("steer angle [rad]")
axes[0].set_ylabel("lateral acceleration [g]")
axes[0].set_title("grip ceiling scales with mu")
axes[0].legend()

circle = np.linspace(0.0, 2.0 * np.pi, 181)
axes[1].plot(np.cos(circle), np.sin(circle), color="gray", lw=0.8)
for mu, traj in zip(MU_VALUES, ramps):
    car.params["mu"] = mu
    _, _, Fx_r, Fy_r = car.compute_tire_physics(traj.x[3:6], traj.x[6:8], car.params)
    axes[1].plot(Fx_r / (mu * Fz_r), Fy_r / (mu * Fz_r), lw=1.0, label=f"mu = {mu}")
car.params["mu"] = MU_SPIN
for traj in spins:
    _, _, Fx_r, Fy_r = car.compute_tire_physics(traj.x[3:6], traj.x[6:8], car.params)
    axes[1].plot(Fx_r / (MU_SPIN * Fz_r), Fy_r / (MU_SPIN * Fz_r), lw=0.8, color="k")
axes[1].set_xlabel("rear Fx / mu Fz [-]")
axes[1].set_ylabel("rear Fy / mu Fz [-]")
axes[1].set_title("rear tire inside its friction circle")
axes[1].set_aspect("equal")

car.params["mu"] = MU_VALUES[0]
a_y_g = car.imu(slow_ramp.x, np.zeros(2))[1] / 9.81
delta_extra = slow_ramp.x[7] - L * slow_ramp.x[5] / np.maximum(slow_ramp.x[3], 0.5)
linear = a_y_g < 0.3
K_us = np.polyfit(a_y_g[linear], delta_extra[linear], 1)[0]
axes[2].plot(a_y_g, delta_extra, label="mu = 1.0")
axes[2].plot(
    a_y_g[linear], K_us * a_y_g[linear], "--", label=f"K_us = {K_us:.3f} rad/g"
)
axes[2].set_xlabel("lateral acceleration [g]")
axes[2].set_ylabel("delta - L / R [rad]")
axes[2].set_title("understeer gradient, quasi-steady ramp")
axes[2].legend()

fig.tight_layout()
plt.show()
