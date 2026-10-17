"""The classical family on the pendulum: P, PI, PD, PID and the rate-measured impedance law, side by side."""

import matplotlib.pyplot as plt

from minilink import PD, PI, PID, ImpedanceController, P, Pendulum, Step, pzmap
from minilink.analysis import step_info

plant = Pendulum()
plant.params["d"] = 2.0  # joint damping, so the undamped laws settle too
r = Step(final_value=[0.5], step_time=1.0)  # the setpoint, 0.5 rad against gravity

# One measurement per axis: the derivative is filtered, a pole at -1/tau.
# The rate measured: the impedance law reads [theta; dtheta] and adds no state.
family = {
    "P": P(Kp=20.0),
    "PI": PI(Kp=20.0, Ki=10.0),
    "PD": PD(Kp=20.0, Kd=2.0, tau=0.05),
    "PID": PID(Kp=20.0, Ki=10.0, Kd=2.0, tau=0.05),
    "Impedance": ImpedanceController(Kp=20.0, Kd=2.0),
}

print(f"{'law':<10}{'closed-loop poles':>19}{'static error':>14}{'overshoot %':>13}")
for name, ctl in family.items():
    loop = r >> ctl @ plant
    _, poles, _ = pzmap(ctl @ plant)
    traj = loop.compute_trajectory(tf=20.0, verbose=False)
    theta = traj.x[
        loop.state_index["plant"][0]
    ]  # the plant's angle, after the law's own states
    info = step_info(traj.t, theta)
    print(
        f"{name:<10}{len(poles):>19}{0.5 - info.steady_state:>14.3f}{info.overshoot:>13.1f}"
    )
    plt.plot(traj.t, theta, label=name)

plt.axhline(0.5, color="k", linestyle=":")
plt.xlabel("t [s]")
plt.ylabel("theta [rad]")
plt.legend()
plt.show()
