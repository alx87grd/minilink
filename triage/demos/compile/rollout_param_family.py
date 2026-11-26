"""One rollout call for a whole family of pendulum lengths (jax.vmap over params)."""

import matplotlib.pyplot as plt
import numpy as np

from minilink import Pendulum

sys = Pendulum()
sys.params["d"] = 0.0  # undamped, so the swings only differ by their period
sys.x0 = np.array([1.0, 0.0])
ev = sys.compile(backend="jax")

DT = 0.005
N_STEPS = 1200
lengths = np.linspace(0.5, 2.0, 7)
params = dict(sys.params, l=lengths)  # the "l" leaf carries the family
x0s = np.tile(sys.x0, (len(lengths), 1))

xs = np.asarray(ev.rollout_batch(x0s, n_steps=N_STEPS, dt=DT, params=params))
t = DT * np.arange(N_STEPS + 1)

fig, (ax_t, ax_pi) = plt.subplots(1, 2, figsize=(10, 3.5))
for x, length in zip(xs, lengths):
    ax_t.plot(t, x[:, 0], label=f"l = {length:.2f} m")
    tau = t * np.sqrt(sys.params["gravity"] / length)  # Buckingham-pi time
    ax_pi.plot(tau, x[:, 0])
ax_t.set_xlabel("t [s]")
ax_t.set_ylabel("theta [rad]")
ax_t.legend(fontsize=7)
ax_pi.set_xlabel("t * sqrt(g / l)  [-]")
ax_pi.set_title("same swing in dimensionless time")
fig.tight_layout()
plt.show()
