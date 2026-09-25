# UdeS GRO501

Notebooks for the GRO501 code exercises (notes C5.1, C5.3, C5.4, C5.5).
C5.2 is Dorf CP11.1–11.3 using the C5.1 tutorial; it has no starter notebook.

C5.1 (`numpy_state_space`) is the same mass–spring example three times:
NumPy/SciPy, python-control, then minilink (`SingleMass`; no `place()` or
Kalman yet). C5.3 / C5.4 keep the Colab design cells and use minilink for
the plant, controllers, simulate, and plot/animate. C5.5
(`racecar_toward_mpc`) is optional: from the LQR program to a finite-horizon
MPC program with a non-quadratic cost and actuator bounds (JAX + SciPy).

`siso_transfer_function_analysis` is not a numbered exercise: it is the
old MATLAB template *Analyse asservissement SISO* (plant, step / pzmap /
bode, PID, L = C H, rlocus / bode, closed loop) redone three times,
NumPy / SciPy, python-control, then minilink.
`ode_simulation` is its simulation twin and a template: a
mass-spring with a cubic damper written as x' = f(x, u, t), an output y = h(x, u, t) = x, an input signal u(t),
a reference r(t) and a state-feedback law ctl(y, r, t), simulated in open loop and
in closed loop with SciPy `solve_ivp`, then with custom minilink blocks
(`DynamicSystem`, `System`, `Controller`).

Do not rename, move, or delete a file here unless the Notes-Commande
`\colab{}` changes in the same commit. For now that PDF is the only
external pointer to these notebooks (no GRO501 course webpage).
