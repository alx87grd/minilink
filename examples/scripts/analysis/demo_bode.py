import numpy as np

from minilink.analysis.frequency import bode, pzmap
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

plant = Pendulum()

plant.params["m"] = 1.0
plant.params["l"] = 1.0
plant.params["I"] = 1.0
plant.params["gravity"] = 9.81
plant.params["d"] = 1.0

# output_index = 0  # angle theta
output_index = 1  # velocity dtheta

x_bar = np.array([0.0, 0.0])  # upright equilibrium (unstable for this model)

w, magnitude_db, phase_deg = bode(
    plant,
    x_bar=x_bar,
    input_port="u",
    output_port="y",
    output_index=output_index,
    w=np.logspace(-1, 2, 5),
)
zeros, poles, gain = pzmap(
    plant,
    x_bar=x_bar,
    input_port="u",
    output_port="y",
    output_index=output_index,
)

print(f"Linearized Bode response: y[{output_index}] / u[0]")
print("w [rad/s]   magnitude [dB]   phase [deg]")
for omega, mag, phase in zip(w, magnitude_db, phase_deg):
    print(f"{omega:8.3g}   {mag:14.3f}   {phase:11.2f}")
print()
print("Zero-pole-gain form:")
print("zeros:", np.round(zeros, 4))
print("poles:", np.round(poles, 4))
print("gain:", np.round(gain, 4))

plant.plot_bode(
    x_bar=x_bar,
    input_port="u",
    output_port="y",
    output_index=output_index,
)
plant.plot_pzmap(
    x_bar=x_bar,
    input_port="u",
    output_port="y",
    output_index=output_index,
)
