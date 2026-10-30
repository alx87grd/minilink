"""A multi-rate computer on a continuous plant: a 100 Hz filter, a 10 Hz controller, one loop on one clock."""

import numpy as np

from minilink import P, SingleMass, Step, closed_loop
from minilink.blocks import LowPassFilter

TF = 3.0
FILTER_PERIOD = 0.01  # 100 Hz
CONTROLLER_PERIOD = 0.1  # 10 Hz

plant = SingleMass()  # a mass on a spring, its position measured

# The computer: a filter on the measurement, a slower gain on the reference error
loop = closed_loop(
    P(5.0, ports="reference") % CONTROLLER_PERIOD,
    plant,
    reference=Step(np.zeros(1), np.array([0.5]), step_time=0.5),
    filter=LowPassFilter(cutoff_hz=3.0) % FILTER_PERIOD,
)
print(loop)

# The run with the plant's substeps kept: the held command is a state of the loop
traj = loop.compute_trajectory(tf=TF)
loop.plot_trajectory(traj)
