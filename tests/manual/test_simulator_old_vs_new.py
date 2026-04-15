

import numpy as np


from minilink.blocks.examples import Pendulum
from minilink.core.analysis import Simulator as OldSimulator
from minilink.simulation import Simulator as NewSimulator


Pendulum = Pendulum()
Pendulum.params["m"] = 1.0
Pendulum.params["l"] = 1.0
Pendulum.x0[0] = 2.0

old_sim = OldSimulator(Pendulum, t0=0, tf=10, dt=0.01, solver="euler")
old_traj = old_sim.solve()

new_sim = NewSimulator(Pendulum, t0=0, tf=10, dt=0.01, solver="euler")
new_traj = new_sim.solve()

print(old_traj)
print(new_traj)