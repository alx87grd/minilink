"""Boat parking by the cascade law: a position loop over an LQR velocity loop, with and without current."""

import numpy as np
from parking import CURRENT, CascadeParkingController, ParkingBoat

from minilink.graphical.catalog.boat_skin import boat_skin_3d

TF = 60.0  # [s]
DT = 0.05  # [s]

for current in (np.zeros(2), CURRENT):
    boat = ParkingBoat(current)
    ctl = CascadeParkingController(boat)
    cl = ctl @ boat
    cl.name = f"Cascade parking, current {current} m/s"
    traj = cl.compute_trajectory(tf=TF, dt=DT)
    cl.plot_trajectory(traj)
    cl.animate(traj)

boat.skin = boat_skin_3d  # the last run again, as the hull on the water in the browser
cl.animate(traj, renderer="meshcat", native=False)
