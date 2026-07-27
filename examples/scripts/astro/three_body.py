"""Three-body gravitational dynamics — nonlinear open-loop simulation.

Run from repo root::

    PYTHONPATH=. MPLBACKEND=Agg python examples/scripts/astro/three_body.py
"""

from minilink.dynamics.catalog.astro.three_body import ThreeBodyProblem

TF = 6.3
N_STEPS = 2000

sys = ThreeBodyProblem(preset="figure_eight")

traj = sys.compute_trajectory(tf=TF, n_steps=N_STEPS, show=False, verbose=False)

sys.plot_trajectory(traj)
sys.animate(traj=traj, show=False)
