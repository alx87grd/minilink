"""Three-body gravitational dynamics — nonlinear open-loop simulation.

Run from repo root::

    PYTHONPATH=. MPLBACKEND=Agg python examples/scripts/astro/three_body.py
"""

from minilink.dynamics.catalog.astro.three_body import ThreeBodyProblem

TF = 30.0
N_STEPS = 30000

sys = ThreeBodyProblem(preset="figure_eight")

sys.x0[0] = 0.9

traj = sys.compute_trajectory(tf=TF, n_steps=N_STEPS)

# sys.plot_trajectory()
sys.animate(renderer="meshcat")
