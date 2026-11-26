"""Three-body gravitational dynamics — nonlinear open-loop simulation."""

from minilink import ThreeBody

TF = 30.0
N_SAMPLES = 30000

sys = ThreeBody(preset="figure_eight")

sys.x0[0] = 0.9

traj = sys.compute_trajectory(tf=TF, n_samples=N_SAMPLES)

# sys.plot_trajectory()
sys.animate(renderer="meshcat")
