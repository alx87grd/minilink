import numpy as np

from minilink.blocks.sources import Step, WhiteNoise

step = Step()
step.params["initial_value"] = np.array([0.0])
step.params["final_value"] = np.array([1.0])
step.params["step_time"] = 10.0

step.show_signal(t0=-2.0, tf=12.0)

noise = WhiteNoise(1)
# Baseline
noise.params["t0"] = 0.0
noise.params["tf"] = 10.0
noise.params["sample_period"] = 0.01
noise.params["mean"] = 0.0
noise.params["var"] = 1.0
noise.params["seed"] = 1

fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
ax.set_title("Baseline")

# Change one parameter at a time to visualize each effect.
demo_changes = [
    ("sample_period", 0.05),
    ("sample_period", 0.5),
    ("sample_period", 1.0),
]

for key, value in demo_changes:
    noise.params[key] = value
    noise.refresh()
    fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
    ax.set_title(f"Changed {key} -> {value}")
