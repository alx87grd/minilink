"""Step and white-noise sources, shown as signals."""

from minilink import Step, WhiteNoise

step = Step(final_value=1.0, step_time=10.0)
step.show_signal(t0=-2.0, tf=12.0)

noise = WhiteNoise(1, psd=0.01, sample_period=0.01, seed=1)

fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
ax.set_title("Baseline")

# Change the sample period to visualize its effect: the intensity stays.
demo_changes = [
    ("sample_period", 0.05),
    ("sample_period", 0.5),
    ("sample_period", 1.0),
]

for key, value in demo_changes:
    noise.params[key] = value
    fig, ax = noise.show_signal(t0=-2.0, tf=12.0)
    ax.set_title(f"Changed {key} -> {value}")
