"""Parallel signal-processing blocks driven by one step source."""

import numpy as np

from minilink import (
    DeadZone,
    DiagramSystem,
    Gain,
    LowPassFilter,
    Mux,
    Relay,
    Saturation,
    Source,
    Step,
    Sum,
)
from minilink.blocks import NotchFilter, Washout

src = Step(final_value=1.0, step_time=2.0)
bias = Source(1)
bias.params["value"] = np.array([0.3])

diagram = DiagramSystem()
diagram.name = "Signal blocks in parallel"
diagram.add_subsystem(src, "src")
diagram.add_subsystem(bias, "bias")
diagram.add_subsystem(Gain(2.0, dim=1), "gain")
diagram.add_subsystem(Saturation(-0.5, 0.5), "sat")
diagram.add_subsystem(DeadZone(0.2), "deadzone")
diagram.add_subsystem(Relay(0.5), "relay")
diagram.add_subsystem(LowPassFilter(cutoff_hz=0.5), "lpf")
diagram.add_subsystem(NotchFilter(notch_hz=0.5, quality=5.0), "notch")
diagram.add_subsystem(Washout(cutoff_hz=0.5), "washout")
diagram.add_subsystem(Sum(signs=(1.0, -1.0)), "sum")
diagram.add_subsystem(Mux(dims=(1, 1)), "mux")

# Every block reads the same source; sum and mux also take the bias.
for block_id in ("gain", "sat", "deadzone", "relay", "lpf", "notch", "washout"):
    diagram.connect("src", "y", block_id, "u")
diagram.connect("src", "y", "sum", "in0")
diagram.connect("bias", "y", "sum", "in1")
diagram.connect("src", "y", "mux", "in0")
diagram.connect("bias", "y", "mux", "in1")

diagram.plot_diagram()

# Relay and dead-zone switch: fixed-step Euler keeps the discontinuities honest.
diagram.compute_trajectory(tf=20.0, solver="euler", dt=0.01)
diagram.plot_trajectory(
    signals=(
        "src:y",
        "gain:y",
        "sat:y",
        "deadzone:y",
        "relay:y",
        "lpf:y",
        "notch:y",
        "washout:y",
        "sum:y",
        "mux:y",
    )
)
# For a sine drive, swap the source: TrajectorySource(t, np.sin(2 * np.pi * 0.5 * t)).
