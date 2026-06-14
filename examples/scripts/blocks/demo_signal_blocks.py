"""Parallel signal-processing blocks driven by one source.

Run from the repo root::

    python examples/scripts/blocks/demo_signal_blocks.py

A source fans out to routing (Gain, Sum, Mux), nonlinear (Saturation, DeadZone,
Relay), and filter (LowPass, Notch, Washout) blocks; all outputs are plotted
together. Two inputs are exercised: a step and a sinusoid.
"""

import numpy as np

from minilink.blocks.filters import LowPassFilter, NotchFilter, Washout
from minilink.blocks.nonlinear import DeadZone, Relay, Saturation
from minilink.blocks.routing import Gain, Mux, Sum
from minilink.blocks.sources import Source, Step, TrajectorySource
from minilink.core.diagram import DiagramSystem

OUTPUT_SIGNALS = (
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


def build_parallel_diagram(source, *, bias_value=0.3):
    bias = Source(1)
    bias.params["value"] = np.array([bias_value])

    diagram = DiagramSystem()
    diagram.add_subsystem(source, "src")
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

    for block_id in ("gain", "sat", "deadzone", "relay", "lpf", "notch", "washout"):
        diagram.connect("src", "y", block_id, "u")

    diagram.connect("src", "y", "sum", "in0")
    diagram.connect("bias", "y", "sum", "in1")
    diagram.connect("src", "y", "mux", "in0")
    diagram.connect("bias", "y", "mux", "in1")
    return diagram


def run_demo(source, *, name, tf=20.0, plot_diagram=False):
    diagram = build_parallel_diagram(source)
    diagram.name = name
    if plot_diagram:
        diagram.plot_diagram()
    diagram.compute_trajectory(tf=tf, solver="euler", dt=0.01, show=False)
    diagram.plot_trajectory(signals=OUTPUT_SIGNALS)


step = Step()
step.params["initial_value"] = np.array([0.0])
step.params["final_value"] = np.array([1.0])
step.params["step_time"] = 2.0
run_demo(step, name="Signal blocks (parallel, step)", plot_diagram=True)

t = np.linspace(0.0, 20.0, 2001)
sine = TrajectorySource(t, np.sin(2.0 * np.pi * 0.5 * t))
run_demo(sine, name="Signal blocks (parallel, sine)")
