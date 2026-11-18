"""Demos whose figures no unit test draws, run with the windows and animations switched off."""

import runpy

import matplotlib.pyplot as plt
import pytest

DEMOS = [
    "examples/demos/value_iteration/vi_quadratic.py",
    "examples/demos/rrt/rrt_holonomic_obstacles.py",
    "examples/demos/mpc/mppi_pendulum.py",
    "examples/projects/mpc/mpc_spatial.py",
    "examples/projects/mpc/mpc_path.py",
]


@pytest.mark.parametrize("path", DEMOS)
def test_demo_figures(path, monkeypatch):
    from minilink.graphical.animation.animator import Animator

    monkeypatch.setattr(plt, "show", lambda *a, **k: None)
    monkeypatch.setattr(plt, "pause", lambda *a, **k: None)
    monkeypatch.setattr(Animator, "animate_simulation", lambda *a, **k: None)
    runpy.run_path(path, run_name="__main__")
