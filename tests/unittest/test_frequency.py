"""Tests for frequency-response analysis."""

import numpy as np
import pytest

from minilink.analysis.frequency import bode, pzmap
from minilink.core.backends import array_module
from minilink.core.diagram import DiagramSystem
from minilink.core.system import DynamicSystem
from minilink.graphical.common import PlotResult


class FrequencyPlant(DynamicSystem):
    def __init__(self):
        super().__init__(n=1)
        self.name = "Frequency Plant"
        self.add_input_port("force", dim=2, labels=["left_force", "right_force"])
        self.add_input_port("bias", labels=["bias"])
        self.add_output_port(
            "y",
            dim=2,
            function=self.h,
            dependencies="all",
            labels=["position", "speed"],
        )
        self.add_output_port(
            "sensor",
            dim=1,
            function=self.sensor,
            dependencies=("bias",),
            labels=["sensor"],
        )

    def f(self, x, u, t=0, params=None):
        force, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)

        # dx = -2 x + 3 force0 + 5 force1 + 7 bias
        dx = xp.array([-2.0 * x[0] + 3.0 * force[0] + 5.0 * force[1] + 7.0 * bias[0]])
        return dx

    def h(self, x, u, t=0, params=None):
        force, _ = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)

        # y = [x + 11 force0, 2 x + 13 force1]
        y = xp.array([x[0] + 11.0 * force[0], 2.0 * x[0] + 13.0 * force[1]])
        return y

    def sensor(self, x, u, t=0, params=None):
        _, bias = self.get_port_values_from_u(u, "force", "bias")
        xp = array_module(x, u)

        # sensor = 4 x + 17 bias
        y = xp.array([4.0 * x[0] + 17.0 * bias[0]])
        return y


def build_frequency_diagram():
    diagram = DiagramSystem()
    diagram.connection_verbose = False
    diagram.add_subsystem(FrequencyPlant(), "plant")
    diagram.add_input_port("force", dim=2)
    diagram.add_input_port("bias")
    diagram.connect("input", "force", "plant", "force")
    diagram.connect("input", "bias", "plant", "bias")
    return diagram


def test_bode_selects_named_port_and_component():
    plant = FrequencyPlant()
    w, mag, phase = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0],
    )

    G = 13.0 + 10.0 / (2.0 + 1.0j)
    np.testing.assert_allclose(w, [1.0])
    np.testing.assert_allclose(mag, [20.0 * np.log10(abs(G))], atol=1e-6)
    np.testing.assert_allclose(phase, [np.angle(G) * 180.0 / np.pi], atol=1e-6)


def test_bode_selects_nonprimary_output_port():
    plant = FrequencyPlant()
    _, mag, phase = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="bias",
        output_port="sensor",
        w=[1.0],
    )

    G = 17.0 + 28.0 / (2.0 + 1.0j)
    np.testing.assert_allclose(mag, [20.0 * np.log10(abs(G))], atol=1e-6)
    np.testing.assert_allclose(phase, [np.angle(G) * 180.0 / np.pi], atol=1e-6)


def test_bode_selects_internal_diagram_output_port():
    diagram = build_frequency_diagram()
    _, mag, phase = bode(
        diagram,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="bias",
        output_port=("plant", "sensor"),
        w=[1.0],
    )

    G = 17.0 + 28.0 / (2.0 + 1.0j)
    np.testing.assert_allclose(mag, [20.0 * np.log10(abs(G))], atol=1e-6)
    np.testing.assert_allclose(phase, [np.angle(G) * 180.0 / np.pi], atol=1e-6)


def test_pzmap_returns_zeros_poles_and_gain_for_selected_channel():
    plant = FrequencyPlant()
    zeros, poles, gain = pzmap(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
    )

    np.testing.assert_allclose(zeros, [-36.0 / 13.0], atol=1e-6)
    np.testing.assert_allclose(poles, [-2.0], atol=1e-6)
    np.testing.assert_allclose(gain, 13.0, atol=1e-6)


@pytest.mark.optional
@pytest.mark.jax
def test_bode_jax_matches_fd_for_selected_channel():
    pytest.importorskip("jax")

    plant = FrequencyPlant()
    fd = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0, 10.0],
        method="fd",
    )
    exact = bode(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0, 10.0],
        method="jax",
    )

    for fd_array, exact_array in zip(fd, exact):
        np.testing.assert_allclose(fd_array, exact_array, atol=1e-6)


@pytest.mark.optional
@pytest.mark.jax
def test_pzmap_jax_matches_fd_for_selected_channel():
    pytest.importorskip("jax")

    plant = FrequencyPlant()
    fd = pzmap(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        method="fd",
    )
    exact = pzmap(
        plant,
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        method="jax",
    )

    for fd_value, exact_value in zip(fd, exact):
        np.testing.assert_allclose(fd_value, exact_value, atol=1e-6)


def test_plot_bode_facade_returns_plot_result():
    import matplotlib.pyplot as plt

    plant = FrequencyPlant()
    result = plant.plot_bode(
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        w=[1.0, 10.0],
        show=False,
    )

    assert isinstance(result, PlotResult)
    assert len(result.axes) == 2
    assert "y[1] / force[1]" in result.axes[0].get_ylabel()
    plt.close(result.figure)


def test_plot_pzmap_facade_returns_plot_result():
    import matplotlib.pyplot as plt

    plant = FrequencyPlant()
    result = plant.plot_pzmap(
        x_bar=[0.0],
        u_bar=[0.0, 0.0, 0.0],
        input_port="force",
        input_index=1,
        output_port="y",
        output_index=1,
        show=False,
    )

    assert isinstance(result, PlotResult)
    assert result.axes is not None
    assert "y[1] / force[1]" in result.axes.get_title()
    plt.close(result.figure)
