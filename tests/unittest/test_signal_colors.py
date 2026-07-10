import unittest

from minilink.graphical.signals.signal_colors import (
    INPUT_COLOR,
    INTERNAL_SIGNAL_COLORS,
    STATE_COLOR,
    color_for_signal,
    is_core_input,
    is_core_state,
    is_internal_signal,
    plotly_color,
)


class TestSignalColors(unittest.TestCase):
    def test_core_state_is_blue(self):
        style = color_for_signal("x")
        self.assertEqual(style.color, STATE_COLOR)
        self.assertEqual(style.linewidth, 2.0)

    def test_core_inputs_are_red(self):
        for name in ("u", "u_cmd"):
            style = color_for_signal(name)
            self.assertEqual(style.color, INPUT_COLOR)
            self.assertEqual(style.linewidth, 2.0)

    def test_internal_signals_are_not_red(self):
        for name in ("ctl:u", "y", "r", "plant:dq"):
            style = color_for_signal(name, internal_index=0)
            self.assertNotEqual(style.color, INPUT_COLOR)

    def test_internal_palette_cycles_by_index(self):
        first = color_for_signal("y", internal_index=0)
        second = color_for_signal("r", internal_index=1)
        self.assertEqual(first.color, INTERNAL_SIGNAL_COLORS[0])
        self.assertEqual(second.color, INTERNAL_SIGNAL_COLORS[1])
        self.assertNotEqual(first.color, second.color)

    def test_multi_component_shading_same_hue_different_alpha(self):
        style0 = color_for_signal("x", component=0, n_components=2)
        style1 = color_for_signal("x", component=1, n_components=2)
        self.assertEqual(style0.color, style1.color)
        self.assertLess(style0.alpha, style1.alpha)

    def test_single_component_alpha_is_one(self):
        style = color_for_signal("x", component=0, n_components=1)
        self.assertEqual(style.alpha, 1.0)

    def test_classification_helpers(self):
        self.assertTrue(is_core_state("x"))
        self.assertFalse(is_core_state("u"))
        self.assertTrue(is_core_input("u"))
        self.assertTrue(is_core_input("u_cmd"))
        self.assertFalse(is_core_input("ctl:u"))
        self.assertTrue(is_internal_signal("ctl:u"))
        self.assertFalse(is_internal_signal("x"))

    def test_plotly_color_maps_tab_names(self):
        self.assertEqual(plotly_color("tab:green"), "#2ca02c")
        self.assertEqual(plotly_color("tab:orange"), "#ff7f0e")


if __name__ == "__main__":
    unittest.main()
