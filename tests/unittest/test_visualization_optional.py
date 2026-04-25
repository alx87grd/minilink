"""Optional visualization dependency smoke tests."""

import unittest

import numpy as np
import pytest

from minilink.graphical.primitives import Point

try:
    import meshcat  # noqa: F401

    from minilink.graphical.renderers.meshcat_renderer import (
        MeshcatCanvas,
        MeshcatRenderer,
        _color_to_meshcat_hex,
    )

    HAS_MESHCAT = True
except ImportError:
    HAS_MESHCAT = False

try:
    import pygame  # noqa: F401

    from minilink.graphical.renderers.pygame_renderer import (
        PygameCanvas,
        PygameRenderer,
        _color_to_rgb,
    )

    HAS_PYGAME = True
except ImportError:
    HAS_PYGAME = False


class _DummyAnimator:
    domain = ((-1.0, 1.0), (-1.0, 1.0))

    class _System:
        name = "dummy"

    sys = _System()


@pytest.mark.optional
@pytest.mark.visualization
@unittest.skipUnless(HAS_MESHCAT, "meshcat not installed")
class TestMeshcatVisualization(unittest.TestCase):
    def test_color_conversion_and_renderer_lifecycle(self):
        self.assertEqual(_color_to_meshcat_hex("red"), 0xFF0000)
        renderer = MeshcatRenderer(_DummyAnimator())
        renderer.open_scene(is_3d=False, show=False)
        self.assertIsNotNone(renderer.vis)
        self.assertIsInstance(renderer.canvas, MeshcatCanvas)
        renderer.close_scene()
        self.assertIsNone(renderer.vis)
        self.assertIsNone(renderer.canvas)


@pytest.mark.optional
@pytest.mark.visualization
@unittest.skipUnless(HAS_PYGAME, "pygame not installed")
class TestPygameVisualization(unittest.TestCase):
    def test_color_conversion_and_headless_draw(self):
        self.assertEqual(_color_to_rgb("red"), (255, 0, 0))
        renderer = PygameRenderer(_DummyAnimator())
        renderer.open_scene(is_3d=False, show=False)
        renderer.draw_frame([Point([0.0, 0.0, 0.0])], [np.eye(4)], t=0.0)
        self.assertEqual(renderer.poll_events(), {"quit": False})
        renderer.close_scene()
        self.assertIsNone(renderer.pygame)

    def test_canvas_world_to_screen_center(self):
        pygame_mod = __import__("pygame")
        pygame_mod.init()
        try:
            surface = pygame_mod.Surface((800, 600))
            canvas = PygameCanvas(surface, _DummyAnimator(), is_3d=False)
            self.assertEqual(canvas._to_screen(0.0, 0.0), (400, 300))
        finally:
            pygame_mod.quit()
