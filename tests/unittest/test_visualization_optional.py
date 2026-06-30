import unittest

import numpy as np
import pytest

from minilink.graphical.animation.primitives import Point
from minilink.graphical.animation.renderers.meshcat_renderer import (
    MeshcatCanvas,
    _import_meshcat,
)
from minilink.graphical.animation.renderers.pygame_renderer import (
    PygameCanvas,
    _import_pygame,
)


def _has_meshcat():
    try:
        _import_meshcat()
    except ImportError:
        return False
    return True


def _has_pygame():
    try:
        _import_pygame()
    except ImportError:
        return False
    return True


class _FakeMeshcatNode:
    """Minimal meshcat path tree for canvas smoke tests (no ZMQ server)."""

    def __init__(self):
        self.children = {}
        self.object = None
        self.transform = None

    def __getitem__(self, key):
        child = self.children.get(key)
        if child is None:
            child = _FakeMeshcatNode()
            self.children[key] = child
        return child

    def set_object(self, obj):
        self.object = obj

    def set_transform(self, transform):
        self.transform = transform

    def delete(self):
        self.children.clear()
        self.object = None
        self.transform = None


@pytest.mark.optional
@pytest.mark.visualization
class TestVisualizationOptionalImports(unittest.TestCase):
    def test_import_meshcat_reports_extra_when_missing(self):
        try:
            meshcat = _import_meshcat()
        except ImportError as exc:
            self.assertIn("minilink[visualization]", str(exc))
        else:
            self.assertTrue(hasattr(meshcat, "Visualizer"))

    def test_import_pygame_reports_extra_when_missing(self):
        try:
            pygame = _import_pygame()
        except ImportError as exc:
            self.assertIn("minilink[visualization]", str(exc))
        else:
            self.assertTrue(hasattr(pygame, "init"))


@pytest.mark.optional
@pytest.mark.visualization
class TestMeshcatOptionalSmoke(unittest.TestCase):
    @pytest.mark.skipif(not _has_meshcat(), reason="meshcat not installed")
    def test_canvas_can_create_point_geometry_without_opening_browser(self):
        canvas = MeshcatCanvas(_FakeMeshcatNode(), is_3d=True)

        canvas.ensure_objects([Point([0.0, 0.0, 0.0])])
        canvas.update_primitive(0, Point([0.0, 0.0, 0.0]), np.eye(4))

        self.assertEqual(canvas._n_slots, 1)
        slot = canvas.scene["p0"]
        self.assertIsNotNone(slot.object)
        self.assertIsNotNone(slot.transform)
        canvas.clear()


@pytest.mark.optional
@pytest.mark.visualization
class TestPygameOptionalSmoke(unittest.TestCase):
    @pytest.mark.skipif(not _has_pygame(), reason="pygame not installed")
    def test_canvas_maps_world_coordinates_to_screen(self):
        pygame = _import_pygame()
        pygame.init()
        try:
            surface = pygame.Surface((100, 100))
            canvas = PygameCanvas(surface, scale=1.0, is_3d=False)
            self.assertEqual(canvas._to_screen(0.0, 0.0), (50, 50))
        finally:
            pygame.quit()

    @pytest.mark.skipif(not _has_pygame(), reason="pygame not installed")
    def test_canvas_draws_point_to_surface(self):
        pygame = _import_pygame()
        pygame.init()
        try:
            surface = pygame.Surface((100, 100))
            canvas = PygameCanvas(surface, scale=1.0, is_3d=False)
            canvas.draw_primitive(Point([0.0, 0.0, 0.0]), np.eye(4), pygame)
        finally:
            pygame.quit()


if __name__ == "__main__":
    unittest.main()
