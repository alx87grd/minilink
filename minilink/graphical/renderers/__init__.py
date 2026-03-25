"""Animation backends (matplotlib, meshcat, pygame stub)."""

from minilink.graphical.renderers.base import AnimationRenderer
from minilink.graphical.renderers.matplotlib_renderer import MatplotlibRenderer
from minilink.graphical.renderers.meshcat_renderer import MeshcatRenderer
from minilink.graphical.renderers.pygame_renderer import PygameRenderer

__all__ = [
    "AnimationRenderer",
    "MatplotlibRenderer",
    "MeshcatRenderer",
    "PygameRenderer",
]
