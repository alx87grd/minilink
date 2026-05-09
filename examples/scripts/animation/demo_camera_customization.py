"""Camera customization basics: zoom, 2D projection plane, look-at target, 3D orientation.

``DynamicSystem.get_camera_transform(x, u, t)`` returns a standard 4×4 matrix built
with :func:`~minilink.graphical.primitives.camera_matrix`. Renderers read:

- ``T[:3, 3]`` — look-at target (world point at the center of the view for the **initial**
  frame in 3D / meshcat; still drives **per-frame** orthographic projection in 2D).
- ``T[:3, :3]`` — camera basis: plot horizontal, plot vertical, view-out (see DESIGN.md §6).
- ``T[3, 3]`` — **scale**: orthographic half-extent (matplotlib 2D/3D, pygame) or eye
  distance (meshcat).

After playback starts, interactive **matplotlib 3D** and **meshcat** leave orbit/zoom to
the UI; override ``get_camera_transform`` to set the **starting** viewpoint.

Run from the repository root::

    python examples/scripts/animation/demo_camera_customization.py zoom
    python examples/scripts/animation/demo_camera_customization.py projection --plane xz
    python examples/scripts/animation/demo_camera_customization.py target
    python examples/scripts/animation/demo_camera_customization.py orientation_3d
    python examples/scripts/animation/demo_camera_customization.py explicit_rotation
    python examples/scripts/animation/demo_camera_customization.py static --plane xz

Optional::

    python examples/scripts/animation/demo_camera_customization.py zoom --renderer meshcat

"""

from __future__ import annotations

import argparse

import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.primitives import camera_matrix

_AXES_MAP = {
    "xy": (0, 1),
    "xz": (0, 2),
    "yz": (1, 2),
}


def make_system(*, with_trajectory: bool = True) -> Pendulum:
    sys = Pendulum()
    sys.params["m"] = 1.0
    sys.params["l"] = 5.0
    sys.x0[0] = 2.0
    if with_trajectory:
        sys.compute_trajectory(tf=10.0)
    return sys


def run_zoom(sys: Pendulum, renderer: str, native: bool) -> None:
    """Tighter framing via smaller orthographic half-extent / shorter meshcat eye distance."""
    sys.get_camera_transform = lambda x, u, t: camera_matrix(scale=3.0)
    sys.animate(renderer=renderer, native=native)


def run_projection(sys: Pendulum, plane: str, renderer: str, native: bool) -> None:
    """Choose which world axes map to plot horizontal / vertical (2D ortho projection)."""
    axes = _AXES_MAP[plane]
    sys.get_camera_transform = lambda x, u, t: camera_matrix(
        target=(0.0, 0.0, 0.0),
        plot_axes=axes,
        scale=10.0,
    )
    sys.animate(renderer=renderer, native=native)


def run_target(sys: Pendulum, renderer: str, native: bool) -> None:
    """Offset look-at target so the diagram is centered away from the origin."""
    sys.get_camera_transform = lambda x, u, t: camera_matrix(
        target=(2.0, 0.5, 0.0),
        plot_axes=(0, 1),
        scale=8.0,
    )
    sys.animate(renderer=renderer, native=native)


def run_orientation_3d(sys: Pendulum, renderer: str, native: bool) -> None:
    """3D window: initial elev/azim come from ``camera_matrix`` once at scene open.

    ``plot_axes=(0, 2)`` is a common side view (world X horizontal, Z vertical).
    """
    sys.get_camera_transform = lambda x, u, t: camera_matrix(
        target=(0.0, 0.0, 0.0),
        plot_axes=(0, 2),
        scale=12.0,
    )
    sys.animate(is_3d=True, renderer=renderer, native=native)


def run_static(sys: Pendulum, plane: str, renderer: str) -> None:
    """Single frame via :meth:`~minilink.core.system.DynamicSystem.render` (no animation)."""
    axes = _AXES_MAP[plane]
    sys.get_camera_transform = lambda x, u, t: camera_matrix(
        target=(0.0, 0.0, 0.0),
        plot_axes=axes,
        scale=8.0,
    )
    u0 = sys.get_u_from_input_ports()
    sys.render(sys.x0, u0, 0.0, is_3d=False, renderer=renderer)


def run_explicit_rotation(sys: Pendulum, renderer: str, native: bool) -> None:
    """Supply an explicit orthonormal ``R`` (camera X,Y,Z in world) instead of ``plot_axes``."""
    theta = np.deg2rad(28.0)
    c, s = np.cos(theta), np.sin(theta)
    R = np.array(
        [
            [c, -s, 0.0],
            [s, c, 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    sys.get_camera_transform = lambda x, u, t: camera_matrix(
        target=(0.0, 0.0, 0.0),
        R=R,
        scale=10.0,
    )
    sys.animate(renderer=renderer, native=native)


def main() -> None:
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument(
        "demo",
        choices=[
            "zoom",
            "projection",
            "target",
            "orientation_3d",
            "explicit_rotation",
            "static",
        ],
        help="Which recipe to run.",
    )
    parser.add_argument(
        "--plane",
        choices=list(_AXES_MAP.keys()),
        default="xy",
        help="For `projection`: world axes shown as horizontal / vertical.",
    )
    parser.add_argument(
        "--renderer",
        default="matplotlib",
        choices=["matplotlib", "meshcat", "pygame"],
        help="Graphics backend.",
    )
    parser.add_argument(
        "--legacy-loop",
        action="store_true",
        help="Use native=False (Python frame loop) instead of FuncAnimation / meshcat Animation.",
    )
    args = parser.parse_args()
    native = not args.legacy_loop

    sys = make_system(with_trajectory=args.demo != "static")
    if args.demo == "zoom":
        run_zoom(sys, args.renderer, native)
    elif args.demo == "projection":
        run_projection(sys, args.plane, args.renderer, native)
    elif args.demo == "target":
        run_target(sys, args.renderer, native)
    elif args.demo == "orientation_3d":
        run_orientation_3d(sys, args.renderer, native)
    elif args.demo == "static":
        run_static(sys, args.plane, args.renderer)
    else:
        run_explicit_rotation(sys, args.renderer, native)


if __name__ == "__main__":
    main()
