"""Camera customization basics: zoom, 2D projection plane, look-at target, 3D orientation.

Public interface (what this script demonstrates)
-------------------------------------------------

**Minilink library**

- ``sys.camera_target``, ``sys.camera_plot_axes``, ``sys.camera_scale``, ``sys.camera_R``
  — stored on every :class:`~minilink.core.system.System`; the default
  :meth:`~minilink.core.system.System.get_camera_transform` builds
  :func:`~minilink.graphical.primitives.camera_matrix` from these fields.
- ``attach_standard_camera(sys, target=..., plot_axes=..., scale=..., R=...)``
  — updates those same fields (subset of ``camera_matrix`` keyword arguments).
- ``camera_matrix(...)`` — underlying 4×4 factory (import elsewhere if you compose matrices yourself).

**This file only**

- :data:`USER_CAMERA` — optional dict merged into every recipe (same keys as ``camera_matrix``).
- ``apply_demo_camera(sys, **recipe)`` — ``{**USER_CAMERA, **recipe}`` then ``attach_standard_camera``.
- CLI: positional demo name + ``--plane``, ``--renderer``, ``--legacy-loop`` (see commands below).

Quick tweaks:

- Edit :data:`USER_CAMERA` (merged into each recipe via ``attach_standard_camera``), or
- Assign ``sys.camera_scale``, ``sys.camera_target``, ``sys.camera_plot_axes``, or
  ``sys.camera_R`` — the default :meth:`~minilink.core.system.System.get_camera_transform`
  reads these attributes.

:func:`~minilink.graphical.primitives.attach_standard_camera` updates those same fields;
it does not replace ``get_camera_transform``.

Renderer slots (see DESIGN.md §6):

- ``T[:3, 3]`` — look-at target (initial framing for interactive 3D / meshcat; still drives
  **per-frame** orthographic projection in 2D).
- ``T[:3, :3]`` — camera basis (plot horizontal, plot vertical, view-out).
- ``T[3, 3]`` — scale (orthographic half-extent; meshcat eye distance).

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
from typing import Any

import numpy as np

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.graphical.primitives import attach_standard_camera

# --- Defaults merged into every demo (override scale, plot_axes, target, R, …). ---
USER_CAMERA: dict[str, Any] = {}

_AXES_MAP = {
    "xy": (0, 1),
    "xz": (0, 2),
    "yz": (1, 2),
}


def apply_demo_camera(sys: Pendulum, **recipe) -> None:
    """Merge :data:`USER_CAMERA` with *recipe* into ``sys`` camera fields."""
    kw = {**USER_CAMERA, **recipe}
    attach_standard_camera(sys, **kw)


def rotation_yaw_about_world_z(angle_rad: float) -> np.ndarray:
    c = np.cos(angle_rad)
    s = np.sin(angle_rad)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]], dtype=float)


def make_system(*, with_trajectory: bool = True) -> Pendulum:
    sys = Pendulum()
    sys.params["m"] = 1.0
    sys.params["l"] = 5.0
    sys.x0[0] = 2.0
    if with_trajectory:
        sys.compute_trajectory(tf=10.0)
    return sys


def run_zoom(sys: Pendulum, renderer: str, native: bool) -> None:
    """Smaller ``scale`` ⇒ tighter 2D box / closer meshcat eye."""
    apply_demo_camera(sys, scale=3.0)
    sys.animate(renderer=renderer, native=native)


def run_projection(sys: Pendulum, plane: str, renderer: str, native: bool) -> None:
    """Pick world axes for plot horizontal / vertical (2D orthographic projection)."""
    apply_demo_camera(sys, plot_axes=_AXES_MAP[plane], scale=10.0)
    sys.animate(renderer=renderer, native=native)


def run_target(sys: Pendulum, renderer: str, native: bool) -> None:
    """Shift look-at center (world point framed in the view)."""
    apply_demo_camera(sys, target=(2.0, 0.5, 0.0), plot_axes=(0, 1), scale=8.0)
    sys.animate(renderer=renderer, native=native)


def run_orientation_3d(sys: Pendulum, renderer: str, native: bool) -> None:
    """``plot_axes=(0, 2)``: common side view (X horizontal, Z vertical) for ``is_3d``."""
    apply_demo_camera(sys, plot_axes=(0, 2), scale=12.0)
    sys.animate(is_3d=True, renderer=renderer, native=native)


def run_static(sys: Pendulum, plane: str, renderer: str) -> None:
    """Single frame via ``render`` (no trajectory)."""
    apply_demo_camera(sys, plot_axes=_AXES_MAP[plane], scale=8.0)
    u0 = sys.get_u_from_input_ports()
    sys.render(sys.x0, u0, 0.0, is_3d=False, renderer=renderer)


def run_explicit_rotation(sys: Pendulum, renderer: str, native: bool) -> None:
    """Orthonormal ``R`` columns = camera X, Y, Z expressed in world coordinates."""
    R = rotation_yaw_about_world_z(np.deg2rad(28.0))
    apply_demo_camera(sys, R=R, scale=10.0)
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
        help="For `projection` / `static`: world axes as horizontal / vertical.",
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
