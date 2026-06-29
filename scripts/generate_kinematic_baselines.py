"""Generate kinematic-rendering PNG baselines for pixel-parity regression.

This is the **source of truth** for the pixel-parity regression: it renders each
catalog plant at three deterministic states through the graphics pipeline and
saves one PNG per (plant, sample) plus a ``manifest.json`` describing the exact
states. ``test_kinematic_regression`` re-renders the same states and compares
pixel-for-pixel against the committed PNGs.

Run from the repo root::

    python scripts/generate_kinematic_baselines.py

Rendering is forced onto the headless matplotlib **Agg** backend at a **fixed DPI**
so the output is deterministic within one environment. Baselines are committed and
must be regenerated only on a deliberate, reviewed visual change.
"""

from __future__ import annotations

import hashlib
import importlib
import importlib.util
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib

# Force the deterministic headless backend before any minilink graphical import.
matplotlib.use("Agg")

import numpy as np  # noqa: E402  (after backend selection, intentionally)

REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_DIR = REPO_ROOT / "tests" / "fixtures" / "kinematic_baseline"
MANIFEST_PATH = FIXTURE_DIR / "manifest.json"

# Saved pixel size is FIGSIZE_ANIMATION (6 x 4.5 in) x BASELINE_DPI.
BASELINE_DPI = 100


@dataclass(frozen=True)
class PlantSpec:
    """A catalog plant to baseline: how to import it and whether it needs an extra."""

    name: str
    module: str
    cls: str
    requires: tuple[str, ...] = ()

    def load(self):
        """Instantiate the plant, or return ``None`` if a required extra is missing."""
        for extra in self.requires:
            if importlib.util.find_spec(extra) is None:
                return None
        module = importlib.import_module(self.module)
        return getattr(module, self.cls)()


# 11 plants x 3 samples. Graphics parity uses NumPy catalog plants only;
# ``DynamicBicycle`` / ``DynamicBicycleCar3D`` cover the vehicle graphics contract.
# (``JaxDynamicBicycleRateInputs`` inherits the same look but is excluded here
# because JAX import order must not affect matplotlib pixel tests.)
_VEHICLES = "minilink.dynamics.catalog.vehicles"
_PENDULUM = "minilink.dynamics.catalog.pendulum"
PLANTS: tuple[PlantSpec, ...] = (
    PlantSpec("dynamic_bicycle", f"{_VEHICLES}.dynamic_bicycle", "DynamicBicycle"),
    PlantSpec(
        "dynamic_bicycle_car3d", f"{_VEHICLES}.dynamic_bicycle", "DynamicBicycleCar3D"
    ),
    PlantSpec("kinematic_bicycle", f"{_VEHICLES}.steering", "KinematicBicycle"),
    PlantSpec(
        "holonomic_mobile_robot", f"{_VEHICLES}.steering", "HolonomicMobileRobot"
    ),
    PlantSpec("pendulum", f"{_PENDULUM}.pendulum", "Pendulum"),
    PlantSpec("cartpole", f"{_PENDULUM}.cartpole", "CartPole"),
    PlantSpec(
        "two_link_manipulator",
        "minilink.dynamics.catalog.manipulators.arms",
        "TwoLinkManipulator",
    ),
    PlantSpec(
        "five_link_planar_manipulator",
        "minilink.dynamics.catalog.manipulators.arms",
        "FiveLinkPlanarManipulator",
    ),
    PlantSpec("drone2d", "minilink.dynamics.catalog.aerial.drone", "Drone2D"),
    PlantSpec(
        "simple_integrator",
        "minilink.dynamics.catalog.equations.integrators",
        "SimpleIntegrator",
    ),
    PlantSpec(
        "single_mass",
        "minilink.dynamics.catalog.mass_spring_damper.linear",
        "SingleMass",
    ),
)


def sample_states(sys):
    """Three deterministic ``(label, x, u, t)`` samples that exercise the geometry.

    Rendering reads only ``get_kinematic_*`` and camera hints resolved by the
    animator (pure trig, never ``f``), so deterministic perturbations off ``x0``
    are a safe, simulator-independent way to pose joints, headings, and steer
    angles.
    """
    n, m = sys.n, sys.m
    x0 = np.asarray(sys.x0, dtype=float).reshape(-1).copy()
    u0 = np.asarray(sys.get_u_from_input_ports(), dtype=float).reshape(-1).copy()
    i_n = np.arange(n, dtype=float)
    i_m = np.arange(m, dtype=float)

    x_a = x0 + 0.4 * np.cos(0.9 * i_n + 0.3)
    x_b = x0 + 0.8 * np.sin(0.6 * i_n + 1.1)
    u_a = u0 + 0.25 * np.cos(0.7 * i_m + 0.2)
    u_b = u0 + 0.5 * np.sin(0.5 * i_m + 0.9)

    return [
        ("rest", x0, u0, 0.0),
        ("pose_a", x_a, u_a, 0.7),
        ("pose_b", x_b, u_b, 1.4),
    ]


def _reset_matplotlib_state() -> None:
    """Return matplotlib to defaults so baselines stay pixel-stable across pytest order."""
    import matplotlib.pyplot as plt

    plt.close("all")
    matplotlib.rcdefaults()


def render_baseline_png(sys, x, u, t, path, *, dpi=BASELINE_DPI):
    """Render one static kinematic frame to ``path`` deterministically (Agg, fixed DPI).

    Uses the same draw path as :meth:`Animator.show` (frame-keyed geometry +
    ``tf`` + dynamic geometry), but saves the figure instead of presenting it.
    """
    _reset_matplotlib_state()
    from minilink.graphical.animation.animator import Animator
    from minilink.graphical.animation.renderers.matplotlib_renderer import (
        MatplotlibRenderer,
    )

    animator = Animator(sys)
    renderer = MatplotlibRenderer(animator)
    kinematic = sys.get_kinematic_geometry()
    frame = animator._resolve_frame(x, u, t, kinematic=kinematic)

    renderer.open_scene(
        is_3d=False, show=False, camera=frame["camera"], title=sys.name
    )
    renderer.draw_frame(frame["primitives"], frame["transforms"], t, frame["camera"])
    renderer.fig.savefig(path, dpi=dpi)
    renderer.close_scene()


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main() -> None:
    FIXTURE_DIR.mkdir(parents=True, exist_ok=True)

    manifest = []
    skipped = []
    for spec in PLANTS:
        sys = spec.load()
        if sys is None:
            skipped.append(spec.name)
            print(f"  skip {spec.name} (missing extra: {', '.join(spec.requires)})")
            continue
        for sample, x, u, t in sample_states(sys):
            file_name = f"{spec.name}__{sample}.png"
            path = FIXTURE_DIR / file_name
            render_baseline_png(sys, x, u, t, path)
            manifest.append(
                {
                    "plant": spec.name,
                    "sample": sample,
                    "module": spec.module,
                    "class": spec.cls,
                    "requires": list(spec.requires),
                    "x": x.tolist(),
                    "u": u.tolist(),
                    "t": float(t),
                    "file": file_name,
                    "sha256": _sha256(path),
                }
            )
            print(f"  wrote {file_name}")

    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"\n{len(manifest)} baselines -> {FIXTURE_DIR.relative_to(REPO_ROOT)}")
    if skipped:
        print(f"skipped (missing extras): {', '.join(skipped)}")


if __name__ == "__main__":
    main()
