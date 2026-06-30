"""Regenerate ``manifest.json`` after extending the kinematic render-smoke plant set.

Writes PNGs locally (gitignored) for optional visual review. Only the manifest
is committed; CI uses ``test_kinematic_regression`` render smoke, not pixels.

Run from the repo root::

    python tests/fixtures/kinematic_baseline/regenerate_manifest.py
"""

from __future__ import annotations

import importlib
import importlib.util
import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[3]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from tests.fixtures.kinematic_baseline.render import render_baseline_png  # noqa: E402

FIXTURE_DIR = Path(__file__).resolve().parent
MANIFEST_PATH = FIXTURE_DIR / "manifest.json"


@dataclass(frozen=True)
class PlantSpec:
    name: str
    module: str
    cls: str
    requires: tuple[str, ...] = ()

    def load(self):
        for extra in self.requires:
            if importlib.util.find_spec(extra) is None:
                return None
        module = importlib.import_module(self.module)
        return getattr(module, self.cls)()


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


def main() -> None:
    manifest = []
    skipped = []
    for spec in PLANTS:
        sys = spec.load()
        if sys is None:
            skipped.append(spec.name)
            print(f"  skip {spec.name} (missing extra: {', '.join(spec.requires)})")
            continue
        for sample, x, u, t in sample_states(sys):
            png_name = f"{spec.name}__{sample}.png"
            path = FIXTURE_DIR / png_name
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
                }
            )
            print(f"  wrote {png_name}")

    MANIFEST_PATH.write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"\n{len(manifest)} entries -> {MANIFEST_PATH.relative_to(Path.cwd())}")
    if skipped:
        print(f"skipped (missing extras): {', '.join(skipped)}")


if __name__ == "__main__":
    main()
