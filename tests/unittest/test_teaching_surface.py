"""The teaching surface is a contract (ROADMAP §2).

One explicit list of the names students meet, tested as a set: every name
resolves through its band facade, carries a docstring, and lives in a
teaching-lane module. A clean-environment smoke (optional packages blocked)
protects the Basic tier: NumPy + SciPy + Matplotlib run simulation, plots,
animation, linearization, LQR, and value iteration.
"""

from __future__ import annotations

import importlib
import os
import subprocess
import sys
import unittest

# band -> names.  Adding a name here is the "entry gate" of ROADMAP §2 (soft
# rule: a demo or notebook, a both-backends test where it defines dynamics,
# and a docstring).  Removing one is a deprecation.
TEACHING_SURFACE: dict[str, tuple[str, ...]] = {
    "minilink": (
        "System",
        "DynamicSystem",
        "StepSystem",
        "DiagramSystem",
        "Pendulum",
        "InvertedPendulum",
        "CartPole",
        "ImpedanceController",
        "lqr",
        "Step",
        "feedback",
    ),
    "minilink.core": (
        "System",
        "DynamicSystem",
        "DiagramSystem",
        "Controller",
        "Trajectory",
        "CostFunction",
        "QuadraticCost",
        "BoxSet",
        "BoxInputSet",
        "BallSet",
        "TimeCost",
        "StepDiagramSystem",
        "closed_loop_qdq",
    ),
    "minilink.blocks": (
        "Integrator",
        "Step",
        "WhiteNoise",
        "TrajectorySource",
        "Sum",
        "Error",
        "Gain",
        "Mux",
        "Demux",
        "Saturation",
        "DeadZone",
        "Relay",
        "LowPassFilter",
        "TransferFunction",
        "Lead",
        "Lag",
        "Source",
        "ZOHHold",
    ),
    "minilink.control": (
        "ProportionalController",
        "StateFeedbackController",
        "PID",
        "PI",
        "PD",
        "ImpedanceController",
        "JointImpedance",
        "TaskImpedance",
        "ComputedTorqueController",
        "SlidingModeController",
        "ImpedanceIntegralController",
    ),
    "minilink.analysis": (
        "jacobian",
        "transfer_function",
        "bode",
        "plot_bode",
        "pzmap",
        "plot_pzmap",
        "margins",
        "nyquist",
        "plot_nyquist",
        "root_locus",
        "plot_root_locus",
        "step_response",
        "plot_step_response",
        "modal_analysis",
        "controllability",
        "observability",
        "find_equilibrium",
    ),
    "minilink.simulation": ("Simulator", "StaticSimulator"),
    "minilink.optimization": ("MathematicalProgram", "Optimizer"),
    "minilink.planning": (
        "PlanningProblem",
        "TrajectoryPlan",
        "TrajectoryOptimizationPlanner",
        "StateSpaceGrid",
        "DynamicProgrammingPlanner",
        "LookupTableController",
        "PolicyEvaluator",
        "RRTPlanner",
        "RRTStarPlanner",
        "RRTOptions",
        "KinodynamicExtender",
        "SteeringExtender",
    ),
}

# Factories whose module name matches the factory (DESIGN §2 exception).
TEACHING_MODULE_FUNCTIONS = (
    ("minilink.control.lqr", "lqr"),
    ("minilink.control.lqr", "lqr_at_operating_point"),
    ("minilink.analysis.linearize", "linearize"),
    ("minilink.analysis.discretize", "discretize"),
)

# Module prefixes that belong to the teaching lane.
TEACHING_LANE_PREFIXES = (
    "minilink.core.",
    "minilink.blocks.",
    "minilink.dynamics.abstraction.",
    "minilink.dynamics.catalog.",
    "minilink.control.",
    "minilink.analysis.",
    "minilink.simulation.simulator",
    "minilink.simulation.static_simulator",
    "minilink.planning.",
    "minilink.optimization.",
)
RESEARCH_LANE_PREFIXES = (
    "minilink.control.mpc",
    "minilink.simulation.realtime",
    "minilink.simulation.computer",
    "minilink.simulation.hybrid_simulator",
    "minilink.core.hybrid",
    "minilink.experimental",
)


class TestTeachingSurface(unittest.TestCase):
    def test_every_name_resolves_with_a_docstring_in_the_teaching_lane(self):
        for band, names in TEACHING_SURFACE.items():
            module = importlib.import_module(band)
            for name in names:
                value = getattr(module, name)
                self.assertTrue(
                    (getattr(value, "__doc__", None) or "").strip(),
                    f"{band}.{name} has no docstring",
                )
                home = getattr(value, "__module__", "")
                self.assertTrue(
                    home.startswith(TEACHING_LANE_PREFIXES),
                    f"{band}.{name} lives in {home}, outside the teaching lane",
                )
                self.assertFalse(
                    home.startswith(RESEARCH_LANE_PREFIXES),
                    f"{band}.{name} lives in the research lane ({home})",
                )
        for module_path, name in TEACHING_MODULE_FUNCTIONS:
            value = getattr(importlib.import_module(module_path), name)
            self.assertTrue((value.__doc__ or "").strip(), f"{module_path}.{name}")

    def test_whole_root_prelude_is_checked_not_just_the_sample(self):
        """Every name the root exports, so DESIGN §2's "tested as a set" is literal.

        The curated table above pins the names a course depends on; this walks
        ``minilink.__all__`` in full, so a new export cannot enter the teaching
        surface undocumented or from the research lane.
        """
        root = importlib.import_module("minilink")
        self.assertGreater(len(root.__all__), 100)
        for name in root.__all__:
            value = getattr(root, name)
            self.assertTrue(
                (getattr(value, "__doc__", None) or "").strip(),
                f"minilink.{name} is exported to students with no docstring",
            )
            home = getattr(value, "__module__", "")
            self.assertTrue(
                home.startswith(TEACHING_LANE_PREFIXES),
                f"minilink.{name} lives in {home}, outside the teaching lane",
            )
            self.assertFalse(
                home.startswith(RESEARCH_LANE_PREFIXES),
                f"minilink.{name} lives in the research lane ({home})",
            )

    def test_catalog_plants_all_resolve(self):
        catalog = importlib.import_module("minilink.catalog")
        for name in catalog.__all__:
            self.assertIsNotNone(getattr(catalog, name), name)


_BASIC_TIER_PROBE = r"""
import builtins, sys, warnings
warnings.filterwarnings("ignore")
BLOCKED = {"graphviz", "jax", "jaxlib", "meshcat", "plotly", "pygame", "sympy",
           "gymnasium", "cyipopt", "torch", "stable_baselines3"}
_real_import = builtins.__import__
def _guard(name, *args, **kwargs):
    if name.split(".")[0] in BLOCKED:
        raise ImportError(f"blocked for the Basic-tier smoke: {name}")
    return _real_import(name, *args, **kwargs)
builtins.__import__ = _guard
for mod in list(sys.modules):
    if mod.split(".")[0] in BLOCKED:
        del sys.modules[mod]

import numpy as np
from minilink import ImpedanceController, Pendulum, lqr
from minilink.analysis.linearize import linearize
from minilink.core import QuadraticCost
from minilink.planning import DynamicProgrammingPlanner, PlanningProblem, StateSpaceGrid

plant = Pendulum()
plant.x0[0] = 1.0
loop = ImpedanceController() @ plant
traj = loop.compute_trajectory(tf=2.0, verbose=False)
loop.plot_trajectory(show=False)
loop.plot_phase_plane(show=False)
loop.animate(show=False)
loop.plot_diagram(show=False)          # degrades without graphviz
lti = linearize(plant, x_bar=[0.0, 0.0])
lqr(lti.A(), lti.B(), np.eye(2), np.eye(1))
goal = np.array([np.pi, 0.0])
problem = PlanningProblem(plant, x_goal=goal,
    cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), xbar=goal))
grid = StateSpaceGrid(problem, x_grid_shape=(21, 21), u_grid_shape=(3,), dt=0.05)
DynamicProgrammingPlanner(problem, grid=grid, max_iterations=20, verbose=False).solve()
print("BASIC-TIER-OK", traj.n_samples)
"""


class TestBasicTier(unittest.TestCase):
    def test_numpy_scipy_matplotlib_only(self):
        """Sim, plots, animation, linearize, LQR, and VI with optional packages blocked."""
        env = dict(os.environ)
        env["PYTHONPATH"] = os.getcwd()
        env["MPLBACKEND"] = "Agg"
        result = subprocess.run(
            [sys.executable, "-c", _BASIC_TIER_PROBE],
            env=env,
            capture_output=True,
            text=True,
            timeout=600,
        )
        self.assertEqual(result.returncode, 0, result.stderr[-3000:])
        self.assertIn("BASIC-TIER-OK", result.stdout)


class TestRootPrelude(unittest.TestCase):
    """The root package exports exactly the teaching surface (one import line)."""

    def test_every_surface_name_is_the_same_object_at_the_root(self):
        import minilink

        for band, names in TEACHING_SURFACE.items():
            module = importlib.import_module(band)
            for name in names:
                self.assertIn(
                    name, minilink.__all__, f"{name} missing from the root prelude"
                )
                self.assertIs(getattr(minilink, name), getattr(module, name), name)
        for module_path, name in TEACHING_MODULE_FUNCTIONS:
            self.assertIs(
                getattr(minilink, name),
                getattr(importlib.import_module(module_path), name),
            )

    def test_every_root_export_is_teaching_lane(self):
        import minilink

        for name in minilink.__all__:
            value = getattr(minilink, name)
            home = getattr(value, "__module__", "")
            self.assertTrue(home.startswith(TEACHING_LANE_PREFIXES), f"{name}: {home}")
            self.assertFalse(home.startswith(RESEARCH_LANE_PREFIXES), f"{name}: {home}")


if __name__ == "__main__":
    unittest.main()
