"""Teaching public-import facades (DESIGN §2)."""

import inspect
import unittest

from minilink.dynamics.catalog.pendulum.pendulum import Pendulum as PendulumDef


class TestPublicImports(unittest.TestCase):
    def test_root_prelude_pendulum_matches_defining_module(self):
        from minilink import Pendulum

        self.assertIs(Pendulum, PendulumDef)

    def test_catalog_band_matches_defining_module(self):
        from minilink.catalog import Pendulum

        self.assertIs(Pendulum, PendulumDef)

    def test_domain_package_matches_defining_module(self):
        from minilink.dynamics.catalog.pendulum import Pendulum

        self.assertIs(Pendulum, PendulumDef)

    def test_control_and_analysis_band_exports(self):
        from minilink.analysis import bode, modal_analysis
        from minilink.analysis.linearize import linearize
        from minilink.control import ImpedanceController
        from minilink.control.lqr import lqr

        self.assertTrue(callable(lqr))
        self.assertTrue(callable(linearize))
        self.assertTrue(callable(bode))
        self.assertTrue(callable(modal_analysis))
        self.assertTrue(callable(ImpedanceController))

    def test_root_prelude_is_the_teaching_surface(self):
        import minilink

        self.assertIn("Pendulum", minilink.__all__)
        self.assertIn("Boat2D", minilink.__all__)  # the catalog is teaching surface
        self.assertIn("ReinforcementLearningPlanner", minilink.__all__)
        self.assertIn("TabularLearningPlanner", minilink.__all__)
        self.assertIn("StochasticPlanningProblem", minilink.__all__)
        self.assertNotIn("ModelPredictiveController", minilink.__all__)  # research lane
        self.assertNotIn("HybridDiagram", minilink.__all__)

    def test_package_exposes_a_version(self):
        import minilink

        self.assertTrue(minilink.__version__)
        self.assertRegex(minilink.__version__, r"^0\.")

    def test_simulation_band_exports_simulators(self):
        from minilink.simulation import Simulator, StaticSimulator
        from minilink.simulation.simulator import Simulator as SimulatorDef
        from minilink.simulation.static_simulator import (
            StaticSimulator as StaticSimulatorDef,
        )

        self.assertIs(Simulator, SimulatorDef)
        self.assertIs(StaticSimulator, StaticSimulatorDef)

    def test_planning_and_core_band_exports(self):
        from minilink.core import (
            DiagramSystem,
            DynamicSystem,
            QuadraticCost,
            Trajectory,
        )
        from minilink.core.costs import QuadraticCost as QuadraticCostDef
        from minilink.core.system import DynamicSystem as DynamicSystemDef
        from minilink.planning import (
            DynamicProgrammingPlanner,
            PlanningProblem,
            StateSpaceGrid,
            TrajectoryOptimizationPlanner,
        )
        from minilink.planning.problems import PlanningProblem as PlanningProblemDef

        self.assertIs(DynamicSystem, DynamicSystemDef)
        self.assertIs(QuadraticCost, QuadraticCostDef)
        self.assertIs(PlanningProblem, PlanningProblemDef)
        for value in (
            DiagramSystem,
            Trajectory,
            TrajectoryOptimizationPlanner,
            DynamicProgrammingPlanner,
            StateSpaceGrid,
        ):
            self.assertTrue(callable(value))

    def test_blocks_band_exports(self):
        from minilink.blocks import Integrator, Step, Sum
        from minilink.blocks.basic import Integrator as IntegratorDef

        self.assertIs(Integrator, IntegratorDef)
        self.assertTrue(callable(Step))
        self.assertTrue(callable(Sum))

    def test_stable_band_exports_all_resolve(self):
        """Every `_EXPORTS` name on a stable band facade resolves lazily."""
        import importlib

        for band in (
            "minilink",
            "minilink.blocks",
            "minilink.simulation",
            "minilink.planning",
            "minilink.optimization",
            "minilink.core",
        ):
            module = importlib.import_module(band)
            for name in module.__all__:
                self.assertIsNotNone(getattr(module, name), f"{band}.{name}")

    def test_catalog_exports_match_domain_all(self):
        """Catalog alias stays in full sync with the domain `__all__` lists."""
        import importlib

        import minilink.catalog as catalog

        domain_names: set[str] = set()
        for module_path in sorted(
            {module_path for module_path, _attr in catalog._EXPORTS.values()}
        ):
            domain = importlib.import_module(module_path)
            domain_names.update(domain.__all__)

        catalog_names = set(catalog._EXPORTS)
        self.assertEqual(
            catalog_names,
            domain_names,
            "minilink.catalog._EXPORTS and domain __all__ lists have drifted",
        )
        for name in sorted(catalog_names):
            self.assertIsNotNone(getattr(catalog, name), f"catalog.{name}")

    def test_catalog_exports_are_systems(self):
        """The teaching catalog exposes System classes only (plants)."""
        import minilink.catalog as catalog
        from minilink.core.system import System

        for name in sorted(catalog._EXPORTS):
            value = getattr(catalog, name)
            self.assertTrue(
                isinstance(value, type) and issubclass(value, System),
                f"catalog.{name} is not a System subclass",
            )


# Every analysis verb reads tool(sys, x_bar=None, u_bar=None, t=0.0, params=None, *, ...),
# with method="auto" and eps=1e-6 where it differentiates. Today's exceptions, each with
# its reason; the list can only shrink.
BAND_PATTERN_DEVIATIONS = {
    "jacobian": "names the function and the variable first: jacobian(sys, of, wrt, ...)",
    "find_equilibrium": "requires the guess x_guess where the others take x_bar",
    "step_info": "reads a sampled response: step_info(time, y)",
    "controllability": "takes the matrices (A, B) or one LTISystem",
    "observability": "takes the matrices (A, C) or one LTISystem",
    "plot_region_of_attraction": "draws a certificate: plot_region_of_attraction(certificate)",
    "region_of_attraction": 'method names the Lyapunov construction ("quadratic")',
}


def band_pattern_issues(fn):
    """How ``fn`` departs from the band's calling pattern; empty when it follows it."""
    params = list(inspect.signature(fn).parameters.values())
    issues = []
    if not params or params[0].name != "sys":
        issues.append("first parameter is not sys")
    prefix = [(p.name, p.default) for p in params[1:5]]
    if prefix != [("x_bar", None), ("u_bar", None), ("t", 0.0), ("params", None)]:
        issues.append(f"positional prefix {prefix}")
    for p in params:
        if p.name == "method" and p.default != "auto":
            issues.append(f"method={p.default!r}")
        if p.name == "eps" and p.default != 1e-6:
            issues.append(f"eps={p.default!r}")
    return issues


class TestBandCallingPattern(unittest.TestCase):
    def test_every_analysis_verb_follows_the_pattern(self):
        import minilink.analysis as band

        for name in band.__all__:
            fn = getattr(band, name)
            if not inspect.isfunction(fn) or name in BAND_PATTERN_DEVIATIONS:
                continue
            with self.subTest(name):
                self.assertEqual(band_pattern_issues(fn), [])

    def test_every_allowed_deviation_still_deviates(self):
        import minilink.analysis as band

        for name in BAND_PATTERN_DEVIATIONS:
            with self.subTest(name):
                self.assertIn(name, band.__all__)
                self.assertNotEqual(band_pattern_issues(getattr(band, name)), [])


if __name__ == "__main__":
    unittest.main()
