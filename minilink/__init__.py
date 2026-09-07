"""minilink — Python/JAX block-diagram toolbox for dynamical systems.

The root package exports the **teaching surface** (ROADMAP §2): every name a
student meets, so a script or notebook needs one import line::

    from minilink import Pendulum, ImpedanceController, lqr
    from minilink import QuadraticCost, PlanningProblem, DynamicProgrammingPlanner

Band facades organise the same names by role (``minilink.catalog``,
``minilink.blocks``, ``minilink.control``, ``minilink.analysis``,
``minilink.simulation``, ``minilink.planning``, ``minilink.optimization``,
``minilink.core``); defining modules stay importable. The surface is tested as
a set in ``tests/unittest/test_teaching_surface.py``; research-lane names never
appear here.
"""

from __future__ import annotations

from minilink.catalog import __all__ as _CATALOG_NAMES
from minilink.core.facade import lazy_facade

# name -> (module path, attribute); catalog plants resolve through minilink.catalog
_EXPORTS: dict[str, tuple[str, str]] = {
    # core
    "System": ("minilink.core.system", "System"),
    "DynamicSystem": ("minilink.core.system", "DynamicSystem"),
    "StepSystem": ("minilink.core.system", "StepSystem"),
    "DiagramSystem": ("minilink.core.diagram", "DiagramSystem"),
    "Controller": ("minilink.core.feedback", "Controller"),
    "Trajectory": ("minilink.core.trajectory", "Trajectory"),
    "CostFunction": ("minilink.core.costs", "CostFunction"),
    "QuadraticCost": ("minilink.core.costs", "QuadraticCost"),
    "BoxSet": ("minilink.core.sets", "BoxSet"),
    "BoxInputSet": ("minilink.core.sets", "BoxInputSet"),
    # blocks
    "Integrator": ("minilink.blocks.basic", "Integrator"),
    "Step": ("minilink.blocks.sources", "Step"),
    "WhiteNoise": ("minilink.blocks.sources", "WhiteNoise"),
    "TrajectorySource": ("minilink.blocks.sources", "TrajectorySource"),
    "Sum": ("minilink.blocks.routing", "Sum"),
    "Gain": ("minilink.blocks.routing", "Gain"),
    "Mux": ("minilink.blocks.routing", "Mux"),
    "Demux": ("minilink.blocks.routing", "Demux"),
    "Saturation": ("minilink.blocks.nonlinear", "Saturation"),
    "DeadZone": ("minilink.blocks.nonlinear", "DeadZone"),
    "Relay": ("minilink.blocks.nonlinear", "Relay"),
    "LowPassFilter": ("minilink.blocks.filters", "LowPassFilter"),
    "TransferFunction": ("minilink.blocks.transfer_function", "TransferFunction"),
    # control
    "ProportionalController": ("minilink.control.output", "ProportionalController"),
    "StateFeedbackController": ("minilink.control.state", "StateFeedbackController"),
    "FilteredController": ("minilink.control.siso", "FilteredController"),
    "ImpedanceController": ("minilink.control.impedance", "ImpedanceController"),
    "JointImpedance": ("minilink.control.robotic", "JointImpedance"),
    "TaskImpedance": ("minilink.control.robotic", "TaskImpedance"),
    "ComputedTorqueController": (
        "minilink.control.modelbased",
        "ComputedTorqueController",
    ),
    "SlidingModeController": ("minilink.control.modelbased", "SlidingModeController"),
    "lqr": ("minilink.control.lqr", "lqr"),
    "lqr_at_operating_point": ("minilink.control.lqr", "lqr_at_operating_point"),
    # analysis
    "jacobian": ("minilink.analysis.derivatives", "jacobian"),
    "linearize": ("minilink.analysis.linearize", "linearize"),
    "transfer_function": ("minilink.analysis.frequency", "transfer_function"),
    "bode": ("minilink.analysis.frequency", "bode"),
    "plot_bode": ("minilink.analysis.frequency", "plot_bode"),
    "pzmap": ("minilink.analysis.frequency", "pzmap"),
    "plot_pzmap": ("minilink.analysis.frequency", "plot_pzmap"),
    "modal_analysis": ("minilink.analysis.modal", "modal_analysis"),
    "controllability": ("minilink.analysis.structural", "controllability"),
    "observability": ("minilink.analysis.structural", "observability"),
    "find_equilibrium": ("minilink.analysis.equilibria", "find_equilibrium"),
    # simulation
    "Simulator": ("minilink.simulation.simulator", "Simulator"),
    "StaticSimulator": ("minilink.simulation.static_simulator", "StaticSimulator"),
    # planning
    "PlanningProblem": ("minilink.planning.problems", "PlanningProblem"),
    "TrajectoryPlan": ("minilink.planning.results", "TrajectoryPlan"),
    "TrajectoryOptimizationPlanner": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationPlanner",
    ),
    "StateSpaceGrid": (
        "minilink.planning.policy_synthesis.discretizer",
        "StateSpaceGrid",
    ),
    "DynamicProgrammingPlanner": (
        "minilink.planning.policy_synthesis.dp",
        "DynamicProgrammingPlanner",
    ),
    "LookupTableController": (
        "minilink.planning.policy_synthesis.lookup_policy",
        "LookupTableController",
    ),
    "PolicyEvaluator": (
        "minilink.planning.policy_synthesis.policy_eval",
        "PolicyEvaluator",
    ),
    "RRTPlanner": ("minilink.planning.search.rrt", "RRTPlanner"),
    # optimization
    "MathematicalProgram": (
        "minilink.optimization.mathematical_program",
        "MathematicalProgram",
    ),
    # band names the teaching demos use
    "TimeCost": ("minilink.core.costs", "TimeCost"),
    "StepDiagramSystem": ("minilink.core.diagram", "StepDiagramSystem"),
    "BallSet": ("minilink.core.sets", "BallSet"),
    "closed_loop_qdq": ("minilink.core.composition", "closed_loop_qdq"),
    "ZOHHold": ("minilink.blocks.step", "ZOHHold"),
    "Source": ("minilink.blocks.sources", "Source"),
    "ImpedanceIntegralController": (
        "minilink.control.impedance",
        "ImpedanceIntegralController",
    ),
    "discretize": ("minilink.analysis.discretize", "discretize"),
    "RRTStarPlanner": ("minilink.planning.search.rrt_star", "RRTStarPlanner"),
    "Optimizer": ("minilink.optimization.optimizer", "Optimizer"),
}

_EXPORTS.update({name: ("minilink.catalog", name) for name in _CATALOG_NAMES})

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)
