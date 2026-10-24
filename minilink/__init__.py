"""Minilink: a Python/JAX block-diagram toolbox for dynamical systems."""

from __future__ import annotations

from minilink.catalog import __all__ as _CATALOG_NAMES
from minilink.core.facade import lazy_facade

# The root package exports the teaching surface (ROADMAP §2): every name a student
# meets, so a script or notebook needs one import line, e.g.
#
#     from minilink import Pendulum, Impedance, lqr
#     from minilink import QuadraticCost, PlanningProblem, DynamicProgrammingPlanner
#     from minilink import ReinforcementLearningPlanner, StochasticPlanningProblem
#
# The band facades organise the same names by role (``minilink.catalog``,
# ``minilink.blocks``, ``minilink.control``, ``minilink.analysis``,
# ``minilink.simulation``, ``minilink.planning``, ``minilink.optimization``,
# ``minilink.core``); the defining modules stay importable. The surface is tested as a
# set in ``tests/unittest/test_teaching_surface.py``; research-lane names never appear
# here. Each exported name maps to its (module path, attribute); the catalog plants
# resolve through ``minilink.catalog``.
_EXPORTS: dict[str, tuple[str, str]] = {
    # Core
    "System": ("minilink.core.system", "System"),
    "DynamicSystem": ("minilink.core.system", "DynamicSystem"),
    "StepSystem": ("minilink.core.discrete.step_system", "StepSystem"),
    "DiagramSystem": ("minilink.core.diagram", "DiagramSystem"),
    "Trajectory": ("minilink.core.trajectory", "Trajectory"),
    "CostFunction": ("minilink.core.costs", "CostFunction"),
    "QuadraticCost": ("minilink.core.costs", "QuadraticCost"),
    "BoxSet": ("minilink.core.sets", "BoxSet"),
    "BoxInputSet": ("minilink.core.sets", "BoxInputSet"),
    # Blocks
    "Integrator": ("minilink.blocks.basic", "Integrator"),
    "Step": ("minilink.blocks.sources", "Step"),
    "Sine": ("minilink.blocks.sources", "Sine"),
    "WhiteNoise": ("minilink.blocks.sources", "WhiteNoise"),
    "TrajectorySource": ("minilink.blocks.sources", "TrajectorySource"),
    "Error": ("minilink.blocks.routing", "Error"),
    "Sum": ("minilink.blocks.routing", "Sum"),
    "Gain": ("minilink.blocks.routing", "Gain"),
    "Mux": ("minilink.blocks.routing", "Mux"),
    "Demux": ("minilink.blocks.routing", "Demux"),
    "Saturation": ("minilink.blocks.nonlinear", "Saturation"),
    "LowPassFilter": ("minilink.blocks.filters", "LowPassFilter"),
    "MLP": ("minilink.blocks.neural", "MLP"),
    "TransferFunction": ("minilink.blocks.transfer_function", "TransferFunction"),
    "Lead": ("minilink.blocks.transfer_function", "Lead"),
    # Control
    "P": ("minilink.control.siso", "P"),
    "StateFeedback": ("minilink.control.state", "StateFeedback"),
    "OutputReference": (
        "minilink.control.state",
        "OutputReference",
    ),
    "PID": ("minilink.control.siso", "PID"),
    "PurePursuit": ("minilink.control.geometric", "PurePursuit"),
    "PI": ("minilink.control.siso", "PI"),
    "PD": ("minilink.control.siso", "PD"),
    "Impedance": ("minilink.control.impedance", "Impedance"),
    "JointImpedance": ("minilink.control.robotic", "JointImpedance"),
    "TaskKinematic": ("minilink.control.robotic", "TaskKinematic"),
    "TaskKinematicNullspace": ("minilink.control.robotic", "TaskKinematicNullspace"),
    "TaskImpedance": ("minilink.control.robotic", "TaskImpedance"),
    "ComputedTorque": (
        "minilink.control.modelbased",
        "ComputedTorque",
    ),
    "SlidingMode": ("minilink.control.modelbased", "SlidingMode"),
    "lqr": ("minilink.control.design", "lqr"),
    "lqr_gain": ("minilink.control.design", "lqr_gain"),
    "lqr_finite_horizon": ("minilink.control.design", "lqr_finite_horizon"),
    "trajectory_lqr": ("minilink.control.design", "trajectory_lqr"),
    "place": ("minilink.control.design", "place"),
    "place_gain": ("minilink.control.design", "place_gain"),
    "lqg": ("minilink.control.design", "lqg"),
    # Estimation
    "StateObserver": ("minilink.estimation.observer", "StateObserver"),
    "kalman": ("minilink.estimation.design", "kalman"),
    "kalman_gain": ("minilink.estimation.design", "kalman_gain"),
    "luenberger": ("minilink.estimation.design", "luenberger"),
    "luenberger_gain": ("minilink.estimation.design", "luenberger_gain"),
    "TimeVaryingFeedback": (
        "minilink.control.state",
        "TimeVaryingFeedback",
    ),
    "TrajectoryFeedback": (
        "minilink.control.state",
        "TrajectoryFeedback",
    ),
    "NeuralPolicyController": ("minilink.control.neural", "NeuralPolicyController"),
    # Analysis
    "jacobian": ("minilink.analysis.derivatives", "jacobian"),
    "linearize": ("minilink.analysis.linearization", "linearize"),
    "transfer_function": ("minilink.analysis.frequency", "transfer_function"),
    "bode": ("minilink.analysis.frequency", "bode"),
    "plot_bode": ("minilink.analysis.frequency", "plot_bode"),
    "pzmap": ("minilink.analysis.frequency", "pzmap"),
    "plot_pzmap": ("minilink.analysis.frequency", "plot_pzmap"),
    "margins": ("minilink.analysis.frequency", "margins"),
    "nyquist": ("minilink.analysis.frequency", "nyquist"),
    "plot_nyquist": ("minilink.analysis.frequency", "plot_nyquist"),
    "root_locus": ("minilink.analysis.frequency", "root_locus"),
    "plot_root_locus": ("minilink.analysis.frequency", "plot_root_locus"),
    "step_response": ("minilink.analysis.time_response", "step_response"),
    "plot_step_response": ("minilink.analysis.time_response", "plot_step_response"),
    "modal_analysis": ("minilink.analysis.modal", "modal_analysis"),
    "controllability": ("minilink.analysis.structural", "controllability"),
    "observability": ("minilink.analysis.structural", "observability"),
    "find_equilibrium": ("minilink.analysis.equilibria", "find_equilibrium"),
    # Simulation
    "Simulator": ("minilink.simulation.simulator", "Simulator"),
    "StaticSimulator": ("minilink.simulation.static_simulator", "StaticSimulator"),
    # Planning
    "PlanningProblem": ("minilink.planning.problems", "PlanningProblem"),
    "PlanningSolution": ("minilink.planning.results", "PlanningSolution"),
    "LQRPlanner": ("minilink.planning.dynamic_programming.lqr", "LQRPlanner"),
    "compare": ("minilink.planning.comparison", "compare"),
    "evaluate": ("minilink.planning.engine", "evaluate"),
    "TrajectoryOptimizationPlanner": (
        "minilink.planning.trajectory_optimization.planner",
        "TrajectoryOptimizationPlanner",
    ),
    "StateSpaceGrid": (
        "minilink.planning.dynamic_programming.discretizer",
        "StateSpaceGrid",
    ),
    "DynamicProgrammingPlanner": (
        "minilink.planning.dynamic_programming.dp",
        "DynamicProgrammingPlanner",
    ),
    "LookupPolicy": (
        "minilink.planning.dynamic_programming.lookup_policy",
        "LookupPolicy",
    ),
    "QuadraticFeatures": (
        "minilink.planning.dynamic_programming.approximation",
        "QuadraticFeatures",
    ),
    "RadialBasisFeatures": (
        "minilink.planning.dynamic_programming.approximation",
        "RadialBasisFeatures",
    ),
    "LinearApproximator": (
        "minilink.planning.dynamic_programming.approximation",
        "LinearApproximator",
    ),
    "PolicyEvaluator": (
        "minilink.planning.dynamic_programming.policy_eval",
        "PolicyEvaluator",
    ),
    "StochasticPlanningProblem": (
        "minilink.planning.problems",
        "StochasticPlanningProblem",
    ),
    "Gaussian": ("minilink.core.distributions", "Gaussian"),
    "Uniform": ("minilink.core.distributions", "Uniform"),
    "ReinforcementLearningPlanner": (
        "minilink.learning.reinforcement_learning.planner",
        "ReinforcementLearningPlanner",
    ),
    "TabularLearningPlanner": (
        "minilink.learning.reinforcement_learning.tabular",
        "TabularLearningPlanner",
    ),
    "Evaluation": ("minilink.planning.engine", "Evaluation"),
    "RRTPlanner": ("minilink.planning.search.rrt", "RRTPlanner"),
    "RRTOptions": ("minilink.planning.search.rrt", "RRTOptions"),
    "KinodynamicExtender": (
        "minilink.planning.search.extenders",
        "KinodynamicExtender",
    ),
    "SteeringExtender": ("minilink.planning.search.extenders", "SteeringExtender"),
    # Optimization
    "MathematicalProgram": (
        "minilink.optimization.mathematical_program",
        "MathematicalProgram",
    ),
    # Band names the teaching demos use
    "TimeCost": ("minilink.core.costs", "TimeCost"),
    "StepDiagramSystem": ("minilink.core.discrete.step_diagram", "StepDiagramSystem"),
    "BallSet": ("minilink.core.sets", "BallSet"),
    "SingletonSet": ("minilink.core.sets", "SingletonSet"),
    "closed_loop": ("minilink.core.closed_loop", "closed_loop"),
    "feedback": ("minilink.core.closed_loop", "feedback"),
    "Source": ("minilink.blocks.sources", "Source"),
    "IntegralImpedance": (
        "minilink.control.impedance",
        "IntegralImpedance",
    ),
    "discretize": ("minilink.analysis.discretization", "discretize"),
    "RRTStarPlanner": ("minilink.planning.search.rrt_star", "RRTStarPlanner"),
    "Optimizer": ("minilink.optimization.optimizer", "Optimizer"),
}

_EXPORTS.update({name: ("minilink.catalog", name) for name in _CATALOG_NAMES})

__all__, __getattr__, __dir__ = lazy_facade(globals(), _EXPORTS)


# Package version: the build's ``_version`` module, else the installed metadata, else
# 0.0.0; the metadata module is released so the package listing stays the surface
try:
    # The build writes _version.py; a source checkout has none
    from minilink._version import __version__
except ImportError:
    from importlib import metadata

    try:
        __version__ = metadata.version("minilink")
    except Exception:
        __version__ = "0.0.0"
    del metadata
