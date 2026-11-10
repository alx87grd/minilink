"""The vocabulary of RULES 2.7: no retired word on the public surface of the tools and the core."""

import dataclasses
import importlib
import inspect
import warnings

import pytest

# The facades the surface is read from; the plants' own physics symbols are their textbook's
FACADES = (
    "minilink",
    "minilink.core",
    "minilink.control",
    "minilink.estimation",
    "minilink.blocks",
    "minilink.analysis",
    "minilink.simulation",
    "minilink.optimization",
    "minilink.planning",
    "minilink.learning",
    "minilink.interfaces",
)
PLANT_PACKAGE = "minilink.dynamics"

# A retired word: the word of RULES 2.7 that replaced it, and the owners it is retired on
# (None: everywhere). A word keeps its other textbook meanings elsewhere: alpha is the CVaR
# level of Evaluation.cvar, N the reference gain of OutputReference. Each sweep adds its own.
RETIRED = {
    "gamma": ("radius_constant", ("RRTStarPlanner",)),
    "settings": ("params", None),
    "final_time": (
        "tf, the problem's t0 + horizon",
        ("DynamicProgrammingPlanner",),
    ),
    "x_start": ("x0", None),
    "t_start": ("t0", None),
    "iterations": (
        "n_iterations",
        (
            "Optimizer",
            "TrajectoryOptimizationPlanner",
            "MPPIPlanner",
            "MPPIRecord",
            "TrajectoryOptimizationRecord",
            "ValueIterationRecord",
            "TreeSearchRecord",
            "ValueTables",
        ),
    ),
    "optimizer_options": ("options", None),
    "tf": (
        "horizon, or episode_length",
        ("PlanningProblem", "Sys2Gym", "RolloutEnvironment"),
    ),
    "require_finite_tf": ("require_finite_horizon(problem)", None),
    "xbar": ("x_bar", None),
    "ubar": ("u_bar", None),
    "ybar": ("y_bar", None),
    "eps": ("radius", ("TimeCost",)),
    "plot_cost2go": ("plot_cost_to_go", None),
    "plot_policy": ("plot_control_law", None),
    "animate_cost2go": ("animate_cost_to_go", None),
    "solve_trajectory": ("solve", None),
    "record_solve_time": ("solve_time_s, always recorded", None),
    "compute_forced": ("compute_trajectory(u=)", None),
    "trajectory_of": ("traj.of(block)", None),
    "solve_forced": ("solve(u=)", None),
    "solve_trajectory_from": ("solve(params=) or compile().solve(params, z0)", None),
    "compile_parametric_program": ("compile()", None),
    "has_parametric_program": ("compile()", None),
    "step": ("compile().solve(params, z0)", ("TrajectoryOptimizationPlanner",)),
    "start_point": ("x0", ("PlanningProblem",)),
    "t2x": ("at", ("Trajectory",)),
    "t2u": ("at(t, 'u')", ("Trajectory",)),
    "sample": ("at", ("Trajectory",)),
    "n": (
        "state_dim on a system; n_rollouts (verify) or n_samples (the plots) on a certificate",
        (
            "System",
            "DynamicSystem",
            "StepSystem",
            "LyapunovCertificate",
            "plot_region_of_attraction",
        ),
    ),
    "episodes": ("n_episodes", None),
    "timesteps": ("n_timesteps", None),
    "with_signal": ("with_signals", ("Trajectory",)),
    "time_duration": ("tf - t0", ("Trajectory",)),
    "copy": ("a frozen record needs no copy", ("Trajectory",)),
    "gradient": ("grad_J", ("MathematicalProgramEvaluator",)),
    "hessian": ("hess_J", ("MathematicalProgramEvaluator",)),
    "jacobian_h": ("jac_h", ("MathematicalProgramEvaluator",)),
    "jacobian_g": ("jac_g", ("MathematicalProgramEvaluator",)),
    "mean_at": ("sample(None, params=)", None),
    "mean_draw": ("sample(None, n, params=)", None),
    "full_cov": ("cov", None),
    "from_system_state": ("sys.state.box", None),
    "from_system_inputs": ("sys.input_box", None),
    "from_bounds": ("BoxSet(lower, upper)", None),
    "solve_policy": ("solve", None),
    "N": ("n_steps", ("StepPlanningProblem",)),
    "n_samples": ("n_rollouts", ("MPPIPlanner",)),
    "nearest_backend": ("nearest_method", None),
    "solver": ("record", ("PlanningSolution",)),
    "algorithm": (
        "method",
        (
            "ReinforcementLearningPlanner",
            "TabularLearningPlanner",
            "ReinforcementLearningRecord",
            "TabularLearningRecord",
        ),
    ),
    "algorithm_kwargs": ("method_numbers", None),
    "method_kwargs": ("method_numbers", None),
    "method": ("family", ("PlanningSolution",)),
    "max_eq": ("max_equality_violation", None),
    "min_ineq": ("min_inequality_margin", None),
    "tree_nodes": ("n_nodes", None),
    "show_3d": ("is_3d", ("StateSpaceGrid",)),
    "axis_x": ("x_axis", None),
    "axis_y": ("y_axis", None),
    "show_live_search": ("callback=LiveSearchPlot()", ("RRTPlanner", "RRTStarPlanner")),
    "x_grid_shape": ("x_grid", ("StateSpaceGrid",)),
    "u_grid_shape": ("u_grid", ("StateSpaceGrid",)),
    "n_substeps": ("n_steps, dt = horizon / n_steps", ("KinodynamicExtender",)),
    "record": ("trajectories", ("ProblemEvaluator", "evaluate")),
    "nodes": ("n_nodes", ("TreeSearchRecord", "TreeSearchIteration")),
    "alpha": (
        "gamma",
        (
            "DynamicProgrammingPlanner",
            "DynamicProgrammingOptions",
            "TabularLearningPlanner",
        ),
    ),
    "get_controller": ("controller", None),
    "value_at": (
        "cost_to_go",
        ("Planner", "DynamicProgrammingPlanner", "TabularLearningPlanner"),
    ),
    "plot_solution": ("plot_trajectory", None),
    "animate_solution": ("animate", None),
    "live_plot": ("callback=LiveSearchPlot()", ("RRTPlanner", "RRTStarPlanner")),
    "live_plot_every": ("LiveSearchPlot(every=)", None),
    "live_plot_pause": ("LiveSearchPlot(pause=)", None),
    "live_plot_after_goal_only": ("LiveSearchPlot(after_goal_only=)", None),
    "live_plot_ax": ("LiveSearchPlot(ax=)", None),
    "animate_policy": ("animate_control_law", None),
    "num_timesteps": ("n_timesteps", ("ReinforcementLearningPlanner",)),
    "controller": ("actor", ("ExplorationPolicy",)),
    "lqr_matrices": ("design_matrices", ("LQRPlanner",)),
    "compile": ("transition, terminal or rollout", ("StepPlanningProblem",)),
    "run": ("rollout, a Trajectory", ("StepPlanningProblem",)),
    "n_pts": ("n_samples", None),
    "n_steps": (
        "n_samples (n_steps counts transitions only)",
        (
            "Simulator",
            "StaticSimulator",
            "lqr_gain_schedule",
            "lqr_finite_horizon",
            "animate_modal",
            "LQRPlanner",
        ),
    ),
    "backend": (
        "renderer (backend is the array module)",
        (
            "plot_bode",
            "plot_pzmap",
            "plot_root_locus",
            "plot_nyquist",
            "plot_step_response",
        ),
    ),
    "plot_backend": ("plot_renderer", None),
    "p": ("dim", ("Source", "WhiteNoise")),
    "dof": (
        "dim (dof stays a manipulator's joints)",
        ("P", "PI", "PD", "PID", "Impedance", "IntegralImpedance"),
    ),
    "filename": ("file_name", None),
    "limits": ("bounds", ("LyapunovCertificate", "plot_region_of_attraction")),
    "show_signal": ("plot_signal", None),
    "time": ("t", ("step_info",)),
    "system": ("sys", ("discretize",)),
    "time_factor_video": ("speed", None),
    "samples": ("n_samples", ("region_of_attraction",)),
    "get_subsys_input_port": ("get_subsystem_input_port", None),
    "compute_subsys_output_port": ("compute_subsystem_output_port", None),
    "get_port_values_from_u": ("ports", None),
    "get_signal": ("signal", None),
    "nodes_n": ("n_nodes", None),
    "actions_n": ("n_actions", None),
    "effective_samples": ("effective_rollouts", None),
    "save_path": ("file_name", None),
    "plot_policy_field": ("plot_control_law_field", None),
    "plot_value": ("plot_cost_to_go", ("StateSpaceGrid",)),
    "eta": ("learning_rate", ("TabularLearningPlanner",)),
    "train_time": ("train_time_s", None),
    "solver_success": ("optimizer_success", None),
    "input_port_id": ("port", None),
    "interpolation": ("hold", ("TrajectorySource",)),
    "lower_bound": ("lower", None),
    "upper_bound": ("upper", None),
    "options": (
        "n_steps",
        (
            "DirectCollocationTranscription",
            "ShootingTranscription",
            "MultipleShootingTranscription",
        ),
    ),
}


# A retired name the facade scan does not reach (a class of a deep module, a member of one), at
# its full path: what replaced it. Each sweep adds its own
RETIRED_NAMES = {
    "minilink.planning.problems.PlanningProblem.has_goal": "problem.Xf is not None",
    "minilink.planning.problems.PlanningProblem.has_cost": "problem.cost is not None",
    "minilink.simulation.simulator.COMPILE_BACKEND_AUTO": (
        'backend="auto" (minilink.core.backends.BACKEND_AUTO)'
    ),
    "minilink.core.signals.InputPort.get_default_value": "port.nominal_value",
    "minilink.core.system.System.get_u_from_input_ports": (
        "sys.u_nominal (each input port's nominal_value, concatenated)"
    ),
    "minilink.core.signals.VectorSignal.set_nominal_value": (
        "signal.nominal_value = value"
    ),
    "minilink.planning.comparison.GivenPolicy": "GivenController",
    "minilink.core.discrete.sampled_system.SampledSystem.counts": "is_tick_law",
    "minilink.core.discrete.sampled_system.SampledSystem.resolves": "is_integrated",
    "minilink.core.sets.is_bounded": "has_finite_face",
    "minilink.planning.trajectory_optimization.path_integral.problem_leaves": (
        "episode_leaves (problem_leaves reads a problem: planning.evaluation's)"
    ),
    "minilink.planning.step_problems.coerce_horizon": (
        "coerce_n_steps (coerce_horizon is the continuous problem's)"
    ),
    "minilink.planning.trajectory_optimization.transcription.FixedGridOptions": (
        "FixedGridTranscription(n_steps=)"
    ),
    "minilink.planning.trajectory_optimization.direct_collocation.DirectCollocationOptions": (
        "DirectCollocationTranscription(n_steps=)"
    ),
    "minilink.planning.trajectory_optimization.shooting.ShootingOptions": (
        "ShootingTranscription(n_steps=)"
    ),
    "minilink.planning.trajectory_optimization.multiple_shooting.MultipleShootingOptions": (
        "MultipleShootingTranscription(n_steps=)"
    ),
    "minilink.planning.dynamic_programming.plotting.plot_policy_field": (
        "plot_control_law_field"
    ),
    "minilink.planning.dynamic_programming.discretizer.StateSpaceGrid.plot_value": (
        "plot_cost_to_go"
    ),
    "minilink.planning.trajectory_optimization.planner.TrajectoryOptimizationRecord.solver_success": (
        "optimizer_success"
    ),
    "minilink.planning.trajectory_optimization.mpc.predictive.PredictiveController": (
        "ModelPredictiveController, the class (ruling 15)"
    ),
    # One name per thing in the planners' records (the maintainer's ruling 17 of 2026-11-10)
    "minilink.planning.dynamic_programming.dp.DynamicProgrammingResult": (
        "ValueTables, read as solution.record.tables"
    ),
    "minilink.planning.dynamic_programming.dp.DynamicProgrammingPlanner.result": (
        "planner.tables, the latest solution's"
    ),
    "minilink.planning.search.rrt.TreeSearchRecord.cost": "path_cost",
    "minilink.learning.reinforcement_learning.tabular.TabularLearningRecord.cost": (
        "episode_cost"
    ),
    "minilink.learning.reinforcement_learning.planner.ReinforcementLearningRecord.cost": (
        "episode_cost"
    ),
    "minilink.learning.reinforcement_learning.tabular.TabularLearningPlanner.result": (
        "planner.tables, the latest solution's; planner.greedy_tables, the table as it stands"
    ),
}


def owner_of(path):
    """The module or class a dotted path's last name would sit on: the longest importable module, then attributes."""
    parts = path.split(".")[:-1]
    for split in range(len(parts), 0, -1):
        try:
            owner = importlib.import_module(".".join(parts[:split]))
        except ImportError:
            continue
        for name in parts[split:]:
            owner = getattr(owner, name)
        return owner
    raise ImportError(path)


# A port and the box it builds say the same word (the ruling 5 of 2026-11-10): a bound on an
# object is lower / upper; a limit a law applies to a named signal keeps that signal's name
BOUND_WORDS = {"lower_bound": "lower", "upper_bound": "upper"}

# A retired keyword of a function the facade scan does not reach, at the function's full path:
# the keyword and what replaced it
RETIRED_KEYWORDS = {
    "minilink.planning.trajectory_optimization.mpc.viz.plot_sampled_futures": {
        "axes": "x_axis, y_axis"
    },
    "minilink.planning.dynamic_programming.plotting.plot_control_law": {
        "axis": "u_axis"
    },
    "minilink.planning.dynamic_programming.plotting.animate_cost_to_go": {
        "save_path": "file_name"
    },
    "minilink.planning.dynamic_programming.plotting.animate_control_law": {
        "save_path": "file_name"
    },
    "minilink.planning.search.plotting.animate_search": {"save": "file_name"},
    "minilink.planning.search.plotting.animate_convergence": {"save": "file_name"},
    "minilink.planning.evaluation.score_run": {"tf": "horizon"},
    "minilink.planning.planner.planner_evidence": {"tf": "episode_length"},
    "minilink.planning.evaluator.steps_of": {"tf": "horizon"},
    "minilink.core.facades.plotting.SystemPlotting.plot_diagram": {
        "show_inline": 'show="inline"',
        "show_pdf": 'show="pdf"',
    },
    "minilink.graphical.diagrams.dot.plot_diagram": {
        "show_inline": 'show="inline"',
        "show_pdf": 'show="pdf"',
    },
    "minilink.graphical.diagrams.dot.render_diagram_graph": {
        "show_inline": 'show="inline"',
        "show_pdf": 'show="pdf"',
    },
    "minilink.core.closed_loop.feedback": {"validate": "the loop check always runs"},
    "minilink.core.closed_loop.closed_loop": {"validate": "the loop check always runs"},
    "minilink.core.operators.autowire": {"validate": "the loop check always runs"},
    "minilink.core.wiring.WiredDiagramMixin.autowire": {
        "validate": "the loop check always runs"
    },
    "minilink.core.facades.plotting.StepPlotting.plot_rollout": {"rollout": "traj"},
    "minilink.core.facades.simulation.SystemSimulation.compute_trajectory": {
        "input_port_id": "port"
    },
    "minilink.core.facades.simulation.DynamicSimulation.compute_trajectory": {
        "input_port_id": "port"
    },
    "minilink.simulation.simulator.Simulator.solve": {"input_port_id": "port"},
    "minilink.simulation.static_simulator.StaticSimulator.solve": {
        "input_port_id": "port"
    },
    "minilink.blocks.sources.TrajectorySource.__init__": {"interpolation": "hold"},
    "minilink.dynamics.abstraction.state_space.LTISystem.__init__": {
        "x_bar": "linearize(sys, x_bar), a LinearizedSystem",
        "u_bar": "linearize(sys, x_bar, u_bar), a LinearizedSystem",
        "y_bar": "a LinearizedSystem's params['y_bar']",
    },
    "minilink.dynamics.abstraction.state_space.DiscreteLTISystem.__init__": {
        "x_bar": "linearize(step_sys, x_bar), a LinearizedStepSystem",
        "u_bar": "linearize(step_sys, x_bar, u_bar), a LinearizedStepSystem",
        "y_bar": "a LinearizedStepSystem's params['y_bar']",
    },
    "minilink.blocks.sources.TrajectorySource.from_trajectory": {
        "interpolation": "hold"
    },
    "minilink.core.system.System.add_input_port": BOUND_WORDS,
    "minilink.core.system.System.add_output_port": BOUND_WORDS,
    "minilink.core.signals.VectorSignal.__init__": BOUND_WORDS,
    "minilink.core.signals.OutputPort.__init__": BOUND_WORDS,
    "minilink.core.wiring.WiredDiagramMixin.connect_new_output_port": BOUND_WORDS,
    "minilink.core.wiring.DiagramOutputPort.__init__": BOUND_WORDS,
}


@pytest.mark.parametrize("path", sorted(RETIRED_KEYWORDS))
def test_no_retired_keyword_at_its_path(path):
    function = getattr(owner_of(path), path.rsplit(".", 1)[-1])
    taken = set(inspect.signature(function).parameters)
    hits = {
        word: replaced
        for word, replaced in RETIRED_KEYWORDS[path].items()
        if word in taken
    }
    assert not hits, f"{path}: retired keywords {hits}"


# The model's order, (x, u, t; p) or the subset read (CONSTITUTION, one signature): a method's
# leading parameters, at its full path
ONE_SIGNATURE = {
    "minilink.planning.environment.RolloutEnvironment.step": ("x", "u", "t", "key"),
    "minilink.planning.dynamic_programming.lookup_policy.LookupPolicy.action": (
        "x",
        "t",
        "params",
    ),
    "minilink.control.neural.NeuralPolicy.action": ("x", "t", "params"),
}


@pytest.mark.parametrize("path", sorted(ONE_SIGNATURE))
def test_a_method_takes_the_model_order(path):
    method = getattr(owner_of(path), path.rsplit(".", 1)[-1])
    leading = tuple(inspect.signature(method).parameters)[
        1 : 1 + len(ONE_SIGNATURE[path])
    ]
    assert leading == ONE_SIGNATURE[path], f"{path} takes {leading}"


# Keyword-only where every sibling already is (api-stabilization AS-29): the leading
# parameters a caller may pass by position, at the function's full path; every other one is
# keyword-only
LEADING_POSITIONALS = {
    "minilink.planning.problems.PlanningProblem.__init__": ("sys",),
    "minilink.core.facades.simulation.SystemSimulation.compile": (),
    "minilink.core.diagram.DiagramSystem.compile": (),
    "minilink.core.discrete.step_diagram.StepDiagramSystem.compile": (),
    "minilink.core.discrete.sampled_diagram.SampledDiagramSystem.compile": (),
    "minilink.core.compile.compiler.compile": ("system",),
    "minilink.core.facades.animation.SystemAnimation.animate": ("traj",),
    "minilink.core.facades.animation.SystemAnimation.render": ("x", "u", "t"),
    "minilink.blocks.neural.MLP.__init__": ("input_dim", "output_dim"),
    "minilink.interfaces.gymnasium.Sys2Gym.__init__": ("sys", "cost"),
    "minilink.core.discrete.sampled_system.NativeSampledSystem.__init__": (
        "state_dim",
    ),
    "minilink.planning.evaluator.evaluate": ("problem", "controller", "dt"),
    "minilink.planning.problems.PlanningProblem.compile": ("controller", "dt"),
    # The impedance laws take the PID family's signature: the gains first, dim a keyword
    # (the maintainer's ruling 9 of 2026-11-10)
    "minilink.control.impedance.Impedance.__init__": ("Kp", "Kd"),
    "minilink.control.impedance.IntegralImpedance.__init__": ("Kp", "Ki", "Kd"),
    "minilink.control.siso.PID.__init__": ("Kp", "Ki", "Kd", "tau"),
    # The step reads its final level first: Step() is the unit step at t = 1 s
    # (the maintainer's ruling 13 of 2026-11-10)
    "minilink.blocks.sources.Step.__init__": (
        "final_value",
        "initial_value",
        "step_time",
    ),
}


@pytest.mark.parametrize("path", sorted(LEADING_POSITIONALS))
def test_only_the_leading_parameters_are_positional(path):
    function = getattr(owner_of(path), path.rsplit(".", 1)[-1])
    positional = tuple(
        name
        for name, parameter in inspect.signature(function).parameters.items()
        if name != "self"
        and parameter.kind
        in (parameter.POSITIONAL_ONLY, parameter.POSITIONAL_OR_KEYWORD)
    )
    assert positional == LEADING_POSITIONALS[path], f"{path} takes {positional}"


@pytest.mark.parametrize("path", sorted(RETIRED_NAMES))
def test_no_retired_name_at_its_path(path):
    name = path.rsplit(".", 1)[-1]
    owner = owner_of(path)
    fields = (
        {f.name for f in dataclasses.fields(owner)}
        if dataclasses.is_dataclass(owner)
        else set()
    )
    assert not hasattr(owner, name) and name not in fields, (
        f"{path} is retired, write {RETIRED_NAMES[path]!r}"
    )


def test_a_search_keeps_its_counts_on_its_record_only():
    """RRT's goal flag and extension count are the record's (RULES 2.4), not the planner's."""
    from minilink.planning.search.extenders import KinodynamicExtender
    from minilink.planning.search.rrt import RRTPlanner
    from minilink.planning.search.rrt_star import RRTStarPlanner
    from tests.unittest.planning_helpers import make_holonomic_obstacle_problem

    problem, _ = make_holonomic_obstacle_problem()
    extender = KinodynamicExtender(controls=[[1.0, 0.0]], horizon=0.5, n_steps=2)
    for planner_class in (RRTPlanner, RRTStarPlanner):
        planner = planner_class(problem, extender=extender, max_nodes=5)
        record = planner.solve().record
        assert record.n_iterations >= 0 and not record.reached_goal
        for retired in ("reached_goal", "iterations"):
            assert not hasattr(planner, retired), f"{planner_class.__name__}.{retired}"


def test_a_compiled_planner_counts_its_iterations_as_n_iterations():
    from minilink.planning.trajectory_optimization.mpc.predictive import (
        STANDARD_STATUS,
    )

    assert "n_iterations" in STANDARD_STATUS and "iterations" not in STANDARD_STATUS


# One hold vocabulary on the three tools that read samples between their times (AS-28): the
# step model's rollout, the noise sources and the replay of a trajectory say "zoh" / "linear"
RETIRED_HOLDS = {"foh": "linear", "previous": "zoh"}


@pytest.mark.parametrize("retired", sorted(RETIRED_HOLDS))
def test_no_retired_hold(retired):
    from minilink import Pendulum
    from minilink.blocks import TrajectorySource

    evaluator = Pendulum().compile(backend="numpy")
    for refused in (
        lambda: evaluator.rollout([0.0, 0.0], [[0.0], [0.0]], 0.0, 0.1, hold=retired),
        lambda: TrajectorySource([0.0, 1.0], [0.0, 1.0], hold=retired),
    ):
        with pytest.raises(ValueError, match=RETIRED_HOLDS[retired]):
            refused()


# The optimizer's native numbers are keywords (api-stabilization AS-27): options= is the
# external solver's dict, refused on gd and adam; the native method's statistics say method
@pytest.mark.parametrize("retired", ["maxiter", "gtol"])
def test_a_native_method_refuses_options(retired):
    from minilink.optimization import MathematicalProgram, Optimizer

    program = MathematicalProgram(n_z=1, J=lambda z: z @ z, grad_J=lambda z: 2 * z)
    with pytest.raises(ValueError, match="max_iterations="):
        Optimizer(program, z0=[1.0], method="gd", options={retired: 5})
    result = Optimizer(program, z0=[1.0], method="gd", max_iterations=3).solve()
    assert result.n_iterations == result.stats["nit"] == 3
    assert result.stats["method"] == "gd" and "algorithm" not in result.stats


# A retired value of a keyword: the value that replaced it. The simulator's solver names are the
# step model's words on every tool (the ruling of 2026-11-10)
RETIRED_SOLVERS = {"rk4_fixedsteps": "rk4", "euler_fixedsteps": "euler"}


@pytest.mark.parametrize("retired", sorted(RETIRED_SOLVERS))
def test_no_retired_solver_name(retired):
    from minilink import Pendulum
    from minilink.simulation.simulator import Simulator

    with pytest.raises(
        ValueError, match=f"Unknown solver.*'{RETIRED_SOLVERS[retired]}'"
    ):
        Simulator(Pendulum(), tf=0.1, n_samples=3, solver=retired)


# The optimizer's method names the algorithm (the ruling 12 of 2026-11-10): SciPy's spellings,
# the library left to backend= and the (method, backend) table
RETIRED_METHODS = {"scipy_slsqp": "slsqp", "scipy_trust_constr": "trust-constr"}


@pytest.mark.parametrize("retired", sorted(RETIRED_METHODS))
def test_no_retired_optimizer_method(retired):
    from minilink.optimization import MathematicalProgram, Optimizer
    from minilink.optimization.optimizer import METHODS

    program = MathematicalProgram(n_z=1, J=lambda z: z @ z, grad_J=lambda z: 2 * z)
    with pytest.raises(
        ValueError, match=f"Unknown optimizer method.*{RETIRED_METHODS[retired]}"
    ):
        Optimizer(program, z0=[1.0], method=retired)
    assert {method for method, _ in METHODS} >= set(RETIRED_METHODS.values())
    assert not any(retired == method for method, _ in METHODS)


def test_a_missing_method_backend_pair_is_refused_with_the_valid_list():
    from minilink.optimization.optimizer import preset_of

    with pytest.raises(ValueError, match="the methods on 'torch' are"):
        preset_of("slsqp", "torch")


def test_a_port_and_the_state_carry_lower_and_upper():
    from minilink import Pendulum

    plant = Pendulum()
    plant.add_input_port("w", lower=[-1.0], upper=[2.0])
    for signal in (
        plant.state,
        plant.inputs["u"],
        plant.inputs["w"],
        plant.outputs["y"],
    ):
        assert signal.box.lower.tolist() == signal.lower.tolist()
        assert signal.box.upper.tolist() == signal.upper.tolist()
        for retired in BOUND_WORDS:
            assert not hasattr(signal, retired), f"{signal.id}.{retired} is retired"
    assert plant.inputs["w"].lower.tolist() == [-1.0]


def public_objects():
    """Every exported class and function of the facades, once, plants aside."""
    found = {}
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        for name in FACADES:
            module = importlib.import_module(name)
            for attr in getattr(module, "__all__", []):
                obj = getattr(module, attr, None)
                home = getattr(obj, "__module__", "") or ""
                if not callable(obj) or home.startswith(PLANT_PACKAGE):
                    continue
                found[id(obj)] = (f"{name}.{attr}", obj)
    return list(found.values())


def words_of(qualname, obj):
    """The names a caller writes: parameters, public methods and fields, with where each sits."""
    words = []
    members = [(qualname, obj)]
    if inspect.isclass(obj):
        members = [(qualname, obj.__init__)]
        for member_name, member in vars(obj).items():
            if member_name.startswith("_"):
                continue
            words.append((member_name, f"{qualname}.{member_name}"))
            if callable(member):
                members.append((f"{qualname}.{member_name}", member))
        if dataclasses.is_dataclass(obj):
            words += [(f.name, f"{qualname}.{f.name}") for f in dataclasses.fields(obj)]
    for where, function in members:
        try:
            parameters = inspect.signature(function).parameters
        except (TypeError, ValueError):
            continue
        words += [(p, f"{where}({p}=)") for p in parameters]
    return words


def test_no_retired_word_on_the_public_surface():
    if not RETIRED:
        pytest.skip("no word retired yet")
    hits = sorted(
        f"{where}: {word!r} is retired, write {RETIRED[word][0]!r}"
        for qualname, obj in public_objects()
        for word, where in words_of(qualname, obj)
        if word in RETIRED and retired_on(qualname, RETIRED[word][1])
    )
    assert not hits, "\n".join(hits)


def retired_on(qualname, owners):
    """Whether a word is retired on this object: everywhere, or on the named owners."""
    if owners is None:
        return True
    return qualname.rsplit(".", 1)[-1] in owners


# The helpers that are module functions, never methods of the problems or the planner base
# (RULES 5.25): the refusals and readers a planner reads a problem and its latest solution through
MODULE_HELPERS = {
    "PlanningProblem": {
        "coerce_horizon",
        "coerce_state",
        "require_type",
        "check_state_shape",
        "require_member",
        "require_cost",
        "require_goal",
        "require_finite_horizon",
        "default_x0",
        "move_start",
        "sources_by_id",
        "part_trees",
    },
    "StepPlanningProblem": {"require_cost"},
    "Optimizer": {
        "start_vector",
        "decision_vector",
        "print_solve_preamble",
        "print_solve_report",
    },
    "Planner": {
        "require_solution",
        "require_cost",
        "require_goal",
        "solution_trajectory",
        "open_loop_controller",
    },
}


def test_the_helpers_are_module_functions_not_methods():
    from minilink.optimization.optimizer import Optimizer
    from minilink.planning.planner import Planner
    from minilink.planning.problems import PlanningProblem
    from minilink.planning.step_problems import StepPlanningProblem

    owners = {
        cls.__name__: cls
        for cls in (PlanningProblem, StepPlanningProblem, Planner, Optimizer)
    }
    hits = sorted(
        f"{owner}.{name}"
        for owner, names in MODULE_HELPERS.items()
        for name in names
        if hasattr(owners[owner], name)
    )
    assert not hits, f"module functions, not methods: {hits}"


# A drawing shortcut writes its keywords out (api-stabilization AS-32, DESIGN section 4: every
# editor shows them), the same keywords as the function it hands them to
SHORTCUTS = {
    "minilink.core.facades.plotting.SystemPlotting.plot_input_output_map": (
        "minilink.graphical.port_map.plot_input_output_map"
    ),
    "minilink.core.facades.plotting.SystemPlotting.plot_control_law": (
        "minilink.graphical.port_map.plot_control_law"
    ),
    "minilink.core.discrete.sampled_diagram.SampledDiagramSystem.animate": (
        "minilink.core.facades.animation.SystemAnimation.animate"
    ),
}


@pytest.mark.parametrize("path", sorted(SHORTCUTS))
def test_a_drawing_shortcut_writes_its_keywords_out(path):
    def keywords(dotted):
        function = getattr(owner_of(dotted), dotted.rsplit(".", 1)[-1])
        parameters = inspect.signature(function).parameters.values()
        assert all(p.kind is not p.VAR_KEYWORD for p in parameters), dotted
        return {p.name: p.default for p in parameters if p.kind is p.KEYWORD_ONLY}

    assert keywords(path) == keywords(SHORTCUTS[path])


# One helper per name (api-stabilization AS-31): a helper is defined once, every other module
# imports it; two bodies under one name were two ideas, now two names
ONE_HELPER = {
    "steps_of": (
        "minilink.planning.evaluator",
        "minilink.planning.trajectory_optimization.path_integral",
        "minilink.planning.step_model",
    ),
    "planner_parts": (
        "minilink.planning.trajectory_optimization.planner",
        "minilink.planning.trajectory_optimization.compiled",
    ),
    "require_cost_to_go": (
        "minilink.planning.planner",
        "minilink.planning.comparison",
    ),
}


@pytest.mark.parametrize("name", sorted(ONE_HELPER))
def test_a_helper_name_has_one_body(name):
    home, *users = ONE_HELPER[name]
    defined = getattr(importlib.import_module(home), name)
    for user in users:
        module = importlib.import_module(user)
        if hasattr(module, name):
            assert getattr(module, name) is defined, f"{user}.{name} is another body"


# The workflow a planner's verb takes (DESIGN §6, the four kinds of argument): what is shown,
# never what is computed, a keyword of solve and never of the constructor
WORKFLOW = {
    "verbose",
    "callback",
    "live_plot",
    "record_history",
    "history_stride",
}
SEARCH_DISPLAY = {"callback"}
VERB_WORKFLOW = {
    "DynamicProgrammingPlanner": {"verbose", "record_history"},
    "TrajectoryOptimizationPlanner": {
        "verbose",
        "callback",
        "live_plot",
        "record_history",
    },
    "RRTPlanner": SEARCH_DISPLAY,
    "RRTStarPlanner": SEARCH_DISPLAY | {"record_history", "history_stride"},
    "ReinforcementLearningPlanner": {"verbose"},
    "TabularLearningPlanner": {"verbose"},
}


def planner_classes():
    """Every planner class of the library: the subclasses of Planner, at any depth."""
    import minilink.learning
    import minilink.planning
    from minilink.planning.planner import Planner

    # The facades load lazily: every exported name read once, so each planner registers
    for facade in (minilink.planning, minilink.learning):
        for name in facade.__all__:
            getattr(facade, name)

    found = []
    pending = list(Planner.__subclasses__())
    while pending:
        cls = pending.pop()
        if cls.__module__.startswith("minilink.") and cls not in found:
            found.append(cls)
        pending += cls.__subclasses__()
    return found


# The drawing library of a planning view is its renderer; backend= is the array module
PLANNING_VIEWS = ("plot_trajectory", "plot_cost")


def test_a_planning_view_takes_the_renderer_not_the_backend():
    from minilink.planning.results import PlanningSolution
    from minilink.planning.trajectory_optimization.live_plot import (
        LiveTrajectoryPlotCallback,
    )

    views = [getattr(PlanningSolution, name) for name in PLANNING_VIEWS]
    for view in [*views, LiveTrajectoryPlotCallback.__init__]:
        words = set(inspect.signature(view).parameters)
        assert "renderer" in words and "backend" not in words, view.__qualname__


def test_the_workflow_is_on_the_verb_never_on_the_constructor():
    classes = {cls.__name__: cls for cls in planner_classes()}
    assert set(VERB_WORKFLOW) <= set(classes)

    for name, cls in classes.items():
        constructor = set(inspect.signature(cls.__init__).parameters)
        assert not constructor & WORKFLOW, (
            f"{name}.__init__ takes {constructor & WORKFLOW}"
        )

    for name, words in VERB_WORKFLOW.items():
        cls = classes[name]
        verbs = [cls.solve] + ([cls.solve_steps] if hasattr(cls, "solve_steps") else [])
        for verb in verbs:
            missing = words - set(inspect.signature(verb).parameters)
            assert not missing, f"{name}.{verb.__name__} lacks {missing}"
