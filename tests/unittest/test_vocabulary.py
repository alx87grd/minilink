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
)
PLANT_PACKAGE = "minilink.dynamics"

# A retired word: the word of RULES 2.7 that replaced it, and the owners it is retired on
# (None: everywhere). A word keeps its other textbook meanings elsewhere: alpha is the CVaR
# level of Evaluation.cvar, N the reference gain of OutputReference. Each sweep adds its own.
RETIRED = {
    "gamma": ("radius_constant", ("RRTStarPlanner",)),
    "settings": ("params", None),
    "final_time": ("the problem's t0 + horizon", ("DynamicProgrammingPlanner",)),
    "x_start": ("x0", None),
    "t_start": ("t0", None),
    "iterations": (
        "n_iterations",
        ("Optimizer", "TrajectoryOptimizationPlanner", "MPPIPlanner"),
    ),
    "optimizer_options": ("options", None),
    "tf": ("horizon", ("PlanningProblem",)),
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
    "n": ("state_dim", ("System", "DynamicSystem", "StepSystem")),
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
    "solver": ("record", ("PlanningSolution",)),
    "algorithm": ("method", ("ReinforcementLearningPlanner", "TabularLearningPlanner")),
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
}


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
        "require_cost",
        "require_goal",
        "require_finite_horizon",
        "default_x0",
        "move_start",
        "sources_by_id",
        "part_trees",
    },
    "StepPlanningProblem": {"require_cost"},
    "Planner": {
        "require_solution",
        "require_cost",
        "require_goal",
        "solution_trajectory",
        "open_loop_controller",
    },
}


def test_the_helpers_are_module_functions_not_methods():
    from minilink.planning.planner import Planner
    from minilink.planning.problems import PlanningProblem
    from minilink.planning.step_problems import StepPlanningProblem

    owners = {
        cls.__name__: cls for cls in (PlanningProblem, StepPlanningProblem, Planner)
    }
    hits = sorted(
        f"{owner}.{name}"
        for owner, names in MODULE_HELPERS.items()
        for name in names
        if hasattr(owners[owner], name)
    )
    assert not hits, f"module functions, not methods: {hits}"


# The workflow a planner's verb takes (DESIGN §6, the four kinds of argument): what is shown,
# never what is computed, a keyword of solve and never of the constructor
WORKFLOW = {
    "verbose",
    "callback",
    "live_plot",
    "record_history",
    "live_plot_every",
    "live_plot_pause",
    "live_plot_after_goal_only",
    "live_plot_ax",
    "history_stride",
}
SEARCH_DISPLAY = {
    "callback",
    "live_plot",
    "live_plot_every",
    "live_plot_pause",
    "live_plot_after_goal_only",
    "live_plot_ax",
}
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
    import minilink.learning  # noqa: F401  (the learners register as subclasses)
    from minilink.planning.planner import Planner

    found = []
    pending = list(Planner.__subclasses__())
    while pending:
        cls = pending.pop()
        if cls.__module__.startswith("minilink.") and cls not in found:
            found.append(cls)
        pending += cls.__subclasses__()
    return found


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
