"""Short text summaries for core objects (``print(obj)``, notebook fallback)."""

import numpy as np

###############################################################################
# Public API
###############################################################################


def format_inspect(headline, body=None):
    """One headline and optional indented body lines joined for ``print(obj)``."""
    if not body:
        return headline

    return "\n".join([headline, *body])


def inspect_inline(obj):
    """One line for nesting inside another object's body."""
    return inspect_text(obj).replace("\n", "; ")


def inspect_text(obj):
    """One short summary for a textbook object; unknown types use ``repr``."""
    # Real cycles: every module below imports inspect_text for its own display
    from minilink.core.costs import CostFunction
    from minilink.core.sets import InputSet, Set
    from minilink.core.system import System
    from minilink.core.trajectory import Trajectory
    from minilink.optimization.mathematical_program import (
        MathematicalProgram,
        OptimizationResult,
    )
    from minilink.optimization.optimizer import Optimizer
    from minilink.planning.problems import PlanningProblem, StochasticPlanningProblem
    from minilink.planning.results import PlanningSolution
    from minilink.planning.spatial.state_fields import StateField

    if isinstance(obj, System):
        return system_text(obj)
    if isinstance(obj, Trajectory):
        return trajectory_text(obj)

    text = set_text(obj)
    if text is not None:
        return text

    text = distribution_text(obj)
    if text is not None:
        return text

    if isinstance(obj, CostFunction):
        return cost_text(obj)
    if isinstance(obj, StateField):
        return field_text(obj)
    if isinstance(obj, StochasticPlanningProblem):
        return stochastic_problem_text(obj)
    if isinstance(obj, PlanningProblem):
        return problem_text(obj)
    if isinstance(obj, PlanningSolution):
        return solution_text(obj)
    if isinstance(obj, Optimizer):
        return optimizer_text(obj)
    if isinstance(obj, OptimizationResult):
        return optimization_result_text(obj)
    if isinstance(obj, MathematicalProgram):
        return program_text(obj)

    # Hybrid diagram and simulator: lazy imports from outer bands
    from minilink.core.discrete.hybrid_diagram import HybridDiagram
    from minilink.simulation.simulator import Simulator

    if isinstance(obj, HybridDiagram):
        return hybrid_diagram_text(obj)
    if isinstance(obj, Simulator):
        return simulator_text(obj)

    if isinstance(obj, (Set, InputSet)):
        return type(obj).__name__

    return repr(obj)


def latex_inspect(obj):
    """Notebook LaTeX summary when meaningful; otherwise ``None``."""
    # A real cycle: costs, state_space and mathematical_program import this module
    from minilink.core.costs import QuadraticCost
    from minilink.dynamics.abstraction.state_space import LTISystem
    from minilink.optimization.mathematical_program import MathematicalProgram

    if isinstance(obj, LTISystem):
        return lti_latex()
    if isinstance(obj, QuadraticCost):
        return quadratic_cost_latex()
    if isinstance(obj, MathematicalProgram):
        return program_latex(obj)

    return None


def repr_pretty(obj, p, cycle):
    """IPython pretty-printer: the inspect summary, not the dataclass dump."""
    p.text("..." if cycle else inspect_text(obj))


###############################################################################
# Internal machinery
###############################################################################


def body_line(label, value):
    return f"  {label}: {value}"


def array_text(values):
    return np.array2string(np.asarray(values), precision=3, separator=", ")


def box_interval(lower, upper, var):
    return f"{array_text(lower)} <= {var} <= {array_text(upper)}"


def planning_horizon_text(tf):
    if tf is None:
        return "None"
    if not np.isfinite(tf):
        return "inf"

    return f"{tf:g}"


def system_text(sys):
    cls = type(sys).__name__
    name = sys.name
    head = f"{name} ({cls}), n={sys.n}" if name != cls else f"{name}, n={sys.n}"
    body = []

    if sys.inputs:
        ports = ", ".join(f"{pid} ({port.dim})" for pid, port in sys.inputs.items())
        body.append(body_line("inputs", ports))
    if sys.outputs:
        ports = ", ".join(f"{pid} ({port.dim})" for pid, port in sys.outputs.items())
        body.append(body_line("outputs", ports))

    subsystems = getattr(sys, "subsystems", None)
    if subsystems:
        body.append(body_line("blocks", ", ".join(subsystems)))

    return format_inspect(head, body)


def trajectory_text(traj):
    extra = [name for name in traj.signal_names if name not in {"x", "u"}]
    headline = (
        f"Trajectory, N={traj.n_samples}, t={traj.t0:g}–{traj.tf:g}, "
        f"x {tuple(traj.x.shape)}, u {tuple(traj.u.shape)}"
    )
    if not extra:
        return headline

    return format_inspect(headline, [body_line("signals", ", ".join(extra))])


def set_text(obj):
    # A real cycle: sets imports this module
    from minilink.core.sets import (
        BallSet,
        BoxInputSet,
        BoxSet,
        IntersectionSet,
        SingletonSet,
    )
    from minilink.planning.spatial.state_fields import FieldSet

    if isinstance(obj, BoxSet):
        return f"BoxSet, dim={obj.dim}, {box_interval(obj.lower, obj.upper, 'z')}"
    if isinstance(obj, SingletonSet):
        return f"SingletonSet, dim={obj.dim}, z = {array_text(obj.point)}"
    if isinstance(obj, BallSet):
        center = array_text(obj.center)
        return f"BallSet, dim={obj.dim}, ||z - {center}|| <= {obj.radius:g}"
    if isinstance(obj, IntersectionSet):
        parts = ", ".join(type(member).__name__ for member in obj.sets)
        return f"IntersectionSet of {parts}"
    if isinstance(obj, FieldSet):
        bounds = []
        if obj.lower is not None:
            bounds.append(f"{obj.lower:g} <= value")
        if obj.upper is not None:
            bounds.append(f"value <= {obj.upper:g}")
        return f"FieldSet ({type(obj.field).__name__}), {', '.join(bounds)}"
    if isinstance(obj, BoxInputSet):
        box = obj.box
        return f"BoxInputSet, dim={box.dim}, {box_interval(box.lower, box.upper, 'u')}"

    return None


def distribution_text(obj):
    # A real cycle: distributions import inspect_text at module level
    from minilink.core.distributions import Distribution, Gaussian, Particles, Uniform

    if isinstance(obj, Gaussian):
        return (
            f"Gaussian, dim={obj.dim}, mean={array_text(obj.mean)}, "
            f"std={array_text(obj.std)}"
        )
    if isinstance(obj, Uniform):
        box = obj.box
        return f"Uniform, dim={obj.dim}, {box_interval(box.lower, box.upper, 'x')}"
    if isinstance(obj, Particles):
        return f"Particles, dim={obj.dim}, N={obj.points.shape[0]}"
    if isinstance(obj, Distribution):
        return f"{type(obj).__name__}, dim={obj.dim}, mean={array_text(obj.mean)}"

    return None


def problem_parts(problem):
    sys = problem.sys
    name = getattr(sys, "name", type(sys).__name__)
    headline = (
        f"{type(problem).__name__}, {name}, "
        f"tf={planning_horizon_text(problem.tf)}, rho={problem.discount_rate:g}"
    )
    body = []
    for label, item in (
        ("X", problem.X),
        ("X0", problem.X0),
        ("Xf", problem.Xf),
        ("U", problem.U),
        ("cost", problem.cost),
    ):
        if item is not None:
            body.append(body_line(label, inspect_inline(item)))

    return headline, body


def problem_text(problem):
    headline, body = problem_parts(problem)
    return format_inspect(headline, body)


def stochastic_problem_text(problem):
    headline, body = problem_parts(problem)
    criterion = problem.criterion
    if isinstance(criterion, tuple):
        criterion_s = f"cvar, alpha={criterion[1]:g}"
    else:
        criterion_s = str(criterion)
    headline = f"{headline}, criterion={criterion_s}"

    body.append(body_line("x0", inspect_inline(problem.x0_distribution)))
    if problem.params_distribution:
        names = ", ".join(sorted(problem.params_distribution))
        n = len(problem.params_distribution)
        body.append(body_line("params", f"{n} laws ({names})"))
    if problem.disturbances:
        body.append(body_line("disturbances", ", ".join(sorted(problem.disturbances))))

    return format_inspect(headline, body)


def solution_text(solution):
    law = "u = pi(t), open loop" if solution.open_loop else "u = pi(x), feedback"
    if solution.trajectory is None:
        trajectory = "not rolled out (solve with evaluate=True)"
    else:
        trajectory = (
            f"{solution.trajectory.n_samples} samples over "
            f"{solution.trajectory.time_duration:.3g} s"
        )
    evaluation = "none" if solution.evaluation is None else str(solution.evaluation)
    success = "yes" if solution.success else "no"
    headline = f"PlanningSolution, method={solution.method}, success={success}"
    body = [
        body_line("policy", f"{solution.policy.name} ({law})"),
        body_line("trajectory", trajectory),
        body_line("evaluation", evaluation),
        body_line(
            "cost_to_go",
            "yes" if solution.cost_to_go is not None else "no",
        ),
        body_line("solver", str(solution.solver)),
    ]

    return format_inspect(headline, body)


def optimizer_text(opt):
    # Tools import core (DESIGN §3): the optimization band loads on first use
    from minilink.optimization.reporting import preview_vector

    headline = f"Optimizer, method={opt.method!r}"
    body = [
        body_line("program", inspect_inline(opt.program)),
        body_line("z0", preview_vector(opt.z0)),
        body_line("compile_backend", opt.compile_backend),
    ]

    return format_inspect(headline, body)


def optimization_result_text(result):
    # Tools import core (DESIGN §3): the optimization band loads on first use
    from minilink.optimization.reporting import preview_vector

    success = "yes" if result.success else "no"
    headline = f"OptimizationResult, success={success}"
    body = []

    if result.cost is not None:
        body.append(body_line("J*", f"{float(result.cost):.6g}"))
    body.append(body_line("z*", preview_vector(result.z)))
    if result.message:
        message = result.message.strip()
        if len(message) > 72:
            message = message[:69] + "..."
        body.append(body_line("message", message))
    if result.solve_time_s is not None:
        body.append(body_line("solve_time_s", f"{result.solve_time_s:g}"))

    return format_inspect(headline, body)


def program_text(program):
    # A real cycle: optimizer imports this module
    from minilink.optimization.optimizer import program_class

    kind = program_class(program)
    has_h = program.h is not None
    has_g = program.g is not None
    headline = f"MathematicalProgram, n_z={program.n_z}"
    body = [
        body_line("class", kind),
        body_line("h", f"{'yes' if has_h else 'no'} (n={constraint_rows(program.h)})"),
        body_line("g", f"{'yes' if has_g else 'no'} (n={constraint_rows(program.g)})"),
        body_line("backend", program.backend),
    ]

    sides = []
    if program.lower is not None:
        sides.append("lower")
    if program.upper is not None:
        sides.append("upper")
    if sides:
        body.append(body_line("bounds", ", ".join(sides)))

    return format_inspect(headline, body)


def cost_text(cost):
    # A real cycle: costs imports inspect_text at module level
    from minilink.core.costs import QuadraticCost, ScaledCost, SumCost

    cls = type(cost).__name__
    if isinstance(cost, QuadraticCost):
        params = cost.params
        body = [
            body_line("xbar", array_text(params["xbar"])),
            body_line("ubar", array_text(params["ubar"])),
            body_line("Q", matrix_summary(params["Q"])),
            body_line("R", matrix_summary(params["R"])),
            body_line("S", matrix_summary(params["S"])),
        ]
        return format_inspect(cls, body)

    if isinstance(cost, SumCost):
        terms = ", ".join(
            f"{term_id}: {type(term).__name__}" for term_id, term in cost.terms.items()
        )
        return format_inspect(
            f"{cls}, {len(cost.terms)} terms", [body_line("terms", terms)]
        )

    if isinstance(cost, ScaledCost):
        body = [
            body_line("weight", f"{cost.weight:g}"),
            body_line("cost", type(cost.cost).__name__),
        ]
        return format_inspect(cls, body)

    return cls


def field_text(field):
    cls = type(field).__name__
    body = []

    scene = getattr(field, "scene", None)
    if scene is not None:
        body.append(body_line("scene", type(scene).__name__))
    track = getattr(field, "track", None)
    if track is not None:
        body.append(body_line("track", type(track).__name__))
    body_obj = getattr(field, "body", None)
    if body_obj is not None:
        label = getattr(body_obj, "label", None) or type(body_obj).__name__
        body.append(body_line("body", label))

    if body:
        return format_inspect(cls, body)

    return cls


def hybrid_diagram_text(hybrid):
    computer = hybrid.computer
    plant = hybrid.plant
    diagram = getattr(computer, "diagram", None)
    computer_name = getattr(diagram, "name", type(computer).__name__)
    plant_name = getattr(plant, "name", type(plant).__name__)
    headline = f"HybridDiagram, computer={computer_name}, plant={plant_name}"
    body = [body_line("boundary connections", str(len(hybrid.connections)))]

    return format_inspect(headline, body)


def simulator_text(sim):
    headline = f"Simulator, solver={sim.solver_mode!r}, dt={sim.dt:g}, N={sim.n_pts}"
    body = [body_line("sys", inspect_inline(sim.sys))]

    return format_inspect(headline, body)


def constraint_rows(fn):
    if fn is None:
        return 0

    n_out = getattr(fn, "n_out", None)
    if n_out is not None:
        return int(n_out)

    return "?"


def matrix_summary(matrix):
    values = np.asarray(matrix, dtype=float)
    if values.ndim != 2:
        return array_text(values)

    n, m = values.shape
    if n == m and n <= 6:
        diag = np.diag(values)
        if np.allclose(values, np.diag(diag)):
            return f"diag {array_text(diag)}"

    return f"({n}, {m})"


def lti_latex():
    return (
        r"\begin{align*}"
        r"\dot{x} &= A x + B u \\"
        r"y &= C x + D u"
        r"\end{align*}"
    )


def quadratic_cost_latex():
    return (
        r"\begin{align*}"
        r"g(x,u) &= (x-\bar{x})^\top Q (x-\bar{x}) + (u-\bar{u})^\top R (u-\bar{u}) \\"
        r"h(x) &= (x-\bar{x})^\top S (x-\bar{x})"
        r"\end{align*}"
    )


def program_latex(program):
    parts = [r"\min_{z} J(z)"]
    if program.h is not None:
        parts.append(r"\text{subject to } h(z) = 0")
    if program.g is not None:
        parts.append(r"g(z) \geq 0")
    if program.lower is not None or program.upper is not None:
        parts.append(r"\text{box bounds on } z")

    body = r" \\ ".join(parts)

    return rf"\begin{{align*}}{body}\end{{align*}}"
