"""Short text summaries for core objects (``print(obj)``, notebook fallback)."""

import numpy as np

###############################################################################
# Public API
###############################################################################


def inspect_text(obj):
    """One short summary for a textbook object; unknown types use ``repr``."""
    # Real cycles: every module below imports inspect_text for its own display
    from minilink.core.costs import CostFunction
    from minilink.core.distributions import Distribution, Gaussian, Particles, Uniform
    from minilink.core.sets import (
        BallSet,
        BoxInputSet,
        BoxSet,
        InputSet,
        IntersectionSet,
        Set,
        SingletonSet,
    )
    from minilink.core.system import System
    from minilink.core.trajectory import Trajectory
    from minilink.optimization.mathematical_program import MathematicalProgram
    from minilink.planning.problems import PlanningProblem
    from minilink.planning.spatial.state_fields import FieldSet, StateField

    # Systems and trajectories
    if isinstance(obj, System):
        return system_text(obj)
    if isinstance(obj, Trajectory):
        return trajectory_text(obj)

    # Sets, the specific ones before their bases
    if isinstance(obj, BoxSet):
        lower = array_text(obj.lower)
        upper = array_text(obj.upper)
        return f"BoxSet, dim={obj.dim}, {lower} <= z <= {upper}"
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
        lower = array_text(box.lower)
        upper = array_text(box.upper)
        return f"BoxInputSet, dim={box.dim}, {lower} <= u <= {upper}"
    if isinstance(obj, (Set, InputSet)):
        return type(obj).__name__

    # Distributions, the specific ones before their base
    if isinstance(obj, Gaussian):
        mean = array_text(obj.mean)
        std = array_text(obj.std)
        return f"Gaussian, dim={obj.dim}, mean={mean}, std={std}"
    if isinstance(obj, Uniform):
        box = obj.box
        lower = array_text(box.lower)
        upper = array_text(box.upper)
        return f"Uniform, dim={obj.dim}, {lower} <= x <= {upper}"
    if isinstance(obj, Particles):
        return f"Particles, dim={obj.dim}, N={obj.points.shape[0]}"
    if isinstance(obj, Distribution):
        return f"{type(obj).__name__}, dim={obj.dim}, mean={array_text(obj.mean)}"

    # Costs, state fields, planning problems and programs
    if isinstance(obj, CostFunction):
        return f"{type(obj).__name__}, rho={float(obj.discount_rate):g}"
    if isinstance(obj, StateField):
        return type(obj).__name__
    if isinstance(obj, PlanningProblem):
        return problem_text(obj)
    if isinstance(obj, MathematicalProgram):
        return program_text(obj)

    return repr(obj)


def repr_pretty(obj, p, cycle):
    """IPython pretty-printer: the inspect summary, not the dataclass dump."""
    p.text("..." if cycle else inspect_text(obj))


###############################################################################
# Internal machinery
###############################################################################


def system_text(sys):
    """The system's name, class and state size, then its ports and blocks."""
    cls = type(sys).__name__
    name = sys.name

    head = f"{name} ({cls}), n={sys.n}" if name != cls else f"{name}, n={sys.n}"
    lines = [head]

    if sys.inputs:
        ports = ", ".join(f"{pid} ({port.dim})" for pid, port in sys.inputs.items())
        lines.append(f"  inputs: {ports}")
    if sys.outputs:
        ports = ", ".join(f"{pid} ({port.dim})" for pid, port in sys.outputs.items())
        lines.append(f"  outputs: {ports}")

    subsystems = getattr(sys, "subsystems", None)
    if subsystems:
        lines.append(f"  blocks: {', '.join(subsystems)}")

    return "\n".join(lines)


def trajectory_text(traj):
    """The trajectory's samples, time span and shapes, then its extra signals."""
    extra = [name for name in traj.signal_names if name not in {"x", "u"}]

    lines = [
        f"Trajectory, N={traj.n_samples}, t={traj.t0:g}–{traj.tf:g}, "
        f"x {tuple(traj.x.shape)}, u {tuple(traj.u.shape)}"
    ]
    if extra:
        lines.append(f"  signals: {', '.join(extra)}")

    return "\n".join(lines)


def array_text(values):
    """An array printed to three decimals, its entries separated by commas."""
    return np.array2string(np.asarray(values), precision=3, separator=", ")


def problem_text(problem):
    """The problem's class, system and horizon, then the summary of each set and cost."""
    sys = problem.sys
    name = getattr(sys, "name", type(sys).__name__)
    tf = problem.tf

    # The horizon: none, infinite, or a time
    if tf is None:
        tf_s = "None"
    elif not np.isfinite(tf):
        tf_s = "inf"
    else:
        tf_s = f"{tf:g}"

    lines = [f"{type(problem).__name__}, {name}, tf={tf_s}"]
    for label, item in (
        ("X", problem.X),
        ("X0", problem.X0),
        ("Xf", problem.Xf),
        ("U", problem.U),
        ("cost", problem.cost),
    ):
        if item is not None:
            lines.append(f"  {label}: {inspect_text(item)}")

    return "\n".join(lines)


def program_text(program):
    """The program's decision size, which constraints it has, and which bounds."""
    lines = [f"MathematicalProgram, n_z={program.n_z}"]
    lines.append(
        f"  h: {'yes' if program.h is not None else 'no'}, "
        f"g: {'yes' if program.g is not None else 'no'}"
    )

    sides = []
    if program.lower is not None:
        sides.append("lower")
    if program.upper is not None:
        sides.append("upper")
    if sides:
        lines.append(f"  bounds: {', '.join(sides)}")

    return "\n".join(lines)
