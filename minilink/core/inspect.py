"""Short text summaries for core objects (``print(obj)``, notebook fallback)."""

import numpy as np


def inspect_text(obj):
    """One short summary for a textbook object; unknown types use ``repr``."""
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

    if isinstance(obj, System):
        return _system_text(obj)
    if isinstance(obj, Trajectory):
        return _trajectory_text(obj)
    if isinstance(obj, BoxSet):
        return f"BoxSet, dim={obj.dim}, {_arr(obj.lower)} <= z <= {_arr(obj.upper)}"
    if isinstance(obj, SingletonSet):
        return f"SingletonSet, dim={obj.dim}, z = {_arr(obj.point)}"
    if isinstance(obj, BallSet):
        return f"BallSet, dim={obj.dim}, ||z - {_arr(obj.center)}|| <= {obj.radius:g}"
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
        return (
            f"BoxInputSet, dim={box.dim}, {_arr(box.lower)} <= u <= {_arr(box.upper)}"
        )
    if isinstance(obj, (Set, InputSet)):
        return type(obj).__name__
    if isinstance(obj, Gaussian):
        return f"Gaussian, dim={obj.dim}, mean={_arr(obj.mean)}, std={_arr(obj.std)}"
    if isinstance(obj, Uniform):
        box = obj.box
        return f"Uniform, dim={obj.dim}, {_arr(box.lower)} <= x <= {_arr(box.upper)}"
    if isinstance(obj, Particles):
        return f"Particles, dim={obj.dim}, N={obj.points.shape[0]}"
    if isinstance(obj, Distribution):
        return f"{type(obj).__name__}, dim={obj.dim}, mean={_arr(obj.mean)}"
    if isinstance(obj, CostFunction):
        return f"{type(obj).__name__}, rho={float(obj.discount_rate):g}"
    if isinstance(obj, StateField):
        return type(obj).__name__
    if isinstance(obj, PlanningProblem):
        return _problem_text(obj)
    if isinstance(obj, MathematicalProgram):
        return _program_text(obj)
    return repr(obj)


def repr_pretty(obj, p, cycle):
    """IPython pretty-printer: the inspect summary, not the dataclass dump."""
    p.text("..." if cycle else inspect_text(obj))


def _arr(values):
    return np.array2string(np.asarray(values), precision=3, separator=", ")


def _system_text(sys):
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


def _trajectory_text(traj):
    extra = [name for name in traj.signal_names if name not in {"x", "u"}]
    lines = [
        f"Trajectory, N={traj.n_samples}, t={traj.t0:g}–{traj.tf:g}, "
        f"x {tuple(traj.x.shape)}, u {tuple(traj.u.shape)}"
    ]
    if extra:
        lines.append(f"  signals: {', '.join(extra)}")
    return "\n".join(lines)


def _problem_text(problem):
    sys = problem.sys
    name = getattr(sys, "name", type(sys).__name__)
    tf = problem.tf
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


def _program_text(program):
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
