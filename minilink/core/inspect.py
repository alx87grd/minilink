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


def body_line(label, value):
    """One indented ``label: value`` line of a summary's body."""
    return f"  {label}: {value}"


def inspect_inline(obj):
    """One line for nesting inside another object's body."""
    return inspect_text(obj).replace("\n", "; ")


def inspect_text(obj):
    """One short summary for a textbook object: the core objects' here, any other its own ``str``."""
    # Real cycles: every module below imports inspect_text for its own display
    from minilink.core.costs import CostFunction, StepCost
    from minilink.core.fields import Field
    from minilink.core.sets import InputSet, Set
    from minilink.core.system import System
    from minilink.core.trajectory import Trajectory

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

    if isinstance(obj, (CostFunction, StepCost)):
        return cost_text(obj)
    if isinstance(obj, Field):
        return field_text(obj)
    if isinstance(obj, (Set, InputSet)):
        return type(obj).__name__

    # Any other object prints its own text (a tool band's problem, solution,
    # program or simulator writes it beside its class), else its repr
    return str(obj)


def latex_inspect(obj):
    """Notebook LaTeX summary when meaningful (the core objects' here, any other its own ``_repr_latex_``); otherwise ``None``."""
    # A real cycle: costs imports this module
    from minilink.core.costs import QuadraticCost

    if isinstance(obj, QuadraticCost):
        return quadratic_cost_latex()

    # Any other object writes its own summary beside its class, or has none
    own = getattr(obj, "_repr_latex_", None)
    if own is None:
        return None

    return own()


def repr_pretty(obj, p, cycle):
    """IPython pretty-printer: the inspect summary, not the dataclass dump."""
    p.text("..." if cycle else inspect_text(obj))


###############################################################################
# Internal machinery
###############################################################################


def array_text(values):
    return np.array2string(np.asarray(values), precision=3, separator=", ")


def box_interval(lower, upper, var):
    return f"{array_text(lower)} <= {var} <= {array_text(upper)}"


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
    from minilink.core.fields import EllipsoidSet, FieldSet
    from minilink.core.sets import (
        BallSet,
        BoxInputSet,
        BoxSet,
        IntersectionSet,
        SingletonSet,
    )

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
    if isinstance(obj, EllipsoidSet):
        center = array_text(obj.params["center"])
        return f"EllipsoidSet, dim={obj.params['center'].size}, (z - {center})ᵀ P (z - {center}) <= {obj.level:g}"
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
        return f"Gaussian, dim={obj.dim}, mean={array_text(obj.mean)}, " + (
            f"std={array_text(obj.std)}"
            if obj.params.get("cov") is None
            else f"cov={array_text(obj.params['cov'])}"
        )
    if isinstance(obj, Uniform):
        box = obj.box
        return f"Uniform, dim={obj.dim}, {box_interval(box.lower, box.upper, 'x')}"
    if isinstance(obj, Particles):
        return f"Particles, dim={obj.dim}, N={obj.points.shape[0]}"
    if isinstance(obj, Distribution):
        return f"{type(obj).__name__}, dim={obj.dim}, mean={array_text(obj.mean)}"

    return None


def cost_text(cost):
    # A real cycle: costs imports inspect_text at module level
    from minilink.core.costs import QuadraticCost, ScaledCost, SumCost

    cls = type(cost).__name__
    if isinstance(cost, QuadraticCost):
        params = cost.params
        body = [
            body_line("x_bar", array_text(params["x_bar"])),
            body_line("u_bar", array_text(params["u_bar"])),
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


def quadratic_cost_latex():
    return (
        r"\begin{align*}"
        r"g(x,u) &= (x-\bar{x})^\top Q (x-\bar{x}) + (u-\bar{u})^\top R (u-\bar{u}) \\"
        r"h(x) &= (x-\bar{x})^\top S (x-\bar{x})"
        r"\end{align*}"
    )
