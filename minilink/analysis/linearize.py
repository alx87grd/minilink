"""Equilibrium linearization of any :class:`~minilink.core.system.System`.

``linearize_matrices`` returns the first-order matrices

    dDelta x = A Delta x + B Delta u
    Delta y  = C Delta x + D Delta u

about an operating point ``(x_bar, u_bar)``.  ``linearize`` wraps those
matrices in an :class:`~minilink.dynamics.abstraction.state_space.LTISystem`
for downstream control and analysis tools.
"""

from __future__ import annotations

import warnings

import numpy as np

from minilink.core.diagram import DiagramSystem
from minilink.dynamics.abstraction.state_space import LTISystem


class LinearizationFallbackWarning(RuntimeWarning):
    """Warning emitted when exact JAX linearization falls back to finite differences."""


def linearize_matrices(
    sys,
    x_bar,
    u_bar=None,
    *,
    inputs=None,
    outputs=None,
    method: str = "fd",
    t: float = 0.0,
    params=None,
    epsilon: float = 1e-6,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return ``A, B, C, D``.

    Parameters
    ----------
    sys : System
        System or diagram to linearize.
    x_bar : array of shape (n,)
        Operating-point state.
    u_bar : array of shape (m,), optional
        Full operating-point input. Defaults to the system's nominal port values.
    inputs : sequence of str, optional
        Boundary input ports whose perturbations form the columns of ``B`` and
        ``D``. Defaults to every input port.
    outputs : sequence, optional
        Output selectors forming the rows of ``C`` and ``D``. Boundary outputs
        are named by port id, e.g. ``"y"``. Diagram-internal outputs use
        ``(sys_id, port_id)`` tuples and are finite-difference only.
    method : {"fd", "jax"}, optional
        ``"fd"`` for central finite differences. ``"jax"`` tries exact JAX
        Jacobians and falls back to finite differences with a warning.
    t : float, optional
        Time at which to evaluate the Jacobians.
    params : dict, optional
        Parameter set forwarded to ``f`` and selected output computes.
    epsilon : float, optional
        Central-difference step when ``method="fd"`` or JAX falls back.

    Returns
    -------
    A, B, C, D : tuple of ndarray
        Linearized state, input, output, and feedthrough matrices.
    """
    xbar = np.asarray(x_bar, dtype=float).reshape(-1)
    if u_bar is None:
        u_bar = sys.get_u_from_input_ports()
    ubar = np.asarray(u_bar, dtype=float).reshape(-1)

    method = selectmethod(method, params=params)
    method, f, h, ubar = selectports(
        sys, ubar, inputs, outputs, method, t=t, params=params
    )

    if method == "fd":

        def jacobian(g, z):
            z = np.asarray(z, dtype=float).reshape(-1)
            n = z.size
            p = np.asarray(g(z), dtype=float).reshape(-1).size
            J = np.zeros((p, n))
            for i in range(n):
                dz = np.zeros(n)
                dz[i] = epsilon
                J[:, i] = (g(z + dz) - g(z - dz)) / (2.0 * epsilon)
            return J

        # dDelta x = A Delta x + B Delta u
        A = jacobian(lambda x: f(x, ubar), xbar)
        B = jacobian(lambda u: f(xbar, u), ubar)

        # Delta y = C Delta x + D Delta u
        C = jacobian(lambda x: h(x, ubar), xbar)
        D = jacobian(lambda u: h(xbar, u), ubar)
        return A, B, C, D

    import jax
    import jax.numpy as jnp

    x = jnp.asarray(xbar)
    u = jnp.asarray(ubar)

    # dDelta x = A Delta x + B Delta u
    A = jax.jacfwd(lambda x: f(x, u))(x)
    B = jax.jacfwd(lambda u: f(x, u))(u)

    # Delta y = C Delta x + D Delta u
    C = jax.jacfwd(lambda x: h(x, u))(x)
    D = jax.jacfwd(lambda u: h(x, u))(u)

    return (
        np.asarray(A, dtype=float),
        np.asarray(B, dtype=float),
        np.asarray(C, dtype=float),
        np.asarray(D, dtype=float),
    )


def linearize(
    sys,
    x_bar,
    u_bar=None,
    *,
    inputs=None,
    outputs=None,
    method: str = "fd",
    t: float = 0.0,
    params=None,
    epsilon: float = 1e-6,
):
    """Linearize ``sys`` about ``(x_bar, u_bar)`` and return an ``LTISystem``.

    This is a thin wrapper over :func:`linearize_matrices`.
    """
    A, B, C, D = linearize_matrices(
        sys,
        x_bar,
        u_bar,
        inputs=inputs,
        outputs=outputs,
        method=method,
        t=t,
        params=params,
        epsilon=epsilon,
    )

    lti = LTISystem(A, B, C, D, name=f"Linearized {sys.name}")
    lti.state.labels = [f"Delta {label}" for label in sys.state.labels]
    return lti


def selectmethod(method, *, params=None, reason=None):
    def warn(reason):
        message = " ".join(str(reason).split())
        warnings.warn(
            f"linearize_matrices(method='jax') fell back to finite differences: {message}",
            LinearizationFallbackWarning,
            stacklevel=3,
        )

    key = str(method).strip().lower()
    if key == "fd":
        return "fd"
    if key != "jax":
        raise ValueError(f"linearize method must be 'fd' or 'jax', got {method!r}")
    if reason is not None:
        warn(reason)
        return "fd"
    if params is not None:
        warn("explicit params are finite-difference only")
        return "fd"
    return "jax"


def selectports(sys, ubar, inputs, outputs, method, *, t, params):
    def astuple(selection, name):
        if selection is None:
            return None
        if isinstance(selection, str):
            return (selection,)
        try:
            return tuple(selection)
        except TypeError as exc:
            raise TypeError(f"{name} must be a string or sequence") from exc

    def available(mapping):
        return ", ".join(repr(key) for key in mapping) or "(none)"

    inputids = tuple(sys.inputs) if inputs is None else astuple(inputs, "inputs")
    inputindex = []
    for portid in inputids:
        if not isinstance(portid, str):
            raise TypeError("input selectors must be port-id strings")
        try:
            portslice = sys.get_input_port_slice(portid)
        except KeyError as exc:
            raise ValueError(
                f"Unknown input port {portid!r}; available: {available(sys.inputs)}"
            ) from exc
        inputindex.extend(range(portslice.start, portslice.stop))
    inputindex = np.asarray(inputindex, dtype=int)

    if outputs is None:
        if isinstance(sys, DiagramSystem) and sys.outputs:
            outputspecs = tuple(("boundary", portid) for portid in sys.outputs)
        elif "y" in sys.outputs:
            outputspecs = (("boundary", "y"),)
        else:
            outputspecs = (("state", None),)
    else:
        outputspecs = []
        for selector in astuple(outputs, "outputs"):
            if isinstance(selector, str):
                if selector not in sys.outputs:
                    raise ValueError(
                        f"Unknown output port {selector!r}; "
                        f"available: {available(sys.outputs)}"
                    )
                outputspecs.append(("boundary", selector))
                continue

            isinternal = (
                isinstance(selector, tuple)
                and len(selector) == 2
                and isinstance(selector[0], str)
                and isinstance(selector[1], str)
            )
            if not isinternal:
                raise TypeError(
                    "output selectors must be boundary port ids or "
                    "(sys_id, port_id) tuples"
                )
            if not isinstance(sys, DiagramSystem):
                raise ValueError("internal output selectors require a DiagramSystem")

            sysid, portid = selector
            if sysid not in sys.subsystems:
                raise ValueError(
                    f"Unknown subsystem {sysid!r}; "
                    f"available: {available(sys.subsystems)}"
                )
            subsystem = sys.subsystems[sysid]
            if portid not in subsystem.outputs:
                raise ValueError(
                    f"Unknown output port {sysid!r}:{portid!r}; "
                    f"available: {available(subsystem.outputs)}"
                )
            outputspecs.append(("internal", selector))
        outputspecs = tuple(outputspecs)

    def finiteinput(u):
        fullu = ubar.copy()
        fullu[inputindex] = u
        return fullu

    def finitef(x, u):
        fullu = finiteinput(u)
        return np.asarray(sys.f(x, fullu, t, params), dtype=float).reshape(-1)

    def finiteh(x, u):
        fullu = finiteinput(u)
        parts = []
        for kind, value in outputspecs:
            if kind == "state":
                y = x
            elif kind == "boundary":
                y = sys.outputs[value].compute(x, fullu, t, params)
            else:
                sysid, portid = value
                y = sys.compute_subsys_output_port(
                    x, fullu, t, sysid, portid, params=params
                )
            parts.append(np.asarray(y, dtype=float).reshape(-1))
        return np.concatenate(parts) if parts else np.array([], dtype=float)

    if method == "fd":
        return "fd", finitef, finiteh, ubar[inputindex]

    if any(kind == "internal" for kind, _ in outputspecs):
        method = selectmethod(
            "jax", reason="internal diagram outputs are finite-difference only"
        )
        return method, finitef, finiteh, ubar[inputindex]

    try:
        import jax.numpy as jnp

        evaluator = sys.compile(backend="jax")
        dynamics = evaluator.get_f_jit()
        outputs = evaluator.get_outputs_jit()
    except Exception as exc:
        method = selectmethod("jax", reason=exc)
        return method, finitef, finiteh, ubar[inputindex]

    u0 = jnp.asarray(ubar)
    jaxindex = jnp.asarray(inputindex)
    allinputs = np.array_equal(inputindex, np.arange(ubar.size))
    needsoutputs = any(kind == "boundary" for kind, _ in outputspecs)

    def jaxinput(u):
        if allinputs:
            return u
        return u0.at[jaxindex].set(u)

    def jaxf(x, u):
        return dynamics(x, jaxinput(u), t)

    def jaxh(x, u):
        fullu = jaxinput(u)
        outputvalues = outputs(x, fullu, t) if needsoutputs else {}
        parts = []
        for kind, value in outputspecs:
            if kind == "state":
                y = x
            else:
                y = outputvalues[value]
            parts.append(jnp.ravel(y))
        return jnp.concatenate(parts) if parts else jnp.zeros(0, dtype=x.dtype)

    return "jax", jaxf, jaxh, jnp.asarray(ubar[inputindex])


if __name__ == "__main__":
    from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

    pendulum = Pendulum()
    A, B, C, D = linearize_matrices(pendulum, x_bar=[0.0, 0.0])
    print("A =\n", np.round(A, 4))
    print("B =\n", np.round(B, 4))
    print("C =\n", np.round(C, 4))
    print("D =\n", np.round(D, 4))
    print("open-loop poles:", np.round(np.linalg.eigvals(A), 4))
