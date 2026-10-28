"""The optimizer: one mathematical program, compiled once, solved by a method preset."""

import time
from collections.abc import Callable
from dataclasses import replace

import numpy as np

from minilink.core.backends import array_module
from minilink.core.discrete.step_system import StepSystem
from minilink.core.inspect import inspect_text, repr_pretty
from minilink.optimization.evaluators.compiler import compile_program_evaluator
from minilink.optimization.mathematical_program import (
    MathematicalProgram,
    OptimizationResult,
)
from minilink.optimization.optimizers.optimizer_backend import (
    BackendIterateCallback,
    OptimizerBackend,
)
from minilink.optimization.reporting import (
    DISP_RULE_DIV,
    DISP_RULE_MAIN,
    preview_vector,
)

# User progress hook: current ``z``, objective ``J``, elapsed wall time ``t`` (seconds since solve start).
OptimizationProgressCallback = Callable[[np.ndarray, float, float], None]

# User-facing optimizer methods mapped to (backend_key, default_constructor_kwargs).
_USER_OPTIMIZER_METHODS: dict[str, tuple[str, dict]] = {
    "gd": ("first_order", {"algorithm": "gd", "options": {}}),
    "adam": ("first_order", {"algorithm": "adam", "options": {}}),
    "l-bfgs": (
        "scipy_minimize",
        {"scipy_method": "L-BFGS-B", "options": {}},
    ),
    "scipy_slsqp": (
        "scipy_minimize",
        {"scipy_method": "SLSQP", "options": {"disp": False}},
    ),
    "scipy_trust_constr": (
        "scipy_minimize",
        {"scipy_method": "trust-constr", "options": {}},
    ),
    "ipopt": ("ipopt", {"options": {}, "tol": 1e-8}),
}

# The native methods: a pure law run by the optimizer, with a fixed count or to tolerance
NATIVE_METHODS = ("gd", "adam")

# The program's class, read from its fields, and the methods each class admits
# ("auto" picks the first).
PROGRAM_CLASSES: dict[str, tuple[str, ...]] = {
    "unconstrained": (
        "l-bfgs",
        "gd",
        "adam",
        "scipy_slsqp",
        "scipy_trust_constr",
        "ipopt",
    ),
    "bounded": (
        "l-bfgs",
        "gd",
        "adam",
        "scipy_slsqp",
        "scipy_trust_constr",
        "ipopt",
    ),
    "constrained": ("scipy_slsqp", "scipy_trust_constr", "ipopt"),
}


###############################################################################
# Public API
###############################################################################


class Optimizer:
    """
    Bound optimizer for one mathematical program.

    It owns the program, its compiled evaluator, a default initial guess
    ``z0`` and a solver method preset; :meth:`solve` runs the compiled
    program from ``z0`` or a one-off override::

        program = MathematicalProgram(n_z=2, J=J, grad_J=grad_J)
        opt = Optimizer(program, z0=[0.0, 0.0], method="scipy_slsqp")
        result = opt.solve(verbose=True)

    An optional progress hook ``callback(z, J, t)`` on :meth:`solve` receives
    ``t``, the elapsed wall seconds; see :data:`OptimizationProgressCallback`.

    Parameters
    ----------
    program : MathematicalProgram
        Pure NLP description to compile and solve.
    z0 : array_like, optional
        Default initial decision vector for :meth:`solve`; unset, the program's
        own start (``program.metadata["z0"]``, set by an exported program).
    method : str, optional
        Optimizer method preset, named by algorithm: ``"gd"`` and ``"adam"``
        (first-order, projected on the box), ``"l-bfgs"``, ``"scipy_slsqp"``,
        ``"scipy_trust_constr"``, ``"ipopt"``; or ``"auto"``, the default of the
        program's class (see :func:`program_class`): L-BFGS for an unconstrained
        or box-bounded program, SLSQP for a constrained one. A method the class
        does not admit is refused (``gd`` on a program with constraint rows).
    backend : str, optional
        Mathematical-program evaluator backend, ``"numpy"`` or ``"jax"``.
        Defaults to ``program.backend`` (the native backend of the program's
        callables).
    use_hessian : bool, optional
        For JAX program evaluators, auto-generate a dense objective Hessian
        when no ``hess_J`` is supplied.
    options : dict, optional
        Solver options merged into the preset's ``options`` dict (user keys win).
    iterations : int, optional
        For the native methods (``"gd"``, ``"adam"``): exactly this many steps,
        no tolerance (R149). On a JAX program the solve is one ``lax.scan``,
        pure: ``solve(params=, z0=)`` traces under ``jit``, ``vmap`` and
        ``grad``. ``None`` (default) runs to the method's tolerance, its
        ``maxiter`` the cap. A size, structural: never a parameter (R131).
    **method_kwargs
        Extra keyword arguments override the preset's top-level kwargs and are
        forwarded to the backend constructor.
    """

    def __init__(
        self,
        program: MathematicalProgram,
        z0=None,
        *,
        method: str = "scipy_slsqp",
        backend: str | None = None,
        use_hessian: bool = False,
        options: dict | None = None,
        iterations: int | None = None,
        **method_kwargs,
    ):
        # The method: the class default for "auto", refused when the class does not admit it
        method = method_for(program, method)
        self.iterations = count_of(method, iterations)

        # The program, compiled once at the default start
        self.program = program
        self.z0 = self.decision_vector(start_of(program, z0), program.n_z)
        self.method = method
        self.backend = program.backend if backend is None else backend
        self.program_evaluator = compile_program_evaluator(
            program,
            backend=self.backend,
            sample_z=self.z0,
            use_hessian=use_hessian,
        )

        # The solver adapter: the method's preset, the caller's keywords and options over it
        self.adapter = make_optimizer_backend(method, options=options, **method_kwargs)

    def __str__(self):
        return inspect_text(self)

    def _repr_pretty_(self, p, cycle):
        repr_pretty(self, p, cycle)

    def solve(
        self,
        *,
        params=None,
        z0=None,
        callback: OptimizationProgressCallback | None = None,
        record_solve_time: bool = False,
        verbose: bool = False,
    ) -> OptimizationResult:
        """
        Solve the bound finite-dimensional mathematical program.

        Parameters
        ----------
        params : dict, optional
            The values of the program's parameters for this solve, a partial
            tree merged once onto the program's defaults (R130; a path the
            program does not declare raises). ``None`` solves at the defaults.
            The compiled program is reused: a new value never recompiles.
        z0 : array_like, optional
            One-off initial decision vector, the warm start. If omitted, use
            the default ``z0`` supplied to :class:`Optimizer`.
        record_solve_time : bool, optional
            If True, set
            :attr:`~minilink.optimization.mathematical_program.OptimizationResult.solve_time_s`
            to the wall-clock duration (seconds) of the backend solve only,
            measured with :func:`time.perf_counter`.
        callback : callable, optional
            If not ``None``, called during the solve as ``callback(z, J, t)``:
            current decision ``z`` (1-D :class:`~numpy.ndarray`), objective
            :math:`J(z)`, and ``t`` elapsed wall time (seconds) since the
            backend solve started. Each invocation evaluates ``J(z)`` again
            (one extra objective call per iterate). Not supported by the Ipopt
            backend (native path); use SciPy or omit.
        verbose : bool, optional
            If True, print the framed solve panel (preamble and report).
        """
        # A fixed count of native steps: the pure solve
        if self.iterations is not None:
            require_no_hook(callback)
            return self.solve_count(params, z0)

        z_start = self.z0 if z0 is None else self.decision_vector(z0, self.program.n_z)
        time_solve = record_solve_time or verbose

        # The values of this solve, bound once at the boundary
        self.program_evaluator.bind(params)

        if verbose:
            self.print_solve_preamble(z_start)
        if time_solve:
            t0 = time.perf_counter()

        # The user's hook, fed (z, J(z), elapsed time) at each iterate of the backend
        backend_cb: BackendIterateCallback | None = None
        if callback is not None:
            t_cb0 = time.perf_counter()

            def backend_cb(z: np.ndarray) -> None:
                z_arr = self.decision_vector(z, self.program.n_z)
                J = self.program_evaluator.objective(z_arr)
                t_elapsed = time.perf_counter() - t_cb0
                callback(z_arr.copy(), J, t_elapsed)

        # The backend solve
        result = self.adapter.solve(
            self.program_evaluator,
            z_start,
            callback=backend_cb,
        )

        # The wall-clock time of the backend solve alone
        if time_solve:
            elapsed = time.perf_counter() - t0
            result = replace(result, solve_time_s=elapsed)

        if verbose:
            self.print_solve_report(result)

        return result

    def as_step_system(self, params=None):
        """
        The solve as a step system (R132): the state the law's, one tick one iteration.

        ``x`` is the law's state flattened (``z``; with Adam its moments and step
        count after it), ``x0`` the law's start from :attr:`z0`, the update
        ``x_{k+1} = S(x_k, ∇J(z_k; p))`` and the output ``y = z``; ``params``
        the program's tree (``None`` its defaults). A view of the same kernel,
        so ``compute_rollout(K)`` gives the iterates of a ``K``-step solve.
        """
        require_native(self.method)
        return IterationSystem(self, self.program_evaluator.merged(params))

    def solve_count(self, params, z0):
        """Exactly :attr:`iterations` native steps from ``z0`` on the merged tree: a pure solve, its result traced or on the host."""
        evaluator = self.program_evaluator
        K = self.iterations

        # The start and the tree, merged once, bound to nothing
        tree = evaluator.merged(params)
        z_start = self.z0 if z0 is None else z0

        # z_K = S^K(z0; p), and J(z_K; p)
        z, J = self.adapter.run(evaluator, z_start, tree, K)

        return OptimizationResult(
            z=host_or_traced(z),
            success=True,
            cost=host_or_traced(J, float),
            message=f"{K} steps taken",
            stats={"nit": K, "algorithm": self.method},
        )

    ###########################################################################
    # Internal machinery
    ###########################################################################

    @staticmethod
    def decision_vector(z, n_z: int) -> np.ndarray:
        """``z`` as a flat float array of shape ``(n_z,)``."""
        arr = np.asarray(z, dtype=float).reshape(-1)
        if arr.shape != (int(n_z),):
            raise ValueError(f"decision vector must have shape ({int(n_z)},)")
        return arr

    def print_solve_preamble(self, z0: np.ndarray) -> None:
        """Open the solve panel: the program, the start and the method."""
        print()
        print(DISP_RULE_MAIN)
        print("===               Optimization Program                   ===")
        print(DISP_RULE_MAIN)
        for line in inspect_text(self).splitlines():
            print(line)
        print("z0:", preview_vector(z0))
        print("options:", getattr(self.adapter, "options", {}))
        print(DISP_RULE_DIV)
        print("Running solver...")

    def print_solve_report(
        self,
        result: OptimizationResult,
    ) -> None:
        """Print the **After solve** section and close the solve panel."""
        print(DISP_RULE_DIV)
        for line in inspect_text(result).splitlines():
            print(line)
        print("stats:", result.stats)
        print(DISP_RULE_MAIN)


def make_optimizer_backend(method: str, *, options: dict | None = None, **kwargs):
    """
    The solver adapter of a method preset: the one method table of every solve path.

    ``method`` names a preset of :class:`Optimizer` (``"scipy_slsqp"``,
    ``"ipopt"``, ``"adam"``, …; not ``"auto"``, which needs the program's
    class); ``options`` are merged over the preset's solver options and
    ``kwargs`` over its constructor keywords. The adapter solves any program
    evaluator from a start, ``adapter.solve(evaluator, z0)``, whether the
    evaluator came from one transcription or a compiled program with
    parameters (OPW-1).
    """
    require_method(method)
    require_named_method(method)
    adapter_key, preset = _USER_OPTIMIZER_METHODS[method]
    backend_kwargs = backend_kwargs_of(preset, kwargs, options)

    return adapter_of(adapter_key, backend_kwargs)


###############################################################################
# Internal machinery
###############################################################################


def adapter_of(adapter_key: str, kwargs: dict) -> OptimizerBackend:
    """The solver adapter of a preset's key, built from its keyword arguments."""
    if adapter_key == "scipy_minimize":
        # Lazy import: SciPy's solvers load on first use, not with the optimizer
        from minilink.optimization.optimizers.scipy_minimize import (
            ScipyMinimizeOptimizer,
        )

        return ScipyMinimizeOptimizer(**kwargs)

    if adapter_key == "ipopt":
        # Lazy import: the Ipopt adapter loads on first use, not with the optimizer
        from minilink.optimization.optimizers.ipopt import (
            IpoptOptimizer,
            require_cyipopt,
        )

        # Ipopt is an optional dependency: refuse early when cyipopt is missing
        require_cyipopt()

        return IpoptOptimizer(**kwargs)

    if adapter_key == "first_order":
        # Lazy import: the first-order loop loads on first use
        from minilink.optimization.optimizers.first_order import (
            FirstOrderOptimizer,
        )

        return FirstOrderOptimizer(**kwargs)

    raise ValueError(f"Unknown optimizer adapter key {adapter_key!r}.")


class IterationSystem(StepSystem):
    """A native solve as a step system: the law's state flattened, one tick one step (:meth:`Optimizer.as_step_system`)."""

    def __init__(self, optimizer, tree):
        evaluator = optimizer.program_evaluator
        law = optimizer.adapter.law(evaluator)
        state0 = law.init(np.asarray(optimizer.z0, dtype=float))

        # The state's layout: each entry of the law's state, flattened in order
        self.law = law
        self.evaluator = evaluator
        self.keys = tuple(state0)
        self.shapes = tuple(np.shape(state0[key]) for key in self.keys)
        n = int(sum(int(np.prod(shape)) for shape in self.shapes))

        n_z = int(optimizer.program.n_z)
        StepSystem.__init__(self, n, output_dim=n_z)
        self.name = f"{optimizer.method} on the program"
        self.params = tree
        self.x0 = self.flat(state0)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params

        state = self.unflat(x)

        # x_{k+1} = S(x_k, ∇J(z_k; p))
        grad = self.evaluator.native_gradient(state["z"], params)
        state_next = self.law.update(grad, state)

        return self.flat(state_next)

    def h(self, x, u, k=0, params=None):
        return self.unflat(x)["z"]

    def flat(self, state):
        """The law's state as one flat vector, its entries in order."""
        xp = array_module(*state.values())
        pieces = [
            xp.reshape(xp.asarray(state[key], dtype=float), (-1,)) for key in self.keys
        ]

        return xp.concatenate(pieces)

    def unflat(self, x):
        """The flat vector as the law's state (the step count a whole number)."""
        state = {}
        start = 0
        for key, shape in zip(self.keys, self.shapes):
            size = int(np.prod(shape))
            state[key] = x[start : start + size].reshape(shape)
            start += size

        return state


def host_or_traced(value, cast=None):
    """A concrete result as a NumPy value (or ``cast`` of it), a traced one as it is."""
    try:
        concrete = np.asarray(value, dtype=float)
    except TypeError:  # a tracer has no value
        return value

    return concrete if cast is None else cast(concrete)


def count_of(method, iterations):
    """The fixed step count: ``None``, or a positive whole number for a native method."""
    if iterations is None:
        return None
    require_native(method)
    if isinstance(iterations, bool) or int(iterations) != iterations or iterations < 1:
        raise ValueError(
            f"iterations must be a positive whole number, got {iterations!r}"
        )

    return int(iterations)


def require_native(method):
    """Refuse a fixed count, or the step-system view, for a method that is not native."""
    if method not in NATIVE_METHODS:
        raise ValueError(
            f"iterations= and as_step_system take the native methods {NATIVE_METHODS}; "
            f"{method!r} runs to its own tolerance"
        )


def require_no_hook(callback):
    """Refuse a progress hook on a fixed-count solve: the steps are one pure scan."""
    if callback is not None:
        raise ValueError(
            "a fixed-count solve (iterations=) is one pure scan: no callback; "
            "use as_step_system().compute_rollout(K) for the iterates"
        )


def program_class(program) -> str:
    """The program's class from its fields: ``"constrained"`` (rows ``g`` or ``h``), ``"bounded"`` (a finite box face) or ``"unconstrained"``."""
    if program.g is not None or program.h is not None:
        return "constrained"

    faces = [side for side in (program.lower, program.upper) if side is not None]
    if any(np.any(np.isfinite(side)) for side in faces):
        return "bounded"

    return "unconstrained"


def method_for(program, method):
    """The preset to run: the class default for ``"auto"``; refuse one the class does not admit."""
    require_method(method)
    kind = program_class(program)
    admitted = PROGRAM_CLASSES[kind]

    if method == "auto":
        return admitted[0]
    if method not in admitted:
        raise ValueError(
            f"method {method!r} does not take a {kind} program; it admits "
            f"{', '.join(admitted)} (or method='auto')"
        )

    return method


def start_of(program, z0):
    """The default start: ``z0`` given, else the program's own (``metadata["z0"]``)."""
    if z0 is not None:
        return z0
    if "z0" in program.metadata:
        return program.metadata["z0"]

    raise ValueError(
        "Optimizer needs z0: this program carries no start (metadata['z0'])"
    )


def require_method(method):
    """Refuse a method that names no preset."""
    if method != "auto" and method not in _USER_OPTIMIZER_METHODS:
        valid = ", ".join(["auto", *sorted(_USER_OPTIMIZER_METHODS)])
        raise ValueError(
            f"Unknown optimizer method {method!r}. Expected one of: {valid}."
        )


def require_named_method(method):
    """Refuse ``"auto"`` where no program is at hand to choose from."""
    if method == "auto":
        raise ValueError(
            "method='auto' picks from a program's class: give the program to "
            "Optimizer, or name the method"
        )


def backend_kwargs_of(preset, method_kwargs, options):
    """The backend's constructor keywords: the preset's, then the caller's keywords, then the merged options."""
    # A copy of each preset dict, so a backend never edits the shared preset
    kwargs = {}
    for key, value in preset.items():
        kwargs[key] = dict(value) if isinstance(value, dict) else value
    kwargs.update(method_kwargs)

    # The caller's options win over the preset's
    if options is not None:
        existing = kwargs.get("options", {})
        kwargs["options"] = {**existing, **options}

    return kwargs


if __name__ == "__main__":
    from minilink.optimization.mathematical_program import MathematicalProgram

    z_lo = 0.6
    z_hi = 2.05

    def J(z: np.ndarray):
        y = z**3 + z * z + np.sin(5.0 * np.pi * z) + 0.12 * np.sin(15.0 * np.pi * z)
        return y

    def g(z: np.ndarray) -> np.ndarray:
        return np.array([z[0] - z_lo, z_hi - z[0]], dtype=float)

    prog = MathematicalProgram(
        n_z=1,
        J=J,
        g=g,
    )

    opt = Optimizer(
        prog,
        z0=np.array([0.0]),
        method="scipy_slsqp",
        options={
            "disp": False,
            "maxiter": 200,
            "ftol": 1e-12,
        },
    )

    def callback(z: np.ndarray, J: float, t: float) -> None:
        print(f"z: {z}, J: {J}, t: {t}")

    out = opt.solve(
        callback=callback,
        verbose=True,
    )
    print(out)
