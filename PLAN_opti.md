# Clean Optimization Module Refactor

## Summary
Rebuild `minilink.optimization` around a pure NLP description plus compiled evaluators.

The target user flow is:

```python
program = MathematicalProgram(
    n_z=2,
    J=J,
    h=h,
    g=g,
    lower=lower,
    upper=upper,
    grad_J=grad_J,
    hess_J=hess_J,
    jac_h=jac_h,
    jac_g=jac_g,
)

opt = Optimizer(
    program,
    z0=np.array([0.5, 0.5]),
    method="scipy_slsqp",
    compile_backend="jax",
    options={"maxiter": 200},
)

result = opt.solve()
```

This pass intentionally focuses only on the optimization module. Trajectory optimization can break temporarily and will be updated later.

## Key Changes
- Make `MathematicalProgram` a pure finite-dimensional NLP:
  - required: `n_z`, `J`
  - optional: `h`, `g`, `lower`, `upper`, `grad_J`, `hess_J`, `jac_h`, `jac_g`, `metadata`
  - no `z0`
  - no solver-facing wrappers
  - no `EqualityConstraint` / `InequalityConstraint`
- Add `compile_program_evaluator(program, backend="numpy", sample_z=None, use_hessian=False, verbose=False)`.
  - Evaluator classes live under `minilink.optimization.evaluators`.
  - `sample_z` defaults to `zeros(n_z)` for manual compile calls; `Optimizer` passes its `z0`.
- Add evaluator API:
  - backend-native math methods: `J(z)`, `h(z)`, `g(z)`
  - solver adapters: `objective(z)`, `equality_residual(z)`, `inequality_margin(z)`
  - derivative adapters: `gradient(z)`, `hessian(z)`, `jacobian_h(z)`, `jacobian_g(z)`
  - missing `h/g` return empty vectors
- Add NumPy and JAX evaluators:
  - NumPy evaluator uses provided derivative callables only.
  - JAX evaluator lazily imports JAX, checks traceability, JITs `J/h/g`, auto-generates `grad_J`, `jac_h`, and `jac_g`, and generates dense `hess_J` only when `use_hessian=True`.

## Optimizer API
- Change `Optimizer` into a bound solver object:
  - `Optimizer(program, z0, method="scipy_slsqp", compile_backend="numpy", options=None, **method_kwargs)`
  - compile evaluator in `__init__`
  - store default `z0`
  - `solve(z0=None, callback=None, record_solve_time=False, disp=False)` runs without recompiling
- Replace public `backend=...` with method presets:
  - `method="scipy_slsqp"`
  - `method="scipy_trust_constr"`
  - `method="ipopt"`
- Keep backend adapter names internal:
  - `scipy_slsqp -> scipy_minimize + scipy_method="SLSQP"`
  - `scipy_trust_constr -> scipy_minimize + scipy_method="trust-constr"`
  - `ipopt -> ipopt`
- Update backend adapters to consume `(evaluator, z0)` and never raw `MathematicalProgram`.

## Tests
- Add pure `MathematicalProgram` tests for `n_z`, bounds validation, metadata copy, and absence of `z0`.
- Add NumPy evaluator tests for scalar objective conversion, empty/missing constraints, flat `h/g`, derivative wrappers, and shape errors.
- Add optional JAX evaluator tests for JIT execution, auto `grad_J`, auto `jac_h/jac_g`, and opt-in `hess_J`.
- Update optimization solver tests for:
  - unconstrained quadratic
  - equality constraints through aggregate `h`
  - inequality constraints through aggregate `g`
  - variable bounds
  - callback reporting
  - `solve(z0=...)` override without rebuilding optimizer
  - optional Ipopt if `cyipopt` is installed
- Update optimization demo and benchmark to the new clean API.
- Trajectory optimization now uses the same aggregate `h` / `g` contract:
  transcriptions return pure programs and expose `pack_initial_guess(...)`
  separately for `z0`.

## Assumptions
- `h(z) = 0`, `g(z) >= 0`, and variable bounds remain the core NLP convention.
- Derivative names are `grad_J`, `hess_J`, `jac_h`, and `jac_g`.
- User-facing optimizer selection uses method presets, not backend names.
- Evaluators belong in `minilink.optimization.evaluators`, not `minilink.compile`.
- No finite-difference fallback, QP/LP classes, or trajectory-optimization migration in this pass.
