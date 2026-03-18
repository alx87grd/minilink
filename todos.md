# Minilink Development TODOs

The following tasks are structured in phases to systematically improve the API, architecture, and maintainability of the project.

### Phase 1: Modern Python Project Tooling
- [x] **Implement `pyproject.toml`:** Use a modern build backend (like Poetry or Hatch) to manage dependencies (NumPy, SciPy, Matplotlib, Graphviz) and project metadata.
- [x] **Continuous Integration (CI):** Set up a `.github/workflows/test.yml` to automatically run `pytest` on every push or pull request to the main branch.
- [x] **Linting and Formatting:** Introduce `ruff` or a combination of `black`, `isort`, and `flake8` using `pre-commit` hooks.
- [x] **Type Hinting:** Add `mypy` and use Python type hints (`-> np.ndarray`, `: System`) extensively.

### Phase 2: Core Architecture Bug Fixes
- [ ] **Fix `solve_ivp` External Inputs:** Ensure `core/analysis.py` dynamically polls external diagram inputs `sys.get_u_from_input_ports(t)` and passes them correctly to `f_fast` during Scipy integration.
- [ ] **Fix Stochastic Input Logging:** Make stochastic inputs deterministic by generating all random values at the start of the simulation based on a seed that is a block parameter, so that all source blocks are deterministic.


### Phase 3: API UX Improvements
- [ ] **Declarative Port Definitions:** Use Python class properties or dataclass-like fields to define ports on systems cleanly, removing verbose `add_input_port` calls inside `__init__`.
- [ ] **Reference-Based Connections:** Allow connections using actual port object references rather than strings (e.g., `diagram.connect(step.outputs.y, ctl.inputs.ref)`).
- [ ] **Operator Overloading:** Introduce syntactic sugar to make series or parallel connections intuitive (e.g., `connected_sys = step >> ctl >> plant`).

### Phase 4: Refactoring and Separation of Concerns
- [ ] **Refine Default Dependencies:** Provide a cleaner way to designate or infer MIMO algebraic loop dependencies to avoid artificial loop exceptions.

### Phase 5: New Features & Improvements
- [ ] **Automated Output Port Dependency Inference:** Extend `DiagramSystem` to automatically detect exact dependencies on external inputs when exposing a subsystem's output via `connect_new_output_port(..., dependencies="auto")`. This involves a topological trace through `self.connections` to prevent artificial algebraic loops when building diagrams inside diagrams.

### Phase 6: JAX Compilation & Future-Proofing (Duck-Typing & Optional f_jax)
- [ ] **Refactored Execution Plans:** Modify `DiagramSystem.f_fast` to avoid in-place numpy array mutations (no `global_signals = np.zeros(...)`). Use Python lists, comprehensions, or `jax.tree_util` to gather outputs and derivatives, then concatenate them. This allows the JAX compiler (`jax.jit`) to trace the execution completely natively.
- [ ] **Duck-Typing by Default:** Ensure standard library blocks under `blocks/` are written mathematically without in-place state mutations (`dx[0] = ...`), relying instead on array constructors (`np.array([...])`). Thanks to `__array_function__` dispatch, JAX will naturally trace native `numpy` operations.
- [ ] **Graceful `f_jax` Fallback:** Implement a dual-method architecture where a `System` can optionally override an `f_jax(self, x, u, t)` method for algorithms that cannot be traced natively by JAX (like complex `if`/`else` control flows). The diagram compiler will automatically use `f_jax` if defined when running under a JAX optimization context.
