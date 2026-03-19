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
- [ ] **Eliminate Global Signal Buffers for Reconstruction:** Replace the mutable `DiagramSystem.global_signals` buffer used during internal signal reconstruction with a more functional, encapsulated mechanism that is safe for parallel evaluation and clearer in its data flow.


### Phase 5: New Features & Improvements
- [ ] **Automated Output Port Dependency Inference:** Extend `DiagramSystem` to automatically detect exact dependencies on external inputs when exposing a subsystem's output via `connect_new_output_port(..., dependencies="auto")`. This involves a topological trace through `self.connections` to prevent artificial algebraic loops when building diagrams inside diagrams.

### Phase 5.5: Visualization Robustness
- [ ] **Harden Matplotlib Backend Selection:** Make the plotting backend selection more robust across headless, Linux, and cloud environments (e.g., prefer non-interactive backends by default with clearer configuration hooks, and degrade gracefully when interactive backends are unavailable).

### Phase 6: JAX Compilation & Future-Proofing
- [ ] **Dual Backend Injection (`xp`):** Refactor `System` and basic blocks to accept an array namespace parameter (`xp`), rather than strictly importing `numpy`. This enables passing `jax.numpy` into block mechanics when tracing without breaking standard `numpy` execution for casual users.
- [ ] **Functional Execution Plans:** Refactor the `DiagramSystem.f_fast` compilation loop to support a `backend="jax"` flag. When active, it uses functional array updates (`global_signals.at[i].set(...)`) rather than standard mutable slicing, enabling XLA compilation and GPU acceleration without breaking out-of-the-box Python behavior.
- [ ] **Functional JAX Blocks:** Ensure standard library blocks under `blocks/` utilize structural array builds (`xp.array([val1, val2])`) instead of `np.zeros()` and in-place assignment. Use `xp.where()` for control flow instead of Python `if` statements over signals.
