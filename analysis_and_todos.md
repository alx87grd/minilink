# Minilink Architecture Analysis and TODOs

This document combines the architectural analysis of the `minilink` codebase and outlines the concrete steps (TODOs) needed to scale the project into a robust, maintainable foundation for block-diagram simulation in Python.

## 1. Analysis

### 1.1 Overview and Structure
`minilink` heavily leverages object-oriented design to represent systems, blocks, and connections. 
The core components consist of:
- **`core/`**: Houses the base `System` and port definitions (`framework.py`), the `DiagramSystem` which orchestrates block connections and execution (`diagram.py`), and the `Simulator` / trajectory handling (`analysis.py`).
- **`blocks/`**: Contains pre-built static and dynamic systems such as controllers, linear systems, and signal sources.
- **`graphical/`**: Utilities for plotting simulation results using `matplotlib` and visualizing block diagrams using `graphviz`.

### 1.2 Architecture and Design Flaws

*   **Flaw 1: `solve_ivp` Input Injection for `f_fast`**
    When simulating systems using the `scipy` solver wrapper `def f(t, x):` in `core/analysis.py`, the simulator attempts to use the optimized `sys.f_fast()` if available. However, it passes an empty array as the external input vector `u`. Diagrams that rely on external source inputs simulated with the `scipy` solver will receive zero inputs during integration.
*   **Flaw 2: Stochastic / Non-Deterministic Input Mismatch**
    The `scipy` ODE solver evaluates `f(t, x)` at many internal timestamps. However, the input trajectory `u_traj` returned to the user is constructed *after* integration finishes by re-evaluating over linear time bounds. Stochastic inputs (like `WhiteNoise`) will have recorded trajectories that don't match the actual integrated values.
*   **Flaw 3: Side-Effects in Pure Functions (Global Signals Buffer)**
    `compute_internal_signals()` in `core/analysis.py` is designed to analyze a resultant trajectory and rebuild internal block signals. To do so, it injects values directly into `diagram.global_signals`. Mutating this post-simulation breaks the encapsulation of the `DiagramSystem` and creates race conditions if evaluated in parallel.
*   **Flaw 4: Architectural Rigidity of Dependencies**
    By default, `dependencies="all"`, meaning every output of a block depends on all of its inputs. This creates artificial algebraic loops in complex MIMO systems where some outputs only depend on specific inputs.
*   **Flaw 5: Verbose API Design**
    Creating systems and diagrams relies on a verbose, string-heavy API (`diagram.connect("step", "y", "controller", "ref")`). It is error-prone, lacks IDE autocompletion, and is tedious for large systems.
*   **Flaw 6: Unseparated Compilation**
    The `DiagramSystem` handles both the topological sorting and the highly optimized execution plan building. This makes `diagram.py` large and overly complex.

---

## 2. TODOs

The following recommendations are structured in phases to systematically improve the API, architecture, and maintainability of the project.

### Phase 1: Modern Python Project Tooling
- [ ] **Implement `pyproject.toml`:** Use a modern build backend (like Poetry or Hatch) to manage dependencies (NumPy, SciPy, Matplotlib, Graphviz) and project metadata.
- [ ] **Continuous Integration (CI):** Set up a `.github/workflows/test.yml` to automatically run `pytest` on every push or pull request to the main branch.
- [ ] **Linting and Formatting:** Introduce `ruff` or a combination of `black`, `isort`, and `flake8` using `pre-commit` hooks.
- [ ] **Type Hinting:** Add `mypy` and use Python type hints (`-> np.ndarray`, `: System`) extensively.

### Phase 2: Core Architecture Bug Fixes
- [ ] **Fix `solve_ivp` External Inputs:** Ensure `core/analysis.py` dynamically polls external diagram inputs `sys.get_u_from_input_ports(t)` and passes them correctly to `f_fast` during Scipy integration.
- [ ] **Fix Stochastic Input Logging:** Update the simulator to log `u` values encountered during ODE integration, ensuring deterministic recording of stochastic sources.
- [ ] **Remove `global_signals` Side-Effects:** Refactor `compute_internal_signals()` so state buffer management is localized to the simulation runner, ensuring pure, side-effect-free evaluations.

### Phase 3: API UX Improvements
- [ ] **Declarative Port Definitions:** Use Python class properties or dataclass-like fields to define ports on systems cleanly, removing verbose `add_input_port` calls inside `__init__`.
- [ ] **Reference-Based Connections:** Allow connections using actual port object references rather than strings (e.g., `diagram.connect(step.outputs.y, ctl.inputs.ref)`).
- [ ] **Operator Overloading:** Introduce syntactic sugar to make series or parallel connections intuitive (e.g., `connected_sys = step >> ctl >> plant`).

### Phase 4: Refactoring and Separation of Concerns
- [ ] **Decouple Compilation from Diagram:** Move the compilation of `f_fast` execution plans into a separate module (e.g., `compiler.py` or an `ExecutionEngine` class). The `DiagramSystem` should purely represent the structural graph of blocks and connections.
- [ ] **Refine Default Dependencies:** Provide a cleaner way to designate or infer MIMO algebraic loop dependencies to avoid artificial loop exceptions.

### Phase 5: New Features & Improvements
- [ ] **Automated Output Port Dependency Inference:** Extend `DiagramSystem` to automatically detect exact dependencies on external inputs when exposing a subsystem's output via `connect_new_output_port(..., dependencies="auto")`. This involves a topological trace through `self.connections` to prevent artificial algebraic loops when building diagrams inside diagrams.
