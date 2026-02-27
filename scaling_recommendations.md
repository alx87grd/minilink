# Minilink Architectural Analysis & Recommendations

## 1. Overview and Goal
The goal is to scale the `minilink` repository into the foundation of a large project. The key priorities are:
- A clean, easy-to-use API for manipulating systems.
- Pure Python implementation.
- Easy to maintain, simple, and understandable.
- Flexible for complex, large-scale systems.

After an extensive analysis of the existing codebase (`core/framework.py`, `core/diagram.py`, `core/analysis.py`, and usage in tests and examples), I have identified several areas for improvement and propose the following recommendations.

---

## 2. API Design Improvements (Priority: Clean, Easy API)

### Current State
Creating systems and diagrams relies on a verbose, string-heavy API.
```python
diagram = DiagramSystem()
diagram.add_subsystem(step, "step")
diagram.add_subsystem(ctl, "controller")
diagram.add_subsystem(sys, "plant")
diagram.connect("step", "y", "controller", "ref")
diagram.connect("controller", "u", "plant", "u")
diagram.connect("plant", "y", "controller", "y")
```
*Issues:* It requires repetitive manual string declarations, which is error-prone and tedious for large systems. Building custom blocks also involves verbose `add_input_port` calls inside `__init__`.

### Proposed Changes
1. **Declarative Port Definitions:** Use Python class properties or dataclass-like fields to define ports on systems cleanly.
2. **Operator Overloading:** Introduce syntactic sugar to make series or parallel connections intuitive. For example:
   ```python
   # Connect step to controller, then to plant
   connected_sys = step >> ctl >> plant
   ```
3. **Reference-Based Connections:** Allow connections using actual port object references rather than strings.
   ```python
   diagram.connect(step.outputs.y, ctl.inputs.ref)
   ```
   This provides better IDE autocompletion and type checking, making it much more robust.

---

## 3. Architecture & Maintainability (Priority: Simple, Understandable)

### Current State
The `DiagramSystem` handles both the topological sorting (algebraic loop detection) and the highly optimized execution plan building (`build_execution_plan` and `f_fast`). This makes `diagram.py` large and overly complex.

### Proposed Changes
1. **Decouple Compilation from Diagram:** Move the compilation of `f_fast` execution plans into a separate module (e.g., `compiler.py` or an `ExecutionEngine` class). The `DiagramSystem` should purely represent the structural graph of blocks and connections.
2. **State Management Clean-up:** In `analysis.py`, the trajectory computation currently relies on mutating `diagram.global_signals` (a shared state). This prevents safe concurrent execution. State buffer management should be localized to the simulation runner, ensuring pure, side-effect-free evaluations.

---

## 4. Modern Python Project Tooling (Priority: Easy to Maintain, Scalable)

### Current State
The repository lacks standard packaging, dependency management, and code quality tools. It cannot be easily installed via `pip`.

### Proposed Changes
1. **Implement `pyproject.toml`:** Use a modern build backend (like Poetry or Hatch) to manage dependencies (NumPy, SciPy, Matplotlib, Graphviz) and project metadata.
2. **Continuous Integration (CI):** Set up a `.github/workflows/test.yml` to automatically run `pytest` on every push or pull request to the main branch.
3. **Linting and Formatting:** Introduce `ruff` or a combination of `black`, `isort`, and `flake8` using `pre-commit` hooks. This ensures styling consistency across the large project.
4. **Type Hinting:** Add `mypy` and use Python type hints (`-> np.ndarray`, `: System`) extensively. This drastically improves the "understandability" of the codebase for new contributors.

---

## 5. Verification Plan

Since you have asked for recommendations, no immediate structural changes to the codebase are executed yet.

If you approve this plan, we can begin implementing these recommendations in phases:
1. **Phase 1:** Add project tooling (`pyproject.toml`, Github Actions, `ruff`, `pytest` config).
2. **Phase 2:** Refactor the API to support reference-based connections and operator overloading for a cleaner user experience.
3. **Phase 3:** Extract the `f_fast` compilation into a separate Engine class to simplify `diagram.py` and remove shared-state side effects.

Please review these recommendations. If they align with your vision, let me know which phase you would like to proceed with first!
