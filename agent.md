# Minilink AI Agent Instructions

This document defines the co-programming preferences and architectural philosophy for AI agents collaborating on the `minilink` library. It ensures consistency, mathematical readability, and operational transparency across the development lifecycle.

## 1. Core Directives

- **Mathematical Readability First**: The primary goal is for the source code to read as close as possible to handwritten mathematical operations (e.g., `dx = A@x + B@u`). This is an educational tool; students should recognize the dynamics and control laws directly in the code.
- **Minimalist UI**: Prioritize a clean, minimal, and beginner-friendly interface for the main workflow (system creation, diagram wiring, simulation, and animation).
- **"MVP Prototyping Mode"**: Rapid prototypes are encouraged but must be clearly marked with `TODO: User Architectural Review` to ensure no unverified logic enters the core.
- **Incremental Refactoring**: Avoid large, sprawling changes. Always ask before deleting or renaming files.
- **Automated Documentation**: Keep `DESIGN.md` and `ROADMAP.md` in sync with progress. These files are the agent's primary source of project "Truth."

---

## 2. Coding Standards & Style

- **Python Version**: **3.10+** (LTS stable) for the moment (not yet fully chosen). Use modern syntax like `|` for unions and structural pattern matching. 
- **Type Hinting**: **Uniform & Mandatory**. All functions and methods must have clear type hints.
- **Docstrings**: **NumPy Style**. Required for all public classes and methods.
- **Naming Patterns (The "Math Rule")**:
    - The top priority is that the code reads like math equations. For example, if you have a state-space system, the code should look like `dx = A@x + B@u`, not 'state_derivative = np.dot(linear_dynamics_matrix, state_vector) + np.dot(input_matrix, input_vector)' the goal is for the code to be readable as a math equation, the same equations in a textbook, for student to understand the code and the math at the same time.
    - **Matrices**: Use Uppercase single letters (`A`, `B`, `H`, `M`, `K`).
    - **Vectors**: Use lowercase single letters (`x`, `u`, `y`, `q`, `v`, `dq`).
    - **dimensions**: Use 'n', 'm', 'p' for dimension, for instance matrix A has dimension (n, n), vector x has dimension (n, 1), vector u has dimension (m, 1), vector y has dimension (p, 1), matrix B has dimension (n, m)...
    - **Math context**: All in all for math context, use name convention as close as possible as standard notation in textbooks.
    - **Non-Math Context**: Follow standard **PEP8** (snake_case for methods, CamelCase for classes).

---

## 3. Architectural Philosophy

- **Hybrid Model**: Use **Inheritance** for defining core system types (`System`, `StaticSystem`, `DynamicSystem`). Use **Composition** for diagram assembly and adding optional behaviors (sensors, noise).
- **Readability Over Performance**: Prioritize pure readability in the core library. Optimization shifts (like the `compile` package) should remain isolated so core equations stay clean.
- **JAX Policy**: Support JAX when possible WITHOUT too much complication. If JAX-traceability requires complex code, diverge into specific JAX-optimized tools/subclasses to keep the primary path clean.

---

## 4. 3-Level Testing Strategy

Every feature must pass through three levels of verification:
1.  **Automated Unit Test**: Formal `pytest` suite for regression and the AI agent's internal check.
2.  **User Manual Test**: A minimalist, readable script (usually in `tests/manual/`) for the developer to check hands-on.
3.  **Demo Script**: A high-level script (in `examples/`) demonstrating the feature to end-users (for major features).

---

## 5. Development Lifecycle (TRL)

All features must progress through the following **Task Readiness Levels**:

| Level | Name | Description |
| :--- | :--- | :--- |
| **TRL 1** | Agent MVP | Initial code implemented and functionally working. |
| **TRL 2** | User-check MVP | User performs a high-level functional review. |
| **TRL 3** | Architecture Validated | High-level architectural choice and logic are approved. |
| **TRL 4** | Integration Proposed | Agent proposes the final code integration/refactor. |
| **TRL 5** | Integration Validated | User approves the final integration into the main codebase. |
| **TRL 6** | Automated Tests Pass | Final `pytest` suite is created and passing. |
| **TRL 7** | Details Validated | Naming conventions and implementation details are approved. |
| **TRL 8** | Demo Released | High-level demo script is created and validated. |
| **TRL 9** | MISSION COMPLETE | All 3 levels of tests pass and the feature is user-approved. |

---

## 6. Collaboration & Workflow

- **Just Do It**: 
    - Fix typos and grammatical errors.
    - Add missing type hints or NumPy-style docstrings.
    - Cosmetic PEP8 adjustments (outside of math equations).
- **Always Ask**: 
    - Deleting or renaming files.
    - Architectural changes or core logic refactors.
    - Introducing new heavy dependencies.

---

## 7. Local Development Environment

- **Conda Environment**: Use `dev-h26` for this repository.
  ```bash
  conda activate dev-h26
  ```
- All commands (pytest, ruff, python scripts, etc.) should be run inside this environment.
