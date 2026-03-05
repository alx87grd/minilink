# Minilink Coding Conventions

This document outlines the coding and naming conventions used in the `minilink` repository. Following these conventions ensures consistency and readability across the codebase.

## Variable Naming Conventions

The framework relies on distinct concepts for blocks, programmatic references, and graphical display. The following naming scheme must be adhered to:

### 1. `name` -> Type of System
*   **Purpose**: The string used to identify the overarching class or type of the system block. It is primarily for human readability and high-level categorization.
*   **Usage**: Assigned to the `self.name` attribute of a `System` or its subclasses.
*   **Examples**: `"System"`, `"StaticSystem"`, `"DynamicSystem"`, `"Diagram"`, `"Pendulum"`, `"Controller"`.

### 2. `id` / `xxx_id` -> Identifier of System or Port
*   **Purpose**: The programmatic string identifier used as keys in dictionaries to uniquely reference specific instances of blocks or ports within a diagram.
*   **Usage**: Used for connecting blocks, retrieving signals, and dictionary management. To avoid shadowing the Python built-in `id()`, prefix it with the context (e.g., `sys_id`, `port_id`).
*   **Examples**:
    *   System IDs: `"sys1"`, `"plant"`, `"controller"`
    *   Port/Signal IDs: `"u"`, `"y"`, `"v"`, `"w"`
    *   Variable names: `sys_id`, `port_id`, `source_sys_id`, `target_port_id`

### 3. `label` / `labels` -> String to Print
*   **Purpose**: The formatted string representation used for visual plotting, terminal output, and graphical diagrams. This corresponds to the individual components of a vector signal.
*   **Usage**: Kept as lists of strings mapping to the dimensions of a port or state vector.
*   **Examples**: `"theta"`, `"theta_dot"`, `"torque"`, `"sys1:theta"`.
*   **Note**: Always use the plural form `labels` for the variable name containing the list of strings (e.g., `sys.state.labels`, `input_labels`).

---
*Note: This file is a living document and should be updated as new architectural patterns or conventions are established in the repository.*
