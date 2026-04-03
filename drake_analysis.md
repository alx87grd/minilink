# Drake Analysis — Lessons for Minilink

> Cross-codebase analysis of [Drake](https://github.com/RobotLocomotion/drake) (RobotLocomotion) patterns,
> features, and design strategies. Drake is widely considered the technical benchmark for 
> model-based robotics and control frameworks.

---

## 1. Architecture Comparison at a Glance

| Dimension | minilink | Drake (C++/pydrake) | Gap / Opportunity |
|---|---|---|---|
| **Core Philosophy** | Mathematical clarity, flat arrays | Systems & Contexts, rigorous dependency tracking | Separation of "Methods" vs "Data" is absolute in Drake |
| **State Management** | Passed as `x, u, t` to functions | `Context` object (one-to-one with System) | Drake Context encapsulates state, parameters, time, and cache |
| **Wiring** | String-based `connect(name1, p1, name2, p2)` | Object-based `builder.Connect(out_port, in_port)` | Drake's `DiagramBuilder` is the source of the Collimator pattern |
| **Evaluation** | Compiled `ExecutionPlan` (topological) | Lazy Evaluation via `Eval()` calls | Drake's cache system is extremely granular |
| **AD Support** | JAX-based (`JaxEvaluator`) | `AutoDiffXd` (Forward-mode via templating) | Drake supports both Autodiff and Symbolic types natively |
| **Multibody** | Simple custom blocks | `MultibodyPlant` (massively optimized system) | Move toward specialized plants for physics |

---

## 2. The "System vs Context" Pattern

This is Drake's most influential contribution to systems engineering.

### The Problem
If a `System` stores its own state (e.g., `self.x = 0`), you cannot:
1. Simulate two instances of the same model in parallel easily.
2. Checkpoint and restore simulation state (serialization).
3. Compute gradients `df/dx` without polluting the nominal state.

### Drake's Solution
The `System` is a **stateless** set of equations. The `Context` is the **stateful** instance of those equations.

```python
# Drake concept (pydrake)
system = MySystem()
context = system.CreateDefaultContext()
context.SetContinuousState([1.0, 0.0]) # Data lives here
xdot = system.EvalTimeDerivatives(context) # Equations live there
```

> [!IMPORTANT]
> **Recommendation for minilink**: You have already achieved "headless" evaluation with `JaxEvaluator` and `NumpyEvaluator`, where state is passed in. To reach Drake-level maturity, consider bundling `x, u, t, params` into a single `Context` object to avoid "argument soup" in complex blocks.

---

## 3. Dependency Tracking & Caching

Drake uses a sophisticated "Ticket" system to avoid redundant work.

- **Tickets**: Every source (time, state, input) and every result (output, derivative, kinetic energy) has a `DependencyTicket`.
- **Trackers**: In the `Context`, `DependencyTracker` objects form a graph.
- **Invalidation**: If you change `context.time`, all trackers for computations that depend on `time` are marked "out-of-date".
- **Lazy Eval**: When you call `system.EvalOutput(context)`, it only recomputes if the output tracker is "out-of-date".

> [!TIP]
> **Lesson**: If minilink adds an expensive block (e.g., a neural network or a complex rigid body model), adding a simple `last_t` cache inside the evaluator or a proper `CacheEntry` in a Context would be game-changing for performance.

---

## 4. Scalar Type Polymorphism

Drake is famous for `System<double>`, `System<AutoDiffXd>`, and `System<Expression>`.

1. **AutoDiffXd**: A custom type that carries `(value, gradient_vector)`. Drake's `LeafSystem` methods are written to be "scalar agnostic".
2. **Symbolic**: A type that builds an expression tree. This allows Drake to linearize models *analytically* or extract polynomial representations for SOS (Sum-of-Squares) optimization.

> [!TIP]
> **Recommendation**: minilink's `JaxEvaluator` achieves something similar to `AutoDiffXd` via JAX's internal tracer. However, if you want **analytical** linearization (like SymPy), look at how Drake uses a `Symbolic` scalar type to trace the `f(x, u, t)` function.

---

## 5. DiagramBuilder vs Diagram

Pycollimator's `DiagramBuilder` is a direct clone of Drake's.

- **Builder**: A temporary "factory" used to register systems and connections.
- **Diagram**: The resulting immutable "system of systems".

**Key Drake feature: Port Exporting**
```cpp
builder.ExportInput(subsystem.get_input_port(0));
builder.ExportOutput(subsystem.get_output_port(1));
```
This allows a hierarchy of Diagrams where a sub-assembly looks like a single `LeafSystem` to the parent.

> [!IMPORTANT]
> **Recommendation**: For minilink, ensure `DiagramSystem` can be added *inside* another `DiagramSystem`. This requires the inner diagram to have its own ports. Drake's "Export" pattern is the cleanest way to handle this.

---

## 6. Actionable Insights for Minilink

### Near-Term
- [ ] **Port Invalidation Concept**: In `DiagramSystem`, if an input isn't connected, should it have a `default_value`? Drake enforces this.
- [ ] **One-to-One Context**: When initializing a simulation, create a `Context` object (or dict) that maps 1:1 to the systems, rather than flat global arrays. This makes multi-agent simulation much easier.

### Mid-Term
- [ ] **Scalar Dispatch**: Allow blocks to define `f(x, u, t)` using standard math ops, and let the Evaluator pass in either NumPy arrays, JAX tracers, or even SymPy symbols.
- [ ] **Caching expensive ports**: Add a `cache_results=True` flag to `OutputPort` to avoid re-running `compute()` if time hasn't advanced.

### Long-Term
- [ ] **Multibody Core**: Instead of users writing Pendulum ODEs by hand, introduce a "Plant" system that takes a description (URDF/SDFormat) and handles the `M(q)vdot + C(q, v)v = tau` equations.

---

## 7. Summary

Drake is **"Rigorous first"**. Minilink is **"Math first"**. 

Drake's design is heavily influenced by the constraints of C++ (static typing, performance) and the needs of safety-critical robotics (determinism). Minilink can stay lightweight by adopting Drake's **conceptual** separation (System vs Context) without the **implementation** heavy-lifting (C++ templates, Bazel, etc.).

**Core Takeaway**: 
> "The System is the Map; the Context is the Territory."
> — Drake Philosophy
