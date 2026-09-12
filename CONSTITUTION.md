# The Minilink Constitution

The supreme architectural compass of Minilink. When evaluating whether a feature belongs,
is well-architected, or is worth keeping, this document is the governing authority.

---

## 1. Vision & Identity: The Unified Systems Lab

1. **One toolbox, one model.** A student, engineer, or researcher never changes frameworks to
   move between continuous modeling, simulation, linear/frequency analysis, classical and
   modern control, trajectory optimization, and reinforcement learning. Minilink is the
   integrating systems backbone: one clean Python interface for modeling, simulation,
   control, optimization, and learning. The same `System` does all of those jobs —
   there is no separate simulation stack and learning stack.
2. **Code that reads like a textbook.** Mathematical equations in the code mirror the textbook
   ($\dot{x} = A x + B u$, $dx = f(x, u, t)$). Verbs and nouns belong to the domain of
   mechanical and control engineering, not computer science abstractions.
3. **Simplicity is the ultimate sophistication.** Reading the source code is aimed to be
   like reading a textbook to learn. We work hard to keep core components as clean, beautiful,
   and simple as possible. Complexity belongs in orchestrators and backends, never in the
   core equations or the user's primary workflow.
4. **Product Boundaries (What Minilink is NOT):**
   - It is **not** a graphical block diagram editor or an implicit DAE engine (it is code-first, causal, and differentiable).
   - It is **not** a heavy multibody contact engine. Hard physics (contact, external
     multibody) is wrapped as optional leaves when needed; scale-out plants are not
     the product center.
   - It is **not** an optimization modeling language or solver framework (optimization is a verb that operates on Systems).

## 2. The Contract

### The Continuous Dynamical Core
At the center of Minilink is the continuous dynamic system. A model is defined by its dynamics:

    dx/dt = f(x, u, t; p)

Two companion functions provide observation and rendering geometries:

    y  = h(x, u, t; p)     outputs       (default: full state x)
    T  = tf(x, u, t; p)    body poses    (default: unplaced)

Poses ($T$) and visual shapes are declarative descriptions; dynamic systems contain no rendering engine or display loop.

Memoryless components (static gains, saturations, error sums) define static systems:

    y = h(u, t; p)

### The Five Contract Invariants
1. **Everything that can be viewed as a system is a System.** Plants, controllers, observers,
   filters, neural policies, and diagrams share the identical object model.
2. **Functional purity in equation paths.** Arrays in, arrays out. Equation paths ($f, h$)
   have no internal mutable state, no cached side effects, and no memory between calls.
   Purity is what enables arbitrary nesting, solver stepping, vectorized batches, and autodiff.
3. **Compositional closure.** Wiring continuous Systems together via operators (`@`, `>>`, `+`)
   yields a continuous `System`. The set of continuous Systems is algebraically closed under
   series, parallel, and feedback connections: a flow diagram compiles down to an identical
   pure continuous dynamical system ($\dot{X} = F(X, U, t; P)$). Any property or tool valid
   for a leaf is valid for a composite flow diagram. Hybrid and step composition
   (`Computer @ plant`, `HybridDiagram`) is a sibling algebra; it sits beside this set, not
   inside it.
4. **The continuous contract is primary.** `f` is an instantaneous rate, not a next-step map.
   Discrete and hybrid loops are sibling utilities that sit beside this core, never compromising it.
5. **JAX is an accelerator, not a prerequisite.** Every catalog plant and teaching tool runs on
   pure NumPy, SciPy, and Matplotlib. JAX brings compilation, gradients, and batching without
   locking the user out of standard Python.

## 3. The Two-Audience Guarantee

Minilink bridges two distinct worlds with a single API. This guarantee is a hard constraint,
not a ranked tradeoff against purity:

- **The Teaching Surface:** Undergraduate students run simulations, Bode plots,
  and state feedback in a single line on a laptop with zero boilerplate, using only standard scientific Python.
- **The Research Surface:** Graduate students and researchers scale the exact
  same syntax into trajectory optimization, neural policies, and GPU-batched reinforcement learning, with full
  access to JAX transformations, automatic differentiation, and tunable solver backends.

The student syntax never breaks when graduating to advanced research.

## 4. Governing Principles

1. **One set of tools for every System.** Tools operate on the `System` abstraction. There is no
   tool for plants that cannot also inspect a closed loop or a subsystem.
2. **Tools are verbs on a System.** Simulators, optimizers, and analysis tools take a `System`
   and return trajectories, matrices, or figures. Tools *never* define dynamics.
3. **Systems are descriptions; facades are shortcuts.** A `System` stores equations, ports,
   parameters, and nominal initial states. Facades (`compute_trajectory`, `linearize`) are
   convenient entry points that delegate directly to their standalone tool modules (`Simulator`, `analysis.linearize`).
4. **Explicit data flow.** No hidden registries, no invisible global switches, and no implicit backend state.
5. **Name by diagram role, not implementation technology.**

## 5. Conflict Precedence

When trade-offs or design tensions arise, higher-ranked principles strictly override lower ones:

1. **Equation-path purity** (No statefulness or memory in dynamics; functional purity is non-negotiable).
2. **Compositional closure** (The continuous set must close; a flow diagram behaves identically to a leaf).
3. **Continuous contract is primary** (Do not complicate `f`, `DiagramSystem`, flow `compile()`, or `Simulator` for discrete convenience).
4. **Domain readability** (Textbook mathematics over programmatic abstraction).
5. **Single-path implementation** (Code deduplication is desirable, but never at the expense of readability or purity).

## 6. Architectural Notes

- **Who writes $f$:** The abstraction level of authoring $f$ adapts to the physical scale (handwritten scalar
  equations, multi-link kinematics, or external engines). Once constructed, downstream tools cannot tell the difference.
- **Hard physics behind leaves:** Contact engines and external multibody sit behind optional
  leaves. Downstream tools still see a `System`.
- **Composition grammar is frozen early:** Series, parallel, and feedback (`>>`, `+`, `@`)
  are the composition language. Do not grow a second wiring dialect.
- **Build vs run:** Wiring, validation, and `compile()` freeze diagram structure. Runtime
  stepping must not mutate topology.
- **Backend-native math:** One equation path for NumPy and JAX. A backend-specific plant
  twin exists only when a single class would sacrifice textbook readability.
- **Backend honesty:** Minilink guarantees that its own wiring does not impede JAX tracing. If a user's
  equations use non-traceable Python libraries, standard NumPy tools continue to work seamlessly.
- **Infrastructure exemption:** Machine-facing infrastructure (rendering loops, compiler internals,
  I/O, test harnesses) is evaluated on robustness and maintainability, not on looking like textbook math.
