# Strategic Editorial Review: *Minilink — The Differentiable Systems Lab*

**Date:** 2026-09-12  
**Author:** AI Lead Architect & Senior Pair Programmer  
**Audience:** Prof. Alexandre Girard (Maintainer & Chief Architect)  
**Context:** Pre-v0.1 Consolidation · Post-Pyro Evolution · GRO860 / GRO501 Alignment  
**Reference Documents:** `CONSTITUTION.md` · `RULES.md` · `DESIGN.md` · [alexandregirard.com](https://www.alexandregirard.com)

---

## 1. The Landscape: Where Minilink Stands in the World

To understand what *Minilink* must be, we have to look frankly at what robotics and control engineers actually deal with in 2026.

When an engineer, student, or researcher wants to model a dynamical system and take it from paper to simulation, classical control, trajectory optimization, and reinforcement learning, **the existing landscape forces them to stitch together 4 to 6 incompatible tools**:

```
      [ Modeling & Sim ] ───► [ Classical Control ] ───► [ Trajectory Opt / MPC ] ───► [ Learning / RL ]
             │                         │                            │                          │
 Simulink / MuJoCo / Drake       python-control                  CasADi / acados             Brax / SB3
             │                         │                            │                          │
      (Heavy C++ / GUI)           (LTI only)                 (Symbolic graphs)           (Disjoint physics)
```

### The Anatomy of the Existing Ecosystem

| Framework | What they own | Why they fail the student & modern researcher |
| :--- | :--- | :--- |
| **Simulink** (MathWorks) | Industrial graphical block diagrams, Stateflow, DAE solvers. | **Proprietary, closed, and severed from modern AI.** Binary `.slx` files are a git nightmare. Opaque DAE solvers suffer from algebraic loops. Impossible to take exact analytical JAX gradients ($\partial f / \partial x$) through a diagram to train a neural policy in 30 seconds. |
| **python-control** (Murray et al.) | Clean Python equivalents of MATLAB's CST (`tf`, `ss`, `bode`). | **Stops at linear systems.** As soon as you leave LTI land for nonlinear dynamics, trajectory optimization, or reinforcement learning, it offers no coherent path. Its nonlinear simulation is a thin, slow wrapper over SciPy. |
| **Drake** (Tedrake / MIT) | World-class C++ multibody mechanics, hydroelastic contact, MathProg. | **Enormous cognitive barrier.** `pydrake` requires managing complex C++ object lifecycles, allocating `Context` objects, and declaring abstract port types. Custom dynamics cannot be easily JIT-compiled with JAX or vectorized across GPU batches. It is a research battleship, not an accessible student lab. |
| **MuJoCo / MJX** (DeepMind) | Blisteringly fast rigid-body contact physics; GPU batching. | **A physics engine, not a control framework.** It has no concept of block diagram algebra ($C @ P$), no transfer functions, no Bode plots, no pole placement, no observer wiring. You cannot express a cascaded mechatronic control loop naturally. |
| **CasADi** (Andersson, Diehl) | Symbolic AD graphs (`SX`/`MX`) $\to$ NLP solvers (Ipopt, SNOPT). | **Optimization-only DSL.** It is an engine for defining NLP constraints, not a simulation or control diagram framework. Students spend their time debugging graph construction errors rather than studying system response. |
| **Brax / Isaac** | Highly batched RL physics environments. | **Black-box RL silos.** Severed from control theory (no Lyapunov stability, no frequency response, no classical loop shaping). |
| **Pyro** (Girard / Sherbrooke) | The beloved predecessor. Pure NumPy, textbook equations, intuitive. | **The proof of concept.** Showed that students adore writing $\dot{x} = f(x,u,t)$ directly. But it lacked block diagram algebra, JAX compilation, automatic differentiation, vectorized GPU batches, and native neural policy integration. |

### Minilink's "Blue Ocean"

Minilink does not try to out-MuJoCo MuJoCo or out-CasADi CasADi.

**Minilink occupies the exact missing center:**
> **The Differentiable Systems Lab.**  
> *Write the continuous physical equations once in standard Python: $\dot{x} = f(x, u, t; p)$. Wire systems into diagrams using algebra (`@`, `>>`, `+`). The exact same model simulates in pure NumPy on an undergraduate laptop, plots Bode margins, optimizes trajectories, and JIT-compiles under JAX for GPU-batched reinforcement learning without changing a single line of physics.*

This vision matches Alexandre Girard’s pedagogical DNA: **mechanical engineering intuition first, mathematical rigor in textbook notation, and zero software friction.**

---

## 2. Editorial Review of `CONSTITUTION.md`

The newly streamlined `CONSTITUTION.md` (114 lines) is lean, authoritative, and hits the right frequency. Removing competitor names and course codes elevates it from a local course syllabus into a timeless foundational document.

### What is Exceptional
1. **The Priority Inversion (5.1 > 5.2):** Setting **Equation-path purity** as Rule #1 above Compositional Closure is essential. If $f$ is not pure, JAX tracing dies, batching corrupts memory, and solver steps become stateful. Purity is the physical law of Minilink.
2. **"Simplicity is the ultimate sophistication":** Elevating code beauty and readability to a constitutional mandate (§1.3) protects the repository from creeping enterprise software architecture (no metaclass factories, no abstract context managers).
3. **The Two-Audience Guarantee (§3):** Articulating that the undergraduate syntax *never breaks* when scaling to research establishes the contract that keeps the library unified.

### Approved Constitutional Refinements
- **Formally Anchor the Diagram Algebra (`@`, `>>`, `+`):**  
  The set of Systems is closed under series composition (`>>`), parallel addition (`+`), and feedback closure (`@`). A composite diagram is not a container that schedules leaves; it compiles down to an identical pure dynamical system: $\dot{X} = F(X, U, t; P)$.
- **Preserve General Behavioral Definitions:**  
  Systems and ports remain a clean, general behavioral abstraction. A `System` defines input/output/state mappings without artificially restricting what state or outputs must represent.
- **Pure Core Signature:**  
  The core signature $\dot{x} = f(x, u, t; p)$ is kept pure and uncompromised. Tools that operate on autonomous systems handle time according to their domain without forcing artificial state-lifting.

---

## 3. Editorial Review of `RULES.md`

The newly created `RULES.md` is structured around the **7-Question Review Ladder**. It bridges the gap between high-level philosophy and concrete lines of code for both humans and AI agents.

### What is Exceptional
1. **The Bare Signatures Rule (§5.1):** Prohibiting type hints in $f, h, tf$ is brilliant for mechanical engineers. Writing `def f(x: NDArray[Shape["N"], Float], u: NDArray[Shape["M"], Float]) -> ...` destroys textbook readability. Shapes belong in the docstring; equations belong in the code.
2. **The `xp` Idiom (§5.2) & No `self.` in Equations (§5.3):** Unpacking `params` first and binding `xp = array_module(x)` ensures that continuous equations read like pure mathematics while remaining dual-compatible with NumPy and JAX.
3. **No Pseudo-Private Underscores (§5.8):** Getting rid of `_compute_derivatives` on System classes removes unnecessary OOP noise. `# Public API` and `# Internal machinery` sections communicate intent cleanly.
4. **No Test Harness in Demos (§6.2):** Demos remain pure teaching scripts. CI runners adapt via environment variables (`MPLBACKEND=Agg`) from the outside.

### Approved Rule Refinements
1. **The Feedback and Connection Operators (§4.8):**  
   Operators provide rapid synthesis: `>>` for cascade, `+` for parallel sum, and `@` for feedback closure. Acknowledge that while `@` provides rapid closure for standard negative feedback loops (`controller @ plant`), multiple feedback topologies exist (sensor dynamics, observer feedback, MIMO loops) and can be composed via operator chaining or `DiagramSystem.connect`.
2. **Standard SI Units & Radian Convention (§5.14):**  
   All angles in radians, angular velocities in rad/s, standard SI units (kg, m, s, N, N·m). State variables and angles keep unbounded continuous representations by default unless explicit manifold wrapping is declared.
3. **Zero-Friction Execution (§6.7):**  
   Every teaching notebook and demo must execute on Google Colab (CPU runtime) and on a standard student laptop in under 60 seconds without requiring external C++ build chains, proprietary licenses, or GPU hardware.

---

## 4. The Governance Architecture: 4 Clean Documents

```
                       ┌────────────────────────────────────────┐
                       │            CONSTITUTION.md             │
                       │           (The Supreme Compass)        │
                       │  - Vision, Product Boundaries          │
                       │  - The Sacred Contract: f, h, tf       │
                       │  - Priority: Purity > Composition      │
                       └───────────────────┬────────────────────┘
                                           │ governs
                                           ▼
                       ┌────────────────────────────────────────┐
                       │                RULES.md                │
                       │          (The Universal Ladder)        │
                       │  - 7 Questions: Scope to Consolidation │
                       │  - Textbook style, xp, bare signatures │
                       │  - For humans AND agents               │
                       └──────────┬──────────────────┬──────────┘
                                  │                  │
               supplements for AI │                  │ supplements for dev
                                  ▼                  ▼
       ┌──────────────────────────────┐  ┌──────────────────────────────┐
       │          AGENTS.md           │  │          DESIGN.md           │
       │    (Agent Workflow Only)     │  │    (Technical Contracts)     │
       │  - Preserve user edits       │  │  - DiagramSystem mechanics   │
       │  - Delegation lanes          │  │  - JAX compile tiers         │
       │  - Local CI gate & no polling│  │  - Section 8: Call Chains    │
       └──────────────────────────────┘  └──────────────────────────────┘
```

This distribution of responsibilities ensures that:
- **`CONSTITUTION.md`** answers: *What is the supreme law?*
- **`RULES.md`** answers: *What are the universal code standards and review ladder?*
- **`AGENTS.md`** answers: *How does an AI agent interact with the user and codebase?*
- **`DESIGN.md`** answers: *What are the technical object specifications and call chains?*
