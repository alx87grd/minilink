# Neural Blocks Collection — Plan and Options

Working notes for a long-term collection of neural-network blocks in minilink.
Status: **proposal for architectural review** (`TODO: User Architectural Review`).

Related maturity claims: [ROADMAP.md](../../ROADMAP.md) §5 (`blocks/neural.py` MLP
done at TRL 2; `control/neural.py`, `identification/fitting.py`, and
`interfaces/flax.py` planned).

## Current state (June 2026)

The latest NN work landed in commit `fd0d17d` with two artifacts:

**Block** — `minilink/blocks/neural.py`: a single `NeuralNetwork` `StaticSystem`,
one hidden layer, `y = W2 tanh(W1 u + b1) + b2`, weights in `params`, and
`xp = array_module(u)` for JAX traceability.

**Demo** — `examples/scripts/control/demo_neural_controller_jax.py`: wires
`NeuralNetwork` into a closed-loop diagram (`Mux` → `nn` → `Integrator`), compiles
with `diagram.compile(backend="jax")`, and trains weights via batched RK4 rollouts
and manual SGD through `f_p` and `jax.grad`. There is no dedicated
`examples/scripts/blocks/demo_neural*.py` yet.

**Tests** — shape/equation correctness, params override, JAX `jit` traceability
(`tests/unittest/test_neural_blocks.py`).

**Planned but empty** — `control/neural.py` (policy-oriented blocks),
`interfaces/flax.py` / `torch.py` (external model wrappers),
`identification/fitting.py` (unified param fitting for physical params and NN
weights).

The demo establishes the core contract: **NN blocks are ordinary diagram blocks;
training lives outside; weights are `params` entries differentiated through the
compiled diagram.**

## Design anchors

Any long-term NN collection should respect constraints from [AGENTS.md](../../AGENTS.md) and
`DESIGN.md`:

| Principle | Implication for NN blocks |
| --- | --- |
| Math readability first | Layer equations read like `a = tanh(W @ u + b)`, not framework ceremony |
| Shelve by diagram role | Architecture-neutral layers → `blocks/`; closed-loop policy shapes → `control/` |
| `xp` idiom, lazy JAX | Core blocks stay NumPy/JAX-hybrid without requiring Flax |
| Training outside blocks | Blocks expose `compute` + `params`; `identification/` owns fitting loops |
| Continuous-time only | No RNN/LSTM/GRU blocks; NNs are memoryless `StaticSystem` maps |
| Params = first-class | `params["nn"]` partial override already works in diagram evaluators |

## Organization options

Three viable organization strategies, plus a hybrid recommended as the end state.

### Option A — Composable layer blocks in diagrams

Split the network into atomic `StaticSystem` blocks wired like any other signal
path:

```
u → Dense(2→8) → Tanh → Dense(8→1) → y
```

| Piece | Location | Role |
| --- | --- | --- |
| `Affine` / `Dense` | `blocks/neural/layers.py` | `y = W @ u + b`, params `W`, `b` |
| `Tanh`, `ReLU`, `Sigmoid` | `blocks/neural/activations.py` | elementwise, no params |
| `Mux` / `Demux` | existing `blocks/routing.py` | input stacking, skip connections |

**Advantages**

- Topology is visible in `plot_diagram()` — ideal for controls/mechanical-engineering readers.
- Mix NN layers with `Saturation`, `Gain`, `Sum`, filters in one architecture.
- Each layer is independently replaceable and testable.
- Matches how `demo_signal_blocks.py` fans out parallel paths.

**Disadvantages**

- Deep nets create many subsystems → larger `ExecutionPlan`, noisier `params` nesting
  (`{"dense0": {...}, "dense1": {...}}`).
- Skip connections and residual wiring get verbose without helper factories.
- Training code must manage a nested params tree (already supported, but more bookkeeping).
- Compile overhead scales with subsystem count.

**Best for:** shallow controllers (2–3 layers), architectures where non-NN blocks
sit between layers, teaching/diagram-first workflows.

### Option B — Monolithic `MLP` block (extend current `NeuralNetwork`)

One `StaticSystem` owns the full stack:

```python
MLP(input_dim=2, hidden_dims=(32, 16), output_dim=1, activation="tanh")
```

Params: `W0`, `b0`, `W1`, `b1`, … Forward loop over layers inside `compute`.

**Advantages**

- Minimal diagram clutter — one box labeled "controller".
- Simple params dict for training (`params={"nn": {...}}`).
- Fast compile, easy `jax.jit` / `vmap`.
- Natural evolution of what exists today.

**Disadvantages**

- Architecture invisible in the diagram.
- Harder to insert a `Saturation` between specific layers without splitting the block.
- Every new architecture variant (residual head, dual outputs) tends to grow one class.

**Best for:** standard MLP policies, identification demos, anything trained
end-to-end through rollouts.

### Option C — Factory/orchestrator layer (no new block type)

Keep atomic layers (Option A) but provide builders in `blocks/neural/`:

```python
controller = mlp_block(input_dim=2, hidden_dims=(8, 8), output_dim=1)
```

Or a `SequentialNetwork` wrapper: one `StaticSystem` whose `compute` internally
calls layer functions — diagram shows one box, implementation is compositional.

**Advantages**

- User picks diagram-visible vs. collapsed representation.
- Factories encode naming conventions for params (`W0`, `W1`, …).
- Can generate either a flat `StaticSystem` or a pre-wired `DiagramSystem`.

**Disadvantages**

- Two ways to build the same thing — needs clear docs on when to use which.
- `SequentialNetwork` is a slight abstraction over Option B.

**Best for:** library maintainability; recommended as the **integration point**
between A and B.

### Option D — Policy blocks in `control/neural.py` (orthogonal to A/B/C)

Separate **architecture** (`blocks/neural/`) from **policy shape**
(`control/neural.py`):

```python
StateFeedbackMLP(n_state=4, n_control=2, hidden_dims=(32,))
ReferenceTrackingMLP(n_state=2, n_ref=1, n_control=1, ...)  # wraps Mux internally
```

**Advantages**

- Matches `FilteredPIDController` pattern — opinionated port naming (`x`, `r` → `u`).
- Demo scripts become shorter; less wiring boilerplate.
- Clear split: `blocks/` = math primitives, `control/` = closed-loop semantics.

**Disadvantages**

- Duplication risk if policy blocks just re-wrap `MLP`.
- Policy variants multiply (`StateFeedbackMLP`, `OutputFeedbackMLP`, …).

**Best for:** the 80% case in control demos; keep `blocks/neural/` generic
underneath.

## Flax vs bare JAX

### Option 1 — Bare JAX only (extend current path)

Core `blocks/neural/` written with `xp = array_module(x)`, params as flat NumPy/JAX
arrays in `self.params`.

| Pros | Cons |
| --- | --- |
| Zero new dependencies | No `nnx`/`linen` module patterns, no optax bundled |
| Equations stay textbook-readable | Residual blocks, batch norm, attention are manual |
| Already proven JAX-traceable through `diagram.compile()` | Importing pretrained Flax checkpoints not supported |
| Aligns with [AGENTS.md](../../AGENTS.md) rule 6 (`xp` idiom) | Users who live in Flax ecosystem write their own glue |
| `identification/` can use `jax.grad` uniformly | Parameter init/heuristics are hand-rolled |

### Option 2 — Flax/Linen as core implementation

Rewrite layers as Flax `nn.Module` subclasses, bridge to `StaticSystem`.

| Pros | Cons |
| --- | --- |
| Rich layer zoo, init schemes, Optax integration | Heavy optional dep; fights "textbook math in `compute`" |
| Ecosystem interop (checkpoint loading) | Flax params pytree ≠ minilink `params` dict — adapter layer needed |
| `jax.jit` via `module.apply` is well-trodden | Two param systems: `self.params` vs Flax `variables` |
| Natural home for complex architectures | Controls audience must learn Flax to read block internals |

### Option 3 — Hybrid (recommended)

| Layer | Technology |
| --- | --- |
| `blocks/neural/` | Bare JAX / `xp` — `Dense`, activations, `MLP` |
| `identification/fitting.py` | Bare `jax.grad` + optional Optax (lazy import) |
| `interfaces/flax.py` | Adapter: `FlaxModule → StaticSystem` for external models |
| `interfaces/torch.py` | Same pattern for PyTorch (export/ONNX or manual weight copy) |

| Pros | Cons |
| --- | --- |
| Core stays lightweight and readable | Adapter maintenance when Flax API shifts |
| Advanced users import Flax models without polluting core | Two paths to "a neural block" — needs docs |
| Matches ROADMAP §5 placement (`interfaces/` for ecosystem bridges) | Flax adapter must map pytree → nested `params` dict |
| No forced Flax install for basic MLP control demos | |

**Recommendation:** Option 3. The current `NeuralNetwork` is the right seed — extend
it in bare JAX. Reserve Flax for `interfaces/flax.py` when the use case is "I
already trained this in Flax and want it as a minilink plant/controller."

## Proposed package layout

```
minilink/blocks/neural/
    __init__.py          # namespace marker only (no barrel re-exports)
    layers.py            # Dense / Affine
    activations.py       # Tanh, ReLU, LeakyReLU, Sigmoid, Softplus
    mlp.py               # MLP, SequentialNetwork (monolithic multi-layer)
    init.py              # glorot, he, etc. (optional, small)

minilink/blocks/neural.py  → migrate to blocks/neural/mlp.py, keep re-export or
                           delete with call-site fix

minilink/control/neural.py
    policies.py          # StateFeedbackMLP, ReferenceTrackingMLP factories

minilink/identification/
    fitting.py           # fit_params(system, trajectory, loss="equation_error"|"rollout")
    # NN weights and physical params share the same API

minilink/interfaces/
    flax.py              # from_flax_module(module, example_inputs) -> StaticSystem
    torch.py             # from_torch_module(...) -> StaticSystem
```

### Params naming convention

Keep flat and predictable for monolithic MLP:

```python
# MLP with hidden_dims=(h0, h1):
params = {"W0": ..., "b0": ..., "W1": ..., "b1": ..., "W2": ..., "b2": ...}
```

For diagram-composed layers, use subsystem id as the nesting key (already supported):

```python
params = {"dense0": {"W": ..., "b": ...}, "dense1": {"W": ..., "b": ...}}
```

## Phased roadmap (TRL-oriented)

| Phase | Deliverable | TRL | Notes |
| --- | --- | --- | --- |
| **P0** (done) | `NeuralNetwork` 1-hidden-layer + JAX controller demo + tests | 2 | User review of API shape |
| **P1** | `MLP(hidden_dims=...)` multi-layer, rename/alias `NeuralNetwork` | 3 | Architecture-validated; same `params` pattern |
| **P1** | `Dense`, `Tanh`, `ReLU` atomic blocks | 3 | Enable diagram-composed shallow nets |
| **P1** | `mlp_diagram(...)` factory returning wired `DiagramSystem` | 4 | Best of A + B without forcing a choice |
| **P2** | `control/neural.py` policy factories | 4 | Shorter demos; standard port names |
| **P2** | `identification/fitting.py` with rollout loss | 5 | Extract training loop from demo; one verb for all params |
| **P3** | `interfaces/flax.py` adapter | 4–5 | Optional `[flax]` extra in `pyproject.toml` |
| **P3** | Skip-connection / residual `MLP` variant | 5 | Still continuous-time, still `StaticSystem` |
| **Out of scope** | RNN, LSTM, attention, batch norm in online sim | — | Per DESIGN.md continuous-time decision |

## Architecture decision matrix

| Question | Diagram-composed (A) | Monolithic MLP (B) | Hybrid (A+B+C) |
| --- | --- | --- | --- |
| Visible in `plot_diagram()`? | Yes | No | User choice |
| Training simplicity | Nested params | Flat params | Flat via factory |
| Mix with `Saturation`/filters | Natural | Awkward | Natural |
| Deep (10+ layer) nets | Heavy compile | Fine | Use B |
| Textbook readability | High per block | High in one `compute` | High |
| Matches current demo | Partial | Direct extension | Direct extension |

**Suggested default for minilink's audience:** Hybrid — ship **monolithic `MLP`** as
the 90% path (extend current block), add **atomic layers + `mlp_diagram` factory**
for teaching and hybrid architectures, add **`control/neural` policy wrappers** for
closed-loop ergonomics.

## Summary recommendation

| Layer | Choice |
| --- | --- |
| Core math | Bare JAX + `xp` idiom |
| Multi-layer | `MLP(hidden_dims=...)` first; atomic `Dense`/`Activation` second |
| Diagram composition | Factory helper, not required for every user |
| Policies | `control/neural.py` wrappers with standard ports |
| Training | `identification/fitting.py` — same API for `gravity` and `W1` |
| Flax | `interfaces/flax.py` adapter only; not core block implementation |
| RNNs / discrete | Explicitly out of scope |

The existing demo already proves the hardest integration point — **differentiating
through a compiled closed-loop diagram**. The collection should grow outward from
that proven contract rather than adopting Flax as the foundation.

## Open questions for architectural review

Decide before P1 implementation:

1. **Rename `NeuralNetwork` → `MLP`?** Pre-1.0 no-alias rule says rename cleanly
   if yes.
2. **Submodule package `blocks/neural/` vs single `neural.py` file?** Subpackage
   scales better; single file matches current `filters.py` / `nonlinear.py` pattern
   for small collections.
3. **Activation at output layer?** Current: linear output (standard for control).
   Offer `output_activation="tanh"` for bounded controls?
4. **Where does Optax live?** New `[jax-train]` extra, or lazy import inside
   `identification/` only?
5. **Flax adapter scope:** import-only (user supplies trained weights) vs
   train-in-minilink?
6. **Residual / skip connections:** monolithic `ResidualMLP` class, or document
   manual `Sum` wiring in diagrams?

## Smallest high-value next slice

If approved, start with **P1: multi-layer `MLP` + extract the demo's training loop
into `identification/fitting.py`**, keeping the current `params` dict shape and
diagram wiring unchanged.
