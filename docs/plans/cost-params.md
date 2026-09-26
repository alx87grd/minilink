# Cost parameters: one dictionary, nested like a diagram's

Status: core contract, design agreed with the maintainer 2026-09-13 (composite costs nest
their parameters the way a diagram nests its subsystems'); implementation not started.
Rung: v0.2 wave A, step A2 of [TODO.md](TODO.md).

## Problem

A `System` takes its parameters as a dictionary: `self.params` holds the defaults and every
equation accepts `params`, so `jax.grad` and `vmap` reach masses, lengths and gains. A
`CostFunction` has the same slot, `g(x, u, t, params=None)` and `h(x, t, params=None)`, but
nothing uses it.

- No library cost reads `params`. `FieldCost` only forwards it to its scene.
- `QuadraticCost` is a frozen dataclass that converts its arrays with `np.asarray`, so
  `jax.grad` with respect to `Q` raises `TracerArrayConversionError`.
- `SumCost` hands the same dictionary to every term: two quadratic terms would both read
  `"Q"`, so their weights cannot be set or differentiated separately.
- The differentiable closed-loop cost (V1 in [TODO.md](TODO.md) §6) needs cost weights and
  targets as inputs, next to plant and controller parameters.

## What is already in place

The path runs end to end; only the costs and their composition are missing.

| Caller | Passes |
| --- | --- |
| Trajectory optimization (collocation, shooting, multiple shooting) | `problem.params.cost` to every `g` and `h` |
| Value iteration and policy evaluation | `problem.params.cost` |
| Monte Carlo score (`score_trajectory`) and `evaluate_trajectory` | a `params` argument |
| RL environment, JAX Monte Carlo backend | nothing (the cost's own defaults) |

Census on 2026-09-13: five library costs (`QuadraticCost`, `TimeCost`, `SumCost`,
`ScaledCost`, `FieldCost`) and about twenty user-written costs in demos, notebooks and tests,
none of which reads `params`. No library code reads cost attributes; the pendulum notebooks
read `cost.Q` / `cost.R` and write them in place (`cost.R[0, 0] = 0.1 / DT`).

## Design

### A leaf cost follows the System pattern

```python
class QuadraticCost(CostFunction):
    def __init__(self, Q, R, S, xbar, ubar):
        self.params = {"Q": Q, "R": R, "S": S, "xbar": xbar, "ubar": ubar}

    def g(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        Q, R, xbar, ubar = params["Q"], params["R"], params["xbar"], params["ubar"]

        dx = x - xbar
        du = u - ubar
        return dx @ Q @ dx + du @ R @ du
```

- `params is None` resolves to `self.params`; any dictionary replaces it at that level.
- The attributes stay: `cost.Q` is a view of `cost.params["Q"]`, so the notebooks' in-place
  writes keep working.
- A user cost that hard-codes its constants and ignores `params` keeps working unchanged.

### A composite cost nests its terms' parameters, like a diagram

```python
cost = quadratic + 5.0 * clearance.as_cost() + SwingUpCost()
list(cost.terms)            # ['quadratic', 'field', 'swing_up']
cost.params
# {"quadratic": {"Q": ..., "R": ..., "S": ..., "xbar": ..., "ubar": ...},
#  "field":     {"weight": 5.0, "cost": {"weight": 1.0}},
#  "swing_up":  None}

cost.g(x, u, t, {"field": {"weight": 0.0}})       # only the obstacle term changes
jax.grad(J)(cost.params)["field"]["weight"]        # dJ/dw, as for a plant parameter
```

The diagram's rules, so there is one mental model for both:

| Diagram | Composite cost |
| --- | --- |
| `diagram.subsystems`: `{sys_id: block}` | `SumCost.terms`: `{term_id: cost}` |
| `diagram.params`: live view `{sys_id: block.params}`, setter distributes | `SumCost.params`: live view `{term_id: cost.params}`, setter distributes |
| `f` routes `params.get(sys_id)` to each block, `None` falls back to the block's own | `g` and `h` route `params.get(term_id)` the same way |
| partial dictionaries allowed, unknown ids raise (`validate_diagram_params`) | same (`validate_cost_params`) |
| ids from roles and class names, numeric suffix for repeats, `System.id` overrides | ids from class names, numeric suffix for repeats, `cost.id` overrides |
| nested diagrams nest the dictionary | named sums nest the dictionary |

Rules proposed for the questions below:

- **Term ids.** The class name without its `Cost` / `CostFunction` suffix, normalized as
  diagram ids are (`QuadraticCost` → `quadratic`, `SwingUpCost` → `swing_up`); a repeat gets
  `quadratic2`; a `ScaledCost` takes the id of the cost it scales; `cost.id` overrides.
- **Flattening.** `a + b + c` and `sum([...])` flatten into one level, so the dictionary does
  not depend on Python's left-to-right evaluation. Terms passed by name,
  `SumCost.of(tracking=q, obstacles=a + b)`, and costs with an `id` stay nested.
- **Partial dictionaries.** A composite fills what is missing from its own parameters (a
  term id for `SumCost`, `weight` or `cost` for `ScaledCost`); a leaf dictionary is a full
  replacement, as for a block.
- **Scaled costs.** `ScaledCost` holds its weight and nests the scaled cost,
  `{"weight": w, "cost": {...}}`, even when that cost has a weight of its own.
- **Costs without parameters.** `CostFunction.params = None` at class level: no shared mutable
  default, and `None` is an empty JAX pytree, so a student cost sums, differentiates (no
  leaves) and vectorizes with the rest.
- **Validation** runs where the diagram's does: in the setter and at the entry of `g` and `h`
  of a sum.

A scratchpad prototype of these rules (2026-09-13) checked each line of the example above:
automatic ids, repeats, named nesting, a partial override touching one term, an unknown id
raising, an in-place `cost.R[0, 0]` write reaching the sum, `jax.grad` of a term weight
matching finite differences, `vmap` over that weight, and a student cost without parameters
inside the sum.

### What stays a structural attribute

`horizon` and `discount_rate` choose the algorithm and the effective horizon (planners
convert the rate with `float()`); `TimeCost.eps` is a threshold with no useful gradient. They
stay attributes, like a system's dimensions.

### Scene parameters travel separately

A `FieldCost` passes `params` down to `scene.clearance(world, params=params)`, so every
obstacle term would share one scene override. The plumbing is unused: the shapes accept
`params` and none reads it, and the online planner raises for a `scene` key. A field term's
dictionary therefore holds its own weight, `FieldCost` stops forwarding its parameters to the
field, and scene overrides are designed with pipeline B through `ProblemParameters.scene`,
already reserved for them. Nothing changes numerically today.

## Beyond this plan: the cost as a block

A prototype `CostIntegrator(cost)` is a `DynamicSystem` with state `J` and
`dJ/dt = exp(-rho t) g(x, u, t; p)`, wired to the plant state and the controller output. The
closed loop then holds one nested dictionary, `{"ctl": ..., "sys": ..., "cost": ...}`, and
`jax.grad` through the compiled loop returned `dJ/dK`, `dJ/dQ`, `dJ/dR` and `dJ/dm` in one
call. Its `J` matched the post-hoc `compute_cost` to 0.02 % on a pendulum loop. This is the
route to the differentiable closed-loop cost; it depends on the cost dictionary above.
Caveats: the terminal cost and the exit rule stay outside the block, and integrating `J` with
the ODE solver differs from the trapezoid rule of the post-hoc path.

## Decisions (2026-09-13)

1. Library costs follow the System dual parameter pattern.
2. Composite costs nest parameters by term id, with the diagram's live-view, partial-dictionary
   and unknown-id rules.
3. `horizon`, `discount_rate` and `TimeCost.eps` stay attributes.
4. User costs that ignore `params` keep working; library cost attributes stay readable and
   writable in place.

## Open questions (each with the proposed answer)

1. **Default term ids for `+`.** Proposed: from the class name, as diagram ids are
   (`quadratic`, `field`, `quadratic2`), with `cost.id` to override. Rejected: positional ids
   (`term0`), which change when a term is added or reordered and say nothing in a gradient.
2. **Flattening.** Proposed: anonymous `+` and `sum` flatten; terms passed by name and costs
   with an `id` stay nested. Without flattening, `a + b + c` would nest as
   `{"sum": {"sum": {...}}}` because Python evaluates it left to right.
3. **Default `params` for costs that never set one.** Proposed: `params = None` at class level.
   Rejected: a class-level `{}` (one mutable dictionary shared by every subclass) and a base
   `__init__` (student costs define `__init__` without calling `super()`).
4. **Scene overrides.** Proposed: out of this plan. `FieldCost` keeps its weight only and stops
   forwarding; pipeline B designs the scene channel.
5. **Dataclasses.** Proposed: plain classes, as systems are. Nothing compares, copies with
   `replace` or serializes costs; the keyword constructors and `from_system` keep working;
   the parameters become mutable, as a system's are.
6. **The cost block.** Proposed: a separate step after this plan, in the research lane; its
   name (`CostIntegrator`), a wiring helper on `add_subsystem` / `connect`, which integration
   rule each tool reports, and the terminal cost wait for the terminal-cost decision in ROADMAP §6.

## Consequences

- `SumCost.terms` becomes a dictionary `{term_id: cost}`; `len(cost.terms)` still works.
- `test_params_forwarded_to_terms` changes: a flat dictionary given to a sum raises for its
  unknown keys; the test gives each term its own dictionary instead.
- Planners, evaluators and RL are unchanged for nominal runs (`params=None`).
- Sums of several field costs get ids like `field`, `field2`; code that differentiates them
  names its terms (`SumCost.of(path=..., corridor=...)`) so gradients read by role.
- `ProblemParameters.cost` can default to the cost's live `params`.
- Online cost weights and targets in MPC (a moving `xbar`) become possible once the parametric
  program binds `p` (pipeline B).

## Migration outline

1. Dual pattern on `CostFunction`, `QuadraticCost` and `TimeCost`, attributes as views.
2. Composition: `SumCost.terms` as `{term_id: cost}`, `SumCost.of(*costs, **named)` with the
   id and flattening rules, live `params` views on `SumCost` and `ScaledCost`, routing in `g`
   and `h`, `validate_cost_params`; update the forwarding test.
3. `FieldCost` weight as a parameter, no longer forwarded to the field (open question 4).
4. Fold the landed contract into the DESIGN parameters section and delete this document.
5. Research lane: `CostIntegrator` in `examples/experimental/`, with tutorial 11's rollout and
   `pid_autotuning_jax` rewritten on it; decide promotion with the differentiable closed-loop
   cost.

## Verification

- `jax.grad` of `g` and `h` with respect to every leaf parameter matches central differences
  on NumPy.
- `cost.R[0, 0] = ...` still changes `g`.
- `(q + 5.0 * f).params` is nested; a partial override changes only its term; an unknown id
  raises.
- `q + q + q` and `sum([...])` flatten with suffixed ids; a named term keeps its nesting; a
  scaled term takes the id of the cost it scales.
- A student cost without parameters sums, differentiates and vectorizes inside a sum.
- The MPC and trajectory-optimization projects that add costs with `+` run unchanged.
- Seeded baselines of trajectory optimization, value iteration, Monte Carlo and RL at nominal
  parameters are byte-identical before and after.
- `vmap` over a family of `R` weights runs on the compiled path.
