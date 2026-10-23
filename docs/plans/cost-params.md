# Cost parameters: one dictionary, nested like a diagram's

Status: core contract, design agreed with the maintainer 2026-09-13 (composite costs nest
their parameters the way a diagram nests its subsystems'); the cost's surface ruled 2026-10-22/23
(R9, R20 of [the rulings](../reviews/2026-10-22-rulings-session-1.md), D16b of
[end-state-api.md](end-state-api.md); amended in session 3 by R32, the upper-bound hint
`g_max` / `h_max`, R41, one home per number, `ProblemParameters` retired, and R45, the record's
names); aligned to [continuous-core.md](continuous-core.md) §3.3
on 2026-10-23; implementation not started.
Rung: v0.3, the continuous-core phase: the surface lands in V1's steps 1 and 2 (the cost's
`integrate`, `g_max`, `h_max`, the moves to the problem), the nesting is step A2 of [TODO.md](TODO.md)
§4, V1's step 6.

## The cost's surface after the rulings (2026-10-22/23)

The cost is a pure integrand plus one quadrature; it knows nothing about a horizon, a set, a
price or a discount:

```python
class CostFunction:
    params                                         # Q, R, S, xbar, ubar …; None for a user cost without parameters
    def g(self, x, u, t=0.0, params=None)          # running cost density
    def h(self, x, t=0.0, params=None)             # terminal cost
    def g_max(self, X, U, params=None)             # upper bound of g over the boxes X, U (exact on a box for a quadratic, sampled otherwise)
    def h_max(self, X, params=None)                # upper bound of h over X (0 when h = 0)
    def integrate(self, traj, *, discount_rate=0.0, params=None)   # the running cost (e^{−ρt_k} g_k) and J(t_k) = ∫₀^{t_k} e^{−ρt} g dt, both (N,), trapezoid, on xp; no h
    __add__, __mul__                               # SumCost, ScaledCost; params nested by term id (this plan)

evaluate_trajectory(problem, traj)                 # problem-level: cost = integrate(...)[-1] + e^{−ρ tf} h; the margins; J (priced when the switch is on)
traj.with_cost(cost, discount_rate=0.0)            # the plotting wrapper: the "cost_rate" and "cost" signals
```

Leave the cost: `discount_rate` and `discount_factor` (the problem's,
`PlanningProblem.discount_rate`, read by RL's γ, DP's α and the gym bridge), and
`evaluate_trajectory`, `terminal_cost`, `total_cost` (replaced by the problem-level
`evaluate_trajectory`). No scoring modes (D20): no `constraints=` or `discount=` keyword
anywhere.

**The upper-bound hint (R32).** `g_max` and `h_max` are the cost's one statement about its own
scale, always defined. The problem resolves them to a finite price,
`problem.cost_bound() = g_max(X_b, U_b) · tf + h_max(X_b)` (`/ ρ` on an infinite
horizon), `X_b`, `U_b` the declared boxes, else the plant's own, else `1e6` with one warning;
DP, RL and later MPPI read it when `infeasible_cost` is `None`, and `infeasible_cost=True` puts
it in the run's score `J`. The cost itself still knows no price.

The rest of this document is the parameter design (A2), unchanged in substance, with one home
per number (R41): the cost's `params` is the home of `Q`, `R` and the targets;
`ProblemParameters`, the problem's parameter bundle, is retired, and its readers move to the
cost's own `params` with A2.

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
- The compiled problem (V1 in [TODO.md](TODO.md) §4, v0.3; `pe.params["cost"]`) needs cost
  weights and targets as leaves, next to plant and controller parameters, so
  `jax.grad(lambda p: pe.evaluate(p).objective)` and `pe.program(wrt=["cost.…"])` reach
  them.

## What is already in place

The path runs end to end; only the costs and their composition are missing.

| Caller | Passes today | After V1 and A2 (R41) |
| --- | --- | --- |
| Trajectory optimization (collocation, shooting, multiple shooting) | `problem.params.cost` (the `ProblemParameters` bundle) to every `g` and `h` | `problem.cost.params`, the cost's own (the bundle retired) |
| Value iteration and policy evaluation | `problem.params.cost` | `problem.cost.params` |
| The scoring rule (today's `score_trajectory`, `CostFunction.evaluate_trajectory`) | a `params` argument | one problem-level `evaluate_trajectory(problem, traj)` reading `cost.integrate(..., params=)` (V1's step 2) |
| RL environment, JAX Monte Carlo backend | nothing (the cost's own defaults) | the environment reads `problem.cost.params`; the Monte Carlo path folds into `evaluate` in v0.3, right after it exists (R44) |
| The compiled problem | does not exist | `pe.params["cost"]`, the cost's nested dictionary in the tree assembled from the objects; `pe.evaluate(p)` takes an override tree (V1's step 6) |

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
| partial dictionaries allowed, unknown ids raise (`check_diagram_params`) | same (`validate_cost_params`) |
| the role names `plant`, `controller`, `estimator` (a clash raises), plain names elsewhere with a numeric suffix for repeats, `System.id` naming a block placed by `+` or `>>` | ids from class names, numeric suffix for repeats, `cost.id` overrides |
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

`TimeCost.eps` is a threshold with no useful gradient and stays an attribute, like a system's
dimensions. `discount_rate` leaves the cost for the problem (`PlanningProblem.discount_rate`,
ruled 2026-10-22, R9 and R20): the cost owns the integrand `g` and `h`, the problem owns the
functional's shape (`tf`, the sets, the infeasible price, the discount). So `SumCost` and
`ScaledCost` no longer reconcile or forward a rate: the check that the terms' rates agree goes
with the attribute.

### Scene parameters travel separately

A `FieldCost` passes `params` down to `scene.clearance(world, params=params)`, so every
obstacle term would share one scene override. The plumbing is unused: the shapes accept
`params` and none reads it, and the online planner raises for a `scene` key. A field term's
dictionary therefore holds its own weight, `FieldCost` stops forwarding its parameters to the
field, and scene overrides are designed with S76 (parametric-program.md) as the scene's own
`params` in the assembled tree (R41; the `ProblemParameters.scene` slot reserved for them goes
with the bundle). Nothing changes numerically today.

## Beyond this plan: the cost as a block (closed, R77)

No `CostIntegrator` block: `cost.integrate` on the recorded samples is the one contract for a run's
cost. The augmented-state formulation (`dJ/dt = e^{−ρt} g` as one more state of the loop) is an
optional teaching demo, a few lines of custom `DynamicSystem` compared against `evaluate`'s cost (RULES
6.15), the maintainer's call.

## Decisions (2026-09-13; 3 and 5 from 2026-10-22/23)

1. Library costs follow the System dual parameter pattern.
2. Composite costs nest parameters by term id, with the diagram's live-view, partial-dictionary
   and unknown-id rules.
3. `TimeCost.eps` stays an attribute; `discount_rate` moves to the problem (ruled 2026-10-22).
4. User costs that ignore `params` keep working; library cost attributes stay readable and
   writable in place.
5. The cost's surface is `params`, `g`, `h`, `g_max`, `h_max`, `integrate` and the algebra;
   `evaluate_trajectory`, `terminal_cost`, `total_cost` leave it for the problem-level verb
   (ruled 2026-10-23, R20; `h_max` added by R32).
6. One home per number: the cost's `params`; `ProblemParameters` retired (R41).

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
   forwarding; S76 designs the scene channel.
5. **Dataclasses.** Proposed: plain classes, as systems are. Nothing compares, copies with
   `replace` or serializes costs; the keyword constructors and `from_system` keep working;
   the parameters become mutable, as a system's are.
6. **The cost block.** Proposed (2026-10-23): closed as a demo idea; the compiled problem is
   the route to the differentiable cost (the ask is differentiable-problem.md §9).

## Consequences

- `SumCost.terms` becomes a dictionary `{term_id: cost}`; `len(cost.terms)` still works.
- `test_params_forwarded_to_terms` changes: a flat dictionary given to a sum raises for its
  unknown keys; the test gives each term its own dictionary instead.
- Planners, evaluators and RL are unchanged for nominal runs (`params=None`).
- Sums of several field costs get ids like `field`, `field2`; code that differentiates them
  names its terms (`SumCost.of(path=..., corridor=...)`) so gradients read by role.
- `ProblemParameters` goes (R41): every reader of `problem.params.cost` reads
  `problem.cost.params`; the compiled problem's `pe.params["cost"]` is the same dictionary in the
  assembled tree.
- `SumCost` loses its discount-rate reconciliation (the rate is the problem's).
- Online cost weights and targets in MPC (a moving `xbar`) become possible once the parametric
  program binds `p` (S76, its step PP-3, right after this plan).

## Migration outline

What each step builds; the order and the "done when" are TODO §4 (V1's steps 1, 2 and 6).

1. **The surface** (V1's step 1): `integrate(traj, *, discount_rate, params)` on
   `xp = array_module(traj.x)` (`vmap` on JAX, vector or loop on NumPy, no `float` cast), the
   one quadrature of the running integral; `g_max(X, U)` (exact on a box for a quadratic,
   sampled otherwise) and `h_max(X)`, the upper-bound hint (R32); `traj.with_cost(cost, discount_rate=0.0)` attaching the `cost_rate` and
   `cost` signals. Byte-identical.
2. **The moves to the problem** (V1's step 2): `discount_rate` to `PlanningProblem` (one
   ledger line; RL, DP and the gym bridge read it there; `SumCost`'s reconciliation goes);
   `evaluate_trajectory`, `terminal_cost`, `total_cost` leave the cost for the problem-level
   `evaluate_trajectory`.
3. **The parameters** (A2, V1's step 6): the dual pattern on `CostFunction`, `QuadraticCost`
   and `TimeCost`, attributes as views; `SumCost.terms` as `{term_id: cost}`,
   `SumCost.of(*costs, **named)` with the id and flattening rules, live `params` views on
   `SumCost` and `ScaledCost`, routing in `g` and `h`, `validate_cost_params`; the forwarding
   test updated; `FieldCost`'s weight a parameter, no longer forwarded to the field (open
   question 4); every planner and the compiled problem read `problem.cost.params`, and
   `ProblemParameters` is retired with its readers migrated (R41).
4. Fold the landed contract into the DESIGN parameters section and delete this document.

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
- `jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)["cost"]["Q"]` runs on the
  compiled problem and matches a central difference.
- `vmap` over a family of `R` weights runs on the compiled path.
