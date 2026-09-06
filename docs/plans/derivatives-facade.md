# Partial derivatives on every `System` — plan (draft, 2026-09-06)

Status: **draft v3 for the maintainer's rulings** (§6). Nothing implemented.
Lane: teaching surface (core facade) over the evaluators. v3 steps back from
v2: two methods on `System` instead of six, values instead of functions at the
student tier, the function form on the evaluator, positional `("f", "x")`
selectors that read like the textbook, and a compiled-evaluator cache so an
exact Jacobian inside a loop costs microseconds, not a recompile.

## 1. What a student writes

```python
from minilink import Pendulum

plant = Pendulum()

A = plant.jacobian("f", "x")                    # ∂f/∂x at the nominal point (x0, ū, t = 0)
B = plant.jacobian("f", "u")                    # ∂f/∂u
C = plant.jacobian("y", "x")                    # ∂h/∂x — h is the compute of the y port

A = plant.jacobian("f", "x", x_bar, u_bar)      # any operating point, arguments as in f
S = plant.jacobian("f", "params", x_bar, u_bar) # {"m": (n,), "l": (n,), ...}
plant.jacobian("f", "t", x_bar, u_bar, t=2.0)   # ∂f/∂t, time-varying plants

lin = plant.linearize(x_bar, u_bar)             # LTISystem: lin.A(), lin.B(), lin.C(), lin.D()

loop = controller @ plant
A_cl = loop.jacobian("f", "x")                  # closed loop, same call

K = controller.jacobian("u", "y")               # any output port w.r.t. any input port
```

Every call returns a NumPy array, whatever backend ran underneath, of shape
`(dim(of), dim(wrt))`; `wrt="t"` gives `(dim(of),)` and `wrt="params"` a
dict shaped like `params` whose leaves are `(dim(of), *leaf.shape)`.

Optional keywords, all with textbook defaults:

| keyword | default | meaning |
| --- | --- | --- |
| `x`, `u`, `t`, `params` | `x0`, nominal `u`, `0.0`, the system's | operating point, in the order of `f(x, u, t, params)` |
| `method` | `"auto"` | `"jax"` exact when JAX is installed and the system traces, else central finite differences; `"fd"` / `"jax"` force one |
| `eps` | `1e-6` | central-difference step |

`method="auto"` falls back silently (the student asked for a derivative, not
for a backend); `method="jax"` raises when the system does not trace, like
`Sys2Gym(compile_backend="jax")`.

The research spelling is the same verb on the evaluator, returning a function:

```python
ev = plant.compile("jax")
dfdx = ev.jacobian("f", "x")        # jitted callable (x, u, t, params)
```

## 2. Why `jacobian("f", "x")`

The textbook phrase is "the Jacobian of f with respect to x", written ∂f/∂x,
and every maths tool students meet spells it as a positional pair in that
order: `jacobian(f, x)` in MATLAB and CasADi, `f.jacobian(x)` in SymPy,
`D[f, x]` in Mathematica. `plant.jacobian("f", "x")` is that call with the
symbols named instead of passed, so it needs no explanation in a notebook.

| Spelling | Verdict |
| --- | --- |
| `plant.jacobian("f", "x")` | **recommended** — (function, variable) order of ∂f/∂x and of every symbolic tool |
| `jacobian(of="f", wrt="x")` | the same call with keywords; `of` / `wrt` are the signature names so the docstring reads "Jacobian of `of` with respect to `wrt`"; student material never spells them |
| `jacobian(out="f", in="x")` | rejected — `in` is a Python keyword (SyntaxError), and out/in is block-diagram vocabulary: `f` is not an output signal and `x` is not an input |
| `jacobian("df/dx")` | rejected — string parsing, no completion, port ids with arbitrary characters |
| `df_dx()`, `dh_du()`, … (v2) | rejected — five names that still miss a controller's ∂u/∂y or ∂f/∂params; one verb covers all of them |
| `plant.A(x_bar, u_bar)` | rejected — `A(t, params)` already means something else on `LTISystem`; the textbook path to the four letters is `plant.linearize(x_bar, u_bar).A()` |

**`of` — what is differentiated**

| value | meaning |
| --- | --- |
| `"f"` | the state derivative, the `def f` a student writes (dynamic systems) |
| `"step"` | the state update, `def step` (step systems) |
| an output port id | that port's compute: `"y"` on a plant, `"u"` on a controller, `"p"` on a manipulator |

**`wrt` — differentiated with respect to**

| value | meaning |
| --- | --- |
| `"x"` | the state, the `x` argument of `f` |
| `"u"` | every input stacked in port order, the `u` argument of `f` |
| an input port id | one input port only: `"y"` or `"r"` on a controller |
| `"t"` | time (not offered on step systems, `k` is an integer) |
| `"params"` | the parameter dict; the result is a dict of the same shape |

Argument names win over port names, with one natural exception: a static block
has no state, so `"x"` there reaches the port (`StateFeedback` and
`LookupPolicy` name their measured-state input `x`, and
`ctl.jacobian("u", "x")` is their gain). A dynamic system whose input port is
named `x`, `t` or `params`, or whose output port is named `f` or `step`, gets
an error asking to rename the port rather than a silent wrong answer; none
exist in the library.

Manipulators: `arm.jacobian("p", "x")` differentiates against the full state
`(q, dq)` and is `(3, 2n)`; the textbook task-space Jacobian stays `arm.J(q)`.

## 3. Design — two tiers, one engine per backend

**Evaluator tier (function form).** Every evaluator gains
`jacobian(of, wrt, *, eps=1e-6)` returning a callable with the signature of
its parametric tier: `(x, u, t, params)` for dynamic and static evaluators,
`(x, u, k, params)` for step evaluators.

- JAX evaluators: `jax.jit(jax.jacfwd(trace_p_fn, argnums=…))` composed on the
  existing `_f_trace_p_fn` / `_outputs_trace_p_fn` / step tiers, with the
  output port picked from the `outputs` dict and an input port picked by its
  slice of `u`; cached per `(of, wrt)` on the evaluator like
  `rollout_batch`'s per-layout cache. `wrt="params"` differentiates the
  pytree, which is what `jacobian_f_params` does today.
- NumPy evaluators: the central-difference closure that `linearize_matrices`
  carries, moved here and applied to `f_p` / `outputs_p` / `step_p`; `params`
  perturbed leaf by leaf (nested dicts for diagrams).
- Diagram evaluators: the same on the plan's stacked tiers; port ids are the
  boundary ports. Internal signals keep the research spelling
  `linearize_matrices(outputs=(sys_id, port))`.

`evaluator.jacobian_f_params(x, u, t, params)` becomes
`evaluator.jacobian("f", "params")(x, u, t, params)` (§6, ruling 4).

**System tier (value form)**, in `minilink/core/facades.py`, two-line
delegations like `modal_analysis`:

```python
def jacobian(self, of, wrt, x=None, u=None, t=0.0, params=None, *,
             method="auto", eps=1e-6):
    J = self._compiled(method).jacobian(of, wrt, eps=eps)(x, u, t, params)
    return tree_map(np.asarray, J)
```

- Defaults: `x` → `x0`, `u` → `get_u_from_input_ports()`, `params` → the
  live `sys.params`, coerced to float arrays.
- `method="auto"`: the JAX evaluator when JAX is installed and the system
  traces, else the NumPy evaluator — the same "not JAX-traceable" test
  `Sys2Gym` applies; `"jax"` strict, `"fd"` NumPy.
- `linearize(x=None, u=None, t=0.0, params=None, *, method="auto", eps=1e-6)`
  is four `jacobian` calls (`f` and `y` against `x` and `u`) wrapped in an
  `LTISystem`, `C = I`, `D = 0` for a plant without a `y` port (today's rule).
  The module function `linearize(sys, …)` stays as the functional spelling;
  `linearize_matrices` keeps its `inputs=` / `outputs=` port selection but its
  arithmetic becomes these `jacobian` calls — one finite-difference loop and
  one `jacfwd` in the library instead of two of each.
- Placement: `jacobian` on `SharedSystemFacades` (dynamic, static and step
  systems, the `of` vocabulary following the kind); `linearize` on
  `DynamicSystemFacades` only (no discrete `LTISystem` exists;
  `jacobian("step", "x")` gives the discrete `A`).

**Compiled-evaluator cache on the system**, needed by this tier: a JAX
`jacobian` inside a loop over operating points would otherwise recompile per
call (0.3–1 s). `sys._compiled[backend]` is filled by the facade and cleared
by the structural mutators (`add_input_port`, `add_output_port`, and the
diagram's `add` / `connect`); `refresh()` is not the hook — three blocks
override it without `super()`. Parameter edits need no invalidation because
the facade always calls the parametric tier with the live `sys.params`; the
non-parametric tier bakes `params` in at compile time, which is why the facade
never touches it. The explicit `sys.compile()` stays uncached and is the
research spelling.

## 4. Steps (about five agent-hours)

1. **Evaluator `jacobian`** (2 h): the six evaluators (JAX and NumPy × leaf,
   diagram, static, plus step and step diagram); tests — finite differences
   vs JAX agreement on the catalog plants (reuse the `test_catalog_backends`
   parametrization), diagram closed loop, `params` dict shapes for a leaf and a
   nested diagram, `t` on a time-varying block, an input port of a controller,
   a static block, a step system, the name-clash errors.
2. **Facade + cache + consolidation** (1.5 h): `jacobian` and `linearize`
   methods, `_compiled` with invalidation, `linearize_matrices` rebuilt on the
   evaluator Jacobians (existing tests pass unchanged); tests — cache cleared
   by `add_output_port`, a params edit visible without recompile, `method`
   strictness, JAX-unavailable subprocess probe (Basic tier).
3. **Student material** (1 h, `[ask]`): §5.
4. **Docs** (0.5 h): DESIGN §"Tools" row and §4 `System` contract line, README
   analysis snippet, teaching-surface registry test extended with the two names.

Out of scope, reserved names: second derivatives (Hessians of a cost),
Jacobians along a `Trajectory` (time-varying LQR), sensitivities of a
simulation to `params` (`rollout_batch` + `jax.grad` remain the research
spelling).

## 5. What it does to the student material

| File | Today | With the facade |
| --- | --- | --- |
| `demos/analysis/analysis_linearize.py`, `analysis_structural.py` | `linearize(plant, x_bar)` | `plant.linearize(x_bar)` |
| `demos/analysis/analysis_linearize_fd_vs_jax.py` | 30 lines of `linearize_matrices(plant, xbar, ubar, method=...)` unpacking four matrices to print one | `plant.jacobian("f", "x", xbar, ubar, method="fd")` vs `method="jax"` — about ten lines |
| `demos/compile/params_gradient.py` part 1 | a 30-line `TraceablePendulum`, `compile("jax")`, `jnp` arrays, `evaluator.jacobian_f_params`, a hand-written FD loop | catalog `Pendulum` (it traces now), `plant.jacobian("f", "params", x, u)` against `method="fd"`, one print loop |
| `demos/compile/params_gradient.py` part 2, `pid_autotuning_jax.py`, `neural_controller_jax.py`, `cartpole_rollout_gradients.py` | gradients through rollouts | unchanged — a loss over data is the research spelling |
| `learn/intro/showcase_jax.ipynb` §3–4 | two `jax.jacfwd(lambda x: evaluator.f(x, u, t))` cells, a `linearize_matrices` compare, a `jacobian_f_params` table | `plant.jacobian("f", "x")`, `("f", "u")`, `("f", "params")`; one `jacfwd` cell kept as "what runs underneath" |
| `learn/intro/showcase_minilink.ipynb` cell 48 | `jax.jacfwd(lambda x: jax_eval.f(x, uc, 0.0))(xc)` | `plant.jacobian("f", "x", xc, uc)` |
| `learn/intro/07_compile.ipynb` cell 16 | `jax.jacfwd` over `f_trace` | kept — the notebook is about the evaluator; add `ev.jacobian("f", "x")` beside it |
| `learn/intro/04_analysis.ipynb` | `linearize(plant, x_bar)` + `linearize_matrices(plant, x_bar, method="fd")` | `plant.linearize(x_bar)` + `plant.jacobian("f", "x", x_bar, method="fd")` |
| `learn/intro/03_control.ipynb` (optional) | `lqr_at_operating_point(plant, x_bar, Q, R)` | can also show `A, B = plant.jacobian("f", "x", x_bar), plant.jacobian("f", "u", x_bar); K = lqr(A, B, Q, R)` |

VI / DP, trajectory optimization, and MPC material is untouched.

## 6. Rulings needed

1. **Spelling.** Positional `jacobian("f", "x")` with `of` / `wrt` as the
   signature names (recommended), or another keyword pair.
2. **Selector words.** `"params"` (matches the argument name, recommended)
   or `"p"`; `"step"` for step systems (mirrors `def step`, recommended) or
   `"f"` everywhere.
3. **`method="jax"` strictness.** Raise when the system does not trace
   (recommended, matches `Sys2Gym`) — today `linearize_matrices(method="jax")`
   warns and falls back; and `linearize_matrices`' default moves from `"fd"`
   to `"auto"` (recommended) or stays.
4. **`jacobian_f_params`.** Remove now (recommended, two call sites in
   `params_gradient.py` and `showcase_jax.ipynb`, both rewritten in step 3)
   or keep as an alias for one release.
5. **Cache.** As designed, invalidated at the structural mutators
   (recommended), or compile per call and accept the loop cost.
