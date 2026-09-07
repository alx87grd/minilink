# Partial derivatives and the analysis family on every `System` — plan (draft, 2026-09-06)

Status: **draft v4 for the maintainer's rulings** (§6). Nothing implemented.
Lane: teaching surface (core facade) over the evaluators. v3 stepped back to
two methods on `System` (`jacobian`, `linearize`), values at the student tier
and the function form on the evaluator. v4 extends the same calling pattern to
the rest of the analysis family (`bode`, `pzmap`, `modal_analysis`,
`find_equilibrium`, `lqr_at_operating_point`, a new `transfer_function`), §7,
and settles what "internal variables" of a diagram can mean, §8.

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
G = plant.transfer_function(x_bar, u_bar)       # SISO TransferFunction, default channel
plant.plot_bode(x_bar, of=("y", 1), wrt="u")    # one channel: y[1] from u

loop = controller @ plant
A_cl = loop.jacobian("f", "x")                  # closed loop, same call
loop.jacobian("plant:y", "x")                   # an internal signal, by wire name

K = controller.jacobian("u", "y")               # any output port w.r.t. any input port
```

Every call returns a NumPy array, whatever backend ran underneath, of shape
`(dim(of), dim(wrt))`; `wrt="t"` gives `(dim(of),)` and `wrt="params"` a
dict shaped like `params` whose leaves are `(dim(of), *leaf.shape)`.

Optional keywords, all with textbook defaults, shared by the whole family:

| keyword | default | meaning |
| --- | --- | --- |
| `x_bar`, `u_bar`, `t`, `params` | `x0`, nominal `u`, `0.0`, the system's | operating point, in the order of `f(x, u, t, params)` |
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
| `"block:port"` | on a diagram, the signal on the wire leaving that block's output port (§8) |

**`wrt` — differentiated with respect to**

| value | meaning |
| --- | --- |
| `"x"` | the state, the `x` argument of `f` |
| `"u"` | every input stacked in port order, the `u` argument of `f` |
| an input port id | one input port only: `"y"` or `"r"` on a controller |
| `"t"` | time (not offered on step systems, `k` is an integer) |
| `"params"` | the parameter dict; the result is a dict of the same shape |
| `"block:port"` | on a diagram, an additive perturbation of that wire (§8, ruling 9) |

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
- Diagram evaluators: the same on the plan's stacked tiers; boundary ports by
  id, internal wires by `"block:port"` (§8).

`evaluator.jacobian_f_params(x, u, t, params)` becomes
`evaluator.jacobian("f", "params")(x, u, t, params)` (§6, ruling 4).

**System tier (value form)**, in `minilink/core/facades.py`, two-line
delegations like `modal_analysis`:

```python
def jacobian(self, of, wrt, x_bar=None, u_bar=None, t=0.0, params=None, *,
             method="auto", eps=1e-6):
    J = self._compiled(method).jacobian(of, wrt, eps=eps)(x_bar, u_bar, t, params)
    return tree_map(np.asarray, J)
```

- Defaults: `x_bar` → `x0`, `u_bar` → `get_u_from_input_ports()`, `params` →
  the live `sys.params`, coerced to float arrays.
- `method="auto"`: the JAX evaluator when JAX is installed and the system
  traces, else the NumPy evaluator — the same "not JAX-traceable" test
  `Sys2Gym` applies; `"jax"` strict, `"fd"` NumPy.
- `linearize(x_bar=None, u_bar=None, t=0.0, params=None, *, of=None,
  wrt=None, method="auto", eps=1e-6)` is four `jacobian` calls (`f` and the
  selected outputs against `x` and the selected inputs) wrapped in an
  `LTISystem`; defaults are every input and every output, `C = I`, `D = 0`
  for a plant without a `y` port (today's rule). The module function
  `linearize(sys, …)` stays as the functional spelling; `linearize_matrices`
  takes the same arguments and returns the four arrays — one
  finite-difference loop and one `jacfwd` in the library instead of two of
  each.
- Placement: `jacobian` on `SharedSystemFacades` (dynamic, static and step
  systems, the `of` vocabulary following the kind); `linearize` and the §7
  channel tools on `DynamicSystemFacades` only (no discrete `LTISystem`
  exists; `jacobian("step", "x")` gives the discrete `A`).

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

## 4. Steps (about eight agent-hours)

1. **Evaluator `jacobian`** (2.5 h): the six evaluators (JAX and NumPy × leaf,
   diagram, static, plus step and step diagram), internal wires as `of` and,
   if ruled, as `wrt`; tests — finite differences vs JAX agreement on the
   catalog plants (reuse the `test_catalog_backends` parametrization), diagram
   closed loop, an internal wire of a diagram, `params` dict shapes for a leaf
   and a nested diagram, `t` on a time-varying block, an input port of a
   controller, a static block, a step system, the name-clash errors.
2. **Facade + cache + consolidation** (1.5 h): `jacobian` and `linearize`
   methods, `_compiled` with invalidation, `linearize_matrices` rebuilt on the
   evaluator Jacobians; tests — cache cleared by `add_output_port`, a params
   edit visible without recompile, `method` strictness, JAX-unavailable
   subprocess probe (Basic tier).
3. **Family alignment** (2 h, §7): the signatures in the table, the
   `(port, index)` channel selector, `transfer_function`, the renamed
   keywords swept through library, tests and examples (about 130 sites, listed
   in §7), the facades reduced to delegations.
4. **Student material** (1 h, `[ask]`): §5.
5. **Docs** (0.5 h): DESIGN §"Tools" row and §4 `System` contract line, README
   analysis snippet, teaching-surface registry test extended with the new
   names.

Out of scope, reserved names: second derivatives (Hessians of a cost),
Jacobians along a `Trajectory` (time-varying LQR), sensitivities of a
simulation to `params` (`rollout_batch` + `jax.grad` remain the research
spelling), loop opening at a wire (§8).

## 5. What it does to the student material

| File | Today | With the facade |
| --- | --- | --- |
| `demos/analysis/analysis_linearize.py`, `analysis_structural.py` | `linearize(plant, x_bar)` | `plant.linearize(x_bar)` |
| `demos/analysis/analysis_linearize_fd_vs_jax.py` | 30 lines of `linearize_matrices(plant, xbar, ubar, method=...)` unpacking four matrices to print one | `plant.jacobian("f", "x", xbar, ubar, method="fd")` vs `method="jax"` — about ten lines |
| `demos/analysis/analysis_bode.py` | `bode(plant, x_bar=x_bar, input_port="u", output_port="y", output_index=1)`, four times over | `plant.bode(x_bar, of=("y", 1))`, `plant.pzmap(...)`, `plant.plot_bode(...)`; `plant.transfer_function(x_bar, of=("y", 1))` printed as `num / den` |
| `demos/compile/params_gradient.py` part 1 | a 30-line `TraceablePendulum`, `compile("jax")`, `jnp` arrays, `evaluator.jacobian_f_params`, a hand-written FD loop | catalog `Pendulum` (it traces now), `plant.jacobian("f", "params", x, u)` against `method="fd"`, one print loop |
| `demos/compile/params_gradient.py` part 2, `pid_autotuning_jax.py`, `neural_controller_jax.py`, `cartpole_rollout_gradients.py` | gradients through rollouts | unchanged — a loss over data is the research spelling |
| `learn/intro/showcase_jax.ipynb` §3–4 | two `jax.jacfwd(lambda x: evaluator.f(x, u, t))` cells, a `linearize_matrices` compare, a `jacobian_f_params` table | `plant.jacobian("f", "x")`, `("f", "u")`, `("f", "params")`; one `jacfwd` cell kept as "what runs underneath" |
| `learn/intro/showcase_minilink.ipynb` cell 48 | `jax.jacfwd(lambda x: jax_eval.f(x, uc, 0.0))(xc)` | `plant.jacobian("f", "x", xc, uc)` |
| `learn/intro/07_compile.ipynb` cell 16 | `jax.jacfwd` over `f_trace` | kept — the notebook is about the evaluator; add `ev.jacobian("f", "x")` beside it |
| `learn/intro/04_analysis.ipynb` | `linearize(plant, x_bar)` + `linearize_matrices(plant, x_bar, method="fd")`; cell 7 unpacks `bode` as `mag, phase, omega` although it returns `(w, mag, phase)` — a live bug, hidden because the three arrays have the same length | `plant.linearize(x_bar)` + `plant.jacobian("f", "x", x_bar, method="fd")`; `w, mag, phase = plant.bode(x_bar)` |
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
   warns and falls back; and the family default moves from `"fd"` to `"auto"`
   (recommended) or stays.
4. **`jacobian_f_params`.** Remove now (recommended, two call sites in
   `params_gradient.py` and `showcase_jax.ipynb`, both rewritten in step 4)
   or keep as an alias for one release.
5. **Cache.** As designed, invalidated at the structural mutators
   (recommended), or compile per call and accept the loop cost.
6. **Operating-point names.** `x_bar, u_bar, t, params` family-wide
   (recommended: the textbook x̄, ū, what every tool already takes, no churn
   on 92 call sites, and no `"x", x=` stutter in `jacobian`) — v3 had `x, u`.
7. **Channel selectors on the channel tools** (§7). Keyword-only
   `of=` / `wrt=` with the textbook default channel (recommended: one
   vocabulary for the family, `of` is the numerator of ∂f/∂x and of
   Y(s)/U(s) alike), or `output=` / `input=`; one component as a
   `(port, index)` tuple (recommended) or a separate `_index` keyword.
8. **Additions.** `transfer_function` returning the `TransferFunction` block
   (recommended: `pzmap` already computes `num, den`), `find_equilibrium` as a
   method, `controllability` / `observability` accepting an `LTISystem`,
   `discretize(integrator=)` to match `Sys2Gym` — each yes / no.
9. **Internal wires** (§8). `of="block:port"` now (recommended, free);
   `wrt="block:port"` now (about 1.5 h) or reserved with a clear error; loop
   opening later (recommended).
10. **`lqr_at_operating_point`.** Gains `method` / `eps` with its argument
    order kept (recommended) or moves to `(sys, Q, R, x_bar=None, u_bar=None)`
    so the point defaults like everywhere else.

## 7. The analysis family — one calling pattern

**Today** the family spells the same three things in several ways:

- The channel is `inputs=[…]` / `outputs=[…]` on `linearize_matrices`
  (internal outputs as `(sys_id, port_id)` tuples), `input_port` /
  `input_index` / `output_port` / `output_index` on `bode`, `pzmap` and the
  two plot facades, and nothing on `modal_analysis` and
  `lqr_at_operating_point`.
- The operating point is `x_bar, u_bar` almost everywhere, `x_guess, u` on
  `find_equilibrium`, and `x_bar, Q, R, u_bar` on `lqr_at_operating_point`.
- `method="fd"` is the default everywhere the keyword exists;
  `lqr_at_operating_point` has no `method` at all; the step is `epsilon`.
- `plot_bode`, `plot_pzmap` and `modal_analysis` facades repeat the
  `x_bar = self.x0` default their module function already applies, and
  `plot_bode` re-declares fifteen keywords.
- Nothing returns a transfer function although the `TransferFunction` block
  exists and `pzmap` builds `num, den` on the way to its zeros and poles.
- `discretize(method="rk4")` uses `method` for the integrator, the word
  `Sys2Gym` spells `integrator=`.

**The pattern.** Every tool reads
`tool(<what>, x_bar=None, u_bar=None, t=0.0, params=None, *, method="auto", eps=1e-6, <tool options>)`:
required selectors come first and positional (`jacobian`), optional selectors
are keyword-only `of=` / `wrt=` with the textbook default channel, the
operating point follows in the order of `f`, and the module function is the
same signature with `sys` prepended.

| tool | method form (module form prepends `sys`) | returns |
| --- | --- | --- |
| `jacobian` | `(of, wrt, x_bar=None, u_bar=None, t=0.0, params=None, *, method, eps)` | array, or dict for `"params"` |
| `linearize` | `(x_bar=None, u_bar=None, t=0.0, params=None, *, of=None, wrt=None, method, eps)` | `LTISystem` |
| `linearize_matrices` (module only) | as `linearize` | `A, B, C, D` |
| `transfer_function` (new) | as `linearize`, one channel | `TransferFunction` |
| `bode` | as `linearize`, one channel, plus `w=None, n=200` | `(w, magnitude_db, phase_deg)` |
| `pzmap` | as `linearize`, one channel | `(zeros, poles, gain)` |
| `plot_bode`, `plot_pzmap` | as `bode` / `pzmap` plus `backend, show` | `PlotResult` |
| `modal_analysis` | `(x_bar=None, u_bar=None, t=0.0, params=None, *, mode=None, method, eps, …animation)` | `(poles, modes)` |
| `find_equilibrium` (also a method) | `(x_guess, u_bar=None, t=0.0, params=None, *, tol=1e-9)` | `x_eq` |
| `lqr_at_operating_point` (module) | `(sys, x_bar, Q, R, u_bar=None, t=0.0, params=None, *, method, eps)` | `StateFeedbackController` |
| `discretize` | `(dt=None, *, integrator="rk4", params=None)` | `StepSystem` |
| `controllability`, `observability` | `(A, B)` / `(A, C)`, or an `LTISystem` | unchanged |

Channel selectors take the §2 vocabulary: `of` is an output port id, an
internal wire `"block:port"`, or `(port, index)` for one component; `wrt` is
an input port id, an internal wire, or `(port, index)`; `linearize` and
`linearize_matrices` also accept lists (rows and columns in that order).
Defaults: `linearize` takes every input and every output; the one-channel
tools take component 0 of the default output and of the first input port,
which is what `bode(plant)` means today.

Migration, one pass through library, tests and examples: `epsilon=` → `eps=`
(31 sites), `input_port` / `output_port` / `_index` → `of` / `wrt` (84 sites),
`inputs=` / `outputs=` (12 sites), `(sys_id, port_id)` tuples (3 sites, one
test file), `method="fd"` defaults; `x_bar=` / `u_bar=` (92 sites) untouched.

## 8. Internal variables of a diagram

Both diagram evaluators already compute every wire into one signal buffer,
slice it by `(block, port)`, and expose it as `compute_internal_signals_dict`
keyed `"block:port"` — that string names the signal on the wire leaving
`block`'s output port, and it is the key `reconstruct_internal_signals`
writes into a `Trajectory`. So `"plant:y"` is already the library's name for
that wire; §2 reuses it.

- **`of="block:port"` is free.** The wire is a slice of the buffer,
  `_internal_signals_trace_fn` already traces on JAX, so it is exact where
  today's `(sys_id, port_id)` tuple is finite-difference only. Included in
  step 1.
- **`wrt="block:port"` is small.** An additive perturbation δ of that wire is
  one traced argument added inside `_compute_port_signals(_p)` on both
  backends (about ten lines each); `f`, `outputs` and the internal signals
  all read the perturbed buffer, so `loop.jacobian("plant:y", "ctl:u")` is
  the closed-loop sensitivity of the plant output to a disturbance at its
  input, and `loop.bode(wrt="ctl:u")` its frequency response. About 1.5 h
  with tests; ruling 9.
- **Loop opening is later.** Gain and phase margins need the loop transfer
  function at a wire: the same hook with *replace* instead of *add*, `of`
  read before the break, and one keyword (`open_at=`) on `linearize` /
  `bode`. The code is as small as the perturbation; the design question is
  what `f` sees at the break, so it stays a reserved name until a course
  needs it.
