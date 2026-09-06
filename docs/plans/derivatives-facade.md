# Partial derivatives on every `System` — plan (draft, 2026-09-06)

Status: **draft for the maintainer's rulings** (naming and defaults below).
Lane: teaching surface (core facade + `analysis/`). Nothing implemented yet.

## 1. What a student writes

```python
from minilink import Pendulum

plant = Pendulum()

A = plant.df_dx()                    # ∂f/∂x at the nominal point (x0, ū, t = 0)
B = plant.df_du()                    # ∂f/∂u
C = plant.dh_dx()                    # ∂h/∂x  — the output map y = h(x, u, t)
D = plant.dh_du()                    # ∂h/∂u

A = plant.df_dx(x_bar, u_bar)        # any operating point
S = plant.df_dp(x_bar, u_bar)        # ∂f/∂params → {"m": (n,), "l": (n,), ...}
plant.df_dt(x_bar, u_bar, t=2.0)     # ∂f/∂t, time-varying plants only

loop = controller @ plant
A_cl = loop.df_dx()                  # closed loop: same call, poles of the loop
```

Every call returns a NumPy array (or a dict of arrays for `_dp`), whatever
the backend. Shapes follow the textbook: `df_dx` is `(n, n)`, `df_du` is
`(n, m)`, `dh_dx` is `(p, n)`, `dh_du` is `(p, m)`, `df_dt` is `(n,)`, and
`df_dp[name]` is `(n, *shape(param))`.

The four matrices are the linearization, so `plant.linearize(x_bar, u_bar)`
becomes a method returning the `LTISystem` built from them; the module
function `linearize(sys, ...)` stays as the functional spelling.

Optional keywords, all with textbook defaults:

| keyword | default | meaning |
| --- | --- | --- |
| `x`, `u`, `t` | `x0`, nominal `u`, `0.0` | operating point |
| `params` | the system's | parameter set the derivative is taken at |
| `method` | `"auto"` | `"jax"` exact when JAX is installed and the plant traces, else central finite differences; `"fd"` / `"jax"` force one |
| `eps` | `1e-6` | central-difference step |

`method="auto"` falls back silently (the student asked for a derivative, not
for a backend); `method="jax"` raises when the plant does not trace, like
`Sys2Gym(compile_backend="jax")`.

## 2. Why the evaluator is the wrong tier for this

Today the derivative story in student material is three different things:
`linearize_matrices(sys, x, u, method=...)` returning `A, B, C, D` at once,
hand-rolled `jax.jacfwd(lambda x: evaluator.f(x, u, t))(x)` in the showcases,
and `evaluator.jacobian_f_params(...)` for parameters. All three expose the
compile step and the backend to someone who only wants ∂f/∂x. The evaluator
tiers (`f`, `f_p`, `f_trace`, `jacobian_f_params`) stay as the research lane;
the facade is the only spelling that appears in `learn/` and `demos/`.

## 3. Design

**One engine, `minilink/analysis/derivatives.py`**

```python
def jacobian(sys, of, wrt, x=None, u=None, t=0.0, *, params=None,
             method="auto", eps=1e-6):
    """∂of/∂wrt as a NumPy array; of ∈ {"f", "h", "step"}, wrt ∈ {"x", "u", "t", "params"}."""
```

- Operating point: `x` → `sys.x0`, `u` → `sys.get_u_from_input_ports()`,
  `params` → `sys.params`, coerced to float arrays.
- Backend: compile once per call — `sys.compile("jax")` under `"jax"` /
  `"auto"` when it succeeds, `sys.compile("numpy")` otherwise; the `"auto"`
  fallback catches only the "not JAX-traceable" verdict (same rule as
  `Sys2Gym`).
- Exact path: `jax.jacfwd` over `evaluator.f` / `evaluator.outputs["y"]` /
  `evaluator.step` with the other arguments closed over; params through
  `f_p` / `outputs_p` with `jax.jacfwd` on the pytree, giving the dict.
- Finite-difference path: the central-difference loop that `linearize_matrices`
  already carries, moved here and applied to the same closures; params
  perturbed element by element (nested dicts for diagrams).
- `linearize_matrices` keeps its `inputs=` / `outputs=` port selection (research
  use) but calls this engine for the arithmetic — one implementation of FD and
  of the JAX Jacobian instead of two.

**Facade methods, `minilink/core/facades.py`** — each a two-line delegation,
the pattern `modal_analysis` and `plot_bode` already follow:

| Mixin | Methods |
| --- | --- |
| `SharedSystemFacades` (every `System`, static blocks included) | `dh_dx`, `dh_du`, `dh_dt`, `dh_dp`, `linearize` |
| `DynamicSystemFacades` (continuous leaves and diagrams) | `df_dx`, `df_du`, `df_dt`, `df_dp` |
| `StepSystemFacades` (discrete leaves and step diagrams) | `dstep_dx`, `dstep_du`, `dstep_dp` — the Jacobians of `x_{k+1} = step(x_k, u_k)` |

A static block asked for `df_dx` gets `AttributeError` with the sentence
"static system: no f; use dh_dx / dh_du" (the method is simply not on the
static mixin). A diagram's `df_dx` is the stacked closed-loop Jacobian; internal
ports stay with `linearize_matrices(outputs=(sys_id, port))`.

**Optional: evaluator cache on the system.** A JAX `df_dx()` inside a loop
would re-jit on every call (0.3–1 s). Keep the compiled evaluator on the
system keyed by backend and clear it in `refresh()` (the hook the simulator
already calls when structure changes); params changes need no recompile
because the parametric tier takes `params` at call time. Decide in §5.

## 4. Steps (about five agent-hours)

1. **Engine + consolidation** (1.5 h): `analysis/derivatives.py::jacobian`;
   `linearize_matrices` calls it; tests — FD vs JAX agreement on ten catalog
   plants (reuse the `test_catalog_backends` parametrization), the diagram
   closed loop equals `linearize(...).A()`, `df_dp` dict shapes for a leaf and a
   nested diagram, `df_dt` on a time-varying block, error messages.
2. **Facade** (1 h): the methods above with NumPy docstrings; `sys.linearize`;
   DESIGN §"Tools" row and §4 `System` contract line; README analysis snippet;
   teaching-surface registry test extended with the method names.
3. **Edge cases** (1 h): static blocks, `StepSystem` / `StepDiagramSystem`,
   the JAX-unavailable subprocess probe (Basic tier), strict `method="jax"`
   error, `eps` plumbing.
4. **Student material** (1 h, `[ask]`): `04_analysis.ipynb` uses `plant.df_dx()`
   and `plant.linearize()`; `showcase_jax.ipynb` §derivatives drops the
   hand-rolled `jacfwd` cells for `df_dx` / `df_du` / `df_dp`; `demos/analysis/
   analysis_linearize_fd_vs_jax.py` becomes an eight-line `method="fd"` vs
   `"jax"` compare; `demos/compile/params_gradient.py` part 1 uses `df_dp`;
   `03_control` shows `A, B = plant.df_dx(), plant.df_du()` before LQR.
   Notebook smoke + demo sweep.
5. **Optional cache** (0.5 h) per §3.

Out of scope, reserved names: second derivatives (`d2f_dx2`, Hessians of a
cost), Jacobians along a whole `Trajectory` (a `Trajectory.linearize()` for
time-varying LQR), sensitivities of a simulation with respect to params
(`rollout_batch` + `jax.grad` remain the research spelling).

## 5. Rulings needed

1. **Names.** `df_dx / df_du / df_dt / df_dp` and `dh_dx / dh_du / dh_dt / dh_dp`
   (recommended: they mirror the `def f` / `def h` a student writes), or
   `dy_dx / dy_du …` for the output map. `df_dp` vs `df_dparams`.
2. **`_dp` result.** Dict keyed by parameter name (recommended, same layout as
   `params`) or one stacked matrix with a label list.
3. **Defaults.** Nominal point `(x0, ū, 0)` when no arguments are given
   (recommended) or required `x`, `u`.
4. **Fallback noise.** `method="auto"` silent (recommended) or a warning when
   JAX is asked for and FD is used (what `linearize_matrices` does today).
5. **`sys.linearize()` as a method** alongside the functional `linearize(sys)`
   (recommended).
6. **Step systems.** `dstep_dx` (recommended) or `dx_next_dx`.
7. **Evaluator cache** on the system with `refresh()` invalidation: go or
   compile per call.
