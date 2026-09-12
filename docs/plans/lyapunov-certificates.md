# Lyapunov certificates: the region of attraction as an analysis tool (plan)

Status: **implemented** (2026-09-11), steps L1–L6; awaiting the rulings in §11.
Code: `minilink/analysis/lyapunov.py`, `tests/unittest/test_analysis_lyapunov.py`,
`examples/demos/analysis/analysis_region_of_attraction.py`.
Lane: candidate for the **analysis band** (teaching surface, GRO860 topic).

Related: [DESIGN.md](../../DESIGN.md) §5 (analysis), the derivatives facade
(landed 2026-09-06), [rl-planner-vision.md](rl-planner-vision.md) (the learned
laws this tool is asked about).

Evidence: `examples/experimental/rl/pendulum_rl_lyapunov_certificate.py`
(2026-09-11) does all of this by hand in 200 lines. This plan is the proposal
to turn those 200 lines into one verb.

---

## 0. What the prototype showed

1. **The question is universal.** "From where is this guaranteed to settle?"
   is asked of an LQR, a lookup table from value iteration, an impedance law,
   and a neural policy with exactly the same words. Only the analysis band can
   answer it for all four, because all four are a `System`.
2. **Every ingredient already exists.** `find_equilibrium`, `jacobian`, the
   compiled `f`, and `vmap` over a grid. The tool is assembly, not new math.
3. **The answer is three coupled quantities** — the equilibrium `x_bar`, the
   matrix `P`, and the level `c` — plus the system they belong to. That is a
   record object, like `TrajectoryPlan` or `MonteCarloReport`.
4. **The sublevel search is sampled, so the level is sharp but not rigorous.**
   In the prototype the same policy gave `c = 0.045`, `0.044`, `0.042` on three
   different search windows. A window-refinement pass fixes that instability;
   a *proof* needs interval bounds or sum-of-squares, which is a later step and
   must be named as such rather than implied.
5. **Simulation is the natural counter-check.** Sampling inside the certified
   set and integrating is a two-line falsification test that a student
   understands immediately, and it is how you catch a level search that was
   too coarse.
6. **The tool explains itself when the input is visible.** The prototype's
   certified set ended at a state where the policy commanded exactly its
   saturation torque. That is only legible because the controller is a block
   whose output you can ask for.

---

## 1. Target mental model

```text
any closed loop (a System)
        │
        ├── region_of_attraction(sys)  ──►  LyapunovCertificate
        │                                     V(x) = (x - x_bar)' P (x - x_bar)
        │                                     level c, equilibrium x_bar
        │                                     contains(x)
        │                                     verify(n)  → does simulation agree?
        │                                     plot()     → the certified slice
        │
        └── the same object for LQR, DP, MPC, impedance, or a neural policy
```

One concept (the region of attraction), one verb, one record, two methods on
the record. Nothing about *how* the loop was designed enters the call.

---

## 2. The API

### 2.1 The verb

```python
from minilink.analysis import region_of_attraction

roa = region_of_attraction(sys, x_bar=None, u_bar=None, t=0.0, params=None)
```

Signature matching the rest of the analysis family (`linearize`, `bode`,
`pzmap`): positional operating point, `x_bar=None` meaning "find it", then
keyword-only options.

```python
def region_of_attraction(
    sys,
    x_bar=None,          # None: find_equilibrium from sys.x0
    u_bar=None,
    t=0.0,
    params=None,
    *,
    method="quadratic",  # the certificate family; "sos" reserved (§8)
    Q=None,              # Lyapunov right-hand side, default the identity
    window=None,         # search box; default two passes (§4.2)
    samples=None,        # budget per pass; 201**2 in 2-D, 50_000 above
    search="auto",       # "grid" | "random" (§4.1)
) -> LyapunovCertificate: ...
```

### 2.2 The record

```python
@dataclass
class LyapunovCertificate:
    """Lyapunov function and the level that certifies a region."""

    sys: System
    x_bar: np.ndarray     # found, not assumed
    P: np.ndarray
    level: float          # the certified c in {V <= c}
    Q: np.ndarray
    poles: np.ndarray     # of the linearization, kept as evidence
    window: BoxSet
    limiting_state: np.ndarray   # the sampled state that pinned the level
    method: str = "quadratic"

    def V(self, x): ...          # (x - x_bar)' P (x - x_bar), one state or many
    def V_dot(self, x): ...      # grad V . f(x), along the *nonlinear* loop
    def contains(self, x): ...   # V(x) <= level
    def verify(self, n=200, tf=None, tol=None) -> VerificationReport: ...
    def plot(self, x_axis=0, y_axis=1, basin=False, show=True, ax=None): ...
    def __str__(self): ...       # one line, like MonteCarloReport
```

`__str__` is the teaching surface's first impression:

```text
certified V <= 0.042 about [3.218, 0.000]: |dx| up to [0.194, 0.541], poles -2.03+-0.64j
```

### 2.3 System shortcuts

Per the "systems are descriptions, facades are shortcuts" rule, two thin
delegates on `DynamicSystemFacades` in `core/facades.py`, beside `bode` and
`plot_bode`:

```python
roa = cl_sys.region_of_attraction()
cl_sys.plot_region_of_attraction(x_axis=0, y_axis=1, basin=True)
```

### 2.4 The whole teaching story, in five lines

```python
lqr_loop = lqr_at_operating_point(plant, X_UP, Q, R) @ plant
rl_loop = planner.get_controller() @ plant

print(lqr_loop.region_of_attraction())      # what the classical design guarantees
print(rl_loop.region_of_attraction())       # what the learned law guarantees
rl_loop.region_of_attraction().verify()     # simulation tries to break it
rl_loop.plot_region_of_attraction(basin=True)
```

---

## 3. The math, in the order the module reads

Lyapunov's indirect method, then one nonlinear sweep. Written so the file is
readable top to bottom by a student who has seen the lecture.

1. **Equilibrium.** `x_bar` given, or `find_equilibrium(sys, sys.x0, u_bar, t)`.
2. **Linearization.** `A = jacobian(sys, "f", "x", x_bar, u_bar, t)`.
3. **Stability check.** `A` must be Hurwitz. Otherwise raise with the offending
   pole: *"the loop is unstable at this equilibrium (pole at +1.48); there is
   no region of attraction to certify"*.
4. **Lyapunov equation.** `A' P + P A = -Q` by
   `scipy.linalg.solve_continuous_lyapunov`, `Q = I` by default. `P` is
   positive definite exactly because `A` is Hurwitz, so `V` is a valid
   candidate.
5. **The candidate on the true dynamics.**
   `V(x) = (x - x_bar)' P (x - x_bar)` and
   `V_dot(x) = 2 (x - x_bar)' P f(x, u_bar, t)`, with `f` the **nonlinear**
   closed loop, saturation and network and all.
6. **The level.** The certified region is the largest sublevel set that stays
   inside the region where `V` decreases *and* inside the model's own domain:

   ```text
   c = min { V(x) : V_dot(x) >= 0  or  x not in X },  x sampled, x != x_bar
   ```

   Points outside the state set `X` cap the level for the same reason as
   points where `V` stops decreasing: past them the certificate would be
   claiming something about states the model does not describe.

---

## 4. The level search: the one real design choice

### 4.1 How to sample

| method | when | cost | note |
| --- | --- | --- | --- |
| `search="grid"` | `n <= 2` (default there) | `samples` cells | the plot gets its contours for free |
| `search="random"` | `n >= 3` (default there) | `samples` points | quasi-random (Halton) beats uniform for coverage |

Both are one `vmap` of `V_dot` on the compiled JAX evaluator, with a NumPy
loop fallback for plants that do not trace. A third option, bisecting along
rays from `x_bar`, is more sample-efficient in high `n`; it is a later
refinement, not a first cut.

### 4.2 The window, and why it needs a second pass

The estimate depends on where you look: too wide and the samples are coarse
where it matters, too narrow and the level is capped by the window edge
rather than by the dynamics. `window="auto"` runs the search twice:

1. a first pass over the state bounds (or `x_bar` plus ten times the
   linearization's natural scale when the bounds are infinite);
2. a second pass over a box `1.5x` the extent of the first level set.

The second pass is where the reported level comes from. This removes the
prototype's window sensitivity at the cost of one extra vmapped sweep.

### 4.3 Say what kind of answer this is

The docstring states it plainly: the level is a **sharp estimate from
samples**, optimistic by construction (an unsampled bad state would lower
it), and `verify()` is the counter-check. A rigorous version bounds `V_dot`
between samples; see §8.

---

## 5. Verification: simulation against the theory

Two different Monte Carlo questions, kept separate because they mean
different things.

**`verify(n=200, tf=None, tol=None)` — is the theory right?**
Sample `n` states *uniformly inside the certified ellipsoid*: with the
Cholesky factor `P = L L'`, `x = x_bar + sqrt(c) L^-T z` for `z` uniform in
the unit ball. Integrate each for `tf` and check it ends within `tol` of
`x_bar`. Every sample must converge; the report names the first that does
not.

```python
@dataclass
class VerificationReport:
    converged: np.ndarray      # bool per trial
    final_distance: np.ndarray
    counterexample: np.ndarray | None

    @property
    def holds(self) -> bool: ...        # all converged
    def __str__(self) -> str: ...       # "200/200 trials converged, worst |x - x_bar| = 3e-4"
```

Defaults that come free from work already done: `tf = 10 / |max Re(poles)|`
(the slowest mode of the linearization) and `tol = 1e-2 * sqrt(c / min eig P)`
(a hundredth of the certified extent). A failure is informative: either the
level search was too coarse, or the model is not what the user thinks.

**`plot(basin=True)` — how conservative is it?**
Simulate from every cell of the plotted slice and shade the ones that
converge. This answers a different question (the true basin) and is a
plotting aid, not part of the certificate. In the prototype the certified set
held 6% of the simulated basin: the honest price of a quadratic `V`, and a
good lecture slide.

---

## 6. Plotting

`plot(x_axis=0, y_axis=1, ...)`, naming borrowed from `plot_control_law` so
the two teaching plots are called the same way. For `n > 2` the other
coordinates are pinned at `x_bar`, and the title says so.

Layers, in drawing order:

| layer | default | what it shows |
| --- | --- | --- |
| simulated basin | `basin=False` | shaded, the states that really converge |
| `V = c` contour | always | the certificate |
| `V_dot = 0` contour | `detail=True` | what pins the level, tangent to the above |
| limiting state | `detail=True` | the sampled state that set `c` |
| equilibrium | always | a star at `x_bar` |
| trajectories | `trajectories=(x0, ...)` | optional overlays |

Matplotlib only in the first cut, like `plot_pzmap`; the `backend="plotly"`
option of the frequency tools can follow if wanted.

---

## 7. Placement and dependency law

| piece | home |
| --- | --- |
| `region_of_attraction`, `LyapunovCertificate`, `VerificationReport`, `plot_region_of_attraction` | `minilink/analysis/lyapunov.py` |
| facade exports | `minilink/analysis/__init__.py` |
| `sys.region_of_attraction`, `sys.plot_region_of_attraction` | `minilink/core/facades.py` (`DynamicSystemFacades`) |

Imports: `core.backends`, `analysis.equilibria`, `analysis.derivatives`,
`core.sets`, `scipy.linalg`. Nothing from `planning/` or `control/` — the
tool must not know that a policy came from RL. Module sections follow the
house order: the record first (it is the contract the reader wants), then
the verb, then the search helpers under `# Internal machinery`. A ten-line
`__main__` certifies a pendulum under LQR.

---

## 8. What this is, and what it is not

**Is:** a quadratic Lyapunov function from the linearization, a sampled
sublevel search on the true nonlinear closed loop, and a simulation
counter-check. Continuous time, autonomous loops (no live input), local.

**Is not, and should say so:** a proof; a global result; a non-quadratic or
learned `V`; a discrete-time certificate. Each is a clean later step:

| later | what changes |
| --- | --- |
| rigorous level | bound `V_dot` between samples (interval arithmetic) or SOS on a polynomial surrogate |
| discrete time | `A' P A - P = -Q` and the test `V(x_next) - V(x) < 0`; the rest is identical |
| better `V` | accept a user `P`, or the LQR Riccati `P`, which usually certifies more |
| bigger region | maximize the level over `Q` (a small outer optimization) |

---

## 9. Teaching entry

**The demo** (`examples/demos/analysis/analysis_region_of_attraction.py`, as
built) is 35 lines with no matplotlib in it at all: a torque-limited
pendulum, one LQR, the actuator limit as a `Saturation` block, then
`loop.region_of_attraction(X_UP)`, `verify()` and `plot(basin=True)`. `Q` sits at the top with a comment inviting the reader to
try `diag([1000, 10])`, which places faster poles and proves a region three
times smaller while the basin simulation finds stays the same size — the
lesson arrives by changing one line rather than by running two designs at
once. If a demo of this tool ever needs custom plotting again, the module
is missing a feature.

**The showcase** (`examples/tutorial/showcase_from_rl_to_bode.ipynb`
§11) puts the same verb on the twelve-state UR5 loop with the trained
network inside it, right after the Bode section: frequency response says how
the loop *responds*, the certificate says from how far it *recovers*. It
prints the certificate and its verification, then plots one slice. It is
also where the sampling limit shows, and the printed line says so itself.

A second, canonical validation case belongs in the tests rather than the
demo: the **reverse-time Van der Pol** oscillator, whose region of attraction
is bounded by the unstable limit cycle. It is the textbook example, the
catalog already has `VanderPol`, and the certified ellipse should sit inside
that cycle with `verify()` passing.

---

## 10. Phased delivery

| step | deliverable | done when |
| --- | --- | --- |
| **L0** | this plan reviewed; decisions D1–D4 ruled | review queue cleared |
| **L1** | `LyapunovCertificate`, `region_of_attraction`, grid search, `contains`, `__str__` | certifies a pendulum under LQR; unstable equilibrium raises |
| **L2** | `verify()` + `VerificationReport` | reverse-time Van der Pol: certified set inside the limit cycle, 200/200 trials converge |
| **L3** | `plot()` with the basin and detail layers; `plot_region_of_attraction` | one figure, both backends of the plant (NumPy plant certified too) |
| **L4** | the `"random"` method and the auto window's second pass | a 4-state loop (cart-pole under LQR) certified in seconds, level stable across two window seeds |
| **L5** | `System` shortcuts; facade exports; DESIGN §5 paragraph; TRL row | `cl_sys.region_of_attraction()` in the teaching surface test |
| **L6** | the three-controller demo; notebook cell in `04_analysis` if the maintainer wants it in the course | notebook smoke |

Tests to write with L1–L3: a linear system (level limited only by the window,
`verify` passes), the saturated pendulum (certified set is a subset of the
simulated basin), an unstable equilibrium (raises), a NumPy-only plant (same
answer through the fallback), and the Van der Pol case above.

---

## 10a. Any system, not only the traceable ones

Verified 2026-09-11: the tool works on every `DynamicSystem`, because both
steps already fall back. `jacobian(..., method="auto")` is exact under JAX and
finite differences otherwise, and `compile_auto` gives a vmapped sweep under
JAX and a Python loop on NumPy. Measured on the pendulum loop at the 2-D
default of `201**2` samples: 0.29 s traced, 1.0 s on the NumPy fallback. A
plant with a Python branch on the state (which cannot trace) is certified and
verified in 0.1 s, and `test_a_plant_that_cannot_trace_is_certified_on_numpy`
holds that path.

## 10c. Dimension is the real limit, and the tool says so

Measured 2026-09-11 on the UR5 loop (`n = 12`): the level fell from 5.22 to
2.78 when the sample budget went from 50 000 to 400 000, so it had not
converged — the curse of dimensionality, exactly where a sampled sublevel
search is expected to lose. `verify()` does not catch it either: 200 draws
inside a twelve-dimensional ellipsoid all converge.

So the search now scores itself, at no extra cost in dynamics evaluations:
it computes the level twice more, on two disjoint halves of the samples it
already drew, and reports their relative disagreement as
`sample_spread`. The measurement discriminates cleanly — 0.000 on the
two-state pendulum grid, 0.36 to 0.45 on the UR5 — and
`sample_limited` (`spread > 0.1`) appends a phrase to `__str__` so a printed
certificate never looks sharper than it is. This is the honest ceiling of
the quadratic method; a rigorous level needs §8's interval bounds or SOS.

## 10b. What the build changed from this draft

- `method=` now names the **certificate family** (`"quadratic"`, `"sos"` reserved
  and raising), and the sampling knob became `search="auto" | "grid" | "random"`.
  The repo's convention is that `method` is the mathematical approach.
- The equilibrium is always refined by `find_equilibrium`, so a supplied
  `x_bar` is a guess. That is what surfaces a learned law's steady offset.
- `V̇` is computed in closed form, `2 (x - x_bar)ᵀ P f(x)`, not by autodiff of
  `V`: the gradient of a quadratic is exact and the tool then needs only `f`,
  so it works identically on the NumPy and JAX backends.
- The record carries the compiled evaluator, so one compile serves the level
  search, `verify`, and `plot`.
- The level search reports `sample_spread` / `sample_limited` (§10c).
- `plot` carries the whole figure so demos need no matplotlib: a legend for
  every layer it drew, a title from the system's own name (with the slice and
  sample-limited notes), `verified=N` overlaying the states
  :meth:`verify` tests coloured by whether they converged, and `limits=` to
  override the window. The legend names each layer by what backs it — theory
  for the certified level set, simulation for the basin, the verification
  draw for the dots — since three layers about convergence in one figure are
  otherwise easy to confuse. Two rules keep it honest on a slice: the level's
  limiting state is marked only when it lies on the plane, and the
  `V̇ = 0` curve is drawn only where `V̇` is genuinely positive — it vanishes
  at the equilibrium to within rounding, and that speck is not a curve.
- `slice_extent(x_axis, y_axis)` was added and the plot window follows it.
  For `n > 2` the *slice* of the certified set through the equilibrium is
  smaller than `extent`, which is the shadow the set casts on each axis: on
  the cart-pole's `(x, theta)` plane the shadow is 2.7x the slice in the pole
  angle, so a window sized by `extent` would draw a squashed ellipse in a
  mostly empty frame. On a slice the level set also need not touch the
  `V̇ = 0` curve, and the limiting-state marker is drawn only when that state
  lies on the plane.

## 11. Open decisions

- **D1 — naming.** `region_of_attraction(...)` returning `LyapunovCertificate`
  (recommended: the concept is the verb, the method is the record), against
  `lyapunov_certificate(...)` returning `RegionOfAttraction`, or a single
  name for both.
- **D2 — does the state set cap the level?** Recommended yes (§3.6): a
  certificate should not describe states the model excludes. The alternative
  is to certify the dynamics alone and let the user intersect.
- **D3 — `verify()` on the record, or a free `verify_certificate(roa)`?**
  Recommended on the record: it needs `V`, `c`, `x_bar` and the system, and
  `roa.verify()` reads like the sentence it implements.
- **D4 — discrete time now or later?** Recommended later (§8); the seam is
  one branch in the decrease test, but `StepSystem` is provisional and this
  tool should land on the continuous core first.
- **D6 — the `method` name against the band's calling pattern.** DESIGN §3
  says every analysis tool reads `tool(<what>, x_bar, u_bar, t, params, *,
  method="auto", eps)`, where `method` picks exact-versus-finite-difference
  derivatives. Here `method` names the certificate family and the derivative
  knob is not exposed at all (the Jacobian always runs `method="auto"`, which
  is exact under JAX and finite differences otherwise, and the level sweep
  follows the same fallback). Options: keep it and accept the deviation
  (recommended, since the family is the choice a user actually makes), rename
  the family to `family=`, or expose `jacobian_method=` / `eps=` beside it.
  Only a badly scaled non-traceable plant would want that last knob today.
- **D5 — course placement.** Whether this joins the GRO860 checklist as a
  row of its own, or stays an analysis tool the lecture happens to use.
