# Inverse kinematics on `Manipulator` — proposal

Status: **implemented** on `Manipulator.inverse_kinematics` (inline in
[`manipulator.py`](../../minilink/dynamics/abstraction/manipulator.py)).

Context: task-space demos (`TaskImpedance`, future trajectory refs) often need a
joint configuration `q` that reaches a desired end-effector pose `p_d`. Today
catalog arms expose `forward_kinematics(q)` and `J(q)` only; callers must pick
`q` by hand. A default numerical IK on the `Manipulator` base closes that gap
without requiring closed-form IK per arm.

Related: [`manipulator-abstraction.md`](manipulator-abstraction.md),
[`robot-control-stack.md`](robot-control-stack.md).

---

## Goal

Add a **default numerical IK** on :class:`~minilink.dynamics.abstraction.manipulator.Manipulator`:

```text
Given p_d, find q  such that  f(q) ≈ p_d
```

implemented as root-finding / least-squares on the existing FK hook
(`forward_kinematics`), using SciPy — same spirit as
:func:`~minilink.analysis.equilibria.find_equilibrium` on `f(x,u)=0`.

**Not in v1:** exhaustive multi-branch IK, redundancy resolution optimality,
JAX-traced IK, or analytical IK overrides per catalog arm (optional later).

---

## Problem shapes

| Case | Condition | Solver | Residual |
| --- | --- | --- | --- |
| **Square** | `task_dim == dof` | `scipy.optimize.fsolve` | `f(q) − p_d` |
| **Redundant** | `dof > task_dim` | `scipy.optimize.least_squares` | `f(q) − p_d` (min ‖·‖²) |
| **Overconstrained** | `task_dim > dof` | `least_squares` | same (best-fit only) |

`method="auto"` (default): pick `fsolve` when square, else `least_squares`.

**Multiple solutions:** numerical IK returns **one** root near `q_guess`
(elbow-up vs elbow-down, etc.). Document clearly; branch enumeration is out of
scope for v1.

---

## API (proposed)

**Location:** default method on `Manipulator` in
[`minilink/dynamics/abstraction/manipulator.py`](../../minilink/dynamics/abstraction/manipulator.py).
Optional thin helper in a sibling module if the implementation grows (see
§File layout).

```python
def inverse_kinematics(
    self,
    p_d,
    q_guess=None,
    *,
    params=None,
    tol=1e-9,
    bounds=None,
    method="auto",
):
    """Return joint ``q`` with ``forward_kinematics(q) ≈ p_d``.

    Parameters
    ----------
    p_d : array, shape (task_dim,)
        Desired task-space position ``p_d``.
    q_guess : array, shape (dof,), optional
        Initial guess (selects local IK branch). Default: nominal ``q`` output.
    params : dict, optional
        Passed to ``forward_kinematics`` / ``J`` if needed.
    tol : float
        Success tolerance on ``‖f(q) − p_d‖``.
    bounds : (lower, upper) or None
        Per-joint limits, shape ``(dof,)`` each. Default: ``state.lower_bound``
        / ``state.upper_bound`` on the first ``dof`` state entries when finite.
    method : str
        ``"auto"`` | ``"fsolve"`` | ``"least_squares"``.

    Returns
    -------
    q : np.ndarray, shape (dof,)

    Raises
    ------
    RuntimeError
        If the solver does not meet ``tol``.
    ValueError
        On shape / method errors.
    """
```

**Naming:** `inverse_kinematics` mirrors `forward_kinematics`; no port named `ik`
on the plant (IK is an offline / reference-generation tool, not a diagram signal).

**Optional v2:** return a small result dataclass `{q, success, residual_norm, nfev}`
instead of raising — defer unless callers need soft failure.

---

## Implementation sketch

### Residual

```python
def _ik_residual(q, manipulator, p_d, params):
    p = np.asarray(manipulator.forward_kinematics(q, params), dtype=float)
    return p - np.asarray(p_d, dtype=float).reshape(-1)
```

Use NumPy arrays inside the SciPy loop (FK on catalog arms is NumPy-native;
JAX plants are out of scope for this solver path).

### Square case (`fsolve`)

Follow [`equilibria.py`](../../minilink/analysis/equilibria.py):

```python
from scipy.optimize import fsolve

q_sol = fsolve(lambda q: _ik_residual(q, self, p_d, params), q_guess, xtol=tol)
if np.linalg.norm(_ik_residual(q_sol, ...)) > tol * 1e3:
    raise RuntimeError("inverse_kinematics did not converge")
```

### Redundant / overconstrained (`least_squares`)

```python
from scipy.optimize import least_squares

result = least_squares(
    lambda q: _ik_residual(q, self, p_d, params),
    q_guess,
    bounds=(lower, upper) if bounds else (-np.inf, np.inf),
    ftol=tol, xtol=tol,
)
if not result.success or result.cost > tol**2:
    raise RuntimeError(...)
return result.x
```

### Joint bounds

Default from `self.state.lower_bound[:dof]` / `upper_bound[:dof]` when finite;
`fsolve` has no native bounds — for bounded square systems, use
`least_squares` with `method="trf"` even when square, or document that square
case ignores bounds unless `method="least_squares"`.

**Recommendation:** when any bound is finite, route to `least_squares` regardless
of dimensions.

### Jacobian (optional enhancement)

Pass `jac=lambda q: self.J(q, params)` to `least_squares` for faster convergence
when `J` is available. Not required for v1 MVP.

---

## File layout

| Option | Verdict |
| --- | --- |
| Inline in `manipulator.py` | OK if ≤ ~40 lines |
| `dynamics/kinematics/inverse.py` + one-line delegate on `Manipulator` | **Preferred** if helper grows — keeps abstraction file readable ([agent.md § module order](../../agent.md)) |

Public import path: only from `Manipulator` method (no barrel re-export until API
freeze).

---

## What stays unchanged

- **`MechanicalSystem`** — no IK (no FK task map).
- **Catalog arms** — no per-class IK in v1; inherit default numerical IK.
- **Future:** `TwoLinkManipulator.inverse_kinematics` analytic override for speed
  and explicit elbow branch — optional, must call same signature.

---

## Tests

Add `tests/unittest/test_inverse_kinematics.py`:

| Test | Assert |
| --- | --- |
| `OneLinkManipulator` roundtrip | `FK(IK(p)) ≈ p` for several `q`, use that `q` as `q_guess` |
| `TwoLinkManipulator` roundtrip | same; perturb `q_guess` slightly |
| Wrong branch | different `q_guess` → different valid `q` with same `p` (if reachable) |
| Unreachable / bad guess | `RuntimeError` or high residual (document behavior) |
| `task_dim < dof` | 5-link planar: least-squares best fit, FK close to `p_d` |
| Bounds | solution respects joint limits when enabled |

Run in **`minilink`** conda env.

---

## Demo / docs (post-implementation)

- Short example in [`manipulator-abstraction.md`](manipulator-abstraction.md) §IK.
- Optional robotic demo helper: generate joint ref from task `Step` via
  `arm.inverse_kinematics(p_d, q0)` before `TaskImpedance` — not required for
  IK PR itself.

---

## Out of scope (v1)

- Differential IK velocity map (`dq = J⁺ ṗ_des`)
- Orientation IK (task_dim 6, SE(3))
- CyIPOPT / `MathematicalProgram` formulation (overkill for basic FK inversion)
- Caching / warm-start across trajectory samples

---

## Execution checklist

1. Add `dynamics/kinematics/inverse.py` (or method body in `manipulator.py`).
2. Wire `Manipulator.inverse_kinematics(...)`.
3. Tests on `OneLinkManipulator`, `TwoLinkManipulator`, `FiveLinkPlanarManipulator`.
4. One-line cross-ref in `manipulator-abstraction.md`.
5. Grep gate: no Pyro port blockers unless adding to `pyro-port-remaining.md`.

**Estimated scope:** ~1 new module + ~30 lines on `Manipulator` + ~1 test file.

---

## Decision log

| Topic | Decision |
| --- | --- |
| Base class vs catalog | Default on `Manipulator` |
| Solver | SciPy `fsolve` / `least_squares` (already in env) |
| Default off? | N/A — IK is explicit method call, not a controller option |
| Bounds | Optional; default from `state` limits when finite |
| Multi-solution | Single root near `q_guess`; document limitation |
| JAX | Not in v1 |
