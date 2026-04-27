# Port CartPole and DoublePendulum from pyro into minilink

> **Overview.** Port pyro's `CartPole` and `DoublePendulum` into minilink as `MechanicalSystem` subclasses, translating pyro's `forward_kinematic_lines` visualization to minilink's `get_kinematic_geometry()` + `get_kinematic_transforms()` paired API. Organize them under `minilink/mechanics/` so concrete mechanical systems live next to their base class, mirroring pyro's `dynamic/` layout.

## Scope (confirmed)

- `CartPole` — the linear cart with 1D pole from `pyro/dynamic/cartpole.py`
- `DoublePendulum` — from `pyro/dynamic/pendulum.py`

Related pyro variants (`RotatingCartPole`, `Acrobot`, `InvertedPendulum`, …) are **out of scope** for this port; they can be added later following the same pattern.

## Organization recommendation for the minilink mechanical-systems library

**Proposed convention: put concrete mechanical-system classes in `minilink/mechanics/` alongside the base class.**

Rationale:

- `minilink/mechanics/mechanical.py` already defines the base `MechanicalSystem` (and `JaxMechanicalSystem`). Concrete subclasses that override `H`, `C`, `B`, `g`, `d` belong right next to it — exactly how pyro keeps `pyro/dynamic/{pendulum,cartpole,manipulator,...}.py` next to `pyro/dynamic/mechanical.py`.
- `minilink/core/blocks/` is documented in `DESIGN.md` as diagram primitives (TRL 0). It's the right home for sources, controllers, and small reusable I/O blocks, not for full mechanical models.
- Keep imports explicit, e.g. `from minilink.mechanics.<defining_module> import CartPole, DoublePendulum` (see `agent.md` for package-`__init__` policy).
- `blocks/examples.py::Pendulum` and `blocks/dynamic_bicycle.py` predate this convention; leaving them where they are is fine for now. New mechanical systems should default to `mechanics/`.

Proposed future layout (only the two new files land in this PR):

```text
minilink/mechanics/
├── mechanical.py          # existing — base class
├── cartpole.py            # NEW — CartPole
├── pendulum.py            # NEW — DoublePendulum (named like pyro's file; single pendulum can move in later)
├── symbolic/              # existing
└── __init__.py            # extend __all__ with new classes
```

## Implementation outline

### 1. `minilink/mechanics/cartpole.py` (new)

- Class `CartPole(MechanicalSystem)`, `dof=2, actuators=1`.
- `params = {"l": 3.0, "lcg": 0.5, "m1": 1.0, "m2": 0.1, "gravity": 9.81}` so `f`/`h` stay functional in the `params` argument style used elsewhere in minilink.
- State labels/units: `["x", "theta", "dx", "dtheta"]` with `["m", "rad", "m/s", "rad/s"]`; input `"u"` renamed label `"F"` unit `"[N]"`, bounds `±10`.
- Override `H(q)`, `C(q, dq)`, `B(q)`, `g(q)`, `d(q, dq)` matching pyro equations exactly:

```python
H[0, 0] = m1 + m2
H[0, 1] = H[1, 0] = m2 * lcg * cos(theta)
H[1, 1] = m2 * lcg**2
C[0, 1] = -m2 * lcg * sin(theta) * dtheta
B = [[1], [0]]
g[1] = m2 * gravity * lcg * sin(theta)
```

- `get_kinematic_geometry()`: `[CustomLine ground, CustomLine cart, Point wheel_l, Point wheel_r, Rod pole, Arrow force]` — keep it 2D (pyro CartPole is not 3D).
- `get_kinematic_transforms(x, u, t)`: translate cart + wheels by `x[0]`; rotate `Rod` by `theta`; scale `Arrow` proportional to `F` with `scale_pose2d_matrix` (same pattern as `blocks/examples.py::FloatingMass1D`).

### 2. `minilink/mechanics/pendulum.py` (new)

- Class `DoublePendulum(MechanicalSystem)`, `dof=2, actuators=2`.
- `params = {"l1": 1.0, "l2": 1.0, "lc1": 1.0, "lc2": 1.0, "m1": 1.0, "m2": 1.0, "I1": 0.0, "I2": 0.0, "gravity": 9.81, "d1": 0.0, "d2": 0.0}`.
- State labels `["theta1", "theta2", "dtheta1", "dtheta2"]`; default input labels `["Torque 0", "Torque 1"]` (inherited from base) and bounds `±5` (inherited) — match pyro's defaults.
- Override `H`, `C`, `B`, `g`, `d` exactly as in pyro's `DoublePendulum` (sign conventions kept verbatim so trajectories match pyro).
- `get_kinematic_geometry()`: ground line, two `Rod`s, two `Circle` bobs, two `TorqueArrow`s at each joint (mirrors `blocks/examples.py::Pendulum`).
- `get_kinematic_transforms(x, u, t)`: pose each rod and bob from `q[0]`, `q[0]+q[1]` and link length; two `torque_pose2d_matrix` calls for the per-joint torques.

### 3. `minilink/mechanics/__init__.py`

Extend exports:

```python
from minilink.mechanics.cartpole import CartPole
from minilink.mechanics.pendulum import DoublePendulum
__all__ = ["MechanicalSystem", "JaxMechanicalSystem", "CartPole", "DoublePendulum"]
```

### 4. Docs

- `DESIGN.md` §2: note that `mechanics/` now also hosts concrete mechanical systems (one-line update to the mechanics row) — no contract change.
- No `ROADMAP.md` change required (still TRL 1 mechanics).

## Verification (per `agent.md` §4)

- **Automated**: a small pytest under `tests/unittest/test_mechanics_systems.py` that instantiates both systems, checks dimensions, runs `compile("numpy").f(x0, u0, 0)` for a known state, and spot-checks one analytical equilibrium (cart at rest under zero force → `f = 0`; double pendulum hanging at `q=0, dq=0` → gravity cancels from the equations since `sin(0)=0`, so `f` is zero as well).
- **Manual**: extend `tests/manual/demos_alex.py` (or a new flat script `tests/manual/demo_cartpole_doublependulum.py`) that builds each system, calls `compute_forced(...)`, and `animate()`s the result with the matplotlib renderer.
- **Demo**: not scoped here (TRL 1 mechanics); can follow up in a dedicated demo script if the user wants a published example.

## Todos

- [ ] **cartpole** — create `minilink/mechanics/cartpole.py` with `CartPole` (H/C/B/g/d from pyro) and paired `get_kinematic_geometry()` + `get_kinematic_transforms()`
- [ ] **doublependulum** — create `minilink/mechanics/pendulum.py` with `DoublePendulum` (H/C/B/g/d from pyro) and paired kinematic geometry+transforms with per-joint `TorqueArrow`s
- [ ] **exports** — extend `minilink/mechanics/__init__.py` to export `CartPole` and `DoublePendulum`
- [ ] **design_note** — update `DESIGN.md` §2 to note that `mechanics/` also hosts concrete mechanical systems
- [ ] **tests** — add a pytest under `tests/unittest/` that instantiates each system, checks dimensions and known-equilibrium `f`
- [ ] **manual_demo** — add a flat manual script under `tests/manual/` that runs `compute_forced` + `animate` for both systems

## Out of scope

- No change to `blocks/examples.py::Pendulum` (stays as the minimal didactic demo).
- No move of `blocks/dynamic_bicycle.py` into `mechanics/` (separate refactor, requires user approval).
- No JAX variants of these two classes (follow-up if needed; base class already has `JaxMechanicalSystem`).
