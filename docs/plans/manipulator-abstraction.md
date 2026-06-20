# Manipulator Base Abstraction — Proposal

Status: **proposal for architectural review** (`TODO: User Architectural Review`).

Context: porting pyro `Manipulator` and `control/robotcontrollers.py` requires a
shared robot base in minilink. Today catalog arms subclass `MechanicalSystem`
directly and duplicate FK/Jacobian methods without exposing them as diagram ports.

Design goal: **notation should read like a robotics/controls textbook** in both
the port wiring diagram and the method names — while respecting minilink's
framework-wide control convention (`r` = reference, `y` = measurement, `u` =
actuator command).

## Textbook notation map

Standard manipulator notation (e.g. Spong/Hutchinson, Siciliano):

| Math | Meaning | Minilink role |
| --- | --- | --- |
| `q` | generalized coordinates (joint positions) | plant output port `q` |
| `q̇` | generalized velocities | plant output port `dq` |
| `x = [q; q̇]` | mechanical state | plant state / ports `x`, `y` |
| `τ` | applied joint torques | plant input port `u` (documented as `τ` for arms) |
| `H(q)`, `C(q,q̇)`, `g(q)` | inertia, Coriolis, gravity | methods on `MechanicalSystem` |
| `p = f(q)` | forward kinematics (task position) | plant output port `p` |
| `ṗ = J(q) q̇` | differential kinematics | plant output port `pdot` |
| `J(q)` | manipulator Jacobian `∂p/∂q` | method `J(q, params)` |
| `p_d`, `ṗ_d` | desired task position / velocity | **controller** input `r` (reference), not a plant port |
| `f_ext` | external wrench at end-effector | future plant **input** port (phase 2) |

### Why not pyro's `r` / `dr` on the plant?

Minilink reserves **`r` for the reference input on controllers**
([DESIGN.md §4](../../DESIGN.md)). A plant output named `r` would collide in
every diagram:

```text
  ref.r  ──► controller.r     (desired p_d)   ✓ textbook + framework
  plant.r ──► controller.y    (measured p)    ✗ reads as two “references”
```

**Plant task-space outputs are `p` and `pdot`** (position and its time derivative).
The desired task trajectory connects to the controller's **`r`** port; the
measured task state connects to **`y`** — the same pattern as joint-space PD on
`[q, dq]`.

### Why `dq` and `pdot` (not `qdot` / `v`)?

| Candidate | Verdict |
| --- | --- |
| `dq` | **Joint port.** Matches `q̇` in code; already used in `control/linear.py` docstrings (`[q, dq]`). |
| `qdot` | Readable but breaks the two-letter port pattern (`q`, `dq`, `p`, `pdot`). |
| `v` on a port | **Rejected** — ambiguous (joint rate vs task velocity vs state segment). |
| `pdot` | **Task port.** Parallel to `dq`; reads as `ṗ` without overloading `v`. |
| `p` | **Task port.** Standard task-space position; avoids overloading framework state `x`. |

Internal math may still use short locals `q`, `dq`, `p`, `pdot`, `J`, `tau`
after unpacking `x`, `u`, `params` ([agent.md](../../agent.md) textbook style).

## Goal (wiring picture)

Joint-space PD:

```text
  ref.r ─────────────► controller.r
  plant.q  ──┐
  plant.dq ──┴─ Mux ─► controller.y
  controller.u ────────► plant.u
```

End-effector PD (operational space):

```text
  ref.r ─────────────► controller.r          (desired p_d; rate part via metadata / Mux)
  plant.p ──┐
  plant.pdot ┴─ Mux ──► controller.y         (measured [p, ṗ])
  controller.u ────────► plant.u             (τ)
```

No `Demux` on stacked `y` for new robot demos.

## Proposed hierarchy

```text
DynamicSystem
    └── MechanicalSystem          #  H(q) q̈ + C(q,q̇) q̇ + g(q) = τ
            └── Manipulator       #  +  p = f(q),  ṗ = J(q) q̇
```

Catalog arms (`OneLinkManipulator`, …) subclass `Manipulator`.

Home: `minilink/dynamics/abstraction/manipulator.py`.

## Port contract

### Framework ports (unchanged)

| Port | Role |
| --- | --- |
| `u` | actuator command; for manipulators document as **joint torque `τ`** |
| `y` | primary output (`System.p`); stays **`y = x = [q; dq]`** for backward compatibility |
| `x` | full state (already exposed via `expose_state=True`) |

### `MechanicalSystem` — joint-space outputs (new)

| Port | Dim | Compute | Textbook |
| --- | ---: | --- | --- |
| `q` | `dof` | `x2q(x)[0]` | `q` |
| `dq` | `dof` | `x2q(x)[1]` | `q̇` |

Dependencies: `()` (no feedthrough). Labels/units from the first / second half of
`state.labels` and `state.units`.

### `Manipulator` — task-space outputs (new)

| Port | Dim | Compute | Textbook |
| --- | ---: | --- | --- |
| `p` | `task_dim` | `forward_kinematics(q, params)` | `p = f(q)` |
| `pdot` | `task_dim` | `J(q, params) @ dq` | `ṗ = J(q) q̇` |

Constructor: `Manipulator(dof, actuators=None, task_dim=2)`.

Attribute `task_dim` replaces pyro's `e` / catalog's `effector_dim` in new code
(catalog migration may keep a property alias during transition).

## Method contract (textbook names)

### On `MechanicalSystem` (existing + port helpers)

| Method | Returns | Textbook |
| --- | --- | --- |
| `H(q, params)` | `(dof, dof)` | inertia matrix |
| `C(q, dq, params)` | `(dof, dof)` | Coriolis matrix |
| `g(q, params)` | `(dof,)` | gravity vector |
| `B(q, params)` | `(dof, m)` | actuator map |
| `d(q, dq, u, t, params)` | `(dof,)` | damping / dissipation |
| `generalized_force(...)` | `(dof,)` | RHS force; default `B(q) @ u` → `τ` |
| `forward_dynamics(q, dq, u, ...)` | `ddq` | `q̈` from `τ` |
| `inverse_dynamics(q, dq, ddq, ...)` | `tau` | `τ` from `q̈` |
| `x2q(x)` | `[q, dq]` | split `x = [q; q̇]` |
| `q2x(q, dq)` | `x` | stack state |

Port wiring helpers (thin; not part of the textbook EoM):

| Method | Port |
| --- | --- |
| `h_q(x, u, t, params)` | `q` |
| `h_dq(x, u, t, params)` | `dq` |

Prefer **`h_q` / `h_dq`** over `compute_q` so port functions read as output maps
`h_port` while `f` / `H` / `C` stay the dynamics vocabulary.

### On `Manipulator` (subclass hooks)

| Method | Returns | Textbook |
| --- | --- | --- |
| `forward_kinematics(q, params)` | `(task_dim,)` | `p = f(q)` |
| `J(q, params)` | `(task_dim, dof)` | `J(q) = ∂p/∂q` |

Default implementations return zeros (safe for abstract tests / JAX trace).

Optional inline helper (not a separate port):

```python
# ṗ = J(q) q̇  — used by h_pdot and by controllers
pdot = J @ dq
```

Port wiring:

| Method | Port |
| --- | --- |
| `h_p(x, u, t, params)` | `p` |
| `h_pdot(x, u, t, params)` | `pdot` |

### Pyro → minilink rename (catalog migration)

| Pyro / catalog today | Proposed |
| --- | --- |
| `forward_kinematic_effector(q)` | `forward_kinematics(q, params)` |
| `forward_differential_kinematic_effector(q, dq)` | inline `J(q) @ dq` or `task_velocity(q, dq, params)` if a named helper helps |
| `effector_dim` | `task_dim` |
| `e` (pyro) | `task_dim` |

Keep **`J`** as the method name — it *is* the textbook symbol.

## Math hooks (unchanged)

`Manipulator` does **not** override `f` / `H` / `C` / `g`. It only adds
kinematic outputs. Dynamics remain:

```text
H(q) q̈ + C(q, q̇) q̇ + g(q) + d(q, q̇, u, t) = B(q) u
```

One `forward_kinematics` / `J` definition serves **ports**, **animation**, and
future **`control/robotic.py`** blocks.

## Relation to future controllers

Naming alignment for `control/robotic.py` (planned):

| Controller | Plant inputs wired | Law (textbook) |
| --- | --- | --- |
| Joint PD / PID | `y` ← Mux(`q`, `dq`) | `τ = K_p (q_d - q) + K_d (q̇_d - q̇)` |
| Computed torque | `q`, `dq` (+ ref trajectory) | `τ = H(q) v + C(q,q̇)q̇ + g(q)` |
| End-effector PD | `y` ← Mux(`p`, `pdot`) | `τ = J(q)^T ( K_p (p_d - p) + K_d (ṗ_d - ṗ) )` |
| Kinematic (resolved rate) | `q`, `p` | `q̇ = J(q)^+ ṗ_d` |

Controller ports stay **`r`, `y`, `u`** per DESIGN. Task-space **desired**
trajectories use `r`; **measured** task state uses `y` fed from `plant.p` /
`plant.pdot`.

## Implementation sketch (for later — not in this PR)

```python
# MechanicalSystem.__init__ (after existing ports)
self.add_output_port("q", dim=dof, function=self.h_q, dependencies=())
self.add_output_port("dq", dim=dof, function=self.h_dq, dependencies=())

# Manipulator.__init__
super().__init__(dof=dof, actuators=actuators)
self.task_dim = int(task_dim)
self.add_output_port("p", dim=self.task_dim, function=self.h_p, dependencies=())
self.add_output_port("pdot", dim=self.task_dim, function=self.h_pdot, dependencies=())
```

```python
def h_p(self, x, u, t=0, params=None):
    q, dq = self.x2q(x)
    return self.forward_kinematics(q, params)

def h_pdot(self, x, u, t=0, params=None):
    q, dq = self.x2q(x)
    J = self.J(q, params)
    return J @ dq
```

## Catalog migration (later)

1. Add `Manipulator` in `dynamics/abstraction/`.
2. Rebase `OneLinkManipulator` … `FiveLinkPlanarManipulator` on `Manipulator`.
3. Rename `forward_kinematic_effector` → `forward_kinematics` (clean break;
   pre-1.0 no-alias rule).
4. `SpeedControlledManipulator` — kinematic-only plant; defer or introduce
   `KinematicChain` (positions only, no `H`) when a demo needs it.
5. Tests: `h_p` / `h_pdot` match analytic FK and `J @ dq`.
6. Demo: `TwoLinkManipulator` + joint PD with `plant.q` / `plant.dq` wiring.

## Open questions

1. **3D orientation in `p`:** v1 = position components only (`task_dim=3`);
   orientation as separate ports (`R`, `omega`) or augmented `p` later?
2. **`f_ext` input port:** external wrench; adds `J(q)^T f_ext` to the RHS —
   phase 2, not v1.
3. **`JaxManipulator`:** mirror `JaxMechanicalSystem` when JAX arms are needed.
4. **Stacked convenience port:** optional `task` port = Mux(`p`, `pdot`) factory
   for controllers that want one `y` — diagram helper only, not on the plant?

## Enables (blocked today)

- `control/computed_torque.py` — `inverse_dynamics`, ports `q`, `dq`
- `control/robotic.py` — joint / operational-space laws
- `control/sliding_mode.py`
- Obstacle checks on `p` instead of per-plant `*withObstacles` subclasses

## Smallest implementation slice (when approved)

1. `MechanicalSystem`: ports `q`, `dq` + `h_q`, `h_dq`.
2. `Manipulator`: ports `p`, `pdot` + `forward_kinematics`, `J`, `h_p`, `h_pdot`.
3. Migrate `OneLinkManipulator` only; one joint-PD demo.
