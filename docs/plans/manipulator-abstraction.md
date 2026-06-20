# Manipulator Base Abstraction — Proposal

Status: **proposal for architectural review** (`TODO: User Architectural Review`).

Context: porting pyro `Manipulator` and `control/robotcontrollers.py` requires a
shared robot base in minilink. Today catalog arms subclass `MechanicalSystem`
directly and duplicate FK/Jacobian methods without exposing them as diagram ports.

## Goal

A controls reader should wire

    controller.r  ──┐
    plant.q    ─────┼──► JointPD / ComputedTorque / EndEffectorPD
    plant.dq   ─────┘

without `Demux` on a stacked `y` vector. Cartesian controllers should wire
`plant.r` and `plant.dr` directly.

## Proposed hierarchy

```
DynamicSystem
    └── MechanicalSystem          # any second-order mechanism in q
            └── Manipulator       # serial arms with an end-effector frame
```

Catalog plants (`OneLinkManipulator`, `TwoLinkManipulator`, …) subclass
`Manipulator`, not `MechanicalSystem` directly.

Home: `minilink/dynamics/abstraction/manipulator.py`.

## Port contract

### `MechanicalSystem` — add joint-space outputs

Today: primary `y = [q; v]` (same as `x` when `expose_state=True`), plus
auxiliary port `x`.

**Add** (non-breaking — keep `y` and `x`):

| Port | Dim | Compute | Notes |
| --- | ---: | --- | --- |
| `q` | `dof` | `x2q(x)[0]` | generalized positions |
| `dq` | `dof` | `x2q(x)[1]` | generalized velocities (`v` in math) |

Port labels/units inherit from the first/second half of `state.labels` /
`state.units`. Dependencies: `()` — no feedthrough.

Rationale:

- Matches control naming (`q`, `dq`) used in pyro robot controllers.
- `y` remains the primary output (`System.p` unchanged) for backward compatibility.
- `x` stays for state-feedback blocks that want the full vector.
- Controllers that need only angles or rates connect to named ports.

### `Manipulator` — add task-space outputs

| Port | Dim | Compute | Notes |
| --- | ---: | --- | --- |
| `r` | `e` | `forward_kinematic_effector(q, params)` | end-effector pose/coords |
| `dr` | `e` | `J(q, params) @ dq` | end-effector velocity |

Constructor: `Manipulator(dof, actuators=None, effector_dim=2)`.

Subclasses **must** override:

```python
def forward_kinematic_effector(self, q, params=None):
    ...

def J(self, q, params=None):
    ...  # shape (effector_dim, dof)
```

Default implementations return zeros (traceable, safe for abstract testing).

Optional helper (not a port):

```python
def forward_differential_kinematic_effector(self, q, dq, params=None):
    return self.J(q, params) @ dq
```

## Math hooks (unchanged)

`MechanicalSystem` keeps `H`, `C`, `B`, `g`, `d`, `generalized_force`,
`forward_dynamics`, `inverse_dynamics`. `Manipulator` does not alter `f`/`h`.

Port compute paths call the same FK/Jacobian used by graphics and future
`control/robotic.py` blocks — **one definition, three consumers** (ports,
animation, control laws).

## Relation to pyro

Pyro `Manipulator` stores `e` (effector dim), documents `r = f(q)`,
`dr = J(q) dq`, and expects subclasses to implement FK + `J`. Minilink mirrors
that math but expresses outputs as **explicit ports** instead of methods only.

Pyro `RobotController.x2q(y)` assumed `y` was stacked joint space. Minilink
controllers should prefer `get_port_values_from_u` / diagram connections to
`plant.q` and `plant.dq`, or read ports from compiled internal signals.

## Implementation sketch

In `MechanicalSystem.__init__` after port setup:

```python
self.add_output_port("q", dim=dof, function=self.compute_q, dependencies=())
self.add_output_port("dq", dim=dof, function=self.compute_dq, dependencies=())
```

```python
def compute_q(self, x, u, t=0, params=None):
    q, _ = self.x2q(x)
    return q

def compute_dq(self, x, u, t=0, params=None):
    _, dq = self.x2q(x)
    return dq
```

In `Manipulator.__init__`:

```python
super().__init__(dof=dof, actuators=actuators)
self.effector_dim = int(effector_dim)
self.add_output_port("r", dim=self.effector_dim, function=self.compute_r, dependencies=())
self.add_output_port("dr", dim=self.effector_dim, function=self.compute_dr, dependencies=())
```

## Catalog migration

1. Add `Manipulator` base with ports.
2. Change `OneLinkManipulator` … `FiveLinkPlanarManipulator` to subclass
   `Manipulator`; move shared FK/Jacobian to the subclass (already there).
3. `SpeedControlledManipulator` — separate kinematic-only plant; may subclass
   a lighter `KinematicManipulator` (positions only, no `H`) **or** stay as
   `DynamicSystem` until needed.
4. Update tests: port dimensions, FK/Jacobian match `compute_r`/`compute_dr`.
5. Demos for joint PD and effector PD wire `plant.q` / `plant.r` explicitly.

## Open questions

1. **Port name `dq` vs `v`?** Proposal: `dq` on ports (control language),
   `v` stays as the local in `x2q` and EoM text.
2. **Effector `r` for 3D arms:** `e=3` position only first; orientation as
   Euler/quaternion is a follow-up (`effector_dim` or separate `R` port?).
3. **External forces `f_ext`:** pyro documents `J^T f_ext` on the RHS.
   Implement via optional input port `f_ext` on `Manipulator` later, not in v1.
4. **Breaking change?** Keeping `y=[q;dq]` means no break; deprecate using
   `y` for robot control wiring in new demos only.
5. **`JaxManipulator` twin?** Mirror `JaxMechanicalSystem` pattern if catalog
   gains JAX arm variants.

## Enables (blocked today)

- `control/computed_torque.py` — needs `inverse_dynamics` + `q`, `dq`
- `control/robotic.py` — `JointPD`, `EndEffectorPD`, kinematic controllers
- `control/sliding_mode.py` — same access pattern
- Obstacle-aware arms — collision on `r`, not a separate plant subclass

## Smallest slice

1. `MechanicalSystem` gains `q`/`dq` ports + compute helpers.
2. `Manipulator` base with `r`/`dr` ports; migrate `OneLinkManipulator` only.
3. One demo: `TwoLinkManipulator` + joint PD using port wiring.
