# Articulated mechanism layer (design)

Status: draft plan (July 2026). No implementation in this phase.

## Problem

UR5 spatial RNEA/ABA, kinematics, and the symbolic Lagrange path in
[`articulated_robot_eom.ipynb`](../../examples/learn/teaching/topics/articulated_robot_eom.ipynb)
describe the **same robot twice**. Spatial helpers and tree sweeps live only in
[`ur5.py`](../../minilink/dynamics/catalog/manipulators/ur5.py). Symbolic geometry
lives in [`MechanicalModel.add_dh_chain`](../../minilink/symbolic/mechanics/model.py).

We need a shared **mechanism description** layer (geometry + inertia + topology)
with numeric and symbolic counterparts, while keeping minilink's **stateless
`System`** contract and existing **EoM plant API** (`Manipulator` / `MechanicalSystem`).

## Goals

1. **One resolved spec** feeds spatial RNEA/ABA and symbolic derive (parity / teaching).
2. **Params at the plant boundary** for parametric catalog robots (link length, mass, …).
3. **Adapter-specific geometry build** (DH, joint placement, URDF, …) — no magic reconstruction.
4. **Stateless models** — no configured geometry stored on the plant.
5. **Minimal surface change** for control/sim: `SerialSpatialManipulator(Manipulator)` keeps `f`, `H`, `C`, `g`, FK, `J`.
6. **Short symbolic path:** `SymbolicMechanismModel` → sym `MechanicalSystem` → `to_minilink` (no extra orchestrator class).

## Non-goals (v1)

- Floating base, closed loops, mimic joints, contact.
- Full URDF productization (loader is a later adapter).
- Replacing symbolic Lagrange with RNEA for export.
- `MechanismModel` implementing `H` / `C` / `g` (EoM stays on plant / sym `MechanicalSystem`).
- Branched kinematic trees (design tree-ready; ship serial first).

---

## Architecture (layers)

```
┌──────────────────────────────────────────────────────────────────┐
│ Plant (user-facing)                                               │
│ SerialSpatialManipulator(Manipulator) · catalog UR5Manipulator    │
│ API: f, forward_dynamics, inverse_dynamics, H, C, g, FK, J, tf    │
│ Holds: self.params (defaults), x0, ports, skin                    │
└────────────────────────────┬─────────────────────────────────────┘
                             │ params= override per call
┌────────────────────────────▼─────────────────────────────────────┐
│ Dynamics engines (algorithm choice — not description)             │
│ Numeric spatial: rnea, aba, inertia_from_rnea, bias_force         │
│ Symbolic: SymbolicMechanismModel.derive → sym MechanicalSystem    │
└────────────────────────────┬─────────────────────────────────────┘
                             │ reads resolved spec / SymPy graph
┌────────────────────────────▼─────────────────────────────────────┐
│ Mechanism description (dual contract)                             │
│ MechanismModel (numeric)  |  SymbolicMechanismModel (SymPy)       │
└────────────────────────────┬─────────────────────────────────────┘
                             │ built by adapters (once or per call)
┌────────────────────────────▼─────────────────────────────────────┐
│ Adapters                                                          │
│ from_dh(params) · from_joint_placements(params) · from_urdf(path) │
└──────────────────────────────────────────────────────────────────┘
```

**Principle:** mechanism models describe **what the robot is**; plants describe
**how equations are exposed**; engines describe **how EoM is computed**.

**Symbolic stack (two classes, not three):**

```
SymbolicMechanismModel.from_dh(...)
    .add_gravity(...)          # optional environment / loads on same builder
    .derive(method="lagrange") → symbolic.mechanics.MechanicalSystem
    .to_minilink({...})        → numeric MechanicalSystem / JaxMechanicalSystem
```

`MechanicalModel` is **not** part of the target API — absorbed into
`SymbolicMechanismModel` and deprecated (thin alias during migration only).

---

## Core types (`MechanismSpec`)

Shared vocabulary in `minilink/dynamics/mechanism/spec.py` (dataclasses + validation).

| Type | Fields | Notes |
| --- | --- | --- |
| `MechanismTopology` | `n_bodies`, `parent` | **Static** tree shape; serial v1: `parent = (-1, 0, 1, …, n-2)` |
| `RevoluteJoint` | `placement` (4×4), `axis` (3,) | \(T = E \cdot R_{\text{axis}}(q)\) |
| `PrismaticJoint` | `placement`, `axis` | later |
| `RigidLink` | `mass`, `com` (3,), `inertia` (3×3) | COM frame = link frame after joint |
| `MechanismEnvironment` | `gravity`, optional `gravity_direction` | default world \(-Z\) |
| `MechanismSpec` | topology, joints, links, environment, optional `tool_transform` | immutable once resolved |

`validate_mechanism(spec)` — consistent lengths, acyclic tree, unit axes, PSD inertia.

**DH is not in the contract.** Standard DH is one adapter compiling to joint placements + axes.

---

## Dual description classes

| Class | Module | Role |
| --- | --- | --- |
| **`MechanismModel`** | `dynamics/mechanism/model.py` | Numeric resolved spec (NumPy/JAX arrays) |
| **`SymbolicMechanismModel`** | `symbolic/mechanism/model.py` | Same semantics; SymPy symbols + `dynamicsymbols` \(q\); **`derive()` lives here** |

Both expose: `dof`, topology, per-joint / per-link data, FK API.

Neither implements numeric `H`, `C`, `g`, or `f`.

### `SymbolicMechanismModel` responsibilities

Absorb today's useful `MechanicalModel` surface:

| Method | Purpose |
| --- | --- |
| `from_dh`, `from_joint_placements`, … | adapters |
| `coordinates`, `parameters` | SymPy DOF / constants |
| `add_gravity` | environment (or on `MechanismEnvironment`) |
| `forward_kinematics`, `jacobian` | symbolic FK |
| `derive(method="lagrange"\|"kane")` | → sym `MechanicalSystem` |
| `to_mechanism_model(subs)` | → numeric `MechanismModel` (spatial path) |

Optional later on the same class: `add_spring`, `add_damper`, `add_force` (non-mechanism loads passed into derive). v1: gravity + serial chain is enough for UR5 / pendulum parity.

### Sym → num bridges

```python
# Spatial numeric path
SymbolicMechanismModel.to_mechanism_model(substitutions) → MechanismModel

# Closed-form matrix path
sym_sys = symbolic_model.derive(...)
plant = sym_sys.to_minilink(substitutions, backend="jax")
```

Same substitution dict style for both.

### Retiring `MechanicalModel`

| Current | Target |
| --- | --- |
| `MechanicalModel("X")` | `SymbolicMechanismModel("X")` |
| `m.add_dh_chain(...)` | `SymbolicMechanismModel.from_dh(...)` or `m.build_from_dh(...)` |
| `m.add_gravity(...)` | `m.add_gravity(...)` on `SymbolicMechanismModel` |
| `m.derive()` | `m.derive()` on `SymbolicMechanismModel` |

Keep `MechanicalModel` as a **deprecated alias** (`= SymbolicMechanismModel`) until tests/notebooks migrate, then remove.

Hand-built sym `MechanicalSystem` (no mechanism graph) remains valid for advanced/tests — bypass the mechanism layer entirely.

---

## Naming trio: topology · model · configuration

Three distinct concepts — do not overload one name:

| Name | Depends on | Holds |
| --- | --- | --- |
| **`MechanismTopology`** | nothing | Static tree shape: `n_bodies`, `parent` |
| **`MechanismModel`** / **`MechanismSpec`** | params / adapter / URDF | Full **design**: topology + joints + link inertias + environment |
| **`MechanismConfiguration`** | spec **+ frozen `q`** | **Pose-dependent** kinematics: frames, spatial transforms, Jacobian data |

`MechanismConfiguration` is the output of one FK pass — not stored on the plant.

| Field | Shape | Use |
| --- | --- | --- |
| `q` | `(n,)` | configuration this bundle was built at (audit / debug) |
| `joint_frames` | `(n, 4, 4)` | viz, `tf` |
| `link_frames` | `(n, 4, 4)` | tool frame |
| `X_up` | `(n, 6, 6)` | RNEA / ABA |
| `S` | `(n, 6)` | RNEA / ABA |
| `joint_origins`, `joint_axes` | `(n, 3)` | geometric `J` |
| `tool_frame` | `(4, 4)` | FK |

```python
config = configure(spec, q)   # or forward_kinematics(spec, q)
tau = rnea(q, dq, qdd, spec, config=config)  # optional intra-call reuse
```

**Stateless rules (DESIGN §1):**

- Never `self._config`, never “last `q`” on the plant.
- `MechanismConfiguration` is a **local variable** or optional argument **within one method call** (e.g. building `H` from \(n\) RNEA columns at the same `q`).
- Trajectory state lives in `Simulator` / `Trajectory`, not on `System`.

---

## Params and geometry reconstruction

### Plant boundary (unchanged minilink contract)

Every hook: `params = self.params if params is None else params`  
(DESIGN / `System.f` — explicit override, never `params or self.params`.)

### Two geometry sources

| Source | `params` role | Build |
| --- | --- | --- |
| **Parametric catalog** (UR5, teaching arms) | Geometry arrays in `self.params` (`a`, `d`, `alpha`, `mass`, …); override at call time | **Adapter chosen at plant construction** reconstructs spec from `params` each call (or inline in FK/RNEA) |
| **URDF / fixed file** | No link-geometry keys in `params`; file is the spec | `MechanismModel.from_urdf(path)` once at construct; plant `params` = tunables only (`damping`, `gravity`, payload) |

There is **no generic** `reconstruct_geometry(params)` without naming the adapter.

### Adapter orchestration

Plant (or factory) records geometry kind:

```python
# Parametric DH catalog
self._resolve = lambda p: mechanism_from_dh_params(p)

# Fixed URDF
self._spec = MechanismModel.from_urdf("robot.urdf")
self._resolve = lambda p: self._spec   # p may still tune damping, etc.
```

Per call:

```python
def forward_dynamics(self, q, v, u, t=0.0, params=None):
    params = self.params if params is None else params
    spec = self._resolve(params)
    config = configure(spec, q)
    ...
```

Link length example (DH): `params["a"][1] = 0.50` — **`from_dh`** interprets it.  
Joint-placement robot: different keys — **`from_joint_placements`** interprets it.

### Symbolic parameters

Lengths/masses as SymPy symbols in `SymbolicMechanismModel`; numeric values at:

- `to_mechanism_model({L2: 0.425, …})` → spatial path
- `derive().to_minilink({…})` → closed-form matrix path

---

## Spatial dynamics engine

Module: `minilink/dynamics/mechanism/spatial.py` (pure functions, `xp = array_module`).

Extract from [`ur5.py`](../../minilink/dynamics/catalog/manipulators/ur5.py):

- `_skew`, `_motion_cross`, `_spatial_inertia`
- `configure(spec, q) → MechanismConfiguration` (alias: `forward_kinematics`)
- `rnea`, `aba`, `bias_force`, `inertia_matrix`

UR5 parity tests (ABA ≡ RNEA–H, ID round-trip) become tests on the shared engine.

---

## Plant class

**`SerialSpatialManipulator(Manipulator)`** — `dynamics/mechanism/plant.py`

| Method | Default engine |
| --- | --- |
| `forward_dynamics` / `f` | ABA |
| `inverse_dynamics` | RNEA |
| `H`, `C`, `g` | RNEA columns / bias (analysis) |
| `forward_dynamics_rnea_h` | teaching / verification |
| `d` | viscous damping from `params` |
| `forward_kinematics`, `J`, `tf` | from `MechanismConfiguration` |

Catalog:

```python
class UR5Manipulator(SerialSpatialManipulator):
    def __init__(self):
        self.params = { ... }  # ros-industrial DH + cylinder inertias
        super().__init__(geometry="dh", ...)  # or explicit _resolve
```

---

## Three FD pipelines (teaching)

One robot, three engines — [`articulated_robot_eom.ipynb`](../../examples/learn/teaching/topics/articulated_robot_eom.ipynb):

| Pipeline | Path |
| --- | --- |
| **ABA** | `params` → adapter → spec → `aba` → `f` |
| **RNEA–H** | same spec → `inertia_matrix` + `bias_force` + solve |
| **Symbolic** | `SymbolicMechanismModel` → `derive()` → `to_minilink` |

Notebook should import **one** symbolic mechanism source instead of copying DH tables.

Example:

```python
sym = SymbolicMechanismModel.from_dh(dh_table, link_properties, coords="q1 ... q6")
sym.add_gravity(-g * sym.world.z)
sym_sys = sym.derive(method="lagrange", simplify=False)
plant = sym_sys.to_minilink({g: 9.81, ...}, backend="jax")
```

---

## Package layout (target)

```
minilink/dynamics/mechanism/
    spec.py           # types, validate_mechanism
    adapters.py       # from_dh_params, from_joint_placements_params
    kinematics.py     # configure(spec, q) → MechanismConfiguration
    spatial.py        # rnea, aba, helpers
    plant.py          # SerialSpatialManipulator

minilink/symbolic/mechanism/
    model.py          # SymbolicMechanismModel (+ derive)
    derivation.py     # moved from symbolic/mechanics/derivation.py
    adapters.py       # from_dh (symbolic)

minilink/symbolic/mechanics/
    symbolic_system.py   # sym MechanicalSystem (EoM holder)
    export.py            # to_minilink
    model.py             # DEPRECATED alias → SymbolicMechanismModel (remove later)
```

Later: `adapters/urdf.py` → fixed `MechanismModel`.

---

## Class inventory

| Artifact | Layer | Action |
| --- | --- | --- |
| `MechanismSpec` + validation | Static IR | Add |
| `MechanismTopology` | Static tree shape | Add |
| `MechanismModel` | Numeric design (spec) | Add |
| `MechanismConfiguration` | Kinematics at frozen `q` | Add |
| `SymbolicMechanismModel` | Symbolic description + `derive()` | Add; absorb `MechanicalModel` |
| `spatial.*` | Algorithms | Extract from `ur5.py` |
| `SerialSpatialManipulator` | Plant | Add |
| `MechanicalModel` | Legacy orchestrator | **Deprecate → remove** |
| `sym MechanicalSystem` | Symbolic EoM | Keep |
| `Manipulator` / `MechanicalSystem` | Numeric EoM API | Keep |
| `UR5Manipulator` | Catalog | Thin wrapper |
| `KinematicModel` delegate (ROADMAP) | — | Defer / drop |

---

## Phasing

### P1 — Numeric extract (ROADMAP §6)

- [ ] `spec.py`, `spatial.py`, `kinematics.py` (`MechanismConfiguration`)
- [ ] `from_dh_params` adapter
- [ ] `SerialSpatialManipulator`
- [ ] Refactor `UR5Manipulator`; keep existing tests green

### P2 — Symbolic collapse

- [ ] `SymbolicMechanismModel` with `from_dh`, `add_gravity`, `derive`, `to_mechanism_model`
- [ ] Move derivation helpers to `symbolic/mechanism/`
- [ ] `MechanicalModel = SymbolicMechanismModel` deprecation shim; migrate tests + EOM notebook
- [ ] Remove shim once call sites updated

### P3 — Second robot + adapters

- [ ] `from_joint_placements_params`
- [ ] Second catalog arm or parameterized teaching arm
- [ ] Optional planar adapter

### P4 — URDF (later)

- [ ] `MechanismModel.from_urdf` → fixed spec
- [ ] Plant with tunable-only `params`

---

## Contracts checklist (implementation gate)

- [ ] Plants remain **stateless** (only `x0` + default `params`; no stored `q` / configuration).
- [ ] All equation paths accept `params=None` → `self.params`.
- [ ] Geometry rebuild is **adapter-named**, not implicit.
- [ ] URDF plants may omit geometry from `params`.
- [ ] `MechanismConfiguration` is return-only / intra-call optional arg (not `MechanismTopology` or `MechanismModel`).
- [ ] Native-array paths use `array_module`; JAX trace parity on UR5.
- [ ] Public sim API unchanged: ABA default FD, RNEA default ID.
- [ ] Symbolic path is **two steps** after build: `derive()` → `to_minilink()`.

---

## Relationship to DESIGN / ROADMAP

When landed:

- **DESIGN.md** — mechanism IR, topology / model / configuration trio, plant wiring, params vs fixed spec, symbolic path.
- **ROADMAP.md §6** — mark “Shared RNEA serial-chain stack” / “ABA on other arms” in progress or done.
- Delete or archive this plan doc when P1–P2 complete.

Related ROADMAP items: shared RNEA stack, ABA generalization, optional `KinematicModel` (prefer this plan instead).
