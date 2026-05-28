# Implementation Plan: Custom System Port Definition Ergonomics

**Status:** Draft for maintainer review  
**Related:** [DESIGN.md](../../DESIGN.md) §3 (`System`, ports), [ROADMAP.md](../../ROADMAP.md) P2 (diagram port ergonomics), [agent.md](../../agent.md) (math-first, minimal ceremony)

## 1. Problem

High-level users define custom `System` / `StaticSystem` / `DynamicSystem` blocks by:

1. Passing aggregate dimensions `(n, m, p)` to `super().__init__`.
2. Clearing default ports (`self.inputs = {}`, `self.outputs = {}`).
3. Calling `add_input_port` / `add_output_port` per named signal.
4. Implementing `f`, `h`, or port compute functions against a **flat** concatenated `u`.
5. Declaring `dependencies` on each output port for diagram feedthrough / algebraic-loop scheduling.

Named ports are the right diagram contract (wiring, autowire, compilation). The **authoring path** still feels like a single SISO `u`/`y` plant with MIMO ports bolted on afterward. That ceremony is the main remaining friction after diagram composition shortcuts (`+`, `>>`, `@`, `autowire`).

### Symptoms in the codebase

| Symptom | Example locations |
| --- | --- |
| Clear-and-rebuild ports in every `__init__` | `core/blocks/basic.py`, `control/pendulum_pd.py`, bicycle cascade demos |
| Inconsistent use of flat `u` vs helpers | `u[0]` in controllers; `get_port_values_from_u` in `Pendulum`; `u2input_signal` in `DynamicBicycle` |
| Manual `self.p` when outputs ≠ constructor `p` | `PathPlanner` in cascade demos |
| Extra default output `x` on `DynamicSystem` removed or ignored | `Integrator` clears `outputs` and re-adds `y` |
| `dependencies` chosen by convention | `[]`, explicit lists, or `"all"` with little guidance |

Helpers already exist (`get_port_values_from_u`, `u2input_signal`, `get_input_port_slice`) but are optional and not the primary documented pattern.

## 2. Goals and non-goals

### Goals

- Reduce boilerplate for **diagram blocks** (static controllers, sources, MIMO plants with named ports).
- Keep **one** port model: `InputPort` / `OutputPort`, flat `u` at compile/simulation boundaries.
- Preserve equation readability (`f`, `h`, port compute as functions of `(x, u, t, params)`).
- Stay backward compatible: existing catalog plants and demos keep working without forced migration.
- Align with `agent.md`: coach toward textbook-readable math; avoid heavy metaprogramming.

### Non-goals (this plan)

- Changing diagram wiring (`connect`, `DiagramSystem` structure) or composition shortcuts.
- Replacing aggregate `n` / state metadata (`VectorSignal` for `x`).
- Auto-inferring `dependencies` from arbitrary Python compute bodies (fragile).
- New external dependencies or code generation pipelines.
- Turning `System` into a dataclass or splitting the facade in this pass (see ROADMAP Phase B queue).

## 3. Design constraints (unchanged contracts)

From [DESIGN.md](../../DESIGN.md):

- Port compute callables use `(x, u, t, params=None)`; `u` is the **concatenated** subsystem input vector in port order.
- `dependencies` on `OutputPort` drives `DiagramSystem.get_local_input` and algebraic-loop detection.
- Input port **insertion order** in `self.inputs` defines concatenation order (Python 3.7+ dict ordering).
- Native-array equation rule applies inside port compute and `f` / `h`.

Any new API must compile down to the same `inputs` / `outputs` dicts and the same flat `u` layout.

## 4. Options

Each option can ship independently. Recommended phasing is in §6.

---

### Option A — `define_ports()` builder on `System` (recommended core)

Add a single method (name TBD: `define_ports`, `set_ports`, `configure_ports`) that replaces the clear-and-rebuild ritual.

**Sketch:**

```python
class PropController(StaticSystem):
    def __init__(self):
        super().__init__(0, 0, 0, default_ports=False)  # see Option C
        self.params = {"Kp": 10.0}
        self.name = "Controller"
        self.define_ports(
            inputs={
                "ref": PortSpec(1, nominal=0.0),
                "y": PortSpec(1, nominal=0.0),
            },
            outputs={
                "u": PortSpec(1, compute=self.ctl, depends_on=("ref", "y")),
            },
        )
```

`PortSpec` is a small record (dataclass or named tuple): `dim`, optional `nominal`, `labels`, `units`, `compute`, `depends_on`.

**Behavior:**

- Clears or skips default `u`/`y` when used with `default_ports=False`.
- Calls existing `add_input_port` / `add_output_port` internally.
- Runs `recompute_input_properties()`; optionally sets `self.p` from sum of output dims (see §5.1).
- Preserves dict insertion order for inputs.

| Pros | Cons |
| --- | --- |
| One call site per block; hard to forget `recompute_input_properties` | New type (`PortSpec`) to document |
| Easy migration: mechanical rewrite of existing `__init__` | Name/API bike-shedding |
| No change to compile path | Still flat `u` inside `ctl` unless paired with Option B |

**Effort:** Small–medium (core method + `PortSpec` + tests + one migrated example).

---

### Option B — Named port access in equation paths (recommended companion)

Make `get_port_values_from_u` / `u2input_signal` the **documented default** and add a short alias on `System` for ergonomics.

**Sketch:**

```python
def ctl(self, x, u, t=0.0, params=None):
    ref, y = self.port_vals(u, "ref", "y")  # tuple of 1d arrays
    ...
```

or

```python
def f(self, x, u, t=0.0, params=None):
    inp = self.port_dict(u)  # MappingProxy or dict, read-only view
    tau = inp["u"][0] + inp["w"][0]
```

**Implementation choices:**

| Variant | API | Notes |
| --- | --- | --- |
| B1 | `port_vals(u, *ids)` | Thin wrapper over `u2input_signal`; best for 1–3 ports |
| B2 | `port_dict(u)` | Full dict; matches `Pendulum` style |
| B3 | Optional decorator `@uses_ports("ref", "y")` | Injects kwargs; more magic, not recommended first |

**Recommendation:** Ship **B1 + B2** as aliases; document B1 in examples. Do not add decorators in v1.

| Pros | Cons |
| --- | --- |
| No compile changes | Slight overhead (dict/slices) in hot paths—negligible for most blocks |
| Encourages stable port names vs magic indices | JAX traceability: keep returns as arrays, no Python scalars in traced paths |
| Works with all existing systems immediately | |

**Effort:** Small (aliases + docstring examples + migrate 2–3 reference blocks).

---

### Option C — `default_ports=False` on `System.__init__`

Add a keyword-only flag to `System` / `StaticSystem` / `DynamicSystem`:

```python
def __init__(self, n=0, m=0, p=1, *, default_ports=True):
```

When `default_ports=False`:

- Do **not** create default `u` / `y` (and for `DynamicSystem`, do **not** auto-add `x` unless opted in).
- Require ports via `define_ports` or explicit `add_*_port` before use.
- `m` and `p` may start at 0 and are updated when ports are defined.

| Pros | Cons |
| --- | --- |
| Removes “delete defaults” boilerplate | Another constructor parameter |
| Clear intent: “this is a diagram block” | Existing `DynamicSystem` users rely on default `x` port |
| Pairs naturally with Option A | Need `expose_state_output=True` opt-in for `x` on dynamics |

**Backward compatibility:** Default `default_ports=True` preserves current behavior.

**Effort:** Small (init branch + tests for both modes).

---

### Option D — Class-level port declaration (optional, later)

Declare ports as class attributes; `__init_subclass__` or first `__init__` builds ports.

```python
class PropController(StaticSystem):
  _ports = PortLayout(
      inputs={"ref": 1, "y": 1},
      outputs={"u": (1, "ctl", ("ref", "y"))},
  )
```

| Pros | Cons |
| --- | --- |
| Very compact for simple blocks | Harder to set per-instance nominal from constructor args |
| Reads like a schema | Metaclass/subclass hook complexity |
| | Conflicts with `agent.md` “avoid ceremony” if over-generalized |

**Recommendation:** Defer until Option A is validated in 3+ real blocks. Revisit only if maintainers want declarative style for `core/blocks`.

**Effort:** Medium–large; higher review surface.

---

### Option E — `dependencies` ergonomics (narrow)

Do **not** infer dependencies from code. Improve defaults and validation instead.

| Sub-option | Description |
| --- | --- |
| E1 | Default new output ports to `depends_on=()` instead of `"all"` when created via `define_ports` |
| E2 | `define_ports` **requires** `depends_on` for every output with `compute` (explicit is better than wrong loops) |
| E3 | `diagram.check_port_dependencies()` debug helper: warn if `depends_on` names a port not in `inputs` |
| E4 | Document decision tree in DESIGN.md (measurement vs feedthrough vs state-only) |

**Effort:** Small (defaults + validation + docs).

---

### Option F — Documentation-only “custom block checklist” (lowest risk)

Add a subsection to DESIGN.md §3 (`System`):

1. Choose `StaticSystem` / `DynamicSystem` / `Source`.
2. Clear defaults if multi-port; preserve order of `add_input_port`.
3. Use `u2input_signal` / `get_port_values_from_u` in math.
4. Set `dependencies` explicitly per output port.
5. Run `diagram.check_algebraic_loops()` after wiring.

| Pros | Cons |
| --- | --- |
| Zero API risk | Does not remove boilerplate |
| Immediate value | Friction remains |

**Recommendation:** Do **in parallel with Option A** (not instead of).

**Effort:** Trivial.

---

### Option G — `StaticSystem.from_ports` / `DynamicSystem.from_ports` factories

Alternative entry point instead of extending `__init__`:

```python
class PropController(StaticSystem):
    @classmethod
    def build(cls, Kp=10.0):
        return StaticSystem.from_ports(
            cls,
            inputs={...},
            outputs={...},
            params={"Kp": Kp},
        )
```

| Pros | Cons |
| --- | --- |
| Leaves `__init__` untouched for legacy | Awkward for subclass methods as `compute` |
| | Two ways to construct the same object |

**Recommendation:** Prefer Option A on the instance unless subclassing patterns prove awkward.

**Effort:** Medium; optional.

---

## 5. Cross-cutting decisions

### 5.1 Sync aggregate `p` (and optionally `m`) from ports

Today `m` is recomputed; `p` is not. Proposal:

- After `define_ports`, set `self.p = sum(port.dim for port in self.outputs.values())` when `outputs` non-empty.
- Document that legacy `p` on `__init__` is ignored once ports are defined (or warn once).

### 5.2 Port order stability

- `define_ports` must preserve insertion order (Python 3.7+ `dict`).
- Document that reordering keys changes `u` layout and breaks silently—consider optional `order=("ref", "y")` tuple argument if needed later.

### 5.3 Labels and units on `PortSpec`

Support optional `labels` / `units` on input and output specs so metadata is not a second mutation pass:

```python
"ref": PortSpec(1, nominal=0.0, labels=["θ_ref"], units=["rad"]),
```

### 5.4 JAX and equation paths

- `port_vals` / `port_dict` return array views/slices suitable for JAX when `u` is JAX.
- Examples in `core/blocks` should use `array_module` where applicable (existing pattern).

### 5.5 Relationship to diagram shortcuts

Port definition ergonomics is **orthogonal** to `+` / `>>` / `@` / `autowire`. Better ports make shortcuts more valuable because custom blocks are cheaper to author.

### 5.6 Relationship to ROADMAP Phase B

The “split `System` facade from math contract” item remains separate. Options A–C touch `System` structure but not plotting/simulation shortcuts.

## 6. Recommended phasing

| Phase | Deliverables | Risk |
| --- | --- | --- |
| **0** | Option F: DESIGN.md checklist + link from README “custom blocks” | None |
| **1** | Option C: `default_ports=False` + tests | Low |
| **2** | Option A: `PortSpec` + `define_ports()` + migrate `PropController`, `Integrator` | Low |
| **3** | Option B: `port_vals` / `port_dict` aliases + migrate `Pendulum.f`, one cascade block | Low |
| **4** | Option E: stricter `depends_on` via `define_ports` + optional diagram debug check | Low |
| **5** | Migrate `core/blocks`, `control/`, one cascade demo file; update examples | Medium (diff size) |
| **Defer** | Option D (class-level schema), Option G (factories) | — |

**Exit criteria for “done”:**

- New user can author a 2-input, 1-output static block in ≤10 lines of structure (excluding math).
- No regression in `pytest` for diagram compile, simulation, autowire, JAX tests touching ports.
- DESIGN.md §3 updated with the canonical pattern; ROADMAP P2 item partially satisfied.

## 7. Migration strategy

1. **No breaking changes** in phase 1–3: existing classes unchanged until opt-in.
2. **Opt-in migration** per file: `default_ports=False` + `define_ports`.
3. **Catalog plants** (`Pendulum`, `DynamicBicycle`): migrate math to `port_vals` first; structural `define_ports` when convenient (large diffs—batch separately).
4. **Do not** mass-rewrite bicycle cascade demos in the same PR as the API introduction.

## 8. Testing plan

| Area | Tests |
| --- | --- |
| `define_ports` | Input/output dims, `m`/`p`, order, nominal, labels, duplicate ids raise |
| `default_ports=False` | Empty ports until defined; `DynamicSystem` without auto `x` unless requested |
| Compile/sim | Existing `test_compile_pipeline`, `test_composition`, simulator tests stay green |
| Diagram | One new test: block built with `define_ports` wired in `DiagramSystem`, `f` matches hand-built |
| Algebraic loops | Output with wrong `depends_on` still caught; E3 warnings optional |
| JAX | One block using `port_vals` in `f` with JAX backend if available |

## 9. Open questions (maintainer review)

1. **Preferred name:** `define_ports`, `configure_ports`, or `ports(...)`?
2. **`PortSpec` location:** `minilink.core.system` vs `minilink.core.ports` module?
3. **Auto `x` output on `DynamicSystem`:** always opt-in with `default_ports=False`, or keep `expose_state_output=True` default for dynamics?
4. **Require `depends_on` in `define_ports`?** (recommended: yes for outputs with `compute`)
5. **Sync `self.p` automatically:** always, or only when `p==0` at define time?
6. **Public export:** include `PortSpec` in future `minilink.__init__` freeze?

## 10. Summary recommendation

Ship **Options A + B + C + E2 + F** in phases 0–4. That removes init boilerplate, makes named ports natural in math, keeps compile contracts identical, and tightens `dependencies` without magic inference. Defer declarative class-level ports (D) and factory constructors (G) unless phase 2–3 still feels heavy in practice.

---

## Appendix: before / after sketch

**Before (current):**

```python
class PropController(StaticSystem):
    def __init__(self):
        super().__init__(2, 1)
        self.params = {"Kp": 10.0}
        self.name = "Controller"
        self.inputs = {}
        self.add_input_port(1, "ref", nominal_value=np.array([0.0]))
        self.add_input_port(1, "y", nominal_value=np.array([0.0]))
        self.outputs = {}
        self.add_output_port(1, "u", function=self.ctl, dependencies=["ref", "y"])

    def ctl(self, x, u, t=0, params=None):
        r, y = u[0], u[1]
        ...
```

**After (target):**

```python
class PropController(StaticSystem):
    def __init__(self):
        super().__init__(default_ports=False)
        self.params = {"Kp": 10.0}
        self.name = "Controller"
        self.define_ports(
            inputs={"ref": PortSpec(1, nominal=0.0), "y": PortSpec(1, nominal=0.0)},
            outputs={"u": PortSpec(1, compute=self.ctl, depends_on=("ref", "y"))},
        )

    def ctl(self, x, u, t=0, params=None):
        ref, y = self.port_vals(u, "ref", "y")
        ...
```
