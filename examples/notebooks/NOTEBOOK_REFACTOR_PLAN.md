# Notebook intro refactor

Refactor `demo_showcase.ipynb` into a self-contained compact intro (~26 cells) and
`demo_overview.ipynb` into a self-contained extensive lab (~40 cells). No
cross-references between notebooks. New showcase content: phase-plane one-liner in
§1, LQR stabilization after CartPole `compute_forced` in §5.

## Checklist

- [ ] Trim `demo_showcase`: phase-plane in §1, LQR after CartPole forced in §5, delete §6 + heavy §9–11, no inter-notebook links (~49→~26 cells)
- [ ] Restructure `demo_overview`: intro prose, merge wiring/noise/viz cells, dedupe Pendulum, no links to other notebooks (~67→~40 cells)
- [ ] Add to overview (self-contained): operators recap, plant catalog, trajopt, `plot_diagram` — omit phase-plane and LQR (showcase owns those quick demos)
- [x] Fix WhiteNoise example — `PendulumWithNoisePort` added; still trim variance sweep cells
- [ ] Add standalone title intro to `demo_plots_animations_backends` (no links to other notebooks)
- [ ] Clear stale embedded outputs from both notebooks after refactor

---

## Policy: no cross-references between notebooks

Each notebook is **self-contained**. Do not add markdown or TOC links pointing at
other `.ipynb` files (`demo_showcase`, `demo_overview`,
`demo_plots_animations_backends`, etc.).

- **Remove** from plan: "Where to go next" cells linking notebooks, "see
  demo_overview §N", "after showcase §7", mutual ←/→ links.
- **OK:** links to repo docs (`README.md`, `DESIGN.md`), `examples/scripts/`, and
  the README Examples table listing notebook paths (external entry points only).

Duplication is avoided by **assigning each topic a single home notebook**, not by
cross-linking.

---

## Roles

| Notebook | Target | Notes |
|----------|--------|-------|
| **demo_showcase** | Compact intro (~26 cells) | README entry; §1 phase-plane; §5 CartPole forced + LQR |
| **demo_overview** | Extensive lab (~40 cells) | Manual wiring, noise, JAX depth, symbolic, engine, trajopt |
| **demo_plots_animations_backends** | Viz backends (13 cells) | Standalone backend comparison |
| **demo_optimization**, **simulation_benchmark** | Unchanged | Own scope |

---

## New content

### A. Phase-plane in showcase §1 (one-liner)

**Add** after first `compute_trajectory` + `plot_trajectory` on bare Pendulum
(subsection "A plant by itself"):

```python
sys.plot_phase_plane()  # vector field + cached trajectory overlay
```

(or `sys.plot_phase_plane(sys.traj)` if explicit traj preferred — match existing API)

**Delete** standalone **§6 Phase-plane analysis** (cells 30–31) — absorbed into §1.

**Edit** intro TOC: remove §6; mention phase-plane under §1 quick start.

### B. LQR stabilization after CartPole `compute_forced` (showcase §5)

**Add** new code cell immediately after existing CartPole `compute_forced` cell
(current cell 28).

Pattern from
[`examples/scripts/statespace/cartpole_lqr_stabilization.py`](../scripts/statespace/cartpole_lqr_stabilization.py):

```python
from minilink.control.lqr import lqr_at_operating_point

plant = CartPole()
x_bar = np.array([0.0, np.pi, 0.0, 0.0])  # inverted equilibrium

lqr_ctl = lqr_at_operating_point(
    plant, x_bar, Q=np.diag([1.0, 10.0, 1.0, 1.0]), R=np.array([[0.1]])
)
diagram = lqr_ctl @ plant

plant.x0 = np.array([-1.0, np.pi + 0.3, 0.0, 0.0])
diagram.compute_trajectory(tf=8.0)
diagram.plot_trajectory()
```

Optional one-line markdown before it: "Linearize at the upright equilibrium,
design LQR gains, wire with `@`."

**Overview:** does **not** repeat LQR quick demo (showcase owns it). Overview plant
catalog may list CartPole/KinematicCar/Drone2D without LQR block.

---

## 1. `demo_showcase.ipynb` — compact intro

**Current:** 49 cells | **Target:** ~26 cells

### KEEP

| § | Content |
|---|---------|
| 0–1 | Intro + Colab setup — **edit TOC** (no §6; no §12 to other notebooks) |
| §1 | Quick start + **NEW** `plot_phase_plane()` one-liner after first simulate |
| §2 | Everything is a System |
| §3 | MassSpringDamper |
| §4 | Operator table + autowire only (drop manual cascade) |
| §5 | CartPole `compute_forced` + **NEW** LQR stabilization cell |
| §7 | One `animate()` + backend markdown **table only** (no link to other notebooks) |
| §8 | JAX compile timing + `jacfwd` |

### DELETE

| § / cells | Why |
|-----------|-----|
| §4 manual cascade (24–25) | Overview home |
| §5 catalog loop (27) | Overview home |
| §5 `cartpole.animate()` (29) | Redundant |
| **§6 phase-plane (30–31)** | Moved into §1 |
| §7 commented backends (33) | Dead code; table in markdown suffices |
| §9 full trajopt (39–42) | Overview home |
| §10 full symbolic (43–45) | Overview home |
| §11 full engine (46–48) | Overview home |
| §12 / "Where to go next" to notebooks | No inter-notebook refs |

### ADD

| Where | What |
|-------|------|
| §1 after plot | `sys.plot_phase_plane()` one-liner + optional half-line markdown |
| §5 after forced | LQR `@` loop (see pattern above) |
| §9–11 (optional) | Single merged markdown "Advanced topics" listing capabilities in prose only — **no notebook links** |

### Net cell budget

```
Keep:   ~20
Delete: ~10
Add:    ~2 (phase-plane line + LQR cell; teasers optional)
Net:    ~26
```

---

## 2. `demo_overview.ipynb` — extensive lab

**Current:** 67 cells | **Target:** ~40 cells

### KEEP (overview-only depth)

- Manual `DiagramSystem` wiring + cascade custom systems
- Noise ports (`PendulumWithNoisePort`)
- JAX dense-network benchmark + autodiff
- Full symbolic chain
- 120-sphere engine
- ODE solvers (keep inline **or** trim — no link to simulation_benchmark notebook)

### DELETE

- Duplicate Pendulum setups (cells 30, 36)
- Commented dead cells (7, 32, 34, 37, 38, 57)
- Extra noise variance sweeps (27–28)
- Two of three incremental simulate/plot pairs (11, 14, or 16)
- Any planned cross-notebook pointer cells

### MERGE

- Cells 12–15 → one connect cell
- Cells 11 + 16 → one simulate/plot cell
- Cells 26–28 → one noise sim
- Cells 31 + 33 → default animate + meshcat (self-contained; no "see plots nb")
- Interactive § → one runnable `DynamicBicycleCar3D().game()`

### ADD (self-contained — no notebook links)

| § | Content |
|---|---------|
| Intro | Purpose + numbered TOC (standalone lab, not "after showcase") |
| Operators | `>>`, `@`, `.autowire()`, `plot_diagram()` brief recap |
| Plant catalog | CartPole, KinematicCar, Drone2D loop + `compute_forced` (no LQR — showcase has quick demo) |
| Trajectory optimization | Full cart-pole swing-up (moved from showcase) |

### NOT in overview (showcase owns)

- Phase-plane one-liner
- LQR quick demo
- MassSpringDamper tutorial
- First pendulum `@` closed-loop story

---

## 3. `demo_plots_animations_backends.ipynb`

### KEEP — all 13 cells as-is

### ADD

- One **standalone** markdown title, e.g. "Plot and animation backends" + one
  sentence on comparing matplotlib/plotly/meshcat/pygame
- **No** references to showcase, overview, or other notebooks

### DELETE — nothing

---

## 4. Docs (external entry points only)

- [`README.md`](../../README.md) Examples table: rows for overview +
  plots/animations (paths only — this is fine; not inside notebooks)
- [`REVIEW_CHECKLIST.md`](../../REVIEW_CHECKLIST.md): fix `demo_animations.ipynb`
  → `demo_plots_animations_backends.ipynb`

---

## Duplication matrix (no cross-refs)

| Content | Home | Other notebooks |
|---------|------|-----------------|
| Pendulum quick start + **phase-plane one-liner** | showcase §1 | overview: omit |
| CartPole forced + **LQR stabilization** | showcase §5 | overview: omit LQR |
| Manual wiring + cascade + noise | overview | showcase: omit |
| MassSpringDamper | showcase §3 | overview: omit |
| Multi-plant catalog (no LQR) | overview | showcase: CartPole only in §5 |
| Plot/animate backend sweep | plots/animations | showcase: 1 animate + table |
| Interactive `game()` | overview | showcase: omit |
| JAX teaser | showcase §8 | overview: deep benchmark |
| Trajopt full demo | overview | showcase: omit |
| Symbolic + engine depth | overview | showcase: omit |
| ODE solver matrix | simulation_benchmark | overview: optional trim |

---

## Execution order

1. **showcase** — §1 phase-plane; §5 LQR; delete §6 + heavy tail; fix TOC; no notebook links
2. **overview** — merge/trim; add operators, catalog, trajopt; standalone intro
3. **plots/animations** — standalone title only
4. **README** Examples table
5. Strip outputs; smoke-run light cells
