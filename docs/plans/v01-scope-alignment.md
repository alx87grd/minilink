# v0.1 scope alignment — docs, lanes, examples

**Status:** draft for maintainer review (2026-09-11).  
**When approved:** apply the rulings to [ROADMAP.md](../../ROADMAP.md) (§1, §3 TRL,
§4.1, §5 phases, §6 review queue, §7), [DESIGN.md](../../DESIGN.md) (product
identity, hybrid/MPC framing), [examples/README.md](../../examples/README.md),
[AGENTS.md](../../AGENTS.md) (examples buckets), pitch/showcase only where
they claim teaching-core status.

This plan encodes the maintainer direction from the editorial review follow-up:
**continuous `System` / flow diagrams are the library core**; hybrid/MPC stays
a research scaffold until MPC is a first-class step path; **native RL becomes
the GRO860 teaching tool** with Stable-Baselines3 as an optional bridge only;
**low-TRL features demote or promote one-by-one before v0.1**; **pip after
v0.1**; **examples tree** gains top-level `tutorial/` and `teaching/`.

---

## Rulings to land in ROADMAP (summary)

| Topic | Decision |
| --- | --- |
| **Hybrid / MPC / `HybridDiagram`** | Quick research scaffold to exercise MPC, **not** library core narrative. Long-term vision: MPC controller as `StepSystem`, then a promoted official hybrid loop — **not scheduled for v0.1–v0.2**; track under §6 / §7 and research lane only until then. |
| **Continuous core** | North star for README, intro curriculum, and teaching surface: `DynamicSystem`, flow diagrams, `Simulator`, analysis on `f`. |
| **RL (GRO860)** | **`ReinforcementLearningPlanner`** (+ `NeuralPolicyController`, demos, intro ch. 11) = **primary course path**. `Sys2Gym` + `SB3Controller` = **bridge** (interfaces band); **remove from teaching notebooks/examples**; keep tests/docs noting interop. |
| **pip / PyPI** | Nice-to-have **after v0.1** to simplify Colab install; **not** a v0.1 gate (conda + git clone stay canonical until then). |
| **Examples layout** | Restructure to top-level **`examples/tutorial/`** (library curriculum, was `learn/intro/`) and **`examples/teaching/`** (subject lessons, was `learn/teaching/`). Update Colab URLs, CI smoke paths, README badges in a dedicated migration step. |
| **Pre–v0.1 TRL cleanup** | Walk the module checklist below **one row at a time**; default for unvalidated recent landings: **demote to `experimental/`** (code + demos + teaching-surface exports) until maintainer sign-off, then promote back. |

---

---

## Module-by-module checklist (one-by-one)

Work order: top to bottom. For each row: **Lane** = target after ruling;
**Action** = doc/code moves; **Sign-off** = maintainer checkbox when done.

| # | Module / area | Current ROADMAP lane | Target for v0.1 narrative | Recommended action | Sign-off |
| --- | --- | --- | --- | --- | --- |
| 1 | **Core + diagrams** (`core/system`, `diagram`, `wiring`) | teaching, TRL 7 | **Core** | Keep; emphasize in DESIGN/README as the center of gravity. | [ ] |
| 2 | **Simulation** (`simulation/`, solvers, `Simulator`) | teaching, TRL 7 | **Core** | Keep; finish open gates (fixed output count, `verbose` names). | [ ] |
| 3 | **Dynamics catalog + abstraction** | teaching, TRL 7 | **Core** | Keep; vehicle ladder research rungs stay in projects. | [ ] |
| 4 | **Blocks** | teaching, TRL 5 | **Teaching surface** | Keep; v0.2 extras (Sine, Delay, …) unchanged. | [ ] |
| 5 | **Control** (SISO, LQR, robotic, impedance) | teaching, TRL 6 | **Core / teaching** | Keep. | [ ] |
| 6 | **Analysis** (jacobian, linearize, frequency, modal, structural) | teaching, TRL 6 | **Core / teaching** | Keep; GRO501 frequency rows stay v0.2. | [ ] |
| 7 | **Analysis / Lyapunov** (`region_of_attraction`, `LyapunovCertificate`) | teaching, TRL 6 (landed 2026-09-11) | **Research / experimental until validated** | **Demote:** move public API to `minilink/experimental/` (or keep implementation under `analysis/` but **drop from root prelude + teaching surface**); move `analysis_region_of_attraction.py` → `examples/experimental/`; trim showcase §11 claims until re-promoted; update TRL to 3–4. Re-promote after D1–D5 review + course placement decision. | [ ] |
| 8 | **Compile** (`core/compile/`) | teaching frozen subset, TRL 4 | **Infrastructure (core path)** | Keep frozen subset; do not expand teaching story; optional re-open S17 only if maintainer wants smaller surface. | [ ] |
| 9 | **Optimization** (`MathematicalProgram`, `Optimizer`) | teaching via trajopt, TRL 5 | **Teaching (GRO860)** | Keep; harden SciPy/Ipopt for TRL 6 when trajopt gates close. | [ ] |
| 10 | **Planning / DP** | teaching GRO860, TRL 6 | **Core GRO860** | Keep; close metadata gates (`final_time`, `success`, notebook wiring). | [ ] |
| 11 | **Planning / trajopt** | teaching GRO860, TRL 5 | **Core GRO860** | Keep; float64 + `success` semantics gates. | [ ] |
| 12 | **Planning / RL** (`reinforcement_learning/`) | provisional → teaching | **Primary GRO860 RL tool** | **Promote:** register on teaching surface, `demos/rl/` + intro `11_*` canonical; rewrite teaching notebooks off SB3; align [rl-planner-vision.md](rl-planner-vision.md) R6–R7 (retire duplicate `experimental/ppo_jax` after parity). | [ ] |
| 13 | **Interfaces / Gymnasium + SB3** | teaching GRO860 today | **Bridge only** | Keep module; **remove from GRO860 table as primary**; document as interop; no `learn/teaching` or intro cells that require SB3 for the course path. | [ ] |
| 14 | **Planning / RRT + search** | provisional, TRL 5 | **Experimental until validated** | **Demote:** `RRTPlanner`, extenders, `demos/planning/rrt/` → research lane / experimental examples; drop from root `planning` teaching exports if present; TRL 3–4 until maintainer re-validates. | [ ] |
| 15 | **Geometry / spatial** (`planning/spatial/`, `Scene`, SDF) | provisional, TRL 4 | **Experimental** (tied to RRT/MPC research) | **Demote** with RRT; keep code for projects (`car_trajopt`, MPC stacks); not intro curriculum. | [ ] |
| 16 | **Hybrid / step / MPC** (`StepSystem`, `Computer`, `HybridDiagram`, `control/mpc`) | provisional, TRL 5 | **Research scaffold; long-term vision** | **Reframe docs:** not core product; `06_hybrid` intro notebook → optional/advanced or research-only pointer; `demos/hybrid/`, `demos/mpc/` stay runnable but **out of v0.1 contract**; ROADMAP §6: official hybrid = v1.0+ vision (MPC as step system first). No short-term schedule. | [ ] |
| 17 | **Graphics / animation** | teaching, TRL 5 | **Teaching** | Keep; supports core simulate/plot/animate story. | [ ] |
| 18 | **Realtime** (`simulation/realtime/`) | provisional, TRL 2 | **Research / optional demo** | Keep code; do not present as core; architectural review stays open. | [ ] |
| 19 | **Estimation** | planned GRO501, TRL 1 | **v0.2** | No v0.1 change. | [ ] |
| 20 | **Identification** | planned, TRL 2 | **v0.2+** | No v0.1 change. | [ ] |
| 21 | **C export** | research, TRL 2 | **Research** | Keep repo-only; pitch “experimental”. | [ ] |
| 22 | **Experimental tier** (symbolic, engines, contact) | research, TRL 1 | **Research** | Receives demoted Lyapunov/RRT/spatial **examples**; code placement per row 7/14/15. | [ ] |
| 23 | **`experimental/ppo_jax`** | research, TRL 2 | **Retire after RL promote** | Merge lessons into native planner; delete duplicate scripts when R7 done. | [ ] |
| 24 | **External MJX leaf** | research, TRL 0 | **Research** | Unchanged. | [ ] |
| 25 | **Pyro parity** | v0.2, TRL 3 | **v0.2** | Unchanged; do not expand v0.1 scope. | [ ] |

**Demotion pattern (rows 7, 14, 15):** for each, in one PR-sized slice:

1. Ruling recorded in ROADMAP TRL row + §6.
2. Teaching-surface registry (`minilink/__init__.py`, band facades) updated.
3. Examples moved or relabeled (demos → `experimental/` where appropriate).
4. Tests: keep unit tests in repo; optional `@pytest.mark.research` if CI
   should not gate teaching on demoted APIs.
5. DESIGN one-line pointer: “experimental until promoted.”

**Promotion pattern (row 12):** mirror demotion in reverse + GRO860 §4.1 row
rewrite (RL topic surface column).

---

## Examples folder restructure (planned)

**Target tree:**

```
examples/
  tutorial/          # was learn/intro/ — numbered API + showcases
  teaching/          # was learn/teaching/ — subject / course notebooks
  demos/<chapter>/   # unchanged role
  projects/
  experimental/
  tooling/
```

**Migration steps (separate commit series):**

1. Document target in `examples/README.md` + AGENTS.md examples buckets.
2. `git mv examples/learn/intro → examples/tutorial` and
   `git mv examples/learn/teaching → examples/teaching`.
3. Remove empty `learn/` or leave a stub README redirect for one release.
4. Global replace Colab `/blob/main/examples/learn/…` URLs (README, notebooks,
   docs/pitch, CI notebook manifest).
5. Update `tests/demo_checks/run_notebook_checks.py` paths and
   `test_teaching_imports` if paths are hard-coded.
6. ROADMAP cross-cutting gate #3 path list.

**Naming note:** “tutorial” = learn **minilink**; “teaching” = learn a **subject**
(same semantics as today, clearer top-level folders).

---

## Doc edit sequence (Phase D extension)

Suggested order so ROADMAP stays plan of record:

1. **§1 North star** — continuous core sentence; hybrid/MPC as long-term, not
   short-term plan.
2. **§3 TRL table** — apply checklist lanes (Lyapunov, RRT/spatial, RL, SB3,
   hybrid rows).
3. **§4.1 GRO860** — RL row: native planner primary; drop SB3 from Material/Gate
   (bridge documented in §3 interfaces row).
4. **§6 Review queue** — close “RL course path” toward native planner; move
   Lyapunov to experimental promotion track; hybrid promotion explicitly
   **v1.0+ vision** (MPC step system prerequisite).
5. **§7 Out of scope / subsidiary** — align with hybrid scaffold wording.
6. **DESIGN** — product identity: de-emphasize hybrid in “primary use cases”;
   keep honest subsidiary paragraph.
7. **examples/README + AGENTS** — folder names; demotion/promotion pointers.
8. **TODO.md** — tick items from this checklist as slices land.

Do **not** rewrite intro notebooks or demos until the corresponding checklist
row is signed off (avoid half-migrated teaching material).

---

## Open questions for maintainer (before ROADMAP merge)

1. **Lyapunov code home:** move implementation to `experimental/analysis/` vs
   keep under `analysis/` with exports removed only?
2. **Intro `06_hybrid.ipynb`:** archive to `tutorial/advanced/` vs keep in
   sequence with a “research lane” banner?
3. **RRT demotion:** delete from `minilink.planning` public exports only, or
   also move `planning/search/` behind `experimental/`?
4. **Re-open S17** (evaluator dead methods) or only fix ROADMAP wording?

---

## Done when

- [ ] Maintainer checked all 25 checklist rows (or edited recommendations).
- [ ] ROADMAP §1–§4.1 + TRL + §6 reflect rulings.
- [ ] This doc’s open questions answered in ROADMAP §6 or inline edits here.
- [ ] Examples migration either scheduled in TODO with dates or completed.
- [ ] GRO860 RL path runs without SB3 in the canonical notebook set.
