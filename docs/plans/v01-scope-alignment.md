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
| **Hybrid / MPC / `HybridDiagram`** | **Stop boasting `diagram = MPC @ plant` as a core `System` diagram.** It is simulation orchestration (`HybridSimulator`) for quick demos, not a core diagram yet. Long-term vision: MPC controller as `StepSystem`, then an official `HybridLoop` — deferred to v1.0. |
| **Continuous core** | North star for README, intro curriculum, and teaching surface: `DynamicSystem`, flow diagrams, `Simulator`, analysis on `f`. |
| **RL (GRO860)** | **`ReinforcementLearningPlanner`** (+ `NeuralPolicyController`, demos, intro ch. 11) = **primary course path**. `Sys2Gym` + `SB3Controller` = **bridge** (interfaces band); **remove from teaching notebooks/examples**; keep tests/docs noting interop. |
| **RRT & Lyapunov** | **Keep on teaching facades** (`minilink.planning`, `minilink.analysis`) to support Pitch Slide 3 and `showcase_from_rl_to_bode.ipynb`, but mark both as: **“Provisional — maintainer review before assigning for coursework.”** No demotion to `experimental/`. |
| **pip / PyPI** | Nice-to-have **after v0.1** to simplify Colab install; **not** a v0.1 gate (conda + git clone stay canonical until then). |
| **Examples layout** | Restructure to top-level **`examples/tutorial/`** (library curriculum, was `learn/intro/`) and **`examples/teaching/`** (subject lessons, was `learn/teaching/`). Perform an **atomic simultaneous update** of all Colab URLs, notebook headers, READMEs, and CI manifests so both GRO860 and GRO501 work seamlessly without broken links. |
| **Pre–v0.1 TRL cleanup** | Clarify provisional tools (RRT, Lyapunov, MPC orchestration) with maintainer review before teaching; keep stable core green. |

---

---

## Module-by-module checklist (one-by-one)

Work order: top to bottom. For each row: **Lane** = target after ruling;
**Action** = doc/code moves; **Sign-off** = maintainer checkbox when done.

| # | Module / area | Current ROADMAP lane | Target for v0.1 narrative | Recommended action | Sign-off |
| --- | --- | --- | --- | --- | --- |
| 1 | **Core + diagrams** (`core/system`, `diagram`, `wiring`) | teaching, TRL 7 | **Core** | Keep; emphasize in DESIGN/README as the center of gravity. | [x] |
| 2 | **Simulation** (`simulation/`, solvers, `Simulator`) | teaching, TRL 7 | **Core** | Keep; finish open gates (fixed output count, `verbose` names). | [x] |
| 3 | **Dynamics catalog + abstraction** | teaching, TRL 7 | **Core** | Keep; vehicle ladder research rungs stay in projects. | [x] |
| 4 | **Blocks** | teaching, TRL 5 | **Teaching surface** | Keep; v0.2 extras (Sine, Delay, …) unchanged. | [x] |
| 5 | **Control** (SISO, LQR, robotic, impedance) | teaching, TRL 6 | **Core / teaching** | Keep. | [x] |
| 6 | **Analysis** (jacobian, linearize, frequency, modal, structural) | teaching, TRL 6 | **Core / teaching** | Keep; GRO501 frequency rows stay v0.2. | [x] |
| 7 | **Analysis / Lyapunov** (`region_of_attraction`, `LyapunovCertificate`) | teaching, TRL 6 (landed 2026-09-11) | **Provisional teaching (TRL 4)** | **Keep in `analysis/` with facades and showcase §11 intact.** Mark as: *“Provisional — maintainer review before assigning for coursework.”* No demotion to `experimental/`. | [x] |
| 8 | **Compile** (`core/compile/`) | teaching frozen subset, TRL 4 | **Infrastructure (core path)** | Keep frozen subset; do not expand teaching story; S17 stays closed. | [x] |
| 9 | **Optimization** (`MathematicalProgram`, `Optimizer`) | teaching via trajopt, TRL 5 | **Teaching (GRO860)** | Keep; harden SciPy/Ipopt for TRL 6 when trajopt gates close. | [x] |
| 10 | **Planning / DP** | teaching GRO860, TRL 6 | **Core GRO860** | Keep; close metadata gates (`final_time`, `success`, notebook wiring). | [x] |
| 11 | **Planning / trajopt** | teaching GRO860, TRL 5 | **Core GRO860** | Keep; float64 + `success` semantics gates. | [x] |
| 12 | **Planning / RL** (`reinforcement_learning/`) | provisional → teaching | **Primary GRO860 RL tool** | **Promote:** register on teaching surface, `demos/rl/` + intro `11_*` canonical; rewrite teaching notebooks off SB3; align [rl-planner-vision.md](rl-planner-vision.md) R6–R7 (retire duplicate `experimental/ppo_jax` after parity). | [x] |
| 13 | **Interfaces / Gymnasium + SB3** | teaching GRO860 today | **Bridge only** | Keep module; **remove from GRO860 table as primary**; document as interop; no `teaching/` or intro cells that require SB3 for the course path. | [x] |
| 14 | **Planning / RRT + search** | provisional, TRL 5 | **Provisional teaching (TRL 5)** | **Keep on `minilink.planning` facade** (backs Pitch Slide 3 and demos). Mark as: *“Provisional — maintainer review before assigning for coursework.”* No demotion to `experimental/`. | [x] |
| 15 | **Geometry / spatial** (`planning/spatial/`, `Scene`, SDF) | provisional, TRL 4 | **Provisional research** | Keep in `planning/spatial/`; backs RRT obstacle scenes and upcoming CBF safety filter ([cbf-safety-filter.md](cbf-safety-filter.md)). | [x] |
| 16 | **Hybrid / step / MPC** (`StepSystem`, `Computer`, `HybridDiagram`, `control/mpc`) | provisional, TRL 5 | **Provisional simulation orchestration (TRL 4)** | **Reframe docs:** Stop boasting `diagram = MPC @ plant` as a core `System` diagram. Frame as simulation orchestration (`HybridSimulator`) for quick demos. Add a disclaimer banner to `06_hybrid.ipynb`. ROADMAP §6: official `HybridLoop` = v1.0+ vision. | [x] |
| 17 | **Graphics / animation** | teaching, TRL 5 | **Teaching** | Keep; supports core simulate/plot/animate story. | [x] |
| 18 | **Realtime** (`simulation/realtime/`) | provisional, TRL 2 | **Research / optional demo** | Keep code; do not present as core; architectural review stays open. | [x] |
| 19 | **Estimation** | planned GRO501, TRL 1 | **v0.2** | Scheduled for GRO501 wave: `LuenbergerObserver` and `KalmanFilter`. | [x] |
| 20 | **Identification** | planned, TRL 2 | **v0.2+** | No v0.1 change. | [x] |
| 21 | **C export** | research, TRL 2 | **Research** | Keep repo-only; pitch “experimental”. | [x] |
| 22 | **Experimental tier** (symbolic, engines, contact) | research, TRL 1 | **Research** | Isolated repo-only tier. | [x] |
| 23 | **`experimental/ppo_jax`** | research, TRL 2 | **Retire after RL promote** | Merge lessons into native planner; delete duplicate scripts when R7 done. | [x] |
| 24 | **External MJX leaf** | research, TRL 0 | **Research** | Unchanged. | [x] |
| 25 | **Pyro parity** | v0.2, TRL 3 | **v0.2** | Unchanged; do not expand v0.1 scope. | [x] |

**Provisional tagging pattern (rows 7, 14, 16):**
Mark in docstrings and ROADMAP ledger as:
> *“Provisional — maintainer review before assigning for coursework.”*
This keeps teaching facades, showcases, and the pitch deck fully functional without premature curricular commitments.

**Promotion pattern (row 12):** mirror demotion in reverse + GRO860 §4.1 row
rewrite (RL topic surface column).

---

## Examples folder restructure & two-lane layout

### The Two-Lane Layout

```
examples/
  # --- Teaching Lane (Pedagogical contract: curated, canonical, CI-checked) ---
  tutorial/                    # was learn/intro/ — numbered library API walk (00_core … 11_rl) + showcases
  teaching/                    # was learn/teaching/ — generic topic-first notebooks (no class codes)
    classical_control/         # Frequency domain, Bode/Nyquist, PID, state observers
    optimal_control/           # DP, value iteration, cartpole rollouts, trajopt
    reinforcement_learning/    # PPO, policy gradients, RL swing-up compares
    robotics/                  # Manipulator kinematics, Euler-Lagrange EoM, impedance
  demos/<chapter>/             # Canonical single-file textbook scripts (1:1 with tutorial chapters)

  # --- Research Lane (Repo-only, unconstrained, exploratory) ---
  projects/<name>/             # Multi-file applications (pathtracking, flight sim, active suspension)
  experimental/<topic>/        # Single-file WIP prototypes, engine checks, scratch
```

### Recommendation on `projects/` and `experimental/` vs `demos/`

1. **Do NOT place `projects/` or `experimental/` under `demos/`:**
   - **Purity of `demos/`:** `demos/` is minilink's canonical textbook script collection, mapped 1:1 to tutorial chapters. Every script in `demos/` is a single-file, open-and-run, zero-scaffolding pedagogical reference.
   - **Contract violation:** Putting multi-file projects or exploratory/unstable experimental scripts inside `demos/` mixes the teaching lane and research lane, confusing students and users.
   - **Single-file vs. Multi-file:** Demos are strictly single-file; projects are multi-file systems.

2. **Placement Recommendation:**
   - **Primary (Flat under `examples/`):** Keep `examples/projects/` and `examples/experimental/` at the root of `examples/`. Short paths (`from examples.projects.pathtracking ...`), minimal nesting, documented cleanly in `examples/README.md` under `## Research Lane`.
   - **Alternative (Strict physical isolation):** Group under `examples/research/` (`examples/research/projects/`, `examples/research/experimental/`) if strict physical directory separation from the teaching lane is desired.

### Course-Agnostic Teaching Rule (Universal Topic Modules)

- **Strictly course-agnostic:** Notebooks MUST NOT contain explicit university course codes or semester references (`GRO501`, `GRO860`, `MECH...`, "Assignment 3", "Lab 2") in filenames, titles, Markdown text, or exercise prompts.
- **Topical framing:** Each notebook is a standalone, reusable "Learn [Topic] with [System]" unit (e.g. *Frequency Domain Analysis of Underdamped Systems*, *Value Iteration vs. LQR on Pendulum Swing-Up*, *Equations of Motion for Articulated Manipulators*).
- **Course integration:** Syllabi and course portals link to these generic notebooks as building blocks. This keeps minilink cleanly reusable across multiple universities, courses, and self-study.

### Initial Mapping of the 8 Teaching Notebooks into 4 Categories

| Category | Notebook | Description |
| --- | --- | --- |
| `classical_control/` | `frequency_domain_tools.ipynb` | Frequency response, Bode/Nyquist, gain/phase margins |
| `optimal_control/` | `grid_world_exact_dp.ipynb` | Bellman equation & tabular dynamic programming |
| | `pendulum_swing_up_cost_function_vi.ipynb` | Value iteration & cost shaping for swing-up |
| | `pendulum_swing_up_vi_vs_lqr.ipynb` | Global dynamic programming vs. local linear quadratic regulation |
| | `cartpole_rollout_gradients.ipynb` | Direct shooting & trajectory optimization via autodiff rollouts |
| `reinforcement_learning/` | `drone_ppo_learn_to_fly.ipynb` | Deep RL policy gradients on continuous dynamical flight models |
| | `pendulum_swing_up_vi_vs_lqr_vs_ppo.ipynb` | Method comparison triad: DP vs. LQR vs. PPO |
| `robotics/` | `articulated_robot_eom.ipynb` | Euler-Lagrange equations, SymPy vs. JAX autodiff for manipulators |

*(As the curriculum grows to 30+ notebooks, new notebooks slot directly into these four topic buckets).*

### Migration Steps (Atomic commit series):

*Context: Both GRO860 (running) and GRO501 (starting next week) use this material. The migration must be executed atomically so no broken links occur.*

1. Document target layout in `examples/README.md` and `AGENTS.md`.
2. Move directories:
   - `git mv examples/learn/intro examples/tutorial`
   - Create subfolders under `examples/teaching/`: `classical_control/`, `optimal_control/`, `reinforcement_learning/`, `robotics/`
   - `git mv` each notebook from `examples/learn/teaching/` into its corresponding topic category.
3. Leave stub `README.md` redirect files in `examples/learn/intro/` and `examples/learn/teaching/` pointing to the new paths.
4. Atomically rewrite all Colab URLs (`/blob/main/examples/learn/…` → `/blob/main/examples/tutorial/…` and `/examples/teaching/<category>/…`) across all notebooks, `README.md`, `ROADMAP.md`, `docs/pitch/slides.html`, and the docs landing page.
5. Update `tests/demo_checks/run_notebook_checks.py`, `tests/demo_checks/notebook_overrides.json`, and `tests/unittest/test_teaching_imports.py`.
6. Update `ROADMAP.md` cross-cutting gate #3 path list.

---

## Doc edit sequence (Phase D extension)

Suggested order so ROADMAP stays plan of record:

1. **§1 North star** — continuous core sentence; hybrid/MPC as simulation orchestration, official `HybridLoop` deferred to v1.0.
2. **§3 TRL table** — apply checklist lanes (Lyapunov provisional TRL 4, RRT provisional TRL 5, RL promoted TRL 5, SB3 bridge, hybrid orchestration row).
3. **§4.1 GRO860** — RL row: native planner primary; drop SB3 from Material/Gate (bridge documented in §3 interfaces row).
4. **§6 Review queue** — close “RL course path” toward native planner; mark Lyapunov and RRT as provisional review; hybrid promotion explicitly **v1.0+ vision**.
5. **§7 Out of scope / subsidiary** — align with hybrid scaffold / simulation orchestration wording.
6. **DESIGN** — product identity: de-emphasize hybrid in “primary use cases”; clarify MPC orchestration.
7. **examples/README + AGENTS** — folder names; provisional tagging pointers.
8. **TODO.md** — tick items from this checklist as slices land.

---

## Decisions on former open questions (settled 2026-09-11)

1. **Lyapunov code home:** **Settled.** Kept under `analysis/` with teaching facade and showcase §11 intact; marked *“Provisional — maintainer review before assigning for coursework.”*
2. **Intro `06_hybrid.ipynb`:** **Settled.** Kept in sequence in `tutorial/06_hybrid.ipynb` with a clear disclaimer banner: digital loops and MPC are simulation orchestration utilities (`HybridSimulator`), not a core `System` block diagram yet.
3. **RRT demotion:** **Settled.** Kept in `minilink.planning` (TRL 5) to back Pitch Slide 3 and demos; marked *“Provisional — maintainer review before assigning for coursework.”*
4. **Re-open S17:** **Settled.** Stays closed; no dead evaluator method deletions; wording updated in docs only.

---

## Done when

- [x] Maintainer checked all 25 checklist rows and confirmed rulings.
- [x] ROADMAP §1–§4.1 + TRL + §6 reflect rulings.
- [x] This doc’s open questions answered with maintainer consensus.
- [x] Examples atomic migration executed (`learn/` → `tutorial/` and `teaching/`, URLs updated simultaneously).
- [ ] GRO860 RL path runs without SB3 in the canonical notebook set.
