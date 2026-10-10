# Feedback-loop composition review — 2026-10-10

Scope: step **P10** of the workboard ("document the three `@` dispatch paths, pin the
`e`-input heuristic, one keyword vocabulary for the four feedback wires"), widened at the
maintainer's request to a full census: every place a loop is built with a shortcut, every
place one is wired by hand, and which loop architectures are common enough to deserve a
shortcut. Read on `dev` at `ec6b9d9`; the defects marked *reproduced* were re-run by hand.

Constraint that frames every proposal: the continuous wiring dialect is frozen at five
entry points — `>>`, `+`, `@`, `DiagramSystem.connect()`, `feedback()` — plus `%` for the
hybrid algebra (CONSTITUTION §6, RULES 4.8: "do not add a seventh"). A new architecture
therefore arrives as a keyword on `closed_loop` (what `@` calls) or as a fix that lets `@`
accept more operands, never as a new operator.

## 1. How a loop is built today

`System.__matmul__` (`core/system.py:420`) calls `closed_loop(self, other)` with no
keywords. `closed_loop` (`core/composition.py:285`) then takes one of three paths:

| Path | Chosen when | Builds | Inserted blocks | Boundary |
| --- | --- | --- | --- | --- |
| **(a) return gain** | right operand is not a `System` (`L @ 1`, `L @ K`) | `feedback(L, through=K)` | `error`, `gain` (K ≠ 1), `demux` (scalar entry, vector output) | `r` → `y` |
| **(b) error junction** | left operand is error-driven: one input and no profile (`PID()`, `Lead`, `TransferFunction`, `P(ports="error")`, a plant, a `>>` diagram) | `series(C, G)` closed by `feedback()` | `error`, `demux` | `r` → `y` |
| **(c) two-port** | left operand declares measurement and reference ports (`r, y → u` or `x → u`) | `resolve_standard_feedback` | `mux` for `feedback="qdq"` | `r` → plant `y` (or `x`) |

The hybrid algebra is separate: `block % dt` makes a `Computer`, and `Computer @ plant`
(also `mpc @ plant`) goes through `resolve_hybrid_feedback_ports` → `hybrid_closed_loop`
(`core/hybrid_composition.py`). No Error, Gain, Mux or Demux is ever inserted there.

Options on top of the paths:

- `closed_loop(r=, w=, v=)` (path b and c): a visible `Sum` adds `w` at the plant input
  and `v` on the measurement; `r=False` drops the reference.
- `feedback(through=F)` puts a sensor or filter `F` in the return path; `sign=` weights it.
- `closed_loop(feedback="auto" | "y" | "qdq")` picks the measurement in path (c):
  `y` with equal dimensions, then `Mux(q, dq)`, then `x`.
- `closed_loop_qdq(...)` is `closed_loop(feedback="qdq")`.

The "`e`-input autowire heuristic" of the plan is `_is_controller_like`
(`composition.py:1366`). Its only caller is `_default_subsystem_id`: it decides that a
`Lead` is named `ctl` rather than `sys`. It does not touch `autowire`, which never fills
an `e` port except by exact name. The plan's wording ("a widened autowire heuristic") is
wrong; the rule is a naming rule.

## 2. Census

Executable call sites, definitions and prose excluded.

| Shortcut | Library | Tests | Demos | Tutorial | Teaching | Projects, experimental, bench, README, docs | Total |
| --- | --- | --- | --- | --- | --- | --- | --- |
| `@` between systems | 5 | 75 | 33 | 17 | 23 | 16 | **169** |
| `C >> G` loop gain (analysis) | 1 | 9 | 4 | 2 | 4 | 1 | **21** |
| `hybrid_closed_loop(` | 2 | 15 | 2 | 0 | 1 | 1 | **21** |
| `%` | 1 | 11 | 0 | 2 | 0 | 0 | **14** |
| `closed_loop(` | 2 | 11 | 0 | 0 | 1 | 0 | **14** |
| `closed_loop_qdq(` | 0 | 8 | 2 | 1 | 0 | 1 | **12** |
| `autowire(` | 1 | 5 | 3 | 1 | 0 | 0 | **10** |
| `feedback(` | 2 | 7 | 0 | 0 | 0 | 0 | **9** |

`@` is the language students use: 169 sites against 9 for `feedback()`. Loop shapes, by
frequency:

| # | Shape | Sites | Shortcut |
| --- | --- | --- | --- |
| 1 | State feedback, controller reads `plant.x` (LQR, place, DP, RL, lookup policies) | ~71 | `@` |
| 2 | Sampled computer in the loop (`% dt @`, `mpc @`, `hybrid_closed_loop`) | ~49 | `%`, `@` |
| 3 | Reference/measurement controller on `y` (impedance, `P`, `PID(ports="reference")`) | ~44 | `@` |
| 4 | Unity error feedback with a compensator (`PID @ G`, Error block, auto Demux) | ~24 | `@` |
| 5 | Robot loop on `[q; dq]` | ~21 | `closed_loop_qdq`, auto Mux |
| 6 | Open loop gain for analysis (`C >> G` into pzmap, Bode, root locus) | 15 | `>>` |
| 7 | Return path variants (`L @ 1`, `L @ K`, `feedback(through=F)`, `sign=`) | 13 | `@ K`, `feedback` |
| 8 | Cascade / nested loops | ~10 + ~9 hand-wired | partly (see D5) |
| 9 | Disturbance and noise inputs | 4 + ~11 hand-wired | partly (see §4) |

## 3. Defects

| Id | Severity | Defect | Evidence |
| --- | --- | --- | --- |
| **D1** | high | `(state_ctl >> Saturation()) @ plant` builds a **wrong loop silently**: the series diagram has one free input, so path (b) treats it as error-driven; `ctl.x` stays unconnected and `e = r − y` drives `ctl.r`. | *reproduced*; `tests/unittest/test_simulation.py:1146` builds exactly this and asserts only `sim.dt` |
| **D2** | high | Sampled state feedback is broken: `StateFeedbackController(K) % dt @ Pendulum()` raises `Unknown plant output port 'x'`. `_ensure_plant_boundary_ports` names the exposed port `output_port` while the channel uses `plant_out`. | *reproduced*; `hybrid_composition.py:327-361, 128` |
| **D3** | medium | The two PID layouts are not interchangeable: `PID() @ Pendulum()` closes on θ through a Demux, `PID(ports="reference") @ Pendulum()` raises (measurement dim 1 against `y` dim 2). Same for `ProportionalController()`. | *reproduced* |
| **D4** | medium | Path (b) ignores every port keyword, including `feedback=`: `closed_loop(PID(), Pendulum(), feedback="bogus")` is accepted. | *reproduced* |
| **D5** | medium | A nested diagram operand is refused in path (c): `P() @ ((P() @ I) >> I)` raises `Nested DiagramSystem operands are not supported`; it works only when every controller is error-driven. ROADMAP §4.2 lists "Nested loops — `@` composition: green (verified)". | *reproduced*; `composition.py:717-722` |
| **D6** | low | `PID(ports="reference") @ 1` closes the junction onto `r` and fails with an algebraic loop: `feedback()` picks its entry by port name, `closed_loop` by `error_input`. | probe |
| **D7** | low | `hybrid_closed_loop(...)` called directly resolves no port: it wires its literal defaults. DESIGN §4 (line ~271) says it shares the auto-wiring of `@`. | probe |
| **D8** | doc | The plan calls the `e`-input rule an autowire heuristic; it is the naming rule of §1. | `gro501-classical-control.md` P10, `TODO.md` |

## 4. Inconsistencies

**The four wires carry four vocabularies.**

| Wire | `closed_loop` | resolver / `StandardFeedbackWiring` | `hybrid_closed_loop` | controller declaration |
| --- | --- | --- | --- | --- |
| controller command | `control_port="u"` | `control_out=None` | `computer_out="u"` | `control_port` |
| plant input | `plant_input_port="u"` | `plant_in=None` | `plant_in="u"` | — |
| plant output | `plant_output_port="y"` | `plant_out=None` | `plant_out="y"` | — |
| controller measurement | `measurement_port="y"` | `measurement_in=None` | `computer_in="y"` | `measurement_port` |

`closed_loop` treats its literal defaults as "unset"; passing any one port forwards all
four and silently drops `feedback="qdq"`. `hybrid_closed_loop`'s names are used in 21
sites, one of them a live course notebook (`udes_gro501/racecar_toward_mpc.ipynb`), so a
rename needs aliases until v1.0 (ROADMAP §4.1 gate 7).

**The same filter has three names.** `feedback(through=F)`, `sensitivity(filter=F)`, and
`closed_loop` cannot take it at all.

**"error" has three meanings.** A feedback profile, an `ErrorDriven` port layout
(`ports="error"`) and a `plot_space`.

**`PROFILE_PORTS` (`core/feedback.py:43`).** Seven of eight rows share `(y, r, u)` and differ
only in `plot_space`; three pairs are exact duplicates (`error ≡ siso`,
`modelbased ≡ task`, `output ≡ kinematic`); `output` has no user.

**Defaults point opposite ways.** `PID` defaults to `ports="error"`,
`ProportionalController` to `ports="reference"`. With D3, the two P controllers a student
meets first behave differently on the same pendulum.

**Duplicated machinery.** The port fallback chains (`u → u_ff`, `y → x`, ...) are written
five times; boundary-port exposure four times (two copy labels and units, two do not);
leaf lookup three times.

## 5. Hand-wired loops

About 45 sites build a loop with `add_subsystem` / `connect` or a hand-written loop.

**A shortcut already builds about 20 of them.** Most are on purpose (the explicit API
taught in `00_core`, `diagram_closed_loop.py`, parity tests). Three would read better with
the shortcut: `vi_double_pendulum_jax.py:106` (its sibling `vi_quadratic.py` uses `@`),
`benchmarks/pyro_parity.py:106`, and the `__main__` of `core/diagram.py:280`.

**The rest hit real gaps:**

| Gap | Sites | Examples |
| --- | --- | --- |
| Sources or noise on the plant's own `w` / `v` / `f` ports; exposing a plant port or the command `u` on the loop | ~11 | `diagram_noise_ports.py`, `00_core` cell 27, `cartpole_dynamic_controller` cell 11, `11_reinforcement_learning` cell 24, `showcase_from_rl_to_bode` cell 11, `sensitivity_functions` cells 22 and 25 |
| Cascade with an `r, y`, state or impedance outer controller, or an outer loop around a closed loop (D5) | ~8 | `diagram_nested_loop.py`, `diagram_compiling.py`, `00_core` cell 24, `showcase_from_rl_to_bode` cell 19, the bicycle projects |
| Actuator block between a state-feedback controller and the plant (D1, D5) | 2 | `analysis_region_of_attraction.py`, `test_simulation.py:1146` |
| Multi-channel plant with named inputs (Demux to named ports, a loop on one channel) | 4 | `racecar_toward_mpc` cell 35, `test_mpc.py` ×2 |
| Computer-side composition (filter before the controller, multi-rate) | 6 | `hybrid_multi_rate.py`, `test_hybrid.py`, `mpc_dual_rate.py` |
| Two-degree-of-freedom feedforward + feedback | 4 | `bicycle_los`, `mpc_v1`, `test_diagrams.py:255` |
| Observer + state feedback | 1 | `cartpole_dynamic_controller` cell 7 (P4 will provide the blocks) |
| Pure discrete `StepDiagramSystem` loop | 3 | `step_unity_feedback.py`, `06_hybrid` cell 5 |
| Closed-loop rollouts re-implemented for `jit` / `vmap` | 4 | `planning/evaluation.py`, the RL planner (RN-5's batched rollout) |

## 6. Architectures that earn a shortcut

Ranked by what students write, inside the frozen dialect.

| Rank | Architecture | Today | Proposal |
| --- | --- | --- | --- |
| 1 | Single loop: state, `r/y`, unity compensator, qdq | `@` | keep; fix D1–D4 so every controller layout closes the same way on the same plant |
| 2 | Sampled computer in the loop | `% dt @` | keep; fix D2 |
| 3 | Loop driven by signals: a reference, a load disturbance, sensor noise | `closed_loop(r=, w=, v=)` gives ports only | let each flag take a source block as well as a bool: `closed_loop(C, plant, r=Step(...), w=Sine(...), v=WhiteNoise(...))` wires the source in; `True` keeps the boundary port |
| 4 | Cascade and actuator chains: `outer @ (inner @ plant)`, `ctl @ (sat >> plant)` | fails for `r/y` and state controllers (D5) | no new syntax: let path (c) accept a diagram operand (inline it, as path b does) |
| 5 | Sensor or filter `F` in the return path | `feedback(through=F)` only for error-driven loops | `closed_loop(..., sensor=F)` on both paths, the same word on `feedback` and the sensitivity functions |
| 6 | Observer-based state feedback | none | P4: the observer is a block, `K @ (observer, plant)` or a `compensator(observer, K)` factory decided with P4 |
| 7 | Two-degree-of-freedom feedforward | none | defer: 4 sites, all research projects |
| 8 | Discrete-only loops, computer-side composition, multi-channel adapters | none | defer: research lane, below the frequency that earns syntax |

Rank 3 is the one the sensitivity notebook needed this week; rank 4 removes the most hand
wiring; ranks 1 and 2 are bugs, not features.

## 7. Proposed P10 scope

**Agent lane, no public name changes:**

1. DESIGN: the dispatch table of §1 (operand shape → path → blocks → boundary) and the
   naming rule of `_is_controller_like`; correct DESIGN's hybrid claim (D7) and ROADMAP's
   "nested loops green" (D5).
2. Tests that pin each row of the dispatch table, and regression tests for D1–D4.
3. Fixes:
   - **D1:** a series diagram whose entry block is a two-port controller goes to path (c),
     or is refused with a message; never wired silently.
   - **D2:** the hybrid plant boundary uses `plant_out` for both the exposed port and the channel.
   - **D4:** path (b) validates `feedback=` and refuses port keywords it does not honour.
   - **D6:** `feedback()` picks its entry with `error_input`, like `closed_loop`.
4. `PROFILE_PORTS`: collapse the rows that differ only in `plot_space`, keeping the
   profile names as aliases.

**Maintainer decisions:**

- **D3:** should a reference-layout controller take component 0 of a vector output, as
  the error layout does? Or should `PID` and `ProportionalController` share one default
  layout?
- **The vocabulary of the four wires.** One set of names across `closed_loop`,
  `hybrid_closed_loop` and the resolver, the old names kept as aliases until v1.0.
- **Rank 3:** a source block as the value of `r=`, `w=`, `v=`.
- **Rank 4:** diagram operands in path (c), so cascades and actuator chains use `@` (core).
- **Rank 5:** one name for the return-path filter (`sensor=` proposed).
