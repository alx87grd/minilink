# GRO501 classical-control stack — fix plan (draft, 2026-09-07)

Status: **wave 1 partly landed 2026-09-07.** P1 done (ruled: dedicated
`PI` / `PD` classes); F1, F2, F4 done (second-pass defects, see the audit).
P4 (estimation) and P6 (z tier) **held by the maintainer** — not scheduled.
P2, P3 and wave 2 open.
Lane: teaching surface (`analysis/`, `control/`, `estimation/`, `blocks/`).
Milestone: v0.2, [ROADMAP §4.2](../../ROADMAP.md#42-v02--gro501-end-to-end).
Baseline audit: [../reviews/2026-09-07-gro501-coverage.md](../reviews/2026-09-07-gro501-coverage.md).

Acts on the ten items that audit found: four complexity findings (C1–C4) and
six coverage gaps (G1–G6). Nothing here removes a feature; every step is
additive or replaces boilerplate with generated equivalents.

## 1. Step map

| Step | Closes | Wave | Decision |
| --- | --- | --- | --- |
| **P1** PI / PD without phantom states | C4, G3 | 1 | **done** — ruled: dedicated classes |
| **F1** `frequency_range` brackets the 0 dB crossing | — | 1 | **done** |
| **F2** `PID.f` and `PID.ctl` agree on `tau` | — | 1 | **done** |
| **F4** `closed_loop_poles` singular-gain guard | — | 1 | **done** |
| **P2** `minreal` and order reduction | G3, G6 | 1 | agent |
| **P3** `place()` → `StateFeedbackController` | G2 | 1 | agent (mirrors `lqr`) |
| **P4** `estimation/` — Luenberger, then Kalman | G1 | 2 | **held** |
| **P5** Named `S` / `T` / `PS` / `CS` | G5 | 2 | agent |
| **P6** Discrete (z) tier | G4 | 3 | **held** |
| **P7** `facades.py` boilerplate | C2 | 2 | agent |
| **P8** Small teaching helpers (ζ/ω_n, `N`) | G6 | 2 | agent |
| **P9** `TransferFunction` port construction | C3 | 3 | agent |
| **P10** Document what `@` means | C1 | 3 | agent (docs only) |
| **P11** Two GRO501 notebooks | gate 2 | 3 | maintainer reviews |

Wave 1 is the correctness wave and should land before any GRO501 material is
written: until it is in, the tools disagree with the course's own exercises.
P1, F1, F2 and F4 landed 2026-09-07 (suite 1 063 green, notebook and demos
re-run); P2 and P3 remain. Wave 2 is the missing surface. Wave 3 is polish
and material.

F1, F2 and F4 came out of a second pass that ran the guide's §9.6 / §9.7 /
§9.10 systems through every verb — see
[§4 of the audit](../reviews/2026-09-07-gro501-coverage.md). F1 was the
serious one: `margins()` reported `inf` for any PI or I loop and for any loop
with a large static gain, because the automatic frequency band was read from
the poles and zeros without checking where `|G| = 1`.

---

## Wave 1 — correctness

### P1. PI / PD without phantom states — **landed 2026-09-07**

**Ruled:** option B, dedicated classes. `ProportionalController` already
covers the stateless P case, so only `PI` and `PD` were missing.

**Problem.** `PID` declares `2n` states whatever the gains, so `Ki = 0` leaves
an integrator at `s = 0` and `Kd = 0` leaves a filter pole at `s = -1/tau`,
both unobservable. A pure P loop on a 2-state plant reports four poles, two
of them cancelled by zeros. Guide exercises §9.6 and §9.7 are exactly these
loops.

**Why not simply infer from the gain.** Dropping the state when `Ki == 0` at
construction would break `examples/demos/compile/pid_autotuning_jax.py`, which
starts at `Ki = 0.0` and tunes it *up* through `params`. Gains are tunable;
state dimension must not depend on a tunable value.

**Option A — a `states=` flag on `PID`.** One class, one extra keyword:
`PID(Kp=…, Ki=0.0, states="full")` (default, today's behaviour) or
`states="minimal"` (declare only what the nonzero gains need). Smallest
surface; the flag is one more thing to explain, and `"minimal"` silently
forbids raising `Ki` later.

**Option B — `P`, `PI`, `PD` as thin subclasses** *(recommended)*. Each
declares only the states its law needs and reuses `PID`'s `f` / `ctl` through
the existing `ErrorDriven` machinery; `PID` itself is untouched, so
autotuning keeps working unchanged. ~10 lines each. This is also the guide's
own vocabulary — page 7 asks students to judge "quel type de contrôleur
serait adapté (P, PI, PD, ..)" — so the names carry teaching weight rather
than adding jargon. Cost: three more names on the teaching surface.

**As built.** `PID` gained two class flags, `has_integrator` and
`has_filter`; the state layout, the params dict, the labels and `x0` follow
them, and the law is assembled term by term (`_command`, `_filter_rate`,
`_integrator_rate`). `PI` sets `has_filter = False`, `PD` sets
`has_integrator = False`; `PID` itself is unchanged in behaviour, so
`pid_autotuning_jax.py` still tunes `Ki` up from zero. On the pendulum:

| form | states | loop poles | loop zeros |
| --- | --- | --- | --- |
| `ProportionalController` | 0 | 2 | 0 |
| `PI` | 1 | 3 (one at the origin) | 1 |
| `PD` | 1 | 3 (one at −1/τ) | 1 |
| `PID` | 2 | 4 | 2 |

Every count now matches the hand calculation. `tau > 0` is validated at
construction, which is also what fixed F2.

**Files.** `minilink/control/siso.py`, `minilink/control/__init__.py`,
`minilink/__init__.py`, `tests/unittest/test_control_analysis.py`
(`TestCompensatorStateLayout`), `tests/unittest/test_teaching_surface.py`.

### P2. `minreal` and order reduction

**Problem.** No pole-zero cancellation anywhere in `analysis/`. The guide
names `minreal` in the Lab 2 command list, and §1.1 asks students to "réduire
l'ordre du système" by neglecting fast modes. P1 fixes the `PID` case
structurally; `minreal` fixes the general case (a compensator zero placed on a
plant pole, a cascade with a repeated mode).

**Shape.** In `analysis/linear.py`, on the matrices tier:
`minreal(A, B, C, D, tol=…) -> (A, B, C, D)`, cancelling pole/zero pairs
closer than `tol` — a Kalman decomposition keeping the controllable *and*
observable subspace. Then `sys.minreal()` on the system tier returning an
`LTISystem`, and a `tol` keyword on `transfer_function` / `pzmap` /
`root_locus` (default: no cancellation, so nothing changes silently).

Order reduction by neglecting fast modes is a separate verb —
`truncate(sys, keep=…)` or a `wn_max` cutoff on modal analysis. Propose it in
the same pass; keep it out of `minreal`, which must stay exact.

**Files.** `minilink/analysis/linear.py`, `minilink/analysis/frequency.py`,
`minilink/analysis/__init__.py`, `minilink/core/facades.py` (after P7),
`tests/unittest/test_control_analysis.py`.

**Done when.** The pre-P1 pure-P loop reduces to `200 / (s² + 0.5 s + 4.905)`;
a compensator zero cancelling a plant pole drops both from `pzmap`; the
guide's §9.13 realization survives `minreal` unchanged (it is already minimal).

### P3. `place()`

**Problem.** No pole placement. §1.4.2 eq. (16) asks for `K_sta` putting the
parking-mode poles at `{−1 ± 0.5i, −1}`; `place` is in the Lab 2 command list.

**Shape.** Mirror `lqr` exactly — the API precedent is settled:

```python
place(A, B, poles)                       -> StateFeedbackController
place_gain(A, B, poles)                  -> K
place_at_operating_point(sys, x_bar, poles, ...)  -> StateFeedbackController
```

`scipy.signal.place_poles` does the algorithm; this is a thin wrapper plus
the block construction and an honest error when the pole set is not reachable
(repeated poles beyond the input rank).

**Files.** new `minilink/control/place.py`, `minilink/control/__init__.py`,
`minilink/__init__.py`, `tests/unittest/test_control_analysis.py`, a demo in
`examples/demos/control/`.

**Done when.** `place` on the guide's parking model returns a `K` whose
closed-loop `eigvals(A − BK)` match `{−1 ± 0.5i, −1}` to 1e-9, and the
returned block closes the loop on the nonlinear plant with `@`.

---

## Wave 2 — the missing surface

### P4. `estimation/` — Luenberger, then Kalman — **held**

*On hold by the maintainer 2026-09-07. The design below stands for when it
is picked up.*

**Problem.** The largest GRO501 gap. `minilink/estimation/__init__.py` is a
docstring listing four planned modules and nothing else. The guide needs an
observer in §1.4.4 (bonus), Lab 2 steps 5–6, and oral-review deliverables
8–9 ("schéma bloc de votre proposition de filtre de Kalman" + a simulation).

**Shape.** The band's own docstring already fixes the port contract:
`(u, y) -> estimate`. Build it in that order:

1. `LuenbergerObserver(A, B, C, L)` — a `DynamicSystem` with state `x_hat`,
   `f = A x_hat + B u + L (y − C x_hat)`, output port `x_hat`. Ports `u`, `y`.
2. `luenberger(A, B, C, poles)` — design factory, `place` on the dual pair
   `(Aᵀ, Cᵀ)`. Depends on P3.
3. `kalman(A, B, C, Q, R)` — steady-state gain from the filter Riccati
   equation, returning the same block. `scipy.linalg.solve_continuous_are`,
   the same call `lqr_gain` already makes.

Deliberately *not* in this step: time-varying KF, EKF, discrete KF. They are
`estimation/` rows on the roadmap, not GRO501 blockers — the guide asks for a
steady-state design validated in simulation.

**The composition question is the part to rule on.** The observer must feed
`StateFeedbackController`'s `x` port while the plant's `y` feeds the observer:
that is a two-block loop `@` does not currently express. Either (i) a
`compensator(observer, controller)` helper returning the wired pair as one
block with ports `r`, `y` → `u`, or (ii) an explicit `add_subsystem` /
`connect` recipe in the notebook. Recommend (i): the guide's Figure 12 is
exactly that block, and observer-plus-state-feedback is the standard
output-feedback compensator, not a one-off wiring.

**Files.** new `minilink/estimation/luenberger.py`, `kalman.py`;
`minilink/estimation/__init__.py`; `minilink/__init__.py`;
new `tests/unittest/test_estimation.py`; demo in `examples/demos/control/`.

**Done when.** On the guide's cart-pendulum (Lab 2, eq. 17), an LQR plus a
Kalman observer stabilizes the nonlinear plant from a disturbed start with
noise injected on `u` and `y`, and the estimate converges to the true state;
`plot_diagram` shows the Figure 12 topology.

### P5. Named sensitivity functions

**Problem.** Table 2 states four specs as "sous −40 dB @ 0.015 Hz" style
bounds on disturbance and noise sensitivity. They are computable today only
through the internal-wire selector (`of="error:e"`, `of="ctl:u"`), which is
undocumented for this use, and an *input* disturbance needs a hand-wired `Sum`.

**Shape.** On a closed-loop diagram: `sensitivity(sys)` → `S = e/r`,
`complementary_sensitivity(sys)` → `T = y/r`, plus `PS` (disturbance to
output) and `CS` (noise to command), each returning an `LTISystem` so the
whole `plot_bode` / `margins` family applies. Where the diagram has no
disturbance port, say so in the error rather than guessing.

Also add a `disturbance=True` / `noise=True` option to `feedback()` (or a
documented recipe) so the injection points exist as named boundary inputs
instead of manual `Sum` blocks.

**Files.** `minilink/analysis/frequency.py` or a new
`minilink/analysis/loopshaping.py`, `minilink/analysis/__init__.py`,
`minilink/core/composition.py`, `tests/unittest/test_control_analysis.py`.

**Done when.** `S + T = 1` holds to 1e-12 across the grid on a SISO loop, and
each of the four Table 2 specs is one call plus a comparison.

### P7. Generate the analysis facades

**Problem.** ~400 of `facades.py`'s 1 228 lines are 13 hand-copied signatures
that forward unchanged. Already drifted once (the `settling_horizon`
docstring says five, the code says eight).

**Shape.** One helper that builds a delegating method from the target
function — signature copied with `functools.wraps` / `inspect.signature`, the
docstring taken from the target with a "See ..." line appended, `self` passed
as the first positional. Keep the explicit form only where the facade
genuinely differs from the function (`compute_trajectory`, `animate`).

Guard the readability cost: a test asserting every generated method's
`__signature__` matches its target keeps tab-completion and `help()` intact,
which is the reason the explicit form was written in the first place.

**Files.** `minilink/core/facades.py`, `minilink/core/facade.py`,
`tests/unittest/test_teaching_surface.py`.

**Done when.** The 13 analysis methods are generated; `help(sys.bode)` and
tab-completion are unchanged; the docstring drift is gone because there is
one source.

### P8. Small teaching helpers

- ζ and ω_n from a complex pole pair, so §9.5's `tr ≈ 1.8/ω_n` and
  `ts ≈ 4.6/ζω_n` rules of thumb can be checked against `step_info`. Natural
  home: fields on the `pzmap` result or a `damping(sys)` verb next to
  `modal_analysis`.
- The `N` reference-scaling matrix giving `y = r` at steady state
  (§9.11 Q3) — a factory alongside `place` / `lqr`, or an `N=` argument on
  `StateFeedbackController`.
- Fix the `settling_horizon` docstring mismatch (five vs eight) as part of P7.

**Done when.** §9.5 and §9.11 are each a short notebook cell.

---

## Wave 3 — polish and material

### P6. Discrete (z) tier — **held**

*On hold by the maintainer 2026-09-07.*

**The open question**, recorded in [ROADMAP §6](../../ROADMAP.md#6-review-queue).
§5.2.2 part 4 assigns Dorf 13.1–13.4 and §1.2 requires translating the
compensator to difference equations for the Arduino. Today `discretize` gives
Euler/RK4 *step models*, which simulate correctly but carry no z-domain
analysis.

**Option A — teach it with what exists.** Design in continuous time, discretize
with `discretize`, show by simulation that a fast enough sample rate preserves
the margins, hand-derive the difference equation. No new API. Honest, and it
is close to what the course actually asks (the Arduino code is the
deliverable, not a z-plane plot). Leaves Dorf 13 uncovered by tooling.

**Option B — a real z tier.** `discretize(..., method="zoh" | "tustin")`
returning an `LTISystem` in z, then z-plane `pzmap` (with the unit circle),
discrete Bode to the Nyquist frequency, discrete step response, discrete root
locus. This is a second copy of the analysis family with a different stability
boundary — the single largest addition on this plan, and the one most likely
to duplicate `analysis/linear.py` rather than reuse it.

Recommend A for v0.2 and re-open B once a cohort has run, unless the sommatif
examines z-plane analysis directly. **Maintainer's call — it depends on what
you examine, which the guide does not say.**

### P9. `TransferFunction` port construction

Replace the build-then-demolish in `__init__` (let `LTISystem` create the
plant ports, then clear and re-add) with a layout chosen before the ports are
declared. Behaviour-preserving; the four accepted `ports` values stay.

**Files.** `minilink/blocks/transfer_function.py`,
`minilink/dynamics/abstraction/state_space.py`,
`tests/unittest/test_blocks.py`.

**Done when.** No `self.inputs = {}` in the constructor; the existing
`TransferFunction` / `Lead` / `Lag` tests pass untouched.

### P10. Document what `@` means

C1 is not a bug and does not need a rewrite — the three dispatch paths each
read well and only two of them fire on a classical-control course. What is
missing is one place that says so. Add a short table to
[DESIGN.md](../../DESIGN.md) (operand shape → what `@` builds → the diagram
it produces) and collapse the five `PROFILE_PORTS` rows that differ only in
`plot_space` into one row plus a `plot_space` override, if that reads better
after P1 lands.

Also note in the same pass that `_is_controller_like` now fires on any block
with an `e` input — a widened `autowire` heuristic that no test pins.

**Done when.** DESIGN carries the table; a test pins the `e`-input autowire
behaviour so it cannot drift silently.

### P11. Two GRO501 notebooks

`examples/learn/teaching/gro501_app2_propulsion.ipynb` (multi-physics DC
motor + longitudinal dynamics → linearize → `H(s)` → root locus → PID to the
Table 2 specs → discrete implementation → nonlinear check with disturbance and
noise) and `gro501_app4_autopilot.ipynb` (bicycle model → controllability
across speeds → LQR on the guide's cost → nested loops → pole placement →
Kalman observer). Basic tier only, Colab-first, no JAX.

The existing `frequency_domain_tools.ipynb` stays as the tool tour; these two
are the course path. Maintainer owns student-facing material, so these are
drafted for review, not merged by the agent.

**Done when.** Both run top to bottom in Colab and in the conda env, import
only through the teaching surface, and the numbers agree with the guide's
worked exercises.

---

## 2. Sequencing

```
Wave 1   P1 ─┬─ P2                 correctness; blocks all GRO501 material
             └─ P3 ──┐
Wave 2   P7          ├─ P4 ── P5   the missing surface
                     └─ P8
Wave 3   P6? ── P9 ── P10 ── P11   polish, then the notebooks
```

P3 gates P4 (the Luenberger factory places poles on the dual pair). P1 and P2
gate P11 (until then the notebooks would teach wrong pole counts); P1 is in,
so P2 is the remaining blocker. P7 is independent and worth doing before P2
and P5 add three more facade methods each by hand. With P4 and P6 held, the
next actionable steps are **P2, P3, P5, P7, P8**.

## 3. What this plan does not do

- No feature removal. `PID` kept its full-state form and its behaviour;
  `Sum` keeps its role beside `Error`; the `closed_loop` heuristic path stays
  for the robotics courses.
- No change to the `@` dispatch itself — C1 is documented, not rewritten.
- No EKF, no time-varying Kalman, no Nichols, no multi-system overlay. Those
  stay roadmap rows.
- No hardware, ROS, or Arduino firmware: the course owns those.
