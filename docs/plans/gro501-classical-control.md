# GRO501 classical-control stack — fix plan (draft, 2026-09-07)

Status: **mostly landed.** P1, GRO-F1, GRO-F2, GRO-F4 (2026-09-07); P2, P3, P7 and TB-a (2026-09-26);
P5, P8 and P9 (2026-10-10); P4 (2026-10-14); S61 (2026-10-15). Open: P10 (now
automation-by-convention), TB-b; P6 (z tier) stays held; P11 dropped (2026-10-15). The order is in [TODO.md](TODO.md)
§2.
Rung: v0.2 wave B, steps P2–P10 of [TODO.md](TODO.md); P6's row sits in its v0.9 section.
Lane: teaching surface (`analysis/`, `control/`, `estimation/`, `blocks/`).
Milestone: v0.2, [ROADMAP §4.2](../../ROADMAP.md#42-v02--gro501-end-to-end).
Baseline audit: [../reviews/2026-09-07-gro501-coverage.md](../reviews/2026-09-07-gro501-coverage.md).

Acts on the ten items that audit found: four complexity findings (GRO-C1–GRO-C4) and
six coverage gaps (GRO-G1–GRO-G6); `GRO-` marks the audit's own ids (its C1, G1, F1, …),
apart from the workboard's. Nothing here removes a feature; every step is
additive or replaces boilerplate with generated equivalents.

## 1. Step map

| Step | Closes | Wave | Decision |
| --- | --- | --- | --- |
| **P1** PI / PD without phantom states | GRO-C4, GRO-G3 | 1 | **done** — ruled: dedicated classes |
| **GRO-F1** `frequency_range` brackets the 0 dB crossing | — | 1 | **done** |
| **GRO-F2** `PID.f` and `PID.ctl` agree on `tau` | — | 1 | **done** |
| **GRO-F4** `closed_loop_poles` singular-gain guard | — | 1 | **done** |
| **P2** `minreal` | GRO-G3 | 1 | **done** 2026-09-26; order reduction split out, then deferred out of v0.2 (2026-09-26; TODO §6) |
| **P3** `place()` → `StateFeedbackController` | GRO-G2 | 1 | **done** 2026-09-26 |
| **P4** `estimation/` — `StateObserver`, `luenberger`, `kalman`, `lqg` | GRO-G1 | 2 | landed 2026-10-14 |
| **P5** Named sensitivity functions | GRO-G5 | 2 | landed 2026-10-10 |
| **P6** Discrete (z) tier | GRO-G4 | 3 | **held** for GRO501; a v0.9 row |
| **P7** `facades.py` boilerplate | GRO-C2 | 2 | **done** 2026-09-26 (explicit + pinned) |
| **P8** Small teaching helpers (ζ/ω_n, `N`) | GRO-G6 | 2 | landed 2026-10-10 (`N` only) |
| **P9** `TransferFunction` port construction | GRO-C3 | 3 | landed 2026-10-10 |
| **P10** Document what `@` means | GRO-C1 | 3 | widened 2026-10-10 into [automation-by-convention.md](automation-by-convention.md) |
| **P11** Two GRO501 notebooks | — | 3 | **dropped** 2026-10-15: the students write the APP work themselves |

Wave 1 is the correctness wave and should land before any GRO501 material is
written: until it is in, the tools disagree with the course's own exercises.
P1, GRO-F1, GRO-F2 and GRO-F4 landed 2026-09-07 (suite 1 063 green, notebook and demos
re-run); P2 and P3 landed 2026-09-26. Wave 2 is the missing surface. Wave 3 is polish
and material.

GRO-F1, GRO-F2 and GRO-F4 came out of a second pass that ran the guide's §9.6 / §9.7 /
§9.10 systems through every verb — see
[§4 of the audit](../reviews/2026-09-07-gro501-coverage.md). GRO-F1 was the
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
construction, which is also what fixed GRO-F2.

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

**Landed 2026-09-26, as decided with the maintainer.** `minreal(A, B, C, D, *, tol=1e-9)`
lives on the matrices tier only (`analysis/linear.py`, imported from there): the Kalman
decomposition written step by step, the SVD, rank and projections in the body. There is no
`sys.minreal()` and no prelude name: it is the internal step of `pzmap`, `root_locus`,
`transfer_function`, `plot_pzmap` and `plot_root_locus`, which cancel **by default**
(`minimal=None`, with a one-time warning naming what cancelled; `minimal=False` keeps the
raw realization). The audit's pure-P loop `(10 s² + 200 s) / (s⁴ + 20.25 s³ + 9.905 s² +
98.1 s)` reduces to `10 / (s² + 0.25 s + 4.905)`; the `200 / (s² + 0.5 s + 4.905)` above
was not the audit's own H(s). A lead zero on a plant pole drops both; a minimal realization
(the badly scaled quarter car standing in for §9.13) comes back in its own coordinates.
`linear.zeros` now counts a pencil eigenvalue as finite only when `|β|` is not negligible
against `|α|`, so rounding no longer reports zeros near 1e15. Tests: `TestMinreal`.
Order reduction by neglecting fast modes is out of v0.2 (decided 2026-09-26; TODO §6).

### P3. `place()`

**Problem.** No pole placement. §1.4.2 eq. (16) asks for `K_sta` putting the
parking-mode poles at `{−1 ± 0.5i, −1}`; `place` is in the Lab 2 command list.

**Shape.** Mirror `lqr` exactly — the API precedent is settled:

```python
place_gain(A, B, poles)                  -> K
place(sys, poles, x_bar=...)             -> StateFeedbackController   (a linear model or a plant; arrays go to place_gain, 2026-10-14)
```

`scipy.signal.place_poles` does the algorithm; this is a thin wrapper plus
the block construction and an honest error when the pole set is not reachable
(repeated poles beyond the input rank).

**Files.** new `minilink/control/place.py` (folded into `control/design.py` on 2026-10-14), `minilink/control/__init__.py`,
`minilink/__init__.py`, `tests/unittest/test_control_analysis.py`, a demo in
`examples/demos/control/`.

**Done when.** `place` on the guide's parking model returns a `K` whose
closed-loop `eigvals(A − BK)` match `{−1 ± 0.5i, −1}` to 1e-9, and the
returned block closes the loop on the nonlinear plant with `@`.

**Landed 2026-09-26, amending "thin wrapper" (maintainer decision).** `place_poles`
refuses any pole repeated more than rank(B) times, so with one input it rejects
"critically damped" `[-2, -2]` and "all at −2". `place_gain` therefore splits as MATLAB's
`acker` / `place` do: with one input, Ackermann's formula `K = [0 … 0 1] 𝒞⁻¹ φ(A)`
written step by step (the unique gain, repeated poles included); with several, the
robust `place_poles` choice. Both check `det(sI − (A − BK)) = ∏(s − pᵢ)` by the
characteristic polynomial (a repeated pole's eigenvalues are computed only to about
`eps^(1/3)`), and the errors name the reason (uncontrollable pair, pole count, unpaired
complex pole, multiplicity beyond rank B). Demo `examples/demos/control/pole_placement_pendulum.py`;
tests `TestPolePlacement`. The parking model is not in the repo: a triple integrator
with `{−1 ± 0.5i, −1}` stands in.

---

## Wave 2 — the missing surface

### P4. `estimation/` — the observer and its designs — **landed 2026-10-14**

The design and its rulings: [estimation-and-state-feedback.md](estimation-and-state-feedback.md)
and [the 2026-10-14 record](../reviews/2026-10-14-state-space-design-rulings.md). What
landed: `StateObserver(A, B, C, L, D, xbar, ubar, ybar)` (ports `u`, `y` → `x`, the role
id `observer`), `luenberger` and `kalman` on arrays, a linear model or the plant (the duals
of `place` and `lqr`, which took the same three forms the same day), and `lqg`, the
observer + gain box with `r`, `y` → `u`. The composition question closed as the recipe by
hand plus `lqg()` for the design; `closed_loop(gain, plant, observer=obs)` landed with AC-4
(2026-10-14). The guide's
cart-pendulum check is `examples/demos/control/lqg_cartpole.py`; the Kalman observer alone,
`kalman_observer_pendulum.py`. The GRO501 from-scratch notebooks stay as they are.

### P5. Named sensitivity functions — design agreed 2026-09-26

**Landed 2026-10-10.** The module is `analysis/sensitivity_functions.py` (the name
`sensitivity.py` would shadow the facade's `sensitivity`); a piece with several
channels is refused; the loop inputs are `closed_loop(C, plant, r=True, w=False,
v=False)` (the maintainer's keyword); the teaching notebook is
`classical_control/sensitivity_functions.ipynb`, on the mass-spring-damper (one output, no
`Demux`), with a `Step` on `r` and `Sine` sources on `w` and `v`.

**Problem.** Table 2 states four specs as "sous −40 dB @ 0.015 Hz" style
bounds on disturbance and noise sensitivity. Today three of the four are
reachable only through the undocumented internal-wire selector, which needs
the loop's block names (`transfer_function(C @ G, of="error:e", wrt="r")`
for `S`, `of="ctl:u"` for `CS`); the load sensitivity is out of reach, and
simulating an input disturbance or sensor noise needs hand-wired `Sum` blocks.

**The loop and the four functions.** The book's notation: plant `H(s)`,
controller `C(s)`, optional filter `F(s)` in the return path (the measurement
passes through `F` before the comparison). Load disturbance `w` adds to the
plant input, measurement noise `v` adds to the measurement before `F`.

| Function | Formula | Reads |
| --- | --- | --- |
| `sensitivity` | `S = 1 / (1 + C H F)` | output disturbance to `y`; `e / r` |
| `complementary_sensitivity` | `T = C H / (1 + C H F)` | `y / r` |
| `load_sensitivity` | `PS = H / (1 + C H F)` | `w` to `y` |
| `noise_sensitivity` | `CS = C F / (1 + C H F)` | `v` to `u` |

With `F = 1`, `S + T = 1`.

**Rulings (maintainer, 2026-09-26).**

1. *The functions take the pieces, not the finished loop.* The formula is
   written in the body as on the page (the textbook rule), every function works
   for any plant and controller that linearize, the load sensitivity needs no
   change to loop building, and a nested loop passes the inner closed loop as
   the plant of the outer one. Options weighed: the finished loop
   (`sensitivity(C @ G)` reading its wires; any topology, but the formula
   hidden and the load sensitivity dependent on a disturbance input), both.
2. *Keyword-only, named arguments in the book's letters:*
   `sensitivity(*, plant, controller, filter=None, …)`, the same for the other
   three; the body unpacks `H, C, F` (each linearized to a transfer function,
   `F = 1` when absent) and names the loop gain `L = C H F` on its own line.
   `filter` is the book's word; it shadows the builtin only as a keyword name.
3. *Four descriptive names*, each returning an `LTISystem` so `plot_bode`,
   `margins` and the rest apply: `sensitivity`, `complementary_sensitivity`,
   `load_sensitivity`, `noise_sensitivity` (Åström–Murray; the course's
   "fonction de sensibilité", "sensibilité complémentaire"). No `gang_of_four`
   until a notebook needs the four together; no one-letter names (they clash
   with Python naming and with the `S`, `T` matrices elsewhere).
4. *Simulation gets optional loop inputs.* `closed_loop(controller, plant, …)`
   gains an option adding boundary inputs `w` (summed into the plant input)
   and `v` (summed into the measurement), off by default so every existing
   loop keeps its single input `r`. The names are the project's canonical
   disturbance and noise ports (RULES 4.9), the ports a `WhiteNoise` block
   connects to ([randomness.md](randomness.md)). The operator form `C @ H`
   takes no options; the explicit call does. Options weighed: a documented
   recipe with hand-wired `Sum` blocks, ports declared by the plant itself.
5. *A spec check needs nothing new.* `bode` already evaluates one frequency,
   in rad/s: `_, S_db, _ = bode(S, w=2 * np.pi * 0.015)` then `S_db[0] < -40`,
   the conversion `ω = 2πf` visible on the student's line. Options weighed: a
   scalar `magnitude_db(sys, w)`, a hertz keyword on every frequency tool.

**Settled when P5 landed (2026-10-10)**, kept here as its record. The operating point
of a nonlinear plant (`x_bar`, `u_bar`, `params` forwarded to its
linearization, as every analysis verb does); SISO only, a MIMO piece refused
with the channel named; the option keyword on `closed_loop` for the two
inputs; whether `feedback()` alone (no controller/plant split) offers only
`v`.

**Files.** A new `minilink/analysis/sensitivity_functions.py` (band functions, not
System shortcuts, ROADMAP §6), `minilink/analysis/__init__.py`,
`minilink/core/composition.py` (the loop inputs),
`tests/unittest/test_control_analysis.py`; DESIGN's analysis bullet.

**Done when.** `S + T = 1` holds to 1e-12 across the grid on a SISO loop
without a filter; each function matches the internal-wire transfer function of
the same loop (and `load_sensitivity` the `w`-to-`y` channel of a loop built
with the new inputs); each of the four Table 2 specs is one `bode` call plus a
comparison.

### P7. The analysis facades pinned to their band functions — **landed 2026-09-26**

Ruled 2026-09-26: the shortcuts stay written out (runtime generation would hide their
parameters from static editors such as VS Code/Pylance), and
`tests/unittest/test_system_shortcuts.py` pins each to its band function: parameters,
defaults, forwarding of every argument, and the docstring's target. A second test pins
the band's calling pattern `tool(sys, x_bar, u_bar, t, params, …)` with its named
exceptions (scan analysis#7). `analysis/linearize.py` and `discretize.py` became
`linearization.py` and `discretization.py`, so `from minilink.analysis import
linearize, discretize` returns the functions (scan analysis#11).

### P8. Small teaching helpers

**Landed 2026-10-10, reduced.** The maintainer dropped the ζ / ω_n verb and the `poles` verb
(two textbook lines on `np.linalg.eigvals`, written by the student). The `N` matrix is
`OutputReferenceController(K, N)` (split out of `StateFeedbackController`, 2026-10-14):
`r` is the output reference and `u = ubar - K (x - xbar) + N r`. `print` shows an
`LTISystem`'s matrices, a `TransferFunction`'s `G(s)` and a `StructuralResult`'s rank.

- ζ and ω_n from a complex pole pair, so §9.5's `tr ≈ 1.8/ω_n` and
  `ts ≈ 4.6/ζω_n` rules of thumb can be checked against `step_info`. Natural
  home: fields on the `pzmap` result or a `damping(sys)` verb next to
  `modal_analysis`.
- The `N` reference-scaling matrix giving `y = r` at steady state
  (§9.11 Q3) — a factory alongside `place` / `lqr`, or an `N=` argument on
  `StateFeedbackController`.

**Done when.** §9.5 and §9.11 are each a short notebook cell.

---

## Wave 3 — polish and material

### P6. Discrete (z) tier — **held**

*On hold by the maintainer 2026-09-07.*

**The open question**, recorded in [ROADMAP §6](../../ROADMAP.md#6-decisions).
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

P10 became [automation-by-convention.md](automation-by-convention.md), decided on
2026-10-11: port names drive the wiring, three reserved names (`plant`, `controller`,
`observer`) are the roles every tool reads, and `@` is one port-based dispatch. Its steps
and their "done when" are that plan's §7. The original scope here (a DESIGN table of the
three `@` paths, a test pinning the `e`-input autowire, a smaller `PROFILE_PORTS`) is
superseded: the profiles go, and the history is in
[the decision record](../reviews/2026-10-11-automation-by-convention.md).

### P11. Two classical control notebooks — dropped (2026-10-15)

Dropped by the maintainer: the APP2 propulsion loops and the APP4 bicycle autopilot are the
students' own work, so the repository ships no notebook for either
([the record](../reviews/2026-10-15-gro501-app-notebooks-dropped.md)). The tools they use
are the topic rows of ROADMAP §4.2, each with its demo or topic notebook;
`frequency_response.ipynb` stays the tool tour.

---

## 2. Sequencing

```
Wave 1   P1 ─┬─ P2                 correctness; blocks all GRO501 material
             └─ P3 ──┐
Wave 2   P7          ├─ P4 ── P5   the missing surface
                     └─ P8
Wave 3   P6? ── P9 ── P10          polish
```

P3 gates P4 (the Luenberger factory places poles on the dual pair). P1 and P2
gated all GRO501 material (until then the tools taught wrong pole counts); both are
in (P2 on 2026-09-26). P7 is independent and worth doing before P2
and P5 add three more facade methods each by hand. With P6 held and P11 dropped, what
remains is S62, then **TB-b** (the control objects and the loop)
([TODO.md](TODO.md) §2 holds the order; TB-a, P7, P5, P8, P9, RN-1, P4 and P10's
composition steps have landed).

## 3. What this plan does not do

- No feature removal. `PID` kept its full-state form and its behaviour;
  `Sum` keeps its role beside `Error`.
- `@` and `closed_loop` change under P10
  ([automation-by-convention.md](automation-by-convention.md)), not under this plan.
- No EKF, no time-varying Kalman, no Nichols, no multi-system overlay. Those
  stay roadmap rows.
- No hardware, ROS, or Arduino firmware: the course owns those.
