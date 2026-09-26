# Analysis toolbox audit: the classical-control modules against the textbook rule

**Date:** 2026-09-26
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Branch `dev` @ `dcc132a`, TB-a step 2 of `docs/plans/TODO.md`, after the
shortcut cut of step 1 (`4412a18`; the kept shortcut list is not revisited here). Scope, read
line by line: `minilink/analysis/linear.py`, `frequency.py`, `time_response.py`,
`structural.py`, `linearize.py`, `modal.py`.
**Method:** the §4.1 method of the 2026-09-22 consolidation review, under AGENTS "The
textbook rule": RULES 5.3 (three beats, one equation per named line, no equation on
`return`), 5.8 (plain helper names), 5.22 (equation comments, never in docstrings), 5.10 and
5.23 (section order, no preamble), and the maintainer's standing preferences (a blank line and
a one-line comment per step; every linear-algebra step a named line in the body; plumbing in
helpers under `# Internal machinery`; timeless docstrings). Every `file:line` below was read
at `dcc132a`; every bug was reproduced with a short read-only snippet.
**Reference:** `minilink/planning/policy_synthesis/dp.py`.
**Status:** ruled 2026-09-26: decisions 1–4 of §8 yes; the three bugs and X-5's rule land as a
separate step (3b) after the behaviour-preserving rewrite (3a).

Kinds: **behaviour-preserving** is safe under the seeded-baseline byte-identical recipe
(AGENTS, RULES 7.7); **public name [ask]** and **returned type [ask]** wait on a ruling;
**bug** changes behaviour and gets its own test.

---

## 1. `analysis/linear.py` (421 lines)

**Verdict: close.** `minreal` (linear.py:118-151) already reads like `dp.py`: each SVD, rank
and projection is a named line under its comment. The rest is mostly right in order but
keeps textbook steps in helpers (`margins`), several steps on one line (`gain`), and eleven
underscore helpers.

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| LIN-1 | linear.py:279-421 | Eleven module helpers carry a leading underscore (`_bracket_unit_gain`, `_matrices`, `_sign_changes`, `_interpolate`, `_phase_margin_from_samples`, `_gain_margin_from_samples`, `_open_loop_radius`, `_default_gains`, `_refine_jumps`, `_gain_reaching`, `_matched`). | Plain names (`as_matrices`, `bracket_unit_gain`, `sign_changes`, `interpolate_crossing`, `gain_crossovers`, `phase_crossovers`, `open_loop_radius`, `default_gains`, `refine_jumps`, `gain_reaching`, `matched_branches`); `_INFINITE_ZERO_RATIO` keeps its underscore (a constant). No caller outside the module. | behaviour-preserving |
| LIN-2 | linear.py:176-180, 332-357 | `margins` shows the textbook formulas only as comments; the formulas themselves sit in the helpers: `margin = (phase_c + 180.0 + 180.0) % 360.0 - 180.0` (339) and `gain_c = -np.interp(...)` (354). The comment "wrapped to (−180, 180]" (176) is also off: the fold gives [−180, 180), so ∠L = 0° at crossover reads −180, not 180. | Helpers return only the crossings (`w_gc` with the interpolated `∠L(jω_gc)`, `w_pc` with the interpolated `|L(jω_pc)|_dB`); the body writes `PM = 180 + ∠L(jω_gc)` and `GM = −|L(jω_pc)|_dB` as named lines, then keeps the smallest. Fix the interval in the comment. | behaviour-preserving |
| LIN-3 | linear.py:65-72 | `gain`: two computations on one line (`z, p = zeros(A, B, C, D), poles(A)`); `G0` is assembled across a branch (`G0 = D[0, 0]` then `if A.size: G0 += ...`). The branch is unneeded: `np.linalg.solve` on a 0×0 system returns an empty column (checked). | `z = zeros(...)`, `p = poles(A)`, `s0 = 2.0 * open_loop_radius(...)` (see X-4), then `# G(s0) = C (s0 I − A)⁻¹ B + D` over `G_s0 = (C @ np.linalg.solve(s0 * I - A, B) + D)[0, 0]`, then `k = G_s0 * np.prod(s0 - p) / np.prod(s0 - z)`. Confirm by `cmp` (addition order `D + x` vs `x + D` is exact in IEEE). | behaviour-preserving |
| LIN-4 | linear.py:81-89, 110 | `frequency_response` keeps an empty-`A` early return that the general line already covers (checked: `[2.+0.j]` both ways); `return G` has no blank line. `frequency_range` ends on `return _bracket_unit_gain(...)`: the widening is a step of the algorithm, not output. | Drop the early return; `w_min, w_max = bracket_unit_gain(...)` as a named line under "widen until the band brackets \|G\| = 1", then `return w_min, w_max`. | behaviour-preserving |
| LIN-5 | linear.py:21-29, 188-198 | `poles` binds a local `poles` inside `def poles`; `closed_loop_poles` does the same, so the student reads `poles = np.linalg.eigvals(...)` where `poles(...)` is also a function in scope. | `p = np.linalg.eigvals(A)` in both (the textbook `p`, as `gain` already uses). | behaviour-preserving |
| LIN-6 | linear.py:236-254 | `step_response` validates the grid in the body (six lines of `ValueError` plumbing before the math); the march has no blank line before `return y`. | Helper `uniform_step(t)` returning `dt` under `# Internal machinery`; the body is the ZOH pair (`hold`, `A_d, B_d`), the march, `return y`. | behaviour-preserving |
| LIN-7 | linear.py:201, 212, 367 | `root_locus(..., *, n=400)`: `n` is the number of gains here and in `_default_gains(A, B, C, D, n)`, but the state dimension everywhere else (RULES 5.4; the same file uses `n, m = B.shape` at 39 and 237). Nobody passes it (grep). | Rename the keyword `n_gains`. | public name [ask] |
| LIN-8 | linear.py:188-198, 413-421 | `root_locus` crashes when a caller's `gains` reach `K = −1/d`: `closed_loop_poles` returns an all-`inf` array (195) and `_matched` hands it to `linear_sum_assignment`, which raises `ValueError: cost matrix is infeasible`. Reproduced with `TransferFunction([1, 1], [1, 2])`, `gains=[0, -0.5, -1, -2]`. The default sweep is positive, so only user gains hit it. | `matched_branches` returns `current` unchanged when it is not finite (the branch simply breaks at the singular gain). Test: the sweep above returns four rows. | bug |

## 2. `analysis/frequency.py` (712 lines)

**Verdict: close.** The public verbs are thin and correctly ordered (reduce to a channel,
minimal realization, call `linear`). Three things keep it from `dp.py`: the Bode definition
and the damping formulas live in plot helpers, a return line carries three computations, and
the channel-selection plumbing sits in a section of its own between the API and the
machinery, imported by `time_response`.

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| FRQ-1 | frequency.py:112, 513-516 | `bode` calls `_bode_coordinates(G)`: the definition of a Bode plot (`20 log10 \|G\|`, unwrapped `∠G`) is hidden in a helper, which RULES 5.3 names as the wrong place. `_bode_figure` (581) calls it a second time on the same `G`. | Two named lines in `bode` (`magnitude_db = 20.0 * np.log10(np.abs(G))` under `np.errstate`, `phase_deg = np.degrees(np.unwrap(np.angle(G)))`); `plot_bode` does the same and passes both to the figure helper. See X-3. | behaviour-preserving |
| FRQ-2 | frequency.py:198 | `pzmap` computes zeros, poles and gain on the `return` line. | `z = linear.zeros(...)`, `p = linear.poles(A)`, `k = linear.gain(...)`, blank line, `return z, p, k`. | behaviour-preserving |
| FRQ-3 | frequency.py:302, 312, 126 | `plot_bode` re-derives `margins()` inline with a copied constant (`frequency_grid(A, B, C, D, w, 2000)  # the grid of margins()`); the docstring repeats 2000. The keyword `margins: bool` also shadows the module function `margins` inside `plot_bode`: harmless today, a trap for the next edit. | One module constant (`MARGIN_GRID_POINTS = 2000`) read by `margins`' default and by `plot_bode`; the kwarg stays (public, and the body never calls `margins()`). | behaviour-preserving |
| FRQ-4 | frequency.py:415-505 | A `# Channel helpers` section (seven plain-named plumbing functions: `siso_channel`, `channel_label`, `channel_subtitle`, `siso_matrices`, `minimal_channel`, `caller_stacklevel`, `frequency_grid`) sits between the public API and `# Internal machinery` (RULES 5.10). None is used outside `analysis/` (grep). | Move the selection part to `linearize.py` (X-2); the rest goes under `# Internal machinery`. | behaviour-preserving |
| FRQ-5 | frequency.py:19, 486-494 | `import sys` (stdlib) in a module where every public function takes a parameter named `sys`; `caller_stacklevel` works only because it has no such parameter. A student reading `sys._getframe(1)` next to `sys.name` will stumble. | `import inspect` and `inspect.currentframe().f_back`, or `from sys import _getframe`; no name clash left. | behaviour-preserving |
| FRQ-6 | frequency.py:551-555 | `_root_text` holds textbook formulas in a hover-text helper: `damping = float(-s.real / abs(s))` (ζ = −Re s / \|s\|), `ωn = abs(s)`, with ζ = 1 at s = 0 by convention. P8 adds ζ and ωₙ as a verb or fields. | Now: two named lines `zeta = ...`, `wn = ...` with the comment; when P8 lands the hover calls P8's function, one owner. | behaviour-preserving |
| FRQ-7 | frequency.py:72-74 | The `w` docstring says the automatic grid spans one decade past the slowest and fastest root; `linear.frequency_range` (linear.py:92-110) also widens it until it brackets \|G\| = 1. | Say so in one clause. | behaviour-preserving |
| FRQ-8 | frequency.py:380, 640, 657 | `plot_root_locus` names the gain array `K` (`K, roots = linear.root_locus(...)`), while `linear.root_locus` and `root_locus` call it `gains` and `closed_loop_poles` uses `K` for one scalar gain. | Local `gains`; `K` stays the scalar. | behaviour-preserving |
| FRQ-9 | frequency.py:513-679 | Nine underscore helpers (`_bode_coordinates`, `_component`, `_margins_text`, `_root_text`, `_root_markers`, `_bode_figure`, `_pzmap_figure`, `_root_locus_figure`, `_nyquist_figure`). | Plain names. | behaviour-preserving |

## 3. `analysis/time_response.py` (207 lines)

**Verdict: far.** `step_info` is the textbook page students compare with Dorf & Bishop §5.3
(rise time, settling time, overshoot, peak), and none of it is in the body: the body holds the
definitions as comments and builds `StepInfo` positionally from four helpers on the `return`
line.

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| TRS-1 | time_response.py:77-91, 126-160 | `step_info` carries four definitions as comments (77-80) while the code is in `_rise_time`, `_settling_time`, `_overshoot`, `_steady_state`, called positionally inside `return StepInfo(...)`. | Unroll into the body, one named line per figure under its comment: `y_final`, `settled` (tail in the 2 % band), `y_ss`, `k_p` / `y_p` / `t_p`, `t_r` (first 90 % crossing − first 10 % crossing, signed toward `y_final`), `t_s` (first sample after the last exit from the band), `M_p` (`100 (max sign(y_f) y − \|y_f\|) / \|y_f\|`, floored at 0). Helpers only for the index searches (`first_crossing`, `last_exit`); `StepInfo(...)` built by keyword. | behaviour-preserving |
| TRS-2 | time_response.py:127-128, 153-154 | The "has it settled" test is computed twice, identically (`tail = y[time >= 0.8 * time[-1]]`, 2 % of `max(abs(final), 1e-12)`), once per helper. | One named `settled` line in the body (falls out of TRS-1). | behaviour-preserving |
| TRS-3 | time_response.py:81, 132-149 | Rise time and overshoot are measured against `y[-1]` even when the response has not settled (`steady_state` is `nan`). For a ramp on [0, 5], `step_info` returns `rise_time=4.0` (always 0.8 · horizon) and `overshoot=0.0`, and `plot_step_response` prints "Rise time = 4 s". | `t_r` and `M_p` are `nan` when `settled` is false, as `t_s` and `y_ss` already are. Test: an integrator's `step_info` is all-`nan` except the peak. | bug |
| TRS-4 | time_response.py:60 | Docstring: "`tf` defaults to five times the slowest stable time constant"; `linear.settling_horizon` uses eight (linear.py:258, 265), and `test_control_plots.py:105` pins 8.0. (P8 lists the same line.) | "eight times". | behaviour-preserving |
| TRS-5 | time_response.py:69 | `return time, linear.step_response(A, B, C, D, time)`: the response is computed on the `return` line. | `y = linear.step_response(...)`, blank line, `return time, y`. | behaviour-preserving |
| TRS-6 | time_response.py:126-163 | Five underscore helpers (`_steady_state`, `_overshoot`, `_rise_time`, `_settling_time`, `_step_figure`). | Plain names (most disappear with TRS-1). | behaviour-preserving |

## 4. `analysis/structural.py` (89 lines)

**Verdict: close.** The Kalman matrices are built step by step under their comments; the rank,
which is the theorem, sits inside the `return` call.

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| STR-1 | structural.py:47, 69 | `return StructuralResult(matrix=ctrb, rank=int(np.linalg.matrix_rank(ctrb)), n=n)`: the rank test is on the `return` line. | `# controllable ⇔ rank 𝒞 = n` over `r = int(np.linalg.matrix_rank(ctrb))`, blank line, `return StructuralResult(matrix=ctrb, rank=r, n=n)`; same for 𝒪. | behaviour-preserving |
| STR-2 | structural.py:35-39, 57-61, 72-78 | Coercion and the `LTISystem` branch open the body (five lines of plumbing before the math); `_lti_matrices` has a leading underscore. | One helper `matrix_pair(A, second, name)` under `# Internal machinery` returning the coerced pair; the unpack beat is one line. | behaviour-preserving |
| STR-3 | structural.py:73-89 | No `# Internal machinery` section comment above the helper. | Add it. | behaviour-preserving |

## 5. `analysis/linearize.py` (208 lines)

**Verdict: far.** `linearize_matrices` is the page a student opens first (A = ∂f/∂x, …), and
only `A` is a readable line: `B` is a comprehension inside `_stack_columns`, `D` a four-deep
nest, the `C = I` case returns early in the middle, and the static-block case is split across
two branches.

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| LNZ-1 | linearize.py:56-100 | The four Jacobians are not four lines: `B` (59-65), `C` (68-73) and `D` (77-96, four nested calls) are comprehensions over selectors; `return A, B, np.eye(n), ...` (67) exits mid-body; `B` and `C` of a static block are filled after `D` (97-99). | Unpack beat resolves the selectors and the two special cases (state output, static block); the math beat is four named lines under the existing comment: `A = jacobian(sys, "f", "x", ...)`, `B = input_columns(sys, "f", inputs, ...)`, `C = output_rows(sys, "x", outputs, ...)`, `D = output_input_block(sys, outputs, inputs, ...)`; one `return A, B, C, D`. (T4 listed the `D` unroll and the early return.) | behaviour-preserving |
| LNZ-2 | linearize.py:128, 158-200 | Section comment `# Selector helpers shared with the frequency tools` and six underscore helpers (`_selector`, `_as_list`, `_rows`, `_columns`, `_stack_columns`, `_rows_of`). | `# Internal machinery`, plain names; the channel selectors of X-2 join them. | behaviour-preserving |

## 6. `analysis/modal.py` (150 lines)

**Verdict: close.** Short and in order; the eigendecomposition is on the `return` line and
`animate_modal` carries its plumbing in the body.

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| MOD-1 | modal.py:49-50 | `return np.linalg.eig(A)`: the eigendecomposition is on the `return` line, and it returns NumPy's `EigResult` (fields `eigenvalues`, `eigenvectors`) while the docstring promises `poles, modes`. Every caller unpacks two values (modal.py:103, 148; test_control_analysis.py:591-659). | `poles, modes = np.linalg.eig(A)` under the `A V = V Λ` comment, blank line, `return poles, modes` (a plain tuple). | returned type [ask] |
| MOD-2 | modal.py:96-101 | `animate_modal` re-implements the default operating point (`plant.x0`, `plant.get_u_from_input_ports()`, coercion) already owned by `derivatives.operating_point` (derivatives.py:105-113). | `x_bar, u_bar, params = operating_point(plant, x_bar, u_bar, params)`. | behaviour-preserving |
| MOD-3 | modal.py:112-119 | The per-mode horizon heuristic (`4π/\|λ\| + 1` clipped to [1, 30] s, 5 s near \|λ\| = 0) is six lines of plumbing inside the math loop. | Helper `mode_horizon(pole, tf)`; the loop reads pick λᵢ, vᵢ → horizon → time grid → Δx → x → animate. | behaviour-preserving |
| MOD-4 | modal.py:123-129 | The comment gives `x(t) = x_bar + Δx(t)` but that step is a keyword argument inside `Trajectory(...)`. | Named line `x = x_bar[:, None] + delta_x`, then the trajectory. | behaviour-preserving |
| MOD-5 | modal.py:54, 65 | `animate_modal(plant, ...)` while every other band tool (and the package docstring) reads `tool(sys, x_bar, u_bar, t, params, ...)`. The only caller passes it positionally (facades.py:820); no keyword caller (grep). | `sys`. (`n_steps` stays: `n` is the state dimension, LIN-7.) | public name [ask] |

## 7. Cross-module findings

| id | location | finding | proposed change | kind |
| --- | --- | --- | --- | --- |
| X-1 | linear.py:33-37, 55-63, 78, 189, 232-233; frequency.py:186, 250; structural.py:31, 53; modal.py:76 | Equations in docstrings (the Rosenbrock pencil, `G(s) = k ∏(s − z)/∏(s − p)`, `C (jωI − A)⁻¹ B + D`, `eig(A − B K (1 + K d)⁻¹ C)`, the Kalman matrices, `x = x_bar + Δx`). Most already sit as a comment in the body. `gain`'s docstring (59-62) also argues against an alternative design ("a Markov scan needs a tolerance…"): process, not contract. | Docstrings keep the contract in words; each equation lives once, as the body comment; the design argument goes to a one-line comment on the `s0` line or to this review. | behaviour-preserving |
| X-2 | linearize.py:131-172; frequency.py:420-468, 519-534; time_response.py:18 | Channel selection lives in two modules with two normalizers of different contracts: `linearize._selector` maps a bare id to `(id, None)` (every component) and rejects a `None` name; `frequency._component` maps it to `(id, 0)` and accepts `None` (the state). `time_response` imports its channel from `frequency`, so the time domain depends on the frequency module. | One home, `linearize.py`'s internal machinery (`input_selectors`, `output_selectors`, `siso_channel`, `siso_matrices`, `channel_label`); `frequency` and `time_response` import from there. The two normalizers stay two functions (their contracts differ), side by side and named for it (`selector`, `siso_selector`). | behaviour-preserving |
| X-3 | linear.py:173-174; frequency.py:513-516, 581 | Bode coordinates computed in two places (the `margins` pair has no `np.errstate`, so a zero of `L` on the grid warns there and not in `bode`). | Both bodies show the two named lines (FRQ-1); the figure helper receives them. | behaviour-preserving |
| X-4 | linear.py:66, 360-364; frequency.py:643-649 | The open-loop radius `max(\|p\|, \|z\|, 1)` is written three times: `gain`'s `s0` (×2), `_open_loop_radius` (×1, used by the gain sweep), `_root_locus_figure`'s `reach` (×3, recomputing poles and zeros inline). | One `open_loop_radius(A, B, C, D)`; call sites name `s0 = 2.0 * r` and `reach = 3.0 * r`. | behaviour-preserving |
| X-5 | structural.py:47, 69; linear.py:131-144 | Rank is decided two ways: `controllability` / `observability` use `np.linalg.matrix_rank`'s default (σ > σ₁ · max(dims) · ε), `minreal` keeps σᵢ > 1e-9 σ₁. Reproduced: `A = diag(−1, −2)`, `B = [1; 1e-11]`, `C = [1 1]` is "controllable" (rank 2) while `minreal`, and so `pzmap` and `transfer_function` by default, drop the mode as uncontrollable. RULES 5.3's own example shows one shared rank tolerance. | One rank rule: `controllability(A, B, *, tol=1e-9)` and `observability` count σᵢ > tol · σ₁ from a named SVD line (as `minreal` does), and `minreal` reads the same default. Verdicts change only for pairs whose Kalman matrix has σ_min/σ_max between ~1e-16 and 1e-9. | bug; `tol` keyword public name [ask] |
| X-6 | linear.py:88, 69-71, 193, 248-252 | The SISO functions of `linear` truncate a MIMO realization silently: `frequency_response` returns G₁₁ (reproduced: `A = −I₂, B = C = I₂` gives `0.5−0.5j` at ω = 1, no error), `gain` reads `D[0, 0]` and entry `[0, 0]`, `closed_loop_poles` uses `D[0, 0]` in `1 + K d`, `step_response` steps every input at once (`u = np.ones(m)`) and reports output 0. The system tier always passes one channel, so only direct callers of `linear` are exposed. | `as_matrices(A, B, C, D, siso=True)` raises `ValueError` when `m ≠ 1` or `p ≠ 1` in those four functions. Test: a 2×2 realization raises. | bug |
| X-7 | frequency.py:1-14; time_response.py:1-9; structural.py:1-8; linear.py:1-5 | Module docstrings are preambles (RULES 5.23: a one-line title). `frequency`'s is the `of` / `wrt` contract, which belongs on `siso_channel` and is already in `frequency_response`'s parameters. Out of scope but adjacent: `analysis/__init__.py:3-4` still says "the same verbs are methods on every `System`", stale since `4412a18`. | One-line titles; the channel contract moves to `siso_channel`'s docstring; the `__init__` sentence names the kept shortcuts or drops the claim. | behaviour-preserving |

A student-facing note, no change proposed: the band's positional `t` is the time at which the
Jacobians are evaluated, so the time grids are `time` (time_response.py:68, modal.py:121) while
`linear.step_response` names its grid `t` (linear.py:229). A student may try
`step_response(sys, t=np.linspace(...))`. Renaming `t` across the band is a large public
change for a small gain; the `step_response` docstring can say that the grid is set by `tf`
and `n`.

## 8. Decisions for the maintainer

1. **LIN-7 — `linear.root_locus(n=...)` → `n_gains`.** Recommend yes: no caller passes it,
   and `n` is the state dimension in the same file.
2. **MOD-1 — `modal_analysis` returns a plain tuple `(poles, modes)`** instead of NumPy's
   `EigResult`. Recommend yes: the docstring already promises it, every caller unpacks, and it
   takes the eigendecomposition off the `return` line.
3. **MOD-5 — `animate_modal(plant, ...)` → `animate_modal(sys, ...)`.** Recommend yes: the
   band's calling pattern (and the signature test TODO asks for) reads `sys`; only positional
   callers exist.
4. **X-5 — a `tol` keyword on `controllability` / `observability`, one rank rule shared with
   `minreal`.** Recommend yes, default `1e-9` relative, so "controllable" and "does not cancel
   in `pzmap`" can no longer disagree; it is also the RULES 5.3 example written out.

The three bugs (LIN-8, TRS-3, X-6) change behaviour only where the current output is wrong or
meaningless; they need a go too, as their own commit with their own tests.

## 9. Proposed order for step 3

Each commit: seeded baseline captured twice and `cmp`-identical before the edit, `ruff check`,
`ruff format --check`, the band tests (`test_control_plots.py`, `test_control_analysis.py`,
`test_derivatives.py`), then the baseline again and `cmp`. The diff read next to `dp.py`.

1. **Plain names and sections** (LIN-1, FRQ-9, TRS-6, STR-2's rename, STR-3, LNZ-2, FRQ-5).
   Baseline A: every band verb on the test transfer functions of `test_control_plots.py`
   and on three catalog plants (`Pendulum`, `InvertedPendulum`, `TwoMass`), both `method="fd"`
   and `"jax"`: `linearize_matrices`, `bode`, `margins`, `nyquist`, `pzmap`, `root_locus`,
   `transfer_function` (num, den), `step_response`, `step_info`, `controllability`,
   `observability`, `modal_analysis`, `linear.frequency_range`, `linear.minreal`.
2. **`linear.py` bodies** (LIN-2, LIN-3, LIN-4, LIN-5, LIN-6, X-4 with FRQ-8) and the
   docstring equations of `linear.py` (X-1). Baseline A.
3. **`frequency.py` bodies** (FRQ-1, FRQ-2, FRQ-3, FRQ-6, FRQ-7, X-3) and its docstrings (X-1,
   X-7). Baseline A plus the `ControlFigure` of each `plot_*` rendered to a dict (traces,
   lines, notes, limits) with `show=False`.
4. **`step_info` on the page** (TRS-1, TRS-2, TRS-4, TRS-5). Baseline A plus `step_info` on
   the first-, second-order, non-minimum-phase and negative-gain cases of
   `test_control_plots.py:108-140`.
5. **`structural.py` and `linearize.py`** (STR-1, STR-2, LNZ-1, X-1, X-7). Baseline A plus
   `linearize_matrices` on every catalog plant, a diagram with `of="block:port"` wires, a
   static block, and `of`/`wrt` lists with component indices.
6. **Channel selection in one home** (X-2, FRQ-4). A move: Baseline A plus commit 3's figure
   dicts.
7. **`modal.py`** (MOD-2, MOD-3, MOD-4; MOD-1 and MOD-5 if decided). Baseline A plus
   `animate_modal` with `plant.animate` patched to record each `Trajectory` (t, x, u) for
   `mode="all"` on `Pendulum` and `TwoMass`.
8. **Decided renames** (LIN-7; X-5's keyword), if not already folded into their module's commit.
   Baseline A, unchanged.
9. **Bugs, behaviour-changing** (LIN-8, TRS-3, X-6, X-5's rule): one test each first, then the
   fix; Baseline A re-captured, the diff limited to the intended entries (X-5's rank on the
   ill-conditioned pair, TRS-3's `nan`s), reviewed line by line rather than `cmp`.
