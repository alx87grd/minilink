# Rulings of 2026-10-31: the merge-review run's open points, taken in session

**Audience:** Prof. Alexandre Girard (Maintainer); the agent that implements them.
**Context:** the six points [the merge-review run's log](2026-10-31-merge-review-run-log.md)
raised first (its §1, "To raise first", and §7), taken with the maintainer one at a time
before the v0.3 release, each with its context, options and the agent's recommendation.

| Ruling | Point (the log's id) | In one line |
| --- | --- | --- |
| R188 | How RK4 reads a held block (M-1) | **The special handling goes.** A held block (a noise source, a stochastic policy) is read like any block, at each RK4 stage's time: the last stage of a step ending on a switch reads the next draw, first order at the switches. The maintainer: keep the simulation pipeline as simple as possible; the held-noise model is itself an approximation (no true zero-order-hold noise exists in practice), so the precision is not worth a second reading path. Amends R180's last sentence ("check and fix the RK4 last stage") |
| R189 | Which linear blocks declare a time constant (M-4) | **Kept as landed.** Beyond R180's list, the PID family (integrators at 0, each derivative filter at `−1/τ`) and the `Integrator` (`∞`) read their poles, so a PID loop on a declared plant takes the plant's step, not the 1 ms default |
| R190 | `SlidingMode` declares its surface (M-7) | **Kept as landed.** `τ = 1 / max λᵢ`; the sliding-mode demo steps Euler at 5 ms |
| R191 | The kinematic bicycle declares nothing (M-8) | **As landed for v0.3; later, the declaration.** `KinematicBicycle` and `UdeSRacecar` keep the 1 ms default and their numbers; a v0.4 row gives the car family default input bounds and `τ = L / (v_max tan δ_max)`, with the car-family clean-up (K8-0) |
| R192 | The neural-controller demo's method (M-13) | **Kept as landed.** `Optimizer(method="gd", iterations=10000, learning_rate=0.8)`: the hand loop's numbers to the printed digit; L-BFGS overfits the six training pairs |
| R193 | GRO501's `cartpole_dynamic_controller.ipynb` at 0.1 ms (§7) | **The run names its step:** `cl_sys.compute_trajectory(tf=10.0, dt=0.01)`; the notebook back to about 30 s |

**Implemented** on `dev-v0.3-release-rulings`:

- R188: `035e8eef`. `held_step_body`, the evaluator's `holds`, `held_at`, `f_held` and the
  `t_held` branches removed (145 lines out, 22 back); the twin test now pins the first-order
  convergence (a third of the step, an error about three times smaller, measured 3.1 on NumPy
  and JAX); `noise.white.x` of the v0.3 baseline moves back by 3.9e-3, its one entry re-pinned
  in `benchmarks/baselines/v03_core.json`, every other entry unchanged; DESIGN's noise paragraph
  states the approximation; the KNOWN_ISSUES entry leaves.
- R189, R190, R192: no change.
- R191: the row in TODO §2, beside K8-0.
- R193: `21f3f641`; the notebook check passes (31 s, 64 s before).
