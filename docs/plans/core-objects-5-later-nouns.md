# Core objects, phase 5: later nouns, each when its consumer lands

Status: Later / v0.2 — reviewed 2026-09-15. Open questions Q4 (Hamiltonian) and Q5 (parameter dictionaries)
and finding F9 (the disturbance convention): docs/reviews/2026-09-15-foundations-review.md.

- [ ] **5.1 `Gaussian(mean, std=None, cov=None)` + `log_prob`** with the noise-convention decision
  (F9), scheduled with `estimation/` P4. Its `support` is `QuadraticField(mean, cov⁻¹)
  .as_constraint(upper=k²)` (the k-sigma ellipsoid) once Phase 4 is in. `GaussianHead.log_prob`
  may then read `Gaussian(mu, sigma).log_prob(a)` (optional; bit-identity test if done).
- [ ] **5.2 Parameter-dictionary support on sets and distributions** (Q5): one flatten rule,
  landing with identification or the robust problem class.
- [ ] **5.3 `PlanningProblem.hamiltonian()`** only if Q4 says the course teaches it.
- [ ] **5.4 Later bullets (TODO):** `NoiseSource(distribution, sample_period)` replacing the
  hand-rolled `WhiteNoise` draw; `UnionSet` via `|` when reachability or multi-goal work needs
  it; vector bounds on `Saturation`; library sets and fields reading `params` (same rule as
  cost-params, after it; C4); the RL critic as a `StateField` once training is over.
