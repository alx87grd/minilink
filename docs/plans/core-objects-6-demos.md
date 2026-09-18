# Core objects, phase 6: demos and notebooks to the minimal rule

Status: student-facing — rule agreed 2026-09-15 (C14, RULES 6.1 / 6.10 / 6.11), sweep not started;
every file of the sweep is a maintainer review. Runs alongside phases 2–4 once phase 1 has landed.
The census behind the rule: docs/reviews/2026-09-15-foundations-review.md, comment C14.

- [ ] **6.1 Native reporting the demos hand-roll** (agent lane: plotting interfaces; land only
  what two or more demos need). ~~`print(sys)` (naming quick win 5)~~ landed 2026-09-17;
  remaining: `Optimizer` convergence plot
  (`optim_plot.py` draws it by hand); two planners' trees or solutions side by side
  (`rrt_car_parking.py`, `rrt_holonomic_obstacles.py`); learning curves of several learners on
  one axis (`double_integrator_sarsa_vs_q_learning_rl.py`); a `rollout_batch` family plot
  (`rollout_param_family.py`); a 3-D phase plot (`lorenz_attractor.py`). Each is one `plot_*`
  or `__str__` with a test; DESIGN §7 lists them.
- [ ] **6.2 The flatness ratchet** (6.12). `test_teaching_imports.py` already AST-walks the
  demos; add: no top-level `def` or utility class in `examples/demos/`, and no notebook cell
  that mixes an API call with matplotlib code — today's offenders allowlisted, the list
  shrinking with 6.3, as S12 did for imports.
- [ ] **6.3 The sweep**, one file per step, maintainer reviews each diff (AGENTS: student-facing;
  the term gate is about names, so notebook content may change with an explicit ask). Order:
  the tutorial chapters (`11_reinforcement_learning.ipynb` and `showcase_jax.ipynb` first),
  then the 20 non-clean demos, then the teaching notebooks; the three `compile/` demos wait for
  the differentiable closed-loop cost (their helpers *are* that feature);
  `manipulator_eom.ipynb` is reviewed for which of its 26 helpers the text teaches. Done
  when the allowlist of 6.2 is empty and the notebook checks pass.
