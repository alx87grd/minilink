# Rulings on the open questions (2026-10-25)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the questions of [QUESTIONS.md](../dev/QUESTIONS.md) that need no reading of code,
settled one by one in a session. Each ruling names the question, the answer, and what changed.
Numbering continues the rulings record of 2026-10-22 (R1–R90).

- **R91 (2026-10-25) Gradient descent by hand: the closing PID example, for now.** The one place
  that writes the descent loop out, then the `Optimizer` doing the same (RULES 6.15), is the
  closing PID example (`examples/demos/compile/pid_autotuning_jax.py`), not tutorial 07 as R75
  said. A tutorial follows later, once the generic optimizer and the mathematical-program system
  are extended. Changed: RULES 6.15; a Later line in TODO.
- **R92 (2026-10-25) The problem's key reaches the noise inside the loop, now.** Each trial of
  `evaluate` realizes every random block of the evaluated loop (`System.realize`) under a seed the
  library's cipher derives from the evaluation's key and the trial, so the same integer key gives
  the same noise on both backends. Sized first: about half a day, no current number changes (no
  test, demo or notebook evaluates a noisy loop over several trials). The other randomness jobs
  stay in v0.4: the same key giving the same starts and parameters on both backends (the rest of
  RN-4, every stochastic number changing once), the disturbance ports as signals (the rest of
  RN-5), the distributions reading `params` (RN-2), `NoiseSource` (RN-3).
- **R93 (2026-10-25) A diagram's `params=None` is the snapshot taken at compile, on both
  backends.** A compiled diagram copies each block's `params` at compile, as a compiled single
  system does; an edit after compile needs a recompile or an explicit `params`. Today's
  `bind_params=True` becomes the behaviour, the flag goes. Settles the NumPy / JAX split found on
  2026-10-24 (NumPy read live parameters, JAX the first call's).
- **R94 (2026-10-25) A diverged run scores the cost bound plus the price.** In `score_run` a run
  that is not finite is still integrated on zeros (no NaN), but its cost term in `J` is the
  problem's cost bound, the largest cost a finite run can reach, where it was `0`: with the price
  on, `J = cost_bound + price`, worse than any finite run, failed or not, and finite. Amends R31,
  R32; only diverged runs with the price on change.
- **R95 (2026-10-25) The tabular learner steps with its grid's solver by default.**
  `TabularLearningPlanner`'s `solver` defaults to the grid's (Euler, R8), so with the defaults the
  learner and value iteration solve the same Markov decision process. No course notebook or demo
  uses the learner today (checked); only its tests' expected numbers move.
- **R96 (2026-10-25) The Gym bridge stops at the horizon.** `Sys2Gym` truncates when `t_new` reaches
  `tf` (a float tolerance), not past it, so an episode of `tf = 1`, `dt = 0.1` is 10 steps, as in
  minilink's RL environment; the class the GRO860 `gymnasium_interface` notebook teaches changes
  the same way. The SB3 notebooks train one step shorter per episode.
