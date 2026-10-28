# No key, no draw: the design point (2026-10-28)

The maintainer's ruling in the v0.3 run's chat, on the open `nominal()` question of the run log
(§4 of [the run log](2026-10-29-v03-run-log.md)).

**The question.** `PlanningProblem.nominal()` dropped each parameter's law and kept the part's
own number, which is the law's mean only when the plant was built at it.

**The ruling.** One standard behaviour across the library: no seed, no key, no draw; the
nominal, the mean, the design point.

- A noise block with `seed=None` gives its mean signal (already the contract).
- A law's `sample(None)` returns its mean at its numbers (`Gaussian`'s `mean`, `Uniform`'s
  `(lower + upper) / 2`, `Particles`' mean point), tiled to `(n, dim)` with `n`; `law.mean` stays
  the law's own mean and `law.mean_at(params)` is the mean at moved numbers. `key` stays a
  required argument: `None` is passed explicitly.
- `problem.realize(None)` draws nothing: every law at its mean, every seed `None`.
- `problem.nominal()` is that tree written into a copy of the problem's parts: each parameter at
  its law's mean, the start at its mean, the random sources kept with seed `None`; the caller's
  plant and sources are untouched.

**Landed:** the v0.3 run, step 4 (the run log's §7).
