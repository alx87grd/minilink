# One install story: conda, Colab or pip (2026-10-17)

**Date:** 2026-10-17
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Four places said how to install minilink, and they had drifted into a matrix of
two tiers by four routes: `environment.yml` and `environment-basic.yml`, a 175-line
`install.md`, ten pip extras in `pyproject.toml`, and the README's Install block. CI read none
of the conda files and installed by pip extras in four different combinations. Polishing
before the v0.2 tag, the maintainer ruled.

## Rulings

- **One text, in the README.** `install.md` and `environment-basic.yml` are deleted. The
  README's Install section says, in this order: conda (recommended, everything), Colab (the
  two-line clone-and-path cell), pip (the base, the `full` extra, the two system tools). The
  developer setup (dev tools, the two git hooks, the notebook filter) opens `tests/README.md`,
  the developer entry point.
- **Two pip extras.** `full` is every optional library that works once pip installs it: JAX,
  plotly, meshcat, pygame, sympy, gymnasium. `dev` is the test, lint, hook, build and docs
  tools. The per-feature extras (`jax`, `plotting`, `visualization`, `diagrams`, `symbolic`,
  `rl`, `ipopt`, `docs`) are gone.
- **System tools are conda's job.** Graphviz (`plot_diagram`) and Ipopt both need a system
  install pip cannot do, so neither has an extra. The two refusals say how to get them:
  conda-forge, or the system package then the Python wrapper by pip. The symmetry is the
  point: a student learns one rule, not two exceptions.
- **The base stays NumPy, SciPy, Matplotlib.** JAX is optional and in `full`: the Basic-tier
  contract (ROADMAP §4.1 gate 4, the `laptops` CI job, the teaching-surface test) and GRO501's
  no-JAX path stand. Requiring JAX would delete no code and add a 100 MB install for the
  students who need it least.
- **One conda file, everything.** `environment.yml` keeps PyTorch and Stable-Baselines3 until
  the SB3 notebooks retire (S49); only its tier comments go.
- **Refusal hints name the package.** Every library hint that said `minilink[<extra>]` now
  says `pip install <package>` (RULES 5.24: a refusal says what to pass).

## What install.md lost, by design

The tier table, the venv recipe, and the troubleshooting list; each troubleshooting entry is
now the refusal's own hint, and the one non-obvious recovery (a `git pull` refused on
notebooks when the filter is missing) sits in the tests/README setup section.

## Where it lives

- `README.md` Install; `tests/README.md` "Setup, once per clone"; `pyproject.toml`
  extras and markers; the four CI workflows' install lines; `environment.yml`.
- ROADMAP §5.4 V6 and TODO §5 V6 keep only the pinned Colab cell.
- Earlier records that mention the tiers or `install.md` stay as written.
