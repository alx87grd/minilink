# Pyro 2.0 — remaining port backlog

Living checklist (June 2026). Historical gap analysis:
[archive/pyro-port-gap-todo.md](archive/pyro-port-gap-todo.md).

## Done (do not re-port)

- **Catalog plants** — all EoM models in `dynamics/catalog/` (see
  [archive/catalog-migration-notes.md](archive/catalog-migration-notes.md)).
- **Tool tranche 1** — blocks routing/nonlinear/filters, LQR/PID, analysis
  linearize/modal/Bode (see [archive/tool-migration-notes.md](archive/tool-migration-notes.md)).
- **Trajopt** — direct collocation, shooting, multiple shooting (`planning/trajectory_optimization/`).
- **DP** — `DynamicProgrammingPlanner`, `StateSpaceGrid`, three backends (`planning/policy_synthesis/dp.py`).
- **Search** — `RRTPlanner`, `RRTStarPlanner`, extenders, steering (`planning/search/`).
- **Spatial** — `Scene`, `bind()`, clearance/cost fields (`planning/spatial/`).
- **Graphics** — frame-keyed `tf` / geometry / overlays (DESIGN §4).

## Still missing (library modules)

| Area | Target home | Notes |
| --- | --- | --- |
| Robot control | `control/computed_torque.py`, `sliding_mode.py`, `robotic.py` | Blocked on catalog `Manipulator` rebase |
| Trajectory generation | `planning/trajectory_generation/` | Polynomial / min-snap |
| Estimation | `estimation/luenberger.py`, `kalman.py` | Stubs only today |
| Identification | `identification/fitting.py` | Prototype: `examples/scripts/identification/demo_params_gradient.py` |
| Interfaces | `interfaces/gymnasium.py` | Stubs only |
| Neural policies | `control/neural.py` | See [neural-blocks-collection.md](neural-blocks-collection.md) |
| Frequency tools | `analysis/` | Pole-zero, Nyquist, margins, `ss2tf` |

## Demo gap

- Pyro reports ~195 example scripts; minilink has ~44 under `examples/scripts/` plus 7 notebooks.
- No `examples/courses/` or `examples/projects/` (pyro paths never created here).
- Priority: closed-loop showcases per major plant band, not 1:1 filename parity.

## Explicit non-goals

- Discrete-time / ZOH library, RNN blocks, hybrid events (DESIGN §3).
- Pyro `sys2game` framework — use `animate(..., renderer="pygame")` or drop.
- Re-porting completed catalog EoM or graphics kinematic contract.

## Release criteria (unchanged)

1. In-scope pyro modules have a minilink home or documented replacement.
2. Representative closed-loop demo per major plant family.
3. Robot control suite at TRL ≥ 6 after `Manipulator` catalog rebase.
4. README pyro → minilink API mapping table (not started).
