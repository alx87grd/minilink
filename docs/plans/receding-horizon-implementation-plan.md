# Receding-horizon implementation plan

**Canonical docs moved** to [`mpc-rh-refactor/`](mpc-rh-refactor/).

Product controller type (canonical): **`ModelPredictiveController`**
(System family) — see [mpc-rh-refactor/vision.md](mpc-rh-refactor/vision.md).
Former name `RecedingHorizonController` is retired there.

| Doc | Role |
| --- | --- |
| [mpc-rh-refactor/README.md](mpc-rh-refactor/README.md) | Index, branch, how to read |
| [mpc-rh-refactor/vision.md](mpc-rh-refactor/vision.md) | Locked end-goal contracts |
| [mpc-rh-refactor/phases.md](mpc-rh-refactor/phases.md) | E0–E8 order, gates, PR slices |
| [mpc-rh-refactor/phase-E0.md](mpc-rh-refactor/phase-E0.md) … [phase-E8.md](mpc-rh-refactor/phase-E8.md) | Per-phase cards |

Related requirements (background):
[mpc-controller-architecture.md](mpc-controller-architecture.md),
[planning-pipeline-architecture.md](planning-pipeline-architecture.md),
[standard-planning-problems.md](standard-planning-problems.md).

Do **not** duplicate the full plan here — edit files under `mpc-rh-refactor/`.
