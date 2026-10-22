# Rulings session 1: the shape, the convergence, the step model (2026-10-22)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** The first of the two rulings sessions ROADMAP §5.3 schedules before wave K and
wave S code, run item by item over [end-state-api.md](../plans/end-state-api.md) D1–D10. This
record is appended as the session proceeds; a ruling leaves ROADMAP §6 when it lands here and
in the plan doc it shapes.

## Rulings

- **R1 (D1) The shape: two foundations, six tooling pillars.** Modelling is the primary
  foundation (the `System` and its algebra, the catalog plants, compile and the rollout);
  optimization the secondary one (the `MathematicalProgram` and the `Optimizer`, the solver
  boundary every pillar lowers to); on them six tooling pillars: classical and modern control,
  nonlinear control, trajectory optimization and MPC, reinforcement learning, identification
  and imitation, estimation. RRT a side pillar kept small; deployment experimental until a
  vehicle runs it. The constitution sentence is the maintainer's to place. Lives in:
  end-state-api.md D1; core-consolidation.md §2 reads with this amendment (plants are
  foundation, not optional content).
- **R2 (D2) The catalog clean-up: two import layers, support by test.** No catalog split by
  architecture: a plant is a leaf. The root prelude carries the course set (at most 15
  plants); `minilink.catalog` is the one flat index of every supported plant, each passing the
  both-backends contract test with a docs entry; unsupported plants the maintainer names
  from K1's table move to `minilink/experimental/catalog/`, untested, off the docs site, out
  of the wheel, free to be abandoned; twins merge; the dead helpers go. The defining module
  is not a documented layer. Lives in: end-state-api.md D2; TODO §4 K8 (steps K8-1 to K8-6);
  core-consolidation.md §6.1.
