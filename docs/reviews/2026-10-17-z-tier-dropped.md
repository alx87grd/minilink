# The z tier is not planned: P6 dropped (2026-10-17)

**Date:** 2026-10-17
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** P6, the discrete (z) tier, was held since 2026-09-07 as the one open decision
blocking GRO501 (ROADMAP §6): teach the Arduino law with `discretize` + simulation, or build a
z-domain analysis family (exact ZOH and Tustin `c2d` on `LTISystem`, z-plane `pzmap` with the
unit circle, discrete step response, Bode and root locus). Its answer depended on whether the
course's sommatif examines z-plane analysis. Reviewing the roadmap after the textbook pass, the
maintainer ruled.

## Ruling

- **No z-domain analysis family.** GRO501 designs in continuous time, discretizes the law with
  `discretize` (Euler / RK4 step models), validates the sampled loop by simulation and
  hand-derives the difference equation for the Arduino. That is the "Digital implementation"
  row of ROADMAP §4.2, now green as it stands. The z tier is not planned for any release.
- **What stays, as `discretize` follow-ups** (v0.9, no step id): an exact zero-order-hold
  option on `discretize`, shared with `step_response`; the `params["dt"]` fallback kept or
  `dt` made required; a source whose `y` feeds through from a port other than `u`
  (`PendulumWithNoisePort`) does not discretize. These are properties of the step models, not
  a z-domain family.

## Where it lives

- ROADMAP §4.2: the "Digital implementation" row is green with its gate restated; §3's
  Analysis row and §5.2 drop the held P6; §5.4 lists the `discretize` follow-ups in its
  place; the z tier leaves §6.
- TODO §2 notes the drop; the P6 row of §5 becomes the `discretize` follow-ups row; the
  estimation follow-ups no longer wait on P6.
- [gro501-classical-control.md](../plans/gro501-classical-control.md): step map and §P6
  marked dropped, with Option A recorded as the ruling.
- Earlier records that mention P6 (2026-09-07, 2026-09-22) stay as written.
