# The GRO501 APP notebooks are the students' work: P11 dropped (2026-10-15)

**Date:** 2026-10-15
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** v0.2 wave B ended on P11, two GRO501 notebooks in
`examples/teaching/topics/classical_control/`: `dc_motor_propulsion.ipynb` (APP2: DC motor and
longitudinal dynamics, linearization, root locus, PID to the guide's Table 2 specs, discrete
implementation) and `bicycle_autopilot.ipynb` (APP4: bicycle model, controllability across
speeds, LQR, nested loops, pole placement, Kalman observer). ROADMAP §4.2 gate 2 asked for one
such notebook per APP. Reviewing the next steps, the maintainer dropped the step.

## Ruling

- **The repository ships no APP notebook.** Those two notebooks are what the GRO501 students
  build themselves; a worked version in the repository would hand them the answer. minilink's
  part is the tools: every topic row of ROADMAP §4.2 green, with a demo or a topic notebook,
  on the Basic tier.

## Where it lives

- ROADMAP §4.2, cross-cutting gate 2 reworded (no APP notebook; the Basic-tier,
  Colab-first rule kept for the topic demos and notebooks); the P11 line leaves §5.2.
- TODO §2: the P11 row removed; S61, S62 and TB-b now land before v0.2 ships instead of
  before P11; RN-3 waits for its first consumer alone.
- `gro501-classical-control.md` (plan doc retired 2026-10-17): step map and §P11
  marked dropped, sequencing updated; the plans index row, `randomness.md`,
  `automation-by-convention.md` and `estimation-and-state-feedback.md` no longer cite P11.
- Earlier records that mention P11 (2026-09-22, 2026-10-11, 2026-10-13) stay as written.
