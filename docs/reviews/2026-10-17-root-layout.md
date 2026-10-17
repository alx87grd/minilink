# The root is for users: the governance docs move to docs/dev (2026-10-17)

**Date:** 2026-10-17
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** The repository root carried eight markdown files, four of them contracts a
maintainer or an agent reads (CONSTITUTION, RULES, DESIGN, ROADMAP) and none a student needs.
ROADMAP §6 held "stay at the root or move under `docs/dev/`" as a v1.0 question (V10). Polishing
before the v0.2 tag, after the install story, the maintainer ruled.

## Rulings

- **The root is for users.** It keeps `README.md`, `LICENSE`, `pyproject.toml`,
  `environment.yml`, and the two files the agent tools read at the root, `CLAUDE.md` and
  `AGENTS.md`.
- **`docs/dev/` holds the contracts.** `CONSTITUTION.md`, `RULES.md`, `DESIGN.md` and
  `ROADMAP.md` moved there with their history (`git mv`). Each stays its own file: CONSTITUTION
  is the law RULES cites by section, and merging them would be a rewrite, not a move.
  `docs/plans/` (the workboard and the design writeups) and `docs/reviews/` (dated records)
  stay where they were.
- **Two landed plan docs retired**, per the plans index's rule: `naming.md` (wins 1, 3 and 6
  landed 2026-10-17; the display-name sweep is step A6; the larger alignments are a ROADMAP §6
  bullet) and `gro501-classical-control.md` (P2–P10 landed, P6 and P11 dropped, TB-b stays a
  TODO §2 row; the baseline audit record holds the history).
- **Bare section citations stay.** "RULES 5.3", "DESIGN §4", "CONSTITUTION §2" in code,
  tests and docs are prose, not paths, and name the document wherever it lives.

## Where it lives

- `docs/dev/` and the link rewrites in `CLAUDE.md`, `AGENTS.md`, `README.md`,
  `pyproject.toml`, `tests/README.md`, `docs/plans/` and three records; `LINKED_DOCS` in
  `tests/unittest/test_repo_contract.py`; `docs/conf.py` excludes `dev` as it excludes `plans`
  and `reviews`.
- ROADMAP §6: the V10 root-layout question leaves; the plans index logs the two deletions.
