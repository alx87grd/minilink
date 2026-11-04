# Open design questions for the maintainer

The design questions that wait for the maintainer: the architecture, the API and its naming, the
vision, and what affects performance. Details, versions, process, tests and demos are not here:
they sit in their workboard rows, and the rows that wait for the maintainer's go are listed at
the head of [TODO.md](../plans/TODO.md#rows-waiting-for-the-maintainers-go). Known defects are in
[KNOWN_ISSUES.md](KNOWN_ISSUES.md).

A ruling the maintainer takes updates the doc it changes (DESIGN, ROADMAP, TODO or the plan
doc it shapes) in the same change; the commit message carries its why. It then leaves this
file. The v0.3 core's rulings are the architecture of record. `§N` names the workstream of
[ROADMAP §5.3](ROADMAP.md#53-v03--the-complete-core), the same N as TODO §1.N.

## The core

The v0.3 core's architecture is of record. The questions the independent
review of the two runs raised were ruled on 2026-10-29; they landed on
`dev-v0.3` the same day. Its run's decisions taken without you, and those of the three runs before it, are sorted
for one sitting in the merge-review triage. Nothing else is open here.

## Toward v1.0

### How does a public name change once v1.0 is frozen?
- **Your answer (2026-10-27):** not a `DeprecationWarning` shim for one minor release, as
  proposed; the policy stays open. No alias, one sweep, holds meanwhile.
- **Your answer (2026-11-03):** the pending renames land before the freeze, one sweep per
  group, under the vocabulary of RULES 2.7; so the freeze itself needs no shim.
- **Blocks:** the trust contract (V4), for the names changed after v1.0.
- **Today:** no alias or shim ever. The frozen names are drawn at v0.9 from RULES 2.7.
- **Options still open (after v1.0 only):** a shim of some length; a major version per rename;
  a snapshot test of the export list either way.
- **Detail:** [TODO, V4](../plans/TODO.md#v09--the-trust-contract)
- **Answer:**
