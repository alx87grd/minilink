# Governance stack audit: the four documents against the code

**Date:** 2026-09-12
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Third reading of the constitution stack, after the editorial review
and the second opinion of the same day. Those two argued about how the
documents should be split. This one checks the split that shipped against
`dev-alex` as it stands, and against `main` as the before picture.
**Applied:** the fixes below landed the same day; this file is the decision
record.

---

## Verdict

The four-document shape is right and is a clear upgrade on `main`, where one
185-line `AGENTS.md` carried identity, style law and workflow at once while
`DESIGN.md` carried product positioning and a competitor table. Nothing found
here argues for re-cutting the split. Every finding is drift between the new
files, or between a file and the code it governs.

The drift was fast: rules written on 12 September were already false about the
repository on 12 September. That is the finding behind the finding. Three of
them are tests now.

## What the split bought

- **A decision procedure.** The conflict precedence ladder is the largest
  single addition. `main` listed principles with no ranking, so every tradeoff
  was re-argued from scratch.
- **Citable law.** Numbered rules let a review say "RULES 5.3" instead of
  restating the principle.
- **Explicit lanes.** Maintainer-owned versus agent-managed replaces `main`'s
  single vague line about architecture refactors.
- **Positioning demoted to history.** The competitor table left the contracts
  document for a dated review, where it belongs.
- **Rules that became tests.** The teaching-import rule and the
  no-harness-in-demos rule were already enforced in the suite. That is the
  model the rest of the ladder should follow.

Cost: governance and contract prose grew from 1218 lines to 1917. `DESIGN.md`
grew despite the pass cutting 61 lines from it, because the call-chains section
moved in from README and the branch documented Lyapunov certificates, the RL
planner, the analysis family and the vehicle ladder.

## Findings and disposition

| # | Finding | Disposition |
| --- | --- | --- |
| 1 | Claude Code loaded none of the stack: it reads `CLAUDE.md`, Cursor injects `AGENTS.md`, and no `CLAUDE.md` existed. The always-injected file is the hinge of the whole split, and it was missing for one of the two agents in daily use. | Applied: `CLAUDE.md` stub pointing at `AGENTS.md`, with four non-negotiables inline for an agent that stops reading there. |
| 2 | `AGENTS.md` made ROADMAP §1–§4 maintainer-owned and the TRL ledger agent-managed. The TRL ledger is §3. | Applied: the maintainer-owned range is now §1, §2 and §4, and the agent-managed line names §3. |
| 3 | Constitution invariant 1 claimed diagrams share the object model; `Computer` and `HybridDiagram` are plain classes. ROADMAP §7 overcorrected and put the whole step path outside the hierarchy, though `StepSystem` and `StepDiagramSystem` subclass `System`. | Applied: invariant 1 names the exact seam and points at the v1.0 review-queue entry; ROADMAP §7 states which classes are Systems and which are not. |
| 4 | RULES 3.7 required the intro surfaces to present the hybrid path and MPC; ROADMAP §7 called hybrid not a v0.1 teaching topic; `test_public_imports` treats both as research lane. | Maintainer decision: keep them on the intro surface. ROADMAP §7 now distinguishes a course topic from the intro surface's hybrid exemplar. |
| 5 | The frozen composition grammar named three operators. The code ships `%` for schedules and exports `feedback()` at the root; only DESIGN mentioned `%`. | Maintainer decision: scope the freeze to the continuous algebra. The constitution and RULES 4.8 now name `connect()` and `feedback()` as the explicit-wiring entry points and `%` as the sibling algebra's one operator. |
| 6 | RULES said "the two showcases" in two places; there are three. The competitor-name gate was scoped to those two, so the newest showcase sat outside the check it defines. | Applied in both bullets, and the check now covers all three. |
| 7 | That gate was written as a basic-regex grep, so its alternation was literal and it reported empty on any input. Re-running it correctly over the public prose is clean, so nothing had leaked. | Applied: the rule now carries the working command and names the test that runs it. |
| 8 | Purity is rank 1 of the precedence ladder; `core/system.py` called it "convention; not enforced". | Applied: the docstring now states the invariant and says the burden sits with the author, since Python cannot check it. |
| 9 | RULES 5.8 bans leading-underscore methods on the System family and the simulators; six sat on `Simulator`, three on `StaticSimulator`, one on `System`. RULES 5.18 capped `__main__` smokes at about ten lines, exceeded in five modules, worst at 54. | Maintainer decision: rename, soften the cap. Ten methods renamed with call sites in the library and benchmarks; 5.18 now states intent instead of a line count. |
| 10 | DESIGN §2 said the teaching surface is tested as a set; the test pinned 11 of 127 root exports. | Applied: the test now walks `minilink.__all__` in full. The claim is literal. |
| 11 | Two entries in the review ladder's own table of contents pointed at slugs that do not exist, because those headings carry a parenthetical suffix. | Applied, and 271 internal links across 22 documents are checked now. |
| 12 | `AGENTS.md` said CI runs exactly the test workflow. `nightly.yml` runs every demo and notebook; `docs.yml` builds the site. | Applied: both are named as non-gates, with the consequence for an agent landing a demo. |
| 13 | Three ghost links: a plan pointing at a vehicle-maps module that was never created, two `file:///Users/...` absolute URLs, and a research README pointing at a notebook that became the RL-to-Bode showcase. | Applied, and the link test now covers `docs/plans/`. |

## Maintainer decisions taken

1. **Hybrid and MPC stay on the intro surface.** Not a v0.1 course topic, but
   the intro's hybrid exemplar, which is what README, the showcases and the
   pitch already do.
2. **The grammar freeze covers the continuous algebra.** `%` is named as the
   sibling algebra's single operator rather than admitted as a fourth
   continuous one.
3. **Code moves to meet RULES 5.8; RULES 5.18 moves to meet the code.**
4. **Contributors enter through README**, which now names the constitution and
   the rules. No `CONTRIBUTING.md`, no fifth governance file.

## Deliberately not done

- **`simulation/realtime/` keeps its eight underscore methods.** It is TRL 2
  and already queued for architectural review; renaming now would churn a
  module that is about to change shape. The naming test excludes it by name,
  so the exemption is visible rather than silent.
- **Build-versus-run stays stated three times** (constitution, RULES 4.11,
  DESIGN §5). The DESIGN copy carries mechanism the others do not, and the
  constitution is maintainer-owned. Accepted duplication.
- **RULES §1 and §2 still echo the constitution** on scope, diagram-role
  naming and arrays-over-wrappers. The second opinion asked for this to be
  cut to one sentence plus operational bullets; it was only partly done.
  Left for the maintainer, since trimming the ladder changes what a reviewer
  is handed.
- **The two-lane contract still appears in four documents.** Only the DESIGN
  copy was trimmed, to the registry fact plus a pointer. The constitution
  holds the promise, ROADMAP §2 the operating table, RULES 3.3 the placement
  shorthand.

## What to watch

The ratchet is the point. Three rules are tests now, so they cannot rot
quietly. The rest of the ladder is still prose, and the evidence of this audit
is that prose rules drift within days when the repository is moving this fast.
When a rule is worth keeping and cheap to check, write the check.
