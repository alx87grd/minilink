# Second opinion: constitution stack

**Date:** 2026-09-12
**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** Independent reading of `CONSTITUTION.md`, `RULES.md`,
`docs/plans/constitution-integration.md`, and the 12 September editorial
review. Not derived from that review’s conclusions.
**Applied:** the amendments below landed the same day; this file is the
decision record.

---

The four-document split is the right architecture. The 12 September editorial
review is too confirmatory: it praises documents that already contain its
“approved refinements,” rebrands the product, and does not pressure-test
remaining overlap or execution risk. This note is the dissenting review.

## Verdict

- **Keep** `CONSTITUTION.md` as the supreme compass, `RULES.md` as the
  7-question review ladder, `AGENTS.md` as agent behavior + repo workflow,
  `DESIGN.md` as technical contracts, `ROADMAP.md` as the plan of record.
- **Do not** make `AGENTS.md` “100% behavioral.” Keep a thin always-loaded
  stub plus the doc map.
- **Do not** target ~400 lines for `DESIGN.md`. Cut philosophy, keep
  contracts. Length follows content.
- **Amend** the constitution before treating it as ratified: invariant 3
  overclaimed, precedence dropped “continuous is primary,” and identity
  naming was split.

Cursor always injects `AGENTS.md`. `RULES.md` and `CONSTITUTION.md` are read
only if AGENTS tells the agent to open them. That is the operational hinge of
the whole split.

## Where the editorial review is right

- Purity above compositional closure is the correct ranking. A stateful `f`
  kills tracing, batching, and solver stepping.
- Bare signatures, `xp`, no `self.` in equation lines, and no `_`
  pseudo-privacy on System/facade classes are the textbook contract.
- Unconnected ports stay silent by design; CI adapts demos from the outside.
- Competitor names and course codes do not belong in the constitution. The
  internal landscape table in the editorial review is fine as a dated review,
  not as product copy.
- Keeping `DESIGN.md` (not merging or renaming it) is correct.

## Where I dissent

**1. The editorial review is a rubber stamp, not a second look.** Its
“approved refinements” (diagram algebra, `tf` declarative, SI/radians, Colab
60s, feedback topologies) were already in the files it reviews. Line counts it
cited were already stale (constitution was 99 lines, not 114). A real review
would have named overclaims and leftover duplication.

**2. Do not rebrand to “Differentiable Systems Lab.”** The constitution’s
**Unified Systems Lab** is the better name. Differentiability is a means;
unification is the product. “Differentiable” sounds JAX-first and undercuts
invariant 5 (NumPy is enough to teach). Keep one name.

**3. Constitution invariant 3 overclaimed.** It said every `@` / `>>` / `+`
composite “compiles down to an identical pure continuous dynamical system.”
That is true of flow diagrams. It is false of `Computer @ plant` /
`HybridDiagram`, which `ROADMAP.md` §7 still calls a sibling outside the
continuous hierarchy (while `StepSystem` already subclasses `System`).
Restrict closure to the **continuous** set. Hybrid is a sibling algebra, not a
hole in the continuous one.

**4. Precedence dropped a principle that fights you every week.** §2.4 said
the continuous contract is primary. §5’s override list did not rank it.
Feature pressure will keep asking to complicate `DiagramSystem` / `Simulator`
for discrete convenience. Restore **continuous-primary** in the override list,
below closure of the continuous set, above readability.

**5. RULES §1–§2 still copied the constitution.** Scope, “tools are verbs,”
“systems are descriptions,” “name by diagram role” appeared in both. RULES
should be the checkable test (“does this PR violate X?”), not a second essay.
Keep one or two sentences plus the operational bullets that the constitution
cannot host (Leaf naming, dual-params student tolerance, `self.traj`
exception).

**6. Several RULES bullets were new product claims, not moved AGENTS text.**

- **§4.10 early validation** — wiring already checks dim mismatch. Shape-wrong
  `f`/`h` still fails silently (see the 2026-09-05 second opinion A.2). Do not
  write the rule as if ODE-time failures are already gone.
- **§6.7 “under 60 seconds”** — Colab-first is real. A hard 60s cap on every
  teaching notebook is unverified and will fail on trajopt/RL cells. Soften to
  “Colab CPU, no C++/license/GPU required; keep the main teaching path short.”
- **§4.9 canonical ports `r`,`w`,`v`** — aspirational nomenclature. Fine as
  “prefer when it fits,” not as a rename sweep.
- **§5.14 / §5.15** — SI/radians and branchless JAX vs student `if` are good;
  keep.

**7. Do not collapse ROADMAP §2 to a pointer.** Constitution §3 is the promise
(“student syntax never breaks”). ROADMAP §2 is the operating contract: soft
entry rule, wheel scope, CI-checked teaching imports, provisional bands.
RULES §3.3 is a shortened copy that already dropped those clauses. Point
upward for philosophy; keep the table.

**8. DESIGN.md 1150 → 400 is the wrong success metric.** §4 Core Object
Contracts alone is ~370 lines because Hybrid, realtime, compile, and
vehicle-ladder behavior are not obvious from code. Deleting §1 (principles +
identity table) is right — identity now lives in the constitution; the
competitor table must not migrate into README/pitch. After that, DESIGN will
still be large. Success = “no philosophy, no style, contracts only,” not a
line budget.

## Answers to the integration plan’s two questions

**AGENTS.md — not 100% behavioral.** Yes: move Core directives, Textbook
style, and Architecture reminders into RULES. No: do not empty AGENTS.

Keep in AGENTS (agent-operational knowledge, not a review-ladder question):

- Opening: read CONSTITUTION for architectural tradeoffs; read RULES before
  writing or reviewing Python.
- Doc map, intro-doc scope, examples buckets, student-facing imports,
  public-facing competitor-name grep, demo-header rule.
- Preserve user edits, ask-first vs agent-managed, local CI gate, no polling,
  revision pass.
- An 8–12 line non-negotiables stub (math-first, preserve user edits, teaching
  imports, no test harness in demos, ask before removing names). Cursor will
  not reliably open a 244-line RULES.md on every turn unless the always-applied
  file says to.

**DESIGN.md — keep the file, drop the 400-line target.** Slim §1. Leave
§2–§8. Contributing style lives in RULES, not AGENTS.

## Constitution amendments applied

- Keep the title **Unified Systems Lab**. Ignore the editorial rebrand.
- Invariant 3: closure applies to continuous Systems; a flow diagram compiles
  to $\dot{X}=F(X,U,t;P)$. Hybrid/step composition is a sibling algebra.
- §5 add **continuous contract is primary** as rank 3 (purity, closure,
  continuous-primary, readability, single-path). Two-audience stays a hard
  constraint in §3, not a ranked tradeoff.
- §1.4 last bullet moved into §1.1 (it was a positive identity sentence
  inside a NOT-list).
- “Sacred Contract” renamed to “The Contract.”
- §6 kept (who writes $f$, backend honesty, infrastructure exemption).

Competitor names and course codes stay out.

## RULES / AGENTS leftovers the plan under-counted

Moved into RULES (code/review law that lived only in AGENTS):

- Two-audience **file** principle (student reader vs library-developer file)
- `__main__` hello-worlds ~10 lines
- Native-array equation paths; conversions at boundaries only
- JAX float64 default (`MINILINK_JAX_X64=0`)
- Prototype honestly (`TODO: User Architectural Review`)
- Incremental refactoring / no broad restructure unless asked

Left in AGENTS (or DESIGN where noted):

- Doc map, intro-doc scope, examples buckets, public-facing grep
- `outputs()` / `outputs_p()` boundary-only → DESIGN
- Inheritance for core types, composition for diagrams → DESIGN
