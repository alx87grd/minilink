# The v0.3 overnight run: decisions to review (2026-10-24)

**Audience:** Prof. Alexandre Girard (Maintainer)
**Context:** the long run of TODO §4 on `dev-v0.3` (the names and structure, the step model,
the path to the cost, the program export, the optimizer, the closing PID example), asked for on
2026-10-23 with the instruction: where a review would have been asked, decide as the maintainer
would, write the decision here, and continue. Rulings R1–R90 are in
[the rulings record](2026-10-22-rulings-session-1.md).

**How to read it.** §1 is the morning summary. §2 lists every decision taken without the
maintainer, each with the step, the choice, the reason, the alternative and the commit to revert.
§3 is the ledger of intended number changes (R80: the same problem and cost before and after).
§4 is what was not done or is blocked. §5 lists the tests skipped in this container.

**Decision rules used** (in order): the rulings, read literally; the Constitution's precedence
(purity, closure, continuous first, textbook readability); the course notes' vocabulary; the
shortest clear name; an existing pattern over a new one; one path, no alias; when torn, the
reversible and smaller option. Never without the maintainer: removing what no ruling removes,
tuned constants in examples, CONSTITUTION, the demo pass (K4).

## 1. Morning summary

(written at the end of the run)

## 2. Decisions

| Id | Step | Question | Choice and reason | Alternative | Commit |
| --- | --- | --- | --- | --- | --- |
| OD-1 | K2 | R54 deletes `ProblemEnv`, but it is the class behind `Sys2Gym.from_problem`, which the `gymnasium_interface` notebook teaches | Kept as internal machinery of the gym module, off the `interfaces` facade; `to_gymnasium` deleted. The ruling's aim (no public name without a caller) holds | Fold its reset/step into `Sys2Gym` (more branches in the plain env) | — |
| OD-2 | K2, K8-1 | R64 takes names off the root but the teaching-surface test required every band name on the root | The test now reads the root prelude as a subset of the band facades: a band name need not be on the root, and where it is, it is the same object | Keep the test and leave the names on the root (contradicts R64) | — |
| OD-3 | K2 | Folding `NeuralNetwork` into `MLP` changes the neural-controller demo's initial weights (Gaussian × 0.2 → orthogonal) and so its numbers | `MLP(2, 1, hidden=(8,), seed=2)`, every tuned constant of the demo kept; it still trains (batch loss 0.32 → 0.0011). No `scale=` option added to `MLP` for one demo | Give `MLP` a Gaussian `scale=` initialisation to keep the old numbers | — |
| OD-4 | Order | Within the clean-up, which items must precede the new core? | K10 (the package moves) and A6 (the renames) go first, since the path to the cost edits those files; the catalog items (K8's cars, root trim, experimental plants, twins), S32 and K3b touch nothing the new core writes, so they run after it. R68 lands with V1.2, which reads the plants' boxes | Every clean-up item first, as planned (the evaluation pipeline would then land later in the night) | — |
| OD-5 | K10 | Where MPC lives after R66 and how students import it | `minilink/planning/trajectory_optimization/mpc/` (the package moved whole, its `__init__` the MPC sub-facade); `minilink.control` re-exports `ModelPredictiveController` (R66) and drops its `mpc` submodule alias (no alias, R82); the demos import from the new sub-facade, which the teaching-import check accepts | Keep a `minilink.control.mpc` alias module | — |

## 3. Intended number changes (R80)

| Step | What changed | Problem and cost | Before | After | Relative difference |
| --- | --- | --- | --- | --- | --- |

## 4. Not done, or blocked

## 5. Tests skipped in this container

No Ipopt (`cyipopt`) in the cloud container: the `ipopt` tests skip and trajectory optimization
falls back to SciPy where a test or demo asks for Ipopt.
