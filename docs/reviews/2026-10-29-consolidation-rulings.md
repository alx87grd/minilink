# Rulings of 2026-10-29: the consolidation after the two runs

**Audience:** Prof. Alexandre Girard (Maintainer); the agent that implements
[the consolidation plan](../plans/consolidation-after-the-runs.md).
**Context:** the design questions the independent review of the v0.3 core run and the night run
raised ([the review](2026-10-29-independent-review-of-the-runs.md)), taken one by one with the
maintainer in session. Each ruling's context, options and design live in the plan; this record is
the index of what was decided.

| Ruling | In one line | Plan |
| --- | --- | --- |
| R168 | No cache for the convenience verbs: they compile each call; a tool holds its own compiled object; the frozen and parametric forms stay inside the evaluator; the edit counter goes with the cache | §1 |
| R169 | Every core object has `params`; every number that is not structure is a leaf, the fields' numbers included; speed comes from the frozen form, never from a number in code | §2 |
| R170 | One tree per planner: the problem's parts plus `"method"` (the method's numbers; its name is structure) plus `"seed"` | §3 |
| R171 | The RL training reward is `−g(x_k, u_k, t_k)·dt`, one call of the cost's `g`, recorded beside the score; the score stays the problem's | §4.1 |
| R172 | One cost bound on the compiled face, the face's own sum with the maxima, read by the judge, the environment and value iteration; the noun is `cost_bound`, `price` goes | §4.2 |
| R173 | A tool's own discount (`alpha`, `gamma`) is internal: consistent inside the tool, named in its record, never the score's; the planning problem defines the problem | §4.3 |
| R174 | The loop's grammar: `%` means "this block runs on the computer at Δ", the block's kind decides its third argument; a sampled loop holds one continuous system, the plant; `discretize` is the model verb; the plant's step is the continuous simulator's rule, `dt=` at runs and `substeps=` at compiles | §5 |
| R175 | z-domain `linearize` on step and sampled systems (R122) lands with the grammar | §5 |
| R176 | One MPC architecture: three pure blocks, the delay a visible `UnitDelay`, `delay=`/`warm_start=`/`lookahead=` gone; the shift and the prediction are face verbs | §6 |
| R177 | The problem's dials: `X0=` and `Xf=` the constraint sets (a vector a singleton), `x_start=` the evaluation start (a point or a law), independent, one consistency check; `x_goal` goes; the cost's target is the cost's own number | §7 |
| R178 | MPPI's names stay on `minilink.planning`, named provisional in ROADMAP §2 until the teaching decision | §8 |
| R179 | The details left to the agent: `hold_x0` a leaf, the compile probe over port functions, the hold slot's label, a native sampled law's period at construction | §8 |

The maintainer's words that frame them: the planning problem defines the problem, a tool may plan
or train on a modified copy and says so; every number a parameter unless structure, and a good
frozen compile so nothing is lost by it; a tool is built once and run many, the high-level verbs
compile every call; one operator with one meaning for the computer side, nothing for the plant
side because it is simply continuous.
