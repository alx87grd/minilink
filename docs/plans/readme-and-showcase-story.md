# The README and the showcase notebook: one story

The README and `examples/tutorial/showcase_minilink.ipynb` tell the same story, beat for beat,
on the same pendulum and with the same names: the README one short snippet per beat, the
notebook the runnable long version. This doc holds the story as ruled and what is left to draft;
the README's public face (TODO V10) and its two broken examples (KNOWN_ISSUES, "The README's
examples") are decided here.

## The spine (ruled 2026-11-20)

1. **Ten lines.** The catalog pendulum, a controller, `@`, simulate.
2. **Diagrams in code.** `>>`, `+`, `%`; your own model (it is its `f`), after the catalog
   plant; the loop is a system, so the classical analysis (`transfer_function`, `plot_bode`,
   root locus) reads it, and `lqr` designs on the linear model (`kalman`, `lqg` its duals). Ends on the parameters: `loop.params` is the tree the wiring carries, and a path
   (`"plant.l"`, `"controller.Kp"`) names any number in any block.
3. **One `f`.** Derivatives and batches in `x`, `u` and any parameter (the parameter Jacobian
   taken away from an equilibrium), exact under JAX; NumPy by default, JAX by one flag.
4. **One problem, every planner.** Value iteration, RRT, trajectory optimization, RL, and MPC (a
   planner on a clock is a controller), each answer a controller closed on the plant. MPC stays in
   the README too, its name changed with GMC714's pins (G3) if they move it.
5. **One judge.** `compare` and `evaluate` right after the planners: the same model, cost, sets
   and draws for every approach, the draws over the start and one uncertain parameter
   (`distributions=`), a noise block on a port optional.
6. **One optimization boundary.** Tune through the whole loop by parameter paths, gains and one
   physical parameter.
7. **The same lines on a robot.** The UR5 in 3-D, one catalog line, `game()`.

**The top of the README** (ruled 2026-11-20): title and tagline, badges, the GIF row, the
pitch, then Install. The GIFs stay first, each captioned by its system and method and linked to
its beat (cart-pole swing-up, trajectory optimization → beat 4; UR5 arm, joint impedance
control → beat 2; racecar, model predictive control → beat 4; rocket landing, reinforcement
learning → beat 4); beat 7 links back to the 3-D pages instead of repeating them. The pipeline
map leaves the first screen for "How it fits together".

**The tone** (ruled 2026-11-20): plain and factual, as a textbook states it. A heading or a
caption says what the thing is or does; no promotional adjectives ("hardware-accelerated",
"zero-friction", "lego-block", "seamless", "exact machine-precision").

The README adds Install first and "How it fits together" (the three foundations, the pipeline
map, what minilink is not) last. Out of the story, linked from the tutorials: MPPI, region of
attraction, the spatial scene, the realtime simulator, C export, sets and fields as objects.

## Where it stands

The README is on the story (2026-11-20, every snippet run as one session in about 50 s, the
contract test green). Ruled while drafting it: `+` is "side by side", not "parallel"; the user's
own model is plain NumPy and the JAX section uses the catalog pendulum; LQR sits with the
classical tools in section 2, so section 3 is derivatives and batches only; the comparison
draws the mass (±20 %), and a terminal weight in the problem's cost makes every planner aim for
the goal; the tuning bounds its three decision variables; reinforcement learning is a link to
the notebook (its default settings do not learn this cost); explicit `connect` wiring stays in
the tutorials; the README stays compact (it does not grow past the old one).

The showcase notebook is on the same story (2026-11-20, the notebook check green, about a
minute on a cloud machine), the README's code cell for cell with the plots and animations
between. Ruled for it: reinforcement learning is its own section, "Learn a law", after the
score, on its own shaped cost and random starts (trained on the README's problem it learns to
stay down), scored by `compare` against the section-2 LQR; the own model is the README's
`MyPendulum` (PID and a partial sensor `h` stay in `basics/`); the run time is measured, not a
budget the content is cut to.

## Left

- The comparison table, in both, re-run when the pendulum's point-mass default lands (TODO §1.10).
