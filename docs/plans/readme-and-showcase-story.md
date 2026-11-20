# The README and the showcase notebook: one story

The README and `examples/tutorial/showcase_minilink.ipynb` tell the same story, beat for beat,
on the same pendulum and with the same names: the README one short snippet per beat, the
notebook the runnable long version. This doc holds the story as ruled and what is left to draft;
the README's public face (TODO V10) and its two broken examples (KNOWN_ISSUES, "The README's
examples") are decided here.

## The spine (ruled 2026-11-20)

1. **Ten lines.** The catalog pendulum, a controller, `@`, simulate.
2. **Diagrams in code.** `>>`, `+`, `%`; your own model (it is its `f`), after the catalog
   plant; the loop is a system, so the classical analysis (`plot_bode`, root locus, margins)
   reads it. Ends on the parameters: `loop.params` is the tree the wiring carries, and a path
   (`"plant.l"`, `"controller.Kp"`) names any number in any block.
3. **One `f`.** Derivatives and batches in `x`, `u` and any parameter (the parameter Jacobian
   taken away from an equilibrium); LQR from the linear model, `kalman` its dual and the observer
   one more block; NumPy by default, JAX by one flag.
4. **One problem, every planner.** Value iteration, RRT, trajectory optimization, RL, and MPC (a
   planner on a clock is a controller), each answer a controller closed on the plant. MPC stays in
   the README too, its name changed with GMC714's pins (G3) if they move it.
5. **One judge.** `compare` and `evaluate` right after the planners: the same model, cost, sets
   and draws for every approach, the draws over the start and one uncertain parameter
   (`distributions=`), a noise block on a port optional.
6. **One optimization boundary.** Tune through the whole loop by parameter paths, gains and one
   physical parameter.
7. **The same lines on a robot.** The UR5 in 3-D, one catalog line, `game()`.

The README adds Install first and "How it fits together" (the three foundations, the pipeline
map, what minilink is not) last. Out of the story, linked from the tutorials: MPPI, region of
attraction, the spatial scene, the realtime simulator, C export, sets and fields as objects.

## Left to draft

- The wording, beat by beat, README and notebook together; the pitch (a draft in the chat of
  2026-11-20 ends on the fair comparison and on NumPy first).
- **[ask]** Beat 6's example: bounds on the decision variables (recommended), gains only, or a
  cost that prices the parameters. The README's co-design today converges to a negative gain and
  a 3.5 cm rod; its three-gain PID tuning to negative gains too.
- **[ask]** Beat 5's example: the planners' numbers chosen so each reaches the goal (value
  iteration needs a cost bound: `discount_rate=` or `failure_cost`), with an uncertain parameter.
- The notebook: the judge moved up after the planners, `compare` beside `evaluate`, one
  classical-analysis cell, one observer cell, the parameter tree at the end of the diagrams.
