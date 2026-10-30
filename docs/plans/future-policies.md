# Projecting the API onto MPPI, behavior cloning and diffusion policy

**Status:** an independent analysis of 2026-10-21, aligned to the rulings since. What it asked
of the core is decided: controller memory as a sampled block's state and its draws from its seed
and the tick (R104), the compiled problem dispatching on the policy's kind (R105), the
seeded stochastic policy (D29, R129, §5.1), the planner's compiled face that the MPC block reads
(R148, DESIGN §6), a problem stochastic by its parts (R143), and
`NeuralPolicy` as the deterministic face (R151). Fitting to data waits for the behavior-cloning
row (R136, §7).
**Lane:** maintainer-owned.
**Rows:** the seeded policy is [TODO.md](TODO.md) §1.4; the sampled loop TODO §1.5 (DESIGN
§5); MPPI S72 (TODO §4, DESIGN §6); behavior cloning ROADMAP §5.6.

The question: do the core's designs (the step model, the sampled loop, the compiled problem and
its program export, the planner's compiled face, planners returning a `PlanningSolution`) carry
the features that come later, or will those features force a redesign? Three are projected
here: MPPI (planned), behavior cloning (planned after v1.0) and diffusion policy (research lane).

## 1. The verdict

The designs carry MPPI well, diffusion policy reasonably, and behavior cloning only half way.
The core nouns hold for all three: the system, the problem, the trajectory, the program and the
optimizer. Two of the three gaps this analysis found are now closed by design:

- **controller memory** in the sampled loop: a sampled controller with memory is a `StepSystem`
  with state, carried by the loop like any state (R104, R146); its draws come from its seed and
  the tick, no key threaded;
- **a shared stochastic policy** outside the RL package: `GaussianPolicy` (R129, §5 point 4);
- still open: no verb to fit parameters to **recorded data** (not critical to the core tools,
  the maintainer judged on 2026-10-22; decided at the behavior-cloning row, R136).

## 2. MPPI

- **It fits the planner contract.** A planner on the existing planning problem: it rolls out
  many held-input control sequences, scores them with the problem's cost, and returns the usual
  solution with an open-loop plan.
- **Its batch is already expressible.** An open-loop plan is a policy whose table of values
  lives in the parameters, and the start is a leaf of the problem's tree (R144). MPPI's inner
  loop is "the batched cost of many tables from one state", with no new machinery.
- **The receding horizon is the one MPC block** (R148): MPPI implements the planner's compiled
  face (its `z` the nominal sequence and its origin, `shift` the shifted sequence, `seed` a leaf),
  and the MPC's state `z` is the shifted warm start (R155); its fresh samples come from the
  block's seed and `⌊t/Δ⌋` (R104, R158); DESIGN §6, "Path integral control".
- **A payoff.** MPPI's update is a weighted average, so it is differentiable end to end: its
  temperature and noise level (`controller.method.*`) can be optimized by gradient through the
  same program export (`pe.program(wrt=[...])` solved by `Optimizer`) as a PID gain.

## 3. Behavior cloning

**Deployment fits.**
- The cloned policy is a `NeuralPolicy` (R151), which is a `System`: it drops into diagrams, the
  sampled loop, `compare()` and animation.
- The recorder is the realtime game loop. Recorded at the step model's sample rate with the same
  input hold, demonstrations and deployment see the same model: a strong argument for the step
  model.

**Training does not fit.**
- The differentiable path is built around a rollout cost: simulate the closed loop, integrate
  the cost. Behavior cloning's loss is supervised: on recorded states, how far is the policy's
  action from the demonstrator's? No rollout is involved.
- Equation-error identification is also a one-step loss, so "identification is the program
  export with a data-mismatch cost" has the same blind spot.
- So the framework needs either **a second training verb** beside the program export (fit these
  parameters to this dataset with this loss, on the same program bridge and optimizer,
  minibatches drawn from a seed leaf) or the program export with a data-mismatch cost over
  `wrt`; R136 keeps both options for the behavior-cloning row.
- It needs **a dataset noun**, or a batch of `Trajectory`s with padding masks, which the native
  `Trajectory` and the scoring kernel's alive mask already almost provide.
- Real demonstrations observe outputs, not the full state, so **the recorder logs `y`**; the
  identification row already asks for it.

**A bonus.** With the sampled loop recording internal signals, distilling MPC or MPPI into a
neural policy is behavior cloning with a library controller as the teacher: a better first
demo than human driving, and free of hardware (a pointer, R136).

## 4. Diffusion policy

- **Run time is the MPPI shape again.** A diffusion policy observes a short history, samples a
  chunk of future actions by iterative denoising, executes a few and replans: a sampled
  controller with memory (the observation history, the current chunk) as its state, seeded
  draws and a slower replanning rate, all covered by R104 and R146.
- **Training reuses the fitting route** chosen at the behavior-cloning row, with a denoising loss
  instead of an action loss.
- **The real risk is scope.** Diffusion needs conditioned noise-prediction networks and noise
  schedules; minilink should not grow a deep-learning library. Its one MLP is right for
  teaching. The line to hold: a policy network is any pure function with a parameter tree,
  wrapped as a `System`; minilink owns the loop, the data path, the training bridge and the
  evaluation; an external JAX network library owns the architecture. Research lane only,
  JAX-only, after behavior cloning (R136).
- **A convergence.** An open-loop action table is the common output of trajectory
  optimization, MPC, MPPI and diffusion planning: one more reason the open-loop plan is a
  first-class policy (`pv.command`, R148).

## 5. What the designs absorb

1. **Ways to train on one bridge.** `solve` for planners, the compiled problem's program export
   for parameters through a closed-loop rollout (no `tune` facade, R23), and, for a dataset, the
   route R136 leaves to the behavior-cloning row; all produce a `MathematicalProgram` for the
   same `Optimizer`.
   ```python
   solution = MPPIPlanner(problem, n_samples=512).solve()                     # a planner
   pe = problem.compile(controller, dt=0.05)                                  # a rollout cost
   params = pe.unpack(Optimizer(pe.program(wrt=[...])).solve().z, wrt=[...])
   cloned   = fit(policy, demos, wrt="policy", loss="action")                 # a dataset loss (open, R136)
   ```
2. **Controller memory is state** (R104): MPPI's shifted sequence, a diffusion policy's
   observation history and action chunk live in the block's `x`, carried by the sampled loop
   (a `SampledDiagramSystem`, R146). No random key is threaded: a controller's draws come from
   its own seed and the tick, the noise block's counter-based pattern
   (DESIGN §4, "Randomness").
3. **The compiled problem dispatches on the policy's kind** (R105): a continuous law runs
   the continuous closed loop; a sampled controller runs `policy @ plant` as one scan of the
   step face (R141), so MPPI and diffusion policies get the same batched `evaluate` and program
   export. Today the evaluator refuses a sampled controller (`require_continuous`,
   `planning/evaluator.py`) until the sampled face joins it (V1's step 7b).
4. **One standard stochastic policy outside the RL package, built like the noise block**
   (the maintainer's direction, 2026-10-22; R129). The white-noise block keeps its `seed` in
   `params`, draws `ε_k = F(seed, ⌊t/Δ⌋)` with a counter-based generator identical on NumPy and
   JAX, holds each sample over its period, returns the mean when `seed=None`, and gets an integer
   seed leaf per trial from `problem.realize(key, n_trials)` (R144). The stochastic policy built
   the same way, `u = μ_θ(x) + σ_θ(x) · ε(seed, ⌊t/Δ⌋)` with `Δ` its `sample_period`:
   - is a plain `System` with a pure `h`: diagrams, the sampled loop, `compare()`, the
     evaluator and `vmap` over seeds, on both backends;
   - has its deterministic face for free: `seed=None` gives the mean, the law one deploys or
     plots;
   - is reproducible by one key, with common random numbers across compared laws;
   - holds its draw over the period, so the continuous evaluator's RK4 stages see one action
     per period, the rule the noise block already states;
   - gives pathwise gradients through the fixed `ε` (the reparameterization), so the program
     export and SAC-style updates reach `θ`.

   What the block form does not give, the contract adds as methods beside `h`, not outputs:
   the log-probability of an action and the entropy (PPO and likelihood-based cloning need
   them; a diffusion policy has none), the squashed Gaussian with its change-of-variables
   term. The RL Gaussian and squashed-Gaussian heads become such blocks; moving the RL
   collectors from their own keys to per-episode seeds changes the RL numbers once (a baseline
   and a re-pin, between cohorts). Sized like the constitution's two-method distributions, not
   a probability library.
5. **A dataset noun**: batched, masked trajectories with outputs logged, if the
   behavior-cloning row picks the `fit` route (R136).
6. **Architectures stay out of the core.** The small MLP stays for teaching; larger networks
   come through a thin wrapper, which TODO's Later ideas already list.

### 5.1 D29, the standard stochastic policy as a seeded block

**Choice, 2026-10-27 (R129, [record](../reviews/2026-10-27-design-questions-rulings.md)):**
- one block, `GaussianPolicy(mean, log_std, *, sample_period, seed=None)`, a plain `System`
  whose `h` is `u = μ_θ(x) + σ_θ(x) · ε(t)`;
- its noise internal and deterministic given the seed: `ε(t)` the `WhiteNoise` signal at the
  sample index `⌊t/Δ⌋` of the block's own period (in a sampled diagram, the tick index of R104);
- `seed` an integer leaf, found by the `seed` key and realized per trial (R144); `seed=None` is
  the deterministic face, `NeuralPolicy` (renamed from `NeuralPolicyController` in the v0.3
  sweep, R151);
- `log_prob(x, u)` and `entropy(x)` as methods of the block, not of `Distribution`;
- variants (the squashed Gaussian) are subclasses; the RL package's internal `StochasticPolicy`
  is renamed so `GaussianPolicy` owns the idea (R134).

Open, a placement detail of its row (TODO §1.4): whether the RL heads' rebuild lands with the
block or in v0.4 with C5.

## 6. Timing

The rows and their rungs are TODO §1.4 (the seeded policy, v0.3's core), §1.5 (the sampled loop)
and S72 (MPPI). Behavior cloning stays after v1.0 and diffusion policy after it; with §5's points
in place, both are additive features rather than redesigns.

## 7. Questions for the maintainer

**Choices, 2026-10-27 (R136, [record](../reviews/2026-10-27-design-questions-rulings.md)):**
- how fitting enters the API (a `fit` verb and a dataset noun, or the program export with a
  data-mismatch cost) is decided at the behavior-cloning row, identification (C4) flagging its
  route for it;
- diffusion policy is research-lane only, JAX-only, after behavior cloning;
- distilling MPC or MPPI into a neural policy as the first imitation demo stays a pointer.

The other questions are answered: controller memory on the tick (R104) and the seeded
policy's contract (D29, R129).
