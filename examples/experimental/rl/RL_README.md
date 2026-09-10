# Tuning PPO on minilink plants: lessons and tips

Notes from getting `minilink.experimental.ppo_jax.PPO` to learn the demos in
this folder (September 2026). Every number below was measured on a laptop CPU
(Apple M4) with the pure-JAX implementation: the plant compiled with the JAX
backend, the rollout a `lax.scan` over the RK4 step with the policy sampled
inside, and the whole PPO update one jitted call.

## The demos and what they took

| demo | plant | budget | training | what the policy learns |
| --- | --- | --- | --- | --- |
| `drone_ppo_jax_learn_to_fly.ipynb` | Drone2D, normalized thrusts | 100k steps | 4 s | hover regulation from Gaussian starts (the SB3 notebook takes ~28 s for the same budget) |
| `pendulum_ppo_jax_swing_up.py` | Pendulum, torque 4 Nm vs 9.8 Nm of gravity | 120k steps | 2 s | one back-swing, one forward swing, catch and balance |
| `cartpole_ppo_jax_swing_up.py` | CartPole, force 10 N | 1M steps | 30 s | swing-up and balance, cart back to center |
| `car_ppo_jax_circuit.py` | BicycleDynRate on the MPC circuit | 1.5M steps | 50 s | laps inside the corridor with the MPC demo's cost, about 9.5 s per 103 m lap at a 12 m/s target |
| `rocket_ppo_jax_landing.py` | Rocket, one-sided thrust and gimbal | 4M steps | 80 s | free fall, braking burn, settles on the pad |

Also solved, not kept as a demo: the mountain car (throttle 1 N against 1.57 N
of slope, 700k steps, 8 s of training, hilltop in 5.7 s from rest).

Not achieved: the Acrobot swing-up (elbow torque 10 Nm). Ten million steps
with periodic features, tip-height shaping, several exit rules and step sizes
never produced a deterministic swing-up from hanging. Ideas not tried yet:
energy-based shaping, a curriculum on the initial states, longer horizons with
a larger critic.

## 1. The state-box exit is the exploit to watch for

The single most common failure. Symptom: the *exploration* return looks great
while the *deterministic* policy does something absurd. Cause: an episode
that leaves the state bounds ends, and ending an episode is often the cheapest
thing the policy can do.

- With `domain_exit="truncate"` (the Gymnasium bridge semantics) the reward is
  bootstrapped with the critic's value at the exit state, which the critic
  never sees in training and extrapolates optimistically. The cart-pole spun
  its pole past the angle bound in 71 % of episodes, the Acrobot spun its
  elbow, the rocket cut its thrust and fell through the ground.
- With `domain_exit="terminate"` and a zero terminal cost, leaving is free and
  the policy leaves: the cart drove out of the box at 450 m.
- The car found two exits in a row: it braked to the wheel-speed lower bound
  at the first corner, then cranked the steering to its 0.55 rad limit.

Rules that held up:

1. Decide per bound whether it is a *physical event* (ground contact, a
   steering stop) or just a *training box*. Widen the training-box faces until
   they are essentially never reached (velocities, wheel rates, angles that
   periodic features already handle).
2. Terminate the physical events with `domain_exit="terminate"` and a terminal
   cost `h(x)` that beats the cost-to-go of doing the task. Too small and the
   policy escapes (rocket with 30: flies away; with 100: lands). Too large and
   it turns timid (cart-pole with 50 never dared to swing; without the penalty
   it learned in 600k steps). The car needed 200, the mountain car 100.
3. Diagnose with the rollout batch: the fraction of episode ends that sit near
   each bound, and the mean episode length. Eight-step episodes mean the task
   is unlearnable as posed (the rocket with the catalog inertia).

## 2. Features: periodic angles, order-one scaling, task-relative sensing

- Feed `cos θ, sin θ` instead of a raw angle. The cart-pole trained on raw
  angles reached the top, overshot, and kept spinning into a region the
  network had never seen. With periodic features the angle bound can be
  removed entirely.
- Scale every feature to order one (`0.1 * dθ`, `0.25 * (y - y_target)`).
  The rocket went from "close" to "lands from every start" by raising the
  position scale from 0.1 to 0.25: a metre of error must be visible to a
  tanh network.
- Give the policy what a sensor would give it. The circuit car sees the
  signed lateral offset, the heading error, its body speeds and the
  body-frame direction to three lookahead points, not the absolute pose. It
  then generalizes along the whole loop from random starts on it.
- The learned law is still `u = pi(x)`: `features` is applied inside the
  networks, so the controller block composes with the plant like any other.

## 3. Actions and exploration

- The policy acts on a normalized `a in [-1, 1]` mapped to the port bounds.
  Exploration with `log_std_init=0` then covers the full input range from the
  first step, which is what the swing-ups need (a 1 N noise on a 10 N input
  learned nothing).
- Lower the initial log-std only when the plant is twitchy. The rocket needed
  `-1`; the pendulum was three times slower with it.
- Bang-bang laws are normal after training (log-std drifts to about -2 to
  -3 and the mean saturates); they are fine for swing-ups, and a heavier
  effort weight smooths them when needed.

## 4. Cost shaping and reward scale

- Swing-ups: `1 + cos θ` (pendulum, cart-pole) or the tip-height deficit
  (Acrobot). Periodic, zero at the goal, bounded.
- Velocity and effort weights must not punish the maneuver. `0.01 dθ²` killed
  the cart-pole pump (it needs 10 rad/s); `0.001` worked. The pendulum liked
  `0.01` on both and failed with `0.001`: look at the deterministic swing and
  adjust, there is no universal value.
- Keep episode returns of the order of 10 to 100. The rocket with a unit
  position weight had returns in the thousands and the critic never caught
  up; scaling the whole cost by 0.1 fixed it. This is the hand-made version of
  reward normalization.
- Reusing planner costs works: the car trains on the MPC demo's quadratic
  terms plus the track distance and corridor fields, with the input weights
  scaled down because they now act per step.

## 5. Discount, horizon, and the task's time scale

- Match `gamma` to the maneuver. The pendulum swing is about 2 s: `gamma =
  0.97` (1.7 s horizon at 20 Hz) was fastest and `0.99` never learned it in
  1M steps. The rocket's lateral motion is slow: it needed `0.995`.
- `tf` must contain the whole maneuver plus the settling. A 6 s horizon made
  the pendulum unlearnable; 10 s was fine. The rocket wanted 15 s.
- A coarser `dt` (0.1 s) halved the pendulum's step count for the same
  simulated time and still worked; the rocket and car needed 0.05 s.

## 5b. Faster laps: raising the car's cruise target

Same cost, same corridor, only `U_TARGET` changed; 2M steps each, deterministic
30 s test from the start line, best snapshot during training:

| cruise target | laps in 30 s | mean speed | lap time | first clean lap at |
| --- | --- | --- | --- | --- |
| 6 m/s | 1.75 | 5.8 m/s | 17 s | 250k steps |
| 10 m/s | 2.98 | 9.4 m/s | 10.1 s | 250k steps |
| 12 m/s | 3.19 | 10.3 m/s | 9.4 s | 400k steps |
| 14 m/s | 3.27 | 10.4 m/s | 9.2 s | 840k steps |
| 15 m/s | leaves the track | | | never |

The gain saturates around 10 m/s of mean speed: the corridor and the tire
friction (mu = 1, corner radii of a few metres) set the limit, not the target.
Higher targets also learn later and push the driven line to the corridor edge
(max distance 2.0 to 2.2 m of 2.5). Beyond this point the objective itself has
to change, e.g. rewarding progress along the track instead of a cruise speed.

## 6. Initial-state distribution

- Spread the starts over the whole task. Uniform initial angles over the
  circle (pendulum, cart-pole, Acrobot) beat Gaussian starts near the hanging
  position, which failed outright on the pendulum.
- The callable `reset_mode` draws task-specific starts: the car starts at a
  random arc length on the loop, within a metre of the centerline, roughly
  aligned, at 2 to 8 m/s.

## 7. Hyperparameters for speed (small plants)

Measured on the pendulum, three seeds each, steps to a confirmed swing-up:

| change from the defaults (16 plants x 128 steps, lr 3e-4, 64 units) | steps | training |
| --- | --- | --- |
| baseline | 197k | 13 s |
| lr 1e-3, 64 plants x 32 steps | 98k to 115k | 4 s |
| plus 32-unit layers | 115k to 131k | 2.5 s |
| plus lr 3e-3 | 82k | 2.1 s |
| plus gamma 0.97 | 65k | 1.9 s |

- The learning rate can be ten times the usual default on these small
  problems (3e-3 was stable on the pendulum and mountain car; 1e-3 on the
  rocket and Acrobot).
- Wider vmaps with shorter rollouts (64 x 32) keep 2048 samples per update
  and cut the wall time because the update, not the simulation, dominates.
- A 32-unit network is enough for one- and two-state plants and trains about
  twice as fast as 64. The car and rocket used 64.
- Minibatches of 256 and 10 epochs were best; 512 was slower to learn, 20
  epochs bought nothing.

## 8. The plant must be learnable by random exploration

The catalog rocket has an inertia of 100 kg m² for 1000 kg; a 0.05 rad gimbal
tumbles it in a second and episodes lasted eight steps. With a rocket-like
1000 kg m² and the gimbal capped at 0.05 rad, random exploration survives long
enough to learn. Check the open-loop response to a small constant input before
training anything.

## 9. Diagnostics that found every bug

- Print the exploration return *and* a deterministic test from fixed starts
  every few iterations; a gap between them is the exit exploit or a test bug.
- Roll the deterministic policy out in the *training* environment (same `dt`,
  ZOH) and print the time series; then confirm in the continuous closed loop
  (`ppo.controller @ plant`, `compute_trajectory`). Both should agree.
- Episode-end statistics from one rollout batch: mean length, fraction of
  ends near each bound, mean |a| and the log-std.
- `plot_control_law` slices: the cart-pole pitch loop and the pendulum's
  energy-pumping bands are visible at a glance.

## 10. Test-harness pitfalls (both happened here)

- Measure the right angle: the error to the upright position is
  `|mod(θ, 2π) - π|`. A sign slip reported a perfect balance as a 3.14 rad
  failure through six sweeps.
- Do not close over the weights in a jitted evaluation. `jax.jit` traces the
  weights as constants, so the test kept scoring the *initial* policy. Pass
  them as an argument: `det_roll(weights, x0)`.

## 11. Why it is fast

At this scale nothing is compute-bound. One `lax.scan` per rollout and one
jitted call per update remove the per-step Python and framework dispatch that
dominates a classic loop. Throughput on the CPU: 30k to 90k plant steps per
second including the update, roughly ten times the Gymnasium plus
stable-baselines3 path on the same drone task. The first iteration pays a
second or two of compilation.
