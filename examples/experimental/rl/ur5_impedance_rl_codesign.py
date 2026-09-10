"""UR5: a learned set-point law over a joint impedance loop, then gains tuned by autodiff through the learned loop."""

import time

import jax
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np

from minilink import CostFunction, DiagramSystem, UR5Manipulator
from minilink.analysis import bode
from minilink.control import JointImpedance
from minilink.core.feedback import Controller
from minilink.planning import (
    Gaussian,
    MonteCarloEvaluator,
    ReinforcementLearningPlanner,
    StochasticPlanningProblem,
    Uniform,
)

TRAINING_TIMESTEPS = 40_000  #
DT = 0.01  # control period of the set-point law (the impedance loop is continuous)
EPISODE = 3.0  # s
PAYLOAD_MAX = 3.0  # the last link's mass is drawn between 1x and 3x its nominal value


# ---------------------------------------------------------------------------
# 1. The classical inner loop: a joint impedance around the UR5, gravity compensated
# ---------------------------------------------------------------------------
class UR5WithToolForce(UR5Manipulator):
    """UR5 with a world-frame force disturbance at the tool: tau = u + J(q)^T f."""

    def __init__(self):
        super().__init__()
        self.add_input_port(
            "f",
            dim=3,
            nominal_value=np.zeros(3),
            labels=["fx", "fy", "fz"],
            units=["N"] * 3,
        )

    def generalized_force(self, q, v, u, t=0.0, params=None):
        tau, f = u[:6], u[6:9]
        return tau + self.J(q, params).T @ f


arm = UR5WithToolForce()
# A 0.5 kg gripper on the last link: the catalog wrist has almost no inertia about
# its own axis, which no damping can be tuned for at a finite time step.
arm.params["mass"] = np.asarray(arm.params["mass"], dtype=float) + np.array(
    [0, 0, 0, 0, 0, 0.5]
)
arm.params["inertia"] = np.asarray(arm.params["inertia"], dtype=float).copy()
arm.params["inertia"][5] += 0.02 * np.eye(3)

q_guess = np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
p_target = np.array([0.3, 0.4, 0.5])  # m, where the end effector must go
q_target = arm.inverse_kinematics(p_target, q_guess=q_guess)
x_target = arm.q2x(q_target, np.zeros(6))

Kp = np.array([100.0, 100.0, 100.0, 10.0, 10.0, 10.0])  # Nm/rad, wrist joints are light
Kd = np.array([20.0, 20.0, 20.0, 1.0, 1.0, 1.0])
impedance = JointImpedance(arm, gravity_comp=True, Kp=Kp, Kd=Kd)
inner = (
    impedance @ arm
)  # a System: input r (joint set-points), output y (the arm state)
inner.name = "Joint impedance loop"
inner.inputs["r"].lower_bound = (
    q_target - 0.6
)  # the set-point box the outer law may use
inner.inputs["r"].upper_bound = q_target + 0.6
inner.state.lower_bound = np.concatenate([q_target - 2.0, -10.0 * np.ones(6)])
inner.state.upper_bound = np.concatenate([q_target + 2.0, 10.0 * np.ones(6)])
inner.x0 = arm.q2x(q_target + np.array([0.4, -0.3, 0.3, -0.4, 0.3, 0.3]), np.zeros(6))
inner.plot_diagram()


# ---------------------------------------------------------------------------
# 2. The task as a stochastic planning problem on the inner loop
# ---------------------------------------------------------------------------
# Cost: end-effector distance to the target (through the forward kinematics),
# a little joint speed, a little set-point effort
class ReachCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        q, dq = x[:6], x[6:]
        e = arm.forward_kinematics(q) - p_target
        return (
            10.0 * (e @ e) + 0.01 * (dq @ dq) + 0.1 * ((u - q_target) @ (u - q_target))
        )

    def h(self, x, t=0.0, params=None):
        return 0.0


# What is random: the start (around a pose away from the target) and the payload,
# an unknown mass on the last link that the gravity compensation does not know
masses = np.asarray(arm.params["mass"], dtype=float)
payload = Uniform(
    masses, masses * np.array([1, 1, 1, 1, 1, PAYLOAD_MAX])
)  # only the last link varies


problem = StochasticPlanningProblem(
    inner,
    cost=ReachCost(),
    tf=np.inf,
    x0_distribution=Gaussian(
        inner.x0, np.concatenate([0.3 * np.ones(6), 0.1 * np.ones(6)])
    ),
    params_distribution={"sys.mass": payload},  # the arm inside the loop
)
evaluator = MonteCarloEvaluator(
    problem, dt=DT, n_trials=64, episode_length=EPISODE, seed=1
)


# The baseline: the inner loop alone, set-point fixed at the target
class ConstantSetpoint(Controller):
    measurement_port = "x"
    ref_port = None
    control_port = "u"
    plot_space = "state"

    def __init__(self):
        super().__init__()
        self.name = "constant set-point"
        self.add_input_port("x", dim=12)
        self.add_output_port("u", dim=6, function=self.ctl, dependencies=("x",))

    def ctl(self, x, u, t=0, params=None):
        return jnp.asarray(q_target) + 0.0 * u[:6]


baseline = evaluator.evaluate(ConstantSetpoint())
print("Impedance loop alone, random starts and payloads:", baseline)


# ---------------------------------------------------------------------------
# 3. The learned outer loop: a neural set-point law trained by PPO
# ---------------------------------------------------------------------------
def features(x):
    q, dq = x[:6], x[6:]
    return jnp.concatenate(
        [q - q_target, 0.2 * dq, 3.0 * (arm.forward_kinematics(q) - p_target)]
    )


planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=features,
    hidden=(64, 64),
    n_envs=32,
    n_steps=64,
    batch_size=256,
    learning_rate=3e-4,
    gamma=0.98,
    episode_length=EPISODE,
)
t0 = time.time()
plan = planner.solve(timesteps=TRAINING_TIMESTEPS, n_trials=64)
print(f"Trained in {time.time() - t0:.0f} s. {plan.metadata.message}")
rl_ctl = planner.get_controller()
learned = evaluator.evaluate(rl_ctl)
print("With the learned set-point law:               ", learned)
planner.plot_learning_curve()

# ---------------------------------------------------------------------------
# 4. The whole thing is one diagram: learned law > impedance > arm
# ---------------------------------------------------------------------------
loop = DiagramSystem()
loop.name = "Learned set-point law > joint impedance > UR5"
loop.add_subsystem(rl_ctl, "rl")
loop.add_subsystem(impedance, "ctl")
loop.add_subsystem(arm, "arm")
loop.connect("arm", "y", "rl", "x")
loop.connect("rl", "u", "ctl", "r")
loop.connect("arm", "y", "ctl", "y")
loop.connect("ctl", "u", "arm", "u")
loop.plot_diagram()

# Its equilibrium and poles, by autodiff through the network, the impedance law and the arm
x_eq = loop.find_equilibrium(x_target)
poles_inner = np.linalg.eigvals(inner.linearize(x_target, q_target).A())
poles_loop = np.linalg.eigvals(loop.linearize(x_eq).A())
print(
    "End-effector at the loop's equilibrium:",
    np.round(np.asarray(arm.forward_kinematics(x_eq[:6])), 3),
)
print(
    "Slowest pole, impedance loop alone:", np.round(np.max(poles_inner.real), 2), "1/s"
)
print(
    "Slowest pole, with the learned law:  ", np.round(np.max(poles_loop.real), 2), "1/s"
)

# ---------------------------------------------------------------------------
# 5. Co-design: tune the impedance gains by gradient, through the learned law
# ---------------------------------------------------------------------------
# The Monte Carlo cost as a differentiable function of the classical gains,
# with the trained neural law in the loop and the arm's physics in between.
evaluator_jax = loop.compile(backend="jax")
n_steps = int(EPISODE / DT)
rng = np.random.default_rng(2)
x0s = jnp.asarray(problem.sample_x0(rng, n=16))
payloads = jnp.stack(
    [jnp.asarray(problem.sample_params(rng)["sys"]["mass"]) for _ in range(16)]
)
u_none = jnp.zeros(0)


def episode_cost(gains, x0, mass):
    params = dict(loop.params)
    params["ctl"] = {**loop.params["ctl"], "Kp": gains["Kp"], "Kd": gains["Kd"]}
    params["arm"] = {**loop.params["arm"], "mass": mass}

    def step(carry, k):
        x, t = carry
        r = rl_ctl.action(x)
        J_k = problem.cost.g(x, r, t) * DT
        x_next = evaluator_jax.rk4_step_trace_p(x, u_none, t, DT, params)
        return (x_next, t + DT), J_k

    _, J_k = jax.lax.scan(step, (x0, 0.0), jnp.arange(n_steps))
    return jnp.sum(J_k)


def monte_carlo_cost(gains):
    return jnp.mean(jax.vmap(episode_cost, in_axes=(None, 0, 0))(gains, x0s, payloads))


cost_and_grad = jax.jit(jax.value_and_grad(monte_carlo_cost))
gains = {"Kp": jnp.asarray(Kp), "Kd": jnp.asarray(Kd)}
print("\nGradient descent on the impedance gains, through the learned law:")
for it in range(3):
    J, g = cost_and_grad(gains)
    print(
        f"  iteration {it}: J = {float(J):.4f}   Kp = {np.round(np.asarray(gains['Kp']), 1)}   Kd = {np.round(np.asarray(gains['Kd']), 2)}"
    )
    # geometric steps keep the gains positive
    gains = {
        k: gains[k]
        * jnp.exp(
            -0.3
            * jnp.sign(g[k])
            * jnp.minimum(1.0, jnp.abs(g[k] * gains[k]) / (float(J) + 1e-9))
        )
        for k in gains
    }
J_tuned, _ = cost_and_grad(gains)
print(f"  tuned: J = {float(J_tuned):.4f}")


# ---------------------------------------------------------------------------
# 6. Disturbance sensitivity: a force at the tool -> tool position, in the frequency domain
# ---------------------------------------------------------------------------
# The same blocks, wired with the tool force f as the boundary input and the
# tool position p as the boundary output. Linearized at the equilibrium, each
# loop is an LTI system the frequency tools read like any other.
def loop_with_tool_force(outer=None):
    d = DiagramSystem()
    if outer is not None:
        d.add_subsystem(outer, "rl")
    d.add_subsystem(impedance, "ctl")
    d.add_subsystem(arm, "arm")
    d.connect("arm", "y", "ctl", "y")
    d.connect("ctl", "u", "arm", "u")
    if outer is not None:
        d.connect("arm", "y", "rl", "x")
        d.connect("rl", "u", "ctl", "r")
    else:
        d.add_input_port("r", dim=6, nominal_value=q_target)
        d.connect("input", "r", "ctl", "r")
    d.add_input_port("f", dim=3, nominal_value=np.zeros(3))
    d.connect("input", "f", "arm", "f")
    d.connect_new_output_port("arm", "p", "p")
    return d


inner_f = loop_with_tool_force()
inner_f.name = "Joint impedance loop, tool force in, tool position out"
learned_f = loop_with_tool_force(rl_ctl)
learned_f.name = "Learned law > joint impedance > UR5, tool force in, tool position out"
learned_f.plot_diagram()

axis = 2  # vertical force -> vertical tool displacement
w, mag_inner, phase_inner = bode(
    inner_f,
    x_target,
    np.concatenate([q_target, np.zeros(3)]),
    of=("p", axis),
    wrt=("f", axis),
)
_, mag_learned, phase_learned = bode(
    learned_f, x_eq, np.zeros(3), of=("p", axis), wrt=("f", axis)
)
fig, (ax_mag, ax_phase) = plt.subplots(2, 1, sharex=True, figsize=(7, 5))
ax_mag.semilogx(w, mag_inner, label="impedance loop alone")
ax_mag.semilogx(w, mag_learned, label="with the learned set-point law")
ax_mag.set_ylabel("|p_z / f_z| [dB re 1 m/N]")
ax_mag.legend()
ax_phase.semilogx(w, phase_inner)
ax_phase.semilogx(w, phase_learned)
ax_phase.set_ylabel("phase [deg]")
ax_phase.set_xlabel("frequency [rad/s]")
for ax in (ax_mag, ax_phase):
    ax.grid(True, which="both", alpha=0.3)
ax_mag.set_title("Tool compliance to a vertical force disturbance")
print(
    f"\nStatic compliance of the tool, vertical: {1e3 * 10 ** (mag_inner[0] / 20):.2f} mm/N (impedance loop), "
    f"{1e3 * 10 ** (mag_learned[0] / 20):.2f} mm/N (with the learned law)"
)

# ---------------------------------------------------------------------------
# 7. Watch the learned loop on the real (continuous-time) closed loop
# ---------------------------------------------------------------------------
arm.x0 = inner.x0
traj = loop.compute_trajectory(tf=EPISODE, dt=0.005, compile_backend="jax")
loop.plot_trajectory(traj, signals=((arm, "p"),))
plt.show()
loop.animate(traj, renderer="meshcat")
