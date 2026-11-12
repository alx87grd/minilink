"""From RL to Bode on a six-axis arm: an impedance loop, a learned set-point law, the Bode plot of the learned loop."""

import jax.numpy as jnp
import numpy as np

from minilink import (
    CostFunction,
    DiagramSystem,
    Gaussian,
    JointImpedance,
    PlanningProblem,
    ReinforcementLearningPlanner,
    Uniform,
    UR5Manipulator,
    evaluate,
)
from minilink.analysis import region_of_attraction

TRAINING_TIMESTEPS = 40_000  # about a minute on a laptop CPU
DT = 0.01  # [s] period of the learned set-point law (the impedance loop is continuous)
EPISODE = 3.0  # [s]

# The arm: a UR5 with a world-frame force f at its tool, τ = u + J(q)ᵀ f, and a 0.5 kg
# gripper on its last link (the catalog wrist has almost no inertia about its own axis)


class UR5WithToolForce(UR5Manipulator):
    def __init__(self):
        super().__init__()
        self.add_input_port("f", dim=3, nominal_value=np.zeros(3))

    def generalized_force(self, q, v, u, t=0.0, params=None):
        tau = u[:6]
        f = u[6:9]

        # τ = u + J(q)ᵀ f
        tau_total = tau + self.J(q, params).T @ f

        return tau_total


arm = UR5WithToolForce()
arm.params["mass"] = np.asarray(arm.params["mass"], dtype=float) + np.array(
    [0, 0, 0, 0, 0, 0.5]
)
arm.params["inertia"] = np.asarray(arm.params["inertia"], dtype=float).copy()
arm.params["inertia"][5] += 0.02 * np.eye(3)

# The task: hold the tool at p*, from a start away from it; q* = IK(p*), x* = [q*; 0]

p_target = np.array([0.3, 0.4, 0.5])  # [m]

q_target = arm.inverse_kinematics(p_target, q_guess=[0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
x_target = arm.q2x(q_target, np.zeros(6))

# The inner loop: gravity-compensated joint impedance, τ = Kp (r − q) − Kd q̇ + g(q).
# Its input is the joint set-point r; the boxes on r and x are the training domain

impedance = JointImpedance(
    arm,
    gravity_comp=True,
    Kp=[100.0, 100.0, 100.0, 10.0, 10.0, 10.0],
    Kd=[20.0, 20.0, 20.0, 1.0, 1.0, 1.0],
)
inner = impedance @ arm
inner.name = "Joint impedance loop"
inner.inputs["r"].lower = q_target - 0.6
inner.inputs["r"].upper = q_target + 0.6
inner.inputs["r"].nominal_value = q_target
inner.state.lower = np.concatenate([q_target - 2.0, -10.0 * np.ones(6)])
inner.state.upper = np.concatenate([q_target + 2.0, 10.0 * np.ones(6)])
inner.x0 = arm.q2x(q_target + np.array([0.4, -0.3, 0.3, -0.4, 0.3, 0.3]), np.zeros(6))

# Before learning: the impedance loop alone, r = q*, and the Bode plot of the vertical
# tool motion from a vertical tool force, P_z / F_z, on the same blocks with f exposed

traj_impedance = inner.compute_trajectory(tf=EPISODE, dt=0.005, backend="jax")
inner.plot_trajectory(traj_impedance, signals=((arm, "p"),))
inner.animate(traj_impedance, renderer="meshcat")

inner_with_force = DiagramSystem()
inner_with_force.name = "Impedance loop, tool force in"
inner_with_force.add_subsystem(impedance, "controller")
inner_with_force.add_subsystem(arm, "arm")
inner_with_force.connect("arm", "y", "controller", "y")
inner_with_force.connect("controller", "u", "arm", "u")
inner_with_force.add_input_port("r", dim=6, nominal_value=q_target)
inner_with_force.connect("input", "r", "controller", "r")
inner_with_force.add_input_port("f", dim=3, nominal_value=np.zeros(3))
inner_with_force.connect("input", "f", "arm", "f")
inner_with_force.plot_bode(x_target, of=("arm:p", 2), wrt=("f", 2), margins=False)

# The cost: g = 10 ‖p(q) − p*‖² + 0.01 ‖q̇‖² + 0.1 ‖r − q*‖², the action being r


class ReachCost(CostFunction):
    def g(self, x, u, t=0.0, params=None):
        e = arm.forward_kinematics(x[:6]) - p_target
        dq = x[6:]
        dr = u - q_target

        g = 10.0 * (e @ e) + 0.01 * (dq @ dq) + 0.1 * (dr @ dr)

        return g

    def h(self, x, t=0.0, params=None):
        return 0.0


# The stochastic problem, posed on the inner loop: a random start, and an unknown
# payload on the last link that the gravity compensation does not know

masses = np.asarray(arm.params["mass"], dtype=float)

problem = PlanningProblem(
    inner,
    cost=ReachCost(),
    horizon=np.inf,
    x0=Gaussian(inner.x0, np.concatenate([0.3 * np.ones(6), 0.1 * np.ones(6)])),
    distributions={
        "plant": {"plant": {"mass": Uniform(masses, masses * [1, 1, 1, 1, 1, 3.0])}}
    },
    U=inner.input_box,
)

# The learned law: a network r = π(z(x)) trained by PPO, its features the joint error,
# the scaled rates and the tool error; the impedance gains stay as they are

planner = ReinforcementLearningPlanner(
    problem,
    dt=DT,
    features=lambda x: jnp.concatenate(
        [
            x[:6] - q_target,
            0.2 * x[6:],
            3.0 * (arm.forward_kinematics(x[:6]) - p_target),
        ]
    ),
    hidden=(64, 64),
    n_rollouts=32,
    n_steps_per_update=64,
    batch_size=256,
    learning_rate=3e-4,
    gamma=0.98,
    episode_length=EPISODE,
)
solution = planner.solve(n_timesteps=TRAINING_TIMESTEPS)
print(solution.record)
planner.plot_learning_curve()

# The network is a controller block: scored on the problem's own draws, then wired
# over the impedance loop, the tool force f left as the diagram's input

neural_law = planner.controller
neural_law.show_setpoint = True  # green ball: the commanded set-point p(r)
neural_law.task_target = p_target  # gold ball: the task p*
print(evaluate(problem, neural_law, dt=DT, n_trials=64, episode_length=EPISODE, key=1))

loop = DiagramSystem()
loop.name = "Learned set-point law over joint impedance"
loop.add_subsystem(neural_law, "rl")
loop.add_subsystem(impedance, "controller")
loop.add_subsystem(arm, "arm")
loop.connect("arm", "y", "rl", "x")
loop.connect("rl", "u", "controller", "r")
loop.connect("arm", "y", "controller", "y")
loop.connect("controller", "u", "arm", "u")
loop.add_input_port("f", dim=3, nominal_value=np.zeros(3))
loop.connect("input", "f", "arm", "f")
loop.plot_diagram()

loop.x0 = inner.x0
traj = loop.compute_trajectory(tf=EPISODE, dt=0.005, backend="jax")
loop.plot_trajectory(traj, signals=((arm, "p"),))
loop.animate(traj, renderer="meshcat")

# After learning: the same Bode plot, of a loop that holds a neural network, at the
# equilibrium the learned law keeps

x_eq = loop.find_equilibrium(x_target)
loop.plot_bode(x_eq, of=("arm:p", 2), wrt=("f", 2), margins=False)

# From how far does it recover: a Lyapunov certificate of the learned loop, drawn on
# the shoulder's angle and rate

certificate = region_of_attraction(loop, x_target)
print(certificate)
print(certificate.verify())
certificate.plot(x_axis=1, y_axis=7)

# Push the tool: held keys command the force f [N] in real time (UP/DOWN f_x,
# RIGHT/LEFT f_y, W/S f_z)

# loop.inputs["f"].lower = -20.0 * np.ones(3)
# loop.inputs["f"].upper = +20.0 * np.ones(3)
# loop.game(renderer="meshcat", is_3d=True, x0=x_eq)
