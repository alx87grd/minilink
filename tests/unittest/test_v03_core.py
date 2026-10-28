"""The v0.3 core's definition of done: one test per acceptance script, named by its number and lane."""

import numpy as np
import pytest

from minilink import (
    PID,
    CostFunction,
    DiagramSystem,
    Integrator,
    P,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    Sine,
    Step,
    StepSystem,
    WhiteNoise,
    closed_loop,
)
from minilink.analysis import discretize
from minilink.core.backends import array_module
from minilink.core.discrete.sampled_diagram import SampledDiagramSystem


class LowPass(StepSystem):
    """``z_{k+1} = α z_k + (1 − α) u_k``, ``y = z``."""

    def __init__(self, alpha=0.6):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)

        alpha = params["alpha"]

        # z_{k+1} = α z_k + (1 − α) u_k
        z_next = xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

        return z_next

    def h(self, x, u, k=0, params=None):
        return x


def digital_pid_problem():
    """A digital PID on the pendulum, its setpoint 1 rad, scored over 2 s."""
    plant = Pendulum()
    plant.solver_info["smallest_time_constant"] = 0.005
    cost = QuadraticCost.from_system(plant, xbar=np.array([1.0, 0.0]))
    problem = PlanningProblem(plant, cost=cost, x_start=np.zeros(2), tf=2.0)
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    pid.inputs["e"].nominal_value = np.zeros(1)
    return problem, pid


# Script 1

LQG_DT, LQG_TF = 0.01, 5.0
PROCESS_PSD = 0.04  # W, the torque's white-noise intensity
SENSOR_STD = 1.0  # the rate sensor's per-sample standard deviation over LQG_DT


def lqg_problem():
    """An LQG compensator on the noisy pendulum: white noise on the torque, a sampled sensor on the rate."""
    from minilink import Gaussian, NoisyPendulum
    from minilink.blocks import NoiseSource
    from minilink.control.design import lqg

    plant = NoisyPendulum()
    Q, r = np.diag([10.0, 1.0]), 0.1
    V = SENSOR_STD**2 * LQG_DT  # the sensor's intensity: its variance over its period
    box = lqg(
        plant,
        Q,
        np.array([[r]]),
        Q_n=np.diag([0.0, PROCESS_PSD]),
        R_n=np.diag([1e-4, V]),
    )
    cost = QuadraticCost.from_system(plant, Q=Q, R=np.diag([r, 0.0, 0.0]))
    problem = PlanningProblem(
        plant,
        cost=cost,
        x_start=np.zeros(2),
        tf=LQG_TF,
        disturbances={
            "d": WhiteNoise(1, psd=PROCESS_PSD, sample_period=LQG_DT, seed=1),
            "n": NoiseSource(
                Gaussian([0.0], [SENSOR_STD]), sample_period=LQG_DT, seed=2
            ),
        },
    )
    return problem, box, Q, r, V


def predicted_cost(plant, box, Q, r, V):
    """``E[J] = ∫₀ᵀ tr(M P(t)) dt``, ``Ṗ = A P + P Aᵀ + N``: the loop's covariance from a zero start."""
    from minilink.analysis import linearize

    lin = linearize(plant)
    K = box.subsystems["gain"].params["K"]
    L = box.subsystems["observer"].params["L"]
    A, C = lin.A(), lin.C()
    B_u, B_d, D_n = lin.B()[:, :1], lin.B()[:, 1:2], lin.D()[:, 2:3]

    # X = [x; x̂]: ẋ = A x − B K x̂ + B_d d, the estimate driven by y = C x + D_n n
    A_cl = np.block([[A, -B_u @ K], [L @ C, A - B_u @ K - L @ C]])
    B_w = np.vstack([B_d, np.zeros((2, 1))])
    B_v = np.vstack([np.zeros((2, 1)), L @ D_n])
    N = PROCESS_PSD * B_w @ B_w.T + V * B_v @ B_v.T
    M = np.zeros((4, 4))
    M[:2, :2] = Q
    M[2:, 2:] = r * K.T @ K

    # The Lyapunov differential equation by RK4, E[J] by the trapezoid
    h = 1e-3
    P = np.zeros((4, 4))
    expected = 0.0
    for _ in range(round(LQG_TF / h)):

        def rate(P):
            return A_cl @ P + P @ A_cl.T + N

        k1 = rate(P)
        k2 = rate(P + 0.5 * h * k1)
        k3 = rate(P + 0.5 * h * k2)
        k4 = rate(P + h * k3)
        P_next = P + h / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        expected += 0.5 * h * (np.trace(M @ P) + np.trace(M @ P_next))
        P = P_next

    return expected


@pytest.mark.optional
@pytest.mark.jax
def test_script_01_objects_an_lqg_loop_scores_its_covariance_prediction():
    pytest.importorskip("jax")
    problem, box, Q, r, V = lqg_problem()
    expected = predicted_cost(problem.sys, box, Q, r, V)

    # The Monte Carlo mean within the sampling error of the prediction (3 standard errors)
    runs = problem.compile(box, dt=LQG_DT, backend="jax").evaluate(n_trials=400, key=1)
    J = np.asarray(runs.J)
    assert abs(J.mean() - expected) < 3.0 * J.std(ddof=1) / np.sqrt(J.size)

    # One key, the same draws on NumPy and JAX
    on_numpy = problem.compile(box, dt=LQG_DT, backend="numpy").evaluate(
        n_trials=4, key=1
    )
    np.testing.assert_allclose(np.asarray(on_numpy.J), J[:4], rtol=1e-9)

    # The computer sampled: its noise blocks realized per trial, the same prediction
    sampled = problem.compile(box % LQG_DT, backend="jax").evaluate(n_trials=400, key=1)
    J_s = np.asarray(sampled.J)
    assert len(set(J_s[:10])) == 10
    assert abs(J_s.mean() - expected) < 3.0 * J_s.std(ddof=1) / np.sqrt(J_s.size)


# Script 2


@pytest.mark.optional
@pytest.mark.jax
def test_script_02_loop_a_digital_pid_has_a_gradient():
    jax = pytest.importorskip("jax")
    problem, pid = digital_pid_problem()
    pe = problem.compile(pid % 0.01)
    assert pe.dt == 0.01 and pe.backend == "jax"
    params = pe.params
    params["r"] = np.array([1.0])

    grad = jax.grad(lambda p: pe.evaluate(p).objective)(params)

    eps = 1e-6
    for gain in ("Kp", "Ki", "Kd"):
        up = {**params, "controller": {**params["controller"]}}
        down = {**params, "controller": {**params["controller"]}}
        up["controller"][gain] = params["controller"][gain] + eps
        down["controller"][gain] = params["controller"][gain] - eps
        central = (pe.evaluate(up).objective - pe.evaluate(down).objective) / (2 * eps)
        np.testing.assert_allclose(grad["controller"][gain], central, rtol=1e-6)

    with pytest.raises(ValueError, match="own clock"):
        problem.compile(pid % 0.01, dt=0.02)


# Script 3


def test_script_03_loop_the_run_verbs_and_the_score_give_one_run():
    problem, pid = digital_pid_problem()
    loop = (pid % 0.01) @ problem.sys
    loop.inputs["r"].nominal_value = np.array([1.0])
    pe = problem.compile(loop, backend="numpy")
    S = loop.subsystems["plant"].substeps

    scored = pe.trajectory()
    ticks = loop.compute_rollout(200)
    dense = loop.compute_trajectory(tf=2.0)
    host = loop.compute_trajectory(tf=2.0, solver="scipy")

    np.testing.assert_array_equal(scored.x, ticks.x)
    np.testing.assert_array_equal(dense.x[:, ::S], ticks.x)
    np.testing.assert_allclose(host.x, dense.x, atol=1e-7)


# Script 10


def multi_rate_loop():
    """A 100 Hz filter and a 10 Hz gain on a continuous integrator, the reference a step."""
    return closed_loop(
        P(10.0, ports="reference") % 0.1,
        discretize(Integrator(), 0.01, substeps=5),
        reference=Step(np.array([1.0]), np.array([0.0]), step_time=1.0),
        filter=LowPass(alpha=0.6) % 0.01,
    )


@pytest.mark.parametrize("backend", ["numpy", "jax"])
def test_script_10_loop_a_multi_rate_computer_on_a_continuous_plant(backend):
    if backend == "jax":
        pytest.importorskip("jax")
    loop = multi_rate_loop()
    assert isinstance(loop, SampledDiagramSystem)
    assert loop.sample_period == 0.01
    held = loop.hold_index[(("controller",), "u")]

    run = loop.compute_rollout(200, backend=backend)

    # The same loop by hand, the filter read at the same tick (R125), the gain
    # firing on k mod 10 = 0 and its command held in between
    x_f, x_p, u_held = 0.0, 0.0, 0.0
    states = []
    for k in range(200):
        r = 1.0 if 0.01 * k < 1.0 else 0.0
        if k % 10 == 0:
            u_held = 10.0 * (r - x_f)
        x_f = 0.6 * x_f + 0.4 * x_p
        x_p = x_p + 0.01 * u_held
        states.append((x_f, x_p, u_held))
    a, b = loop.state_index["plant"]
    np.testing.assert_allclose(
        run.x[a, 1:], [s[1] for s in states], rtol=1e-12, atol=1e-14
    )
    np.testing.assert_allclose(
        run.x[held[0], 1:], [s[2] for s in states], rtol=1e-12, atol=1e-14
    )


# Script 11


def test_script_11_loop_a_bare_reference_reads_seconds():
    dt = 0.01
    plant = Pendulum()
    pid = PID(Kp=8.0, Ki=1.0, Kd=2.0)
    bare = Sine(amplitude=0.5, omega=2.0) >> (pid % dt) @ plant
    clocked = (Sine(amplitude=0.5, omega=2.0) % dt) >> (pid % dt) @ plant
    np.testing.assert_array_equal(
        bare.compute_rollout(100).x, clocked.compute_rollout(100).x
    )

    # A white noise on its own period reads seconds: bare in the loop, the draws
    # of its continuous diagram at the ticks, from the same seed
    noise = WhiteNoise(1, psd=0.1, sample_period=0.05, seed=3)
    sampled = SampledDiagramSystem()
    sampled.add_subsystem(noise, "noise")
    sampled.add_subsystem(discretize(Integrator(), dt), "plant")
    sampled.connect("noise", "y", "plant", "u")
    continuous = DiagramSystem()
    continuous.add_subsystem(noise, "noise")
    continuous.add_subsystem(Integrator(), "plant")
    continuous.connect("noise", "y", "plant", "u")
    ev = sampled.compile()
    ticks = sampled.compute_rollout(40)
    draws = [ev.signals(ticks.x[:, k], np.zeros(0), k)[0] for k in range(40)]
    t = dt * np.arange(40)
    np.testing.assert_array_equal(draws, [noise.h(None, None, tk)[0] for tk in t])
    traj = continuous.compute_trajectory(
        tf=0.4, dt=dt, solver="euler_fixedsteps", verbose=False
    )
    np.testing.assert_allclose(ticks.x[0], traj.x[0], rtol=1e-12, atol=1e-15)


class SineTracking(CostFunction):
    """``g = (θ − a sin(ω t + φ))² + r u²``, ``h = (θ − a sin(ω t + φ))²``: a reference in absolute time."""

    def __init__(self, phase=0.0):
        self.params = {"phase": float(phase), "r": 0.01}

    def error(self, x, t, params):
        xp = array_module(x, t)
        phase = params["phase"]

        # e = θ − a sin(ω t + φ)
        e = x[0] - 0.3 * xp.sin(2.0 * t + phase)

        return e

    def g(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        r = params["r"]

        e = self.error(x, t, params)

        # g = e² + r u²
        g = e**2 + r * u[0] ** 2

        return g

    def h(self, x, t=0.0, params=None):
        params = self.params if params is None else params

        e = self.error(x, t, params)

        return e**2


def tracking_problem(t_start, phase):
    plant = Pendulum()
    return PlanningProblem(
        plant,
        x_start=[0.2, 0.0],
        cost=SineTracking(phase),
        tf=2.0,
        t_start=t_start,
        discount_rate=0.3,
    )


@pytest.mark.parametrize(
    "backend", ["numpy", pytest.param("jax", marks=pytest.mark.jax)]
)
@pytest.mark.parametrize("sampled", [False, True], ids=["continuous", "sampled"])
def test_script_12_program_a_loop_started_at_100_scores_as_from_0_shifted(
    backend, sampled
):
    def controller():
        pd = PID(Kp=8.0, Kd=2.0)
        return pd % 0.01 if sampled else pd

    # From t = 100 with sin(2t), and from 0 with the reference shifted by 2·100
    late = tracking_problem(100.0, 0.0).compile(controller(), dt=0.01, backend=backend)
    early = tracking_problem(0.0, 200.0).compile(controller(), dt=0.01, backend=backend)
    J_late = float(late.evaluate().J[0])
    J_early = float(early.evaluate().J[0])
    assert J_late == pytest.approx(J_early, rel=1e-9)

    # The run is stamped in absolute time, and the discount counts from t_start
    traj = late.trajectory()
    assert traj.t[0] == 100.0 and traj.t[-1] == pytest.approx(102.0)
    undiscounted = tracking_problem(100.0, 0.0).replace(discount_rate=0.0)
    J_flat = float(
        undiscounted.compile(controller(), dt=0.01, backend=backend).evaluate().J[0]
    )
    assert J_flat > J_late > 0.5 * J_flat  # e^{-0.3 (t - 100)} ≥ e^{-0.6}, not e^{-30}


@pytest.mark.jax
def test_script_12_program_the_gradient_in_t_start_matches_a_central_difference():
    import jax

    pe = tracking_problem(100.0, 0.0).compile(
        PID(Kp=8.0, Kd=2.0), dt=0.01, backend="jax"
    )
    params = pe.merged({})
    grad = jax.grad(lambda p: pe.evaluate(p).objective)(params)

    h = 1e-5
    J_plus = float(pe.evaluate(pe.merged({"t_start": 100.0 + h})).objective)
    J_minus = float(pe.evaluate(pe.merged({"t_start": 100.0 - h})).objective)
    central = (J_plus - J_minus) / (2 * h)
    assert float(grad["t_start"]) == pytest.approx(central, rel=1e-5)
    assert abs(central) > 1e-3  # the reference moves under the run


@pytest.mark.parametrize(
    "backend", ["numpy", pytest.param("jax", marks=pytest.mark.jax)]
)
@pytest.mark.parametrize("transcription", ["direct_collocation", "multiple_shooting"])
def test_script_12_program_a_plan_from_100_tracks_as_from_0(backend, transcription):
    from minilink.planning import TrajectoryOptimizationPlanner

    def plan(t_start, phase):
        problem = tracking_problem(t_start, phase).replace(discount_rate=0.0)
        planner = TrajectoryOptimizationPlanner(
            problem,
            n_steps=11,
            transcription=transcription,
            backend=backend,
            optimizer_options={"maxiter": 100},
        )
        return planner.solve()

    late, early = plan(100.0, 0.0), plan(0.0, 200.0)
    np.testing.assert_allclose(late.trajectory.t, 100.0 + early.trajectory.t)
    np.testing.assert_allclose(late.trajectory.u, early.trajectory.u, atol=1e-6)
    assert late.solver.cost == pytest.approx(early.solver.cost, rel=1e-6)


def bicycle_planner(mu=1.0, target_y=0.0, *, method="scipy_slsqp", backend="jax"):
    """The rate bicycle's lane-keeping MPC (the e4 parity scenario), its friction and its lane given."""
    from minilink import BicycleDynRate
    from minilink.planning import TrajectoryOptimizationPlanner

    sys = BicycleDynRate(named_ports=True)
    sys.params["mu"] = mu
    sys.state.lower_bound[6], sys.state.upper_bound[6] = 0.0, 40.0
    sys.state.lower_bound[7], sys.state.upper_bound[7] = -0.5, 0.5
    sys.inputs["w_rear_dot"].lower_bound[0] = -60.0
    sys.inputs["w_rear_dot"].upper_bound[0] = 60.0
    sys.inputs["delta_dot"].lower_bound[0] = -1.5
    sys.inputs["delta_dot"].upper_bound[0] = 1.5
    r_r = float(sys.params["r_r"])
    speed = 6.0
    target = np.array([0.0, target_y, 0.0, speed, 0.0, 0.0, speed / r_r, 0.0])
    cost = QuadraticCost.from_system(
        sys,
        Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
        R=np.diag([1.0, 25.0]),
        S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
        xbar=target,
        ubar=np.zeros(2),
    )
    x_start = np.array([0.0, 1.0, 0.0, 0.9 * speed, 0.0, 0.0, 0.9 * speed / r_r, 0.0])
    problem = PlanningProblem(
        sys, x_start=x_start, cost=cost, tf=1.0, X=sys.state.box, U=sys.input_box
    )
    options = (
        {"max_iter": 300, "tol": 1e-10}
        if method == "ipopt"
        else {"maxiter": 300, "ftol": 1e-12}
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=12,
        transcription="direct_collocation",
        backend=backend,
        method=method,
        optimizer_options=options,
    )


@pytest.mark.jax
def test_script_04_program_one_compiled_mpc_program_rebinds_like_the_rebuilds():
    compiled = bicycle_planner()
    compiled.compile_parametric_program()
    x_start = compiled.problem.x_start

    plans = {}
    for mu, target_y in ((1.0, 0.0), (0.4, 0.0), (0.4, 0.6)):
        values = {
            "plant": {"mu": np.asarray(mu)},
            "cost": {"xbar": bicycle_planner(target_y=target_y).problem.cost.xbar},
        }
        plan = compiled.solve_trajectory_from(x_start, params=values)
        rebuilt = bicycle_planner(mu, target_y).solve()
        np.testing.assert_allclose(plan.trajectory.x, rebuilt.trajectory.x, atol=1e-6)
        np.testing.assert_allclose(plan.trajectory.u, rebuilt.trajectory.u, atol=1e-5)
        plans[mu, target_y] = plan.trajectory

        # The NumPy rebuild path answers the same values on the same API
        numpy_plan = bicycle_planner(backend="numpy").solve_trajectory_from(
            x_start, params=values
        )
        # (SLSQP on finite differences: the same optimum on a flat valley)
        assert numpy_plan.solver.cost == pytest.approx(plan.solver.cost, rel=1e-6)
        np.testing.assert_allclose(
            numpy_plan.trajectory.x, plan.trajectory.x, atol=1e-3
        )

    # The friction and the target change the plan, and the user's plant keeps its mu
    assert not np.allclose(plans[1.0, 0.0].x, plans[0.4, 0.0].x, atol=1e-3)
    assert not np.allclose(plans[0.4, 0.0].x, plans[0.4, 0.6].x, atol=1e-3)
    assert compiled.problem.sys.params["mu"] == 1.0

    # An unknown path raises, naming the known ones
    with pytest.raises(ValueError, match="known entries"):
        compiled.solve_trajectory_from(x_start, params={"plant": {"nu": 1.0}})


def pendulum_planner(m=1.0, *, method="scipy_slsqp"):
    """The pendulum brought back from 0.5 rad on 11 collocation knots, its mass given."""
    from minilink.planning import TrajectoryOptimizationPlanner

    plant = Pendulum()
    plant.params["m"] = m
    problem = PlanningProblem(
        plant,
        tf=2.0,
        x_start=np.array([0.5, 0.0]),
        cost=QuadraticCost.from_system(plant, R=0.1 * np.eye(1)),
        U=plant.input_box,
    )
    options = (
        {"max_iter": 500, "tol": 1e-10}
        if method == "ipopt"
        else {"maxiter": 300, "ftol": 1e-14}
    )
    return TrajectoryOptimizationPlanner(
        problem,
        n_steps=11,
        transcription="direct_collocation",
        backend="jax",
        method=method,
        optimizer_options=options,
    )


@pytest.mark.jax
def test_script_04_program_ipopt_solves_the_compiled_program():
    # The adapter's quasi-Newton (no Lagrangian Hessian, the plan's §6) needs
    # thousands of iterations on the stiff bicycle: the clause runs on the
    # pendulum's compiled program, rebound with a new mass (the run log, §4)
    pytest.importorskip("cyipopt")
    compiled = pendulum_planner(method="ipopt")
    compiled.compile_parametric_program()
    plan = compiled.solve_trajectory_from(
        compiled.problem.x_start, params={"plant": {"m": np.asarray(1.3)}}
    )
    reference = pendulum_planner(1.3).solve()
    assert plan.success
    np.testing.assert_allclose(plan.trajectory.x, reference.trajectory.x, atol=1e-4)
    assert plan.solver.cost == pytest.approx(reference.solver.cost, rel=1e-6)


# Script 6


def test_script_06_join_a_grid_world_solves_by_value_iteration_on_transition():
    from minilink.planning import DynamicProgrammingPlanner
    from tests.unittest.test_step_problems import grid_world

    problem = grid_world()
    face = problem.compile()
    vi = DynamicProgrammingPlanner(
        problem, x_grid=(7, 7), u_grid=(3, 3), tol=1e-9, max_iterations=50
    )
    vi.solve()

    # J(x) = the moves to the goal, the face's c summed: the Chebyshev distance
    J = vi.result.J.reshape(7, 7)
    cells = np.arange(7.0)
    np.testing.assert_array_equal(
        J, np.maximum.outer(np.abs(cells - 5.0), np.abs(cells - 3.0))
    )

    # The greedy walk from the start, rolled out on the face, pays J(x_start)
    law = vi.get_controller()
    walk = face.run(
        lambda x, k: np.asarray(law.outputs["u"].compute(None, x, k)), n_steps=8
    )
    assert walk[3] == J[0, 0] == 5.0


def test_script_06_join_value_iteration_the_judge_and_the_rl_environment_give_one_j():
    from minilink.learning.reinforcement_learning.environment import (
        RolloutEnvironment,
    )
    from minilink.planning import DynamicProgrammingPlanner

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant, S=10.0 * np.eye(2)),
        x_start=np.array([0.5, 0.0]),
        tf=3.0,
        U=plant.input_box,
        discount_rate=0.5,
    )
    dt = 0.1
    vi = DynamicProgrammingPlanner(
        problem, x_grid=(41, 41), u_grid=(11,), dt=dt, solver="rk4"
    )
    solution = vi.solve(evaluate=True)
    law = vi.get_controller()

    def policy(x, t):
        return np.asarray(law.outputs["u"].compute(None, x, t)).reshape(-1)

    # The step tool's sum on the face value iteration backs up: Σ γᵏ c_k + γᴺ h
    face = problem.compile(dt=dt, solver="rk4")
    J_face = face.run(policy)[3]

    # The judge, the lookup law sampled at dt (R105) on the continuous plant
    J_judge = problem.compile(law % dt, backend="numpy").evaluate().objective

    # The RL environment's discounted return of the same law
    env = RolloutEnvironment(problem, dt=dt, backend="numpy")
    x, t, J_env = problem.x_start, 0.0, 0.0
    for k in range(env.n_steps_per_episode):
        x, t, reward, terminated, _ = env.step(x, t, policy(x, t), None)
        J_env += face.gamma**k * -float(reward)
    assert bool(terminated)

    # One J, the planner's own score among them (the law held over each period, R159)
    J_planner = float(np.squeeze(solution.evaluation.J))
    assert J_judge == J_planner
    np.testing.assert_allclose([J_judge, J_env], J_face, rtol=1e-14, atol=0.0)

    # The planner's evidence is the same held loop on the face
    evidence = vi.nominal_trajectory()
    np.testing.assert_array_equal(evidence.x.T, face.run(policy)[0])


# Script 5


def native_mpc_problem():
    """The pendulum regulated from 0.5 rad over 2 s, and a native fixed-iteration planner on it (single shooting, Adam)."""
    from minilink.planning import TrajectoryOptimizationPlanner
    from minilink.planning.trajectory_optimization.shooting import (
        ShootingOptions,
        ShootingTranscription,
    )

    plant = Pendulum()
    problem = PlanningProblem(
        plant,
        tf=2.0,
        x_start=np.array([0.5, 0.0]),
        cost=QuadraticCost.from_system(plant, R=0.1 * np.eye(1)),
        U=plant.input_box,
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        transcription=ShootingTranscription(ShootingOptions(n_steps=11)),
        backend="jax",
        method="adam",
        iterations=30,
        optimizer_options={"learning_rate": 0.05},
    )
    return problem, planner


@pytest.mark.optional
@pytest.mark.jax
def test_script_05_join_the_native_mpcs_closed_loop_is_one_scan_tuned_by_gradient():
    jax = pytest.importorskip("jax")
    import jax.numpy as jnp

    from minilink.control import ModelPredictiveController

    problem, planner = native_mpc_problem()
    mpc = ModelPredictiveController(planner)
    pe = problem.compile(mpc % 0.1, backend="jax")
    params = pe.params

    def objective(p):
        return pe.evaluate(p).objective

    J = objective(params)
    assert np.isfinite(J)

    # vmap over starts: one closed loop per start, the same J as one by one
    starts = jnp.asarray([[0.5, 0.0], [0.3, 0.2]])
    batch = jax.vmap(lambda x: objective({**params, "x_start": x}))(starts)
    np.testing.assert_allclose(batch[0], J, rtol=1e-10)
    np.testing.assert_allclose(
        batch[1], objective({**params, "x_start": np.asarray(starts[1])}), rtol=1e-10
    )

    # The gradient in the controller's effort weight R, against a central difference
    def J_of_R(r):
        cost = {**params["controller"]["cost"], "R": r}
        return objective(
            {**params, "controller": {**params["controller"], "cost": cost}}
        )

    R = params["controller"]["cost"]["R"]
    g = jax.grad(J_of_R)(R)
    eps = 1e-5
    central = (J_of_R(R + eps) - J_of_R(R - eps)) / (2 * eps)
    np.testing.assert_allclose(g.sum(), central, rtol=1e-4)

    # One gradient step on R lowers the loop's J
    assert J_of_R(R - 0.05 * g / jnp.abs(g).max()) < J


@pytest.mark.optional
@pytest.mark.jax
def test_script_05_join_the_dual_rate_form_gives_the_plan_one_slow_period_late():
    from minilink.control import ModelPredictiveController, PlanFollower

    problem, planner = native_mpc_problem()
    mpc = ModelPredictiveController(planner)
    computer = (mpc % 0.05) >> (PlanFollower(planner) % 0.01)
    pe = problem.compile(computer, backend="jax")
    u = np.asarray(pe.trajectory().get_signal("plant:u"))[0]

    # The first slow period applies the guess (zero); the plan solved at t = 0 from
    # x(0) arrives at t = 0.05 and is read at the fast ticks
    np.testing.assert_array_equal(u[:5], 0.0)
    pv = mpc.pv
    z0, _ = pv.solve({"x_start": problem.x_start, "t_start": 0.0}, z0=pv.initial())
    expected = [float(pv.command(z0, None, 0.01 * k)[0]) for k in range(5, 10)]
    np.testing.assert_allclose(u[5:10], expected, rtol=1e-10)


# Script 7


def test_script_07_fields_the_certificates_v_is_a_quadratic_field():
    from minilink.analysis.lyapunov import region_of_attraction
    from minilink.core.fields import EllipsoidSet, QuadraticField

    plant = Pendulum()
    plant.params["d"] = 0.5
    roa = region_of_attraction(plant)
    X = roa.x_bar + np.random.default_rng(0).normal(scale=0.3, size=(20, 2))
    d = X - roa.x_bar

    # V = (x − x̄)ᵀ P (x − x̄), its sublevel set the certified region
    assert isinstance(roa.V, QuadraticField)
    np.testing.assert_array_equal(roa.V(X), np.einsum("ij,jk,ik->i", d, roa.P, d))
    assert isinstance(roa.region, EllipsoidSet)
    np.testing.assert_array_equal(
        roa.contains(X), [bool(roa.region.contains(x)) for x in X]
    )

    # The same field is a terminal cost: h(x) = V(x)
    h = roa.V.as_terminal_cost()
    assert h.h(X[0]) == roa.V(X[0])


@pytest.mark.optional
@pytest.mark.jax
def test_script_07_fields_the_lqr_value_ends_a_short_mpc():
    from scipy.linalg import solve_continuous_are

    from minilink import InvertedPendulum
    from minilink.analysis import linearize
    from minilink.control import ModelPredictiveController
    from minilink.core.fields import QuadraticField
    from minilink.planning import TrajectoryOptimizationPlanner

    plant = InvertedPendulum()
    Q = np.diag([10.0, 1.0])
    R = 0.1 * np.eye(1)
    x_start = np.array([0.4, 0.0])
    score = QuadraticCost.from_system(plant, Q=Q, R=R)
    problem = PlanningProblem(
        plant, cost=score, x_start=x_start, tf=4.0, U=plant.input_box
    )
    model = linearize(plant)
    V = QuadraticField(model.xbar, solve_continuous_are(model.A(), model.B(), Q, R))

    def J_of(cost):
        planner = TrajectoryOptimizationPlanner(
            PlanningProblem(
                plant, cost=cost, x_start=x_start, tf=0.3, U=plant.input_box
            ),
            n_steps=3,
            transcription="single_shooting",
            backend="jax",
            method="adam",
            iterations=40,
            optimizer_options={"learning_rate": 0.2},
        )
        pe = problem.compile(ModelPredictiveController(planner) % 0.1, backend="jax")
        return float(pe.evaluate(pe.params).objective)

    # Three periods ahead the pendulum falls; ended by the LQR value it is held up
    J_short = J_of(score)
    J_ended = J_of(score + V.as_terminal_cost())
    assert J_ended < 0.1 * J_short
