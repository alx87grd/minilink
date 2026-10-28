"""Numbers of every branch the v0.3 core run touches, for its before/after checks.

    python benchmarks/v03_baseline.py OUT.json                         # capture
    python benchmarks/v03_baseline.py OUT.json --compare REF.json [--rtol R]

The step model's entries (``step_model_baseline.py``) plus the sampled loop (the hybrid
demos, the step diagrams, the MPC), the problem evaluator (deterministic, stochastic,
its program and its gradient), the draws, the noise blocks, the optimizer's methods,
the RL environment and training, and the fields. Repeated calls are captured twice.
No timing is recorded, so two captures of the same code are byte-identical; the
committed reference is ``benchmarks/baselines/v03_core.json``.
"""

from __future__ import annotations

import json
import sys
import warnings

import numpy as np

from minilink import (
    CostFunction,
    Integrator,
    P,
    Pendulum,
    PlanningProblem,
    QuadraticCost,
    StepDiagramSystem,
    StepSystem,
)
from minilink.core.backends import array_module, configure_jax

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from step_model_baseline import ENTRIES as STEP_MODEL_ENTRIES  # noqa: E402
from step_model_baseline import as_list  # noqa: E402
from step_model_baseline import compare as compare_flat  # noqa: E402

configure_jax(enable_x64=True)
warnings.simplefilter("ignore")

DT = 0.05
X0 = np.array([0.4, -0.2])


class DiscreteLowPass(StepSystem):
    """``x_{k+1} = α x_k + (1 − α) u_k``, ``y = x``."""

    def __init__(self, alpha=0.25):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.params = {"alpha": float(alpha)}
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, u)
        alpha = params["alpha"]
        return xp.array([alpha * x[0] + (1.0 - alpha) * u[0]])

    def h(self, x, u, k=0, params=None):
        return x


class Accumulator(StepSystem):
    """``x_{k+1} = x_k + u_k``, ``y = x``."""

    def __init__(self):
        super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
        self.x0 = np.zeros(1)

    def step(self, x, u, k=0, params=None):
        xp = array_module(x, u)
        return xp.array([x[0] + u[0]])

    def h(self, x, u, k=0, params=None):
        return x


def pendulum_problem(tf=2.0, x_start=X0, **kwargs):
    plant = Pendulum()
    cost = QuadraticCost.from_system(plant)
    return PlanningProblem(plant, cost=cost, x_start=x_start, tf=tf, **kwargs)


def pd_controller():
    from minilink.control import StateFeedback

    return StateFeedback(K=[[8.0, 2.0]])


# Entries: the sampled loop


def step_diagram_entries(out):
    """A pure step diagram's rollout, twice (repeated calls)."""
    diagram = StepDiagramSystem()
    diagram.add_subsystem(P(0.3, ports="reference"), "controller")
    diagram.add_subsystem(Accumulator(), "plant")
    diagram.add_input_port("r")
    diagram.connect("input", "r", "controller", "r")
    diagram.connect("plant", "y", "controller", "y")
    diagram.connect("controller", "u", "plant", "u")
    diagram.connect_new_output_port("plant", "y", "y")
    for i in range(2):
        rollout = diagram.compute_rollout(n_steps=40, u=np.array([1.0]))
        out[f"step_diagram.unity.x.{i}"] = rollout.x
    for backend in ("numpy", "jax"):
        ev = diagram.compile(backend=backend)
        xs = ev.rollout(diagram.x0, np.ones((40, 1)), n_steps=40)
        out[f"step_diagram.unity.ev.{backend}"] = np.asarray(xs).T


def hybrid_entries(out):
    """The hybrid demos' loops: multi-rate filter and controller, sampled SMC, sampled PD."""
    from minilink import SlidingMode
    from minilink.core.discrete.hybrid_composition import hybrid_closed_loop
    from minilink.simulation import StepSchedule

    # The multi-rate computer on an integrator (hybrid_multi_rate)
    computer = StepDiagramSystem()
    computer.add_subsystem(DiscreteLowPass(alpha=0.6), "filter")
    computer.add_subsystem(P(10.0, ports="reference"), "controller")
    computer.add_input_port("r")
    computer.add_input_port("y_meas")
    computer.connect("input", "r", "controller", "r")
    computer.connect("input", "y_meas", "filter", "u")
    computer.connect("filter", "y", "controller", "y")
    computer.connect_new_output_port("filter", "y", "y_f")
    computer.connect_new_output_port("controller", "u", "u_cmd")
    schedule = StepSchedule.from_rates(
        dt_base=0.01, rates_hz={"filter": 100.0, "controller": 10.0}
    )
    hybrid = hybrid_closed_loop(
        computer,
        Integrator(),
        schedule=schedule,
        computer_out="u_cmd",
        computer_in="y_meas",
    )
    for i in range(2):
        result = hybrid.compute_forced(
            lambda t: np.array([1.0 if t < 1.0 else 0.0]),
            t_start=0,
            tf=2.0,
            input_port_id="r",
            plant_dt_inner=0.002,
        )
        out[f"hybrid.multi_rate.plant_x.{i}"] = result.plant.x
        out[f"hybrid.multi_rate.computer.{i}"] = dict(result.computer.signals)

    # The sampled sliding-mode controller on the pendulum (sampled_smc_pendulum)
    plant = Pendulum(length=1.0, mass=1.0)
    plant.x0 = np.array([np.pi + 0.25, 0.0])
    ctl = SlidingMode(plant, lam=2.0, gain=8.0, nab=0.15)
    hybrid = hybrid_closed_loop(
        ctl, plant, schedule=0.05, computer_in="y", plant_out="y"
    )
    result = hybrid.compute_forced(
        np.array([0.0, 0.0]),
        input_port_id="r",
        t_start=0.0,
        tf=1.0,
        plant_dt_inner=0.001,
    )
    out["hybrid.smc.plant_x"] = result.plant.x
    out["hybrid.smc.computer"] = dict(result.computer.signals)

    # A sampled state feedback, trajectory and rollout verbs
    plant = Pendulum()
    plant.x0 = X0.copy()
    hybrid = hybrid_closed_loop(
        pd_controller(), plant, schedule=0.05, computer_in="x", plant_out="x"
    )
    traj = hybrid.compute_trajectory(tf=1.0, plant_dt_inner=0.01)
    out["hybrid.pd.trajectory_x"] = traj.plant.x


def mpc_entries(out):
    """The NumPy MPC on a single integrator (mpc_integrator_numpy), shortened."""
    from minilink import (
        DynamicSystem,
        SingletonSet,
        TrajectoryOptimizationPlanner,
    )
    from minilink.control import ModelPredictiveController

    class SingleIntegrator(DynamicSystem):
        def __init__(self):
            super().__init__(n=1, input_dim=1, output_dim=1, y_dependencies=())
            self.state.lower_bound = np.array([-10.0])
            self.state.upper_bound = np.array([10.0])
            self.inputs["u"].lower_bound = np.array([-10.0])
            self.inputs["u"].upper_bound = np.array([10.0])

        def f(self, x, u, t=0, params=None):
            return np.array([u[0]])

        def h(self, x, u, t=0, params=None):
            return x

    plant = SingleIntegrator()
    x0 = np.array([0.5])
    x_goal = np.array([2.0])
    plant.x0 = x0.copy()
    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=plant,
            tf=1.0,
            x_start=x0,
            Xf=SingletonSet(x_goal),
            cost=QuadraticCost.from_system(
                plant, Q=np.eye(1), R=0.1 * np.eye(1), xbar=x_goal
            ),
            X=plant.state.box,
            U=plant.input_box,
        ),
        n_steps=8,
        transcription="direct_collocation",
        backend="numpy",
        optimizer_options={"maxiter": 80, "ftol": 1e-6},
        verbose=False,
    )
    mpc = ModelPredictiveController(planner, dt_mpc=0.2, warm_start=True, verbose=False)
    hybrid = mpc @ plant
    traj = hybrid.compute_trajectory(
        tf=1.0, x0_plant=x0, plant_dt_inner=0.01, backend="numpy"
    )
    out["mpc.integrator.x"] = traj.plant.x


# Entries: the problem evaluator and its program


def evaluator_entries(out):
    """``problem.compile(controller)``: trajectory, evaluate, gradient, program, repeated."""
    problem = pendulum_problem(U=Pendulum().input_box)
    for backend in ("numpy", "jax"):
        pe = problem.compile(pd_controller(), dt=DT, backend=backend)
        key = f"pe.pd.{backend}"
        out[f"{key}.trajectory_x"] = pe.trajectory().x
        for i in range(2):
            report = pe.evaluate()
            out[f"{key}.J.{i}"] = report.J
        params = pe.params
        params["controller"]["K"] = np.array([[6.0, 1.5]])
        out[f"{key}.J_params"] = pe.evaluate(params).J
        program = pe.program(wrt=["controller.K"])
        z = pe.pack(pe.params, ["controller.K"])
        out[f"{key}.program_J"] = program.J(z)
        if backend == "jax":
            import jax

            grad = jax.grad(lambda p: pe.evaluate(p).objective)(pe.params)
            out[f"{key}.grad_K"] = grad["controller"]["K"]
            out[f"{key}.grad_x_start"] = grad["x_start"]


def optimizer_entries(out):
    """The optimizer's methods on a textbook program and on the evaluator's program."""
    from minilink import MathematicalProgram, Optimizer

    def rosenbrock(z):
        return (1.0 - z[0]) ** 2 + 100.0 * (z[1] - z[0] ** 2) ** 2

    program = MathematicalProgram(n_z=2, J=rosenbrock)
    for method in ("gd", "adam", "scipy_slsqp", "l-bfgs"):
        options = {"maxiter": 50}
        if method in ("gd", "adam"):
            options["learning_rate"] = 1e-3
        result = Optimizer(
            program,
            z0=np.array([-1.0, 1.0]),
            method=method,
            backend="jax",
            options=options,
        ).solve()
        out[f"optimizer.rosenbrock.{method}"] = np.asarray(result.z)

    problem = pendulum_problem(tf=1.0)
    pe = problem.compile(pd_controller(), dt=DT, backend="jax")
    wrt = ["controller.K"]
    result = Optimizer(
        pe.program(wrt=wrt),
        method="gd",
        options={"maxiter": 10, "learning_rate": 1e-3},
    ).solve()
    out["optimizer.pe.gd"] = np.asarray(result.z)


# Entries: the draws


def draw_entries(out):
    """The distributions, a stochastic problem's realization and its Monte Carlo score."""
    from minilink import Gaussian, StochasticPlanningProblem, Uniform

    for backend in ("numpy", "jax"):
        key = 0
        if backend == "jax":
            import jax

            key = jax.random.PRNGKey(0)
        out[f"draws.uniform.{backend}"] = Uniform([-1.0, -2.0], [1.0, 2.0]).sample(
            key, n=4
        )
        out[f"draws.gaussian.{backend}"] = Gaussian([0.0, 1.0], [1.0, 0.5]).sample(
            key, n=4
        )

    plant = Pendulum()
    problem = StochasticPlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        tf=1.0,
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
        params_distribution={"m": Uniform([0.8], [1.2])},
        U=plant.input_box,
    )
    x0s, draws = problem.realize(0, 3)
    out["draws.realize.x0s"] = x0s
    out["draws.realize.m"] = draws["params"]["m"]
    for backend in ("numpy", "jax"):
        pe = problem.compile(pd_controller(), dt=DT, backend=backend)
        for i in range(2):
            out[f"draws.pe.{backend}.J.{i}"] = pe.evaluate(n_trials=4, key=1).J


def noise_entries(out):
    """A white-noise source on a plant's port, run as a continuous diagram."""
    from minilink import DiagramSystem, WhiteNoise

    plant = Pendulum()
    plant.x0 = X0.copy()
    noise = WhiteNoise(1, psd=0.1, sample_period=0.05, seed=3)
    diagram = DiagramSystem()
    diagram.add_subsystem(noise, "noise")
    diagram.add_subsystem(plant, "plant")
    diagram.connect("noise", "y", "plant", "u")
    traj = diagram.compute_trajectory(
        tf=1.0, dt=0.01, solver="rk4_fixedsteps", verbose=False
    )
    out["noise.white.x"] = traj.x
    out["noise.white.h"] = [noise.h(None, None, t) for t in (0.0, 0.07, 0.5)]


# Entries: learning


def rl_entries(out):
    """A short seeded PPO training and the learned law at a few states."""
    import jax.numpy as jnp

    from minilink import (
        ReinforcementLearningPlanner,
        StochasticPlanningProblem,
        Uniform,
    )

    plant = Pendulum()
    plant.inputs["u"].lower_bound = np.array([-4.0])
    plant.inputs["u"].upper_bound = np.array([4.0])
    plant.state.lower_bound = np.array([-4 * np.pi, -20.0])
    plant.state.upper_bound = np.array([4 * np.pi, 20.0])

    class SwingUpCost(CostFunction):
        def g(self, x, u, t=0.0, params=None):
            return (1.0 + jnp.cos(x[0])) + 0.01 * x[1] ** 2 + 0.01 * u[0] ** 2

        def h(self, x, t=0.0, params=None):
            return 0.0

    problem = StochasticPlanningProblem(
        plant,
        cost=SwingUpCost(),
        tf=np.inf,
        x0_distribution=Uniform([-np.pi, -1.0], [np.pi, 1.0]),
        U=plant.input_box,
    )
    planner = ReinforcementLearningPlanner(
        problem,
        dt=DT,
        hidden=(8,),
        algorithm="ppo",
        n_envs=4,
        n_steps=16,
        batch_size=32,
        gamma=0.97,
        verbose=False,
    )
    planner.solve(timesteps=256, verbose=False)
    law = planner.get_controller()
    states = np.array([[0.0, 0.0], [1.0, -0.5], [3.0, 1.0]])
    out["rl.ppo.u"] = [law.ctl(None, x) for x in states]
    out["rl.ppo.history"] = [
        {k: v for k, v in row.items() if k not in ("elapsed", "fps")}
        for row in planner.history
    ]


def gym_problem_entries(out):
    """``Sys2Gym.from_problem``: reset and a few steps."""
    from minilink import StochasticPlanningProblem, Uniform
    from minilink.interfaces import Sys2Gym

    plant = Pendulum()
    problem = StochasticPlanningProblem(
        plant,
        cost=QuadraticCost.from_system(plant),
        tf=1.0,
        x0_distribution=Uniform([-0.5, -0.5], [0.5, 0.5]),
        U=plant.input_box,
    )
    env = Sys2Gym.from_problem(problem, dt=DT)
    y, _ = env.reset(seed=2)
    rows = [np.asarray(y)]
    for k in range(5):
        y, r, terminated, truncated, info = env.step(np.array([0.1 * k]))
        rows.append(np.concatenate([np.asarray(y), [r]]))
    out["gym.from_problem"] = rows


# Entries: fields


def field_entries(out):
    """A state field's set and cost."""
    from minilink.planning.spatial.state_fields import StateField

    class Energy(StateField):
        def value(self, x, u=None, t=0.0, params=None):
            return 0.5 * x[1] ** 2 - np.cos(x[0])

    field = Energy()
    states = [np.array([0.1, 0.2]), np.array([2.0, -1.0])]
    constraint = field.as_constraint(lower=-0.5, upper=0.5)
    cost = field.as_cost(weight=2.0, shaping=lambda v: v**2)
    out["field.margin"] = [constraint.margin(x) for x in states]
    out["field.cost"] = [cost.g(x, np.zeros(1), 0.0) for x in states]


ENTRIES = (*STEP_MODEL_ENTRIES,) + (
    step_diagram_entries,
    hybrid_entries,
    mpc_entries,
    evaluator_entries,
    optimizer_entries,
    draw_entries,
    noise_entries,
    rl_entries,
    gym_problem_entries,
    field_entries,
)


def flattened(data):
    """Every nested entry as its own key (``key/name``, ``key/0``): arrays and strings only."""
    flat = {}
    for key, value in data.items():
        if isinstance(value, dict):
            flat.update(flattened({f"{key}/{k}": v for k, v in value.items()}))
        elif isinstance(value, list) and not is_array(value):
            flat.update(flattened({f"{key}/{i}": v for i, v in enumerate(value)}))
        else:
            flat[key] = value
    return flat


def is_array(value):
    """Whether a JSON list is one rectangular array of numbers."""
    try:
        np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return False
    return True


def compare(current, reference, rtol):
    """Every key that is missing, new, an error, or differs beyond ``rtol``."""
    return compare_flat(flattened(current), flattened(reference), rtol)


def capture():
    out = {}
    for entry in ENTRIES:
        try:
            entry(out)
        except Exception as error:  # noqa: BLE001  (an entry that breaks is a result)
            out[f"{entry.__name__}.error"] = f"{type(error).__name__}: {error}"
    return {key: as_list(value) for key, value in out.items()}


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1].startswith("--"):
        raise SystemExit(
            "usage: v03_baseline.py OUT.json [--compare REF.json [--rtol R]]"
        )
    path = sys.argv[1]
    data = capture()
    with open(path, "w", encoding="utf-8") as file:
        json.dump(data, file, indent=1, sort_keys=True)
    if "--compare" in sys.argv:
        reference_path = sys.argv[sys.argv.index("--compare") + 1]
        rtol = (
            float(sys.argv[sys.argv.index("--rtol") + 1])
            if "--rtol" in sys.argv
            else 0.0
        )
        with open(reference_path, encoding="utf-8") as file:
            reference = json.load(file)
        report = compare(data, reference, rtol)
        print(
            "\n".join(report)
            if report
            else f"all {len(data)} entries agree (rtol {rtol:g})"
        )
        sys.exit(1 if report else 0)
    print(f"{len(data)} entries; errors: {[k for k in data if k.endswith('.error')]}")
