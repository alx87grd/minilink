"""Build every README figure: the library map, the showcase GIFs and their 3-D pages, the diagram PNG, the tool montage."""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
STATIC = ROOT / "docs" / "_static"
SHOWCASE = STATIC / "showcase"  # interactive 3-D pages, served by the docs site

import numpy as np  # noqa: E402
from asset_export import html_to_gif, montage_png, shrink_gif, svg_to_png  # noqa: E402

from minilink.core.sets import SingletonSet  # noqa: E402


def save_gif(
    sys_or_diagram, traj, name: str, *, speed: float = 2.0, camera=None
) -> None:
    out = STATIC / name
    sys_or_diagram.animate(
        traj, renderer="matplotlib", show=False, speed=speed, camera=camera
    ).save(out)
    shrink_gif(out)


def oblique_camera(target, *, azimuth_deg, elevation_deg, distance):
    """A constant 3-D view: the eye ``distance`` from ``target``, at the given bearing."""
    azimuth, elevation = np.radians(azimuth_deg), np.radians(elevation_deg)
    camera = np.eye(4)
    camera[:3, 2] = [
        np.cos(elevation) * np.cos(azimuth),
        np.cos(elevation) * np.sin(azimuth),
        np.sin(elevation),
    ]
    camera[:3, 3] = target
    camera[3, 3] = distance
    return camera


def save_meshcat(sys_or_diagram, traj, page: str, gif: str, **animate_kwargs) -> None:
    """Write the interactive 3-D page, then the GIF screenshot of that page."""
    SHOWCASE.mkdir(parents=True, exist_ok=True)
    html = SHOWCASE / page
    sys_or_diagram.animate(traj, renderer="meshcat", show=False, **animate_kwargs).save(
        html
    )
    print(f"{html}: {html.stat().st_size / 1e6:.2f} MB")
    html_to_gif(html, STATIC / gif)


def diagram_png() -> None:
    """The README's first loop: a PID closed on the catalog's mass on a spring."""
    from minilink import PID, SingleMass
    from minilink.graphical.diagrams.dot import get_diagram

    diagram = PID(Kp=10.0, Ki=1.0, Kd=2.0) @ SingleMass()
    get_diagram(diagram).render(
        filename=str(STATIC / "diagram_closed_loop"), format="png", cleanup=True
    )
    print("diagram_closed_loop.png")


# The README figures' style: one blue series on a light surface, recessive gray axes
BLUE, GHOST, SURFACE, INK, MUTED, GRID = (
    "#2a78d6",
    "#9ec5f4",
    "#fcfcfb",
    "#0b0b0b",
    "#52514e",
    "#e6e5e1",
)


def figure_style(axes) -> None:
    """The plain style of the README's plots: no top or right spine, a light grid behind."""
    for ax in axes:
        ax.set_facecolor(SURFACE)
        ax.grid(color=GRID, lw=0.6)
        ax.set_axisbelow(True)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)


def style_rc() -> dict:
    """Matplotlib settings for the README's plots: small gray ticks and labels, left-aligned bold titles."""
    return {
        "font.size": 9,
        "axes.edgecolor": MUTED,
        "axes.labelcolor": MUTED,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
        "axes.titlecolor": INK,
        "axes.titlesize": 10,
        "axes.titleweight": "bold",
        "axes.titlelocation": "left",
        "axes.titlepad": 8,
    }


def swing_up_iterates():
    """The README's swing-up solved by trajectory optimization: the initial guess and every iterate."""
    from minilink import TrajectoryOptimizationPlanner
    from minilink.planning.initial_guess import default_initial_trajectory

    _, problem = readme_problem()
    solution = TrajectoryOptimizationPlanner(problem, n_steps=60).solve(
        record_history=True
    )
    history = solution.record.history
    guess = default_initial_trajectory(problem, history[0].trajectory.t)

    return guess, history


def breadth_strip() -> None:
    """The README's one equation, many tools: the frequency response, trajectory optimization, PPO."""
    import matplotlib.pyplot as plt

    from minilink import (
        Pendulum,
        PlanningProblem,
        QuadraticCost,
        ReinforcementLearningPlanner,
        Uniform,
    )

    plant = Pendulum()
    up = np.array([np.pi, 0.0])

    # G(jω) = C (jωI − A)⁻¹ B about the hanging rest, the angle per torque
    A = plant.jacobian("f", "x", np.zeros(2))
    B = plant.jacobian("f", "u", np.zeros(2)).reshape(2, 1)
    C = np.array([[1.0, 0.0]])
    w = np.logspace(-1, 2, 400)
    G = np.array([(C @ np.linalg.solve(1j * wk * np.eye(2) - A, B))[0, 0] for wk in w])

    guess, history = swing_up_iterates()

    start = Uniform([np.pi - 0.5, -1.0], [np.pi + 0.5, 1.0])
    cost = QuadraticCost.from_system(plant, x_bar=up)
    balance = PlanningProblem(
        plant, x0=start, cost=cost, horizon=5.0, U=plant.input_box
    )
    learner = ReinforcementLearningPlanner(balance, dt=0.05)
    learner.solve(n_timesteps=100_000)
    steps = np.array([h["n_timesteps"] for h in learner.history]) / 1e3
    returns = np.array([h["ep_return_mean"] for h in learner.history])

    plt.rcParams.update(style_rc())
    fig, axes = plt.subplots(1, 3, figsize=(12.0, 3.0), facecolor=SURFACE)
    figure_style(axes)

    ax = axes[0]
    ax.semilogx(w, 20 * np.log10(np.abs(G)), color=BLUE, lw=2)
    ax.set(title="Frequency response", xlabel="ω [rad/s]", ylabel="|θ / τ| [dB]")

    ax = axes[1]
    ax.axhline(np.pi, color=MUTED, lw=0.8, ls=(0, (3, 3)))
    ax.text(0.05, np.pi + 0.18, "upright", color=MUTED, fontsize=8)
    ax.plot(
        guess.t, guess.x[0], color=MUTED, lw=1.0, ls=(0, (4, 3)), label="initial guess"
    )
    for k, it in enumerate(history[:-1]):
        label = "iterates" if k == 0 else None
        ax.plot(
            it.trajectory.t,
            it.trajectory.x[0],
            color=GHOST,
            lw=0.8,
            alpha=0.6,
            label=label,
        )
    final = history[-1].trajectory
    ax.plot(final.t, final.x[0], color=BLUE, lw=2.2, label="solution")
    ax.set(title="Trajectory optimization", xlabel="t [s]", ylabel="θ [rad]")
    ax.set(xlim=(0.0, 6.0), ylim=(-2.0, 4.2))
    ax.legend(loc="lower right", frameon=False, fontsize=8, labelcolor=MUTED)

    ax = axes[2]
    ax.plot(steps, returns, color=BLUE, lw=2)
    ax.set(title="Reinforcement learning (PPO)", xlabel="training steps [×1000]")
    ax.set(ylabel="mean episode return")

    fig.tight_layout(w_pad=2.5)
    fig.savefig(STATIC / "tool_breadth.png", dpi=130, facecolor=SURFACE)
    plt.close(fig)
    print("tool_breadth.png")


def trajopt_iterations() -> None:
    """Trajectory optimization iterate by iterate: the guess, then each iterate drawn over the faded ones."""
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation, PillowWriter

    guess, history = swing_up_iterates()
    frames = [(guess, "initial guess")] + [
        (
            it.trajectory,
            f"iteration {it.iteration + 1}    J = {it.J:,.0f}    "
            f"defect = {it.max_equality_violation:.1e}",
        )
        for it in history
    ]
    n = len(frames)
    # The first iterates move the most: they stay three frames each, the guess six, the end fourteen
    shown = [0] * 6 + [k for k in range(1, 8) for _ in range(3)]
    shown += [k for k in range(8, n) if k % 3 == 0 or k == n - 1] + [n - 1] * 14

    plt.rcParams.update(style_rc())
    fig, (ax_q, ax_u) = plt.subplots(
        2,
        1,
        figsize=(6.4, 3.8),
        sharex=True,
        facecolor=SURFACE,
        gridspec_kw={"height_ratios": [3, 2]},
    )
    figure_style((ax_q, ax_u))
    ax_q.axhline(np.pi, color=MUTED, lw=0.8, ls=(0, (3, 3)))
    ax_q.text(0.05, np.pi + 0.15, "upright", color=MUTED, fontsize=8)
    ax_q.set(ylabel="θ [rad]", xlim=(0.0, 6.0), ylim=(-2.0, 4.2))
    ax_u.set(ylabel="τ [Nm]", xlabel="t [s]", ylim=(-5.8, 5.8))
    for bound in (-5.0, 5.0):
        ax_u.axhline(bound, color=MUTED, lw=0.8, ls=(0, (1, 2)))
    line_q = ax_q.plot([], [], color=BLUE, lw=2.2)[0]
    line_u = ax_u.plot([], [], color=BLUE, lw=2.0)[0]
    label = ax_q.text(
        0.99, 0.04, "", transform=ax_q.transAxes, ha="right", va="bottom", color=INK
    )
    fig.suptitle(
        "Trajectory optimization, iterate by iterate",
        x=0.02,
        ha="left",
        color=INK,
        fontsize=10,
        fontweight="bold",
    )
    fig.tight_layout()
    faded = []

    def draw(frame):
        k = shown[frame]
        while len(faded) < k:
            previous = frames[len(faded)][0]
            ax_q.plot(previous.t, previous.x[0], color=GHOST, lw=0.8, alpha=0.5)
            ax_u.plot(previous.t, previous.u[0], color=GHOST, lw=0.8, alpha=0.5)
            faded.append(previous)
        trajectory, text = frames[k]
        line_q.set_data(trajectory.t, trajectory.x[0])
        line_u.set_data(trajectory.t, trajectory.u[0])
        label.set_text(text)

    out = STATIC / "trajopt_iterations.gif"
    animation = FuncAnimation(fig, draw, frames=len(shown))
    animation.save(
        out, writer=PillowWriter(fps=6), dpi=100, savefig_kwargs={"facecolor": SURFACE}
    )
    plt.close(fig)
    shrink_gif(out)
    print("trajopt_iterations.gif")


def readme_problem():
    """The README's swing-up problem: from hanging to the upright ball, with a terminal weight on where it ends."""
    from minilink import BallSet, Pendulum, PlanningProblem, QuadraticCost

    plant = Pendulum()
    up = np.array([np.pi, 0.0])
    cost = QuadraticCost.from_system(plant, x_bar=up, S=100 * np.eye(2))
    problem = PlanningProblem(
        plant,
        x0=[0.0, 0.0],
        Xf=BallSet(up, 0.2),
        horizon=6.0,
        cost=cost,
        X=plant.state.box,
        U=plant.input_box,
    )

    return plant, problem


def readme_swing_up():
    """The README's swing-up problem and its four answers, as its planner section builds them."""
    from minilink import (
        DynamicProgrammingPlanner,
        RRTPlanner,
        TrajectoryOptimizationPlanner,
    )
    from minilink.control import ModelPredictiveController

    plant, problem = readme_problem()
    vi_planner = DynamicProgrammingPlanner(
        problem, x_grid=(101, 101), u_grid=(11,), dt=0.05
    )
    look_ahead = problem.replace(horizon=2.0, Xf=None)
    mpc_planner = TrajectoryOptimizationPlanner(look_ahead, n_steps=20, backend="jax")
    answers = {
        "VI": vi_planner.solve(),
        "RRT": RRTPlanner(problem, seed=0).solve(),
        "TrajOpt": TrajectoryOptimizationPlanner(problem, n_steps=60).solve(),
        "MPC": ModelPredictiveController(mpc_planner) % 0.1,
    }

    return plant, problem, answers


def comparison_runs() -> None:
    """The README's comparison over the uncertain mass: every controller's twenty runs."""
    from minilink import Uniform, compare

    _, problem, answers = readme_swing_up()
    mass = Uniform(0.8, 1.2)
    uncertain = problem.replace(distributions={"plant": {"m": mass}})
    comparison = compare(uncertain, n_trials=20, **answers)
    comparison.plot_simulated_trajectory(show=False).save(
        STATIC / "comparison_runs.png"
    )
    print("comparison_runs.png")


def tool_montage() -> None:
    """One plant, six tools: the README's "One toolbox" figure, each panel a library verb."""
    import matplotlib.pyplot as plt

    from minilink import (
        BallSet,
        DynamicProgrammingPlanner,
        Pendulum,
        PlanningProblem,
        QuadraticCost,
        RRTPlanner,
        TrajectoryOptimizationPlanner,
        lqr,
    )

    plant = Pendulum()
    x_up = np.array([np.pi, 0.0])
    problem = PlanningProblem(
        sys=plant,
        x0=np.zeros(2),
        Xf=BallSet(x_up, 0.2),
        horizon=4.0,
        cost=QuadraticCost.from_system(plant, Q=np.eye(2), R=np.eye(1), x_bar=x_up),
        X=plant.state.box,
        U=plant.input_box,
    )

    # Each verb draws its own figure; the montage pastes the six side by side
    def panel(title, draw):
        plt.close("all")
        draw()
        figure = plt.gcf()
        figure.suptitle(title, fontsize=14, fontweight="bold")
        figure.set_size_inches(5.2, 4.0)
        figure.tight_layout()
        return figure

    swing = Pendulum()
    swing.x0[0] = 2.0
    swing.compute_trajectory(tf=10.0)
    loop = lqr(plant, np.eye(2), np.eye(1), x_bar=x_up) @ plant
    vi = DynamicProgrammingPlanner(
        problem.replace(horizon=None), x_grid=(101, 101), u_grid=(11,), dt=0.05
    )
    vi.solve()
    rrt = RRTPlanner(problem, seed=0)
    rrt.solve()
    opt = TrajectoryOptimizationPlanner(
        problem, n_steps=40, transcription="direct_collocation"
    )
    solution = opt.solve()

    figures = [
        panel(
            "plot_phase_plane", lambda: swing.plot_phase_plane(swing.traj, show=False)
        ),
        panel("plot_bode", lambda: plant.plot_bode(show=False)),
        panel(
            "lqr @ plant: plot_step_response",
            lambda: loop.plot_step_response(x_up, show=False),
        ),
        panel(
            "DynamicProgrammingPlanner: plot_cost_to_go",
            lambda: vi.plot_cost_to_go(jmax=60.0, show=False),
        ),
        panel("RRTPlanner: plot_tree", lambda: rrt.plot_tree(show=False)),
        panel(
            "TrajectoryOptimizationPlanner: plot_trajectory",
            lambda: solution.plot_trajectory(show=False),
        ),
    ]
    montage_png(figures, STATIC / "tool_montage.png", columns=3)
    plt.close("all")
    print("tool_montage.png")


def gif_cartpole() -> None:
    from minilink import (
        CartPole,
        PlanningProblem,
        QuadraticCost,
        TrajectoryOptimizationPlanner,
    )
    from minilink.graphical.catalog import camera_matrix

    plant = CartPole()
    plant.inputs["u"].lower[0] = -10.0
    plant.inputs["u"].upper[0] = 10.0
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    problem = PlanningProblem(
        sys=plant,
        horizon=4.0,
        x0=np.array([-2.0, 1.0, 0.0, 0.0]),
        Xf=SingletonSet(x_goal),
        cost=QuadraticCost.from_system(
            plant, Q=np.diag([1.0, 1.0, 0.0, 0.0]), R=np.diag([0.01]), x_bar=x_goal
        ),
        U=plant.input_box,
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=40,
        transcription="direct_collocation",
        backend="jax",
        method="slsqp",
    )
    traj = planner.solve().trajectory.resample(n_samples=240)
    camera = camera_matrix(
        target=(-1.0, 0.75, 0.0), scale=3.6
    )  # the cart's run and the pole
    save_gif(plant, traj, "cartpole_swingup.gif", speed=1.0, camera=camera)


def ur5_impedance() -> None:
    from minilink.blocks.sources import TrajectorySource
    from minilink.control.robotic import TaskImpedance
    from minilink.dynamics.catalog.manipulators import UR5Manipulator

    # The hand's target glides through four waypoints and back, holding at each.
    HOLD, GLIDE = 0.5, 1.5  # [s]
    waypoints = np.array(
        [
            [-0.5, 0.0, 0.5],
            [0.0, 0.5, 0.5],
            [-0.3, 0.3, 0.2],
            [-0.5, -0.3, 0.4],
            [-0.5, 0.0, 0.5],
        ]
    )
    t_knots, p_knots = [0.0], [waypoints[0]]
    for p_from, p_to in zip(waypoints[:-1], waypoints[1:]):
        t_knots += [t_knots[-1] + HOLD, t_knots[-1] + HOLD + GLIDE]
        p_knots += [p_from, p_to]
    TF = t_knots[-1] + HOLD

    arm = UR5Manipulator()
    q0 = arm.inverse_kinematics(
        waypoints[0], q_guess=np.array([0.0, -1.0, 1.2, -1.4, 0.0, 0.0])
    )
    arm.x0 = arm.q2x(q0, np.zeros(6))

    ref = TrajectorySource(t_knots, np.array(p_knots).T)
    ctl = TaskImpedance(arm, gravity_comp=True, show_task_force=True)
    # a soft spring: the hand trails its target, so the ball and the force show
    ctl.params["Kp"] = np.array([100.0, 100.0, 100.0])
    ctl.params["Kd"] = np.array([20.0, 20.0, 20.0])
    ctl.task_force_scale = 0.08  # [m/N]

    diagram = ref >> ctl @ arm
    traj = diagram.compute_trajectory(tf=TF, n_samples=int(30 * TF) + 1)
    # the eye on the hand's side of the base: the target ball and the force in view
    camera = oblique_camera(
        [-0.33, 0.08, 0.4], azimuth_deg=160.0, elevation_deg=18.0, distance=0.95
    )
    save_meshcat(
        diagram,
        traj,
        "ur5_impedance.html",
        "ur5_meshcat.gif",
        speed=1.0,
        camera=camera,
    )


def racecar_mpc() -> None:
    from minilink import PlanningProblem, QuadraticCost, TrajectoryOptimizationPlanner
    from minilink.catalog import UdeSRacecarDyn
    from minilink.control import ModelPredictiveController
    from minilink.graphical.catalog.racecar_skin import racecar_skin_3d
    from minilink.planning import (
        ReferenceTrack,
        Scene,
        Sphere,
        car_outline,
        circuit_waypoints,
        from_waypoints,
        mount,
        point_probe,
        quadratic_excess,
        quadratic_hinge,
    )
    from minilink.planning.trajectory_optimization.mpc import (
        mpc_animation_overlays,
        plant_run,
    )

    # examples/demos/udes_racecar/mpc_racecar_dynamic.py: a lap past two cones,
    # the plan believing in a little more grip than the floor gives. At 4.5 m/s
    # the car keeps clear of both cones and inside the lane the whole lap.
    LENGTH, WIDTH, RADIUS = 12.0, 8.0, 2.0  # [m]
    V_TARGET, P_CRUISE = 4.5, 10.0  # [m/s], [W]
    TF_SIM, SIM_DT = 10.3, 0.002  # [s] one lap, back to the start
    PLAYBACK = 2.0  # the page and the GIF play the lap at twice real time
    MPC_DT, MPC_HORIZON, MPC_STEPS = 0.1, 1.2, 12
    BODY_LENGTH, BODY_WIDTH, BODY_MARGIN = 0.34, 0.20, 0.02  # [m]
    CORRIDOR_HALF_WIDTH = 0.6  # [m]
    CONE_RADIUS, CONE_MARGIN, CONE_CLEARANCE = 0.10, 0.15, 0.30  # [m]
    CONE_X, CONE_LEAN = (2.0, -2.0), (-0.13, 0.13)  # [m]
    PLANNER_MU, PLANT_MU = 0.7, 0.6
    LATERAL_START, HEADING_NUDGE = 0.15, 1.0e-4  # [m], [rad]
    CAMERA_SCALE = 2.1  # [m] follow-camera distance

    path = circuit_waypoints(length=LENGTH, width=WIDTH, radius=RADIUS)
    track = ReferenceTrack(from_waypoints(path), half_width=CORRIDOR_HALF_WIDTH)
    cones = [(x, 0.5 * WIDTH + lean) for x, lean in zip(CONE_X, CONE_LEAN)]
    scene = Scene(obstacles=[Sphere(c, CONE_RADIUS + CONE_MARGIN) for c in cones])

    design = UdeSRacecarDyn(named_ports=False)
    design.params["terrain"]["friction"] = PLANNER_MU
    design.state.lower[6] = 0.0
    design.state.upper[6] = 3.0 * V_TARGET / design.params["r_r"]
    design.state.lower[7] = -design.params["delta_max"]
    design.state.upper[7] = design.params["delta_max"]
    design.state.lower[8] = -design.params["P_max"]
    design.state.upper[8] = design.params["P_max"]

    centre = mount(point_probe(), on=design)
    body = mount(
        car_outline(length=BODY_LENGTH, width=BODY_WIDTH, margin=BODY_MARGIN),
        on=design,
    )
    x_cruise = np.zeros(design.n)
    x_cruise[3] = V_TARGET
    x_cruise[6] = V_TARGET / design.params["r_r"]
    x_cruise[8] = P_CRUISE
    cost = (
        QuadraticCost.from_system(
            design,
            Q=np.diag([0.0, 0.0, 0.0, 4.0, 1.0, 0.0, 0.0, 2.0, 0.0]),
            R=np.diag([0.0005, 60.0]),
            S=np.diag([0.0, 0.0, 0.0, 8.0, 2.0, 0.0, 0.0, 2.0, 0.0]),
            x_bar=x_cruise,
            u_bar=np.array([P_CRUISE, 0.0]),
        )
        + track.distance_field(centre).as_cost(
            weight=60.0, shaping=quadratic_excess(threshold=0.05)
        )
        + track.corridor_field(body).as_cost(
            weight=200.0, shaping=quadratic_hinge(threshold=0.0)
        )
        + scene.clearance_field(body).as_cost(
            weight=400.0, shaping=quadratic_hinge(threshold=CONE_CLEARANCE)
        )
    )

    first = int(np.argmin(np.abs(path[:, 0]) + np.abs(path[:, 1] + 0.5 * WIDTH)))
    step = path[(first + 1) % path.shape[0]] - path[first]
    heading = np.arctan2(step[1], step[0]) + HEADING_NUDGE
    start = path[first] + LATERAL_START * np.array([-np.sin(heading), np.cos(heading)])
    x0 = np.array(
        [
            start[0],
            start[1],
            heading,
            V_TARGET,
            0.0,
            0.0,
            V_TARGET / design.params["r_r"],
            0.0,
            P_CRUISE,
        ]
    )

    planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=design,
            x0=x0,
            cost=cost,
            horizon=MPC_HORIZON,
            X=design.state.box,
            U=design.input_box,
        ),
        n_steps=MPC_STEPS,
        transcription="direct_collocation",
        backend="jax",
        method="slsqp",
        options={"maxiter": 60, "ftol": 1.0},
    )
    mpc = ModelPredictiveController(planner, delay=True)

    car = UdeSRacecarDyn(named_ports=False)
    car.params["terrain"]["friction"] = PLANT_MU
    car.x0 = x0.copy()
    car.camera_scale = CAMERA_SCALE
    car.skin = racecar_skin_3d

    closed_loop = (mpc % MPC_DT) @ car
    result = closed_loop.compute_trajectory(tf=TF_SIM, dt=SIM_DT)
    # one sample per video frame: the trail's pages stay small
    frames = result.resample(n_samples=int(30 * TF_SIM / PLAYBACK) + 1)
    traj = plant_run(frames, closed_loop)
    overlays = mpc_animation_overlays(
        result, mpc, loop=closed_loop, scene=scene, track=track, traj=traj
    )
    save_meshcat(
        closed_loop,
        frames,
        "racecar_mpc.html",
        "racecar_mpc.gif",
        speed=PLAYBACK,
        overlays=overlays,
    )


def gif_rocket() -> None:
    from minilink import (
        CostFunction,
        Gaussian,
        PlanningProblem,
        ReinforcementLearningPlanner,
        Rocket,
    )
    from minilink.graphical.catalog import camera_matrix

    # examples/demos/rl/rocket_landing_rl.py: PPO learns to land from 20 m.
    TRAINING_TIMESTEPS = 4_000_000
    DT, TF = 0.05, 15.0  # [s]
    GIMBAL, THRUST_TO_WEIGHT = 0.05, 2.0  # [rad], [-]
    X_LANDED = np.array([0.0, 1.0, 0.0, 0.0, 0.0, 0.0])
    FEATURE_SCALE = np.array([0.25, 0.25, 1.0, 0.25, 0.25, 1.0])
    TF_SHOWN = 7.0  # [s] the clip: the descent, the touchdown, a beat after

    plant = Rocket()
    plant.params["inertia"] = 1000.0
    weight = plant.params["mass"] * plant.params["gravity"]
    plant.inputs["u"].lower = np.array([0.0, -GIMBAL])
    plant.inputs["u"].upper = np.array([THRUST_TO_WEIGHT * weight, GIMBAL])
    plant.state.lower = np.array([-100.0, 0.0, -1.0, -50.0, -50.0, -5.0])
    plant.state.upper = np.array([100.0, 200.0, 1.0, 50.0, 50.0, 5.0])

    class LandingCost(CostFunction):
        Q = 0.1 * np.diag([1.0, 1.0, 10.0, 0.1, 0.1, 1.0])
        R = 0.1 * np.diag([1e-8, 1.0])

        def g(self, x, u, t=0.0, params=None):
            Q, R = self.Q, self.R
            dx = x - X_LANDED
            return dx @ Q @ dx + u @ R @ u

        def h(self, x, t=0.0, params=None):
            return 0.0

    problem = PlanningProblem(
        plant,
        cost=LandingCost(),
        horizon=TF,
        x0=Gaussian([0.0, 20.0, 0.0, 0.0, 0.0, 0.0], [10.0, 8.0, 0.3, 2.0, 2.0, 0.3]),
        penalize_failure=True,
        failure_cost=100.0,
        X=plant.state.box,
        U=plant.input_box,
    )
    planner = ReinforcementLearningPlanner(
        problem,
        dt=DT,
        features=lambda x: (x - X_LANDED) * FEATURE_SCALE,
        hidden=(64, 64),
        method="ppo",
        n_rollouts=64,
        n_steps_per_update=32,
        batch_size=256,
        learning_rate=1e-3,
        gamma=0.995,
        log_std_init=-1.0,
        seed=0,
    )
    planner.solve(n_timesteps=TRAINING_TIMESTEPS)

    plant.x0 = np.array([6.0, 14.0, 0.0, 0.0, 0.0, 0.0])
    cl_sys = planner.controller @ plant
    traj = cl_sys.compute_trajectory(tf=TF_SHOWN, dt=0.01)
    camera = camera_matrix(
        target=(3.0, 7.0, 0.0), scale=8.0
    )  # the whole descent, still
    save_gif(cl_sys, traj, "rocket_landing.gif", speed=1.0, camera=camera)


def pipeline_map() -> None:
    """The library on one page: its SVG from docs/pipeline_figures.py, and a PNG for PyPI."""
    from pipeline_figures import library_map

    svg = STATIC / "pipeline_map.svg"
    svg.write_text(library_map())
    svg_to_png(svg, STATIC / "pipeline_map.png")
    print("pipeline_map.png")


# name: (builder, the files it writes under docs/_static)
ASSETS = {
    "map": (pipeline_map, ["pipeline_map.png"]),
    "diagram": (diagram_png, ["diagram_closed_loop.png"]),
    "breadth": (breadth_strip, ["tool_breadth.png"]),
    "trajopt": (trajopt_iterations, ["trajopt_iterations.gif"]),
    "comparison": (comparison_runs, ["comparison_runs.png"]),
    "montage": (tool_montage, ["tool_montage.png"]),
    "cartpole": (gif_cartpole, ["cartpole_swingup.gif"]),
    "ur5": (ur5_impedance, ["ur5_meshcat.gif", "showcase/ur5_impedance.html"]),
    "racecar": (racecar_mpc, ["racecar_mpc.gif", "showcase/racecar_mpc.html"]),
    "rocket": (gif_rocket, ["rocket_landing.gif"]),
}


if __name__ == "__main__":
    for name in sys.argv[1:] or list(ASSETS):
        print(f"--- {name}")
        ASSETS[name][0]()
