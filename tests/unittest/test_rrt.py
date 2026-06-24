import numpy as np
import pytest

from minilink.core.geometry import Sphere
from minilink.core.sets import BallSet, BoxSet
from minilink.dynamics.catalog.vehicles.steering import (
    HolonomicMobileRobot,
    KinematicBicycle,
)
from minilink.planning.problems import PlanningProblem
from minilink.planning.search.edge import Edge
from minilink.planning.search.extenders import KinodynamicExtender, SteeringExtender
from minilink.planning.search.metric import euclidean, weighted
from minilink.planning.search.rrt import RRTOptions, RRTPlanner
from minilink.planning.search.steering import DubinsSteering, StraightLineSteering
from minilink.planning.search.tree import Node, Tree
from minilink.planning.spatial.robot import sphere
from minilink.planning.spatial.scene import Scene

X_START = np.array([-4.0, -4.0])
X_GOAL = np.array([4.0, 4.0])


def make_problem():
    sys = HolonomicMobileRobot()  # dx = u
    sys.state.lower_bound = np.array([-6.0, -6.0])
    sys.state.upper_bound = np.array([6.0, 6.0])
    sys.inputs["u"].lower_bound = np.array([-1.0, -1.0])
    sys.inputs["u"].upper_bound = np.array([1.0, 1.0])

    scene = Scene(obstacles=(Sphere([0.0, 0.0], 1.0),))
    robot = sphere(radius=0.2, position=(0, 1))
    X = BoxSet.from_system_state(sys) & scene.clearance_field(robot).as_constraint()
    return PlanningProblem(sys=sys, x_start=X_START, x_goal=X_GOAL, X=X), X


COMPASS = [
    np.array([np.cos(a), np.sin(a)])
    for a in np.linspace(0.0, 2.0 * np.pi, 8, endpoint=False)
]


# --- metric + tree --------------------------------------------------------


def test_metrics():
    assert euclidean([0.0, 0.0], [3.0, 4.0]) == pytest.approx(5.0)
    d = weighted([1.0, 0.0])
    assert d([0.0, 0.0], [3.0, 4.0]) == pytest.approx(3.0)  # y ignored


def test_tree_nearest_and_near():
    tree = Tree(Node(np.array([0.0, 0.0]), None, None, 0.0))
    tree.add(Node(np.array([1.0, 0.0]), tree.root, None, 1.0))
    tree.add(Node(np.array([5.0, 0.0]), tree.root, None, 5.0))
    assert tree.nearest(np.array([0.9, 0.0]), euclidean).x[0] == pytest.approx(1.0)
    near = tree.near(np.array([0.0, 0.0]), 1.5, euclidean)
    assert len(near) == 2  # root and (1,0)


# --- kinodynamic ----------------------------------------------------------


def test_kinodynamic_reaches_goal_and_stays_free():
    problem, X = make_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    traj = planner.compute_solution()
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))
    assert traj.x.shape[0] == 2
    assert traj.u.shape == traj.x.shape  # (n, N), m == n here
    assert traj.t.shape[0] == traj.x.shape[1]


def test_kinodynamic_random_controls_reaches_goal():
    problem, _ = make_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=12, horizon=0.6, n_substeps=6),
        options=RRTOptions(seed=1, goal_tolerance=0.6, max_nodes=6000),
    )
    traj = planner.compute_solution()
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.6


def test_seeded_run_is_deterministic():
    problem, _ = make_problem()

    def run():
        return RRTPlanner(
            problem,
            extender=KinodynamicExtender(controls=COMPASS, horizon=0.6, n_substeps=6),
            options=RRTOptions(seed=7, goal_tolerance=0.5, max_nodes=4000),
        ).compute_solution()

    a, b = run(), run()
    assert a.x.shape == b.x.shape
    assert np.allclose(a.x, b.x)


# --- steering -------------------------------------------------------------


def test_steering_reaches_goal_and_stays_free():
    problem, X = make_problem()
    planner = RRTPlanner(
        problem,
        extender=SteeringExtender(
            StraightLineSteering(speed=1.0), max_distance=0.6, resolution=0.05
        ),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=4000),
    )
    traj = planner.compute_solution()
    assert np.linalg.norm(traj.x[:, -1] - X_GOAL) < 0.5
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


def test_steering_edge_is_dynamically_feasible():
    problem, _ = make_problem()
    evaluator = problem.sys.compile(backend="numpy", verbose=False)
    (edge,) = list(
        SteeringExtender(
            StraightLineSteering(1.0), max_distance=0.6, resolution=0.05
        ).propose(X_START, X_GOAL, problem, np.random.default_rng(0))
    )
    x = np.asarray(edge.states[0], dtype=float)
    for k in range(len(edge.inputs)):
        dt = float(edge.times[k + 1] - edge.times[k])
        x = np.asarray(evaluator.rk4_step(x, edge.inputs[k], 0.0, dt), dtype=float)
        assert np.allclose(x, edge.states[k + 1], atol=1e-6)


# --- dubins steering ------------------------------------------------------


def make_dubins_problem():
    sys = KinematicBicycle()  # state [x, y, theta], input [speed, steering]
    sys.state.lower_bound = np.array([-6.0, -6.0, -np.pi])
    sys.state.upper_bound = np.array([6.0, 6.0, np.pi])
    sys.inputs["u"].lower_bound = np.array([0.0, -0.5])  # forward only
    sys.inputs["u"].upper_bound = np.array([1.5, 0.5])
    return sys


def test_dubins_connect_reaches_pose_exactly_and_distance_is_finite():
    dubins = DubinsSteering(wheelbase=1.0, max_steering=0.5, speed=1.0)
    x0 = np.array([0.0, 0.0, 0.0])
    x1 = np.array([2.0, 1.5, np.pi / 2])

    states, inputs, times, cost = dubins.connect(
        x0, x1, max_distance=1e3, resolution=0.05
    )
    assert np.allclose(states[-1, :2], x1[:2], atol=1e-9)  # exact xy
    assert times.shape[0] == states.shape[0] == inputs.shape[0] + 1
    assert cost == pytest.approx(dubins.distance(x0, x1))  # arc length == metric


def test_dubins_edge_is_dynamically_feasible():
    sys = make_dubins_problem()
    evaluator = sys.compile(backend="numpy", verbose=False)
    dubins = DubinsSteering(wheelbase=sys.params["length"], max_steering=0.5, speed=1.0)

    states, inputs, times, _ = dubins.connect(
        np.array([0.0, 0.0, 0.3]),
        np.array([3.0, -1.0, -0.6]),
        max_distance=1e3,
        resolution=0.1,
    )
    x = np.asarray(states[0], dtype=float)
    for k in range(len(inputs)):
        dt = float(times[k + 1] - times[k])
        x = np.asarray(evaluator.rk4_step(x, inputs[k], 0.0, dt), dtype=float)
        assert np.allclose(x, states[k + 1], atol=1e-6)  # analytic arc == bicycle f


def test_dubins_rrt_reaches_goal_pose():
    sys = make_dubins_problem()
    x_start = np.array([-3.0, -3.0, 0.0])
    x_goal = np.array([3.0, 3.0, np.pi / 2])
    X = BoxSet.from_system_state(sys)
    problem = PlanningProblem(
        sys=sys, x_start=x_start, x_goal=x_goal, X=X, Xf=BallSet(x_goal, 0.6)
    )
    dubins = DubinsSteering(wheelbase=sys.params["length"], max_steering=0.5, speed=1.5)

    planner = RRTPlanner(
        problem,
        extender=SteeringExtender(dubins, max_distance=1.5, resolution=0.1),
        metric=dubins.distance,
        options=RRTOptions(seed=0, goal_bias=0.2, max_nodes=8000),
    )
    traj = planner.compute_solution()
    assert np.linalg.norm(traj.x[:, -1] - x_goal) < 0.6
    assert all(X.contains(traj.x[:, i]) for i in range(traj.x.shape[1]))


# --- selection ------------------------------------------------------------


def test_select_rejects_colliding_candidate_for_free_one():
    problem, _ = make_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(seed=0),
    )
    x_rand = np.array([4.0, 4.0])
    # closer to x_rand but its endpoint sits inside the obstacle
    colliding = Edge(
        states=np.array([[-0.1, -0.1], [0.0, 0.0]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 0.1]),
        cost=0.1,
    )
    # farther but collision-free
    free = Edge(
        states=np.array([[-4.0, -4.0], [-3.5, -3.5]]),
        inputs=np.zeros((1, 2)),
        times=np.array([0.0, 0.1]),
        cost=0.1,
    )
    assert planner._select([colliding, free], x_rand) is free


# --- built-in visualization -----------------------------------------------


def test_plot_tree_and_animate_search_smoke():
    mpl = pytest.importorskip("matplotlib")
    mpl.use("Agg")
    problem, _ = make_problem()
    planner = RRTPlanner(
        problem,
        extender=KinodynamicExtender(controls=COMPASS),
        options=RRTOptions(seed=0, goal_tolerance=0.5, max_nodes=2000),
    )
    planner.compute_solution()

    fig, ax = planner.plot_tree(x_axis=0, y_axis=1, show=False)
    assert fig is not None and ax is not None
    anim = planner.animate_search(x_axis=0, y_axis=1, step=20, show=False)
    assert anim is not None
