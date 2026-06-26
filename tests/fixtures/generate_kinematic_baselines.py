"""Generate kinematic regression baselines (run once before contract upgrade)."""

from __future__ import annotations

import numpy as np

from minilink.core.kinematics import identity_matrix
from minilink.core.trajectory import Trajectory
from minilink.dynamics.catalog.manipulators.arms import TwoLinkManipulator
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
    DynamicBicycle,
    DynamicBicycleCar3D,
    JaxDynamicBicycleRateInputs,
)
from minilink.dynamics.catalog.vehicles.steering import HolonomicMobileRobot
from minilink.graphical.animation.primitives import (
    CustomLine,
    HorizonPolyline,
    TrajectoryPolyline,
)
from minilink.graphical.animation.skins import merge_skins
from tests.fixtures.kinematic_capture import (
    capture_draw_list,
    capture_frame_png_hash,
    save_baseline,
)

# Synthetic MPC overlay plant (minimal stand-in for demo subclass).


class _MpcPlanBicycleRateBaseline(JaxDynamicBicycleRateInputs):
    def __init__(self):
        super().__init__()
        t = np.linspace(0.0, 2.0, 20)
        x = np.zeros((8, t.size))
        x[0] = np.linspace(0.0, 4.0, t.size)
        x[3] = 2.0
        x[6] = 10.0
        u = np.zeros((2, t.size))
        executed = Trajectory(t=t, x=x, u=u)
        plan_x = np.zeros((8, 15))
        plan_x[0] = np.linspace(2.0, 6.0, 15)
        plan_x[3] = 2.0
        plan_x[6] = 10.0
        plan_t = np.linspace(1.0, 2.5, 15)
        plan_u = np.zeros((2, 15))
        plan_traj = Trajectory(t=plan_t, x=plan_x, u=plan_u)
        self._ref = CustomLine(
            np.array([[-1.0, 0.0, 0.0], [8.0, 0.0, 0.0]]),
            color="k",
            linewidth=1.0,
            style="--",
        )
        self._executed = TrajectoryPolyline(
            executed, window="prefix", color="b", style="--", linewidth=1.0
        )
        self._mpc_plan = HorizonPolyline(
            [(1.0, plan_traj)],
            color="tab:orange",
            linewidth=2.0,
            style="--",
        )

    def get_kinematic_geometry(self):
        return merge_skins(
            super().get_kinematic_geometry(),
            {"world": [self._ref]},
        )

    def get_dynamic_geometry(self, x, u, t, params=None):
        return {
            "world": [
                CustomLine(
                    self._executed.points_at(t),
                    color=self._executed.color,
                    linewidth=self._executed.linewidth,
                    style=self._executed.style,
                ),
                CustomLine(
                    self._mpc_plan.points_at(t),
                    color=self._mpc_plan.color,
                    linewidth=self._mpc_plan.linewidth,
                    style=self._mpc_plan.style,
                ),
            ]
        }

    def tf(self, x, u, t, params=None):
        return {**super().tf(x, u, t, params), "world": identity_matrix()}


REFERENCE_SYSTEMS = {
    "dynamic_bicycle": (
        lambda: DynamicBicycle(),
        [
            {"x": np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]), "u": np.array([10.0, 0.1]), "t": 0.0},
            {"x": np.array([5.0, 2.0, 0.3, 4.0, 0.2, 0.05]), "u": np.array([20.0, -0.2]), "t": 0.5},
        ],
        False,
    ),
    "dynamic_bicycle_car3d": (
        lambda: DynamicBicycleCar3D(),
        [
            {"x": np.array([0.0, 0.0, 0.0, 1.0, 0.0, 0.0]), "u": np.array([10.0, 0.1]), "t": 0.0},
            {"x": np.array([3.0, 1.0, -0.2, 3.0, 0.1, 0.02]), "u": np.array([15.0, 0.3]), "t": 1.0},
        ],
        True,
    ),
    "pendulum": (
        lambda: Pendulum(length=1.0),
        [
            {"x": np.array([0.5, 0.0]), "u": np.array([2.0]), "t": 0.0},
            {"x": np.array([-0.8, 0.3]), "u": np.array([-1.5]), "t": 0.0},
        ],
        False,
    ),
    "two_link_manipulator": (
        lambda: TwoLinkManipulator(),
        [
            {"x": np.array([0.3, -0.4]), "u": np.array([0.0, 0.0]), "t": 0.0},
            {"x": np.array([1.0, 0.5]), "u": np.array([1.0, -0.5]), "t": 0.0},
        ],
        False,
    ),
    "holonomic_mobile_robot": (
        lambda: HolonomicMobileRobot(),
        [
            {"x": np.array([1.0, 2.0, 0.5]), "u": np.array([0.5, 0.0, 0.1]), "t": 0.0},
            {"x": np.array([-0.5, 1.0, -0.3]), "u": np.array([0.0, 0.2, 0.0]), "t": 0.0},
        ],
        False,
    ),
    "mpc_plan_bicycle": (
        _MpcPlanBicycleRateBaseline,
        [
            {
                "x": np.array([2.0, 0.0, 0.0, 2.0, 0.0, 0.0, 10.0, 0.0]),
                "u": np.zeros(2),
                "t": 1.2,
            },
            {
                "x": np.array([4.0, 0.5, 0.1, 2.0, 0.0, 0.0, 15.0, 0.1]),
                "u": np.zeros(2),
                "t": 1.8,
            },
        ],
        False,
    ),
}


def main():
    for name, (factory, cases, is_3d) in REFERENCE_SYSTEMS.items():
        sys = factory()
        baseline_cases = []
        for case in cases:
            x, u, t = case["x"], case["u"], case["t"]
            baseline_cases.append(
                {
                    "x": x.tolist(),
                    "u": u.tolist(),
                    "t": t,
                    "draw_list": capture_draw_list(sys, x, u, t),
                    "png_hash": capture_frame_png_hash(sys, x, u, t, is_3d=is_3d),
                }
            )
        path = save_baseline(name, baseline_cases)
        print(f"Wrote {path}")


if __name__ == "__main__":
    main()
