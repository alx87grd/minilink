"""Regenerate the committed UR5 simulation baseline fixture.

Run from repo root::

    python tests/fixtures/ur5_sim_baseline/regenerate.py
"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from minilink.dynamics.catalog.manipulators.ur5 import UR5Manipulator
from minilink.simulation.simulator import Simulator

HERE = Path(__file__).resolve().parent

TF = 0.5
DT = 0.001
SOLVER = "rk4_fixedsteps"
GRAVITY = 0.0
Q0 = np.array([0.0, -np.pi / 2 + 0.2, 0.0, -np.pi / 2, 0.0, 0.0])
DQ0 = np.array([0.2, -0.1, 0.15, -0.05, 0.08, -0.03])


def main() -> None:
    x0 = np.concatenate([Q0, DQ0])
    u0 = np.zeros(6)
    arm = UR5Manipulator()
    arm.params["gravity"] = GRAVITY
    arm.x0 = x0
    arm.inputs["u"].nominal_value = u0

    traj = Simulator(
        arm,
        x0=x0,
        t0=0.0,
        tf=TF,
        dt=DT,
        solver=SOLVER,
        compile_backend="numpy",
        verbose=False,
    ).solve()
    if not np.all(np.isfinite(traj.x)):
        raise RuntimeError("baseline trajectory is non-finite")

    np.savez_compressed(
        HERE / "trajectory.npz",
        t=np.asarray(traj.t, dtype=np.float64),
        x=np.asarray(traj.x, dtype=np.float64),
        u=np.asarray(traj.u, dtype=np.float64),
    )
    manifest = {
        "plant": "UR5Manipulator",
        "module": "minilink.dynamics.catalog.manipulators.ur5",
        "tf": TF,
        "dt": DT,
        "solver": SOLVER,
        "compile_backend": "numpy",
        "input_mode": "zero_torque",
        "gravity": GRAVITY,
        "x0": x0.tolist(),
        "u": u0.tolist(),
        "n_samples": int(traj.t.size),
    }
    (HERE / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    print(f"wrote {HERE / 'trajectory.npz'} shape={traj.x.shape}")


if __name__ == "__main__":
    main()
