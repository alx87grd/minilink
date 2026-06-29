"""Headless pixel-parity gate for the v2 render pipeline (Phase 3 catalog migration).

Compares ``sys.render(x, u, t)`` (legacy ``Animator``) against
``sys.render_v2(x, u, t)`` (``Animator2`` + frame-keyed hooks) by rasterizing both
to an RGBA buffer with the Agg backend and diffing pixels. A case may compare two
*different* plants (e.g. the old ``DynamicBicycleCar3D`` via ``render`` vs the base
``DynamicBicycle`` skinned with ``car_skin_3d`` via ``render_v2``) — that is the
invariant that lets Phase 5 retire the bespoke 3D subclass.

Run: ``python scripts/check_render_v2_parity.py``
"""

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402

from minilink.graphical.animation.renderers import (  # noqa: E402
    matplotlib_renderer as mr,
)

# Capture the backend figure instead of letting ``show`` block / close it.
_CAPTURED = {}


def _present_noop(self, *a, **k):
    return None


def _close_capture(self):
    _CAPTURED["fig"] = self.fig


mr.MatplotlibRenderer.present = _present_noop
mr.MatplotlibRenderer.close_scene = _close_capture


def _capture(render_call):
    plt.close("all")
    _CAPTURED.clear()
    render_call()
    fig = _CAPTURED["fig"]
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba()).copy()


def compare(name, old_sys, new_sys, x, u, t, *, is_3d=False, gate=True):
    """Render *old_sys* via ``render`` and *new_sys* via ``render_v2`` and diff.

    With ``gate=False`` the case is informational (e.g. a deliberate look change)
    and does not affect the overall pass/fail.
    """
    # Match the window title so text rendering does not perturb the diff.
    new_sys.name = old_sys.name
    x = np.asarray(x, dtype=float)
    u = np.asarray(u, dtype=float)

    a = _capture(lambda: old_sys.render(x, u, t, is_3d=is_3d))
    b = _capture(lambda: new_sys.render_v2(x, u, t, is_3d=is_3d))

    if a.shape != b.shape:
        print(f"[FAIL] {name}: shape {a.shape} vs {b.shape}")
        return False

    diff = np.abs(a.astype(int) - b.astype(int))
    n_diff = int((diff.any(axis=-1)).sum())
    max_diff = int(diff.max())
    if not gate:
        tag = "INFO"
    else:
        tag = "PASS" if n_diff == 0 else "FAIL"
    print(f"[{tag}] {name}: {n_diff} px differ, max channel diff {max_diff}")
    return (not gate) or n_diff == 0


def main():
    from minilink.dynamics.catalog.vehicles.dynamic_bicycle import (
        DynamicBicycle,
        DynamicBicycleCar3D,
    )
    from minilink.dynamics.catalog.vehicles.mountain_car import MountainCar
    from minilink.dynamics.catalog.vehicles.propulsion import (
        LongitudinalFrontWheelDriveCarWithTorqueInput,
        LongitudinalFrontWheelDriveCarWithWheelSlipInput,
    )
    from minilink.dynamics.catalog.vehicles.steering import (
        ConstantSpeedKinematicCar,
        HolonomicMobileRobot,
        HolonomicMobileRobot3D,
        KinematicBicycle,
        KinematicCar,
        UdeSRacecar,
    )

    ok = True

    # --- DynamicBicycle 2-D centerline self-parity --------------------------
    x_db = np.array([2.0, 1.0, 0.5, 8.0, 0.3, 0.2])
    u_db = np.array([30.0, 0.1])
    ok &= compare(
        "DynamicBicycle 2D", DynamicBicycle(), DynamicBicycle(), x_db, u_db, 0.0
    )

    # --- DynamicBicycleCar3D self-parity (3-D four-wheel + corner arrows) ----
    ok &= compare(
        "DynamicBicycleCar3D",
        DynamicBicycleCar3D(),
        DynamicBicycleCar3D(),
        x_db,
        u_db,
        0.0,
        is_3d=True,
    )

    # --- KinematicBicycle self-parity ---------------------------------------
    x_kb = np.array([1.2, -0.7, 0.6])
    u_kb = np.array([2.0, 0.2])
    ok &= compare(
        "KinematicBicycle", KinematicBicycle(), KinematicBicycle(), x_kb, u_kb, 0.3
    )
    ok &= compare("KinematicCar", KinematicCar(), KinematicCar(), x_kb, u_kb, 0.3)
    ok &= compare(
        "ConstantSpeedKinematicCar",
        ConstantSpeedKinematicCar(),
        ConstantSpeedKinematicCar(),
        x_kb,
        np.array([0.2]),
        0.3,
    )
    # In-frame speed: at this plant's extreme camera zoom a longer arrow is
    # clipped at the axis spine, where a 1-ulp coordinate difference flips a
    # handful of anti-aliased boundary pixels (same float fragility as the
    # Phase 0 baseline drift; not a geometry difference).
    ok &= compare(
        "UdeSRacecar", UdeSRacecar(), UdeSRacecar(), x_kb, np.array([0.3, 0.2]), 0.3
    )
    ok &= compare(
        "HolonomicMobileRobot",
        HolonomicMobileRobot(),
        HolonomicMobileRobot(),
        np.array([1.0, 2.0]),
        np.array([0.5, -0.3]),
        0.0,
    )
    ok &= compare(
        "HolonomicMobileRobot3D",
        HolonomicMobileRobot3D(),
        HolonomicMobileRobot3D(),
        np.array([1.0, 2.0, 0.5]),
        np.array([0.5, -0.3, 0.2]),
        0.0,
        is_3d=True,
    )

    # --- propulsion ---------------------------------------------------------
    ok &= compare(
        "FWD car (slip input)",
        LongitudinalFrontWheelDriveCarWithWheelSlipInput(),
        LongitudinalFrontWheelDriveCarWithWheelSlipInput(),
        np.array([1.0, 5.0]),
        np.array([-0.1]),
        0.0,
    )
    ok &= compare(
        "FWD car (torque input)",
        LongitudinalFrontWheelDriveCarWithTorqueInput(),
        LongitudinalFrontWheelDriveCarWithTorqueInput(),
        np.array([1.0, 5.0, 2.0, 0.5]),
        np.array([100.0]),
        0.0,
    )

    # --- mountain car -------------------------------------------------------
    ok &= compare(
        "MountainCar",
        MountainCar(),
        MountainCar(),
        np.array([-0.5, 0.2]),
        np.array([0.7]),
        0.0,
    )

    # --- suspension ---------------------------------------------------------
    from minilink.dynamics.catalog.vehicles.suspension import QuarterCarOnRoughTerrain

    ok &= compare(
        "QuarterCar",
        QuarterCarOnRoughTerrain(),
        QuarterCarOnRoughTerrain(),
        np.array([0.3, 0.8, 2.0]),
        np.array([3.0]),
        0.0,
    )

    # --- pendulum family (3b) ----------------------------------------------
    from minilink.dynamics.catalog.pendulum.cartpole import (
        CartPole,
        JaxCartPole,
        RotatingCartPole,
        UnderactuatedRotatingCartPole,
    )
    from minilink.dynamics.catalog.pendulum.double_pendulum import (
        Acrobot,
        DoublePendulum,
    )
    from minilink.dynamics.catalog.pendulum.pendulum import (
        InvertedPendulum,
        Pendulum,
        TwoIndependentPendulums,
    )

    ok &= compare(
        "Pendulum", Pendulum(), Pendulum(), np.array([0.7, 0.3]), np.array([1.0]), 0.0
    )
    ok &= compare(
        "InvertedPendulum",
        InvertedPendulum(),
        InvertedPendulum(),
        np.array([0.3, 0.1]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "TwoIndependentPendulums",
        TwoIndependentPendulums(),
        TwoIndependentPendulums(),
        np.array([0.5, -0.4, 0.0, 0.0]),
        np.array([1.0, -0.5]),
        0.0,
    )
    ok &= compare(
        "DoublePendulum",
        DoublePendulum(),
        DoublePendulum(),
        np.array([0.4, -0.3, 0.0, 0.0]),
        np.array([1.0, -0.5]),
        0.0,
    )
    ok &= compare(
        "Acrobot",
        Acrobot(),
        Acrobot(),
        np.array([0.3, -0.2, 0.0, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "CartPole",
        CartPole(),
        CartPole(),
        np.array([1.0, 2.0, 0.0, 0.0]),
        np.array([5.0]),
        0.0,
    )
    ok &= compare(
        "JaxCartPole",
        JaxCartPole(),
        JaxCartPole(),
        np.array([1.0, 2.0, 0.0, 0.0]),
        np.array([5.0]),
        0.0,
    )
    ok &= compare(
        "RotatingCartPole",
        RotatingCartPole(),
        RotatingCartPole(),
        np.array([1.0, 0.25, 0.0, 0.0]),
        np.array([1.0, 0.5]),
        0.0,
        is_3d=True,
    )
    ok &= compare(
        "UnderactuatedRotatingCartPole",
        UnderactuatedRotatingCartPole(),
        UnderactuatedRotatingCartPole(),
        np.array([1.0, 0.25, 0.0, 0.0]),
        np.array([1.0]),
        0.0,
        is_3d=True,
    )

    # --- manipulators (3c) --------------------------------------------------
    from minilink.dynamics.catalog.manipulators.arms import (
        FiveLinkPlanarManipulator,
        OneLinkManipulator,
        SpeedControlledManipulator,
        ThreeLinkManipulator3D,
        TwoLinkManipulator,
    )

    ok &= compare(
        "OneLinkManipulator",
        OneLinkManipulator(),
        OneLinkManipulator(),
        np.array([0.5, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "TwoLinkManipulator",
        TwoLinkManipulator(),
        TwoLinkManipulator(),
        np.array([0.2, -0.3, 0.0, 0.0]),
        np.array([0.5, -0.3]),
        0.0,
    )
    ok &= compare(
        "FiveLinkPlanarManipulator",
        FiveLinkPlanarManipulator(),
        FiveLinkPlanarManipulator(),
        np.array([0.1, 0.2, -0.1, 0.3, -0.2, 0.0, 0.0, 0.0, 0.0, 0.0]),
        np.array([0.5, -0.3, 0.2, 0.1, -0.1]),
        0.0,
    )
    ok &= compare(
        "ThreeLinkManipulator3D",
        ThreeLinkManipulator3D(),
        ThreeLinkManipulator3D(),
        np.array([0.3, 0.4, -0.2, 0.0, 0.0, 0.0]),
        np.array([0.0, 0.0, 0.0]),
        0.0,
        is_3d=True,
    )
    sc = SpeedControlledManipulator(dof=2, effector_dim=2)
    sc2 = SpeedControlledManipulator(dof=2, effector_dim=2)
    ok &= compare(
        "SpeedControlledManipulator",
        sc,
        sc2,
        np.array([0.3, -0.2]),
        np.array([0.5, -0.3]),
        0.0,
    )

    # --- mass-spring-damper (3d) -------------------------------------------
    from minilink.dynamics.catalog.mass_spring_damper.linear import (
        FloatingSingleMass,
        FloatingThreeMass,
        FloatingTwoMass,
        SingleMass,
        ThreeMass,
        TwoMass,
    )

    ok &= compare(
        "SingleMass",
        SingleMass(),
        SingleMass(),
        np.array([0.5, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "TwoMass",
        TwoMass(),
        TwoMass(),
        np.array([0.3, -0.2, 0.0, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "ThreeMass",
        ThreeMass(),
        ThreeMass(),
        np.array([0.3, -0.2, 0.4, 0.0, 0.0, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "FloatingSingleMass",
        FloatingSingleMass(),
        FloatingSingleMass(),
        np.array([0.5, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "FloatingTwoMass",
        FloatingTwoMass(),
        FloatingTwoMass(),
        np.array([0.3, -0.2, 0.0, 0.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "FloatingThreeMass",
        FloatingThreeMass(),
        FloatingThreeMass(),
        np.array([0.3, -0.2, 0.4, 0.0, 0.0, 0.0]),
        np.array([1.0]),
        0.0,
    )

    # --- aerial (3d) --------------------------------------------------------
    from minilink.dynamics.catalog.aerial.drone import (
        ConstantSpeedHelicopterTunnel,
        Drone2D,
        Drone2DWithSideThruster,
        SpeedControlledDrone2D,
    )
    from minilink.dynamics.catalog.aerial.plane import Plane2D
    from minilink.dynamics.catalog.aerial.rocket import Rocket

    ok &= compare(
        "Drone2D",
        Drone2D(),
        Drone2D(),
        np.array([0.5, 1.0, 0.2, 0.0, 0.0, 0.0]),
        np.array([5.0, 4.0]),
        0.0,
    )
    ok &= compare(
        "Drone2DWithSideThruster",
        Drone2DWithSideThruster(),
        Drone2DWithSideThruster(),
        np.array([0.5, 1.0, 0.2, 0.0, 0.0, 0.0]),
        np.array([5.0, 4.0, 2.0]),
        0.0,
    )
    ok &= compare(
        "SpeedControlledDrone2D",
        SpeedControlledDrone2D(),
        SpeedControlledDrone2D(),
        np.array([0.5, 1.0]),
        np.array([0.5, -0.3]),
        0.0,
    )
    ok &= compare(
        "ConstantSpeedHelicopterTunnel",
        ConstantSpeedHelicopterTunnel(),
        ConstantSpeedHelicopterTunnel(),
        np.array([0.2, 0.8, 2.0]),
        np.array([3.0]),
        0.0,
    )
    ok &= compare(
        "Rocket",
        Rocket(),
        Rocket(),
        np.array([0.5, 1.0, 0.1, 0.0, 0.0, 0.0]),
        np.array([15000.0, 0.05]),
        0.0,
    )
    ok &= compare(
        "Plane2D",
        Plane2D(),
        Plane2D(),
        np.array([0.0, 0.0, 0.05, 12.0, 0.5, 0.0]),
        np.array([2.0, -0.1]),
        0.0,
    )

    # --- marine (3d) --------------------------------------------------------
    from minilink.dynamics.catalog.marine.boat import Boat2D, Boat2DWithCurrent

    ok &= compare(
        "Boat2D",
        Boat2D(),
        Boat2D(),
        np.array([0.5, 1.0, 0.3, 0.5, 0.1, 0.05]),
        np.array([1000.0, 50.0]),
        0.0,
    )
    bh1, bh2 = Boat2D(), Boat2D()
    bh1.show_hydrodynamic_forces = True
    bh2.show_hydrodynamic_forces = True
    ok &= compare(
        "Boat2D (hydro)",
        bh1,
        bh2,
        np.array([0.5, 1.0, 0.3, 0.5, 0.1, 0.05]),
        np.array([1000.0, 50.0]),
        0.0,
    )
    ok &= compare(
        "Boat2DWithCurrent",
        Boat2DWithCurrent(),
        Boat2DWithCurrent(),
        np.array([0.5, 1.0, 0.3, 0.5, 0.1, 0.05]),
        np.array([1000.0, 50.0]),
        0.0,
    )

    # --- integrators (3d) ---------------------------------------------------
    from minilink.dynamics.catalog.equations.integrators import (
        DoubleIntegrator,
        SimpleIntegrator,
        TripleIntegrator,
    )

    ok &= compare(
        "SimpleIntegrator",
        SimpleIntegrator(),
        SimpleIntegrator(),
        np.array([2.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "DoubleIntegrator",
        DoubleIntegrator(),
        DoubleIntegrator(),
        np.array([2.0, -1.0]),
        np.array([1.0]),
        0.0,
    )
    ok &= compare(
        "TripleIntegrator",
        TripleIntegrator(),
        TripleIntegrator(),
        np.array([2.0, -1.0, 0.5]),
        np.array([1.0]),
        0.0,
    )

    # --- transfer function (3e) --------------------------------------------
    from minilink.blocks.transfer_function import TransferFunction

    ok &= compare(
        "TransferFunction",
        TransferFunction([1.0], [1.0, 1.0]),
        TransferFunction([1.0], [1.0, 1.0]),
        np.array([2.0]),
        np.array([1.5]),
        0.0,
    )

    # --- diagram aggregation (3e) ------------------------------------------
    from minilink.control.linear import ProportionalController
    from minilink.core.diagram import DiagramSystem

    def _pendulum_loop():
        pend = Pendulum()
        ctl = ProportionalController(K=np.array([[1.0, 0.5]]))
        d = DiagramSystem()
        d.name = "Pendulum Loop"
        d.add_subsystem(pend, "pend")
        d.add_subsystem(ctl, "ctl")
        d.add_input_port("r", dim=2)
        d.connect("input", "r", "ctl", "r")
        d.connect("pend", "y", "ctl", "y")
        d.connect("ctl", "u", "pend", "u")
        return d

    ok &= compare(
        "DiagramSystem (pendulum loop)",
        _pendulum_loop(),
        _pendulum_loop(),
        np.array([0.7, 0.3]),
        np.array([0.2, 0.0]),
        0.0,
    )

    # --- engines (3d) -------------------------------------------------------
    try:
        from minilink.dynamics.engines.contact_jax import (
            PlaneModel,
            SphereModel,
            make_world_model,
        )
        from minilink.dynamics.engines.world import PhysicsWorldSystem

        world = make_world_model(
            [SphereModel(mass=1.0, radius=0.3), SphereModel(mass=2.0, radius=0.5)],
            PlaneModel(normal=(0.0, 0.0, 1.0), offset=0.0),
        )
        pw1, pw2 = PhysicsWorldSystem(world), PhysicsWorldSystem(world)
        xw = np.array(pw1.x0, dtype=float)
        xw[0:3] = [0.0, 0.0, 1.0]
        xw[13:16] = [1.0, 0.5, 1.5]
        uw = np.zeros(12)
        ok &= compare("PhysicsWorldSystem", pw1, pw2, xw, uw, 0.0, is_3d=True)
    except ImportError:
        print("[SKIP] PhysicsWorldSystem (JAX unavailable)")

    try:
        from minilink.dynamics.engines.ancf_tire_jax import (
            ANCFTireSystem,
            make_ancf_tire_model,
        )

        ancf_model = make_ancf_tire_model(n_nodes=12, radius=0.5, mass=14.0)
        a1 = ANCFTireSystem(
            ancf_model,
            center=(0.0, 0.0, 0.4),
            contact_force_scale=0.1,
            contact_force_threshold=0.1,
        )
        a2 = ANCFTireSystem(
            ancf_model,
            center=(0.0, 0.0, 0.4),
            contact_force_scale=0.1,
            contact_force_threshold=0.1,
        )
        ok &= compare(
            "ANCFTireSystem",
            a1,
            a2,
            np.array(a1.x0, dtype=float),
            np.zeros(a1.inputs["u"].dim),
            0.0,
            is_3d=True,
        )
    except ImportError:
        print("[SKIP] ANCFTireSystem (JAX unavailable)")

    print("\nALL PASS" if ok else "\nSOME FAILED")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
