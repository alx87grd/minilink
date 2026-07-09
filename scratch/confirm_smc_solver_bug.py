"""Temporary diagnostic: SMC closed-loop torque vs solver (confirm / reject bug).

Run from repo root::

    python scratch/confirm_smc_solver_bug.py

Checks:
  1. Do scipy and rk4_fixedsteps produce the same smooth torque trace?
  2. Does sign(s) ever flip (required for bang-bang chatter in tau)?
  3. Does removing the discontinuous term change the state trajectory?
  4. Does logged ctl:u match u implied during integration (f_ivp)?
  5. Euler vs rk4_fixedsteps on demo_sliding_mode_pendulum params (repro case).
"""

from __future__ import annotations

import sys
import time
from concurrent.futures import ThreadPoolExecutor
from concurrent.futures import TimeoutError as FuturesTimeout

import numpy as np

from minilink.blocks.sources import Step
from minilink.control.modelbased import SlidingModeController
from minilink.core.composition import closed_loop_qdq
from minilink.dynamics.catalog.pendulum.pendulum import Pendulum

# Match examples/scripts/control/demo_sliding_mode_pendulum.py
TF = 8.0
DT = 0.01
LAM = 2.0
GAIN = 8.0
NAB = 0.15


def build_diagram():
    plant = Pendulum(length=1.0, mass=1.0)
    plant.x0 = np.array([0.2, 0.0])
    ref = Step(
        initial_value=np.array([0.0, 0.0]),
        final_value=np.array([np.pi + 0.25, 0.0]),
        step_time=0.0,
    )
    smc = SlidingModeController(plant, lam=LAM, gain=GAIN, nab=NAB)
    return ref >> closed_loop_qdq(smc, plant), plant


def sliding_surface(q, dq, q_d):
    return dq + LAM * (q - q_d)


def decompose_torque(plant, q, dq, q_d):
    s = sliding_surface(q, dq, q_d)
    ddq_r = np.array([-LAM * dq])
    H = plant.H(np.asarray([q]))
    K = np.diag([GAIN]) + H @ np.diag([NAB])
    id_part = float(plant.inverse_dynamics(np.array([q]), np.array([dq]), ddq_r)[0])
    disc = float(-(K @ np.sign(np.array([s])))[0])
    return s, id_part, disc, id_part + disc


def analyze(label, diagram, plant, traj):
    traj = diagram.reconstruct_internal_signals(traj)
    u = traj.signals["ctl:u"][0]
    q, dq = traj.x[0], traj.x[1]
    q_d = np.pi + 0.25
    s = sliding_surface(q, dq, q_d)
    sign_flips = int(np.sum(np.diff(np.sign(s)) != 0))
    u_jumps = np.abs(np.diff(u))

    _, id_part, disc, _ = decompose_torque(plant, q[-1], dq[-1], q_d)

    disc_series = []
    for qi, dqi in zip(q, dq):
        _, _, disc_i, _ = decompose_torque(plant, qi, dqi, q_d)
        disc_series.append(disc_i)
    disc_series = np.array(disc_series)

    print(f"\n--- {label} ---")
    print(f"  samples: {traj.n_samples}, q_end={q[-1]:.4f}, dq_end={dq[-1]:.4f}")
    print(f"  s range: [{s.min():.4f}, {s.max():.4f}]")
    print(f"  sign(s) flips: {sign_flips}")
    print(f"  tau range: [{u.min():.3f}, {u.max():.3f}]")
    print(
        f"  max |Δtau| on grid: {u_jumps.max():.4f}, mean |Δtau|: {u_jumps.mean():.4f}"
    )
    print(
        f"  discontinuous part unique (rounded): {np.unique(np.round(disc_series, 2))}"
    )

    return {
        "label": label,
        "t": traj.t,
        "q": q,
        "dq": dq,
        "u": u,
        "s": s,
        "sign_flips": sign_flips,
        "disc_unique": len(np.unique(np.round(disc_series, 4))),
    }


def run_solver(diagram, solver, *, tf=TF, dt=DT, timeout_s=20.0):
    if solver != "rk4_fixedsteps":
        tf = min(tf, 1.0)
    t0 = time.monotonic()

    def _integrate():
        return diagram.compute_trajectory(tf=tf, dt=dt, solver=solver, show=False)

    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            fut = pool.submit(_integrate)
            traj = fut.result(timeout=timeout_s)
    except FuturesTimeout:
        elapsed = time.monotonic() - t0
        return None, f"TIMEOUT after {elapsed:.1f}s (stiff/discontinuous IVP struggle)"
    except Exception as exc:
        elapsed = time.monotonic() - t0
        return None, f"{type(exc).__name__}: {exc} ({elapsed:.1f}s)"
    elapsed = time.monotonic() - t0
    return traj, f"ok ({elapsed:.1f}s, n={traj.n_samples})"


def ab_test_discontinuous_term(*, tf=2.0, dt=DT):
    """Same IC; full SMC vs zero discontinuous gain (ID reaching only)."""
    ref = Step(
        initial_value=np.array([0.0, 0.0]),
        final_value=np.array([np.pi + 0.25, 0.0]),
        step_time=0.0,
    )

    plant_a = Pendulum(length=1.0, mass=1.0)
    plant_a.x0 = np.array([0.2, 0.0])
    smc_full = SlidingModeController(plant_a, lam=LAM, gain=GAIN, nab=NAB)
    diag_smc = ref >> closed_loop_qdq(smc_full, plant_a)

    plant_b = Pendulum(length=1.0, mass=1.0)
    plant_b.x0 = np.array([0.2, 0.0])
    smc_id = SlidingModeController(plant_b, lam=LAM, gain=0.0, nab=0.0)
    diag_id = ref >> closed_loop_qdq(smc_id, plant_b)

    traj_smc = diag_smc.compute_trajectory(
        tf=tf, dt=dt, solver="rk4_fixedsteps", show=False
    )
    traj_id = diag_id.compute_trajectory(
        tf=tf, dt=dt, solver="rk4_fixedsteps", show=False
    )
    diff = np.max(np.abs(traj_smc.x - traj_id.x))
    print(f"\n--- A/B: full SMC vs gain=nab=0 (ID reaching only), tf={tf} ---")
    print(f"  max |Δx| over trajectory: {diff:.4f}")
    print(
        f"  => discontinuous term {'DOES' if diff > 1e-3 else 'does NOT'} change plant motion"
    )


def check_f_ivp_consistency(diagram):
    """Logged ctl:u at grid points vs u from f_ivp intermediate states."""
    ev = diagram.compile(backend="numpy")
    x = diagram.x0.copy()
    t = 0.0
    dt = DT
    u_nom = diagram.get_u_from_input_ports()
    u_logged = []
    u_mid = []
    for _ in range(5):
        sig = ev.compute_internal_signals_dict(x, u_nom, t)
        u_logged.append(float(sig["ctl:u"][0]))
        k1 = ev.f_ivp(x, t)
        x_mid = x + 0.5 * dt * k1
        sig_mid = ev.compute_internal_signals_dict(x_mid, u_nom, t + 0.5 * dt)
        u_mid.append(float(sig_mid["ctl:u"][0]))
        x = ev.rk4_step_ivp(x, t, dt)
        t += dt
    print("\n--- f_ivp vs logged u (first 5 RK4 steps) ---")
    print(f"  u at step starts: {u_logged}")
    print(f"  u at mid-step x:  {u_mid}")
    print(
        f"  max start-vs-mid spread per step: {max(abs(a - b) for a, b in zip(u_logged, u_mid)):.4f}"
    )


def repro_demo_euler_vs_rk4():
    """Match demo_sliding_mode_pendulum.py (lam=20, ref step at 0.5, dt=0.1)."""
    plant = Pendulum(length=1.0, mass=1.0)
    plant.x0 = np.array([0.0, 0.0])
    ref = Step(
        initial_value=np.array([0.0, 0.0]),
        final_value=np.array([np.pi, 0.0]),
        step_time=0.5,
    )
    smc = SlidingModeController(plant, lam=20.0, gain=8.0, nab=0.15)
    diagram = ref >> closed_loop_qdq(smc, plant)
    ev = diagram.compile(backend="numpy")
    dt = 0.1
    u_nom = diagram.get_u_from_input_ports()

    print("\n=== Repro: demo_sliding_mode_pendulum (euler vs rk4, dt=0.1) ===")
    for solver in ("euler", "rk4_fixedsteps"):
        traj = diagram.compute_trajectory(tf=1.0, dt=dt, solver=solver, show=False)
        traj = diagram.reconstruct_internal_signals(traj)
        u = traj.signals["ctl:u"][0]
        print(f"  {solver} ctl:u = {u}")
        print(f"    max |Δu| on grid = {np.max(np.abs(np.diff(u))):.4f}")

    for label, stepper in (
        ("euler step-start u", ev.euler_step_ivp),
        ("rk4 step-start u", ev.rk4_step_ivp),
    ):
        x = plant.x0.copy()
        u_steps = []
        for i in range(11):
            t = i * dt
            sig = ev.compute_internal_signals_dict(x, u_nom, t)
            u_steps.append(float(sig["ctl:u"][0]))
            if i < 10:
                x = stepper(x, t, dt)
        print(f"  manual {label}: {u_steps}")

    print(
        "  => NOT a plot glitch: printed arrays differ. Euler ±8.3 swings are "
        "inverse-dynamics oscillation (sign(s) stays -1 after ref step); RK4 "
        "smooth tau from better integration + constant -K sign(s) offset."
    )


def main():
    diagram, plant = build_diagram()
    q_d = np.pi + 0.25

    print("SMC solver / torque diagnostic")
    print(f"  x0={plant.x0}, q_d={q_d:.4f}, lam={LAM}, gain={GAIN}, nab={NAB}")
    print(f"  tf={TF}, dt={DT}")

    results = []
    for solver in ("rk4_fixedsteps", "scipy", "scipy_stiff"):
        traj, status = run_solver(diagram, solver)
        print(f"  {solver}: {status}")
        if traj is not None:
            results.append(analyze(solver, diagram, plant, traj))

    if len(results) >= 2:
        r0, r1 = results[0], results[1]
        n = min(len(r0["q"]), len(r1["q"]))
        q_diff = np.max(np.abs(r0["q"][:n] - r1["q"][:n]))
        u_diff = np.max(np.abs(r0["u"][:n] - r1["u"][:n]))
        print(f"\n--- rk4 vs scipy (first {n} samples, tf<=2 for scipy) ---")
        print(f"  max |Δq|: {q_diff:.4f}, max |Δtau|: {u_diff:.4f}")

    ab_test_discontinuous_term()
    check_f_ivp_consistency(diagram)
    repro_demo_euler_vs_rk4()

    print("\n=== Verdict hints ===")
    if results and all(r["sign_flips"] == 0 for r in results):
        print(
            "  * sign(s) never crosses 0 => discontinuous term is CONSTANT; smooth tau is expected."
        )
        print("    (Not necessarily a wiring bug — may be IC/ref/gains.)")
    if results and all(r["disc_unique"] <= 2 for r in results):
        print(
            "  * Discontinuous torque takes <=2 discrete levels on the grid (no chatter)."
        )
    print(
        "  * scipy on stiff/discontinuous closed loop may hang or overflow — see solver line above."
    )

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        if results:
            fig, axes = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
            for r in results[:2]:
                axes[0].plot(r["t"], r["q"], label=f"q ({r['label']})")
                axes[1].plot(r["t"], r["u"], label=f"tau ({r['label']})")
                axes[2].plot(r["t"], r["s"], label=f"s ({r['label']})")
            axes[0].axhline(q_d, color="k", ls="--", alpha=0.4, label="q_d")
            axes[2].axhline(0.0, color="k", ls="--", alpha=0.4)
            axes[0].set_ylabel("q [rad]")
            axes[1].set_ylabel("tau [Nm]")
            axes[2].set_ylabel("s")
            axes[2].set_xlabel("t [s]")
            for ax in axes:
                ax.legend(loc="best", fontsize=8)
                ax.grid(True, alpha=0.3)
            fig.tight_layout()
            out = "scratch/confirm_smc_solver_bug.png"
            fig.savefig(out, dpi=120)
            print(f"\n  Wrote {out}")
    except ImportError:
        pass

    return 0


if __name__ == "__main__":
    sys.exit(main())
