"""Build the README / landing-page assets: diagram PNG, GIFs, standalone pitch deck."""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
STATIC = ROOT / "docs" / "_static"
PITCH = ROOT / "docs" / "pitch"

import numpy as np  # noqa: E402

GIF_WIDTH = 480  # px
GIF_FPS = 15
GIF_MAX_BYTES = 1_000_000
# ur5_meshcat.gif is a screen capture of the meshcat viewer (not rebuilt here).


def shrink_gif(path: Path, *, width: int = GIF_WIDTH, fps: int = GIF_FPS) -> None:
    """Resize and resample a GIF written at export DPI down to README size."""
    from PIL import Image, ImageSequence

    with Image.open(path) as im:
        src_ms = im.info.get("duration", 1000 / 30)
        keep_every = max(1, int(round((1000 / fps) / src_ms)))
        frames = []
        for k, frame in enumerate(ImageSequence.Iterator(im)):
            if k % keep_every:
                continue
            rgb = frame.convert("RGB")
            h = int(round(rgb.height * width / rgb.width))
            frames.append(rgb.resize((width, h), Image.LANCZOS).quantize(colors=96))
    frames[0].save(
        path,
        save_all=True,
        append_images=frames[1:],
        duration=int(1000 / fps),
        loop=0,
        optimize=True,
    )
    size = path.stat().st_size
    print(f"{path.name}: {len(frames)} frames, {size / 1e6:.2f} MB")
    if size > GIF_MAX_BYTES:
        print(f"  warning: above {GIF_MAX_BYTES / 1e6:.0f} MB budget")


def save_gif(
    sys_or_diagram, traj, name: str, *, time_factor_video: float = 2.0, camera=None
) -> None:
    out = STATIC / name
    sys_or_diagram.animate(
        traj,
        renderer="matplotlib",
        html=False,
        show=False,
        save=True,
        file_name=str(out.with_suffix("")),
        time_factor_video=time_factor_video,
        camera=camera,
    )
    shrink_gif(out)


def diagram_png() -> None:
    from minilink import ImpedanceController, Pendulum
    from minilink.graphical.diagrams.dot import get_diagram

    diagram = ImpedanceController() @ Pendulum()
    get_diagram(diagram).render(
        filename=str(STATIC / "diagram_closed_loop"), format="png", cleanup=True
    )
    print("diagram_closed_loop.png")


def gif_pendulum() -> None:
    from minilink import ImpedanceController, Pendulum

    plant = Pendulum()  # catalog defaults: the camera frames the rod
    plant.x0[0] = 2.0
    diagram = ImpedanceController() @ plant
    traj = diagram.compute_trajectory(tf=8.0, verbose=False)
    save_gif(diagram, traj, "pendulum_impedance.gif")


def gif_cartpole() -> None:
    from minilink import (
        CartPole,
        PlanningProblem,
        QuadraticCost,
        TrajectoryOptimizationPlanner,
    )

    plant = CartPole()
    plant.inputs["u"].lower_bound[0] = -10.0
    plant.inputs["u"].upper_bound[0] = 10.0
    x_goal = np.array([0.0, np.pi, 0.0, 0.0])
    problem = PlanningProblem(
        sys=plant,
        tf=4.0,
        x_start=np.array([-2.0, 1.0, 0.0, 0.0]),
        x_goal=x_goal,
        cost=QuadraticCost.from_system(
            plant, Q=np.diag([1.0, 1.0, 0.0, 0.0]), R=np.diag([0.01]), xbar=x_goal
        ),
    )
    planner = TrajectoryOptimizationPlanner(
        problem,
        n_steps=40,
        transcription="direct_collocation",
        compile_backend="jax",
        optimizer_method="ipopt",
        verbose=False,
    )
    traj = planner.solve().trajectory.resample(n_samples=240)
    save_gif(
        plant, traj, "cartpole_swingup.gif", time_factor_video=1.0
    )  # default camera


def gif_mpc_car() -> None:
    from minilink import (
        BicycleDynRate,
        PlanningProblem,
        QuadraticCost,
        TrajectoryOptimizationPlanner,
    )
    from minilink.control.mpc import ModelPredictiveController, mpc_animation_overlays
    from minilink.graphical.animation.camera import follow_frame_camera

    u_target = 4.0
    plant = BicycleDynRate()
    r_r = plant.params["r_r"]
    x_ref = np.array([0.0, 0.0, 0.0, u_target, 0.0, 0.0, u_target / r_r, 0.0])
    x0 = np.array([0.0, 3.0, 0.0, 0.8 * u_target, 0.0, 0.0, 0.8 * u_target / r_r, 0.0])
    plant.x0 = x0.copy()
    mpc_planner = TrajectoryOptimizationPlanner(
        PlanningProblem(
            sys=plant,
            tf=2.0,
            x_start=x0,
            cost=QuadraticCost.from_system(
                plant,
                Q=np.diag([0.0, 12.0, 18.0, 0.5, 4.0, 6.0, 0.1, 100.0]),
                R=np.diag([1.0, 25.0]),
                S=np.diag([0.0, 30.0, 40.0, 2.0, 12.0, 18.0, 0.1, 100.0]),
                xbar=x_ref,
            ),
        ),
        n_steps=5,
        transcription="direct_collocation",
        compile_backend="jax",
        optimizer_method="scipy_slsqp",
        optimizer_options={"maxiter": 10, "ftol": 1.0},
    )
    mpc = ModelPredictiveController(
        mpc_planner, dt_mpc=0.2, warm_start=True, verbose=False
    )
    hybrid = mpc @ plant
    result = hybrid.compute_trajectory(
        tf=8.0, x0_plant=x0, plant_dt_inner=0.02, compile_backend="jax"
    )
    out = STATIC / "mpc_car.gif"
    hybrid.animate(
        renderer="matplotlib",
        html=False,
        show=False,
        save=True,
        file_name=str(out.with_suffix("")),
        time_factor_video=2.0,
        overlays=mpc_animation_overlays(result, mpc_planner, reference_pad=20.0),
        camera=follow_frame_camera("plant:body", scale=7.0),
    )
    shrink_gif(out)


def pitch_html() -> None:
    """Inline slides.html + pitch.css into a standalone deck (offline-capable)."""
    slides = PITCH / "slides.html"
    css = PITCH / "pitch.css"
    if not slides.exists() or not css.exists():
        print("pitch: docs/pitch/slides.html or pitch.css missing, skipped")
        return
    body = slides.read_text().replace("_static/", "")
    nav = """
<script>
(() => {
  const slides = [...document.querySelectorAll('.slide')];
  const counter = document.createElement('div');
  counter.className = 'ml-counter';
  document.body.appendChild(counter);
  let i = 0;
  const current = () => {
    const y = window.scrollY + window.innerHeight / 2;
    return slides.findIndex(s => s.offsetTop <= y && y < s.offsetTop + s.offsetHeight);
  };
  const go = k => { i = Math.max(0, Math.min(slides.length - 1, k)); slides[i].scrollIntoView({behavior: 'smooth'}); };
  const update = () => { const k = current(); if (k >= 0) i = k; counter.textContent = `${i + 1} / ${slides.length}`; };
  window.addEventListener('scroll', update, {passive: true});
  window.addEventListener('keydown', e => {
    if (['ArrowRight', 'ArrowDown', 'PageDown', ' '].includes(e.key)) { e.preventDefault(); go(i + 1); }
    else if (['ArrowLeft', 'ArrowUp', 'PageUp'].includes(e.key)) { e.preventDefault(); go(i - 1); }
    else if (e.key === 'Home') go(0);
    else if (e.key === 'End') go(slides.length - 1);
    else if (e.key.toLowerCase() === 'f') document.documentElement.requestFullscreen?.();
  });
  update();
})();
</script>
"""
    html = (
        '<!doctype html>\n<html lang="en">\n<head>\n<meta charset="utf-8">\n'
        '<meta name="viewport" content="width=device-width, initial-scale=1">\n'
        "<title>minilink</title>\n<style>\n" + css.read_text() + "\n</style>\n</head>\n"
        '<body class="ml-deck">\n' + body + nav + "</body>\n</html>\n"
    )
    (STATIC / "pitch.html").write_text(html)
    print("pitch.html")


STEPS = {
    "diagram": diagram_png,
    "pendulum": gif_pendulum,
    "cartpole": gif_cartpole,
    "mpc": gif_mpc_car,
    "pitch": pitch_html,
}

if __name__ == "__main__":
    wanted = sys.argv[1:] or list(STEPS)
    for key in wanted:
        print(f"--- {key}")
        STEPS[key]()
