"""Prototype pygame 2D animation backend (world XY projected to the window)."""

from __future__ import annotations

import numpy as np

import matplotlib.colors as mcolors

from minilink.graphical.primitives import Circle, CustomLine, Point
from minilink.graphical.renderers.base import AnimationRenderer


def _import_pygame():
    try:
        import pygame
    except ImportError as e:
        raise ImportError(
            "pygame is required for renderer='pygame'. "
            "Install with: pip install 'minilink[visualization]'"
        ) from e
    return pygame


def _color_to_rgb(color) -> tuple[int, int, int]:
    r, g, b = mcolors.to_rgb(color)
    return (int(r * 255), int(g * 255), int(b * 255))


class PygameCanvas:
    """Maps primitives to pygame draws (world X/Y → screen, Y flipped)."""

    def __init__(self, surface, animator, is_3d: bool):
        self.surface = surface
        self.animator = animator
        self.is_3d = is_3d
        self.domain = animator.domain
        self.margin = 40
        self.w, self.h = surface.get_size()
        self._xmin, self._xmax = self.domain[0]
        self._ymin, self._ymax = self.domain[1]

    def _to_screen(self, wx: float, wy: float) -> tuple[int, int]:
        m = self.margin
        iw = self.w - 2 * m
        ih = self.h - 2 * m
        dx = self._xmax - self._xmin
        dy = self._ymax - self._ymin
        if dx <= 0:
            dx = 1.0
        if dy <= 0:
            dy = 1.0
        sx = m + (wx - self._xmin) / dx * iw
        sy = m + (self._ymax - wy) / dy * ih
        return int(round(sx)), int(round(sy))

    def _world_scale(self) -> float:
        """Pixels per world unit (average of x and y scales)."""
        m = self.margin
        iw = self.w - 2 * m
        ih = self.h - 2 * m
        dx = self._xmax - self._xmin
        dy = self._ymax - self._ymin
        if dx <= 0:
            dx = 1.0
        if dy <= 0:
            dy = 1.0
        return 0.5 * (iw / dx + ih / dy)

    def draw_primitive(self, primitive, transform_matrix, pygame_mod):
        if isinstance(primitive, Point):
            local_pt = np.append(primitive.pt, 1.0)
            world_pt = transform_matrix @ local_pt
            x, y = world_pt[0], world_pt[1]
            sx, sy = self._to_screen(x, y)
            r = max(2, int(0.5 * float(primitive.size)))
            pygame_mod.draw.circle(
                self.surface,
                _color_to_rgb(primitive.color),
                (sx, sy),
                r,
            )

        elif isinstance(primitive, CustomLine):
            local_pts = primitive.pts
            if local_pts.shape[1] == 2:
                local_pts_hom = np.hstack(
                    (local_pts, np.zeros((local_pts.shape[0], 1)))
                )
            else:
                local_pts_hom = local_pts
            local_pts_hom = np.hstack((local_pts_hom, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            pts = []
            for i in range(world_pts.shape[0]):
                wx, wy = world_pts[i, 0], world_pts[i, 1]
                pts.append(self._to_screen(wx, wy))
            if len(pts) >= 2:
                lw = max(1, int(round(primitive.linewidth)))
                pygame_mod.draw.lines(
                    self.surface,
                    _color_to_rgb(primitive.color),
                    False,
                    pts,
                    lw,
                )

        elif isinstance(primitive, Circle):
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            local_center = np.append(local_center, 1.0)
            world_center = transform_matrix @ local_center
            x, y = world_center[0], world_center[1]
            col = _color_to_rgb(primitive.color)
            lw = max(1, int(round(primitive.linewidth)))
            scale = self._world_scale()

            if self.is_3d:
                th = np.linspace(0, 2 * np.pi, 50)
                pts_z = primitive.center[2] if len(primitive.center) > 2 else 0.0
                pts = np.vstack(
                    (
                        primitive.center[0] + primitive.radius * np.cos(th),
                        primitive.center[1] + primitive.radius * np.sin(th),
                        np.full_like(th, pts_z),
                        np.ones_like(th),
                    )
                )
                world_pts = transform_matrix @ pts
                screen_pts = [
                    self._to_screen(world_pts[0, i], world_pts[1, i])
                    for i in range(world_pts.shape[1])
                ]
                pygame_mod.draw.lines(
                    self.surface,
                    col,
                    True,
                    screen_pts,
                    lw,
                )
            else:
                cx, cy = self._to_screen(x, y)
                r_px = max(1, int(round(primitive.radius * scale)))
                if primitive.fill:
                    pygame_mod.draw.circle(self.surface, col, (cx, cy), r_px)
                else:
                    pygame_mod.draw.circle(self.surface, col, (cx, cy), r_px, lw)


class PygameRenderer(AnimationRenderer):
    """800×600 window; ESC or close to quit."""

    def __init__(self, animator):
        super().__init__(animator)
        self._size = (800, 600)

    def _paint_frame(self, screen, primitives, transforms, pygame_mod, t: float, is_3d: bool):
        pygame_mod.display.set_caption(f"{self.sys.name} — t = {t:.2f} s")
        canvas = PygameCanvas(screen, self.animator, is_3d=is_3d)
        screen.fill((250, 250, 250))
        for prim, T in zip(primitives, transforms):
            canvas.draw_primitive(prim, T, pygame_mod)
        pygame_mod.display.flip()

    def render_static(self, x, u, t: float, is_3d: bool) -> None:
        pygame = _import_pygame()
        pygame.init()

        primitives = self.sys.get_kinematic_geometry()
        transforms = self.sys.get_kinematic_transforms(x, u, t)
        if len(primitives) != len(transforms):
            raise ValueError(
                "System graphical error: Number of transforms must equal number of base geometric primitives."
            )

        screen = pygame.display.set_mode(self._size)
        self._paint_frame(screen, primitives, transforms, pygame, t, is_3d)

        clock = pygame.time.Clock()
        running = True
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False
            clock.tick(30)
        pygame.quit()

    def render_animation(
        self,
        traj,
        *,
        time_factor_video: float,
        is_3d: bool,
        save: bool,
        file_name: str,
        show: bool,
        html: bool,
    ):
        if html:
            print(
                "html=True is only supported for renderer='matplotlib'; "
                "ignoring html for pygame animation."
            )
        if save:
            print(
                f"save=True (GIF to {file_name!r}) is not supported for "
                "renderer='pygame'; skipping export."
            )

        from minilink.graphical.renderers.timing import (
            sim_index_for_frame,
            trajectory_frame_schedule,
        )

        pygame = _import_pygame()
        pygame.init()

        primitives = self.sys.get_kinematic_geometry()
        sched = trajectory_frame_schedule(traj, time_factor_video)
        n_frames = sched.n_frames
        if n_frames <= 0:
            pygame.quit()
            return None

        if not show:
            pygame.quit()
            return None

        screen = pygame.display.set_mode(self._size)
        pygame.display.set_caption(f"Animation: {self.sys.name}")
        canvas = PygameCanvas(screen, self.animator, is_3d=is_3d)
        clock = pygame.time.Clock()

        running = True
        for frame_idx in range(n_frames):
            if not running:
                break
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
                    running = False

            sim_idx = sim_index_for_frame(frame_idx, sched)
            x = traj.x[:, sim_idx]
            if len(traj.u) > 0:
                u = traj.u[:, sim_idx]
            else:
                u = np.array([])
            t = traj.t[sim_idx]

            transforms = self.sys.get_kinematic_transforms(x, u, t)
            if len(primitives) != len(transforms):
                raise ValueError(
                    "System graphical error: Number of transforms must equal number of base geometric primitives."
                )

            screen.fill((250, 250, 250))
            for prim, T in zip(primitives, transforms):
                canvas.draw_primitive(prim, T, pygame)
            pygame.display.flip()

            clock.tick(int(sched.target_fps))

        pygame.quit()
        return None
