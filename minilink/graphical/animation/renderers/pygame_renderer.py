"""Prototype pygame 2D animation backend (world XY projected to the window)."""

from __future__ import annotations

from itertools import product

import matplotlib.colors as mcolors
import numpy as np

from minilink.graphical.animation.primitives import (
    Arrow,
    Box,
    Circle,
    CustomLine,
    ExtrudedPolygon,
    Plane,
    Point,
    Rod,
    Sphere,
    TorqueArrow,
    world_to_camera,
)
from minilink.graphical.animation.renderers.renderer import AnimationRenderer


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
    """Maps primitives to pygame draws (camera-frame X/Y → screen, Y flipped).

    Body transforms are expected to already be expressed in the camera frame
    (the pygame renderer pre-multiplies them by ``world_to_camera(camera)``);
    this class then maps the camera-frame square ``[-scale, +scale]^2`` to the
    visible window.
    """

    def __init__(self, surface, scale: float, is_3d: bool):
        self.surface = surface
        self.is_3d = is_3d
        self.margin = 40
        self.w, self.h = surface.get_size()
        self._xmin, self._xmax = -float(scale), float(scale)
        self._ymin, self._ymax = -float(scale), float(scale)

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

        elif isinstance(primitive, (Arrow, TorqueArrow)):
            # Honest arrow / torque arc: baked ``pts`` drawn as a polyline at its
            # pose (no column-norm scale, no ``T[3, 3]`` channel).
            local_pts = primitive.pts
            local_pts_hom = np.hstack((local_pts, np.ones((local_pts.shape[0], 1))))
            world_pts = (transform_matrix @ local_pts_hom.T).T
            pts = [
                self._to_screen(world_pts[i, 0], world_pts[i, 1])
                for i in range(world_pts.shape[0])
            ]
            if len(pts) >= 2:
                lw = max(1, int(round(primitive.linewidth)))
                pygame_mod.draw.lines(
                    self.surface, _color_to_rgb(primitive.color), False, pts, lw
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

        elif isinstance(primitive, Sphere):
            local_center = np.zeros(3)
            local_center[: len(primitive.center)] = primitive.center
            world_center = transform_matrix @ np.append(local_center, 1.0)
            cx, cy = self._to_screen(world_center[0], world_center[1])
            r_px = max(1, int(round(float(primitive.radius) * self._world_scale())))
            pygame_mod.draw.circle(
                self.surface,
                _color_to_rgb(primitive.color),
                (cx, cy),
                r_px,
            )

        elif isinstance(primitive, Plane):
            n = np.asarray(primitive.normal, dtype=float)
            off = float(primitive.offset)
            half = 0.5 * float(primitive.size)
            col = _color_to_rgb(primitive.color)
            lw = max(1, int(round(max(primitive.linewidth, 2))))
            if abs(n[1]) > 1e-9:
                x0, x1 = -half, half
                y0 = (off - n[0] * x0) / n[1]
                y1 = (off - n[0] * x1) / n[1]
                p0 = self._to_screen(x0, y0)
                p1 = self._to_screen(x1, y1)
            else:
                x = off / (n[0] + 1e-12)
                p0 = self._to_screen(x, -half)
                p1 = self._to_screen(x, half)
            pygame_mod.draw.line(self.surface, col, p0, p1, lw)

        elif isinstance(primitive, Rod):
            local = np.array([[0.0, 0.0, 0.0], [0.0, -primitive.length, 0.0]])
            local_h = np.hstack((local, np.ones((2, 1))))
            world = (transform_matrix @ local_h.T).T
            p0 = self._to_screen(world[0, 0], world[0, 1])
            p1 = self._to_screen(world[1, 0], world[1, 1])
            lw = max(1, int(round(max(primitive.linewidth, primitive.radius * 10.0))))
            pygame_mod.draw.line(
                self.surface, _color_to_rgb(primitive.color), p0, p1, lw
            )

        elif isinstance(primitive, Box):
            lx, ly, lz = primitive.length_x, primitive.length_y, primitive.length_z
            c = np.asarray(primitive.center, dtype=float).reshape(3)
            corners = []
            for sx, sy, sz in product((-1.0, 1.0), repeat=3):
                corners.append(
                    np.array([sx * lx / 2, sy * ly / 2, sz * lz / 2], dtype=float) + c
                )
            corners_h = np.hstack((np.array(corners), np.ones((8, 1))))
            world_c = (transform_matrix @ corners_h.T).T[:, :3]
            edges = (
                (0, 1),
                (0, 2),
                (0, 4),
                (1, 3),
                (1, 5),
                (2, 3),
                (2, 6),
                (3, 7),
                (4, 5),
                (4, 6),
                (5, 7),
                (6, 7),
            )
            col = _color_to_rgb(primitive.color)
            lw = max(1, int(round(primitive.linewidth)))
            for i, j in edges:
                p0 = self._to_screen(world_c[i, 0], world_c[i, 1])
                p1 = self._to_screen(world_c[j, 0], world_c[j, 1])
                pygame_mod.draw.line(self.surface, col, p0, p1, lw)

        elif isinstance(primitive, ExtrudedPolygon):
            vertices = primitive.vertices_local()
            vertices_h = np.hstack((vertices, np.ones((vertices.shape[0], 1))))
            world_v = (transform_matrix @ vertices_h.T).T[:, :3]
            col = _color_to_rgb(primitive.color)
            for i, j in primitive.edges():
                p0 = self._to_screen(world_v[i, 0], world_v[i, 1])
                p1 = self._to_screen(world_v[j, 0], world_v[j, 1])
                pygame_mod.draw.line(self.surface, col, p0, p1, 1)


class PygameRenderer(AnimationRenderer):
    """800×600 window; ESC or close to quit."""

    def __init__(self, animator):
        super().__init__(animator)
        self._size = (800, 600)
        self.pygame = None
        self.screen = None
        self.is_3d = False
        self.show = True

    def _paint_frame(self, primitives, transforms, t: float, camera):
        pygame_mod = self.pygame
        screen = self.screen
        pygame_mod.display.set_caption(f"{self.sys.name} — t = {t:.2f} s")
        canvas = PygameCanvas(screen, scale=float(camera[3, 3]), is_3d=self.is_3d)
        screen.fill((250, 250, 250))
        W = world_to_camera(camera)
        for prim, T in zip(primitives, transforms):
            canvas.draw_primitive(prim, W @ T, pygame_mod)
        pygame_mod.display.flip()

    def open_scene(
        self,
        *,
        is_3d: bool,
        show: bool,
        camera,
        title: str | None = None,
    ) -> None:
        self.pygame = _import_pygame()
        self.pygame.init()
        self.is_3d = is_3d
        self.show = show
        if show:
            self.screen = self.pygame.display.set_mode(self._size)
            if title:
                self.pygame.display.set_caption(title)

    def draw_frame(self, primitives, transforms, t: float, camera) -> None:
        if not self.show or self.screen is None:
            return
        self._paint_frame(primitives, transforms, t, camera)

    def present(self, *, block: bool, interval_s: float | None = None) -> None:
        if not self.show:
            return
        if block:
            clock = self.pygame.time.Clock()
            while True:
                events = self.poll_events()
                if events.get("quit", False):
                    break
                clock.tick(30)
        elif interval_s is not None:
            fps = max(1, int(round(1.0 / max(interval_s, 1e-3))))
            self.pygame.time.Clock().tick(fps)

    def poll_events(self) -> dict[str, bool]:
        if self.pygame is None:
            return {"quit": True}
        for event in self.pygame.event.get():
            if event.type == self.pygame.QUIT:
                return {"quit": True}
            if event.type == self.pygame.KEYDOWN and event.key == self.pygame.K_ESCAPE:
                return {"quit": True}
        return {"quit": False}

    def close_scene(self) -> None:
        if self.pygame is not None:
            self.pygame.quit()
        self.pygame = None
        self.screen = None
