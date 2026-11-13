"""Terrain: the ground as fields over the plane, a height, its gradient and a friction."""

import numpy as np

from minilink.core.backends import array_module
from minilink.core.params import own_params
from minilink.core.system import System

###############################################################################
# Public API
###############################################################################


class Terrain:
    """Flat, hard ground: height ``0``, gradient ``0``, friction factor ``1``.

    A terrain is a hard surface described by named fields of the position ``(x, y)``
    [m] and the time ``t`` [s]: its height ``z(x, y)``, the gradient of that height,
    and a friction factor that multiplies each tire's ``mu``: the road's grip
    relative to the reference surface (dry asphalt), ``params["friction"]``, 1 by
    default, uniform unless a subclass maps it over the ground. A subclass writes its
    gradient in closed form, the exact derivative of its height, and keeps its numbers
    in ``params``.
    """

    params = own_params()

    def __init__(self):
        # Parameters: the road's friction factor [-] (1 on the reference surface)
        self.params = {"friction": 1.0}

    def height(self, x, y, t=0.0, params=None):
        """Ground height ``z`` [m] under each point."""
        xp = array_module(x, y)

        z = xp.zeros_like(x * y)

        return z

    def gradient(self, x, y, t=0.0, params=None):
        """Gradient ``(dz_dx, dz_dy)`` [-] of the height under each point."""
        xp = array_module(x, y)

        dz_dx = xp.zeros_like(x * y)
        dz_dy = xp.zeros_like(x * y)

        return dz_dx, dz_dy

    def friction(self, x, y, t=0.0, params=None):
        """Friction factor [-] under each point: the tire's ``mu`` is multiplied by it."""
        params = self.params if params is None else params
        xp = array_module(x, y)

        friction = params["friction"]

        # Uniform over the ground
        factor = friction * xp.ones_like(x * y)

        return factor


class Slope(Terrain):
    """A plane rising along ``x`` at the angle ``slope`` [rad]; its friction factor is 1."""

    def __init__(self):
        # Parameters: the slope angle
        self.params = {"slope": 0.1, "friction": 1.0}

    def height(self, x, y, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, y)

        beta = params["slope"]

        # z = x tan β
        z = x * xp.tan(beta) * xp.ones_like(y)

        return z

    def gradient(self, x, y, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, y)

        beta = params["slope"]

        # ∂z/∂x = tan β, ∂z/∂y = 0
        dz_dx = xp.tan(beta) * xp.ones_like(x * y)
        dz_dy = xp.zeros_like(x * y)

        return dz_dx, dz_dy


class Bumps(Terrain):
    """Sinusoidal bumps along ``x`` and ``y``: an egg-crate of amplitude ``amplitude``; friction 1."""

    def __init__(self):
        # Parameters: the bumps' amplitude and wavelength
        self.params = {"amplitude": 0.05, "wavelength": 2.0, "friction": 1.0}

    def height(self, x, y, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, y)

        A = params["amplitude"]
        wavelength = params["wavelength"]

        k = 2.0 * np.pi / wavelength

        # z = A sin(k x) sin(k y)
        z = A * xp.sin(k * x) * xp.sin(k * y)

        return z

    def gradient(self, x, y, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(x, y)

        A = params["amplitude"]
        wavelength = params["wavelength"]

        k = 2.0 * np.pi / wavelength

        # ∂z/∂x = A k cos(k x) sin(k y), ∂z/∂y = A k sin(k x) cos(k y)
        dz_dx = A * k * xp.cos(k * x) * xp.sin(k * y)
        dz_dy = A * k * xp.sin(k * x) * xp.cos(k * y)

        return dz_dx, dz_dy


class TerrainBlock(System):
    """The terrain as a static block: points in, the ground's fields under them out.

    Input ``u = [x_1, ..., x_n, y_1, ..., y_n]``: the horizontal positions [m] of
    ``n_points`` points. Outputs ``height`` (``n_points`` heights [m]), ``gradient``
    (``dz_dx`` of every point, then ``dz_dy``) and ``friction`` (``n_points`` factors).
    The block's ``params`` hold the terrain's own dict at ``"terrain"``.
    """

    def __init__(self, terrain: Terrain | None = None, n_points: int = 4):
        super().__init__()

        # Parameters: the terrain's own dict, held at its key (one home)
        self.terrain = Terrain() if terrain is None else terrain
        self.n_points = int(n_points)
        self.params = {"terrain": self.terrain.params}

        # Ports
        n = self.n_points
        self.add_input_port("u", dim=2 * n)
        self.add_output_port(
            "height", dim=n, function=self.h_height, dependencies="all"
        )
        self.add_output_port(
            "gradient", dim=2 * n, function=self.h_gradient, dependencies="all"
        )
        self.add_output_port(
            "friction", dim=n, function=self.h_friction, dependencies="all"
        )

    def h_height(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        terrain = self.terrain
        terrain_params = params["terrain"]
        n = self.n_points

        p_x = u[:n]
        p_y = u[n:]

        z = terrain.height(p_x, p_y, t, terrain_params)

        return z

    def h_gradient(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params
        xp = array_module(u)

        terrain = self.terrain
        terrain_params = params["terrain"]
        n = self.n_points

        p_x = u[:n]
        p_y = u[n:]

        dz_dx, dz_dy = terrain.gradient(p_x, p_y, t, terrain_params)
        gradient = xp.concatenate([dz_dx, dz_dy])

        return gradient

    def h_friction(self, x, u, t=0.0, params=None):
        params = self.params if params is None else params

        terrain = self.terrain
        terrain_params = params["terrain"]
        n = self.n_points

        p_x = u[:n]
        p_y = u[n:]

        factor = terrain.friction(p_x, p_y, t, terrain_params)

        return factor
