import numpy as np


class GraphicPrimitive:
    """Base class for all geometric objects rendered by the animator engine."""

    def __init__(self, color="blue", linewidth=1, style="-"):
        self.color = color
        self.linewidth = linewidth
        self.style = style


class CustomLine(GraphicPrimitive):
    """A generic sequence of connected line segments."""

    def __init__(self, pts, color="blue", linewidth=1, style="-"):
        """
        Parameters
        ----------
        pts : list or np.ndarray
            Nx2 or Nx3 array of points connected sequentially.
        """
        super().__init__(color, linewidth, style)
        self.pts = np.array(pts)


class Point(GraphicPrimitive):
    """An individual point marker."""

    def __init__(self, pt=[0, 0, 0], color="red", marker="o", size=5):
        super().__init__(color)
        self.pt = np.array(pt)
        self.marker = marker
        self.size = size


class Circle(GraphicPrimitive):
    """A basic circle primitive. Lives in the XY plane by default."""

    def __init__(self, radius=1.0, center=[0, 0, 0], color="blue", fill=False):
        super().__init__(color)
        self.radius = radius
        self.center = np.array(center)
        self.fill = fill


class Rectangle(GraphicPrimitive):
    """A standard axis-aligned rectangle in local frame (XY plane by default)."""

    def __init__(
        self, width=1.0, height=1.0, center=[0, 0, 0], color="blue", fill=False
    ):
        super().__init__(color)
        self.width = width
        self.height = height
        self.center = np.array(center)
        self.fill = fill


######################################################################
# Transformation Matrix Helpers
######################################################################


def translation_matrix(dx=0.0, dy=0.0, dz=0.0):
    """
    Generate a 4x4 pure translation matrix.

    Parameters
    ----------
    dx, dy, dz : float
        Translation along the x, y, and z axes.

    Returns
    -------
    np.ndarray
        A 4x4 transformation matrix.
    """
    T = np.eye(4)
    T[0, 3] = dx
    T[1, 3] = dy
    T[2, 3] = dz
    return T


def pose2d_matrix(x=0.0, y=0.0, theta=0.0):
    """
    Generate a 4x4 transformation matrix for a 2D pose (XY plane).

    Parameters
    ----------
    x, y : float
        Translation in the XY plane.
    theta : float
        Rotation around the Z axis in radians.

    Returns
    -------
    np.ndarray
        A 4x4 transformation matrix.
    """
    T = np.eye(4)
    c, s = np.cos(theta), np.sin(theta)
    T[0, 0] = c
    T[0, 1] = -s
    T[1, 0] = s
    T[1, 1] = c
    T[0, 3] = x
    T[1, 3] = y
    return T
