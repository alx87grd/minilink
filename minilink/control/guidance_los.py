import math

import matplotlib.pyplot as plt
import numpy as np

from minilink.core.system import System
from minilink.graphical.animation.primitives import Point  # , pose2d_matrix

# ============================================================
# Utilitaires géométriques LOS
# ============================================================

_EPS = 1e-12


def wrap_pi(angle):
    """
    Ramène un angle dans [-pi, pi].
    Remplace simpleSpeedBoatSim._wrap_pi si non disponible.
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def _iter_segments(path_xy, closed: bool):
    n = len(path_xy)

    if n < 2:
        return

    if closed:
        for i in range(n):
            j = (i + 1) % n
            yield i, j
    else:
        for i in range(n - 1):
            yield i, i + 1


def _closest_point_on_segment(px, py, x0, y0, x1, y1):
    vx, vy = x1 - x0, y1 - y0
    wx, wy = px - x0, py - y0

    den = vx * vx + vy * vy

    if den <= _EPS:
        return x0, y0, 0.0

    t = (wx * vx + wy * vy) / den
    t = max(0.0, min(1.0, t))

    qx = x0 + t * vx
    qy = y0 + t * vy

    return qx, qy, t


def project_on_path_with_t(path_xy, x, y, closed=True):
    """
    Projette le point (x, y) sur une polyligne.

    Retourne:
        idx, t, qx, qy, e_perp, psi_path
    """

    best = None

    for i, j in _iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]  # P0
        x1, y1 = path_xy[j]  # P a +1

        qx, qy, t = _closest_point_on_segment(x, y, x0, y0, x1, y1)

        dx = x1 - x0
        dy = y1 - y0

        seg_len = math.hypot(dx, dy)

        if seg_len <= _EPS:
            continue

        tx = dx / seg_len
        ty = dy / seg_len

        nx = -ty
        ny = tx

        e_perp = (x - qx) * nx + (y - qy) * ny
        d2 = (x - qx) ** 2 + (y - qy) ** 2

        if best is None or d2 < best[0]:
            psi_path = math.atan2(ty, tx)
            best = (d2, i, t, qx, qy, e_perp, psi_path)

    if best is None:
        raise ValueError("Path invalide pour projection.")

    _, idx, t, qx, qy, e_perp, psi_path = best

    return idx, t, qx, qy, e_perp, psi_path


def path_total_length(path_xy, closed=True):
    L = 0.0

    for i, j in _iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        L += math.hypot(x1 - x0, y1 - y0)

    return L


def point_ahead_along_path(path_xy, idx, t, ds, closed=True):
    """
    Avance de ds mètres le long de la polyligne à partir de la projection.
    """
    # Semble refaire beaucoup de calcul from "project_on_path_with_t"

    n = len(path_xy)

    if n < 2:
        raise ValueError("Path trop court.")

    if closed:
        Lt = path_total_length(path_xy, closed=True)

        if Lt > _EPS:
            ds = ds % Lt

    i = idx % n
    j = (i + 1) % n

    x0, y0 = path_xy[i]
    x1, y1 = path_xy[j]

    dx = x1 - x0
    dy = y1 - y0

    seg_len = math.hypot(dx, dy)

    if seg_len <= _EPS:
        i = (i + 1) % n
        j = (i + 1) % n

        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        dx = x1 - x0
        dy = y1 - y0

        seg_len = math.hypot(dx, dy)

    # Point sur le vehicule projeter sur un segment du path
    cx = x0 + t * dx
    cy = y0 + t * dy

    # Distance qui reste sur le segment
    rem = (1.0 - t) * seg_len

    steps = 0
    max_steps = len(path_xy) + 2

    while ds > rem and steps < max_steps:
        # Si ds est plus grand que ce qui reste sur le segment passe au prochain
        ds -= rem

        i = (i + 1) % n
        j = (i + 1) % n

        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]

        dx = x1 - x0
        dy = y1 - y0

        seg_len = math.hypot(dx, dy)

        if seg_len <= _EPS:
            rem = 0.0
            steps += 1
            continue

        cx = x0
        cy = y0
        rem = seg_len

        steps += 1

        if not closed and i == n - 1:
            return x0, y0, i, 0.0, math.atan2(dy, dx)

    lam = 0.0 if seg_len <= _EPS else min(1.0, ds / max(_EPS, seg_len))

    ax = cx + lam * dx
    ay = cy + lam * dy
    psi_a = math.atan2(dy, dx)

    return float(ax), float(ay), int(i), float(lam), float(psi_a)


# ============================================================
# Contrôleur LOS
# ============================================================


class LOSController:
    def __init__(
        self,
        path_xy,
        Delta=4.0,
        control_point_ahead=4.0,
        closed=False,
    ):
        self.path = np.asarray(path_xy, dtype=float)

        self.Delta = float(Delta)

        self.control_point_ahead = float(control_point_ahead)
        self.closed = bool(closed)

    def compute(self, x, y, psi, u_body=None, v_body=None):
        if self.control_point_ahead != 0.0:
            x_ctrl = x + self.control_point_ahead * math.cos(psi)
            y_ctrl = y + self.control_point_ahead * math.sin(psi)
        else:
            x_ctrl = x
            y_ctrl = y

        idx, t, qx, qy, e_perp, psi_path = project_on_path_with_t(
            self.path,
            x_ctrl,
            y_ctrl,
            closed=self.closed,
        )

        ax, ay, _, _, _ = point_ahead_along_path(
            self.path,
            idx,
            t,
            self.Delta,
            closed=self.closed,
        )

        # Desired LOS angle
        chi_d = math.atan2(ay - y_ctrl, ax - x_ctrl)

        # Measured course angle
        chi_meas = psi

        if u_body is not None and v_body is not None:
            beta = math.atan2(v_body, max(1e-6, u_body))
            chi_meas = wrap_pi(psi + beta)

        # Course angle error
        err_chi = wrap_pi(chi_d - chi_meas)

        info = {
            "e_perp": e_perp,
            "chi_d": chi_d,
            "idx": idx,
            "ax": ax,
            "ay": ay,
            "x_ctrl": x_ctrl,
            "y_ctrl": y_ctrl,
            "err_chi": err_chi,
            "psi_path": psi_path,
        }

        return chi_d, info


# ============================================================
# System Minilink
# ============================================================


class Los(System):
    """
    Bloc Minilink qui wrap le contrôleur LOS.

    Sortie:
        los[0] = vx_ref
        los[1] = chi_d
    """

    def __init__(
        self,
        path_pts,
        Delta=1.0,
        control_point_ahead=0.5,
        closed=False,
    ):
        super().__init__(0)

        self.name = "Los"

        self.path_pts = np.asarray(path_pts, dtype=float)

        # On garde seulement x, y si les points sont en 3D.
        if self.path_pts.shape[1] >= 2:
            self.path_xy = self.path_pts[:, :2]
        else:
            raise ValueError("path_pts doit contenir au moins deux colonnes: x, y.")

        self.controller = LOSController(
            path_xy=self.path_xy,
            Delta=Delta,
            control_point_ahead=control_point_ahead,
            closed=closed,
        )

        self.inputs = {}
        self.add_input_port("x", nominal_value=np.array([0.0]))
        self.add_input_port("y", nominal_value=np.array([0.0]))
        self.add_input_port("psi", nominal_value=np.array([0.0]))

        self.add_output_port(
            "theta",
            dim=1,
            function=self.los,
            dependencies=["x", "y", "psi"],
        )

        self.add_output_port(
            "logs",
            dim=1,
            function=self.logs,
            dependencies=["x", "y", "psi"],
        )

    def los(self, x, u, t=0.0, params=None):
        """
        Fonction appelée par Minilink.

        Hypothèse sur l'état:
            x[0] = position x
            x[1] = position y
            x[2] = psi
            x[3] = vx, optionnel
            x[4] = vy, optionnel
            x[5] = r, optionnel
        """

        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        # vx = float(x[3]) if len(x) > 3 else None
        # vy = float(x[4]) if len(x) > 4 else None

        chi_ref, _ = self.controller.compute(
            px,
            py,
            psi,
            # vx=vx,
            # vy=vy,
        )

        return np.array([chi_ref], dtype=float)

    def logs(self, x, u, t=0.0, params=None):

        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        _, info = self.controller.compute(
            px,
            py,
            psi,
            # vx=vx,
            # vy=vy,
        )

        idx = info["idx"]
        return np.array([idx], dtype=float)

    def get_kinematic_geometry(self):
        """
        Return the graphical primitives used to display LOS geometry.

        Green point = control point
        Red point   = lookahead point
        """

        return [
            Point(
                pt=[0.0, 0.0, 0.0],
                color="orange",
                marker="o",
                size=4,
            ),
            Point(
                pt=[0.0, 0.0, 0.0],
                color="red",
                marker="o",
                size=4,
            ),
        ]

    # def get_kinematic_transforms(self, x, u, t):
    #     """
    #     Position the two LOS points in the world frame.
    #     """

    #     px = float(u[0])
    #     py = float(u[1])
    #     psi = float(u[2])

    #     _, info = self.controller.compute(px, py, psi)

    #     x_ctrl = info["x_ctrl"]
    #     y_ctrl = info["y_ctrl"]

    #     ax = info["ax"]
    #     ay = info["ay"]

    #     return [
    #         pose2d_matrix(x=x_ctrl, y=y_ctrl, theta=0.0),
    #         pose2d_matrix(x=ax, y=ay, theta=0.0),
    #     ]


# ============================================================
# Exemple de chemin
# ============================================================

_PATH_X0 = 0.0
_PATH_X1 = 0.0

pts = np.array(
    [
        [0.0, 3.0, 0.0],
        [5.0, 3.0, 0.0],
        [5.0, -_PATH_X1, 0.0],
    ]
)


def main():
    los_system = Los(
        path_pts=pts,
        Delta=1.0,
        control_point_ahead=0.5,
        closed=False,
    )

    # Exemple d'état:
    # x, y, psi, vx, vy, r
    x_state = np.array([4.0, 1.0, 0.0, 1.0, 0.0, 0.0])

    los_output = los_system.los(x_state, u=None, t=0.0, params={"dt": 0.01})

    print("Sortie LOS:")
    print(f"vx_ref  = {los_output[0]:.3f}")
    print(f"r_ref   = {los_output[1]:.3f}")
    print(f"e_perp  = {los_output[2]:.3f}")
    print(f"chi_d   = {los_output[3]:.3f}")
    print(f"ax, ay  = ({los_output[4]:.3f}, {los_output[5]:.3f})")
    print(f"x_ctrl  = {los_output[6]:.3f}")
    print(f"y_ctrl  = {los_output[7]:.3f}")

    # Visualisation simple
    plt.figure()
    plt.plot(pts[:, 0], pts[:, 1], "k-o", label="Path")

    plt.plot(x_state[0], x_state[1], "bo", label="Bateau")
    plt.plot(los_output[6], los_output[7], "go", label="Point de contrôle")
    plt.plot(los_output[4], los_output[5], "ro", label="Point lookahead")

    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.xlabel("x")
    plt.ylabel("y")
    plt.title("Géométrie LOS")
    plt.show()


if __name__ == "__main__":
    main()
