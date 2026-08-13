import math

import numpy as np

from minilink.core.system import System
from minilink.graphical.animation.primitives import Point, pose2d_matrix

_EPS = 1e-12


def wrap_pi(angle):
    """
    Ramène un angle dans [-pi, pi].
    Remplace simpleSpeedBoatSim._wrap_pi si non disponible.
    """
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def precompute_path_geometry(path_xy, closed=True):
    """
    Precompute segment heading, heading variation, and segment length.
    Arrays are indexed by segment start index i (segment i -> i+1).
    """
    if len(path_xy) < 2:
        raise ValueError("Path too short")

    path = np.asarray(path_xy, dtype=float)
    if closed:
        nxt = np.vstack((path[1:], path[:1]))
    else:
        nxt = path[1:]
        path = path[:-1]

    dx = nxt[:, 0] - path[:, 0]
    dy = nxt[:, 1] - path[:, 1]

    dir_path = np.arctan2(dy, dx)
    dir_path = (dir_path + math.pi) % (2.0 * math.pi) - math.pi

    if closed:
        prev = np.roll(dir_path, 1)
        dir_var_path = dir_path - prev
    else:
        dir_var_path = np.zeros_like(dir_path)
        if len(dir_path) > 1:
            dir_var_path[1:] = dir_path[1:] - dir_path[:-1]
    dir_var_path = (dir_var_path + math.pi) % (2.0 * math.pi) - math.pi

    dist_path = np.hypot(dx, dy)

    # dir_path -> direction des path (arrays)
    # dir_var_path -> variation de la direction des path (arrays)
    # dist_path -> distance des path(arrays)
    return dir_path, dir_var_path, dist_path


def _interp_linear_with_zero_outside(x, x_lut, y_lut):
    """Linear interpolation with y=0 outside LUT range."""
    return float(np.interp(float(x), x_lut, y_lut, left=0.0, right=0.0))


def compute_curve_demand_norm(
    dir_var_path, dist_path, idx, t_proj, dist_lut, gain_lut, closed=True
):
    """
    Compute weighted sum of upcoming absolute heading variation.

    The accumulation starts at the remaining distance on the current segment
    and walks forward until max(dist_lut). Curvature demand is normalized by pi/2.
    """
    dir_var_path = np.asarray(dir_var_path, dtype=float)
    dist_path = np.asarray(dist_path, dtype=float)
    dist_lut = np.asarray(dist_lut, dtype=float)
    gain_lut = np.asarray(gain_lut, dtype=float)

    n = len(dist_path)
    if n == 0:
        return 0.0
    if len(dist_lut) == 0:
        return 0.0

    i = int(idx) % n
    t_proj = float(np.clip(t_proj, 0.0, 1.0))
    max_dist = float(dist_lut[-1])

    dist_ahead = (1.0 - t_proj) * float(dist_path[i])
    curve_raw = 0.0
    steps = 0
    max_steps = n if closed else max(1, n - i)

    while dist_ahead <= max_dist and steps < max_steps:
        # Fait +1 ici donc regarde le prochain segment.
        # a chaque prochain segment on additione au poids curve_raw
        # Lorsque j'ai regarder un distance totale >= max_dist curve_raw "a le bon poids"
        i = (i + 1) % n
        steps += 1

        abs_dir_var = abs(float(dir_var_path[i]))
        # Gain selon la longueur du segment du path
        # plus il est court plus le gain est grand
        # Le gain multiplie la variation du path
        gain = _interp_linear_with_zero_outside(dist_ahead, dist_lut, gain_lut)
        curve_raw += abs_dir_var * gain

        dist_ahead += float(dist_path[i])
        if (not closed) and (i == n - 1):
            break

    return float(curve_raw / (math.pi / 2.0))


def speed_limit_from_curve_demand(curve_demand_norm, curve_demand_lut, speed_limit_lut):
    """Map normalized curve demand to speed limit via linear interpolation."""
    x = np.asarray(curve_demand_lut, dtype=float)
    y = np.asarray(speed_limit_lut, dtype=float)
    if len(x) == 0:
        return float("inf")
    # Clamp above LUT to the last value, as in MATLAB interp1(..., 'linear', last).
    return float(
        np.interp(float(curve_demand_norm), x, y, left=float(y[0]), right=float(y[-1]))
    )


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
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    qx, qy = x0 + t * vx, y0 + t * vy
    return qx, qy, t


def project_on_path_with_t(path_xy, x, y, closed=True):
    """
    Projette (x,y) sur la polyline (fermée par défaut).
    Retourne: idx, t∈[0,1], qx,qy, e_perp, psi_path
    """
    best = None
    for i, j in _iter_segments(path_xy, closed):
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        qx, qy, t = _closest_point_on_segment(x, y, x0, y0, x1, y1)
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len <= _EPS:
            continue  # ignorer segments dégénérés
        tx, ty = dx / seg_len, dy / seg_len
        nx, ny = -ty, tx
        e = (x - qx) * nx + (y - qy) * ny
        d2 = (x - qx) ** 2 + (y - qy) ** 2
        if best is None or d2 < best[0]:
            psi_path = math.atan2(ty, tx)
            best = (d2, i, t, qx, qy, e, psi_path)
    if best is None:
        raise ValueError("Path invalide pour projection (tous segments dégénérés ?)")
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
    À partir de (idx, t) sur le segment idx, avance de ds (m) le long de la polyligne.
    Traverse les segments; boucle si closed=True. Retourne (ax, ay, idx_a, t_a, psi_a).
    """
    n = len(path_xy)
    if n < 2:
        raise ValueError("Path trop court")

    # Si fermé, ramener ds modulo la longueur totale pour éviter les tours infinis:
    if closed:
        Lt = path_total_length(path_xy, closed=True)
        if Lt > _EPS:
            ds = ds % Lt

    # point initial sur le segment (idx -> idx_next)
    i = idx % n
    j = (i + 1) % n
    x0, y0 = path_xy[i]
    x1, y1 = path_xy[j]
    dx, dy = x1 - x0, y1 - y0
    seg_len = math.hypot(dx, dy)
    if seg_len <= _EPS:
        # sauter les segments nuls
        i = (i + 1) % n
        j = (i + 1) % n
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)

    cx, cy = x0 + t * dx, y0 + t * dy
    rem = (1.0 - t) * seg_len

    # avancer
    steps = 0
    max_steps = len(path_xy) + 2  # garde-fou
    while ds > rem and steps < max_steps:
        ds -= rem
        i = (i + 1) % n
        j = (i + 1) % n
        x0, y0 = path_xy[i]
        x1, y1 = path_xy[j]
        dx, dy = x1 - x0, y1 - y0
        seg_len = math.hypot(dx, dy)
        if seg_len <= _EPS:
            rem = 0.0
            steps += 1
            continue
        cx, cy = x0, y0
        rem = seg_len
        steps += 1

        if not closed and i == n - 1:
            # fin de chemin ouvert : on s'arrête au dernier point
            return x0, y0, i, 0.0, math.atan2(dy, dx)

    lam = 0.0 if seg_len <= _EPS else min(1.0, ds / max(_EPS, seg_len))
    ax, ay = cx + lam * dx, cy + lam * dy
    psi_a = math.atan2(dy, dx)
    return float(ax), float(ay), int(i), float(lam), float(psi_a)


# ===========================
#  Contrôleurs: LOS
# ===========================
class LOSController:
    """
    LOS avec lookahead Δ le long du PATH + régulateur PD sur la course,
    dimensionné pour un amortissement ζ et une pulsation ωn en tenant compte de ct_yr.
    Option: control_point_ahead = ℓ (m) pour projeter un point devant le bateau.
    """

    def __init__(
        self,
        path_xy,
        vx_nom=1.0,  # vitesse nominale (m/s)
        Delta=4.0,  # projection du le path
        ct_yr_nom=1.0,  # constante de temps de la boucle r (même ordre de grandeur que sim.ct_yr)
        control_point_ahead: float = 4.0,  # projection d'un point de contrôle devant le bateau (m)
        closed: bool = True,  # chemin fermé ou ouvert
        dist_lut=None,  # LUT distance (m) pour pondération courbure
        gain_lut=None,  # LUT gain associé à dist_lut
        curve_demand_lut=None,  # LUT demande de courbure normalisée
        speed_limit_lut=None,
    ):  # LUT limitation de vitesse (m/s)
        self.path = path_xy
        self.Delta = float(Delta)
        self.ct_yr_nom = float(ct_yr_nom)
        self.control_point_ahead = float(control_point_ahead)
        self.closed = bool(closed)

        # limites pratiques
        self.vx_nom = vx_nom
        self.r_max = math.radians(60.0)

        # Pré-calcul géométrie path pour limitation de vitesse par courbure.
        _, self._dir_var_path, self._dist_path = precompute_path_geometry(
            self.path, closed=self.closed
        )

        # Active la limitation seulement si les 4 LUT sont fournies.
        self.dist_lut = None if dist_lut is None else np.asarray(dist_lut, dtype=float)
        self.gain_lut = None if gain_lut is None else np.asarray(gain_lut, dtype=float)
        self.curve_demand_lut = (
            None
            if curve_demand_lut is None
            else np.asarray(curve_demand_lut, dtype=float)
        )
        self.speed_limit_lut = (
            None
            if speed_limit_lut is None
            else np.asarray(speed_limit_lut, dtype=float)
        )
        self.enable_curve_speed_limit = all(
            v is not None
            for v in (
                self.dist_lut,
                self.gain_lut,
                self.curve_demand_lut,
                self.speed_limit_lut,
            )
        )

    def compute(self, x, y, psi, r_meas=None, vx=None, vy=None):
        # 1) Point de contrôle (optionnel)
        if self.control_point_ahead != 0.0:
            x_ctrl = x + self.control_point_ahead * math.cos(psi)
            y_ctrl = y + self.control_point_ahead * math.sin(psi)
        else:
            x_ctrl, y_ctrl = x, y

        # 2) Projection + lookahead Δ le long du path
        idx, t, qx, qy, e_perp, psi_path = project_on_path_with_t(
            self.path, x_ctrl, y_ctrl, closed=self.closed
        )
        ax, ay, _, _, _ = point_ahead_along_path(
            self.path, idx, t, self.Delta, closed=self.closed
        )

        # 3) Course désirée (ligne de visée vers A)
        chi_d = math.atan2(ay - y_ctrl, ax - x_ctrl)

        vx_ref = self.vx_nom

        curve_demand_norm = None
        vx_limit_curve = None
        if self.enable_curve_speed_limit:
            curve_demand_norm = compute_curve_demand_norm(
                self._dir_var_path,
                self._dist_path,
                idx,
                t,
                self.dist_lut,
                self.gain_lut,
                closed=self.closed,
            )
            vx_limit_curve = speed_limit_from_curve_demand(
                curve_demand_norm, self.curve_demand_lut, self.speed_limit_lut
            )
            vx_ref = min(vx_ref, vx_limit_curve)

        info = dict(
            e_perp=e_perp,
            chi_d=chi_d,
            idx=idx,
            t=t,
            ax=ax,
            ay=ay,
            x_ctrl=x_ctrl,
            y_ctrl=y_ctrl,
            curve_demand_norm=curve_demand_norm,
            vx_limit_curve=vx_limit_curve,
        )

        return vx_ref, chi_d, info


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
        vx_nom=1.0,
        Delta=1.0,
        control_point_ahead=0.5,
        closed=False,
        dist_lut=None,  # LUT distance (m) pour pondération courbure
        gain_lut=None,  # LUT gain associé à dist_lut
        curve_demand_lut=None,  # LUT demande de courbure normalisée
        speed_limit_lut=None,
    ):
        super().__init__(0)

        self.name = "Los"

        self.path_pts = np.asarray(path_pts, dtype=float)

        # self.chi_ref = 0.0

        # On garde seulement x, y si les points sont en 3D.
        if self.path_pts.shape[1] >= 2:
            self.path_xy = self.path_pts[:, :2]
        else:
            raise ValueError("path_pts doit contenir au moins deux colonnes: x, y.")

        self.controller = LOSController(
            path_xy=self.path_xy,
            vx_nom=vx_nom,
            Delta=Delta,
            control_point_ahead=control_point_ahead,
            closed=closed,
            dist_lut=dist_lut,  # LUT distance (m) pour pondération courbure
            gain_lut=gain_lut,  # LUT gain associé à dist_lut
            curve_demand_lut=curve_demand_lut,  # LUT demande de courbure normalisée
            speed_limit_lut=speed_limit_lut,
        )

        self.inputs = {}
        self.add_input_port("x", nominal_value=np.array([0.0]))
        self.add_input_port("y", nominal_value=np.array([0.0]))
        self.add_input_port("psi", nominal_value=np.array([0.0]))

        self.add_output_port(
            "cmd",
            dim=2,
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
            x[3] = u, optionnel
            x[4] = v, optionnel
            x[5] = r, optionnel
        """

        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        u_d, chi_ref, _ = self.controller.compute(
            px,
            py,
            psi,
        )
        # self.chi_ref = chi_ref
        # print(f"LOS:{chi_ref},  {u_d}")
        return np.array([chi_ref, u_d], dtype=float)

    def logs(self, x, u, t=0.0, params=None):

        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        _, _, info = self.controller.compute(
            px,
            py,
            psi,
        )

        idx = info["chi_d"]
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

    def get_kinematic_transforms(self, x, u, t):
        """
        Position the two LOS points in the world frame.
        """

        px = float(u[0])
        py = float(u[1])
        psi = float(u[2])

        _, _, info = self.controller.compute(px, py, psi)

        x_ctrl = info["x_ctrl"]
        y_ctrl = info["y_ctrl"]

        ax = info["ax"]
        ay = info["ay"]

        return [
            pose2d_matrix(x=x_ctrl, y=y_ctrl, theta=0.0),
            pose2d_matrix(x=ax, y=ay, theta=0.0),
        ]


def main():
    pass


if __name__ == "__main__":
    main()
