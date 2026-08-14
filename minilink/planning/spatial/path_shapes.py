import math

import matplotlib.pyplot as plt
import numpy as np


def make_rectangle_path(Lx=40.0, Ly=20.0, closed=True):
    path = [
        (Lx, Ly),  # top-right
        (-Lx, Ly),  # top-left
        (-Lx, -Ly),  # bottom-left
        (Lx, -Ly),  # bottom-right
    ]

    # Close the rectangle
    if closed:
        path.append(path[0])

    path = np.array(path, dtype=float)

    return path


def make_rounded_rectangle_from_path(
    rect_path,
    R=5.0,
    nseg=8,
    narc=12,
    min_ds=0.10,
    closed=True,
):
    """
    Create a rounded rectangle from a raw rectangle path.

    rect_path must contain the 4 rectangle corners:
    [
        [ Lx,  Ly],
        [-Lx,  Ly],
        [-Lx, -Ly],
        [ Lx, -Ly],
    ]
    """

    x = rect_path[:, 0]
    y = rect_path[:, 1]

    # Recover rectangle dimensions from the raw path
    Lx = np.max(np.abs(x))
    Ly = np.max(np.abs(y))

    # Reuse the original rounded rectangle generator
    return make_rounded_rectangle_path(
        Lx=Lx,
        Ly=Ly,
        R=R,
        nseg=nseg,
        narc=narc,
        min_ds=min_ds,
        closed=closed,
    )


def make_rounded_rectangle_path(
    Lx=40.0, Ly=20.0, R=5.0, nseg=8, narc=12, min_ds=0.10, closed=True
):
    """
    Chemin rectangulaire avec coins arrondis (sens anti-horaire), SANS doublons.
    - Lx, Ly : demi-longueur / demi-largeur (m)
    - R : rayon des coins (m)
    - nseg : nb de points par segment droit (>=2 recommandé)
    - narc : nb de points par quart de cercle (>=3 recommandé)
    - min_ds : distance minimale entre points successifs (m)
    - closed : si True, le chemin est pensé fermé (mais on NE rajoute pas
               le premier point en fin de liste pour éviter le doublon)
    Retourne : liste [(x,y), ...]
    """
    eps = 1e-12
    nseg = max(2, int(nseg))
    narc = max(2, int(narc))
    min_ds = float(min_ds)

    # Centres des coins
    c_TR = (Lx - R, Ly - R)  # top-right
    c_TL = (-(Lx - R), Ly - R)  # top-left
    c_BL = (-(Lx - R), -(Ly - R))  # bottom-left
    c_BR = (Lx - R, -(Ly - R))  # bottom-right

    def add_seq(path, pts):
        """Ajoute les points en respectant min_ds (et sans doublon local)."""
        for x, y in pts:
            if not path:
                path.append((float(x), float(y)))
            else:
                lx, ly = path[-1]
                if math.hypot(x - lx, y - ly) >= max(min_ds - eps, 0.0):
                    path.append((float(x), float(y)))
        return path

    def arc_pts(cx, cy, a0_deg, a1_deg, n):
        thetas = np.linspace(
            math.radians(a0_deg), math.radians(a1_deg), n, endpoint=True
        )
        return [(cx + R * math.cos(t), cy + R * math.sin(t)) for t in thetas]

    path = []

    # 1) Segment haut : x= Lx-R -> -(Lx-R) ; y = +Ly
    xs = np.linspace(Lx - R, -(Lx - R), nseg, endpoint=True)
    seq = [(x, Ly) for x in xs]
    add_seq(path, seq)

    # 2) Arc haut-gauche : 90° -> 180°
    add_seq(path, arc_pts(*c_TL, 90, 180, narc))

    # 3) Segment gauche : y= +Ly-R -> -(Ly-R) ; x = -Lx
    ys = np.linspace(Ly - R, -(Ly - R), nseg, endpoint=True)
    seq = [(-Lx, y) for y in ys]
    add_seq(path, seq)

    # 4) Arc bas-gauche : 180° -> 270°
    add_seq(path, arc_pts(*c_BL, 180, 270, narc))

    # 5) Segment bas : x= -(Lx-R) -> (Lx-R) ; y = -Ly
    xs = np.linspace(-(Lx - R), (Lx - R), nseg, endpoint=True)
    seq = [(x, -Ly) for x in xs]
    add_seq(path, seq)

    # 6) Arc bas-droite : 270° -> 360°
    add_seq(path, arc_pts(*c_BR, 270, 360, narc))

    # 7) Segment droit : y= -(Ly-R) -> (Ly-R) ; x = +Lx
    ys = np.linspace(-(Ly - R), (Ly - R), nseg, endpoint=True)
    seq = [(Lx, y) for y in ys]
    add_seq(path, seq)

    # 8) Arc haut-droite : 0° -> 90°
    add_seq(path, arc_pts(*c_TR, 0, 90, narc))

    # TODO: EST-ce que la logique est inversée ?
    # if path and closed:

    # Nettoyage final : si le dernier ≈ premier, on l'enlève (pas de doublon de fermeture)
    if path and not closed:
        x0, y0 = path[0]
        xN, yN = path[-1]
        if math.hypot(xN - x0, yN - y0) < max(min_ds - eps, 0.0):
            path.pop()

    path = np.array(path, dtype=float)
    # path_3d = np.column_stack((path, np.zeros(len(path))))

    return path


def main():
    # path_original = make_rounded_rectangle_path(
    #     Lx=40.0, Ly=20.0, R=5.0, nseg=4, narc=2, min_ds=0.10, closed=True
    # )

    # Raw rectangle
    path_raw = make_rectangle_path(Lx=40.0, Ly=20.0)

    # Rounded rectangle generated FROM the raw rectangle
    path = make_rounded_rectangle_from_path(
        path_raw,
        R=5.0,
        nseg=1,
        narc=2,
        min_ds=0.10,
        closed=True,
    )

    plt.figure(figsize=(10, 5))

    plt.plot(path[:, 0], path[:, 1], "-o", markersize=3)
    # plt.plot(
    #     path_raw[:, 0],
    #     path_raw[:, 1],
    #     "-o",
    #     color="gray",
    #     linewidth=2,
    # )

    plt.axis("equal")
    plt.grid(True)

    plt.title("Rounded Rectangle From Rectangle Path")
    plt.xlabel("x")
    plt.ylabel("y")

    plt.show()


if __name__ == "__main__":
    main()
