import matplotlib.pyplot as plt
import numpy as np

from minilink.compile.jax_utils import require_jax_numpy


class TireModel:
    """Base Strategy for Tire-Road Interaction"""

    def __init__(self, logs=False):
        self.v_min_epsilon = 0.1

        self.logs = logs
        self.kappa = []
        self.alpha = []

    def vel2slip(self, vx, vy, w, R):
        """Compute longitudinal and lateral slip"""

        # TODO: VERIF Changement ici

        # Adjusted longitudinal velocity to avoid division by zero
        # vx_adj = np.abs(vx) + self.v_min_epsilon

        wr = w * R
        # robust denominator (physically meaningful)
        denom = np.maximum(np.maximum(np.abs(vx), np.abs(wr)), self.v_min_epsilon)

        # Lateral slip angle (alpha)
        # alpha = -np.arctan(vy / vx_adj)
        # alpha = -np.arctan2(vy, vx_adj)

        alpha = -np.arctan2(vy, np.maximum(np.abs(vx), self.v_min_epsilon))

        # Longitudinal slip ratio (kappa)
        # kappa = (w * R - vx) / vx_adj
        kappa = (wr - vx) / denom

        if self.logs:
            if np.abs(kappa) > 1.0:
                print(
                    f"WARNING: |kappa| > 1 -> vx={vx:.2f}, wr={wr:.2f}, w={w:.2f}, kappa={kappa:.4f}"
                )

        return alpha, kappa

    def slip2forces(self, alpha, kappa, Fz):
        """Convert slip values to forces using the tire model"""
        raise NotImplementedError

    def vel2forces(self, vx, vy, w, R, Fz):
        """Compute forces directly from velocities"""
        alpha, kappa = self.vel2slip(vx, vy, w, R)
        return self.slip2forces(alpha, kappa, Fz)


class LinearTire(TireModel):
    def __init__(self, Ca=60000, Ck=100000, mu=1.0):

        TireModel.__init__(self)

        self.Ca, self.Ck = Ca, Ck
        self.mu = mu

    def slip2forces(self, alpha, kappa, Fz):
        # Forces brutes
        Fx = self.Ck * kappa
        Fy = self.Ca * alpha

        # Saturation circulaire simple (Friction circle)
        F_max = self.mu * Fz
        F_total = np.sqrt(Fx**2 + Fy**2)

        if F_total > F_max:
            ratio = F_max / F_total
            Fx *= ratio
            Fy *= ratio

        return Fx, Fy


class Pacejka(TireModel):
    """Magic Formula'"""

    def __init__(
        self,
        Bx=10.0,
        Cx=1.3,
        Dx=1.0,
        Ex=0.97,
        By=10.0,
        Cy=1.3,
        Dy=1.0,
        Ey=0.97,
        logs=False,
    ):
        super().__init__(logs)
        self.Bx, self.Cx, self.Dx, self.Ex = Bx, Cx, Dx, Ex
        self.By, self.Cy, self.Dy, self.Ey = By, Cy, Dy, Ey

        # Pour cosine weighting function
        # Pour Longitudinal Force Coefficients at Combined Slip
        self.r_Bx1 = 1.0
        self.r_Bx2 = 1.0

        self.r_Cx1 = 1.0

        self.r_Ex1 = 1.0
        self.r_Ex2 = 1.0

        # Pour Lateral Force Coefficients at Combined Slip
        self.r_By1 = 1.0
        self.r_By2 = 1.0
        self.r_By3 = 1.0

        self.r_Cy1 = 1.0

        self.r_Ey1 = 1.0
        self.r_Ey2 = 1.0

        # Combined slip constants
        self.d_fz = 0.0
        self.lambda_ya = 1.0
        self.lambda_yk = 1.0

        # Pour cosine weighting function
        self.mu = 1.0

        # Logs
        self.Fx_log = []
        self.Fz = 1.0

    def Gxa(self, a):
        B = self.r_Bx1 * np.cos(np.arctan(self.r_Bx2)) * self.lambda_ya
        C = self.r_Cx1
        E = self.r_Ex1 + self.r_Ex2 * self.d_fz
        ratio = np.cos(
            C
            * np.arctan(B * np.tan(a) - E * (B * np.tan(a) - np.arctan(B * np.tan(a))))
        )
        return ratio

    def Gyk(self, k):
        B = (
            self.r_By1
            * np.cos(np.arctan(self.r_By2 * (np.tan(k) - self.r_By3)))
            * self.lambda_yk
        )
        C = self.r_Cy1
        E = self.r_Ey1 + self.r_Ey2 * self.d_fz
        ratio = np.cos(C * np.arctan(B * k - E * (B - np.arctan(B))))
        return ratio

    def combined_slip(self, Fx_0, Fy_0, kappa, alpha, Fz, mode=None):
        """
        Combined slip mode:
        mode = None -> No combined slip
        mode = w -> Cosine weighting function
        mode = c -> Friction circle
        """

        Fx = Fx_0
        Fy = Fy_0

        if mode == "w":
            # TODO: Verif
            Fx = Fx_0 * self.Gxa(alpha)
            Fy = Fy_0 * self.Gyk(kappa)
        elif mode == "c":
            # Saturation circulaire simple (Friction circle)
            F_max = self.mu * Fz
            F_total = np.sqrt(Fx**2 + Fy**2)

            if F_total > F_max:
                ratio = F_max / F_total
                Fx *= ratio
                Fy *= ratio

        return Fx, Fy

    def slip2forces(self, alpha, kappa, Fz):
        # Fonction magique
        def mf(x, B, C, D, E, fz):
            D_scaled = D * fz
            return D_scaled * np.sin(
                C * np.arctan(B * x - E * (B * x - np.arctan(B * x)))
            )

        Fx = mf(kappa, self.Bx, self.Cx, self.Dx, self.Ex, Fz)
        Fy = mf(alpha, self.By, self.Cy, self.Dy, self.Ey, Fz)

        Fx, Fy = self.combined_slip(Fx, Fy, kappa, alpha, Fz, mode=None)
        # Logs
        if self.logs:
            self.kappa.append(kappa)
            self.alpha.append(alpha)
            self.Fx_log.append(Fx)
            self.Fz = Fz

        return Fx, Fy


def main():

    # -----------------------------
    # Parameters
    # -----------------------------
    mu = 1.0
    Fz = 3000.0  # example normal load [N]

    # Example raw tire forces before saturation
    Fx_raw = 2800.0
    Fy_raw = 2200.0

    # -----------------------------
    # Friction circle saturation
    # -----------------------------
    F_max = mu * Fz
    F_total = np.sqrt(Fx_raw**2 + Fy_raw**2)

    Fx_sat = Fx_raw
    Fy_sat = Fy_raw

    if F_total > F_max:
        ratio = F_max / F_total
        Fx_sat *= ratio
        Fy_sat *= ratio

    # -----------------------------
    # Circle points
    # -----------------------------
    theta = np.linspace(0, 2 * np.pi, 400)
    Fx_circle = F_max * np.cos(theta)
    Fy_circle = F_max * np.sin(theta)

    # -----------------------------
    # Plot
    # -----------------------------
    plt.figure(figsize=(7, 7))

    # Friction circle
    plt.plot(
        Fx_circle, Fy_circle, label=r"Friction circle: $F_x^2 + F_y^2 = (\mu F_z)^2$"
    )

    # Axes
    plt.axhline(0, color="black", linewidth=0.8)
    plt.axvline(0, color="black", linewidth=0.8)

    # Raw force vector
    plt.plot([0, Fx_raw], [0, Fy_raw], "--", color="red", alpha=0.7)
    plt.scatter(Fx_raw, Fy_raw, color="red", s=80, label="Raw force")

    # Saturated force vector
    plt.plot([0, Fx_sat], [0, Fy_sat], "-", color="green", alpha=0.8)
    plt.scatter(Fx_sat, Fy_sat, color="green", s=80, label="Saturated force")

    # Optional projection line
    plt.plot([Fx_raw, Fx_sat], [Fy_raw, Fy_sat], ":", color="gray", alpha=0.8)

    plt.gca().set_aspect("equal", adjustable="box")
    plt.xlim(-1.2 * F_max, 1.2 * F_max)
    plt.ylim(-1.2 * F_max, 1.2 * F_max)

    plt.xlabel(r"$F_x$ [N]")
    plt.ylabel(r"$F_y$ [N]")
    plt.title("Friction Circle")
    plt.grid(True)
    plt.legend()
    plt.show()


if __name__ == "__main__":
    main()
