"""Tire models used by Simon's rear-wheel-drive engine bicycle project."""

import numpy as np


class TireModel:
    """Base strategy for tire-road interaction."""

    def __init__(self):
        self.v_min_epsilon = 0.1

    def vel2slip(self, vx, vy, w, R):
        """Compute longitudinal slip ratio and lateral slip angle."""
        wr = w * R
        denom = np.maximum(np.maximum(np.abs(vx), np.abs(wr)), self.v_min_epsilon)
        alpha = -np.arctan2(vy, np.maximum(np.abs(vx), self.v_min_epsilon))
        kappa = (wr - vx) / denom
        return alpha, kappa

    def slip2forces(self, alpha, kappa, Fz):
        """Convert slip values to tire forces."""
        raise NotImplementedError

    def vel2forces(self, vx, vy, w, R, Fz):
        """Compute tire forces directly from wheel velocities."""
        alpha, kappa = self.vel2slip(vx, vy, w, R)
        return self.slip2forces(alpha, kappa, Fz)


class LinearTire(TireModel):
    """Linear slip tire with friction-circle saturation."""

    def __init__(self, Ca=60000, Ck=100000, mu=1.0):
        super().__init__()
        self.Ca = Ca
        self.Ck = Ck
        self.mu = mu

    def slip2forces(self, alpha, kappa, Fz):
        Fx = self.Ck * kappa
        Fy = self.Ca * alpha

        F_max = self.mu * Fz
        F_total = np.sqrt(Fx**2 + Fy**2)
        if F_total > F_max:
            ratio = F_max / F_total
            Fx *= ratio
            Fy *= ratio

        return Fx, Fy


class Pacejka(TireModel):
    """Magic-formula tire used by the student model."""

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
        combined_slip_mode=None,
    ):
        super().__init__()
        self.combined_slip_mode = combined_slip_mode

        self.Bx, self.Cx, self.Dx, self.Ex = Bx, Cx, Dx, Ex
        self.By, self.Cy, self.Dy, self.Ey = By, Cy, Dy, Ey

        self.r_Bx1 = 1.0
        self.r_Bx2 = 1.0
        self.r_Cx1 = 1.0
        self.r_Ex1 = 1.0
        self.r_Ex2 = 1.0

        self.r_By1 = 1.0
        self.r_By2 = 1.0
        self.r_By3 = 1.0
        self.r_Cy1 = 1.0
        self.r_Ey1 = 1.0
        self.r_Ey2 = 1.0

        self.d_fz = -1.0
        self.lambda_ya = 1.0
        self.lambda_yk = 1.0
        self.mu = 1.0

    def Gxa(self, k, a):
        B = self.r_Bx1 * np.cos(np.arctan(self.r_Bx2 * k)) * self.lambda_ya
        C = self.r_Cx1
        E = self.r_Ex1 + self.r_Ex2 * self.d_fz
        ratio = np.cos(
            C
            * np.arctan(B * np.tan(a) - E * (B * np.tan(a) - np.arctan(B * np.tan(a))))
        )
        return ratio

    def Gyk(self, k, a):
        B = (
            self.r_By1
            * np.cos(np.arctan(self.r_By2 * (np.tan(a) - self.r_By3)))
            * self.lambda_yk
        )
        C = self.r_Cy1
        E = self.r_Ey1 + self.r_Ey2 * self.d_fz
        ratio = np.cos(C * np.arctan(B * k - E * (B - np.arctan(B))))
        return ratio

    def combined_slip(self, Fx_0, Fy_0, kappa, alpha, Fz, mode=None):
        """Apply Simon's optional combined-slip weighting."""
        Fx = Fx_0
        Fy = Fy_0

        if mode == "w":
            Fx *= self.Gxa(kappa, alpha)
            Fy *= self.Gyk(kappa, alpha)
        elif mode == "c":
            F_max = self.mu * Fz
            F_total = np.sqrt(Fx**2 + Fy**2)
            if F_total > F_max:
                ratio = F_max / F_total
                Fx *= ratio
                Fy *= ratio

        return Fx, Fy

    def slip2forces(self, alpha, kappa, Fz):
        def mf(x, B, C, D, E, fz):
            D_scaled = D * fz
            return D_scaled * np.sin(
                C * np.arctan(B * x - E * (B * x - np.arctan(B * x)))
            )

        Fx = mf(kappa, self.Bx, self.Cx, self.Dx, self.Ex, Fz)
        Fy = mf(alpha, self.By, self.Cy, self.Dy, self.Ey, Fz)
        Fx, Fy = self.combined_slip(
            Fx, Fy, kappa, alpha, Fz, mode=self.combined_slip_mode
        )

        return Fx, Fy
