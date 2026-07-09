"""Default explicit integration helpers for dynamics evaluators."""

import numpy as np


class IntegrationMixin:
    """RK4 / Euler integration on :meth:`f` and :meth:`f_p`."""

    def rk4_step(self, x, u, t, dt):
        k1 = self.f(x, u, t)
        k2 = self.f(x + 0.5 * dt * k1, u, t + 0.5 * dt)
        k3 = self.f(x + 0.5 * dt * k2, u, t + 0.5 * dt)
        k4 = self.f(x + dt * k3, u, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def euler_step(self, x, u, t, dt):
        return x + dt * self.f(x, u, t)

    def rk4_integrate_forced(self, x0, u_knots, t0, dt):
        x = np.asarray(x0).reshape(self.n)
        u = np.asarray(u_knots)
        x_samples = np.zeros((u.shape[0], self.n))
        x_samples[0] = x
        t = t0
        for k in range(u.shape[0] - 1):
            u0 = u[k]
            u1 = u[k + 1]
            umid = 0.5 * (u0 + u1)
            k1 = self.f(x, u0, t)
            k2 = self.f(x + 0.5 * dt * k1, umid, t + 0.5 * dt)
            k3 = self.f(x + 0.5 * dt * k2, umid, t + 0.5 * dt)
            k4 = self.f(x + dt * k3, u1, t + dt)
            x = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
            t = t + dt
            x_samples[k + 1] = x
        return x_samples

    def integrate(self, x0, u_sequence, t0, dt):
        u_sequence = np.asarray(u_sequence)
        if u_sequence.ndim != 2 or u_sequence.shape[1] != self.m:
            raise ValueError(f"u_sequence must have shape (N, {self.m})")

        x = np.asarray(x0).reshape(self.n)
        x_samples = np.zeros((u_sequence.shape[0] + 1, self.n))
        x_samples[0] = x

        t = t0
        for k, u_k in enumerate(u_sequence):
            x = self.rk4_step(x, u_k, t, dt)
            t = t + dt
            x_samples[k + 1] = x

        return x_samples

    def rk4_step_p(self, x, u, t, dt, params):
        k1 = self.f_p(x, u, t, params)
        k2 = self.f_p(x + 0.5 * dt * k1, u, t + 0.5 * dt, params)
        k3 = self.f_p(x + 0.5 * dt * k2, u, t + 0.5 * dt, params)
        k4 = self.f_p(x + dt * k3, u, t + dt, params)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def integrate_p(self, x0, u_sequence, t0, dt, params):
        u_sequence = np.asarray(u_sequence)
        if u_sequence.ndim != 2 or u_sequence.shape[1] != self.m:
            raise ValueError(f"u_sequence must have shape (N, {self.m})")

        x = np.asarray(x0).reshape(self.n)
        x_samples = np.zeros((u_sequence.shape[0] + 1, self.n))
        x_samples[0] = x

        t = t0
        for k, u_k in enumerate(u_sequence):
            x = self.rk4_step_p(x, u_k, t, dt, params)
            t = t + dt
            x_samples[k + 1] = x

        return x_samples

    def rk4_step_ivp(self, x, t, dt):
        k1 = self.f_ivp(x, t)
        k2 = self.f_ivp(x + 0.5 * dt * k1, t + 0.5 * dt)
        k3 = self.f_ivp(x + 0.5 * dt * k2, t + 0.5 * dt)
        k4 = self.f_ivp(x + dt * k3, t + dt)
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)

    def euler_step_ivp(self, x, t, dt):
        return self.euler_step(x, self._u_nominal, t, dt)

    def rk4_integrate_ivp(self, x0, t0, dt, n_steps):
        x_seq = [x0]
        x = x0
        t = t0
        for _ in range(n_steps):
            x = self.rk4_step_ivp(x, t, dt)
            t = t + dt
            x_seq.append(x)
        return np.asarray(x_seq)
