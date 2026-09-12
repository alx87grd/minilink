# Control Barrier Functions (CBF) & Spatial Safety Filters

**Status:** Design draft (2026-09-11).  
**Lane:** Research lane $\to$ provisional (`minilink.control.cbf`, `minilink.planning.spatial`).  
**Audience:** Graduate research and GRO860 advanced topics (safe control, MPC, RL deployment).

---

## 1. Motivation: The "Third Way" for Obstacle Avoidance

When navigating dynamical systems around spatial obstacles, control engineers typically face a dilemma:
1. **Soft penalty costs** (e.g., $J_{\mathrm{obs}} = w \exp(-d/\epsilon)$): Easy to optimize, but offer **no hard safety guarantees**. High speeds or steep gradients can cause the system to penetrate obstacles.
2. **Hard NLP constraints** ($h(x_k) \ge 0$ in trajectory optimization or MPC): Guarantee collision avoidance across the planning horizon, but suffer from **finite-horizon recursive infeasibility**—a vehicle traveling at high speed may find an open path up to step $N$, but inertia makes stopping before step $N+1$ dynamically impossible, crashing the NLP solver.
3. **Control Barrier Functions (CBF)**: The **third way**. Rather than planning a full trajectory, a CBF acts as a forward-invariant filter that minimally modifies a nominal control $u_{\mathrm{nom}}$ (from RL, MPC, or a human operator) via a lightweight Quadratic Program (QP) at each time step.

$$\min_{u, \omega} \frac{1}{2} \|u - u_{\mathrm{nom}}\|^2 + \frac{1}{2} w_\omega \omega^2 \quad \text{s.t.} \quad \Delta h(x, u) \ge -\gamma h(x) - \omega, \quad u_{\min} \le u \le u_{\max}, \quad \omega \ge 0$$

---

## 2. Mathematical Formulation

### 2.1 Discrete-Time CBF (DCBF) with Slack Variables
In sampled and digital implementations with control step $\Delta t$, continuous condition $\dot{h}(x, u) \ge -\alpha(h(x))$ becomes a discrete step condition:

$$h(x_{k+1}) \ge (1 - \gamma) h(x_k) - \omega_k$$

where:
- $\gamma \in (0, 1]$ governs the rate of approach to the barrier boundary $\{x : h(x) = 0\}$.
- $\omega_k \ge 0$ is a **critical slack variable**. Without $\omega_k$, actuator saturation ($u \in [u_{\min}, u_{\max}]$) or sudden disturbance makes the QP instantly infeasible. Penalizing $\omega_k^2$ heavily in the objective ensures the solver remains strictly feasible while signaling safety boundary violation.

### 2.2 $C^1$-Continuous Signed Distance Fields (SDF) in JAX
A standard discretized obstacle grid has discontinuous gradients across voxel/pixel boundaries, causing chattering or gradient collapse in optimization.
- By using **bicubic interpolation** (`order=3` spline interpolation) on 2D/3D regular grids in JAX (`jax.scipy.ndimage.map_coordinates`), the obstacle barrier $h(p) = \mathrm{SDF}(p)$ becomes **$C^1$-continuous everywhere**.
- The gradient $\nabla_p h(p)$ is analytically smooth and non-zero outside obstacles, pointing directly away from the nearest collision surface.

### 2.3 Chain-Rule Propagation to Control Inputs
For a system $\dot{x} = f(x, u)$ with position map $p = \phi(x)$:

$$\frac{\partial h}{\partial u} = \nabla_p h(p) \cdot \frac{\partial \phi}{\partial x} \cdot \frac{\partial f}{\partial u}$$

Minilink's JAX compile backend (`sys.jacobian()`) propagates these derivatives automatically from the digital map's spatial field all the way back to actuator inputs (e.g. steering angle, motor torque).

### 2.4 High-Order CBF (HOCBF) for Relative Degree $r > 1$
When $u$ controls acceleration (e.g., dynamic vehicle models, torque-driven manipulators), the spatial barrier $h(x)$ has **relative degree $r = 2$** ($\frac{\partial \dot{h}}{\partial u} = 0$). A standard CBF cannot directly constrain $u$.
- Define the higher-order barrier sequence:
  $$\psi_0(x) = h(x)$$
  $$\psi_1(x) = \dot{\psi}_0(x) + \alpha_1(\psi_0(x)) = \nabla h(p) \dot{p} + \alpha_1(h(x))$$
  $$\dot{\psi}_1(x, u) \ge -\alpha_2(\psi_1(x))$$
- In discrete time, this defines a dynamic braking curve that prevents the vehicle from entering a state where its kinetic energy exceeds its braking capacity.

---

## 3. Architecture in Minilink

```
+-------------------------------------------------------------+
|                      DiagramSystem                          |
|                                                             |
|  +--------------------+             +--------------------+  |
|  | Nominal Controller |             |   CBFSafetyFilter  |  |
|  |  (RL / MPC / LQR)  |--- u_nom -->|      (QP solve)    |--|---> u_safe
|  +--------------------+             +--------------------+  |
|            ^                                   ^            |
|            |                                   |            |
+------------|-----------------------------------|------------+
             |                                   |
             +------------------ x --------------+
```

1. **Geometry / Spatial layer** (`minilink.planning.spatial`):
   - `GridSDF`: Holds 2D/3D signed distance grid with JAX bicubic interpolation.
   - `Scene.as_cbf(body_points)`: Returns an obstacle barrier function $h(x)$ evaluateable under NumPy and JAX.
2. **Control layer** (`minilink.control.cbf`):
   - `CBFSafetyFilter(controller, cbf, bounds, gamma=0.1, slack_penalty=1e5)`: Wraps an existing controller or stands as an independent filter block in a `DiagramSystem`.
   - Per step: Solves a 1D–4D QP (OSQP, Clarabel, or an analytical projection for SISO).
3. **Analysis layer** (`minilink.analysis`):
   - Forward-invariance verification tool comparing safe sets against simulated rollouts.

---

## 4. Work Breakdown & Implementation Steps

| Step | Scope | Description |
| --- | --- | --- |
| **C1** | `spatial/sdf.py` | Add `BicubicGridSDF` using `jax.scipy.ndimage.map_coordinates(..., order=3)`. Test gradient continuity against analytic primitives. |
| **C2** | `planning/spatial/scene.py` | Expose `scene.barrier_function(body_points)` returning $h(x)$ and $\nabla_x h(x)$. |
| **C3** | `control/cbf.py` | Implement `DiscreteCBFFilter` with QP solve and slack variable $\omega_k$. |
| **C4** | `examples/demos/control/` | Canonical demo: `car_circuit_cbf_safety.py` — an aggressive or random nominal control law filtered in real time around obstacles. |
| **C5** | HOCBF extension | Relative-degree 2 velocity-braking boundary for dynamic vehicles. |
