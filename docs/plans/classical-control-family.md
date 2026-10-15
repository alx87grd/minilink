# The classical control family: one law per block, what is measured decides the derivative

Status: draft 2026-10-15, for the maintainer's rulings (§6). Ids: `CC-` marks this doc's
rules (`CC-R1`–`CC-R4`) and steps (`CC-1`–`CC-5`); the workboard rows are S62 (the
proportional block) and S73 (the rest).
Rung: v0.2 wave B, before TB-b, so the textbook pass runs over the final shape.

## The design in one screen

A classical law is one family. Every member is written on the tracking error, takes the
same constructor grammar (per-axis gains, `dof`, `ports`, the actuator limits), and carries
exactly the states its terms need, so its poles and zeros are the hand calculation's.

What is measured decides the derivative. A block that measures one signal per axis filters
its derivative, which puts a pole at `-1/tau`. A block that measures `[pos; rate]` reads the
rate. An unfiltered PD on one measurement is improper and has no block.

The three pieces the members share are plain functions: the integrator rate with its
anti-windup hold, the filtered-derivative rate, and the clip to the actuator limits. A
class declares its ports and its state layout and writes `f` and `ctl` as three terms and
one exception.

## 1. The code as it stands

Two families implement the same laws, and one block sits outside both.

| Block | Law | States per axis | Default ports | Module | Examples |
| --- | --- | --- | --- | --- | --- |
| `ProportionalController(K)` | `u = K e`, `K` a matrix | 0 | `reference` | `control/output.py` | 7 |
| `PI(Kp, Ki)` | `u = Kp e + Ki ∫e`, anti-windup | 1 | `error` | `control/siso.py` | 0 |
| `PD(Kp, Kd, tau)` | `u = Kp e + Kd ė`, `ė` filtered | 1 | `error` | `control/siso.py` | 0 |
| `PID(Kp, Ki, Kd, tau)` | both | 2 | `error` | `control/siso.py` | 17 |
| `ImpedanceController(Kp, Kd)` | `u = Kp (r − pos) − Kd rate` | 0 | `r`, `y = [pos; rate]` | `control/impedance.py` | 25 |
| `ImpedanceIntegralController` | the same plus `Ki ∫(r − pos)`, no anti-windup | 1 | `r`, `y = [pos; rate]` | `control/impedance.py` | 2 |
| `JointImpedance(dof)` | a factory around `ImpedanceController` with `q`, `dq` labels | 0 | as above | `control/robotic.py` | 4 |
| `Lead`, `Lag`, `TransferFunction(ports=)` | any `C(s)` | order | `error` | `blocks/transfer_function.py` | 9 |

What is wrong with it:

- `ProportionalController` is the family's `P` under another name, in another module, with
  another gain name (`K`, a matrix), another default layout (`reference`) and the display
  name "P Controller". Two tests use the matrix form; every example passes a scalar.
- `ImpedanceController` is `PD` with the rate measured; `ImpedanceIntegralController` is
  `PID` with the rate measured and the integrator written a second time, without the
  anti-windup `PID` has. The integrator, the filter and the clip live as methods of `PID`,
  so the impedance blocks can share them only by inheriting from `PID`.
- `siso.py` imports `as_dof_vector` from `impedance.py`: the generic family depends on the
  robotics one.
- `PI` and `PD` have no example.

What is right and stays: `ports="error"` / `"reference"` as the one port-layout switch
(`ErrorDriven`); the limits as parameters with infinite defaults, so the plain linear law is
the default and anti-windup is what the integrator does once a limit is declared (windup is
shown with `PID(...) >> Saturation(...)`, its cure with `PID(..., u_max=...)`); `Lead` as
the frequency-domain spelling of the filtered PD; the model-based robotic laws
(`ModelJointImpedance`, `TaskImpedance`, computed torque) and the state-feedback family,
which are not duplicates of anything here.

## 2. The target

| One measurement per axis, filtered derivative | `[pos; rate]` measured |
| --- | --- |
| `P(Kp)` | — |
| `PI(Kp, Ki)` | — |
| `PD(Kp, Kd, tau)` | `ImpedanceController(Kp, Kd)` |
| `PID(Kp, Ki, Kd, tau)` | `ImpedanceIntegralController(Kp, Ki, Kd)` |

All six in the one grammar `Block(gains..., *, dof=1, ports="error", u_min, u_max,
e_int_min, e_int_max)`; the rate-measured pair keeps `tracking_ref` as its second
port-layout switch (`r` of size `dof`, or `[pos_d; rate_d]` of size `2·dof`).

The shared pieces, plain functions under `# Internal machinery` of `control/siso.py`:

```python
def held_integral(e, e_int, u_cmd, params, xp):
    """ė_int = e, held at zero while the command or the integral state sits on a limit."""

def filtered_rate(m, m_filt, params, xp):
    """tau ṁ_filt = m − m_filt: the first-order filter whose state approximates ṁ."""

def clipped(u_cmd, params, xp):
    """The command inside the actuator limits u_min ≤ u ≤ u_max."""
```

A member then reads as the textbook page. `PID`:

```python
def f(self, x, u, t=0, params=None):
    params = self.params if params is None else params
    xp = array_module(x)
    n = self.dof
    e = self.error(u)
    e_int, m_filt = x[:n], x[n:]
    m = self.derivative_signal(u)

    # ṁ_filt from the filter, ė_int = e except when held
    dm_filt = filtered_rate(m, m_filt, params, xp)
    u_cmd = self.command(e, e_int, dm_filt, params, xp)
    de_int = held_integral(e, e_int, u_cmd, params, xp)

    return xp.concatenate([de_int, dm_filt])
```

and `ImpedanceIntegralController`:

```python
def f(self, x, u, t=0, params=None):
    params = self.params if params is None else params
    xp = array_module(x)
    e_int = x
    e, rate_error = self.errors(u)   # r − pos, and rate_d − rate (rate_d = 0 unless tracking)

    # ė_int = e except when held
    u_cmd = self.command(e, e_int, rate_error, params, xp)
    de_int = held_integral(e, e_int, u_cmd, params, xp)

    return de_int
```

`control/output.py` leaves; `control/impedance.py` keeps the two rate-measured classes and
`as_dof_vector` moves to `siso.py`, so the dependency runs robotics → generic.

## 3. Rules

- **CC-R1 One law per block.** A member carries the states its terms need and no mode
  switch in its law; `ports` and `tracking_ref` are port layouts, not laws.
- **CC-R2 What is measured decides the derivative.** One measurement: the filtered
  derivative, pole at `-1/tau`. `[pos; rate]`: the rate as measured. No unfiltered PD on one
  measurement; the docstring says why and names `Lead` as the frequency-domain twin.
- **CC-R3 The limits are parameters, infinite by default.** The plain linear law is the
  default; anti-windup is the integrator's behaviour once a limit is declared. No separate
  anti-windup classes.
- **CC-R4 The shared pieces are pure functions.** `held_integral`, `filtered_rate` and
  `clipped` are module-level functions of `siso.py`, arrays in, arrays out, called from
  each member's `f` and `ctl`; no member inherits them from another member.

- **CC-R5 The family is decoupled; a coupled gain is a `Gain` in the loop.** Every member
  runs `dof` independent loops with per-axis gains, because the integrator hold pairs each
  command with its own error and the filter is per axis; a coupled MIMO PID needs another
  anti-windup (back-calculation) and no course asks for it. A coupled static output feedback
  `u = K e` is `Gain(K) @ plant` (the loop builder inserts the Error block; same trajectory
  as today's matrix `ProportionalController`), and a coupled law on the state is
  `StateFeedbackController(K)`. `P` takes no matrix: a length-2 vector would be two axes or
  a `1×2` row.
## 4. Steps

- **CC-1 (S62) `P` joins the family.** `P(Kp, *, dof, ports="error", u_min, u_max)` in
  `siso.py`, zero states, display name "P"; `ProportionalController` and `control/output.py`
  leave with no alias; the seven example call sites and the root prelude migrate in the same
  commit; the two matrix-gain tests move to `Gain(K) @ plant` (CC-R5). **[ask — public name]**
- **CC-2 The three functions.** `held_integral`, `filtered_rate`, `clipped` extracted from
  `PID`'s methods, behaviour-preserving (seeded baseline, byte-identical); `PI`, `PD`, `PID`
  call them. `as_dof_vector` moves to `siso.py`.
- **CC-3 The rate-measured pair on the family.** `ImpedanceController` and
  `ImpedanceIntegralController` take the family's grammar and limits, call the three
  functions, and get anti-windup; `JointImpedance(dof=)` stays the robotics-named factory.
  The three `ImpedanceIntegralController` examples migrate where a keyword changed.
  **[ask — names, §6]**
- **CC-4 Evidence.** One teaching demo that shows the family side by side on the pendulum:
  `P`, `PI`, `PD`, `PID` and `ImpedanceController`, the static error of `PD`, the pole at
  `-1/tau`, windup versus its cure. The hand-calculation pole and zero test of
  `test_control_analysis.py` extends to the rate-measured pair.
- **CC-5 Docs.** DESIGN's "Control laws" paragraph gets CC-R2 in one sentence and the family
  table; the `control` facade docstring lists the six; the analysis facade's step response
  text is unchanged.

Order: CC-1, then CC-2 and CC-3 together, CC-4, CC-5; then TB-b runs its textbook pass over
the result.

## 5. Done when

`from minilink.control import P, PI, PD, PID, ImpedanceController,
ImpedanceIntegralController` share one constructor grammar; `grep "def integrator_rate\|def
filter_rate"` finds nothing and the three functions have one definition each; `pzmap` of
each member in series with the pendulum reports the hand-calculation pole count (0, 1, 1,
2, 0, 1); the seeded baseline of every existing demo that uses one of the six compares
byte-identical before and after CC-2; the family demo runs on the Basic tier.

## 6. Open for the maintainer

1. The name `P` for the proportional block (CC-1), replacing `ProportionalController`.
2. The rate-measured names: keep `ImpedanceController` and `ImpedanceIntegralController`
   (the GMC714 vocabulary, 25 examples), or rename into the family's grammar (`PDRate`,
   `PIDRate`). Proposed: keep.
3. `tracking_ref` stays the rate-measured pair's second port layout. Proposed: keep.
4. Closed 2026-10-15 by CC-R5: the matrix-gain form of today's `ProportionalController` is
   `Gain(K) @ plant`, verified to give the same trajectory.
