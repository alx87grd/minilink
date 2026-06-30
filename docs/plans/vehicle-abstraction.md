# Vehicle abstraction (design only)

Status: draft plan (June 2026). No implementation in this phase.

## Scope

Flat-state vehicles (e.g. `DynamicBicycle`, rate-input models) do not subclass
`MechanicalSystem`. This doc defines optional **view ports** so the same control
**feedback profiles** in [DESIGN.md §4](../../DESIGN.md#control-feedback-profiles) apply
without renaming dynamics shelves.

## View ports (proposed)

| Port | Role | Example |
| --- | --- | --- |
| `x` | Full flat state | `[x, y, psi, u, …]` |
| `pose` | Configuration slice | position + heading |
| `bodyvel` | Body-frame velocity | `[u, v, r]` or subset |
| `u` | Actuator command | steering, throttle |

## Control profile mapping

| Profile | Vehicle wiring |
| --- | --- |
| `state` | LQR / MPC on `x` |
| `impedance` | Mux(`pose`, `bodyvel`) or stacked `y` when dims match |
| `siso` | Per-channel loops on scalar speed / heading (cascade demos) |
| `modelbased` | Not used until a vehicle EoM shelf exists |

## Non-goals (v1)

- No `MechanicalSystem` rebase for vehicles.
- No new catalog base class until a second flat vehicle shares the same port layout.

## Follow-on

When implementing: add view ports on `DynamicBicycle` behind this contract, reuse
`ImpedanceController` / `FilteredController` with vehicle-specific labels.
