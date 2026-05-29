# Custom System Ports Implementation Plan

**Status:** Implemented as the explicit port API cleanup.  
**Related:** [DESIGN.md](../../DESIGN.md) §4 `System` contract, [README.md](../../README.md)
(custom block examples), and [ROADMAP.md](../../ROADMAP.md) P2 diagram ergonomics.

## Final Direction

Ports are explicit, ID-first, and metadata-aware. Base `System` and
`StaticSystem` instances start with no input or output ports. `DynamicSystem`
creates standard textbook ports only when requested through constructor options.

No `PortSpec`, `define_ports`, `port_vals`, or `port_dict` API was added. The
single extraction method is `get_port_values_from_u`.

## Public API

Declare ports by signal id first:

```python
self.add_input_port("u")
self.add_input_port("y", dim=2)
self.add_input_port("y", nominal_value=[0.0, 0.0])
self.add_input_port("y", labels=["theta", "theta_dot"], units=["rad", "rad/s"])

self.add_output_port("u", dim=1, function=self.ctl, dependencies=("r", "y"))
self.add_output_port("y", dim=2, function=self.h, dependencies=())
```

Dimension inference:

- use explicit `dim` when provided;
- otherwise infer from `nominal_value`, `labels`, `units`, `lower_bound`, or
  `upper_bound`;
- otherwise default to `dim=1`;
- validate all metadata lengths against the final dimension.

Extract named input slices with one method:

```python
signals = self.get_port_values_from_u(u)
r = self.get_port_values_from_u(u, "r")
r, y = self.get_port_values_from_u(u, "r", "y")
```

## Dimensions

- `n` remains constructor-owned state dimension metadata.
- `m` is always derived from declared input ports.
- `p` is derived from the primary output port named `"y"`.
- Auxiliary outputs such as `"x"` do not redefine `p`.
- No `"y"` output means `p == 0`; use `outputs` for multi-output blocks without
  a primary `y`.

## Standard Dynamic Ports

Textbook dynamic systems opt in to standard ports explicitly:

```python
super().__init__(
    n=2,
    input_dim=1,
    output_dim=1,
    expose_state=True,
    y_dependencies=(),
)
```

This creates:

- input `"u"`;
- primary output `"y"` calling `self.h`;
- optional state output `"x"` calling `self.compute_state`.

`StateSpaceSystem`, mechanical abstractions, physics systems, and simple
textbook examples should use these options when their public shape is naturally
`u`, `y`, and optionally `x`.

## Naming Convention

Generic control examples use:

- `"r"` for reference input;
- `"y"` for plant measurement;
- `"u"` for controller output / plant input.

Domain-specific names such as `theta_ref`, `r_ref`, and `u_ref` can stay when
they describe a physical signal rather than the generic closed-loop reference
port.

## Migration Checklist

- Remove all `u2input_signal` uses and keep the method deleted from `System`.
- Update `add_input_port` and `add_output_port` call sites to ID-first syntax.
- Replace generic `"ref"` controller ports with `"r"`.
- Update `closed_loop()` and autowire preferences to favor `"r"`.
- Review examples, manual scripts, bug repros, benchmark helpers, and unit tests
  for implicit default-port assumptions.
- Keep dependency declarations explicit; output dependencies may be `()`, a tuple
  of input port names, or `"all"`.

## Test Coverage

The cleanup should stay covered by tests for:

- metadata defaulting when `add_input_port("y", dim=2)` is used;
- dimension inference from nominal values, labels, units, and bounds;
- metadata length mismatch errors;
- primary `"y"` output updating `p`;
- auxiliary `"x"` output not changing `p`;
- systems with no `"y"` reporting `p == 0`;
- dependency validation for unknown input names;
- full, single-port, and multi-port `get_port_values_from_u` returns;
- absence of `u2input_signal`;
- explicit `DynamicSystem` standard ports;
- migrated `r`/`y`/`u` closed-loop composition and autowire behavior;
- existing compile, simulation, plotting, and pytest coverage.
