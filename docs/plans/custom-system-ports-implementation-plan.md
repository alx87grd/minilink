# Custom System Ports

**Status:** Implemented.

Ports are explicit and ID-first. Base `System`/`StaticSystem` have no implicit
ports. `DynamicSystem` creates `u`/`y`/`x` only via constructor options
(`input_dim`, `output_dim`, `expose_state`).

Single extraction helper: `get_port_values_from_u`. Control convention: `r`, `y`,
`u`. Dimensions: `n` constructor-owned; `m` from inputs; `p` from primary `"y"`.

Full contract: [DESIGN.md](../../DESIGN.md) §4. Examples: [README.md](../../README.md).
