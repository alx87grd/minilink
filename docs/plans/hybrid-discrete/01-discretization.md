# Discretization Layer (Conversion)

Provides a clean bridge from the continuous physical world to the discrete computational world.

## `discretize` verb

**File:** `minilink/analysis/discretize.py`

```python
def discretize(
    system: DynamicSystem,
    dt: float,
    method: str = "rk4",
) -> StepSystem:
    ...
```

### Contract
- **Inputs:** Continuous `DynamicSystem`, integration time step `dt`, and numerical integration method (`euler`, `rk4`, or `zoh` for LTI exact).
- **Outputs:** A `StepSystem` representing the discretized dynamics. 
- **Math:** Computes `x⁺ = φ(x, u, k)` by integrating `f(x, u, t)` from $t = k \cdot dt$ to $t = (k+1) \cdot dt$, assuming $u$ is held constant (Zero-Order Hold). The step index `k` explicitly replaces `t`.

### Tests
- `test_discretize.py`: Ensure `euler` and `rk4` correctly match continuous integration of the parent system over a fixed `dt`.\n