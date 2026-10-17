# Names of blocks and signals: one rule

Status: core contract, audit done 2026-09-13. Quick win 5 (`print(sys)`) landed
2026-09-17; wins 1, 3 and 6 (step A4) landed 2026-10-17, with one rule for the composite
names: a shortcut is named as the expression it builds (`PID @ Pendulum`,
`Step >> Integrator`, `A + B`, `feedback(G)`); wins 2 and 4 moved to P10. The "not quick" alignments are open decisions in ROADMAP §6.

**Since 2026-10-11** ([automation-by-convention.md](automation-by-convention.md) §2.2): the
role names `plant`, `controller` and `estimator` are reserved keys. In a shortcut a role name
wins over `System.id`, and a clash raises instead of taking a suffix. The text below is
amended to match.

## Problem

A block carries several names that nothing keeps consistent: its class name, its display
`name`, its optional `id`, the key a diagram gives it, and the strings built from that key
(wires, params paths, labels). Users write the variable name and must then learn and type the
key the library chose, for instance `params_distribution={"sys.mass": ...}` for a plant held
in `arm`.

## Settled (2026-09-13)

- String keys stay. Params dictionaries, gradients, distributions and saved files need string
  keys, and a key must not change when a variable is renamed.
- `id` is the explicit name of a block placed by `+` or `>>`: `arm.id = "arm"` keys the block
  `arm`, so the params dictionary, the dotted paths (`"arm.m"`) and the wires (`"arm:y"`) all
  read `arm`. In `@` the role names win: `ctl @ arm` keys the arm `plant`. Costs keep their own
  `cost.id` (cost-params plan).
- Rejected as too complicated for what they save: reading variable names from the caller's
  frame, registering blocks by attribute assignment (`loop.plant = Pendulum()`), keyword
  builders, accepting objects wherever an id is written, ports as objects in `connect`, and
  systems as JAX pytrees.

## Audit (2026-09-13)

| Where a name shows | Today |
| --- | --- |
| `print(plant)`, `print(loop)` | `<...Pendulum object at 0x...>`: no text form; the UR5 showcase prints `arm` and gets nothing useful (notebooks display the SVG diagram instead) |
| A user subclass of `DynamicSystem` | named `DynamicSystem`: block label `DynamicSystem::sys`, loop "Closed loop DynamicSystem with P Controller"; the id code already special-cases that literal name |
| Closed-loop names | `StateFeedbackController @ Pendulum` → "Closed loop Pendulum with State Feedback Controller"; `PID() @ SingleMass()` → "Closed loop of Diagram" |
| `a >> b`, `a + b`, the sampled loop's plant wrapper | all named `Diagram`, so plot titles read "Time signals for Diagram" |
| One plant in two loop kinds | keyed `sys` under `ctl @ plant`, `plant` under `ctl % dt @ plant`; the sampled path ignores `id`, and `test_hybrid.py` asserts the `plant` key |
| Separators | wires `ctl:u`, params paths `sys.mass`, diagram block labels `Name::key`, duplicate state labels `sys:x1` (the wire separator) |
| Plot labels | states under their labels (`theta`), internal signals under their key (`ctl:u`) |
| Display names | mixed styles: `P Controller`, `Single Mass Spring Damper`, `ZOHHold`, `WhiteNoise`, `MLP`; some diagram ids derive from them |
| Role keys | documented in DESIGN §4 (`ref`, `ctl`, `sys`, numeric suffix on collision, `System.id` override); `id` itself is not mentioned in the `System` docstring |

## Quick wins

Each is a few lines plus a test, no test asserts the current behavior, and none restructures
anything.

1. ~~The default `name` is the class name.~~ Landed 2026-10-17: `System.__init__` sets
   `type(self).__name__`, the id code's special case is gone.
2. One closed-loop name. The error-driven path (`PID() @ plant`) names the loop
   "Closed loop {plant} with {controller}", as the other path does.
3. ~~Informative default names where a shortcut would say `Diagram`.~~ Landed 2026-10-17 as
   the expression rule: `A >> B`, `A + B`, `C @ G`, `feedback(G)`; the sampled loop's plant
   wrapper takes the plant's name. A diagram operand still carries its own name, so
   `a >> b >> c` built with operators reads `A >> B`.
4. ~~The sampled loop's plant wrapper honors `plant.id`.~~ Replaced by the role names
   (P10): the sampled loop's plant is `plant`, as in every loop kind.
5. ~~`print(sys)` gives a short text summary: name, class, `n`, and the ports with their
   dimensions; a diagram adds its keys.~~ Landed 2026-09-17 (`System.__str__`).
6. ~~The `System` docstring documents `id` as the key override.~~ Landed 2026-10-17.

## Not quick

- One separator rule for wires, params paths, block labels and duplicate state labels.
- ~~`sys` versus `plant` for the same plant across loop kinds.~~ Settled: `plant` everywhere
  (P10's step AC-2).
- Plot labels for internal signals: port labels or keys.
- ~~A style sweep of display names.~~ Now step A6 of [TODO.md](TODO.md) (RULES 2.6): short compact
  class names, the long explicit display names dropped; diagram ids derive from names, so each
  rename migrates the notebooks in the same commit.

## Verification (for the quick wins)

- A user subclass without a `name` appears under its class name in `plot_diagram` labels, plot
  titles and closed-loop names.
- `PID() @ SingleMass()` and `StateFeedbackController @ Pendulum()` produce names of the same
  form.
- Composed diagrams' keys, params dictionaries and trajectories are byte-identical before and
  after quick wins 1–4 and 6.
