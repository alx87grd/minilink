"""Advanced autowire example for a multi-loop bicycle cascade topology.

Run from the repo root:

    python examples/scripts/diagrams/demo_advanced_autowire.py
"""

from minilink.core.system import StaticSystem, System
from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle


class DemoPathPlanner(System):
    """Small source block mirroring the bicycle cascade planner ports."""

    def __init__(self):
        super().__init__(0, 0, 1)
        self.name = "Path planner"
        self.inputs = {}
        self.outputs = {}
        self.recompute_input_properties()
        self.add_output_port(1, "u_ref", function=self.h_u_ref, dependencies=[])
        self.add_output_port(2, "path", function=self.h_path, dependencies=[])
        self.p = 3

    def h_u_ref(self, x, u, t=0.0, params=None):
        return [5.0]

    def h_path(self, x, u, t=0.0, params=None):
        return [2.0, 20.0]


class DemoTracking(StaticSystem):
    """Small tracking block with the same ports as the bicycle cascade demo."""

    def __init__(self):
        super().__init__(8, 1)
        self.name = "Tracking"
        self.inputs = {}
        self.add_input_port(2, "path", nominal_value=[2.0, 20.0])
        self.add_input_port(6, "y")
        self.outputs = {}
        self.add_output_port(
            1,
            "theta_ref",
            function=self.h_theta_ref,
            dependencies=["path", "y"],
        )

    def h_theta_ref(self, x, u, t=0.0, params=None):
        return [0.0]


class DemoHeadingLoop(StaticSystem):
    """Small heading loop with the same ports as the bicycle cascade demo."""

    def __init__(self):
        super().__init__(7, 1)
        self.name = "Heading loop"
        self.inputs = {}
        self.add_input_port(1, "theta_ref")
        self.add_input_port(6, "y")
        self.outputs = {}
        self.add_output_port(
            1,
            "r_ref",
            function=self.h_r_ref,
            dependencies=["theta_ref", "y"],
        )

    def h_r_ref(self, x, u, t=0.0, params=None):
        return [0.0]


class DemoYawRateLoop(StaticSystem):
    """Small yaw-rate loop with the same ports as the bicycle cascade demo."""

    def __init__(self):
        super().__init__(7, 1)
        self.name = "Yaw-rate loop"
        self.inputs = {}
        self.add_input_port(1, "r_ref")
        self.add_input_port(6, "y")
        self.outputs = {}
        self.add_output_port(
            1,
            "delta",
            function=self.h_delta,
            dependencies=["r_ref", "y"],
        )

    def h_delta(self, x, u, t=0.0, params=None):
        return [0.0]


class DemoVelocityLoop(StaticSystem):
    """Small velocity loop with the same ports as the bicycle cascade demo."""

    def __init__(self):
        super().__init__(7, 1)
        self.name = "Velocity PID"
        self.inputs = {}
        self.add_input_port(1, "u_ref", nominal_value=[5.0])
        self.add_input_port(6, "y")
        self.outputs = {}
        self.add_output_port(
            1,
            "w_rear",
            function=self.h_w_rear,
            dependencies=["u_ref", "y"],
        )

    def h_w_rear(self, x, u, t=0.0, params=None):
        return [0.0]


# This mirrors the topology in:
# examples/scripts/diagrams/demo_dynamic_bicycle_basic_cascade_path_tracking.py
#
# The explicit version there uses six add_subsystem calls and these ten
# connection calls:
#
# diagram.connect("planner", "u_ref", "vel_pid", "u_ref")
# diagram.connect("planner", "path", "tracking", "path")
# diagram.connect("vehicle", "y", "vel_pid", "y")
# diagram.connect("vehicle", "y", "tracking", "y")
# diagram.connect("tracking", "theta_ref", "heading_loop", "theta_ref")
# diagram.connect("vehicle", "y", "heading_loop", "y")
# diagram.connect("heading_loop", "r_ref", "yaw_rate_loop", "r_ref")
# diagram.connect("vehicle", "y", "yaw_rate_loop", "y")
# diagram.connect("vel_pid", "w_rear", "vehicle", "w_rear")
# diagram.connect("yaw_rate_loop", "delta", "vehicle", "delta")
#
# With matching named ports, autowire replaces those ten connection calls.
vehicle = DynamicBicycle()
vehicle.name = "Vehicle"
diagram = (
    DemoPathPlanner()
    + DemoTracking()
    + DemoHeadingLoop()
    + DemoYawRateLoop()
    + DemoVelocityLoop()
    + vehicle
).autowire(strict=True)
diagram.name = "Autowired bicycle cascade topology"
diagram.plot_graphe()
