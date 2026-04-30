"""
Tests algebraic loop detection via diagram.check_algebraic_loops() and diagram.compile().

Usage:
    python tests/manual/test_algebraic_loop.py
"""

from minilink.core.diagram import DiagramSystem
from minilink.core.system import System


class FeedthroughSystem(System):
    def __init__(self, id_str):
        super().__init__(0, 1, 1)
        self.name = id_str
        self.inputs = {}
        self.add_input_port(1, "u")
        self.outputs = {}
        self.add_output_port(1, "y", function=self.h, dependencies=["u"])

    def h(self, x, u, t=0, params=None):
        return u * 2


# Build a valid DAG (no loop)
diag = DiagramSystem()
diag.add_subsystem(FeedthroughSystem("A"), "A")
diag.add_subsystem(FeedthroughSystem("B"), "B")
diag.add_subsystem(FeedthroughSystem("C"), "C")

diag.connect("A", "y", "B", "u")
diag.connect("B", "y", "C", "u")
diag.plot_graphe()

diag.check_algebraic_loops()

# # ── Now create an actual algebraic loop ──────────────────────────
# diag.connect("C", "y", "A", "u")
# diag.plot_graphe()

# diag.check_algebraic_loops()
