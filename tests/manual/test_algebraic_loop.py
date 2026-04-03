"""
Tests algebraic loop detection via diagram.check_algebraic_loops() and diagram.compile().

Usage:
    python tests/manual/test_algebraic_loop.py
"""

from minilink.core.diagram import DiagramSystem
from minilink.core.framework import System


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


def test_algebraic_loop():
    # ── Build a valid DAG (no loop) ──────────────────────────────────
    diag = DiagramSystem()
    diag.add_subsystem(FeedthroughSystem("A"), "A")
    diag.add_subsystem(FeedthroughSystem("B"), "B")
    diag.add_subsystem(FeedthroughSystem("C"), "C")

    diag.connect("A", "y", "B", "u")
    diag.connect("B", "y", "C", "u")

    try:
        evaluator = diag.compile()
        assert evaluator is not None
        print("PASS: Compiled valid DAG successfully — no algebraic loop.")
    except RuntimeError as e:
        print(f"FAIL: Unexpected algebraic loop raised: {e}")

    # ── check_algebraic_loops should return a topological order ──────
    order = diag.check_algebraic_loops()
    assert isinstance(order, list) and len(order) == 3, (
        f"Expected 3 ports in topological order, got {order}"
    )
    print(f"PASS: check_algebraic_loops returned order: {order}")

    # ── Now create an actual algebraic loop ──────────────────────────
    diag.connect("C", "y", "A", "u")

    try:
        diag.compile()
        print("FAIL: compile() should have raised RuntimeError for algebraic loop.")
    except RuntimeError as e:
        print(f"PASS: Caught expected algebraic loop: {e}")

    # check_algebraic_loops should also raise (not silently swallow)
    try:
        diag.check_algebraic_loops()
        print("FAIL: check_algebraic_loops() should have raised RuntimeError.")
    except RuntimeError as e:
        print(f"PASS: check_algebraic_loops raised: {e}")


if __name__ == "__main__":
    test_algebraic_loop()
