import numpy as np

from minilink.core.framework import System
from minilink.core.diagram import DiagramSystem


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
    diag = DiagramSystem()
    diag.add_subsystem(FeedthroughSystem("A"), "A")
    diag.add_subsystem(FeedthroughSystem("B"), "B")
    diag.add_subsystem(FeedthroughSystem("C"), "C")

    diag.connect("A", "y", "B", "u")
    diag.connect("B", "y", "C", "u")

    diag.plot_graphe()

    try:
        diag.compile()
        print("SUCCESS: Compiled successfully with no algebraic loop.")
    except RuntimeError as e:
        print(f"FAIL: Caught unexpected algebraic loop: {e}")

    # Now create an algebraic loop

    diag.connect("C", "y", "A", "u")

    diag.plot_graphe()

    try:
        diag.compile()
        print("FAIL: Compiled successfully with algebraic loop.")
    except RuntimeError as e:
        print(f"SUCCESS: Caught expected algebraic loop: {e}")


if __name__ == "__main__":
    test_algebraic_loop()
