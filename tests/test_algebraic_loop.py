import numpy as np
import sys
import os

sys.path.insert(0, os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))
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

    diag.connect("A", "y", "B", "u")
    diag.connect("B", "y", "A", "u")

    try:
        diag.compile()
        print("FAIL: Expected AlgebraicLoopError, but it compiled successfully.")
    except RuntimeError as e:
        print(f"SUCCESS: Caught expected algebraic loop: {e}")


if __name__ == "__main__":
    test_algebraic_loop()
