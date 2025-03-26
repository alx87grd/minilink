from framework import DynamicSystem, StaticSystem, Step
import numpy as np


sys1 = DynamicSystem(2, 1, 1)

sys1.add_input_port("w", 2, default_value=np.array([7.7, 2.2]))
sys1.add_input_port("v", 1, default_value=np.array([1.1]))

print("Sys1 u dim:", sys1.m)
print("Sys1 x dim:", sys1.n)
print("Sys1 y dim:", sys1.p)

default_input_signals = sys1.collect_input_signals()
u = sys1.input_signals2u(default_input_signals)
u2 = sys1.get_u_from_input_ports()
assert np.allclose(u, u2)

print("Default u:", u)
# print("Default u:", u2)
print("Default input signals:", default_input_signals)


u = np.random.rand(sys1.m)
u2 = sys1.input_signals2u(sys1.u2input_signals(u))
assert np.allclose(u, u2)

sys1.print_html()

sys1.show_diagram()
