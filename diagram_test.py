from framework import DynamicSystem, StaticSystem, Step, GrapheSystem
import numpy as np


sys1 = DynamicSystem(2, 1, 1)
sys1.add_input_port("w", 2, default_value=np.array([7.7, 2.2]))
sys1.add_input_port("v", 1, default_value=np.array([1.1]))
sys2 = DynamicSystem(2, 1, 1)
sys3 = StaticSystem(1, 1)
sys4 = DynamicSystem(2, 1, 1)
step = Step(np.array([0.0]), np.array([1.0]), 1.0)

gsys = GrapheSystem()


gsys.add_system(sys1, "sys1")
gsys.add_system(sys2, "sys2")
gsys.add_system(sys3, "sys3")
gsys.add_system(sys4, "sys4")
gsys.add_system(step, "step")

print("List of subsystems:\n")
print(gsys.subsystems)

print("List of edges before connections:\n")
print(gsys.edges)

gsys.add_edge("sys1", "y", "sys2", "u")
gsys.add_edge("sys2", "y", "sys3", "u")
gsys.add_edge("sys2", "y", "sys4", "u")
gsys.add_edge("sys4", "y", "sys1", "u")
gsys.add_edge("step", "y", "sys1", "v")
gsys.add_edge("sys4", "y", "sys1", "w")
# gsys.add_edge("sys4", "y", "sys1", "u")
# gsys.add_edge("sys3", "y", "sys1", "w")

print("List of edges after connections:\n")
print(gsys.edges)

g = gsys.render_graphe()

print("sys.n = ", gsys.n)
print("sys.m = ", gsys.m)
print("sys.p = ", gsys.p)
print("sys.state_label = ", gsys.state_label)
