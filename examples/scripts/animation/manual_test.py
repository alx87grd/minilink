from minilink.dynamics.catalog.pendulum.pendulum import Pendulum
from minilink.dynamics.catalog.pendulum.cartpole import JaxCartPole

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycle


# sys = JaxCartPole()
# sys.inputs["u"].nominal_value = 2.2
# sys.x0[0] = 2.0

# sys.compute_trajectory(tf=10, show=False)


# sys.animate()
# sys.animate(renderer="meshcat")
# sys.animate(renderer="plotly")
# sys.animate(renderer="pygame")


# sys = Pendulum()
# sys.inputs["u"].nominal_value = 2.2
# sys.x0[0] = 2.0

# sys.compute_trajectory(tf=10, show=False)


# sys.animate()
# sys.animate(renderer="meshcat")
# sys.animate(renderer="plotly")
# sys.animate(renderer="pygame")


sys = DynamicBicycle()
sys.inputs["w_rear"].nominal_value = 20.0
sys.inputs["delta"].nominal_value = 0.2


sys.compute_trajectory(tf=10, show=False)


sys.animate()
sys.animate(renderer="meshcat")
sys.animate(renderer="plotly")
sys.animate(renderer="pygame")
