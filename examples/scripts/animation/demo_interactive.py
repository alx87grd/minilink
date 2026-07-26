
# Live loop: RealtimeSimulator + PygameInput behind the sys.game() facade;
# keys command the input-port bounds and the session returns a Trajectory.

# Plant system
# sys = Pendulum()
# sys.params["m"] = 1.0
# sys.params["l"] = 1.0
# sys.x0[0] = 0.0


# sys.game()
# sys.game(renderer="pygame")
# sys.game(renderer="matplotlib")
# sys.game(renderer="meshcat")

from minilink.dynamics.catalog.vehicles.dynamic_bicycle import DynamicBicycleCar3D

car = DynamicBicycleCar3D()

car.inputs["w_rear"].upper_bound[0] = 200.0
car.inputs["w_rear"].lower_bound[0] = -200.0

car.game(renderer="meshcat")
