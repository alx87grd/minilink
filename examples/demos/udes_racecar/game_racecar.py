"""Drive the 1/10 racecar with the keyboard — ``sys.game()`` facade."""

# UP/DOWN    -> drive power ``P_cmd``
# LEFT/RIGHT -> steer ``delta_cmd``

from minilink import UdeSRacecarDyn3D

sys = UdeSRacecarDyn3D()

# sys.plot_bode(of="speed", wrt="P_cmd")


sys.game(renderer="meshcat", is_3d=True)
# sys.game(renderer="pygame")
# sys.game(renderer="matplotlib")
