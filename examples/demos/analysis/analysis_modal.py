"""Modal analysis: poles and mode shapes of a double pendulum hanging down."""

import numpy as np

from minilink import DoublePendulum

plant = DoublePendulum()
plant.animate_modal([np.pi, 0.0, 0.0, 0.0])
