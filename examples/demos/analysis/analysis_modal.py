"""Modal analysis: poles and mode shapes of a double pendulum hanging down."""

import numpy as np

from minilink import DoublePendulum

plant = DoublePendulum()
plant.modal_analysis([np.pi, 0.0, 0.0, 0.0], mode="all")
