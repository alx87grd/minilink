"""Modal analysis demo."""

import numpy as np

from minilink import DoublePendulum

cartpole = DoublePendulum()
cartpole.modal_analysis(x_bar=[np.pi, 0.0, 0.0, 0.0], mode="all")
