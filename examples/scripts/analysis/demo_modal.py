"""Modal analysis demo.

Run from the repo root::

    python examples/scripts/analysis/demo_modal.py
"""

import numpy as np

from minilink.dynamics.catalog.pendulum.double_pendulum import DoublePendulum

cartpole = DoublePendulum()
cartpole.modal_analysis(x_bar=[np.pi, 0.0, 0.0, 0.0], mode="all")
