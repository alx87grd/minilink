"""Shared Plotly layout defaults."""

# Plotly figures use pixels. Keep signal plots and animation canvases aligned in
# notebooks by using one shared width.
PLOTLY_FIG_WIDTH = 700
PLOTLY_ANIMATION_HEIGHT = 560
PLOTLY_SIGNAL_MIN_HEIGHT = 300
PLOTLY_SIGNAL_ROW_HEIGHT = 170
PLOTLY_TEMPLATE = "plotly_white"

PLOTLY_SIGNAL_MARGIN = {"l": 70, "r": 20, "t": 50, "b": 45}
PLOTLY_2D_MARGIN = {"l": 55, "r": 25, "t": 50, "b": 50}
PLOTLY_3D_MARGIN = {"l": 0, "r": 0, "t": 45, "b": 0}
PLOTLY_ANIMATION_2D_MARGIN = {"l": 55, "r": 25, "t": 50, "b": 95}
PLOTLY_ANIMATION_3D_MARGIN = {"l": 0, "r": 0, "t": 45, "b": 95}
