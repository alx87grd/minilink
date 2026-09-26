"""Headless shim that ``run_all_demos.py`` puts first on each demo's path.

A live Meshcat animation opens a browser tab and waits for it to connect;
on a runner nothing connects, so the demo hangs until its timeout. Here
``open`` and ``wait`` do nothing: the frames still go to the Meshcat server
and the rest of the demo runs unchanged. Demo code never sees this file.
"""

try:
    import meshcat
except ImportError:
    meshcat = None

if meshcat is not None:
    meshcat.Visualizer.open = lambda self, *args, **kwargs: None
    meshcat.Visualizer.wait = lambda self, *args, **kwargs: None
