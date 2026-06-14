"""Signal-conditioning filter blocks.

Plant-agnostic linear filters for references, measurements, or derivatives.
Each is a thin :class:`~minilink.blocks.transfer_function.TransferFunction`
(hence an ``LTISystem``) built from a cutoff/centre frequency given in **Hz**.

- :class:`LowPassFilter` — Butterworth low-pass (first-order by default).
- :class:`NotchFilter` — second-order band-stop at a centre frequency.
- :class:`Washout` — first-order high-pass (steady-state washout).
"""

import numpy as np
from scipy import signal

from minilink.blocks.transfer_function import TransferFunction


class LowPassFilter(TransferFunction):
    """Butterworth low-pass with ``-3 dB`` cutoff at ``cutoff_hz`` (order 1 default).

    First order is ``H(s) = ω_c / (s + ω_c)`` with ``ω_c = 2π·cutoff_hz``;
    higher ``order`` uses the analog Butterworth realization.
    """

    def __init__(self, cutoff_hz=1.0, order=1):
        self.cutoff_hz = float(cutoff_hz)
        self.order = int(order)

        wc = 2.0 * np.pi * self.cutoff_hz
        num, den = signal.butter(self.order, wc, btype="low", analog=True, output="ba")
        super().__init__(num, den, name="Low-Pass Filter")


class NotchFilter(TransferFunction):
    """Second-order notch (band-stop) at ``notch_hz`` with quality factor ``quality``.

    ``H(s) = (s² + ω₀²) / (s² + (ω₀/Q)·s + ω₀²)`` with ``ω₀ = 2π·notch_hz``;
    larger ``quality`` narrows the rejected band.
    """

    def __init__(self, notch_hz=1.0, quality=10.0):
        self.notch_hz = float(notch_hz)
        self.quality = float(quality)

        w0 = 2.0 * np.pi * self.notch_hz
        num = [1.0, 0.0, w0**2]
        den = [1.0, w0 / self.quality, w0**2]
        super().__init__(num, den, name="Notch Filter")


class Washout(TransferFunction):
    """First-order high-pass / washout ``H(s) = s / (s + ω_c)``.

    Passes transients and rejects the steady component; ``ω_c = 2π·cutoff_hz``.
    """

    def __init__(self, cutoff_hz=1.0):
        self.cutoff_hz = float(cutoff_hz)

        wc = 2.0 * np.pi * self.cutoff_hz
        super().__init__([1.0, 0.0], [1.0, wc], name="Washout Filter")


if __name__ == "__main__":
    lpf = LowPassFilter(cutoff_hz=0.5)
    print("Low-pass poles:", lpf.poles)

    # Drive a 2 Hz sinusoid through the 0.5 Hz low-pass; expect strong attenuation.
    traj = lpf.compute_forced(
        lambda t: np.array([np.sin(2.0 * np.pi * 2.0 * t)]), tf=10.0
    )
    print("output amplitude (last 2 s):", float(np.max(np.abs(traj.x[:, -200:]))))
