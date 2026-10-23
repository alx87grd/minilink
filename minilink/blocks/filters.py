"""Signal-conditioning filters: the Butterworth low-pass, from a frequency in Hz."""

import numpy as np
from scipy import signal

from minilink.blocks.transfer_function import TransferFunction

###############################################################################
# Public API
###############################################################################


class LowPassFilter(TransferFunction):
    """Butterworth low-pass with ``-3 dB`` cutoff at ``cutoff_hz`` (order 1 default).

    First order is ``H(s) = ω_c / (s + ω_c)`` with ``ω_c = 2π·cutoff_hz``;
    higher ``order`` uses the analog Butterworth realization. A filter is
    ``u -> y`` and not a plant: it exposes no state ``x``.
    """

    def __init__(self, cutoff_hz=1.0, order=1):
        # Parameters: the cutoff frequency in Hz and the order
        self.cutoff_hz = float(cutoff_hz)
        self.order = int(order)

        f_c = self.cutoff_hz
        order = self.order

        # The analog Butterworth polynomials of that order, cut off at ω_c = 2π f_c
        wc = 2.0 * np.pi * f_c
        num, den = signal.butter(order, wc, btype="low", analog=True, output="ba")

        super().__init__(num, den, name="Low-Pass Filter", expose_state=False)


if __name__ == "__main__":
    lpf = LowPassFilter(cutoff_hz=0.5)
    print("Low-pass poles:", lpf.poles)

    # Drive a 2 Hz sinusoid through the 0.5 Hz low-pass; expect strong attenuation.
    traj = lpf.compute_forced(
        lambda t: np.array([np.sin(2.0 * np.pi * 2.0 * t)]), tf=10.0
    )
    print("output amplitude (last 2 s):", float(np.max(np.abs(traj.x[:, -200:]))))
