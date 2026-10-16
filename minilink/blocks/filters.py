"""Signal-conditioning filters: Butterworth low-pass, notch and washout, from a frequency in Hz."""

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


class NotchFilter(TransferFunction):
    """Second-order notch (band-stop) at ``notch_hz`` with quality factor ``quality``.

    ``H(s) = (s² + ω₀²) / (s² + (ω₀/Q)·s + ω₀²)`` with ``ω₀ = 2π·notch_hz``;
    larger ``quality`` narrows the rejected band. A filter is ``u -> y`` and
    not a plant: it exposes no state ``x``.
    """

    def __init__(self, notch_hz=1.0, quality=10.0):
        # Parameters: the notch frequency in Hz and the quality factor
        self.notch_hz = float(notch_hz)
        self.quality = float(quality)

        f_0 = self.notch_hz
        Q = self.quality

        # H(s) = (s² + ω₀²) / (s² + (ω₀/Q) s + ω₀²), with ω₀ = 2π f₀
        w0 = 2.0 * np.pi * f_0
        num = [1.0, 0.0, w0**2]
        den = [1.0, w0 / Q, w0**2]

        super().__init__(num, den, name="Notch Filter", expose_state=False)


class Washout(TransferFunction):
    """First-order high-pass / washout ``H(s) = s / (s + ω_c)``.

    Passes transients and rejects the steady component; ``ω_c = 2π·cutoff_hz``.
    A filter is ``u -> y`` and not a plant: it exposes no state ``x``.
    """

    def __init__(self, cutoff_hz=1.0):
        # Parameters: the cutoff frequency in Hz
        self.cutoff_hz = float(cutoff_hz)

        f_c = self.cutoff_hz

        # H(s) = s / (s + ω_c), with ω_c = 2π f_c
        wc = 2.0 * np.pi * f_c
        num = [1.0, 0.0]
        den = [1.0, wc]

        super().__init__(num, den, name="Washout Filter", expose_state=False)


if __name__ == "__main__":
    lpf = LowPassFilter(cutoff_hz=0.5)
    print("Low-pass poles:", lpf.poles)

    # Drive a 2 Hz sinusoid through the 0.5 Hz low-pass; expect strong attenuation.
    traj = lpf.compute_forced(
        lambda t: np.array([np.sin(2.0 * np.pi * 2.0 * t)]), tf=10.0
    )
    print("output amplitude (last 2 s):", float(np.max(np.abs(traj.x[:, -200:]))))
