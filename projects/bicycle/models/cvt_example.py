import matplotlib.pyplot as plt
import numpy as np


def cvt_ratio_vs_speed(vehicle_speed, mode="H"):
    """
    vehicle_speed: m/s
    returns approximate total ratio
    """

    if mode == "H":
        ratio_max = 40.0
        ratio_min = 11.5
        v_shift_start = 2.0  # m/s
        v_shift_end = 22.0  # m/s, ~79 km/h

    elif mode == "XL":
        ratio_max = 65.0
        ratio_min = 18.0
        v_shift_start = 1.0  # m/s
        v_shift_end = 12.0  # m/s, ~43 km/h

    else:
        raise ValueError("mode must be 'H' or 'XL'")

    x = (vehicle_speed - v_shift_start) / (v_shift_end - v_shift_start)
    x = np.clip(x, 0.0, 1.0)

    # Smooth transition
    s = x * x * (3 - 2 * x)

    ratio = ratio_max + s * (ratio_min - ratio_max)

    return ratio


def main():

    # ---------------------------------------------------------------------
    # Plot
    # ---------------------------------------------------------------------

    # Speed vector
    v_ms = np.linspace(0.0, 30.0, 600)  # m/s
    v_kmh = v_ms * 3.6  # km/h

    # Ratios
    ratio_H = cvt_ratio_vs_speed(v_ms, mode="H")
    ratio_XL = cvt_ratio_vs_speed(v_ms, mode="XL")

    # Figure
    fig, ax = plt.subplots(figsize=(9, 5))

    ax.plot(v_kmh, ratio_H, linewidth=2, label="Mode H")
    ax.plot(v_kmh, ratio_XL, linewidth=2, label="Mode Extra-L")

    ax.set_xlabel("Vehicle speed [km/h]")
    ax.set_ylabel("Total ratio engine/wheel [-]")
    ax.set_title("Approximate CVT total ratio vs vehicle speed")
    ax.grid(True)
    ax.legend()

    # Optional shift boundary markers
    ax.axvline(2.0 * 3.6, linestyle="--", linewidth=1, alpha=0.7)
    ax.axvline(22.0 * 3.6, linestyle="--", linewidth=1, alpha=0.7)
    ax.axvline(1.0 * 3.6, linestyle=":", linewidth=1, alpha=0.7)
    ax.axvline(12.0 * 3.6, linestyle=":", linewidth=1, alpha=0.7)

    ax.text(2.0 * 3.6, 42, "Start H shift", rotation=90, va="bottom", ha="right")
    ax.text(22.0 * 3.6, 13, "End H shift", rotation=90, va="bottom", ha="right")
    ax.text(1.0 * 3.6, 67, "Start XL shift", rotation=90, va="bottom", ha="left")
    ax.text(12.0 * 3.6, 20, "End XL shift", rotation=90, va="bottom", ha="left")

    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
