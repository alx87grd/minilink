import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------
# Can-Am Defender HD9 / Rotax 976 cc V-twin engine model
# Specs approx:
#   Max power  = 65 hp ≈ 48.5 kW at ~7000 rpm
#   Max torque = 59 lb-ft ≈ 80 Nm
# ---------------------------------------------------------------------

rpm_min = 800
rpm_max = 7500
resolution = 400

rpm = np.linspace(rpm_min, rpm_max, resolution)


# Approximate full-load torque curve [Nm]
# Shape:
#   - usable torque at low rpm
#   - peak / plateau around mid-range
#   - roll-off near high rpm
def max_torque_curve(rpm):
    # Anchor points estimated for HD9 Rotax 976 cc V-twin
    rpm_points = np.array(
        [
            800,
            1000,
            1500,
            2000,
            2500,
            3000,
            3500,
            4000,
            4500,
            5000,
            5500,
            6000,
            6500,
            7000,
            7500,
        ]
    )

    torque_points = np.array(
        [35, 45, 58, 68, 74, 78, 80, 80, 78, 76, 73, 70, 68, 66, 55]
    )

    # Interpolation between points
    torque = np.interp(rpm, rpm_points, torque_points)

    return torque


def main():
    torque_full = max_torque_curve(rpm)

    # Power equation:
    # P [W] = torque [Nm] * angular_speed [rad/s]
    omega = rpm * 2.0 * np.pi / 60.0
    power_full_w = torque_full * omega
    power_full_kw = power_full_w / 1000.0
    power_full_hp = power_full_kw * 1.34102

    # ---------------------------------------------------------------------
    # Print key values for verification
    # ---------------------------------------------------------------------

    idx_torque_max = np.argmax(torque_full)
    idx_power_max = np.argmax(power_full_kw)

    print("Max torque:")
    print(f"  {torque_full[idx_torque_max]:.1f} Nm at {rpm[idx_torque_max]:.0f} rpm")

    print("Max power:")
    print(f"  {power_full_kw[idx_power_max]:.1f} kW")
    print(f"  {power_full_hp[idx_power_max]:.1f} hp at {rpm[idx_power_max]:.0f} rpm")

    print("Power at 7000 rpm:")
    power_7000_kw = np.interp(7000, rpm, power_full_kw)
    power_7000_hp = power_7000_kw * 1.34102
    torque_7000 = np.interp(7000, rpm, torque_full)
    print(f"  Torque = {torque_7000:.1f} Nm")
    print(f"  Power  = {power_7000_kw:.1f} kW = {power_7000_hp:.1f} hp")

    # ---------------------------------------------------------------------
    # 1D torque and power curves
    # ---------------------------------------------------------------------

    fig, ax1 = plt.subplots(figsize=(9, 5))

    ax1.plot(rpm, torque_full, color="tab:blue", linewidth=2, label="Torque")
    ax1.set_xlabel("Engine speed [RPM]")
    ax1.set_ylabel("Torque [Nm]", color="tab:blue")
    ax1.tick_params(axis="y", labelcolor="tab:blue")
    ax1.grid(True)

    ax2 = ax1.twinx()
    ax2.plot(rpm, power_full_kw, color="tab:red", linewidth=2, label="Power")
    ax2.set_ylabel("Power [kW]", color="tab:red")
    ax2.tick_params(axis="y", labelcolor="tab:red")

    plt.title("Can-Am Defender HD9 Rotax 976 cc V-twin Torque and Power Curve")
    fig.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
