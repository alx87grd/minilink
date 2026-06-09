import matplotlib.pyplot as plt
import numpy as np

# ---------------------------------------------------------------------
# Synthetic combustion engine model
# ---------------------------------------------------------------------

rpm_min = 800
rpm_max = 7000
resolution = 400

rpm = np.linspace(rpm_min, rpm_max, resolution)


# Example full-load torque curve [Nm]
# Shape: rises at low rpm, peaks mid-range, falls at high rpm
def max_torque_curve(rpm):
    torque_peak = 260.0  # Nm
    rpm_peak = 4000.0

    # Gaussian-like main torque shape
    torque = torque_peak * np.exp(-(((rpm - rpm_peak) / 2300.0) ** 2))

    # Add low-speed torque floor
    torque += 80.0

    # High-rpm roll-off
    rolloff = 1.0 / (1.0 + np.exp((rpm - 6200.0) / 300.0))

    return torque * rolloff


torque_full = max_torque_curve(rpm)


# Power equation:
# P [W] = torque [Nm] * angular_speed [rad/s]
omega = rpm * 2.0 * np.pi / 60.0
power_full_w = torque_full * omega
power_full_kw = power_full_w / 1000.0
power_full_hp = power_full_kw * 1.34102


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

plt.title("Synthetic Combustion Engine Torque and Power Curve")
fig.tight_layout()
plt.show()
