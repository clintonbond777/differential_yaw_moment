
import numpy as np
import matplotlib.pyplot as plt
from simple_tire_model.tire_model import simple_tire_model

def plot_vs_slip_angle(vertical_load=1000, slip_ratio=0):
    """Plots forces and moments vs. slip angle."""
    slip_angles = np.linspace(-0.2, 0.2, 100)
    results = [simple_tire_model(sa, vertical_load, slip_ratio) for sa in slip_angles]

    lat_forces = [r["lateral_force"] for r in results]
    lon_forces = [r["longitudinal_force"] for r in results]
    align_moments = [r["aligning_moment"] for r in results]

    fig1, axs = plt.subplots(3, 1, figsize=(8, 12))
    fig1.suptitle(f'Tire Model vs. Slip Angle (Fz={vertical_load}N, SR={slip_ratio})')

    axs[0].plot(slip_angles, lat_forces)
    axs[0].set_xlabel('Slip Angle (rad)')
    axs[0].set_ylabel('Lateral Force (N)')
    axs[0].grid(True)

    axs[1].plot(slip_angles, lon_forces)
    axs[1].set_xlabel('Slip Angle (rad)')
    axs[1].set_ylabel('Longitudinal Force (N)')
    axs[1].grid(True)

    axs[2].plot(slip_angles, align_moments)
    axs[2].set_xlabel('Slip Angle (rad)')
    axs[2].set_ylabel('Aligning Moment (Nm)')
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

def plot_vs_slip_ratio(vertical_load=1000, slip_angle=0):
    """Plots forces and moments vs. slip ratio."""
    slip_ratios = np.linspace(-0.2, 0.2, 100)
    results = [simple_tire_model(slip_angle, vertical_load, sr) for sr in slip_ratios]

    lat_forces = [r["lateral_force"] for r in results]
    lon_forces = [r["longitudinal_force"] for r in results]
    align_moments = [r["aligning_moment"] for r in results]

    fig2, axs = plt.subplots(3, 1, figsize=(8, 12))
    fig2.suptitle(f'Tire Model vs. Slip Ratio (Fz={vertical_load}N, SA={slip_angle}rad)')

    axs[0].plot(slip_ratios, lat_forces)
    axs[0].set_xlabel('Slip Ratio')
    axs[0].set_ylabel('Lateral Force (N)')
    axs[0].grid(True)

    axs[1].plot(slip_ratios, lon_forces)
    axs[1].set_xlabel('Slip Ratio')
    axs[1].set_ylabel('Longitudinal Force (N)')
    axs[1].grid(True)

    axs[2].plot(slip_ratios, align_moments)
    axs[2].set_xlabel('Slip Ratio')
    axs[2].set_ylabel('Aligning Moment (Nm)')
    axs[2].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])

if __name__ == '__main__':
    # Plot with constant vertical load and slip ratio, varying slip angle
    plot_vs_slip_angle(vertical_load=4000, slip_ratio=0.05)

    # Plot with constant vertical load and slip angle, varying slip ratio
    plot_vs_slip_ratio(vertical_load=4000, slip_angle=0.1)

    plt.show()
