
import numpy as np
import matplotlib.pyplot as plt
from simple_tire_model.tire_model import simple_tire_model, DEFAULT_COEFFS
from scipy.optimize import fsolve

# Vehicle parameters
MASS = 1350.0  # kg
WB = 2.710  # m (wheelbase)
TW = 2.0  # m (trackwidth)
L_F = WB / 2.0  # m (CG location from front axle)
L_R = WB / 2.0 # m (CG location from rear axle)
CG_HEIGHT = 0.5 # m (assumed CG height, needed for load transfer)
WHEEL_RADIUS = 0.33 # m (assumed wheel radius)

# LSD parameters
RAMP_ANGLE_ACCEL = 45.0
RAMP_ANGLE_DECEL = 60.0
PRELOAD = 10.0
FRICTION_COEFF = 0.5
LSD_SMOOTHING_FACTOR = 100 # Controls the smoothness of the LSD transition

# Tire Coefficients
# Start with default for the rear
REAR_TIRE_COEFFS = DEFAULT_COEFFS.copy()
# Create a high-grip version for the front to achieve neutral steer
FRONT_TIRE_COEFFS = DEFAULT_COEFFS.copy()
FRONT_TIRE_COEFFS["b_lat"] = 4.75 # Default is 10.0. Increased for more front grip.


class Vehicle:
    """
    A 4-wheel vehicle model for steady-state cornering analysis.

    Sign Convention (Right-Hand Coordinate System):
    - Coordinate System (as viewed from the driver):
      - X-axis: Longitudinal, positive forward.
      - Y-axis: Lateral, positive to the left.
      - Z-axis: Vertical, positive upwards.
    - Yaw Rate (yaw_rate): Positive for a left turn (counter-clockwise from above).
    - Steer Angle (steer_angle): Positive for a left turn.
    - Vehicle Slip Angle (beta): Positive when the vehicle's velocity vector is to the right of its longitudinal axis.
    - Tire Slip Angle (slip_angle): Positive when the velocity vector of the wheel is to the right of the direction the wheel is pointing.
    - Forces (Fx, Fy): Positive in the direction of the corresponding axis.
    - Yaw Moment (Mz): Positive for a yawing motion to the left (counter-clockwise).
    """
    def __init__(self, mass, wb, tw, lf, lr, h_cg):
        self.mass = mass
        self.wb = wb
        self.tw = tw
        self.lf = lf
        self.lr = lr
        self.h_cg = h_cg
        self.g = 9.81

    def lsd_model(self, input_torque):
        if input_torque > 0:
            ramp_angle = np.deg2rad(RAMP_ANGLE_ACCEL)
        else:
            ramp_angle = np.deg2rad(RAMP_ANGLE_DECEL)
        locking_torque = np.abs(input_torque) * np.tan(ramp_angle) * FRICTION_COEFF + PRELOAD
        return locking_torque

    def find_slip_ratio(self, Fx_target, Fz, coeffs):
        d_lon = coeffs['d_lon']
        fx_max = Fz * d_lon
        if abs(Fx_target) > fx_max:
            Fx_target = np.sign(Fx_target) * fx_max

        def equation(sr):
            tire = simple_tire_model(0, Fz, sr, coeffs)
            return tire['longitudinal_force'] - Fx_target
        
        sr_solution, info, ier, msg = fsolve(equation, 0.05, full_output=True)
        if ier != 1:
            return np.sign(Fx_target) * 0.2
        return sr_solution[0]

    def calculate_forces_and_moments(self, vx, steer_angle, beta, yaw_rate, input_torque):
        slip_angle_fl = np.arctan((vx * np.sin(beta) + self.lf * yaw_rate) / (vx * np.cos(beta) - self.tw/2 * yaw_rate)) - steer_angle
        slip_angle_fr = np.arctan((vx * np.sin(beta) + self.lf * yaw_rate) / (vx * np.cos(beta) + self.tw/2 * yaw_rate)) - steer_angle
        slip_angle_rl = np.arctan((vx * np.sin(beta) - self.lr * yaw_rate) / (vx * np.cos(beta) - self.tw/2 * yaw_rate))
        slip_angle_rr = np.arctan((vx * np.sin(beta) - self.lr * yaw_rate) / (vx * np.cos(beta) + self.tw/2 * yaw_rate))

        lat_accel = yaw_rate * vx
        Fz_static = self.mass * self.g / 4.0
        Fz_lat_front = (self.mass * lat_accel * self.h_cg / self.tw) * (self.lr / self.wb)
        Fz_lat_rear = (self.mass * lat_accel * self.h_cg / self.tw) * (self.lf / self.wb)

        Fz_fl = Fz_static - Fz_lat_front
        Fz_fr = Fz_static + Fz_lat_front
        Fz_rl = Fz_static - Fz_lat_rear
        Fz_rr = Fz_static + Fz_lat_rear

        locking_torque = self.lsd_model(input_torque)
        delta_T = (locking_torque / 2.0) * np.tanh(LSD_SMOOTHING_FACTOR * yaw_rate)
        
        T_rl = input_torque / 2.0 + delta_T
        T_rr = input_torque / 2.0 - delta_T

        Fx_rl_target = T_rl / WHEEL_RADIUS
        Fx_rr_target = T_rr / WHEEL_RADIUS

        slip_ratio_rl = self.find_slip_ratio(Fx_rl_target, Fz_rl, REAR_TIRE_COEFFS)
        slip_ratio_rr = self.find_slip_ratio(Fx_rr_target, Fz_rr, REAR_TIRE_COEFFS)

        tire_fl = simple_tire_model(slip_angle_fl, Fz_fl, 0, FRONT_TIRE_COEFFS)
        tire_fr = simple_tire_model(slip_angle_fr, Fz_fr, 0, FRONT_TIRE_COEFFS)
        tire_rl = simple_tire_model(slip_angle_rl, Fz_rl, slip_ratio_rl, REAR_TIRE_COEFFS)
        tire_rr = simple_tire_model(slip_angle_rr, Fz_rr, slip_ratio_rr, REAR_TIRE_COEFFS)

        Fy_fl = tire_fl['lateral_force']
        Fy_fr = tire_fr['lateral_force']
        Fy_rl = tire_rl['lateral_force']
        Fy_rr = tire_rr['lateral_force']

        Fx_fl = tire_fl['longitudinal_force']
        Fx_fr = tire_fr['longitudinal_force']
        Fx_rl = tire_rl['longitudinal_force']
        Fx_rr = tire_rr['longitudinal_force']

        sum_Fy = (Fy_fl * np.cos(steer_angle) + Fx_fl * np.sin(steer_angle) +
                  Fy_fr * np.cos(steer_angle) + Fx_fr * np.sin(steer_angle) +
                  Fy_rl + Fy_rr)
        
        sum_Mz = (self.lf * (Fy_fl * np.cos(steer_angle) + Fx_fl * np.sin(steer_angle)) +
                  self.lf * (Fy_fr * np.cos(steer_angle) + Fx_fr * np.sin(steer_angle)) -
                  self.lr * (Fy_rl + Fy_rr) +
                  self.tw/2 * (-Fx_fl * np.cos(steer_angle) + Fy_fl * np.sin(steer_angle) +
                               Fx_fr * np.cos(steer_angle) - Fy_fr * np.sin(steer_angle) -
                               Fx_rl + Fx_rr))

        return sum_Fy, sum_Mz

def plot_milliken_moment_diagram(vx=20, torque_values=[0, 1000]):
    vehicle = Vehicle(MASS, WB, TW, L_F, L_R, CG_HEIGHT)
    
    fig, axs = plt.subplots(2, 1, figsize=(12, 16), sharex=True)
    
    colors = ['b', 'r', 'g', 'm']
    linestyles = ['--', '-', '-.', ':']

    for i, torque in enumerate(torque_values):
        steer_angles_deg = [-6, -4, -2, 0, 2, 4, 6]
        beta_rad_range = np.linspace(np.deg2rad(-8), np.deg2rad(8), 50)

        for steer_deg in steer_angles_deg:
            steer_rad = np.deg2rad(steer_deg)
            lat_accels_g = []
            yaw_moments = []
            betas_deg_result = []

            for beta_rad in beta_rad_range:
                def force_balance_equation(yaw_rate):
                    sum_Fy, _ = vehicle.calculate_forces_and_moments(vx, steer_rad, beta_rad, yaw_rate, torque)
                    return sum_Fy - vehicle.mass * yaw_rate * vx

                yaw_rate_solution, info, ier, msg = fsolve(force_balance_equation, 0.0, full_output=True)
                if ier != 1:
                    continue

                _, sum_Mz = vehicle.calculate_forces_and_moments(vx, steer_rad, beta_rad, yaw_rate_solution[0], torque)
                
                lat_accel = yaw_rate_solution[0] * vx
                lat_accels_g.append(lat_accel / vehicle.g)
                yaw_moments.append(sum_Mz)
                betas_deg_result.append(np.rad2deg(beta_rad))

            sorted_indices = np.argsort(lat_accels_g)
            lat_accels_g = np.array(lat_accels_g)[sorted_indices]
            yaw_moments = np.array(yaw_moments)[sorted_indices]
            betas_deg_result = np.array(betas_deg_result)[sorted_indices]

            label = f'Steer={steer_deg}deg (T={torque}Nm)'
            if steer_deg == 0:
                axs[0].plot(lat_accels_g, yaw_moments, color=colors[i], linestyle=linestyles[i], label=label, linewidth=2.5)
                axs[1].plot(lat_accels_g, betas_deg_result, color=colors[i], linestyle=linestyles[i], label=label, linewidth=2.5)
            else:
                axs[0].plot(lat_accels_g, yaw_moments, color=colors[i], linestyle=linestyles[i], label=label)
                axs[1].plot(lat_accels_g, betas_deg_result, color=colors[i], linestyle=linestyles[i], label=label)

    axs[0].set_ylabel("Yaw Moment (Nm)")
    axs[0].set_title(f"Tuned for Neutral Steer (Front b_lat = {FRONT_TIRE_COEFFS['b_lat']})")
    axs[0].legend()
    axs[0].grid(True)
    axs[0].axhline(0, color='black', linewidth=0.5)

    axs[1].set_xlabel("Lateral Acceleration (g)")
    axs[1].set_ylabel("Vehicle Side Slip Angle (deg)")
    axs[1].grid(True)

    plt.axvline(0, color='black', linewidth=0.5)
    plt.tight_layout()
    plt.show()

def plot_lsd_behavior(vx=20, yaw_rate_deg=5):
    """
    Plots the LSD output torque at each wheel vs. the input torque
    for a constant cornering condition.
    """
    vehicle = Vehicle(MASS, WB, TW, L_F, L_R, CG_HEIGHT)
    yaw_rate_rad = np.deg2rad(yaw_rate_deg)

    input_torques = np.linspace(-2000, 2000, 200)
    torques_rl = []
    torques_rr = []

    for t_in in input_torques:
        locking_torque = vehicle.lsd_model(t_in)
        delta_T = (locking_torque / 2.0) * np.tanh(LSD_SMOOTHING_FACTOR * yaw_rate_rad)
        
        t_rl = t_in / 2.0 + delta_T
        t_rr = t_in / 2.0 - delta_T

        torques_rl.append(t_rl)
        torques_rr.append(t_rr)

    plt.figure(figsize=(10, 7))
    plt.plot(input_torques, torques_rl, label='Torque - Rear Left (Inner Wheel)')
    plt.plot(input_torques, torques_rr, label='Torque - Rear Right (Outer Wheel)')
    plt.plot(input_torques, input_torques / 2.0, '--', color='gray', label='Open Differential (50/50 split)')
    plt.xlabel("Input Torque to Differential (Nm)")
    plt.ylabel("Output Torque at Wheel (Nm)")
    plt.title(f"LSD Behavior in a Constant Left Turn (Yaw Rate = {yaw_rate_deg} deg/s)")
    plt.grid(True)
    plt.legend()
    plt.show()

if __name__ == '__main__':
    plot_milliken_moment_diagram(vx=20, torque_values=[-150,0,150])
