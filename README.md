# Differential Yaw Moment Analysis

This project provides a vehicle dynamics model in Python for analyzing the yaw moment and handling balance of a 4-wheel vehicle. It is designed to explore the effects of a limited-slip differential (LSD) and tire characteristics on vehicle behavior during steady-state cornering.

## Features

- **4-Wheel Vehicle Model:** Simulates the forces and moments on a four-wheeled vehicle.
- **Tunable Vehicle Parameters:** Easily modify vehicle parameters such as mass, wheelbase, center of gravity, and more.
- **Limited-Slip Differential Model:** Includes a 1.5-way plate-type LSD model with adjustable ramp angles, preload, and friction.
- **Customizable Tire Model:** Based on a simplified Pacejka-like formula. The tire grip characteristics for the front and rear axles can be tuned independently.
- **Yaw Moment Analysis:** Generates Milliken Moment Diagrams to visualize the vehicle's understeer/oversteer characteristics under different torque applications.
- **Side Slip Visualization:** Plots the vehicle's side slip angle against lateral acceleration.

## Installation

To run the simulation, you need Python 3 and the following packages. You can install them using the provided `requirements.txt` file:

```bash
pip install -r requirements.txt
```

## Usage

The main script for running the analysis is `yaw_moment_analysis.py`.

To generate the default plots (Milliken Moment Diagram and Side Slip Angle plot), simply run the script:

```bash
python yaw_moment_analysis.py
```

### Tuning

You can adjust various parameters at the top of the `yaw_moment_analysis.py` script to see their effect on the vehicle's handling:

- **Vehicle Parameters:** `MASS`, `WB` (Wheelbase), `CG_HEIGHT`, etc.
- **LSD Parameters:** `RAMP_ANGLE_ACCEL`, `RAMP_ANGLE_DECEL`, `PRELOAD`, etc.
- **Tire Coefficients:** Modify `FRONT_TIRE_COEFFS` and `REAR_TIRE_COEFFS` to change the front and rear axle grip.
- **Plotting Conditions:** Change the `torque_values` list in the `if __name__ == '__main__':` block to compare different levels of drive torque.