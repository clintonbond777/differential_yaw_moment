import numpy as np

# Default coefficients
DEFAULT_COEFFS = {
    "b_lat": 10.0, "c_lat": 1.9, "d_lat": 1.0, "e_lat": -1.0,
    "b_lon": 12.0, "c_lon": 2.3, "d_lon": 0.8, "e_lon": -1.2,
    "b_align": 5.0, "c_align": 2.0, "d_align": 0.5, "e_align": -1.5,
}

def simple_tire_model(slip_angle, vertical_load, slip_ratio, coeffs=None):
    """
    A simplified tire model to calculate tire forces and moments.

    Args:
        slip_angle (float): Slip angle in radians.
        vertical_load (float): Vertical load in Newtons.
        slip_ratio (float): Slip ratio.
        coeffs (dict, optional): A dictionary of tire coefficients. 
                                 If None, uses default values.

    Returns:
        dict: A dictionary containing the lateral force, longitudinal force, and aligning moment.
    """

    # Use provided coeffs or default
    p = coeffs if coeffs is not None else DEFAULT_COEFFS

    # Simplified Pacejka-like coefficients
    b_lat = p['b_lat']
    c_lat = p['c_lat']
    d_lat = p['d_lat']
    e_lat = p['e_lat']

    b_lon = p['b_lon']
    c_lon = p['c_lon']
    d_lon = p['d_lon']
    e_lon = p['e_lon']

    b_align = p['b_align']
    c_align = p['c_align']
    d_align = p['d_align']
    e_align = p['e_align']

    # Lateral Force (Fy)
    lateral_force = vertical_load * d_lat * np.sin(c_lat * np.arctan(b_lat * slip_angle - e_lat * (b_lat * slip_angle - np.arctan(b_lat * slip_angle))))

    # Longitudinal Force (Fx)
    longitudinal_force = vertical_load * d_lon * np.sin(c_lon * np.arctan(b_lon * slip_ratio - e_lon * (b_lon * slip_ratio - np.arctan(b_lon * slip_ratio))))

    # Aligning Moment (Mz)
    aligning_moment = vertical_load * d_align * np.sin(c_align * np.arctan(b_align * slip_angle - e_align * (b_align * slip_angle - np.arctan(b_align * slip_angle))))


    return {
        "lateral_force": lateral_force,
        "longitudinal_force": longitudinal_force,
        "aligning_moment": aligning_moment,
    }