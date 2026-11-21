import numpy as np

PARAMS = {}

# Physical parameters supplied by the team.
# All values use SI units.
PARAMS.update({
    "m": 1.6069,             # Total mass [kg]
    "g": 9.81,               # Gravity [m/s^2]
    "rho": 1.225,            # Air density [kg/m^3]
    "L": 0.35,               # Arm length [m]
    "Rp": 0.381 / 2.0,       # Propeller radius [m]

    "Ixx": 0.0526,           # Roll inertia [kg m^2]
    "Iyy": 0.0526,           # Pitch inertia [kg m^2]
    "Izz": 0.1044,           # Yaw inertia [kg m^2]
    "Jr": 5.0e-4,            # Rotor spin inertia [kg m^2]

    "kT": 4.87e-5,           # Thrust coefficient; called k_f in report
    "kQ": 9.21e-7,           # Aerodynamic torque coefficient

    "eta_m": 0.85,           # Motor efficiency
    "eta_ESC": 0.95,         # ESC efficiency
    "P_aux": 5.0,            # Avionics power [W]
    "E_usable": 140700.0,    # Usable battery energy [J]

    "Omega_max": 503.0,      # Maximum rotor speed [rad/s]
})

# Controller gains and actuator limits used for the submitted simulations.
PARAMS.update({
    "kp_pos": np.array([6.0, 6.0, 8.0]),
    "kd_pos": np.array([5.0, 5.0, 6.0]),

    "kp_att": np.array([110.0, 110.0, 65.0]),
    "kd_att": np.array([21.0, 21.0, 16.0]),

    "motor_time_constant": 0.025,       # Rotor-speed control setting [s]
    "tilt_max": np.deg2rad(25.0),       # Commanded tilt limit [rad]

    # Estimated constant torque limit for MN3110 KV470:
    # 60/(2*pi*470) * (15 - 0.3) ~= 0.30 N m.
    # Based on the 180 s current rating; not a measured torque-speed envelope.
    "M_max": 0.30,  # N m per motor
})

# Simulation acceptance tolerances chosen by our team;
# these are not additional requirements stated by the assignment.
TOLERANCES = {
    "position_m": 0.02,
    "circle_radius_m": 0.02,
    "speed_mps": 0.02,
    "stop_speed_mps": 0.02,
    "yaw_deg": 1.0,
    "straight_cross_track_m": 0.02,
}
