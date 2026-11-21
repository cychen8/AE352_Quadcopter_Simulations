"""Team 15 quadrotor ODE and stationary-hover check.

Run: python drone_dynamics.py
Dependencies: numpy, scipy (SciPy is only needed for the hover test).
Matches Team_15_Preliminary_Report.pdf dated November 18, 2026,
Eqs. (3)-(17), including its quasi-steady yaw approximation.
"""

import numpy as np


def drone_dynamics(t, s, u, params):
    """Return the 16 state derivatives for the report's baseline model.

    s = [x, y, z, vx, vy, vz, phi, theta, psi, p, q, r,
         Omega1, Omega2, Omega3, Omega4]
    u = [M1, M2, M3, M4], motor torques [N m], in nominal spin directions.
    params: dictionary containing m, g, L, Ixx, Iyy, Izz, Jr, kT, kQ.

    Body axes: x forward, y left, z up. Inertial Z is upward.
    Rotor positions: 1 left, 2 front, 3 right, 4 rear.
    Rotors 1 and 3: CCW; 2 and 4: CW, viewed from above.
    Angles are radians; body rates and nonnegative rotor speeds are rad/s.
    t is accepted for ODE-solver compatibility; the model is autonomous
    for a supplied u. Inputs and states are not modified.

    No drag, contact, controller, saturation, battery cutoff, gyroscopic
    coupling, or transient motor-reaction yaw moments are added here.
    Euler angles require cos(theta) != 0. Rotor speeds must remain nonnegative.
    """
    s = np.asarray(s, dtype=float)
    u = np.asarray(u, dtype=float)
    if s.shape != (16,) or u.shape != (4,):
        raise ValueError("Expected s.shape == (16,) and u.shape == (4,).")

    x, y, z, vx, vy, vz, phi, theta, psi, p, q, r = s[:12]
    Omega = s[12:16]

    m, g, L = params["m"], params["g"], params["L"]
    Ixx, Iyy, Izz = params["Ixx"], params["Iyy"], params["Izz"]
    Jr, kT, kQ = params["Jr"], params["kT"], params["kQ"]

    sin_phi, cos_phi = np.sin(phi), np.cos(phi)
    sin_theta, cos_theta = np.sin(theta), np.cos(theta)
    sin_psi, cos_psi = np.sin(psi), np.cos(psi)
    if abs(cos_theta) < 1.0e-8:
        raise ValueError("Z-Y-X Euler angles are singular near theta = +/- pi/2.")

    # Thrusts and quasi-steady aerodynamic yaw torque: Eqs. (7)-(9).
    Omega_sq = Omega**2
    T1, T2, T3, T4 = kT * Omega_sq
    T = T1 + T2 + T3 + T4
    tau_x = L * (T1 - T3)
    tau_y = L * (T4 - T2)
    tau_z = kQ * (-Omega_sq[0] + Omega_sq[1]
                  - Omega_sq[2] + Omega_sq[3])

    ds = np.zeros(16, dtype=float)

    # Position kinematics: r_dot = v.
    ds[0:3] = [vx, vy, vz]

    # Inertial accelerations: Eqs. (3)-(5).
    ds[3] = (T / m) * (cos_psi * sin_theta * cos_phi + sin_psi * sin_phi)
    ds[4] = (T / m) * (sin_psi * sin_theta * cos_phi - cos_psi * sin_phi)
    ds[5] = (T / m) * cos_theta * cos_phi - g

    # Euler-angle rates: Eqs. (13)-(15), not simply [p, q, r].
    ds[6] = p + (sin_theta / cos_theta) * (q * sin_phi + r * cos_phi)
    ds[7] = q * cos_phi - r * sin_phi
    ds[8] = (q * sin_phi + r * cos_phi) / cos_theta

    # Body angular accelerations: Eqs. (10)-(12).
    ds[9] = (tau_x + (Iyy - Izz) * q * r) / Ixx
    ds[10] = (tau_y + (Izz - Ixx) * p * r) / Iyy
    ds[11] = (tau_z + (Ixx - Iyy) * p * q) / Izz

    # Actual rotor speeds evolve through applied motor torques: Eq. (16).
    ds[12:16] = (u - kQ * Omega_sq) / Jr

    return ds


if __name__ == "__main__":
    from scipy.integrate import solve_ivp

    from parameters import PARAMS

    params = PARAMS.copy()
    Omega_hover = np.sqrt(params["m"] * params["g"] / (4.0 * params["kT"]))
    M_hover = params["kQ"] * Omega_hover**2

    s0 = np.zeros(16)
    s0[2] = 1.0                          # initial altitude [m]
    s0[12:16] = Omega_hover              # four identical initial speeds
    u_hover = np.full(4, M_hover)        # four constant holding torques

    ds0 = drone_dynamics(0.0, s0, u_hover, params)
    np.testing.assert_allclose(ds0, np.zeros(16), atol=1.0e-10, rtol=0.0)

    # u_hover is constant in this equilibrium test; no controller is used.
    solution = solve_ivp(
        lambda t, s: drone_dynamics(t, s, u_hover, params),
        t_span=(0.0, 120.0),
        y0=s0,
        method="RK45",
        t_eval=np.linspace(0.0, 120.0, 1201),
        rtol=1.0e-9,
        atol=1.0e-11,
        max_step=0.05,
    )
    if not solution.success:
        raise RuntimeError(solution.message)

    # Check every state at every saved time, not just final altitude.
    error = np.max(np.abs(solution.y - s0[:, None]), axis=1)
    np.testing.assert_allclose(
        solution.y, np.repeat(s0[:, None], solution.t.size, axis=1),
        atol=1.0e-7, rtol=0.0,
    )

    print(f"Hover rotor speed: {Omega_hover:.6f} rad/s")
    print(f"Hover rotor speed: {Omega_hover * 60 / (2 * np.pi):.2f} rpm")
    print(f"Hover motor torque: {M_hover:.6f} N m per rotor")
    print("Initial derivative vector:", ds0)
    print(f"Final position [m]: {solution.y[0:3, -1]}")
    print(f"Maximum altitude error: {error[2]:.3e} m")
    print(f"Maximum attitude error: {np.max(error[6:9]):.3e} rad")
    print("PASS: all 16 states remain at hover within numerical tolerance for 120 s.")
    print("This verifies equilibrium, not disturbance rejection or battery endurance.")
