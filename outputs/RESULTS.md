# Simulation results

These are model predictions for the selected components and documented parameter assumptions, not validated hardware performance.
Each mission starts with a fresh battery. No wind, sensor noise or disturbances are applied.
Engineering tolerances are in parameters.py and metrics.json; they are not instructor-specified.

| Mission | Energy used (Wh) | Usable energy (Wh) | Checks |
|---|---:|---:|---|
| hover | 3.6674 | 39.083 | Pass |
| circle | 2.1396 | 39.083 | Pass |
| sequence | 4.8261 | 39.083 | Pass |

Sum of the three separate mission energies: 10.6331 Wh. This excludes any connecting travel; it is not a continuous combined-mission test.

## Full numerical results

```json
[
  {
    "mission": "hover",
    "duration_s": 120.0,
    "time_step_s": 0.01,
    "energy_used_Wh": 3.6674013246497053,
    "energy_usable_Wh": 39.083333333333336,
    "peak_electrical_power_W": 110.0220397395067,
    "peak_motor_torque_Nm": 0.07452955630903604,
    "max_rotor_speed_rad_s": 284.46867688977085,
    "torque_saturation_samples": 0,
    "allocation_saturation_samples": 0,
    "tilt_limit_samples": 0,
    "max_abs_roll_pitch_deg": 0.0,
    "checks": {
      "energy_within_usable": true,
      "hover_120_s_at_1_m": true
    },
    "tolerances": {
      "position_m": 0.02,
      "circle_radius_m": 0.02,
      "speed_mps": 0.02,
      "stop_speed_mps": 0.02,
      "yaw_deg": 1.0,
      "straight_cross_track_m": 0.02
    },
    "max_altitude_error_m": 0.0,
    "max_horizontal_position_error_m": 0.0,
    "max_speed_mps": 0.0,
    "all_checks_passed": true
  },
  {
    "mission": "circle",
    "duration_s": 70.0,
    "time_step_s": 0.01,
    "energy_used_Wh": 2.1395722104541766,
    "energy_usable_Wh": 39.083333333333336,
    "peak_electrical_power_W": 110.05755268657097,
    "peak_motor_torque_Nm": 0.0750979341239801,
    "max_rotor_speed_rad_s": 284.5507000376656,
    "torque_saturation_samples": 0,
    "allocation_saturation_samples": 0,
    "tilt_limit_samples": 0,
    "max_abs_roll_pitch_deg": 1.0739109203549952,
    "checks": {
      "energy_within_usable": true,
      "circle_60_s_radius_2_m": true,
      "speed_0p5_mps": true,
      "altitude_1_m": true
    },
    "tolerances": {
      "position_m": 0.02,
      "circle_radius_m": 0.02,
      "speed_mps": 0.02,
      "stop_speed_mps": 0.02,
      "yaw_deg": 1.0,
      "straight_cross_track_m": 0.02
    },
    "evaluation_window_s": [
      10.0,
      70.0
    ],
    "max_radius_error_m": 1.1528957937212425e-05,
    "max_speed_error_mps": 2.9034287999385455e-06,
    "mean_speed_mps": 0.4999971318058472,
    "max_altitude_error_m": 1.1342012207204988e-07,
    "all_checks_passed": true
  },
  {
    "mission": "sequence",
    "duration_s": 163.0,
    "time_step_s": 0.01,
    "energy_used_Wh": 4.826109260223477,
    "energy_usable_Wh": 39.083333333333336,
    "peak_electrical_power_W": 406.9627744041629,
    "peak_motor_torque_Nm": 0.3,
    "max_rotor_speed_rad_s": 312.4183222229182,
    "torque_saturation_samples": 76,
    "allocation_saturation_samples": 0,
    "tilt_limit_samples": 0,
    "max_abs_roll_pitch_deg": 6.713932901986687,
    "checks": {
      "energy_within_usable": true,
      "leg_1_distance": true,
      "leg_1_average_speed": true,
      "leg_1_altitude": true,
      "leg_1_straight": true,
      "leg_1_stop": true,
      "leg_2_distance": true,
      "leg_2_average_speed": true,
      "leg_2_altitude": true,
      "leg_2_straight": true,
      "leg_2_stop": true,
      "vertical_takeoff_to_1_m": true,
      "yaw_90_deg_left": true,
      "yaw_while_hovering": true,
      "touchdown_occurred": true,
      "vertical_landing": true,
      "descent_at_most_0p01_mps": true
    },
    "tolerances": {
      "position_m": 0.02,
      "circle_radius_m": 0.02,
      "speed_mps": 0.02,
      "stop_speed_mps": 0.02,
      "yaw_deg": 1.0,
      "straight_cross_track_m": 0.02
    },
    "liftoff_time_s": 1.02,
    "touchdown": {
      "time_s": 157.0,
      "vz_before_contact_mps": -0.007999999999999912,
      "horizontal_speed_mps": 4.025798651137404e-14,
      "position_m": [
        5.0000000000000275,
        5.000000000000019,
        0.0
      ]
    },
    "legs": [
      {
        "window_s": [
          8,
          13
        ],
        "axis": "x",
        "displacement_m": 4.997027069451866,
        "average_horizontal_speed_mps": 0.9994242128030033,
        "max_altitude_error_m": 0.0007197781153354121,
        "max_cross_track_error_m": 3.9770030041972775e-07,
        "speed_at_leg_end_mps": 0.0018280176985199595,
        "speed_after_stop_mps": 0.00011302535369440593
      },
      {
        "window_s": [
          23,
          28
        ],
        "axis": "y",
        "displacement_m": 4.997027069257337,
        "average_horizontal_speed_mps": 0.9994242127657988,
        "max_altitude_error_m": 0.0007230954232380071,
        "max_cross_track_error_m": 3.9821056052602444e-07,
        "speed_at_leg_end_mps": 0.0018280175181991615,
        "speed_after_stop_mps": 0.00011302535500341036
      }
    ],
    "yaw_change_deg": 90.00000000375721,
    "max_yaw_turn_position_drift_m": 5.3072366616515544e-05,
    "maximum_descent_speed_mps": 0.008020113651167338,
    "max_landing_horizontal_drift_m": 5.0657287163009964e-05,
    "max_takeoff_horizontal_displacement_m": 1.6630136285774377e-16,
    "all_checks_passed": true
  }
]
```