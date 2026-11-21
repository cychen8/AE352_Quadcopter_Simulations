"""Measured quantities and explicit, editable engineering tolerances."""
import numpy as np
from parameters import TOLERANCES

def measure(d,P):
    t,s,ref=d['t'],d['s'],d['ref'];tol=TOLERANCES
    m={'mission':d['name'],'duration_s':float(t[-1]),'time_step_s':d['dt'],
       'energy_used_Wh':float(d['energy'][-1]/3600),'energy_usable_Wh':P['E_usable']/3600,
       'peak_electrical_power_W':float(max(d['power'])),
       'peak_motor_torque_Nm':float(d['u'].max()),'max_rotor_speed_rad_s':float(s[:,12:].max()),
       'torque_saturation_samples':int(d['flags'][:,1].sum()),
       'allocation_saturation_samples':int(d['flags'][:,0].sum()),
       'tilt_limit_samples':int(d['flags'][:,2].sum()),
       'max_abs_roll_pitch_deg':float(np.rad2deg(np.abs(s[:,6:8])).max()),
       'checks':{},'tolerances':tol}
    checks=m['checks'];checks['energy_within_usable']=bool(d['energy'][-1]<=P['E_usable'])
    if d['name']=='hover':
        m['max_altitude_error_m']=float(max(abs(s[:,2]-1)))
        m['max_horizontal_position_error_m']=float(max(np.linalg.norm(s[:,:2],axis=1)))
        m['max_speed_mps']=float(max(np.linalg.norm(s[:,3:6],axis=1)))
        checks['hover_120_s_at_1_m']=bool(t[-1]>=120 and m['max_altitude_error_m']<=tol['position_m'] and m['max_horizontal_position_error_m']<=tol['position_m'] and m['max_speed_mps']<=tol['stop_speed_mps'])
    elif d['name']=='circle':
        k=t>=10;m['evaluation_window_s']=[10.,70.]
        radius=np.linalg.norm(s[k,:2],axis=1);speed=np.linalg.norm(s[k,3:5],axis=1)
        m.update(max_radius_error_m=float(max(abs(radius-2))),max_speed_error_mps=float(max(abs(speed-.5))),
                 mean_speed_mps=float(np.mean(speed)),max_altitude_error_m=float(max(abs(s[k,2]-1))))
        checks['circle_60_s_radius_2_m']=bool(t[-1]-10>=60 and m['max_radius_error_m']<=tol['circle_radius_m'])
        checks['speed_0p5_mps']=m['max_speed_error_mps']<=tol['speed_mps']
        checks['altitude_1_m']=m['max_altitude_error_m']<=tol['position_m']
    else:
        at=lambda tt: s[np.argmin(abs(t-tt))]
        m['liftoff_time_s']=d['liftoff'];m['touchdown']=d['touchdown']
        legs=[]
        for a,b,axis,stop in [(8,13,0,16),(23,28,1,31)]:
            k=(t>=a)&(t<=b)
            leg={'window_s':[a,b], 'axis':'x' if axis==0 else 'y',
                 'displacement_m':float(at(b)[axis]-at(a)[axis]),
                 'average_horizontal_speed_mps':float(np.trapezoid(np.linalg.norm(s[k,3:5],axis=1),t[k])/(b-a)),
                 'max_altitude_error_m':float(max(abs(s[k,2]-1))),
                 'max_cross_track_error_m':float(max(abs(s[k,1-axis]-at(a)[1-axis]))),
                 'speed_at_leg_end_mps':float(np.linalg.norm(at(b)[3:6])),
                 'speed_after_stop_mps':float(np.linalg.norm(at(stop)[3:6]))}
            legs.append(leg)
            checks[f'leg_{axis+1}_distance']=abs(leg['displacement_m']-5)<=tol['position_m']
            checks[f'leg_{axis+1}_average_speed']=abs(leg['average_horizontal_speed_mps']-1)<=tol['speed_mps']
            checks[f'leg_{axis+1}_altitude']=leg['max_altitude_error_m']<=tol['position_m']
            checks[f'leg_{axis+1}_straight']=leg['max_cross_track_error_m']<=tol['straight_cross_track_m']
            checks[f'leg_{axis+1}_stop']=leg['speed_after_stop_mps']<=tol['stop_speed_mps']
        m['legs']=legs
        m['yaw_change_deg']=float(np.rad2deg(at(23)[8]-at(16)[8]))
        m['max_yaw_turn_position_drift_m']=float(max(np.linalg.norm(s[(t>=16)&(t<=23),:3]-at(16)[:3],axis=1)))
        td=d['touchdown'];land=(t>=31)&(t< (td['time_s'] if td else t[-1]))
        m['maximum_descent_speed_mps']=float(max(np.maximum(-s[land,5],0)))
        m['max_landing_horizontal_drift_m']=float(max(np.linalg.norm(s[land,:2]-at(31)[:2],axis=1)))
        take=(t<=8)
        m['max_takeoff_horizontal_displacement_m']=float(max(np.linalg.norm(s[take,:2],axis=1)))
        checks['vertical_takeoff_to_1_m']=bool(d['liftoff'] is not None and abs(at(8)[2]-1)<=tol['position_m'] and m['max_takeoff_horizontal_displacement_m']<=tol['position_m'])
        checks['yaw_90_deg_left']=abs(m['yaw_change_deg']-90)<=tol['yaw_deg']
        checks['yaw_while_hovering']=m['max_yaw_turn_position_drift_m']<=tol['position_m']
        checks['touchdown_occurred']=td is not None
        checks['vertical_landing']=m['max_landing_horizontal_drift_m']<=tol['position_m']
        checks['descent_at_most_0p01_mps']=bool(td is not None and m['maximum_descent_speed_mps']<=.01 and abs(td['vz_before_contact_mps'])<=.01)
    m['all_checks_passed']=bool(all(checks.values()))
    return m
