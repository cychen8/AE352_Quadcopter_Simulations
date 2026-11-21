"""Run after run_missions.py: independent force/moment checks and step refinement."""
from pathlib import Path
import json
import numpy as np
from scipy.spatial.transform import Rotation
from drone_dynamics import drone_dynamics
from parameters import PARAMS
from simulation import run
from metrics import measure

if __name__=='__main__':
    P=PARAMS;out=Path(__file__).resolve().parent/'outputs'
    if not (out/'metrics.json').exists():raise SystemExit('Run python run_missions.py first.')
    rng=np.random.default_rng(352);I=np.diag([P['Ixx'],P['Iyy'],P['Izz']])
    arms=np.array([[0,P['L'],0],[P['L'],0,0],[0,-P['L'],0],[-P['L'],0,0]])
    for _ in range(30):
        s=rng.uniform(-.3,.3,16);s[12:]=rng.uniform(300,700,4);u=rng.uniform(.02,.08,4)
        before=s.copy();ds=drone_dynamics(0,s,u,P)
        forces=np.zeros((4,3));forces[:,2]=P['kT']*s[12:]**2
        tau=np.cross(arms,forces).sum(axis=0);tau[2]=np.dot([-1,1,-1,1],P['kQ']*s[12:]**2)
        R=Rotation.from_euler('ZYX',[s[8],s[7],s[6]]).as_matrix()
        np.testing.assert_allclose(ds[3:6],R@forces.sum(axis=0)/P['m']-[0,0,P['g']],atol=1e-12)
        np.testing.assert_allclose(ds[9:12],np.linalg.solve(I,tau-np.cross(s[9:12],I@s[9:12])),atol=1e-12)
        f,h=s[6:8];W=np.array([[1,0,-np.sin(h)],[0,np.cos(f),np.sin(f)*np.cos(h)],[0,-np.sin(f),np.cos(f)*np.cos(h)]])
        np.testing.assert_allclose(W@ds[6:9],s[9:12],atol=1e-12)
        np.testing.assert_array_equal(s,before)
    print('PASS: 30 vector-form dynamics and kinematics checks.',flush=True)
    results=[];coarse_metrics=json.loads((out/'metrics.json').read_text())
    for name,cm in zip(['hover','circle','sequence'],coarse_metrics):
        base=np.load(out/f'{name}.npz');dt=cm['time_step_s']
        print(f'Refining {name}: {dt:g} -> {dt/2:g} s ...',flush=True)
        fine=run(name,P,dt/2);fm=measure(fine,P)
        aligned=np.column_stack([np.interp(base['t'],fine['t'],fine['s'][:,i]) for i in range(16)])
        delta=np.abs(aligned-base['s'])
        row={'mission':name,'coarse_dt_s':dt,'fine_dt_s':dt/2,
             'max_position_difference_m':float(delta[:,:3].max()),
             'max_attitude_difference_deg':float(np.rad2deg(delta[:,6:9]).max()),
             'max_rotor_speed_difference_radps':float(delta[:,12:].max()),
             'energy_difference_Wh':abs(fm['energy_used_Wh']-cm['energy_used_Wh']),
             'all_fine_mission_checks_passed':fm['all_checks_passed']}
        # Explicit numerical sensitivity targets, not assignment tolerances.
        row['refinement_check_passed']=bool(row['max_position_difference_m']<.005 and row['max_attitude_difference_deg']<.2 and row['energy_difference_Wh']<.01 and row['all_fine_mission_checks_passed'])
        results.append(row);print(json.dumps(row,indent=2),flush=True)
    result={'note':'Both RK4 step and controller update period are halved; this measures combined numerical/sampling sensitivity.',
            'vector_checks':30,'refinement':results}
    (out/'validation.json').write_text(json.dumps(result,indent=2))
    if not all(r['refinement_check_passed'] for r in results):raise SystemExit('Refinement check failed; inspect validation.json.')
    print('PASS: all numerical refinement checks.')
