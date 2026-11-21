"""Run all missions, export measured data, metrics, PNG/PDF figures and LaTeX."""
from pathlib import Path
import argparse,json
import numpy as np
from parameters import PARAMS
from simulation import run
from metrics import measure
from plotting import figures

STATE_NAMES=['x_m','y_m','z_m','vx_mps','vy_mps','vz_mps','phi_rad','theta_rad','psi_rad','p_radps','q_radps','r_radps','Omega1_radps','Omega2_radps','Omega3_radps','Omega4_radps']

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--dt',type=float,default=.01)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'outputs')
    parser.add_argument('--no-plots',action='store_true');args=parser.parse_args()
    if not 0<args.dt<=.02:parser.error('Choose 0 < dt <= 0.02 s.')
    out=args.output;out.mkdir(parents=True,exist_ok=True);all_data=[];all_metrics=[]
    for name in ['hover','circle','sequence']:
        print(f'Running {name} ...',flush=True);d=run(name,PARAMS,args.dt);m=measure(d,PARAMS)
        all_data.append(d);all_metrics.append(m)
        np.savez_compressed(out/f'{name}.npz',**{k:v for k,v in d.items() if isinstance(v,np.ndarray)})
        table=np.column_stack([d['t'],d['s'],d['u'],d['ref'],d['omega_cmd'],d['att_cmd'],d['power'],d['energy'],d['flags']])
        header=['time_s']+STATE_NAMES+[f'M{i}_Nm' for i in range(1,5)]+['x_ref_m','y_ref_m','z_ref_m','vx_ref_mps','vy_ref_mps','vz_ref_mps','yaw_ref_rad']+[f'Omega{i}_cmd_radps' for i in range(1,5)]+['phi_cmd_rad','theta_cmd_rad','psi_cmd_rad','power_W','energy_J','allocation_limited','torque_limited','tilt_limited']
        np.savetxt(out/f'{name}.csv',table,delimiter=',',header=','.join(header),comments='',fmt='%.10g')
        print(f'  {"PASS" if m["all_checks_passed"] else "CHECK FAILED"}: {m["energy_used_Wh"]:.3f} Wh; metrics saved',flush=True)
    params={k:v.tolist() if isinstance(v,np.ndarray) else v for k,v in PARAMS.items()}
    (out/'parameters_used.json').write_text(json.dumps(params,indent=2))
    (out/'metrics.json').write_text(json.dumps(all_metrics,indent=2))
    text=['# Simulation results','',
          'These are model predictions for the selected components and documented parameter assumptions, not validated hardware performance.',
          'Each mission starts with a fresh battery. No wind, sensor noise or disturbances are applied.',
          'Engineering tolerances are in parameters.py and metrics.json; they are not instructor-specified.',
          '', '| Mission | Energy used (Wh) | Usable energy (Wh) | Checks |','|---|---:|---:|---|']
    for m in all_metrics:text.append(f'| {m["mission"]} | {m["energy_used_Wh"]:.4f} | {m["energy_usable_Wh"]:.3f} | {"Pass" if m["all_checks_passed"] else "Fail; inspect metrics.json"} |')
    total=sum(m['energy_used_Wh'] for m in all_metrics)
    text+=['',f'Sum of the three separate mission energies: {total:.4f} Wh. This excludes any connecting travel; it is not a continuous combined-mission test.',
           '', '## Full numerical results','', '```json',json.dumps(all_metrics,indent=2),'```']
    (out/'RESULTS.md').write_text('\n'.join(text))
    if not args.no_plots:figures(all_data,all_metrics,PARAMS,out/'figures')
    tex=[r'% Add \usepackage{graphicx} to your report preamble.',r'% Upload the outputs/figures folder preserving its path, then input this file.',r'\subsection{Numerical simulation results}',
         r'The following results use the selected hardware parameters and documented modeling assumptions. The 16-state model is integrated using fixed-step fourth-order Runge--Kutta with a '+f'{args.dt:g}'+r'~s step. Motor torques are updated once per step and held during its four stages. Position and attitude feedback, thrust/moment allocation, and rotor-speed feedback generate the motor torque inputs.',
         r'Each mission begins with a fresh battery. No wind or sensor noise is included. Hover starts at its exact equilibrium. Circular-flight statistics cover $t=10$--$70$~s; energy includes the initial acceleration. The two straight legs use smooth 5~s trajectories, so 1~m/s is an average, not a constant speed. Ground contact is an ideal, non-bouncing constraint; landing speed is measured before the contact velocity reset.',
         r'The yaw model retains the report assumption that transient motor reaction and rotor gyroscopic terms are neglected. Usable battery energy is '+f'{PARAMS["E_usable"]/3600:.3f}'+r'~Wh.']
    captions={'six_dof':'Position and Euler angles versus time. Solid curves are simulated states; dashed curves are references or attitude commands.',
              'performance':'Actual and reference paths, horizontal speed, tracking error, and consumed electrical energy.',
              'actuators':'Actual rotor speeds, applied motor torques, electrical power, and consumed fraction of usable battery energy.',
              'detail':'Straight-leg speeds and measured averages, left yaw maneuver, and descent-speed limit.'}
    for name in ['hover','circle','sequence']:
        for suffix in ['six_dof','performance','actuators']+(['detail'] if name=='sequence' else []):
            tex += [r'\begin{figure}[htbp]',r'\centering',r'\includegraphics[width=\textwidth]{outputs/figures/'+name+'_'+suffix+r'.pdf}',r'\caption{'+name.capitalize()+': '+captions[suffix]+r'}',r'\label{fig:'+name+'-'+suffix+r'}',r'\end{figure}']
    (out/'report_figures.tex').write_text('\n'.join(tex))
    print(f'Outputs: {out.resolve()}')
    if not all(m['all_checks_passed'] for m in all_metrics):raise SystemExit('Some targets failed: see metrics.json.')

if __name__=='__main__':main()
