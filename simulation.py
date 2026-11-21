"""Fixed-step RK4; controller updated once per step, motor torque held during step."""
import numpy as np
from drone_dynamics import drone_dynamics
from missions import DURATIONS, reference
from controller import control

def power(s,u,P):
    return float(np.dot(u,s[12:])/(P['eta_m']*P['eta_ESC'])+P['P_aux'])

def run(mission,P,dt=.01):
    n=int(np.ceil(DURATIONS[mission]/dt));dt=DURATIONS[mission]/n
    t=np.linspace(0,DURATIONS[mission],n+1)
    s=np.zeros(16);s[:3]=reference(0,mission)[0]
    if mission!='sequence':s[12:]=np.sqrt(P['m']*P['g']/(4*P['kT']))
    states=np.empty((n+1,16));inputs=np.empty((n+1,4));commands=np.empty((n+1,4))
    angles=np.empty((n+1,3));flags=np.zeros((n+1,3));energy=np.zeros(n+1);powers=np.empty(n+1)
    landed=False;touchdown=None;lifted=False;liftoff=None
    for i,ti in enumerate(t):
        u,cmd,ang,flag=control(ti,s,P,mission)
        if landed:u[:]=0;cmd[:]=0;ang[:]=[0,0,np.pi/2];flag[:]=0
        states[i]=s;inputs[i]=u;commands[i]=cmd;angles[i]=ang;flags[i]=flag;powers[i]=power(s,u,P)
        if i==n:break
        ground=(mission=='sequence' and s[2]<=0 and not lifted) or landed
        def rhs(z):
            d=drone_dynamics(ti,z,u,P)
            if ground and (landed or d[5]<=0):d[:12]=0
            return d
        k1=rhs(s);s2=s+dt*k1/2;k2=rhs(s2);s3=s+dt*k2/2;k3=rhs(s3);s4=s+dt*k3;k4=rhs(s4)
        nxt=s+dt*(k1+2*k2+2*k3+k4)/6
        energy[i+1]=energy[i]+dt*(power(s,u,P)+2*power(s2,u,P)+2*power(s3,u,P)+power(s4,u,P))/6
        if energy[i+1]>P['E_usable']:raise RuntimeError(f'{mission}: battery exhausted at {ti+dt:.3f} s')
        if mission=='sequence' and not lifted and nxt[2]>0:
            lifted=True;liftoff=float(ti+dt)
        if mission=='sequence' and lifted and not landed and s[2]>0 and nxt[2]<=0:
            alpha=s[2]/(s[2]-nxt[2]);impact=s+alpha*(nxt-s)
            touchdown={'time_s':float(ti+alpha*dt),'vz_before_contact_mps':float(impact[5]),
                       'horizontal_speed_mps':float(np.linalg.norm(impact[3:5])),
                       'position_m':impact[:3].tolist()}
            nxt[:3]=impact[:3];nxt[2]=0;nxt[3:8]=0;nxt[9:12]=0;landed=True
        if ground and not lifted:nxt[2]=max(nxt[2],0.)
        if not np.all(np.isfinite(nxt)) or np.min(nxt[12:]) < -1e-8:
            raise RuntimeError('Invalid simulation state; reduce time step or inspect control.')
        s=nxt
    refs=np.array([np.r_[reference(tt,mission)[0],reference(tt,mission)[1],reference(tt,mission)[3]] for tt in t])
    return dict(name=mission,t=t,s=states,u=inputs,omega_cmd=commands,att_cmd=angles,
                flags=flags,energy=energy,power=powers,ref=refs,touchdown=touchdown,liftoff=liftoff,dt=dt)
