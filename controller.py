"""Cascaded position/attitude control, rotor allocation and torque control."""
import numpy as np
from missions import reference

def rotation(angles):
    f,h,y=angles;sf,cf=np.sin(f),np.cos(f);sh,ch=np.sin(h),np.cos(h);sy,cy=np.sin(y),np.cos(y)
    return np.array([[cy*ch,cy*sh*sf-sy*cf,cy*sh*cf+sy*sf],
                     [sy*ch,sy*sh*sf+cy*cf,sy*sh*cf-cy*sf],[-sh,ch*sf,ch*cf]])

def desired_rotation(force,yaw):
    b3=force/np.linalg.norm(force)
    heading=np.array([np.cos(yaw),np.sin(yaw),0.])
    b2=np.cross(b3,heading);b2/=np.linalg.norm(b2)
    return np.column_stack((np.cross(b2,b3),b2,b3))

def vee(A):
    return np.array([A[2,1],A[0,2],A[1,0]])

def control(t,s,P,mission):
    pos,vel,acc,yaw,yr=reference(t,mission)
    ac=acc+P['kp_pos']*(pos-s[:3])+P['kd_pos']*(vel-s[3:6])
    force=P['m']*(ac+np.array([0.,0.,P['g']]))
    force[2]=max(force[2],.15*P['m']*P['g'])
    max_horizontal=force[2]*np.tan(P['tilt_max'])
    norm=np.linalg.norm(force[:2])
    tilt_limited=norm>max_horizontal
    if tilt_limited:force[:2]*=max_horizontal/norm
    Rd=desired_rotation(force,yaw); R=rotation(s[6:9])
    # Reference-only angular velocity feedforward; no differentiation of state noise.
    eps=.001
    nominal=[]
    for tj in (t-eps,t,t+eps):
        _,_,a,jyaw,_=reference(tj,mission)
        nominal.append(desired_rotation(a+np.array([0,0,P['g']]),jyaw))
    W=nominal[1].T@((nominal[2]-nominal[0])/(2*eps))
    wd=vee((W-W.T)/2)
    er=vee((Rd.T@R-R.T@Rd)/2)
    ew=s[9:12]-R.T@Rd@wd
    inertia=np.array([P['Ixx'],P['Iyy'],P['Izz']])
    tau=inertia*(-P['kp_att']*er-P['kd_att']*ew)+np.cross(s[9:12],inertia*s[9:12])
    thrust=max(np.dot(force,R[:,2]),0.)
    # Maps squared actual rotor speeds to [T,tau_x,tau_y,tau_z].
    kT,kQ,L=P['kT'],P['kQ'],P['L']
    B=np.array([[kT,kT,kT,kT],[L*kT,0,-L*kT,0],
                [0,-L*kT,0,L*kT],[-kQ,kQ,-kQ,kQ]])
    raw=np.linalg.solve(B,np.r_[thrust,tau])
    square=np.clip(raw,0,P['Omega_max']**2)
    cmd=np.sqrt(square)
    raw_u=kQ*s[12:]**2+P['Jr']*(cmd-s[12:])/P['motor_time_constant']
    u=np.clip(raw_u,0,P['M_max'])  # no regeneration; zero torque permits aerodynamic coast-down
    angles=np.array([np.arctan2(Rd[2,1],Rd[2,2]),np.arcsin(-Rd[2,0]),np.arctan2(Rd[1,0],Rd[0,0])])
    flags=np.array([np.any(raw!=square),np.any(raw_u!=u),tilt_limited],dtype=float)
    return u,cmd,angles,flags
