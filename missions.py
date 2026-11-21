"""Smooth reference trajectories; these never overwrite the simulated state."""
import numpy as np
DURATIONS = {'hover': 120., 'circle': 70., 'sequence': 163.}
CIRCLE_WINDOW = (10., 70.)  # 60 s after spin-up/settling, all included in energy
SEQUENCE_EVENTS = [(0,'Ground'),(1,'Takeoff'),(5,'Hover'),(8,'Leg 1'),
                   (13,'Stop'),(16,'Yaw left'),(20,'Hover'),(23,'Leg 2'),
                   (28,'Stop'),(31,'Land')]

def smooth(t, start, duration):
    a = np.clip((t-start)/duration,0.,1.)
    h = 10*a**3-15*a**4+6*a**5
    hd = (30*a**2-60*a**3+30*a**4)/duration
    hdd = (60*a-180*a**2+120*a**3)/duration**2
    return h, hd, hdd

def reference(t, mission):
    pos=np.array([0.,0.,1.]); vel=np.zeros(3); acc=np.zeros(3); yaw=0.; yaw_rate=0.
    if mission=='circle':
        # Ramp angular speed from 0 to .25 rad/s in 5 s on a 2 m circle.
        a=np.clip(t/5.,0.,1.)
        h,hd,_=smooth(t,0.,5.)
        angle=.25*(5*(2.5*a**4-3*a**5+a**6)+max(t-5,0))
        w=.25*h; wd=.25*hd
        c,s=np.cos(angle),np.sin(angle)
        pos[:2]=[2*c,2*s];vel[:2]=[-2*s*w,2*c*w]
        acc[:2]=[-2*c*w*w-2*s*wd,-2*s*w*w+2*c*wd]
    elif mission=='sequence':
        h,hd,hdd=smooth(t,1.,4.);pos[2]=h;vel[2]=hd;acc[2]=hdd
        for axis,start in [(0,8.),(1,23.)]:
            h,hd,hdd=smooth(t,start,5.)
            pos[axis]=5*h;vel[axis]=5*hd;acc[axis]=5*hdd
        h,hd,_=smooth(t,16.,4.);yaw=np.pi/2*h;yaw_rate=np.pi/2*hd
        if t>=31:
            # Smoothly ramp to .008 m/s descent (< .01 m/s requirement).
            # Continue reference below zero until physical contact triggers disarm.
            a=np.clip((t-31)/2.,0.,1.)
            h,hd,_=smooth(t,31.,2.)
            distance=.008*(2*(2.5*a**4-3*a**5+a**6)+max(t-33,0))
            pos[2]=1-distance;vel[2]=-.008*h;acc[2]=-.008*hd
    elif mission!='hover':
        raise ValueError(mission)
    return pos,vel,acc,yaw,yaw_rate
