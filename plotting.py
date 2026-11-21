"""Report figures: actual/reference motion, all six DOF, motor inputs and energy."""
import matplotlib
matplotlib.use('Agg')  # save figures reliably without requiring GUI windows
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.backends.backend_pdf import PdfPages

BLUE='#1864ab';ORANGE='#d97706'
plt.rcParams.update({'font.size':10,'axes.titlesize':11,'axes.labelsize':10,
                     'axes.spines.top':False,'axes.spines.right':False,'savefig.dpi':170})
TITLES={'hover':'Mission 1 | Stationary hover','circle':'Mission 2 | Circular flight',
        'sequence':'Mission 3 | Flight sequence'}

def save(fig,out,name,book):
    fig.savefig(out/f'{name}.png',bbox_inches='tight')
    fig.savefig(out/f'{name}.pdf',bbox_inches='tight')
    book.savefig(fig,bbox_inches='tight');plt.close(fig)

def decorate(ax,x='Time (s)'):
    ax.set_xlabel(x);ax.grid(alpha=.22)

def figures(data,metrics,P,out):
    out.mkdir(parents=True,exist_ok=True)
    with PdfPages(out/'all_mission_figures.pdf') as book:
        for d,m in zip(data,metrics):
            name=d['name'];t=d['t'];s=d['s'];ref=d['ref'].copy();prefix=TITLES[name]
            if d['touchdown']:
                ref[t>=d['touchdown']['time_s'],2]=0
                ref[t>=d['touchdown']['time_s'],3:6]=0
            fig,axes=plt.subplots(2,3,figsize=(12,6.5),layout='constrained')
            fig.suptitle(prefix+' | Six degrees of freedom\n',fontsize=14)
            for j,ax in enumerate(axes[0]):
                ax.plot(t,s[:,j],color=BLUE,label='Actual')
                ax.plot(t,ref[:,j],color=ORANGE,ls='--',label='Reference')
                ax.set_ylabel(['x (m)','y (m)','z (m)'][j]);decorate(ax)
                if np.ptp(s[:,j])<.01:ax.set_ylim(s[0,j]-.025,s[0,j]+.025)
            for j,ax in enumerate(axes[1]):
                ax.plot(t,np.rad2deg(s[:,j+6]),color=BLUE,label='Actual')
                ax.plot(t,np.rad2deg(d['att_cmd'][:,j]),color=ORANGE,ls='--',label='Attitude command')
                ax.set_ylabel(['Roll (deg)','Pitch (deg)','Yaw (deg)'][j]);decorate(ax)
                if np.ptp(s[:,j+6])<.001:ax.set_ylim(-1,1)
            axes[0,0].legend();axes[1,0].legend()
            save(fig,out,name+'_six_dof',book)
            fig,ax=plt.subplots(2,2,figsize=(11,7),layout='constrained')
            fig.suptitle(prefix+' | Path and performance\n',fontsize=14)
            ax[0,0].plot(s[:,0],s[:,1],color=BLUE,label='Actual')
            ax[0,0].plot(ref[:,0],ref[:,1],color=ORANGE,ls='--',label='Reference')
            ax[0,0].scatter(s[0,0],s[0,1],marker='o',color=BLUE,label='Start',zorder=4)
            ax[0,0].scatter(s[-1,0],s[-1,1],marker='x',color='black',label='End',zorder=5)
            ax[0,0].set_aspect('equal',adjustable='box');ax[0,0].set_ylabel('y (m)');decorate(ax[0,0],'x (m)');ax[0,0].legend(fontsize=8)
            if name=='hover':ax[0,0].set(xlim=(-.05,.05),ylim=(-.05,.05))
            speed=np.linalg.norm(s[:,3:5],axis=1);vs=np.linalg.norm(ref[:,3:5],axis=1)
            ax[0,1].plot(t,speed,color=BLUE,label='Actual horizontal speed')
            ax[0,1].plot(t,vs,color=ORANGE,ls='--',label='Reference')
            ax[0,1].set_ylabel('Horizontal speed (m/s)');decorate(ax[0,1]);ax[0,1].legend(fontsize=8)
            if name=='sequence':ax[0,1].set_xlim(0,31)
            if name=='circle':
                ax[0,1].axvspan(10,70,alpha=.08,color=BLUE);ax[0,1].set_title('Scored window: 10-70 s')
                ax[1,0].plot(t,np.linalg.norm(s[:,:2],axis=1)-2,color=BLUE)
                ax[1,0].set_ylabel('Radius error (m)');ax[1,0].axvspan(10,70,alpha=.08,color=BLUE)
            else:
                visible=ref[:,2].copy()
                if d['touchdown']:visible[t>=d['touchdown']['time_s']]=0
                ax[1,0].plot(t,s[:,2]-visible,color=BLUE);ax[1,0].set_ylabel('Altitude tracking error (m)')
            decorate(ax[1,0])
            ax[1,1].plot(t,d['energy']/3600,color=BLUE,label='Used')
            ax[1,1].axhline(P['E_usable']/3600,color=ORANGE,ls='--',label='Usable capacity')
            ax[1,1].set_ylabel('Electrical energy (Wh)');decorate(ax[1,1]);ax[1,1].legend()
            save(fig,out,name+'_performance',book)
            fig,ax=plt.subplots(2,2,figsize=(11,7),layout='constrained')
            fig.suptitle(prefix+' | Actuators and electrical power\n',fontsize=14)
            for j in range(4):
                ax[0,0].plot(t,s[:,12+j]*60/(2*np.pi),label=f'Rotor {j+1}',lw=1)
                ax[0,1].plot(t,d['u'][:,j],label=f'Motor {j+1}',lw=1)
            ax[0,0].axhline(P['Omega_max']*60/(2*np.pi),color='black',ls=':',label='Speed command limit')
            ax[0,0].set_ylabel('Actual rotor speed (rpm)');ax[0,0].legend(fontsize=8)
            ax[0,1].axhline(P['M_max'],color='black',ls=':',label='Torque limit');ax[0,1].set_ylabel('Motor torque (N m)');ax[0,1].legend(fontsize=8)
            ax[1,0].plot(t,d['power'],color=BLUE);ax[1,0].set_ylabel('Electrical power (W)')
            ax[1,1].plot(t,100*d['energy']/P['E_usable'],color=BLUE);ax[1,1].set_ylabel('Usable energy consumed (%)')
            for a in ax.flat:decorate(a)
            save(fig,out,name+'_actuators',book)
            if name=='sequence':
                fig,ax=plt.subplots(2,2,figsize=(11,7),layout='constrained')
                fig.suptitle('Mission 3 | Requirement checks in detail\n',fontsize=14)
                for j,(a,b) in enumerate([(8,13),(23,28)]):
                    k=(t>=a)&(t<=b)
                    ax[0,j].plot(t[k]-a,speed[k],color=BLUE,label='Actual speed')
                    ax[0,j].plot(t[k]-a,vs[k],color=ORANGE,ls='--',label='Reference')
                    avg=m['legs'][j]['average_horizontal_speed_mps']
                    ax[0,j].axhline(avg,color='black',ls=':',label=f'Actual average: {avg:.4f} m/s')
                    ax[0,j].set(title=f'Leg {j+1}: {m["legs"][j]["displacement_m"]:.4f} m in 5 s',ylabel='Horizontal speed (m/s)')
                    decorate(ax[0,j],'Time within leg (s)');ax[0,j].legend(fontsize=8)
                k=(t>=13)&(t<=23)
                ax[1,0].plot(t[k],np.rad2deg(s[k,8]),color=BLUE,label='Actual yaw')
                ax[1,0].plot(t[k],np.rad2deg(ref[k,6]),color=ORANGE,ls='--',label='Reference')
                ax[1,0].set_ylabel('Yaw (deg)');decorate(ax[1,0]);ax[1,0].legend()
                k=(t>=31)&(t<d['touchdown']['time_s']) if d['touchdown'] else t>=31
                ax[1,1].plot(t[k],-s[k,5],color=BLUE,label='Actual descent speed')
                ax[1,1].axhline(.01,color='#b91c1c',ls='--',label='Project maximum: 0.01 m/s')
                ax[1,1].set_ylim(-.001,.011)
                ax[1,1].set_ylabel('Downward speed (m/s)');decorate(ax[1,1]);ax[1,1].legend(fontsize=8)
                save(fig,out,'sequence_detail',book)
