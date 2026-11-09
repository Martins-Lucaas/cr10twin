"""Réplica float32 do update_taxels() do firmware 5x5 (CHANGELOG [15])."""
import numpy as np, pandas as pd
DT=np.float32(0.2); VTH=30.0; NB=25
SA=dict(a=0.02,b=0.2,c=-65.0,d=8.0); RA=dict(a=0.1,b=0.2,c=-65.0,d=2.0)

def load(R):
    a=pd.read_csv(R+'adc.csv').iloc[:-1]
    t=a.tempo.values; X=a[[f'taxel_{i}' for i in range(25)]].values.astype(np.float32)
    s=pd.read_csv(R+'samples.csv',usecols=['t_unix','phase','cycle','force_net_n']).iloc[:-1]
    se=pd.read_csv(R+'sensors.csv',usecols=['t_unix','touch_t_stm_s']).iloc[:-1]
    # MCU -> unix via ajuste linear (relógio monotônico neste run)
    k=np.polyfit(se.touch_t_stm_s.values,se.t_unix.values,1)
    tu=np.polyval(k,t)
    j=np.searchsorted(s.t_unix.values,tu).clip(1,len(s)-1)-1
    return t,X,s.phase.values[j],s.cycle.values[j],s.force_net_n.values[j]

def run(X,base,gsa,gra,frame_ms=1.0,dead=0.001):
    """X: (T,25) adc; gsa/gra: arrays (G,) -> spikes SA,RA (T,G,25) bool como contagem por taxel."""
    gsa=np.asarray(gsa,np.float32)[:,None]; gra=np.asarray(gra,np.float32)[:,None]
    G=max(gsa.shape[0],gra.shape[0]); T=len(X)
    Vn=np.clip((base[None,:]-X)/base[None,:],0,None).astype(np.float32)
    vS=np.full((G,25),-30,np.float32); uS=SA['b']*vS; vR=vS.copy(); uR=RA['b']*vR
    buf=np.zeros((NB,25),np.float32)
    nS=np.zeros((T,G,25),bool); nR=np.zeros((T,G,25),bool)
    for n in range(T):
        I=Vn[n]; old=buf[n%NB].copy(); buf[n%NB]=I
        dI=np.abs(I-old)/np.float32(NB*frame_ms); dI[dI<dead]=0
        for v,u,Iin,p,out in ((vS,uS,gsa*I,SA,nS),(vR,uR,gra*dI,RA,nR)):
            v+=DT*(0.04*v*v+5*v+140-u+Iin); u+=DT*(p['a']*(p['b']*v-u))
            f=v>=VTH; v[f]=p['c']; u[f]+=p['d']; out[n]=f
    return nS,nR


if __name__ == '__main__':
    # Uso: python3 sim_neuronios.py <run_dir/> [G_SA G_RA]
    # Confere contra o spikes.csv do próprio run antes de confiar numa varredura.
    import sys
    R = sys.argv[1].rstrip('/') + '/'
    gsa, gra = (float(sys.argv[2]), float(sys.argv[3])) if len(sys.argv) > 3 else (120, 16000)
    t, X, ph, cy, F = load(R)
    base = np.median(X[(ph == 0) & (np.abs(F) < 0.05)], 0).astype(np.float32)
    nS, nR = run(X, base, [gsa], [gra])
    for k, m in {'repouso': ((ph == 0) | (ph == 1)) & (np.abs(F) < 0.05),
                 'HOLD': ph == 2, 'SLIDE': ph == 3}.items():
        d = m.sum() / 1000
        print(f'{k:8s} SA {nS[m].sum() / d:7.2f} Hz  RA {nR[m].sum() / d:7.2f} Hz')
