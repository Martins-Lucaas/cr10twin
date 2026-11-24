import numpy as np, sys
sys.path.insert(0,'src/grasp_ml_pack')
from grasp_ml_pack.kinematics import hand_fk, HAND_LIMITS
F=('Thumb','Index','Middle','Ring','Little')
cs=np.linspace(0,1,41)
def tips(f,rot):
    out=[]
    for c in cs:
        st={g:0.0 for g in F}; st[f]=c*HAND_LIMITS[f]; st['Rotate']=rot*HAND_LIMITS['Rotate']
        out.append(hand_fk(st)['tip_'+f])
    return np.array(out)
rot=float(sys.argv[1])
T={f:tips(f,rot) for f in F}
D=0.023
res=[]
for y0 in np.arange(0.03,0.08,0.005):
  for z0 in (0.03,0.033,0.036):
    for x0 in (-0.01,):
      n=0;info=[]
      for f in F:
        t=T[f]; d=np.hypot(t[:,1]-y0,t[:,2]-z0); ok=np.abs(t[:,0]-x0)<0.055
        idx=np.where((d<=D)&ok)[0]
        if len(idx):
            n+=1; info.append((f[:2],round(cs[idx[0]],2)))
      # clearance at open: tip z of open fingers vs tube bottom
      res.append((n,y0,z0,x0,info))
res.sort(key=lambda r:-r[0])
for r in res[:25]: print(r)
