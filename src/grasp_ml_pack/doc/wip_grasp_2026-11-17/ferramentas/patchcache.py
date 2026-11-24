import re,subprocess,sys
f='src/grasp_ml_pack/grasp_ml_pack/poses.py'
out=subprocess.check_output(['python3','recompute.py'],text=True,stderr=subprocess.DEVNULL)
vals={}
for l in out.splitlines():
    n,d=l.split(' ',1); vals[n]=eval(d)
s=open(f).read()
for name,cache in (('PRE_APPROACH_POSES_DEG','_CACHED_PRE_APPROACH_POSES_DEG'),('APPROACH_POSES_DEG','_CACHED_APPROACH_POSES_DEG'),('PICK_POSES_DEG','_CACHED_PICK_POSES_DEG')):
    v=vals[name]
    new="    'tubo': {'joint1': %9.4f, 'joint2': %9.4f, 'joint3': %9.4f,\n               'joint4': %9.4f, 'joint5': %9.4f, 'joint6': %9.4f},\n"%tuple(v['joint%d'%i] for i in range(1,7))
    i=s.index(cache+' = {'); j=s.index("    'tubo':",i); k=s.index('},\n',j)+3
    s=s[:j]+new+s[k:]
open(f,'w').write(s)
