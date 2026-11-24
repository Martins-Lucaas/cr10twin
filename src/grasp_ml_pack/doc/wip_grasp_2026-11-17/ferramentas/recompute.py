import os,sys
os.environ['GRASP_RECOMPUTE_POSES']='1'
sys.path.insert(0,'src/grasp_ml_pack')
from grasp_ml_pack import poses as p
for n in ('PRE_APPROACH_POSES_DEG','APPROACH_POSES_DEG','PICK_POSES_DEG'):
    print(n, {k:round(v,4) for k,v in getattr(p,n)['tubo'].items()})
