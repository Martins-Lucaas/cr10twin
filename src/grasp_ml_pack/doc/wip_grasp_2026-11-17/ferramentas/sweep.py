import numpy as np, sys
sys.path.insert(0,'/tmp/claude-1000/-home-lucas-lpc-cr10twin/847d668c-3d80-4485-b424-058577a0b830/scratchpad/orch/ws_tubo/src/grasp_ml_pack')
from grasp_ml_pack.kinematics import hand_fk, HAND_LIMITS, HAND_LOWER
print(HAND_LOWER, HAND_LIMITS)
for c in (0,0.2,0.4,0.6,0.8,1.0):
    for rot in (0,0.5,1.0):
        st={f:c*HAND_LIMITS[f] for f in ('Thumb','Index','Middle','Ring','Little')}; st['Rotate']=rot*HAND_LIMITS['Rotate']
        fk=hand_fk(st)
        print(c,rot,' '.join(f"{f[:2]}:({fk['tip_'+f][0]*1000:.0f},{fk['tip_'+f][1]*1000:.0f},{fk['tip_'+f][2]*1000:.0f})" for f in ('Thumb','Index','Middle','Ring','Little')))
