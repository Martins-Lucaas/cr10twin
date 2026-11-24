import json, sys, time, rclpy
from rclpy.node import Node
from gazebo_msgs.msg import LinkStates, ModelStates
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

obj = sys.argv[1]


class D(Node):
    def __init__(s):
        super().__init__('diag')
        s.links = {}; s.models = {}; s.log = []; s.state = ''; s.done = False
        s.create_subscription(LinkStates, '/gazebo/link_states', s.cl, 5)
        s.create_subscription(ModelStates, '/gazebo/model_states', s.cm, 5)
        s.create_subscription(String, '/cell/status', s.cs, 10)
        s.q={}
        s.create_subscription(JointState, '/joint_states', lambda m: s.q.update(zip(m.name,m.position)), 5)

    def cl(s, m):
        s.links = {n: [p.position.x, p.position.y, p.position.z] for n, p in zip(m.name, m.pose)}

    def cm(s, m):
        s.models = {n: [p.position.x, p.position.y, p.position.z] for n, p in zip(m.name, m.pose)}

    def cs(s, m):
        d = json.loads(m.data); s.state = d['state']
        if d['state'] == 'CYCLE_DONE':
            s.done = True

    def snap(s):
        o = [v for k, v in s.models.items() if k == 'pick_object']
        if not o:
            return
        row = {'t': time.time(), 'state': s.state, 'obj': o[0]}
        row['q']=[s.q.get('joint%d'%i) for i in range(1,7)]
        row['links']=[k for k in s.links if 'hand' in k.lower() or 'link6' in k.lower()] if not s.log else []
        for k, v in s.links.items():
            kl = k.lower()
            if 'hand_base' in kl or 'distal' in kl or 'proximal' in kl:
                row[k] = v
        s.log.append(row)


rclpy.init(); n = D()


def call(name):
    c = n.create_client(Trigger, name); c.wait_for_service(5)
    f = c.call_async(Trigger.Request())
    while not f.done():
        rclpy.spin_once(n, timeout_sec=0.1)
    return f.result()


call('/conveyor/reset'); time.sleep(1)
print(call(f'/conveyor/spawn_{obj}').message); time.sleep(3)
print(call('/cell/execute_grasp').message)
end = time.time() + 150
while not n.done and time.time() < end:
    rclpy.spin_once(n, timeout_sec=0.05); n.snap()
json.dump(n.log, open(sys.argv[2], 'w'))
print('snaps', len(n.log), 'keys', len(n.log[0]) if n.log else 0)
