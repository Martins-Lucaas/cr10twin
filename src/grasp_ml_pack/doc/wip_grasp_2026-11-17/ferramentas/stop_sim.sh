#!/bin/bash
# uso: stop_sim.sh DOMAIN (>=11) -> encerra só os processos com esse ROS_DOMAIN_ID
D=$1; [ "$D" -ge 11 ] 2>/dev/null || { echo "domínio inválido"; exit 1; }
for p in $(pgrep -u $USER -f "ros2|gzserver|gzclient|grasp_ml_pack|spawner|robot_state|conveyor|object_detector|pipeline|ros2_control" ); do
  [ "$p" = "$$" ] && continue
  tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$D" && kill -INT $p 2>/dev/null
done; sleep 6
for p in $(pgrep -u $USER -f "gzserver|gzclient|ros2"); do tr '\0' '\n' < /proc/$p/environ 2>/dev/null | grep -qx "ROS_DOMAIN_ID=$D" && kill -9 $p 2>/dev/null; done; echo "domínio $D parado"
