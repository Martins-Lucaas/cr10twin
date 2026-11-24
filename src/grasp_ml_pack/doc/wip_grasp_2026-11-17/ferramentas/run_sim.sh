#!/bin/bash
# uso: run_sim.sh NOME DOMAIN  -> sobe a sim headless isolada (domínio ROS + porta gazebo próprios); log em ws_NOME/launch.log
N=$1; D=$2; S=/tmp/claude-1000/-home-lucas-lpc-cr10twin/847d668c-3d80-4485-b424-058577a0b830/scratchpad; W=$S/orch/ws_$N
source /opt/ros/humble/setup.bash; source /home/lucas-lpc/cr10twin/install/setup.bash; source $W/install/setup.bash
export ROS_DOMAIN_ID=$D GAZEBO_MASTER_URI=http://127.0.0.1:$((11345+D))
cd $W; nohup ros2 launch grasp_ml_pack conveyor_cell.launch.py no_gui:=true > $W/launch.log 2>&1 &
echo $! > $W/launch.pid; echo "sim $N domínio $D iniciada (pid $(cat $W/launch.pid)); aguarde ~55 s"
