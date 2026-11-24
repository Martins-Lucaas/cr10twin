#!/bin/bash
# uso: setup_ws.sh NOME  -> cria workspace isolado com cópia de grasp_ml_pack e faz build overlay
set -e; N=$1; S=/tmp/claude-1000/-home-lucas-lpc-cr10twin/847d668c-3d80-4485-b424-058577a0b830/scratchpad; W=$S/orch/ws_$N
mkdir -p $W/src; rsync -a --delete --exclude __pycache__ /home/lucas-lpc/cr10twin/src/grasp_ml_pack $W/src/
cd $W; source /opt/ros/humble/setup.bash; source /home/lucas-lpc/cr10twin/install/setup.bash
colcon build --packages-select grasp_ml_pack --symlink-install 2>&1 | tail -2; echo "workspace: $W (edite $W/src/grasp_ml_pack/...)"
