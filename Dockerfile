# cr10twin — ambiente de build reprodutível
# ROS 2 Humble / Ubuntu 22.04 (Gazebo Classic 11 vem com gazebo-ros-pkgs).
#
#   xhost +local:docker                     # 1x por sessão, p/ a GUI
#   docker compose run --build cr10twin     # shell interativo no workspace
#
# Standalone (só provar que compila + testa limpo):
#   docker build -t cr10twin:humble .
#   docker run --rm cr10twin:humble bash -lc 'xvfb-run -a colcon test && colcon test-result --verbose'

FROM ros:humble-ros-base
SHELL ["/bin/bash", "-c"]

# ── deps de sistema (espelha a lista "apt packages" do README) ───────────
RUN apt-get update && apt-get install -y --no-install-recommends \
      python3-pip python3-colcon-common-extensions git xvfb \
      ros-humble-gazebo-ros-pkgs \
      ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gazebo-ros2-control \
      ros-humble-xacro ros-humble-joint-state-publisher-gui \
      ros-humble-vision-msgs ros-humble-cv-bridge ros-humble-control-msgs \
      ros-humble-admittance-controller ros-humble-kinematics-interface-kdl \
      ros-humble-force-torque-sensor-broadcaster \
      python3-tk python3-pil python3-vtk9 \
 && rm -rf /var/lib/apt/lists/*

# ── deps de Python (numpy<2 = ABI do cv_bridge do Humble) ────────────────
RUN pip install --no-cache-dir "numpy<2" pyserial matplotlib

# ── workspace ───────────────────────────────────────────────────────────
WORKDIR /ws
COPY . /ws/src/cr10twin

# resolve o que os package.xml declaram e compila
RUN source /opt/ros/humble/setup.bash \
 && (rosdep init 2>/dev/null || true) \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && colcon build --symlink-install

# ── entrypoint: sourceia o ROS e o workspace ────────────────────────────
RUN printf '%s\n' \
      '#!/bin/bash' \
      'set -e' \
      'source /opt/ros/humble/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      'exec "$@"' > /entrypoint.sh \
 && chmod +x /entrypoint.sh \
 && printf '%s\n' \
      'source /opt/ros/humble/setup.bash' \
      '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' \
      >> /root/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
