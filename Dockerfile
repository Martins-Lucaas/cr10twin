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

# extras opcionais do grasp_ml_pack (torch CPU); constraints mantêm numpy<2 e opencv compatível
RUN printf 'numpy<2\nopencv-python<4.11\nopencv-python-headless<4.11\n' > /tmp/c.txt \
 && pip install --no-cache-dir -c /tmp/c.txt scikit-learn covvi-eci==1.1.6 \
 && pip install --no-cache-dir -c /tmp/c.txt torch torchvision --index-url https://download.pytorch.org/whl/cpu \
 && pip install --no-cache-dir -c /tmp/c.txt ultralytics

# pesos YOLO embutidos (use_yolo:=true offline; yolo_model resolve relativo ao CWD=/ws)
ENV YOLO_CONFIG_DIR=/tmp/Ultralytics
RUN mkdir -p /ws && curl -fsSL -o /ws/yolov8n.pt https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8n.pt

# mediapipe (CAM teleop) traz opencv-contrib: deixa UM só cv2 (contrib reinstalado p/ restaurar os arquivos)
# 0.10.21 = última com mp.solutions, que o hand_camera_teleop usa
RUN pip install --no-cache-dir -c /tmp/c.txt "mediapipe==0.10.21" \
 && pip uninstall -y opencv-python opencv-python-headless \
 && pip install --no-cache-dir --force-reinstall --no-deps "opencv-contrib-python<4.11"

# torch puxa setuptools novo, que quebra o ament_python do Humble (packaging do sistema é antigo)
RUN pip install --no-cache-dir "setuptools==59.6.0"

# ── workspace ───────────────────────────────────────────────────────────
WORKDIR /ws
COPY . /ws/src/cr10twin

# resolve o que os package.xml declaram e compila
RUN source /opt/ros/humble/setup.bash \
 && (rosdep init 2>/dev/null || true) \
 && apt-get update \
 && rosdep update \
 && rosdep install --from-paths src --ignore-src -r -y \
 && colcon build --symlink-install \
 && rm -rf /var/lib/apt/lists/*

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
