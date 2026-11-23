"""Launch standalone da mão COVVI no Gazebo."""

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from hand_pack.urdf_helpers import apply_all


def _load_patched_urdf(urdf_path: str) -> str:
    with open(urdf_path, 'r') as f:
        raw = f.read()
    # spawn_entity.py faz fromstring(str): a declaração <?xml encoding?> quebra
    raw = re.sub(r'^\s*<\?xml[^>]*\?>\s*', '', raw)
    # sem xacro aqui: resolve $(find hand_pack) (yaml do gazebo_ros2_control)
    raw = raw.replace('$(find hand_pack)', get_package_share_directory('hand_pack'))
    return apply_all(raw, skin_inflate_m=0.002)


def _minimal_urdf(urdf: str) -> str:
    # gazebo_ros2_control lê robot_description do RSP via '--param nome:=<xml>';
    # XML com comentários/': ' quebra esse parser (mesmo truque do cr10_covvi).
    for tag in ('visual', 'collision', 'inertial'):
        urdf = re.sub(rf'<{tag}\b[^>]*>.*?</{tag}>', '', urdf, flags=re.DOTALL)
    urdf = re.sub(r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '', urdf,
                  flags=re.DOTALL)
    urdf = re.sub(r'<!--.*?-->', '', urdf, flags=re.DOTALL)
    return ' '.join(urdf.split())


def generate_launch_description():
    package_name = 'hand_pack'
    pkg_share = get_package_share_directory(package_name)
    urdf_file = os.path.join(pkg_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')

    robot_desc = _load_patched_urdf(urdf_file)
    # Gazebo Classic não resolve package:// ao spawnar; mesh vira file://
    spawn_desc = robot_desc.replace('package://hand_pack', f'file://{pkg_share}')
    spawn_path = '/tmp/covvi_hand_spawn.urdf'
    with open(spawn_path, 'w') as f:
        f.write(spawn_desc)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': _minimal_urdf(robot_desc), 'use_sim_time': True}]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
    )

    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', spawn_path, '-entity', 'covvi_hand', '-z', '0.1'],
        output='screen'
    )

    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    load_hand_position_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["hand_position_controller", "--controller-manager", "/controller_manager"],
        output='screen'
    )

    delay_broadcaster_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    delay_controller_after_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_hand_position_controller],
        )
    )

    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        delay_broadcaster_after_spawn,
        delay_controller_after_broadcaster
    ])
