"""Abre o Gazebo com o CR10 + FA7155 + mastro (cad/Joystick.step).

    ros2 launch rehab_pack joystick_cell.launch.py
    ros2 launch rehab_pack joystick_cell.launch.py sim_ft:=true   # F/T simulada
    ros2 launch rehab_pack joystick_cell.launch.py gui:=false     # sem cliente
    ros2 launch rehab_pack joystick_cell.launch.py force_source:=sim sim_ft:=true
    ros2 launch rehab_pack joystick_cell.launch.py painel:=false  # sem o painel Tk
"""
import math
import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from rehab_pack.constants import home_q
from rehab_pack.urdf_builder import build_robot_urdf

_TRUE = ('1', 'true', 'yes')


def _ft_defaults():
    """Modo e baud da FA7155 do touch_pack (a unidade da bancada só responde
    a poll 0x03 a 1 Mbps). Se o touch_pack não estiver instalado, os mesmos
    valores escritos aqui."""
    try:
        from touch_pack.constants import FT_MODE_POLLED, FT_SERIAL_BAUD
        return FT_MODE_POLLED, str(FT_SERIAL_BAUD)
    except Exception:
        return 'polled', '1000000'


def _flag(context, name: str) -> bool:
    return LaunchConfiguration(name).perform(context).strip().lower() in _TRUE


def launch_setup(context, *args, **kwargs):
    # levanta antes de criar qualquer ação: com o touch_pack aberto nada sobe
    from touch_pack.pack_lock import acquire
    acquire('rehab_pack')
    sim_ft = _flag(context, 'sim_ft')
    gui = _flag(context, 'gui')
    source = LaunchConfiguration('force_source').perform(context).strip()
    if source not in ('sim', 'real'):
        raise RuntimeError(f'force_source={source!r}: use sim ou real')
    if source == 'sim' and not sim_ft:
        raise RuntimeError('force_source:=sim exige sim_ft:=true')

    control_mode = LaunchConfiguration('control_mode').perform(context).strip().lower()
    if control_mode not in ('sim_only', 'mirror'):
        raise RuntimeError(f'control_mode={control_mode!r}: use sim_only ou mirror')
    mirror = control_mode == 'mirror'
    robot_ip = LaunchConfiguration('robot_ip').perform(context).strip()
    if mirror and source != 'real' and robot_ip != 'sim':
        raise RuntimeError(f'control_mode:=mirror com force_source:={source}: o robô real '
                           'ignoraria o paciente. Use force_source:=real.')

    rehab = get_package_share_directory('rehab_pack')
    cra = get_package_share_directory('cra_description')
    gazebo_ros = get_package_share_directory('gazebo_ros')

    full_urdf, minimal_urdf = build_robot_urdf(cra, rehab, sim_ft=sim_ft, home=home_q())
    # tempfile e não caminho fixo: duas sessões simultâneas não se pisam.
    fd, urdf_path = tempfile.mkstemp(prefix='joystick_cell_', suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(full_urdf)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gazebo.launch.py')),
        launch_arguments={
            'world': os.path.join(rehab, 'worlds', 'rehab_lab.world'),
            'verbose': 'false',
            'gui': 'true' if gui else 'false',
        }.items())

    rsp = Node(package='robot_state_publisher',
               executable='robot_state_publisher',
               parameters=[{'robot_description': minimal_urdf,
                            'use_sim_time': True}])

    # Mesma pose do touch_pack: robô no centro da mesa, topo a z = 0,75 m.
    spawn = Node(package='gazebo_ros', executable='spawn_entity.py',
                 arguments=['-file', urdf_path, '-entity', 'cr10_joystick',
                            '-x', '0.30', '-y', '0', '-z', '0.75'],
                 parameters=[{'use_sim_time': True}])

    load_jsb = Node(package='controller_manager', executable='spawner',
                    arguments=['joint_state_broadcaster',
                               '--controller-manager', '/controller_manager'])
    load_arm = Node(package='controller_manager', executable='spawner',
                    arguments=['cr10_group_controller',
                               '--controller-manager', '/controller_manager'])

    ft_sign = float(LaunchConfiguration('ft_sign').perform(context))
    ft_yaw = float(LaunchConfiguration('ft_yaw_deg').perform(context))
    if ft_sign not in (1.0, -1.0) or not math.isfinite(ft_yaw):
        raise RuntimeError(f'ft_sign={ft_sign} deve ser 1 ou -1 e ft_yaw_deg={ft_yaw} finito')

    sim_time = {'use_sim_time': True}
    nodes = [Node(package='rehab_pack', executable='rehab_force_source',
                  parameters=[sim_time, {
                      'source': source,
                      'real_sign': ft_sign,
                      'real_yaw_deg': ft_yaw}])]
    if source == 'real':
        # Mesmo nó e mesmos argumentos do touch_pack: o ft_receiver é o único
        # dono da porta serial da FA7155 e publica /ft_sensor/wrench.
        nodes.append(Node(
            package='touch_pack', executable='ft_receiver',
            parameters=[{
                'ft_serial_port': LaunchConfiguration('ft_port').perform(context).strip(),
                'ft_mode': LaunchConfiguration('ft_mode').perform(context).strip().lower(),
                'ft_baud': int(LaunchConfiguration('ft_baud').perform(context)),
                'ft_autozero': _flag(context, 'ft_autozero'),
                'ft_filter': _flag(context, 'ft_filter')}]))
    if _flag(context, 'admittance'):
        nodes.append(Node(package='rehab_pack', executable='rehab_admittance',
                          parameters=[sim_time, {'force_source': source,
                                                 'require_real': mirror}]))
    # Sempre no ar: em sim_only nasce desconectado e a GUI conecta, como no touch_pack.
    nodes.append(Node(package='rehab_pack', executable='rehab_mirror',
                      parameters=[{'robot_ip': robot_ip,
                                   'control_mode': control_mode}]))
    if mirror:
        nodes.append(LogInfo(msg='MIRROR: o CR10 REAL vai seguir o braço '
                                 'simulado quando a admitância estiver ATIVA, depois de Conectar. '
                                 'Mantenha a mão no E-Stop físico.'))
    if _flag(context, 'logger'):
        nodes.append(Node(package='rehab_pack', executable='rehab_logger',
                          parameters=[sim_time]))
    if _flag(context, 'painel'):
        nodes.append(Node(package='rehab_pack', executable='rehab_gui',
                          parameters=[sim_time]))

    return [gazebo, rsp, spawn, *nodes,
            RegisterEventHandler(OnProcessExit(target_action=spawn,
                                               on_exit=[load_jsb])),
            RegisterEventHandler(OnProcessExit(target_action=load_jsb,
                                               on_exit=[load_arm]))]


def generate_launch_description():
    ft_mode, ft_baud = _ft_defaults()
    return LaunchDescription([
        DeclareLaunchArgument(
            'control_mode', default_value='sim_only',
            description='sim_only (nada move o robô real) | mirror (o CR10 real '
                        'segue o simulado, como no touch_pack).'),
        DeclareLaunchArgument(
            'robot_ip', default_value='',
            description="IP do CR10. Vazio = ~/.config/touch_pack/robot.json "
                        "(o mesmo da GUI do touch_pack), senão 192.168.5.2. "
                        "'sim' = robô real simulado, para ensaiar sem hardware."),
        DeclareLaunchArgument(
            'ft_port', default_value='',
            description='Porta do conversor USB-RS485 da FA7155 (ex.: /dev/ttyUSB0). '
                        'Vazio = auto-detect pelo VID.'),
        DeclareLaunchArgument('ft_mode', default_value=ft_mode,
                              description='polled (a unidade da bancada) | stream.'),
        DeclareLaunchArgument('ft_sign', default_value='1.0',
                              description='Sinal da FA7155 (1 ou -1).'),
        DeclareLaunchArgument('ft_yaw_deg', default_value='0.0',
                              description='Giro da montagem da FA7155 em torno do seu z.'),
        DeclareLaunchArgument('ft_baud', default_value=ft_baud,
                              description='Baud da linha RS485 da FA7155.'),
        DeclareLaunchArgument('ft_autozero', default_value='false',
                              description='Auto-zero lento. Ligado, come a força fraca '
                                          'e sustentada do voluntário.'),
        DeclareLaunchArgument('ft_filter', default_value='false',
                              description='One-Euro do ft_receiver (corte até 2 Hz, ~80 ms '
                                          'de atraso na malha). O nó de admitância já '
                                          'faz a média das amostras de cada tick.'),
        DeclareLaunchArgument(
            'sim_ft', default_value='false',
            description='true = F/T simulada na junta célula→placa, publicada '
                        'em /sim/load_cell/wrench. Preserva a junta fora do '
                        'lumping do ODE; valide a estabilidade de J4 antes '
                        'de usar em ensaio.'),
        DeclareLaunchArgument(
            'gui', default_value='true',
            description='false = só o gzserver, sem janela.'),
        DeclareLaunchArgument(
            'force_source', default_value='real',
            description='real (FA7155 em /ft_sensor/wrench) | sim (exige '
                        'sim_ft:=true).'),
        DeclareLaunchArgument('admittance', default_value='true',
                              description='false = não sobe o controle.'),
        DeclareLaunchArgument('logger', default_value='true',
                              description='true = sobe o rehab_logger.'),
        DeclareLaunchArgument('painel', default_value='true',
                              description='true = sobe a GUI rehab_gui.'),
        OpaqueFunction(function=launch_setup),
    ])
