"""
tactile_cell.launch.py — Launcher principal do touch_pack.

Argumentos (todos opcionais):
    end_effector     hand | touch_tool  (default: hand)
                       hand       → CR10 + mão COVVI; tcp_link = ponta do Index
                       touch_tool → CR10 + TCP de palpação (acoplador do robô +
                                    célula axial de 100 kg + acoplador da
                                    ferramenta + touch_tool + ponteira D com o
                                    laminado tátil 5×5); tcp_link = face do
                                    laminado, +162,2 mm do flange
    control_mode     sim_only | mirror | real_from_sim (default sim_only)
    robot_ip         IP do controlador CR10 real (default 192.168.5.2)
    no_gui           true = não abrir palpation_gui (default false);
                       com control_mode:=mirror, sobe o mirror_node
                       standalone no lugar do espelhamento da GUI
    sensor           4 | 5  (default 4) — grade do sensor de toque
                       4 → sensor 4×4 (firmware com TOTAL/Ifinal)
                       5 → sensor 5×5 (sem TOTAL; ativação média por frame)
    force_sensor     load_cell | ft6  (default: load_cell)
                       load_cell → célula axial de 100 kg no XIAO ESP32C6 +
                                   HX711, pela USB (force_receiver)
                       ft6       → célula FA7155 de 6 eixos, pela RS485
                                   (ft_receiver)
                       Os dois publicam /load_cell/force_net; SÓ UM sobe.
    force_source     real | sim  (default: real)
                       real → o driver acima, lendo a célula física. Sem
                              célula no cabo o tópico fica mudo e o ensaio
                              é recusado por "leitura velha".
                       sim  → sim_force_bridge: força do plugin FT do
                              Gazebo. Opt-in explícito, e a GUI marca a aba
                              Load Cell como SIMULADA.
    lc_port          porta USB do XIAO (ex.: /dev/ttyACM0). Vazio (default) =
                       auto-detect pelo VID da Espressif.
    ft_port          porta do conversor USB-RS485 (ex.: COM5, /dev/ttyUSB0).
                       Vazio (default) = auto-detect pelo VID.

Exemplos:
    ros2 launch touch_pack tactile_cell.launch.py
    ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool
    ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool sensor:='4'
    ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool sensor:='5'
    ros2 launch touch_pack tactile_cell.launch.py end_effector:=touch_tool no_gui:=true
"""
import os
import re
import tempfile
import xacro

from ament_index_python.packages import get_package_share_directory
from hand_pack.urdf_helpers import (
    clamp_hand_joint_limits,
    inject_visual_skin_layer,
    HAND_DRIVER_LOWER,
    INTER_FINGER_COLLISION_LINKS,
)
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, OpaqueFunction,
                             RegisterEventHandler, IncludeLaunchDescription)
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Helpers de saneamento do URDF combinado
def _fix_virtual_link_inertia(urdf_body: str) -> str:
    phantom = (
        r'<inertial>\s*<mass value="1"\s*/>\s*'
        r'<inertia ixx="1\.0" ixy="0\.0" ixz="0\.0" iyy="1\.0" iyz="0\.0" izz="1\.0"\s*/>'
        r'\s*</inertial>'
    )
    minimal = (
        '<inertial><mass value="0.001"/>'
        '<inertia ixx="1e-9" ixy="0.0" ixz="0.0" iyy="1e-9" iyz="0.0" izz="1e-9"/>'
        '</inertial>'
    )
    return re.sub(phantom, minimal, urdf_body, flags=re.DOTALL)


def _stabilize_hand_joints(urdf_body: str) -> str:
    def _patch(m: re.Match) -> str:
        jxml = m.group(0)
        if 'type="revolute"' not in jxml:
            return jxml
        is_mimic = '<mimic' in jxml
        damp, fric = (30.0, 10.0) if is_mimic else (5.0, 1.0)
        dyn = f'<dynamics damping="{damp}" friction="{fric}"/>'
        if '<dynamics' in jxml:
            jxml = re.sub(r'<dynamics[^/]*/>', dyn, jxml)
        else:
            jxml = jxml.replace('</joint>', f'      {dyn}\n    </joint>')
        if not is_mimic:
            jxml = re.sub(r'effort="[\d.]+"', 'effort="8.0"', jxml)
        return jxml
    return re.sub(r'<joint\b[^>]*>.*?</joint>', _patch,
                  urdf_body, flags=re.DOTALL)


def _inject_hand_initial_values(hand_body: str) -> str:
    """Define <param name="initial_value"> nas juntas da mão (ros2_control)."""
    init = dict(HAND_DRIVER_LOWER)
    for m in re.finditer(
            r'<joint\s+name="([^"]+)"\s+type="revolute">'
            r'(?:(?!</joint>).)*?<mimic\s+joint="([^"]+)"'
            r'[^>]*multiplier="([-\d.eE]+)"',
            hand_body, flags=re.DOTALL):
        name, driver, mult = m.group(1), m.group(2), float(m.group(3))
        if driver in HAND_DRIVER_LOWER:
            init[name] = mult * HAND_DRIVER_LOWER[driver]

    def _patch(m: re.Match) -> str:
        val = init.get(m.group(1))
        if val is None:
            return m.group(0)
        return (f'<joint name="{m.group(1)}">'
                f'<command_interface name="position"/>'
                f'<state_interface name="position">'
                f'<param name="initial_value">{val:.5f}</param>'
                f'</state_interface>')

    return re.sub(
        r'<joint name="([^"]+)"><command_interface name="position"/>'
        r'<state_interface name="position"/>',
        _patch, hand_body)


# Construção do URDF combinado (roteado por end_effector)
def _build_robot_urdf(end_effector: str, force_source: str = 'real'):
    hand_pack_share  = get_package_share_directory('hand_pack')
    cra_share        = get_package_share_directory('cra_description')
    touch_pack_share = get_package_share_directory('touch_pack')

    # YAML de controllers
    if end_effector == 'hand':
        controllers_yaml = os.path.join(
            hand_pack_share, 'config', 'cr10_covvi_controllers.yaml')
    else:  # touch_tool
        controllers_yaml = os.path.join(
            touch_pack_share, 'config', 'tactile_controllers.yaml')

    # CR10 (xacro)
    cr10_xacro_path = os.path.join(cra_share, 'urdf', 'cr10_robot.xacro')
    doc = xacro.parse(open(cr10_xacro_path))
    xacro.process_doc(doc)
    cr10_urdf = doc.toxml()
    cr10_urdf = re.sub(
        r'<parameters>[^<]*/ros2_controllers\.yaml</parameters>',
        f'<parameters>{controllers_yaml}</parameters>',
        cr10_urdf)

    # Fim do URDF: links/juntas do efector + Gazebo refs
    # Unifica selfCollide: o xacro do CR10 usa a tag LEGADA <selfCollide> com
    # valor textual ("true"/"false") nos Link1..Link6, enquanto arm_gz/tool_gz
    # abaixo emitem a tag canônica <self_collide>. Na redução de juntas fixas o
    # sdformat compara os dois e reclamava
    #   "multiple inconsistent <self_collide> ... [false] with [0]"
    cr10_urdf = re.sub(r'<selfCollide>\s*(?:true|false|0|1)\s*</selfCollide>',
                       '<self_collide>0</self_collide>', cr10_urdf)
    arm_links = re.findall(r'<link\s+name="([^"]+)"', cr10_urdf)
    # Só adiciona self_collide para links sem <gazebo reference="..."> no xacro.
    existing_arm_gz = set(re.findall(r'<gazebo\s+reference="([^"]+)"', cr10_urdf))
    arm_gz = ''.join(
        f'\n  <gazebo reference="{n}"><self_collide>0</self_collide></gazebo>'
        for n in arm_links if n not in existing_arm_gz)

    if end_effector == 'hand':
        full_urdf = _build_hand_suffix(
            cr10_urdf, hand_pack_share, arm_gz, touch_pack_share)
    else:
        full_urdf = _build_touch_tool_suffix(cr10_urdf, touch_pack_share,
                                             arm_gz, force_source)

    # URDF mínimo para o robot_state_publisher
    minimal = full_urdf
    minimal = re.sub(r'<visual\b[^>]*>.*?</visual>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<collision\b[^>]*>.*?</collision>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<inertial\b[^>]*>.*?</inertial>', '', minimal, flags=re.DOTALL)
    minimal = re.sub(
        r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '',
        minimal, flags=re.DOTALL)
    minimal = re.sub(r'<!--.*?-->', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<\?xml[^?]*\?>', '', minimal)
    minimal = ' '.join(minimal.split())

    return full_urdf, minimal


def _build_hand_suffix(cr10_urdf: str, hand_pack_share: str, arm_gz: str,
                       touch_pack_share: str) -> str:
    """Injeta a mão COVVI + tcp_link (Index distal) no CR10."""
    hand_urdf_path = os.path.join(
        hand_pack_share, 'urdf', 'linear_covvi_hand_gazebo.urdf')
    with open(hand_urdf_path) as f:
        hand_urdf = f.read()
    hand_urdf = hand_urdf.replace(
        'package://hand_pack', f'file://{hand_pack_share}')
    hand_body = re.search(
        r'<robot[^>]*>(.*)</robot>', hand_urdf, re.DOTALL).group(1)
    hand_body = re.sub(r'<link\s+name="world"\s*/>\s*', '', hand_body)
    hand_body = re.sub(r'<link\s+name="base_footprint"\s*/>\s*', '', hand_body)
    hand_body = re.sub(
        r'<joint\s+name="world_fixed"[^>]*>.*?</joint>', '',
        hand_body, flags=re.DOTALL)
    hand_body = re.sub(
        r'<joint\s+name="base_joint"[^>]*>.*?</joint>', '',
        hand_body, flags=re.DOTALL)
    hand_body = hand_body.replace('"base_link"', '"hand_base_link"')
    hand_body = re.sub(
        r'<gazebo>\s*<plugin[^>]*gazebo_ros2_control[^>]*>.*?</plugin>\s*</gazebo>',
        '', hand_body, flags=re.DOTALL)
    hand_body = hand_body.replace(
        '<ros2_control name="GazeboSystem"',
        '<ros2_control name="HandGazeboSystem"')
    hand_body = _fix_virtual_link_inertia(hand_body)
    hand_body = clamp_hand_joint_limits(hand_body)
    hand_body = _stabilize_hand_joints(hand_body)
    # As 26 juntas mimic FICAM no <ros2_control>. A tentativa de tirá-las de
    # lá e entregá-las ao libgazebo_mimic_joint_plugin.so (preensão por
    # física) desestabiliza o modelo inteiro: os elos mimic são virtuais
    # (mass 1e-3 / inertia 1e-9, ver _fix_virtual_link_inertia), e sem o
    # ros2_control segurando-os nada os prende — o ODE diverge e o robô
    # reaparece em 0 0 0, dentro do chão. Medido no Gazebo, com o modelo
    # correto em 0.30/0/0.75 na pausa e em 0 0 0 assim que a física roda.
    # Não adianta contornar: falha igual com inércia 1e-6 e 1e-5, com o
    # damping antigo (30/10) e com hasPID nas 26 (não só nas 10 de contato).
    # Para preensão por atrito, o caminho é dar inércia/massa REAIS às
    # falanges antes de tirá-las do ros2_control — ver
    # strip_mimic_joints_from_ros2_control() e inject_mimic_joint_plugins()
    # em hand_pack/urdf_helpers.py, que seguem no pacote, sem uso.
    hand_body = _inject_hand_initial_values(hand_body)
    # Remove <gazebo reference="..."> estáticos com propriedades de física (mu1,
    # kd, etc.) para evitar "multiple inconsistent" do parser_urdf ao reduzir
    # fixed joints: o loop abaixo adiciona valores canônicos para todos os links.
    hand_body = re.sub(
        r'<gazebo\s+reference="[^"]+">(?:(?!</gazebo>).)*?<mu1>(?:(?!</gazebo>).)*?</gazebo>\s*',
        '', hand_body, flags=re.DOTALL)
    hand_body = inject_visual_skin_layer(hand_body)

    hand_link_names = re.findall(r'<link\s+name="([^"]+)"', hand_body)
    fc = set(INTER_FINGER_COLLISION_LINKS)
    for lname in hand_link_names:
        is_grip = lname in fc
        sc = '1' if is_grip else '0'   # forma canônica — ver _build_robot_urdf
        mu = '2.5' if is_grip else '0.8'
        hand_body += (
            f'\n  <gazebo reference="{lname}">'
            f'<gravity>false</gravity>'
            f'<self_collide>{sc}</self_collide>'
            f'<mu1>{mu}</mu1><mu2>{mu}</mu2>'
            f'<kp>5e4</kp><kd>50.0</kd>'
            f'<maxContacts>8</maxContacts>'
            f'<minDepth>0.0005</minDepth>'
            f'<maxVel>0.01</maxVel>'
            f'</gazebo>'
        )

    # Acoplador da prótese (PecasProtese.stl) entre Link6 e a mão.
    coupler_mesh = os.path.join(
        touch_pack_share, 'meshes', 'PecasProtese.stl')
    attach_joint = f'''
    <link name="hand_coupler_link">
      <inertial>
        <origin xyz="0 0 0.02773" rpy="0 0 0"/>
        <mass value="0.150"/>
        <inertia ixx="9.12e-5" ixy="0.0" ixz="0.0"
                 iyy="9.12e-5" iyz="0.0" izz="1.055e-4"/>
      </inertial>
      <visual>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <geometry>
          <mesh filename="file://{coupler_mesh}" scale="0.001 0.001 0.001"/>
        </geometry>
        <material name="coupler_black">
          <color rgba="0.03 0.03 0.03 1.0"/>
        </material>
      </visual>
      <collision name="col_hand_coupler">
        <origin xyz="0 0 0.02773" rpy="0 0 0"/>
        <geometry>
          <cylinder radius="0.0375" length="0.05546"/>
        </geometry>
      </collision>
    </link>

    <joint name="coupler_attach" type="fixed">
      <parent link="Link6"/>
      <child link="hand_coupler_link"/>
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </joint>

    <joint name="hand_attach_joint" type="fixed">
      <parent link="hand_coupler_link"/>
      <child link="hand_base_link"/>
      <origin xyz="0 0 0.05546" rpy="1.5708 0 0"/>
    </joint>

    <gazebo reference="hand_coupler_link">
      <gravity>false</gravity>
      <self_collide>0</self_collide>
      <visual>
        <material>
          <ambient>0.02 0.02 0.02 1</ambient>
          <diffuse>0.03 0.03 0.03 1</diffuse>
          <specular>0.10 0.10 0.10 1</specular>
        </material>
      </visual>
    </gazebo>'''

    tcp_alias = '''
    <link name="tcp_link"/>
    <joint name="tcp_alias_joint" type="fixed">
      <parent link="index_distal"/>
      <child link="tcp_link"/>
      <origin xyz="0 0 0.022" rpy="0 0 0"/>
    </joint>'''

    full_urdf = cr10_urdf.replace(
        '</robot>', hand_body + attach_joint + tcp_alias + '</robot>')
    full_urdf = full_urdf.replace('</robot>', arm_gz + '\n</robot>')
    return full_urdf


def _build_touch_tool_suffix(cr10_urdf: str, touch_pack_share: str,
                              arm_gz: str, force_source: str = 'real') -> str:
    """Injeta o TCP de palpação (urdf/touch_tool_tcp.urdf) no CR10.

    Com `force_source != 'sim'` remove a F/T simulada da junta
    `load_cell_attach` — ver o bloco correspondente em touch_tool_tcp.urdf.
    Ela custa o lumping da junta, e sem lumping os 561,5 g da pilha da
    ferramenta viram corpo separado ligado por restrição elástica, pendurado
    onde J4 trabalha. Com a célula física no cabo essa F/T não é usada por
    ninguém, e o punho rígido é o que produziu as descidas verticais."""
    # Colisão dos elos 1–4 do braço: são a STL cheia do fabricante (sem
    # decomposição convexa) e nesta célula o braço não encosta em nada por
    # ali — só a ferramenta toca a amostra. Remover baixa o custo do solver
    # ODE e o risco de instabilidade no contato de palpação. base_link,
    # Link5 e Link6 (perto da ferramenta) mantêm a colisão.
    for _ln in ('Link1', 'Link2', 'Link3', 'Link4'):
        cr10_urdf = re.sub(
            rf'(<link name="{_ln}"\s*>.*?)<collision\b.*?</collision>\s*',
            r'\1', cr10_urdf, count=1, flags=re.DOTALL)

    tool_urdf_path = os.path.join(
        touch_pack_share, 'urdf', 'touch_tool_tcp.urdf')
    with open(tool_urdf_path, encoding='utf-8') as f:
        tool_urdf = f.read()

    tool_urdf = tool_urdf.replace(
        'package://touch_pack', f'file://{touch_pack_share}')
    tool_body = re.search(
        r'<robot[^>]*>(.*)</robot>', tool_urdf, re.DOTALL).group(1)

    # Andaime standalone: o CR10 fornece world e Link6.
    tool_body = re.sub(r'<link\s+name="world"\s*/>\s*', '', tool_body)
    tool_body = re.sub(r'<joint\s+name="world_to_base"[^>]*>.*?</joint>\s*',
                       '', tool_body, flags=re.DOTALL)
    tool_body = re.sub(r'<link\s+name="Link6">.*?</link>\s*', '',
                       tool_body, flags=re.DOTALL)
    # arm_gz já emite o <gazebo reference="Link6">; dois blocos para o mesmo
    # link fazem o parser_urdf reclamar de "multiple inconsistent" ao reduzir
    # as juntas fixas.
    tool_body = re.sub(
        r'<gazebo\s+reference="Link6">.*?</gazebo>\s*', '',
        tool_body, flags=re.DOTALL)

    if force_source != 'sim':
        # Sai o <gazebo reference="load_cell_attach"> (disableFixedJointLumping
        # + provideFeedback) e o plugin que o consome. Os dois juntos: sem o
        # lumping desligado a junta some do SDF e o plugin ficaria apontando
        # para uma junta inexistente.
        #
        # O comentário que documenta o par sai junto (grupo opcional na
        # frente): deixá-lo no URDF gerado descreveria um bloco ausente.
        # Casa só um comentário ADJACENTE ao <gazebo>, e o [\s\S] guardado
        # por (?!-->) impede que a busca atravesse o fim de um comentário
        # anterior e engula o que vem antes dele.
        tool_body = re.sub(
            r'(?:<!--(?:(?!-->)[\s\S])*?-->\s*)?'
            r'<gazebo\s+reference="load_cell_attach">[\s\S]*?</gazebo>\s*'
            r'|<gazebo>\s*<plugin\s+name="sim_load_cell_ft"[\s\S]*?</gazebo>\s*',
            '', tool_body)

    full_urdf = cr10_urdf.replace('</robot>', tool_body + '</robot>')
    full_urdf = full_urdf.replace('</robot>', arm_gz + '\n</robot>')
    return full_urdf


# OpaqueFunction: monta nodes/handlers após resolver os argumentos
_CONTROL_MODE_MAP = {
    'sim_only':      'SIM_ONLY',
    'mirror':        'MIRROR',
    'real_from_sim': 'REAL_FROM_SIM',
}


def launch_setup(context, *args, **kwargs):
    end_effector = LaunchConfiguration('end_effector').perform(context)
    control_mode = LaunchConfiguration('control_mode').perform(context)
    robot_ip     = LaunchConfiguration('robot_ip').perform(context)
    no_gui_val   = LaunchConfiguration('no_gui').perform(context)
    no_gui       = no_gui_val.strip().lower() in ('true', '1', 'yes')
    # Tipo do sensor de toque: '4' (4×4, com Ifinal/TOTAL) | '5' (5×5, sem
    # TOTAL).
    sensor = LaunchConfiguration('sensor').perform(context).strip()
    if sensor not in ('4', '5'):
        sensor = '5'
    # Qual célula está na bancada. A axial de 100 kg é a montada — valor
    # desconhecido cai nela, não na de 6 eixos, para que um erro de digitação
    # não suba silenciosamente o driver da célula que não está no cabo.
    force_sensor = LaunchConfiguration(
        'force_sensor').perform(context).strip().lower()
    if force_sensor not in ('load_cell', 'ft6'):
        force_sensor = 'load_cell'
    lc_port = LaunchConfiguration('lc_port').perform(context).strip()
    ft_port = LaunchConfiguration('ft_port').perform(context).strip()
    # Real x simulado é decisão SEPARADA de qual célula está no cabo e de
    # qual control_mode roda. Valor desconhecido cai em 'real': um erro de
    # digitação não pode fazer a GUI mostrar força de Gazebo como se fosse
    # da bancada.
    force_source = LaunchConfiguration(
        'force_source').perform(context).strip().lower()
    if force_source not in ('real', 'sim'):
        force_source = 'real'

    robot_mode = _CONTROL_MODE_MAP.get(control_mode, 'SIM_ONLY')

    pkg_touch  = get_package_share_directory('touch_pack')
    pkg_gazebo = get_package_share_directory('gazebo_ros')

    # URDFs tempfile (não path fixo): duas sessões simultâneas não colidem.
    full_urdf, minimal_urdf = _build_robot_urdf(end_effector, force_source)
    fd, urdf_spawn_path = tempfile.mkstemp(
        prefix='tactile_cell_robot_', suffix='.urdf')
    with os.fdopen(fd, 'w') as f:
        f.write(full_urdf)

    world_file = os.path.join(pkg_touch, 'worlds', 'research_lab.world')

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file, 'verbose': 'false'}.items())

    # Robot State Publisher
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': minimal_urdf,
                     'use_sim_time': True}])

    # Spawn do robô
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_spawn_path, '-entity', 'cr10_tcp',
                   '-x', '0.30', '-y', '0', '-z', '0.75'],
        parameters=[{'use_sim_time': True}])

    # Sincronização com o robô real Lê a pose real via rede e move o braço
    # simulado até ela via JTC.
    pose_sync = Node(
        package='touch_pack', executable='real_pose_sync',
        parameters=[{'use_sim_time': True, 'robot_ip': robot_ip}])

    # Controllers
    load_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'])
    load_arm = Node(
        package='controller_manager', executable='spawner',
        arguments=['cr10_group_controller',
                   '--controller-manager', '/controller_manager'])

    # Aplicação (explorer + GUI + logger + force_rx)
    explorer_node = Node(
        package='touch_pack', executable='tactile_explorer',
        parameters=[{
            'arm_base_z':   0.78,
            'use_sim_time': True,
        }])

    gui_node = Node(
        package='touch_pack', executable='palpation_gui',
        parameters=[{'use_sim_time': True,
                     'robot_ip':     robot_ip,
                     'robot_mode':   robot_mode,
                     # Gate do modo Palpação: só liberado com end_effector=touch_tool.
                     'end_effector': end_effector,
                     # Grade do sensor de toque (4×4 | 5×5).
                     'sensor':       sensor,
                     # Qual célula está no cabo. A GUI monta a aba "Load
                     # Cell" desta célula: com a axial, Reading + Calibration;
                     # com a FA7155, 6 Axes. Vai pelo MESMO argumento que
                     # escolheu o receiver acima, senão a tela mostraria os
                     # painéis de uma célula e o dado viria da outra.
                     'force_sensor': force_sensor,
                     # Real x simulado, para a aba "Load Cell" dizer na tela
                     # de onde vem o número — ver o comentário do force_rx.
                     'force_source': force_source,
                     # URDF completo (com <visual>) que foi para o Gazebo —
                     # a aba "3D Manipulation" renderiza ESTE modelo.
                     'robot_description_path': urdf_spawn_path}],
        condition=UnlessCondition(LaunchConfiguration('no_gui')))

    # A grade vai para o logger pelo MESMO parâmetro da GUI: se as duas
    # discordarem, ele descarta 100% dos frames táteis e o run sai com os
    # taxel_* vazios (era o que acontecia com o default 4×4 contra o 5×5 real).
    logger_node = Node(
        package='touch_pack', executable='palpation_logger',
        parameters=[{'sensor': sensor,
                     # Proveniência do run: sem isto o params.json não diz
                     # qual célula gerou o CSV, e as colunas lc_voltage_*
                     # trocam de unidade entre as duas.
                     'force_sensor': force_sensor}])

    # Fonte de /load_cell/force_net — escolhida por `force_source`, NUNCA
    # pelo control_mode. Amarrar as duas coisas fazia o default (sim_only)
    # subir o sim_force_bridge no lugar do driver: a aba "Load Cell" da GUI
    # mostrava ~5,5 N (o peso estático da pilha abaixo da junta load_cell_attach,
    # 0,5615 kg) com a célula física DESLIGADA, sem nada na tela dizendo que
    # o número vinha do Gazebo.
    #   real (default) → driver da célula que está no cabo. Sem célula, o
    #                    tópico fica mudo e o explorer recusa o ensaio por
    #                    "leitura velha" — falha honesta, que é o certo.
    #   sim            → sim_force_bridge (wrench do plugin FT em Gazebo),
    #                    para fechar a malha sem bancada. Opt-in explícito.
    # UM só publica por vez — os dois no ar fariam o explorer regular contra
    # a média de duas fontes.
    if force_source == 'sim':
        force_rx_node = Node(
            package='touch_pack', executable='sim_force_bridge',
            parameters=[{'use_sim_time': True}])
    elif force_sensor == 'ft6':
        force_rx_node = Node(
            package='touch_pack', executable='ft_receiver',
            parameters=[{'ft_serial_port': ft_port}])
    else:
        force_rx_node = Node(
            package='touch_pack', executable='force_receiver',
            parameters=[{'lc_serial_port': lc_port}])

    # Receptor do touch sensor (STM32 → UDP 8081).
    touch_rx_node = Node(
        package='touch_pack', executable='touch_receiver')

    # Pareador célula+toque → /touch_sync/data (50 Hz, com idades p/ auditoria).
    force_sync_node = Node(
        package='touch_pack', executable='force_sync')

    # Nós que não dependem de controllers — sobem logo após o spawn.
    early_nodes = [gui_node, logger_node, force_rx_node,
                   touch_rx_node, force_sync_node]

    # Mirror standalone — só sem GUI
    # Com a GUI aberta o espelhamento mora nela (conexão única ao CR10);
    # sem GUI (no_gui:=true) este nó assume para o MIRROR não quebrar.
    if robot_mode == 'MIRROR' and no_gui:
        early_nodes.append(Node(
            package='touch_pack', executable='mirror_node',
            parameters=[{'robot_ip': robot_ip}]))
    # Explorer precisa da action do cr10_group_controller — sobe por último.
    late_nodes  = [explorer_node]

    # Cadeia de dependências: varia com end_effector
    # GUI, logger e force_rx sobem em paralelo com load_jsb — sem esperar controllers.
    after_spawn = RegisterEventHandler(
        OnProcessExit(target_action=spawn_robot,
                      on_exit=[load_jsb] + early_nodes))
    after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=load_jsb, on_exit=[load_arm]))

    if end_effector == 'hand':
        load_hand = Node(
            package='controller_manager', executable='spawner',
            arguments=['hand_position_controller',
                       '--controller-manager', '/controller_manager'])

        # O tampo fica LIMPO: nem o pick_object (cilindro de 60 g que
        # nascia em 0,45/0,15/0,80) nem as amostras complacentes do
        # research_lab.world. O pick_object existia para a preensão POR
        # FÍSICA, que saiu junto com o plugin mimic (ver
        # _build_hand_suffix) — sem ela ele só ficava parado na mesa.
        # kinematic_attacher continua no pacote (console_script) como
        # fallback para demo de pick-and-place em que a força de preensão
        # não é o objeto de estudo — NÃO sobe por default. Para usá-lo,
        # spawnar um objeto à mão e:
        #   ros2 run touch_pack kinematic_attacher
        #   ros2 service call /kinematic_attach/attach std_srvs/srv/Trigger

        # pose_sync só precisa do cr10_group_controller — paralelo ao load_hand.
        after_arm = RegisterEventHandler(
            OnProcessExit(target_action=load_arm,
                          on_exit=[load_hand, pose_sync]))
        after_last = RegisterEventHandler(
            OnProcessExit(target_action=load_hand, on_exit=late_nodes))
        chain = [after_spawn, after_jsb, after_arm, after_last]
    else:  # touch_tool — sem hand controller
        after_arm = RegisterEventHandler(
            OnProcessExit(target_action=load_arm,
                          on_exit=late_nodes + [pose_sync]))
        chain = [after_spawn, after_jsb, after_arm]

    return [gazebo, rsp, spawn_robot] + chain


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector', default_value='hand',
            description='Efector final: hand (COVVI) | touch_tool '
                        '(acoplador do robô + célula axial de 100 kg + '
                        'acoplador da ferramenta + touch_tool + ponteira D '
                        'com o laminado tátil 5×5 colado na face; tcp_link a '
                        '+162,2 mm do flange). A GEOMETRIA segue a célula '
                        'PARAFUSADA — force_sensor só escolhe o driver.'),
        DeclareLaunchArgument(
            'control_mode', default_value='sim_only',
            description='sim_only | mirror | real_from_sim'),
        DeclareLaunchArgument(
            'robot_ip', default_value='192.168.5.2'),
        DeclareLaunchArgument(
            'no_gui', default_value='false'),
        DeclareLaunchArgument(
            'sensor', default_value='5',
            description="Sensor de toque: '5' (5×5, sem TOTAL — o montado na "
                        "bancada, DEFAULT) | '4' (4×4, com Ifinal)"),
        DeclareLaunchArgument(
            'force_sensor', default_value='load_cell',
            description='Célula de força: load_cell (axial de 100 kg no '
                        'XIAO+HX711, DEFAULT — a montada na bancada) | ft6 '
                        '(FA7155 de 6 eixos por RS485). Só um driver sobe: '
                        'os dois publicam /load_cell/force_net.'),
        DeclareLaunchArgument(
            'force_source', default_value='real',
            description='De onde vem /load_cell/force_net: real (DEFAULT — '
                        'driver da célula no cabo; sem célula o tópico fica '
                        'mudo e o ensaio é recusado) | sim (sim_force_bridge, '
                        'wrench do plugin FT do Gazebo). Independente de '
                        'control_mode e de force_sensor. Com sim a GUI marca '
                        'a aba Load Cell como SIMULADA.'),
        DeclareLaunchArgument(
            'lc_port', default_value='',
            description='Porta USB do XIAO ESP32C6 (ex.: /dev/ttyACM0). '
                        'Vazio = auto-detect pelo VID da Espressif.'),
        DeclareLaunchArgument(
            'ft_port', default_value='',
            description='Porta do conversor USB-RS485 do FA7155 (ex.: COM5, '
                        '/dev/ttyUSB0). Vazio = auto-detect pelo VID.'),
        OpaqueFunction(function=launch_setup),
    ])
