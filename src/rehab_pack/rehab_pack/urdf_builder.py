"""Monta o URDF do CR10 com o mastro de reabilitação acoplado no Link6.

Duas saídas, como no touch_pack:
  * `full`    — para o spawn no Gazebo (visual, colisão, inércia, plugins);
  * `minimal` — para o robot_state_publisher (só links e juntas).
"""
from __future__ import annotations

import os
import re

import xacro

from .constants import ARM_JOINTS, HOME_Q_RAD


def _read_tool(rehab_share: str) -> str:
    with open(os.path.join(rehab_share, 'urdf', 'joystick_tool.urdf'),
              encoding='utf-8') as f:
        return f.read()


def _tool_body(tool_urdf: str, rehab_share: str, sim_ft: bool) -> str:
    tool_urdf = tool_urdf.replace('package://rehab_pack',
                                  f'file://{rehab_share}')
    body = re.search(r'<robot[^>]*>(.*)</robot>', tool_urdf, re.DOTALL).group(1)

    # Andaime standalone: o CR10 fornece world e Link6.
    body = re.sub(r'<link\s+name="world"\s*/>\s*', '', body)
    body = re.sub(r'<joint\s+name="world_to_base"[^>]*>.*?</joint>\s*', '',
                  body, flags=re.DOTALL)
    body = re.sub(r'<link\s+name="Link6">.*?</link>\s*', '', body,
                  flags=re.DOTALL)
    # O bloco gazebo do Link6 já sai do arm_gz; duplicar faz o parser reclamar.
    body = re.sub(r'<gazebo\s+reference="Link6">.*?</gazebo>\s*', '', body,
                  flags=re.DOTALL)

    if sim_ft:
        body = body.replace('<!--SIM_FT_BEGIN-->', '').replace(
            '<!--SIM_FT_END-->', '')
    else:
        body = re.sub(r'<!--SIM_FT_BEGIN-->.*?<!--SIM_FT_END-->\s*', '', body,
                      flags=re.DOTALL)
    return body


def _set_home(cr10: str) -> str:
    """Troca os 6 initial_value do bloco ros2_control por HOME_Q_RAD."""
    m = re.search(r'<ros2_control\b.*?</ros2_control>', cr10, re.DOTALL)
    block, n = m.group(0), 0
    for joint, q in zip(ARM_JOINTS, HOME_Q_RAD):
        block, k = re.subn(
            rf'(<joint name="{joint}"\s*>.*?name="initial_value"\s*>)[^<]*',
            rf'\g<1>{q:.4f}', block, count=1, flags=re.DOTALL)
        n += k
    if n != len(ARM_JOINTS):
        raise RuntimeError(f'esperava {len(ARM_JOINTS)} initial_value, '
                           f'troquei {n}')
    return cr10.replace(m.group(0), block)


def build_robot_urdf(cra_share: str, rehab_share: str,
                     sim_ft: bool = False) -> tuple[str, str]:
    """Devolve (urdf completo, urdf mínimo do robot_state_publisher)."""
    controllers_yaml = os.path.join(rehab_share, 'config',
                                    'rehab_controllers.yaml')

    doc = xacro.parse(open(os.path.join(cra_share, 'urdf', 'cr10_robot.xacro')))
    xacro.process_doc(doc)
    cr10 = _set_home(doc.toxml())
    cr10 = re.sub(r'<parameters>[^<]*/ros2_controllers\.yaml</parameters>',
                  f'<parameters>{controllers_yaml}</parameters>', cr10)

    # O xacro do CR10 usa a tag legada <selfCollide>; a ferramenta usa a
    # canônica. Sem unificar, o sdformat reclama de valores inconsistentes.
    cr10 = re.sub(r'<selfCollide>\s*(?:true|false|0|1)\s*</selfCollide>',
                  '<self_collide>0</self_collide>', cr10)
    existing = set(re.findall(r'<gazebo\s+reference="([^"]+)"', cr10))
    arm_gz = ''.join(
        f'\n  <gazebo reference="{n}"><self_collide>0</self_collide></gazebo>'
        for n in re.findall(r'<link\s+name="([^"]+)"', cr10)
        if n not in existing)

    # Links 1–4 usam a STL cheia do fabricante como colisão e nada encosta
    # neles nesta célula: tirar a colisão alivia o solver do ODE.
    for ln in ('Link1', 'Link2', 'Link3', 'Link4'):
        cr10 = re.sub(rf'(<link name="{ln}"\s*>.*?)<collision\b.*?</collision>\s*',
                      r'\1', cr10, count=1, flags=re.DOTALL)

    body = _tool_body(_read_tool(rehab_share), rehab_share, sim_ft)
    full = cr10.replace('</robot>', body + '</robot>')
    full = full.replace('</robot>', arm_gz + '\n</robot>')

    minimal = full
    for tag in ('visual', 'collision', 'inertial'):
        minimal = re.sub(rf'<{tag}\b[^>]*>.*?</{tag}>', '', minimal,
                         flags=re.DOTALL)
    minimal = re.sub(r'<gazebo\s+reference\s*=\s*"[^"]*"\s*>.*?</gazebo>', '',
                     minimal, flags=re.DOTALL)
    minimal = re.sub(r'<!--.*?-->', '', minimal, flags=re.DOTALL)
    minimal = re.sub(r'<\?xml[^?]*\?>', '', minimal)
    return full, ' '.join(minimal.split())
