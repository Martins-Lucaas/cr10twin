"""Pré-voo do hardware: `ros2 run rehab_pack rehab_preflight [--robot-ip IP] [--ft-port DEV]`.

SOMENTE LEITURA. Não envia comando ao robô, não o habilita e não abre a porta
serial da célula: só tenta conectar-se às portas TCP (e fecha na hora), lista
dispositivos e confere o ambiente. Rode antes do primeiro `control_mode:=mirror`.

Nível de cada item: OK, AVISO (confira) ou FALHA (não suba o robô antes de
resolver). Código de saída 1 se houver FALHA.
"""
from __future__ import annotations

import argparse
import fcntl
import grp
import ipaddress
import os
import socket
import subprocess
import sys

from .real_link import SimulatedDriver  # noqa: F401  (garante o módulo importável)
from .real_link import robot_ip as resolve_ip

DASHBOARD_PORT = 29999
FEEDBACK_PORT = 30004
# VIDs aceitos pelo ft_receiver do touch_pack: CH340, CP210x, FTDI, PL2303
FT_VIDS = (0x1A86, 0x10C4, 0x0403, 0x067B)


def check_tcp(ip: str, port: int, timeout: float = 2.0) -> tuple[bool, str]:
    """Só abre e fecha a conexão: nenhum byte é enviado."""
    try:
        with socket.create_connection((ip, port), timeout=timeout):
            return True, 'aceitou a conexão'
    except OSError as exc:
        return False, str(exc)


def local_ipv4() -> list[str]:
    try:
        out = subprocess.run(['ip', '-4', '-o', 'addr'], capture_output=True,
                             text=True, timeout=3).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    return [tok.split('/')[0] for ln in out.splitlines()
            for i, tok in enumerate(ln.split()) if i > 0 and ln.split()[i - 1] == 'inet']


def same_subnet(robot: str, locais: list[str], prefix: int = 24) -> str | None:
    """Endereço local na mesma sub-rede /24 do robô, ou None."""
    try:
        rede = ipaddress.ip_network(f'{robot}/{prefix}', strict=False)
    except ValueError:
        return None
    for a in locais:
        try:
            if ipaddress.ip_address(a) in rede:
                return a
        except ValueError:
            continue
    return None


def serial_candidates() -> list[tuple[str, str]]:
    """(dispositivo, descrição) de conversores USB-RS485 conhecidos."""
    try:
        from serial.tools import list_ports
    except ImportError:
        return []
    return [(p.device, f'VID {p.vid:04X} {p.description}')
            for p in list_ports.comports() if p.vid in FT_VIDS]


def in_dialout() -> bool:
    try:
        nomes = {grp.getgrgid(g).gr_name for g in os.getgroups()}
    except KeyError:
        return False
    return 'dialout' in nomes


def install_is_stale() -> str | None:
    """Texto do problema se o rehab_pack instalado for uma CÓPIA do src (e não um
    link): o launcher rodaria código desatualizado sem avisar."""
    import rehab_pack
    f = os.path.realpath(rehab_pack.__file__)
    if '/install/' in f and '/src/' not in f:
        return f'rehab_pack vem de uma cópia instalada: {f}'
    return None


def touch_pack_open() -> bool:
    """Alguém segura a trava do touch_pack? Só testa e solta (não toma a do rehab)."""
    from touch_pack.pack_lock import _dir
    try:
        with open(os.path.join(_dir(), 'touch_pack.lock')) as f:
            fcntl.flock(f, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except FileNotFoundError:
        return False
    except BlockingIOError:
        return True
    return False


def run_checks(ip_param: str = '', ft_port: str = '') -> list[tuple[str, str, str]]:
    r: list[tuple[str, str, str]] = []
    add = lambda nivel, nome, det='': r.append((nivel, nome, det))   # noqa: E731

    stale = install_is_stale()
    add('FALHA' if stale else 'OK', 'código instalado é o do src',
        stale or 'rehab_pack carregado do repositório (build --symlink-install)')

    try:
        aberto = touch_pack_open()
        add('FALHA' if aberto else 'OK', 'trava do touch_pack',
            'touch_pack aberto: feche-o antes' if aberto else 'livre')
    except Exception as exc:
        add('FALHA', 'trava do touch_pack', f'não consegui conferir: {exc}')
    try:
        import touch_pack.real_driver  # noqa: F401
        add('OK', 'driver do CR10 (touch_pack.real_driver)', 'importável')
    except Exception as exc:
        add('FALHA', 'driver do CR10 (touch_pack.real_driver)', str(exc))
    try:
        out = subprocess.run(['ros2', 'pkg', 'executables', 'touch_pack'],
                             capture_output=True, text=True, timeout=15).stdout
        for exe in ('ft_receiver',):
            add('OK' if exe in out else 'FALHA', f'executável touch_pack/{exe}',
                'encontrado' if exe in out else 'não encontrado: colcon build do touch_pack?')
    except (OSError, subprocess.SubprocessError) as exc:
        add('AVISO', 'executáveis do touch_pack', f'não consegui listar ({exc})')

    ip = resolve_ip(ip_param)
    origem = 'argumento' if ip_param else 'robot.json do touch_pack ou padrão'
    add('OK', 'IP do CR10', f'{ip} ({origem})')
    locais = local_ipv4()
    mesmo = same_subnet(ip, locais)
    add('OK' if mesmo else 'FALHA', 'PC na mesma sub-rede do robô (/24)',
        f'PC em {mesmo}' if mesmo else f'nenhum IP local em {ip}/24 (PC: {", ".join(locais) or "?"})')
    for nome, porta in (('dashboard/ServoJ', DASHBOARD_PORT), ('feedback 8 ms', FEEDBACK_PORT)):
        ok, det = check_tcp(ip, porta)
        add('OK' if ok else 'FALHA', f'{ip}:{porta} ({nome})', det)

    add('OK' if in_dialout() else 'FALHA', 'usuário no grupo dialout (serial)',
        'sim' if in_dialout() else 'adicione: sudo usermod -aG dialout $USER e reabra a sessão')
    try:
        import serial  # noqa: F401
        add('OK', 'pyserial', 'instalado')
    except ImportError:
        add('FALHA', 'pyserial', 'sudo apt install python3-serial')
    if ft_port:
        add('OK' if os.path.exists(ft_port) else 'FALHA', f'porta da célula {ft_port}',
            'existe' if os.path.exists(ft_port) else 'não existe: conversor USB-RS485 plugado?')
    else:
        cands = serial_candidates()
        add('OK' if cands else 'FALHA', 'conversor USB-RS485 da FA7155 (auto-detect)',
            '; '.join(f'{d} ({x})' for d, x in cands) or
            'nenhum encontrado: plugue o conversor e confira o cabo/24 V da célula')
    add('AVISO', 'sentido dos eixos da FA7155', 'NÃO verificável aqui: empurre +X com o mastro tarado '
        'e veja /rehab/ft/wrench; ajuste ft_sign/ft_yaw_deg no launch')
    add('AVISO', 'E-Stop físico', 'confirme que corta o robô com o espelho armado (teste sem pessoa)')
    return r


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--robot-ip', default='')
    ap.add_argument('--ft-port', default='')
    a = ap.parse_args(argv)
    res = run_checks(a.robot_ip, a.ft_port)
    larg = max(len(n) for _, n, _ in res)
    for nivel, nome, det in res:
        print(f'{nivel:5s}  {nome:<{larg}}  {det}')
    falhas = sum(1 for n, *_ in res if n == 'FALHA')
    print(f'\n{falhas} FALHA(s). ' + ('Resolva antes de armar o robô.' if falhas
          else 'Pode subir: ros2 launch rehab_pack joystick_cell.launch.py control_mode:=mirror '
               f'force_source:=real robot_ip:={resolve_ip(a.robot_ip)}'))
    return 1 if falhas else 0


if __name__ == '__main__':
    sys.exit(main())
