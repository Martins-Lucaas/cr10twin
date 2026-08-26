#!/usr/bin/env python3
"""ft_probe.py — Bancada da célula FA7155, SEM ROS.

É o equivalente do env `serial_test` do firmware antigo: o primeiro programa a
rodar depois de ligar o conversor USB-RS485, para responder três perguntas na
ordem em que elas aparecem:

  1. Chega alguma coisa na porta?          → `--raw` (hexdump cru)
  2. Os quadros fecham o CRC?              → o rodapé conta erros/ressincs
  3. Qual eixo/sinal é a força de contato? → aperte a ponteira e olhe a tabela

A (3) é a que decide `ft_force_axis`/`ft_force_sign` do nó: aperte a ponteira
contra a bancada e veja qual canal se move. Se ele ficar NEGATIVO, o nó precisa
de `ft_force_sign:=-1.0` (que já é o default); se ficar positivo, `+1.0`.

Uso:
    python3 ft_probe.py                    # auto-detect da porta
    python3 ft_probe.py --port COM5        # Windows
    python3 ft_probe.py --port /dev/ttyUSB0
    python3 ft_probe.py --list             # portas seriais visíveis
    python3 ft_probe.py --raw              # hexdump, sem interpretar
    python3 ft_probe.py --zero             # tara na partida (1 s parado)

Rota do FLANGE (RS485 pela porta 60000 do controlador, sem conversor USB):

    python3 ft_probe.py --tcp --configure  # configura a 485 e escuta
    python3 ft_probe.py --tcp              # só escuta (já configurada)

O --configure manda SetToolMode/SetTool485/SetToolPower na 29999. Sem ele a
60000 conecta e fica MUDA. NÃO use com o real_driver no ar: a 29999 é dele.
"""
from __future__ import annotations

import argparse
import os
import socket
import sys
import time

# Roda direto da árvore do repo, sem `colcon build` no meio.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import serial                                                   # noqa: E402
from serial.tools import list_ports                             # noqa: E402

from touch_pack.constants import (                              # noqa: E402
    FT_AXES, FT_NOMINAL_RATE_HZ, FT_SERIAL_BAUD, FT_TCP_HOST, FT_TCP_PORT,
    FT_USB_VIDS,
)
from touch_pack.ft_serial import (                              # noqa: E402
    FtFrameParser, detect_ft_serial_port,
)
from touch_pack.ft_tcp import (                                # noqa: E402
    configure_cabinet_485, configure_tool_485,
)

_PRINT_PERIOD_S = 0.2


class _SockReader:
    """Faz o socket da 60000 parecer um serial.Serial.

    cmd_raw e cmd_stream só usam read() e in_waiting — implementando esses
    dois, os dois laços de bancada servem aos DOIS transportes sem alteração,
    que é o que garante que a leitura testada seja a mesma dos dois lados.
    """

    def __init__(self, sock: socket.socket):
        self._s = sock
        self._s.settimeout(0.2)
        self._buf = bytearray()
        # Peer fechou. Sem isto, um recv() que devolve b'' faz read() voltar
        # vazio IMEDIATAMENTE e para sempre: os dois laços de bancada tratam
        # "sem dados" como timeout e giram a 100 % de CPU imprimindo "1 s sem
        # quadro" — que ainda por cima acusa 24 V/A-B quando o que caiu foi o
        # socket. Os laços consultam este flag e saem dizendo a verdade.
        self.eof = False

    @property
    def in_waiting(self) -> int:
        return len(self._buf)

    def read(self, n: int = 1) -> bytes:
        while len(self._buf) < n:
            try:
                chunk = self._s.recv(4096)
            except socket.timeout:
                break
            if not chunk:
                self.eof = True    # controlador fechou
                break
            self._buf += chunk
        out = bytes(self._buf[:n])
        del self._buf[:n]
        return out

    def close(self) -> None:
        self._s.close()


def cmd_list() -> int:
    ports = list(list_ports.comports())
    if not ports:
        print('Nenhuma porta serial visível.')
        return 1
    for p in ports:
        vid = f'0x{p.vid:04X}' if p.vid is not None else '  ?   '
        mark = ' <- conversor RS485' if p.vid in FT_USB_VIDS else ''
        print(f'{p.device:<20} VID={vid} PID='
              f'{f"0x{p.pid:04X}" if p.pid is not None else "?"}  '
              f'{p.description}{mark}')
    return 0


def cmd_raw(ser) -> int:   # serial.Serial ou _SockReader
    """Hexdump do que chega. Linha vazia por segundo = nada na linha:
    célula sem 24 V, A/B trocados ou GND sem referência comum."""
    print('[raw] hexdump — Ctrl+C para sair')
    last = time.monotonic()
    total = 0
    while True:
        data = ser.read(1)
        if data:
            waiting = getattr(ser, 'in_waiting', 0)
            if waiting:
                data += ser.read(waiting)
            total += len(data)
            print(data[:64].hex(' '))
        elif getattr(ser, 'eof', False):
            print('[raw] o controlador FECHOU a conexão da 60000.')
            return 1
        now = time.monotonic()
        if now - last >= 1.0:
            if total == 0:
                print('[raw] 1 s sem NENHUM byte — confira 24 V, A/B e GND.')
            last = now
            total = 0


def cmd_stream(ser, zero: bool) -> int:   # idem cmd_raw
    parser = FtFrameParser()
    tare = [0.0] * len(FT_AXES)
    zero_buf: list[tuple] = []
    zeroed = not zero
    if zero:
        print('[ft] tara: mantenha a célula DESCARREGADA por ~1 s...')

    last_print = time.monotonic()
    t_rate = last_print
    n_rate = 0
    rate = 0.0
    peak = [0.0] * len(FT_AXES)
    print('[ft] Ctrl+C para sair. Aperte a ponteira e veja qual canal se move.')
    while True:
        data = ser.read(1)
        if not data:
            if getattr(ser, 'eof', False):
                print('[ft] o controlador FECHOU a conexão da 60000. '
                      'Reconecte (a 485 do flange pode precisar de '
                      '--configure de novo).')
                return 1
            now = time.monotonic()
            if now - last_print >= 1.0:
                last_print = now
                print('[ft] 1 s sem quadro — confira 24 V, A/B e GND.')
            continue
        waiting = getattr(ser, 'in_waiting', 0)
        if waiting:
            data += ser.read(waiting)
        frames = parser.feed(data)
        if not frames:
            continue
        n_rate += len(frames)

        if not zeroed:
            zero_buf.extend(frames)
            if len(zero_buf) >= int(FT_NOMINAL_RATE_HZ):
                n = float(len(zero_buf))
                tare = [sum(f[k] for f in zero_buf) / n
                        for k in range(len(FT_AXES))]
                zeroed = True
                print('[ft] tara aplicada: '
                      + '  '.join(f'{FT_AXES[k]}={tare[k]:+.3f}'
                                  for k in range(len(FT_AXES))))
            continue

        vals = [frames[-1][k] - tare[k] for k in range(len(FT_AXES))]
        for k in range(len(FT_AXES)):
            if abs(vals[k]) > abs(peak[k]):
                peak[k] = vals[k]

        now = time.monotonic()
        if now - t_rate >= 1.0:
            rate = n_rate / (now - t_rate)
            n_rate = 0
            t_rate = now
        if now - last_print < _PRINT_PERIOD_S:
            continue
        last_print = now
        cur = '  '.join(f'{FT_AXES[k]}={vals[k]:+8.3f}'
                        for k in range(len(FT_AXES)))
        pk = '  '.join(f'{peak[k]:+.2f}' for k in range(len(FT_AXES)))
        print(f'{cur}   |  {rate:5.1f} Hz  crc={parser.crc_errors} '
              f'resync={parser.resyncs}  pico[{pk}]')


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--port', default=None, help='porta serial do conversor')
    ap.add_argument('--baud', type=int, default=FT_SERIAL_BAUD)
    ap.add_argument('--list', action='store_true', help='lista as portas e sai')
    ap.add_argument('--raw', action='store_true', help='hexdump sem interpretar')
    ap.add_argument('--zero', action='store_true', help='tara na partida')
    ap.add_argument('--tcp', action='store_true',
                    help='usa a RS485 do flange (porta 60000) em vez do USB')
    ap.add_argument('--host', default=FT_TCP_HOST, help='IP do controlador')
    ap.add_argument('--tcp-port', type=int, default=FT_TCP_PORT)
    ap.add_argument('--configure', action='store_true',
                    help='com --tcp: prepara a 485 pela 29999')
    ap.add_argument('--bus', choices=('flange', 'cabinet', 'both'),
                    default='flange',
                    help='qual 485 o --configure prepara (default: flange)')
    args = ap.parse_args()

    if args.list:
        return cmd_list()

    if args.tcp:
        if args.configure:
            print(f'[485] configurando ({args.bus}) em {args.host}...')
            try:
                if args.bus in ('flange', 'both'):
                    for cmd, resp in configure_tool_485(host=args.host,
                                                        baud=args.baud):
                        print(f'[485/flange]  {cmd} -> {resp}')
                if args.bus in ('cabinet', 'both'):
                    for cmd, resp in configure_cabinet_485(host=args.host,
                                                           baud=args.baud):
                        print(f'[485/armario] {cmd} -> {resp}')
            except Exception as exc:
                print(f'Falha ao configurar a 485 em {args.host}: {exc}')
                return 1
        else:
            print('[485] sem --configure: se a ponta nunca foi preparada '
                  'nesta sessão, a 60000 vai ficar muda.')
        print(f'[ft] {args.host}:{args.tcp_port} (485 do flange)')
        try:
            sock = socket.create_connection((args.host, args.tcp_port),
                                            timeout=3.0)
        except Exception as exc:
            print(f'Falha ao conectar em {args.host}:{args.tcp_port}: {exc}')
            return 1
        ser = _SockReader(sock)
    else:
        port = args.port or detect_ft_serial_port()
        if port is None:
            print('Conversor USB-RS485 não encontrado. Use --list para ver as '
                  'portas e --port para escolher uma.')
            return 1
        print(f'[ft] {port} @ {args.baud}')
        try:
            ser = serial.Serial(port, args.baud, timeout=0.2)
        except Exception as exc:
            print(f'Falha ao abrir {port}: {exc}')
            return 1
    try:
        return cmd_raw(ser) if args.raw else cmd_stream(ser, args.zero)
    except KeyboardInterrupt:
        return 0
    finally:
        ser.close()


if __name__ == '__main__':
    sys.exit(main())
