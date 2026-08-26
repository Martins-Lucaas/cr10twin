"""ft_tcp.py — Transporte do FA7155 pela RS485 do FLANGE (porta 60000 do CR).

Mesmo sensor, mesmo quadro de 28 bytes, MESMO parser do ft_serial.py: muda só
o cano. Em vez do conversor USB-RS485 na mesa, a célula entra no conector
aviação do punho (manual CR A §6.2 — pino 1 = 485A, pino 2 = 485B, pino 5 =
24 V com 1 A nominal, pino 8 = GND) e o controlador expõe esse barramento cru
como um socket TCP.

Por que isso vale a pena: o cabo sobe por DENTRO do braço (não há laço
pendurado girando com o punho) e os 24 V saem do próprio flange, sem depender
do painel de I/O do armário.

O preço, e ele é real: a 485 do flange NÃO fala sozinha ao ligar. Ela precisa
de três comandos no dashboard (29999) antes de o primeiro byte aparecer — ver
`configure_tool_485`. E a 29999 é socket EXCLUSIVO do real_driver quando o robô
está em uso: por isso a configuração aqui é OPT-IN e não roda dentro do nó por
default. Rodar as duas coisas ao mesmo tempo embaralha comando e resposta no
dashboard.

Diferenças de comportamento em relação ao ft_serial.py:

* Não há auto-detect: o endereço é conhecido (o IP do robô). A "porta ausente"
  vira "conexão recusada", que o laço trata igual — continua tentando.
* `recv()` já devolve blocos; não existe o `in_waiting` do pyserial. O parser
  não se importa: ele foi escrito para consumir bytes crus em qualquer
  granularidade e ressincronizar por cabeçalho + CRC.
* O controlador é um INTERMEDIÁRIO. Ele repassa a linha 485, mas o jitter agora
  tem dois saltos (sensor→controlador, controlador→PC). O carimbo continua
  sendo do host e retro-datado pelo período nominal, exatamente como no USB.
"""
from __future__ import annotations

import socket
import threading
import time
from typing import Callable, Optional

from .constants import (
    FT_NOMINAL_RATE_HZ,
    FT_SERIAL_BAUD,
    FT_TCP_HOST,
    FT_TCP_PORT,
)
from .ft_cmd_channel import LineTapMixin
from .ft_serial import FtFrameParser

# Intervalo entre tentativas de reconexão (robô desligado, cabo de rede fora).
_RETRY_S = 2.0
# Timeout do recv: curto o bastante para o laço reavaliar self._running e o
# stop() não esperar, longo o bastante para não virar espera ocupada.
_RECV_TIMEOUT_S = 0.2

# Dashboard do CR. Mesmo valor de real_driver.DASH_PORT, repetido aqui de
# propósito: importar o real_driver arrastaria numpy para um módulo que só
# precisa de socket.
DASH_PORT = 29999


def _recv_line(sock: socket.socket) -> str:
    """Lê uma resposta ASCII do dashboard, terminada em ';' ou '\\n'."""
    buf = b''
    try:
        while b'\n' not in buf and b';' not in buf:
            chunk = sock.recv(2048)
            if not chunk:
                break
            buf += chunk
    except socket.timeout:
        pass
    return buf.decode('ascii', errors='replace').strip()


def configure_tool_485(host: str = FT_TCP_HOST,
                       baud: int = FT_SERIAL_BAUD,
                       dash_port: int = DASH_PORT,
                       timeout_s: float = 3.0,
                       power_cycle: bool = True) -> list[tuple[str, str]]:
    """Prepara a RS485 do flange e devolve ``[(comando, resposta), ...]``.

    Sem isto a porta 60000 conecta e fica MUDA — foi exatamente o que a bancada
    mostrou antes desta função existir: 8 s de escuta, zero bytes.

    Os três comandos, nesta ordem (guia TCP/IP §SetToolMode/§SetTool485/
    §SetToolPower):

      SetToolMode(1)          pinos 1/2 do aviação em modo 485, não AI
      SetTool485(baud,"N",1)  casa o baud com os 115200 do FA7155
      SetToolPower(0) → (1)   religa a ponta, para o sensor reinicializar

    O ciclo de energia é o que garante que o sensor comece do zero no baud de
    fábrica; o manual recomenda ≥4 ms entre chamadas de SetToolPower, e aqui a
    pausa é de 0,5 s para dar tempo de o firmware do sensor subir.

    ATENÇÃO: energiza o conector do flange. Não chame com o cabeamento da ponta
    em dúvida. E não chame com o real_driver conectado — a 29999 é dele.
    """
    cmds = ['RequestControl()', 'SetToolMode(1)', f'SetTool485({baud},"N",1)']
    if power_cycle:
        cmds += ['SetToolPower(0)', 'SetToolPower(1)']
    else:
        cmds += ['SetToolPower(1)']

    out: list[tuple[str, str]] = []
    sock = socket.create_connection((host, dash_port), timeout=timeout_s)
    try:
        sock.settimeout(0.5)
        _recv_line(sock)                     # descarta o banner de boas-vindas
        sock.settimeout(timeout_s)
        for cmd in cmds:
            sock.sendall((cmd + '\n').encode('ascii'))
            out.append((cmd, _recv_line(sock)))
            # Pausa só depois do power-off, que é onde ela significa algo.
            if cmd == 'SetToolPower(0)':
                time.sleep(0.5)
    finally:
        sock.close()
    return out


def configure_cabinet_485(host: str = FT_TCP_HOST,
                         baud: int = FT_SERIAL_BAUD,
                         slave_id: int = 1,
                         dash_port: int = DASH_PORT,
                         timeout_s: float = 3.0) -> list[tuple[str, str]]:
    """Configura a RS485 dos BORNES DO ARMÁRIO (485A/485B/RG do painel de I/O).

    Barramento diferente do flange: o do armário só tem uma via documentada de
    configuração, o `ModbusRTUCreate(slave_id, baud, parity, data_bit,
    stop_bit)` (guia TCP/IP), que faz o controlador virar MESTRE Modbus RTU
    naqueles bornes.

    Serve ao nosso propósito por um detalhe: `ModbusRTUCreate` só CRIA o
    mestre — quem gera tráfego são os `GetInRegs`/`GetCoils` seguintes. Criado
    e deixado quieto, ele configura a UART no baud pedido e não fala nada, que
    é exatamente a condição para escutar um talker passivo como o FA7155.

    PEGADINHA: o default de paridade é **"E"** (par), não "N". O FA7155 é
    8N1 — sem passar "N" explicitamente, a UART enquadra errado e o que sair
    da linha vira lixo em vez de quadro.

    Só faz sentido sob a hipótese de que a porta 60000 também cobre o
    barramento do armário, o que NÃO está documentado (ver docstring do
    módulo). Devolve ``[(comando, resposta), ...]``; a resposta do
    ModbusRTUCreate traz o índice do mestre, útil para o ModbusClose.
    """
    cmds = ['RequestControl()',
            f'ModbusRTUCreate({slave_id},{baud},"N",8,1)']
    out: list[tuple[str, str]] = []
    sock = socket.create_connection((host, dash_port), timeout=timeout_s)
    try:
        sock.settimeout(0.5)
        _recv_line(sock)
        sock.settimeout(timeout_s)
        for cmd in cmds:
            sock.sendall((cmd + '\n').encode('ascii'))
            out.append((cmd, _recv_line(sock)))
    finally:
        sock.close()
    return out


class FtTcpSource(LineTapMixin):
    """Leitor do FA7155 pela 60000, em thread de fundo.

    API idêntica à do FtSerialSource (start/stop/connected/last_rx/error/port/
    parser) — é o que permite o ft_receiver_node trocar de transporte sem saber
    de qual dos dois se trata.

    O callback recebe ``(seq, t_us, (fx, fy, fz, mx, my, mz))``, forças em N e
    momentos em N·m, igual ao caminho USB.
    """

    def __init__(self, host: str = FT_TCP_HOST,
                 tcp_port: int = FT_TCP_PORT,
                 rate_hz: float = FT_NOMINAL_RATE_HZ,
                 on_sample: Optional[
                     Callable[[int, int, tuple], None]] = None):
        self._host = str(host)
        self._tcp_port = int(tcp_port)
        self._period_us = 1e6 / max(float(rate_hz), 1.0)
        self._on_sample = on_sample
        # Mesmo nome do campo do FtSerialSource: é o que o nó imprime no log.
        self.port: Optional[str] = f'{self._host}:{self._tcp_port}'
        self.connected = False
        self.last_rx: float = 0.0
        self.error: str = ''
        self.parser = FtFrameParser()
        self._seq = 0
        self._running = False
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._tap_init()

    def start(self) -> bool:
        """Arma a thread. Sempre True: não há dependência opcional aqui (o
        socket é da stdlib), e robô inalcançável não é falha de partida — a
        thread fica tentando, igual ao hot-plug do USB."""
        if self._running:
            return True
        self._running = True
        self._thread = threading.Thread(
            target=self._worker, daemon=True, name='ft-tcp')
        self._thread.start()
        return True

    def stop(self) -> None:
        self._running = False
        sock = self._sock
        if sock is not None:
            # shutdown antes do close: destrava um recv() que já esteja
            # bloqueado, em vez de esperar o timeout.
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _worker(self) -> None:
        while self._running:
            try:
                sock = socket.create_connection(
                    (self._host, self._tcp_port), timeout=3.0)
            except Exception as exc:
                self.error = f'{self._host}:{self._tcp_port} — {exc}'
                time.sleep(_RETRY_S)
                continue
            sock.settimeout(_RECV_TIMEOUT_S)
            self._sock = sock
            self.connected = True
            self.error = ''
            try:
                self._read_loop(sock)
            except Exception as exc:
                self.error = str(exc)
            finally:
                self.connected = False
                self._sock = None
                try:
                    sock.close()
                except Exception:
                    pass
            if self._running:
                time.sleep(_RETRY_S)

    def _line_write(self, data: bytes) -> None:
        """Manda bytes pela 60000, que o controlador repassa à 485 do
        flange (canal de comando Modbus — ver ft_modbus).

        O controlador é intermediário também na ida: o `SetToolMode(1)` do
        `configure_tool_485` tem de ter rodado, senão os pinos 1/2 estão
        em modo AI e o byte não chega a virar sinal na linha.
        """
        sock = self._sock
        if sock is None:
            raise RuntimeError('socket da 60000 não está aberto')
        sock.sendall(data)

    def _read_loop(self, sock: socket.socket) -> None:
        while self._running:
            try:
                data = sock.recv(4096)
            except socket.timeout:
                continue
            if not data:
                # Peer fechou. Sai para o _worker reconectar.
                self.error = 'controlador fechou a conexão da 60000'
                return
            self._tap_feed(data)
            frames = self.parser.feed(data)
            if not frames:
                continue
            self.last_rx = time.monotonic()
            self._stamp_frames(frames)

    def _stamp_frames(self, frames: list[tuple[float, ...]]) -> None:
        """Numera e carimba os quadros de UMA leitura.

        Cópia deliberada do FtSerialSource._stamp_frames: o raciocínio é o
        mesmo (o sensor não manda relógio, um recv traz k quadros de uma vez, e
        dar a todos o mesmo instante faria dt=0 e derrubaria o One-Euro para a
        taxa nominal). Se um terceiro transporte aparecer, vale extrair para um
        mixin; com dois, a herança custaria mais do que estas dez linhas.
        """
        t_now_us = time.perf_counter() * 1e6
        k = len(frames)
        cb = self._on_sample
        for idx, vals in enumerate(frames):
            t_us = int(t_now_us - (k - 1 - idx) * self._period_us) & 0xFFFFFFFF
            seq = self._seq
            self._seq = (seq + 1) & 0xFFFFFFFF
            if cb is not None:
                cb(seq, t_us, vals)
