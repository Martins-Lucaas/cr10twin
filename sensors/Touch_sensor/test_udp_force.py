"""Teste mínimo da recepção UDP da célula de carga (sem GUI, sem serial).

Rode no PC (Windows ou Linux) que NÃO está recebendo força:

    python test_udp_force.py

Saída esperada (firmware multi-assinante de 13/07 + rede OK):

    hello enviado p/ 192.168.5.105:8090
    1.0s: 100 pacotes, 1000 amostras, v_sensor=0.000123
    2.0s: 100 pacotes, ...

Se ficar em "0 pacotes" por mais de ~3 s:
  • Firewall do Windows bloqueando UDP de entrada p/ o Python — permita
    python.exe em Redes Privadas (ou crie regra de entrada UDP 8080).
  • PC fora da rede 192.168.5.x — confira com `ipconfig`.
  • Firmware antigo na ESP (um destino só) — regrave: pio run -e ota -t upload
"""

import socket
import struct
import time

ESP_IP = "192.168.5.105"
UDP_PORT = 8080
DISCOVERY_PORT = 8090
MAGIC = b"FRCV"
SAMPLE_FMT = "<IIf"
SAMPLE_SZ = struct.calcsize(SAMPLE_FMT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
try:
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
except (AttributeError, OSError):
    pass
sock.settimeout(0.5)
# Windows: ICMP unreachable de um hello não pode matar o recvfrom.
# ValueError: socket.ioctl pode rejeitar SIO_UDP_CONNRESET — o except
# ConnectionResetError do laço cobre sozinho nesse caso.
try:
    SIO_UDP_CONNRESET = getattr(socket, "SIO_UDP_CONNRESET", 0x9800000C)
    sock.ioctl(SIO_UDP_CONNRESET, struct.pack("I", 0))
except (AttributeError, ValueError, OSError):
    pass
sock.bind(("", UDP_PORT))
print(f"bind OK em 0.0.0.0:{UDP_PORT}")

t0 = time.monotonic()
last_hello = 0.0
last_report = t0
pkts = samples = 0
v_last = float("nan")

while True:
    now = time.monotonic()
    if now - last_hello >= 2.0:
        last_hello = now
        try:
            sock.sendto(MAGIC, (ESP_IP, DISCOVERY_PORT))
            print(f"hello enviado p/ {ESP_IP}:{DISCOVERY_PORT}")
        except OSError as e:
            print("erro ao enviar hello:", e)
    try:
        raw, addr = sock.recvfrom(2048)
    except socket.timeout:
        raw = None
    except ConnectionResetError:
        print("ConnectionResetError (ICMP) — ignorando, ESP pode estar fora")
        raw = None
    if raw:
        n = len(raw) // SAMPLE_SZ
        if n:
            pkts += 1
            samples += n
            (_seq, _t_us, v_last) = struct.unpack_from(
                SAMPLE_FMT, raw, (n - 1) * SAMPLE_SZ)
    if now - last_report >= 1.0:
        last_report = now
        print(f"{now - t0:5.1f}s: {pkts} pacotes, {samples} amostras, "
              f"v_sensor={v_last:.6f}")
        pkts = samples = 0
