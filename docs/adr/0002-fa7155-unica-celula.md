---
status: accepted
---

# FA7155 é a única célula de carga; o caminho XIAO/ESP32 foi removido

Desde 2026-09-15 não há mais ESP32 na bancada (nem o XIAO ESP32S3 original nem o ESP32C6 que o substituiu em 2026-08-28), e a célula axial de 100 kg com HX711 saiu junto. A única célula de carga é a **FA7155 de 6 eixos**, ligada por RS485 a um conversor **CH343** (VID `0x1A86`, PID `0x55d3`) e lida pelo `ft_receiver`, sem firmware nosso no caminho.

Em 2026-10-21 todo o caminho antigo foi apagado do repositório: `sensors/ForceDriver/`, `force_receiver`, `lc_serial`, `lc_health_probe`, a aba de calibração da GUI, `sensors/load_cell_calib.json` e os argumentos de launch `force_sensor`/`lc_port`. Para recuperar algo, o último commit que ainda tem esse código é `ae6c29a`.

## Consequences

- Runs gravados antes da troca têm `force_sensor: load_cell` no `params.json` e as colunas `lc_voltage_*` em volts. Os novos gravam `ft6` e newtons.
- A taxa assumida até a primeira medida é `FT_NOMINAL_RATE_HZ` (400 Hz). O datasheet promete 500 a 1000 Hz, mas o exemplar só responde a poll e o round-trip USB do CH343 limita a entrega a cerca de 390 a 399 Hz (medido em 12 runs de 09/2026 e ao vivo em 2026-10-21).
- O ruído vem da FA7155 em repouso (2026-10-21): σ de 22 mN no Fz cru e de 2,1 mN depois do filtro.
- O canal lento (`/load_cell/force_net_slow`) existia para baixar o σ do HX711. Com 2,1 mN no canal rápido, contra uma banda de 20 mN, ele foi removido.
