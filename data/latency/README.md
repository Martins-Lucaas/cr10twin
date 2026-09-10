# Medições de latência Sim-to-Real / Real-to-Sim

Artefatos gerados pelo nó `touch_pack latency_probe` para preencher os
valores de latência da Seção 4.6 do TCC (e slides da defesa).

## Como gerar (bancada, versão com a mão — `end_effector:=hand`)

```bash
# Terminal 1 — plataforma em MIRROR com o CR10 real conectado:
ros2 launch touch_pack tactile_cell.launch.py end_effector:=hand \
    control_mode:=mirror robot_ip:=192.168.5.2

# Terminal 2 — Sim-to-Real: durante os 20 s, mova o braço PELA GUI
# (sliders / movimentos salvos), de forma ampla e contínua:
ros2 run touch_pack latency_probe --ros-args \
    -p direction:=sim_to_real -p duration_s:=20.0

# Terminal 2 — Real-to-Sim: ative o Drag na GUI e conduza o braço À MÃO:
ros2 run touch_pack latency_probe --ros-args \
    -p direction:=real_to_sim -p duration_s:=20.0
```

Repetir **3–5 capturas por sentido**. Depois: `git add sensors/Data/latency`
e commit — a análise final é feita a partir destes arquivos.

## O que cada arquivo contém

| Sufixo | Conteúdo |
|---|---|
| `_raw.csv` | As DUAS séries brutas: `t_mono_s, source(sim\|real), joint1..6_rad`. Relógio monotônico comum do processo. Permite refazer qualquer análise. |
| `_aligned.csv` | Par reamostrado (grade uniforme) da junta usada na correlação, em graus. Pronto para plotar. |
| `_result.json` | Latência (ms), convenção de sinal, correlação de pico, lag por junta, nº de amostras, parâmetros da captura. |
| `.png` | Gráfico sobreposto sim × real com o atraso anotado — direto para o slide. |

## Convenção de sinal

- `lag > 0` → o REAL atrasa em relação ao SIM → latência **Sim-to-Real**.
- `lag < 0` → o SIM atrasa em relação ao REAL → latência **Real-to-Sim**.

## Critérios de qualidade de uma captura

- `peak_corr ≥ 0.9` (o nó avisa se ficar abaixo);
- movimento com amplitude ≥ 0.5° na junta dominante;
- para o número final: média ± desvio das 3–5 capturas de cada sentido.
