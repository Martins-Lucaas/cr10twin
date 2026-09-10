# Medições de latência da MÃO COVVI (Real-to-Sim)

Artefatos gerados pelo nó `touch_pack hand_latency_probe` — o análogo do
`latency_probe` (braço, em `../latency/`) para o outro elo do gêmeo digital.

Campanha de **10/09/2026**. As capturas em `pre-correcao/` foram feitas com o
bug de escala descrito no histórico abaixo: valem para LATÊNCIA, não para erro
angular. As do diretório principal são pós-correção e valem para as duas coisas.

## Como gerar

```bash
# Terminal 1 — plataforma com a mão, e a mão física conectada pela GUI
# (botão Connect → o power ON automático liga o mirror real→sim):
ros2 launch touch_pack tactile_cell.launch.py end_effector:=hand

# Terminal 2 — 20 s movimentando os dedos pela GUI (sliders ou botões de
# grip; os dois dão a mesma precisão). `auto` detecta o sentido pelo sinal:
ros2 run touch_pack hand_latency_probe --ros-args \
    -p direction:=auto -p duration_s:=20.0
```

O probe grava direto aqui (`data/latency_hand/`), que é versionado. Depois:

```bash
ros2 run touch_pack latency_report -- data/latency_hand
```

## O que cada arquivo contém

Mesmo formato do braço (ver `../latency/README.md`), com as 6 juntas
primárias da mão (`Thumb, Index, Middle, Ring, Little, Rotate`) no lugar das
6 do braço. O `_result.json` traz dois campos a mais: a taxa efetiva da
telemetria (`real_telemetry_rate_hz`) e a quantização da contagem ECI
(`real_quantization_deg`).

## Diferenças de instrumentação em relação ao braço

| | Braço | Mão |
|---|---|---|
| timeline REAL | poll readonly do CR10, 125 Hz | telemetria `DigitPosnAll`, **200 Hz** em movimento |
| quantização | — | 0,596°/contagem ECI |
| resolução do estimador | ±0,15 ms | ±0,9 ms (teste sintético, janela de 20 s) |

O stream `DigitPosnAll` é orientado a evento: cai para 10 Hz de keepalive com
a mão parada e sobe para ~200 Hz enquanto os dedos se movem. Os 10 Hz vistos
com a mão em repouso **não** limitam a medição.

## Resultado (pós-correção — 4 capturas, 24 pares)

| grandeza | mão (5 dedos) | braço (`../latency/`) |
|---|---|---|
| latência Real-to-Sim | **135,2 ± 3,1 ms** (N=20) | 71,5 ± 0,3 ms (N=17) |
| MAE (atraso compensado) | 0,977 ± 0,187° | 0,009° |
| RMSE compensado / cru | 1,63° / 5,39° (3,3×) | 0,024° / 0,558° (23×) |
| ganho sim/real | 0,936 ± 0,016 | — |

**O `Rotate` é mais rápido que os dedos, e isso agora está sustentado:**
111,6 ± 3,0 ms (N=4) contra 135,2 ± 3,1 ms. A diferença de 23,6 ms é ~7σ da
dispersão de qualquer um dos dois grupos. Nas duas primeiras capturas ele
tinha amplitude de só 4,7° e correlação 0,97, e por isso valia descartar; nas
duas últimas ele se moveu 10,4° com correlação 0,98 e o número não mudou. É
resultado, não artefato — reportar as duas latências separadas, não uma média.

Hipótese para a diferença (não verificada): o `Rotate` move o chassi do
polegar, com cadeia mimic mais curta que a dos dedos — menos juntas para o
controlador do Gazebo assentar.

## Histórico: o bug de escala (resolvido em 10/09/2026)

As 4 primeiras capturas (`pre-correcao/`) mediram ganho 0,77 e MAE de 6,3°.
Não era infidelidade do gêmeo: a junta do URDF é o **driver**, cuja faixa é
0,12–1,0 rad (6,9–57,3°), e 1,0 rad de driver dá ~163° na ponta do dedo pela
cadeia mimic. A conversão mandava grau de ponta de dedo (0–90°) direto para o
driver, então tudo acima de 57,3° ceifava no teto e o piso de 6,9° levantava a
base — exatamente o ganho de 0,77 com offset de +7°.

O `Rotate` escapava (ganho 0,95) porque é mapeado para 0–60° contra um teto de
57,3°, quase coincidente. Foi a pista que descartou as outras hipóteses.

Duas explicações foram testadas e falharam antes de chegar nesta: limite de
velocidade do URDF (o sim atinge 200+°/s contra um limite de 57°/s, ou seja
não é aplicado) e filtro passa-baixa da dinâmica do controlador (o ganho não
cai com a frequência — correlação ganho×frequência = +0,21, sinal errado).

Confirmação quantitativa: modelar o bug como `clip(pose, 0.12, 1.0)` reproduz
o ganho medido nos 20 pares com erro médio de 0,017, sem parâmetro livre.

**Correção:** `touch_pack.constants.hand_deg_to_driver_rad` passou a ser a
fronteira entre os dois espaços angulares, e `HAND_DRIVER_{LOWER,UPPER}_RAD`
viraram autoridade única — `hand_pack.urdf_helpers` (clamp do URDF) e
`touch_pack.kinematics` importam de lá em vez de manter cópias. Efeito medido:
ganho 0,77 → 0,95, offset +7° → +1,2°, MAE 6,3° → 0,86°. **A latência não
mudou** (136,58 → 136,57 ms): a correlação cruzada é invariante a escala, que
é por que as capturas antigas seguem válidas para ela.

## Critérios de qualidade de uma captura

- `peak_corr >= 0.9`;
- amplitude >= 2° no dedo dominante, em graus de DRIVER (a contagem ECI
  quantiza em 0,6° de ponta de dedo, ~0,33° de driver);
- >= 150 amostras de telemetria (o probe avisa) — na prática, 20 s de janela.
