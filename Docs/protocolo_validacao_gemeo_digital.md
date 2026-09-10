# Plano de correção do artigo + protocolo de coleta

**Projeto:** cr10twin / `touch_pack` · **Bancada:** Dobot CR10 (`192.168.5.2`) + mão
COVVI (ECI `192.168.5.103`) · **Efetuador:** `end_effector:=hand`
**Artigo:** *From Simulation to Hardware: A Digital Twin as a Development Platform
for a Collaborative Robotic Arm and a Multi-Articulated Hand*

**Criado:** 10/09/2026 · **Revisado:** 10/09/2026 (dados recuperados, tabelas geradas)

---

## 1. Estado atual — o que já está resolvido

Os dados brutos das 7 capturas do artigo **foram recuperados da lixeira** e agora
vivem em `data/latency/` (versionado; ver o comentário no `.gitignore`). Com eles,
uma parte grande do parecer se fecha **sem bancada nenhuma**.

```bash
ros2 run touch_pack latency_report -- data/latency
# → data/latency/latency_summary.csv    (uma linha por par captura-junta)
# → data/latency/latency_tables.tex     (tabelas em IEEEtran, prontas)
```

### 1.1 Resultados já disponíveis

| Condição | N | \|Δt\| (ms) | MAE (°) | RMSE (°) | r mínimo |
|---|---|---|---|---|---|
| drag teach (amplitude 0,36–33,16°) | 17 | 71,54 ± 0,30 | 0,009 ± 0,007 | 0,024 ± 0,022 | 0,999976 |
| GUI jog (amplitude 2,86–3,87°) | 3 | 78,22 ± 1,42 | 0,116 ± 0,017 | 0,271 ± 0,050 | 0,995260 |

Por junta, na condição drag teach — **o atraso é o mesmo em todas as seis**, o que
sustenta empiricamente a afirmação de que ele é dominado pelo caminho de transporte
compartilhado e não pela dinâmica de cada junta:

| Junta | N | \|Δt\| (ms) | MAE (°) | erro máx (°) | r |
|---|---|---|---|---|---|
| joint1 | 3 | 71,6 ± 0,1 | 0,003 | 0,15 | 1,00000 |
| joint2 | 3 | 71,4 ± 0,2 | 0,006 | 0,18 | 0,99999 |
| joint3 | 4 | 71,4 ± 0,1 | 0,012 | 0,17 | 0,99999 |
| joint4 | 4 | 71,7 ± 0,5 | 0,011 | 1,09 | 0,99998 |
| joint5 | 2 | 71,3 ± 0,2 | 0,015 | 1,33 | 0,99999 |
| joint6 | 1 | 71,9 | 0,010 | 0,58 | 1,00000 |

**O argumento novo mais forte do artigo sai daqui:** sem compensar o atraso o RMSE
é 0,558°; compensando, cai para 0,024° — **23× menor**. Ou seja, o desacordo entre
gêmeo e braço físico é **atraso, não infidelidade**. O gêmeo reproduz a postura real
com fidelidade de centésimos de grau.

### 1.2 Unidade de análise

A reanálise usa o par **(captura, junta)**, não a captura. Cada junta que se moveu é
uma medição independente do mesmo acoplamento. É assim que 4 capturas de drag teach
viram 17 medições cobrindo duas ordens de grandeza de amplitude — variedade de
configuração de movimento a partir do que já foi coletado, que é metade do item 1
do parecer.

---

## 2. O achado que obriga a corrigir o texto

> **As duas latências publicadas (78,2 e 71,8 ms) medem o MESMO caminho de dados.**
> Não são Sim-to-Real e Real-to-Sim.

**Prova nos próprios dados:** os sete arquivos se chamam `latency_real_to_sim_*`.
Nas três capturas de "GUI jog", o `_result.json` traz
`direction_requested: sim_to_real` e `direction_detected: real_to_sim`. A ferramenta
avisou na hora da coleta.

**Causa no código:** em `MIRROR` com a fase de palpação `IDLE` — o estado durante o
jog manual — o `_mirror_poll_loop` chama `_mirror_follow_tick()`
(`palpation_gui.py:3155`), e esse tick lê o feedback do braço real e o **republica no
tópico de comando do Gazebo** (`:3363-3369`). Cada `MovJ` de jog abre uma janela de
*follow* real→sim de até 15 s (`:3006-3008`). Durante a captura, o Gazebo seguia o
braço físico.

Os ~6 ms de diferença entre as condições são amplitude de movimento, não direção.

### 2.1 Onde o Sim-to-Real de verdade acontece

| Caminho | Quando | Transporte | Contaminado pelo follow? |
|---|---|---|---|
| `_run_movement_once` (aba Poses & Motions) | execução de movimento salvo | `MovJ` por pose | **Não** — `_exec_movement_id` faz o poll loop pular o follow (`:3147`) |
| `_mirror_poll_loop` ServoJ | fase de palpação ATIVA | `ServoJ` 33 Hz de `/joint_states` | **Não** |
| jog por slider / manip 3D | fase `IDLE` | `MovJ` debounce 80 ms | **Sim** — mede real→sim |

### 2.2 O que dá para dizer sobre Sim-to-Real hoje

Uma estimativa por instante de início de movimento nas 3 capturas de GUI jog dá
**8 eventos entre 60 e 264 ms**. Indica que o caminho Sim-to-Real é algumas vezes
mais lento que o espelhamento (coerente com 80 ms de debounce + `MovJ` + rampa), mas
com esse espalhamento não é número de tabela. Some-se que a velocidade de pico do sim
chega a 49°/s enquanto a do real fica travada em 19,5°/s pelo `SpeedFactor` — os
perfis diferem, e o limiar de início já vicia a medida.

**Serve como justificativa do trabalho futuro. Não serve como resultado.**

---

## 3. Triagem do parecer

### 3.1 Fechado pela reanálise (sem bancada)

| Item do parecer | Como fecha |
|---|---|
| 6 — MAE/RMSE/erro máx/correlação/Δt por junta | Tabelas do §1.1, já geradas |
| 1 — mais configurações de movimento | 17 medições, 6 juntas, 0,36–33,16° |

### 3.2 Só texto e figura (sem bancada)

| # | O quê | Esforço |
|---|---|---|
| 1 | Corrigir os rótulos da Tabela II e da §Results — sem isso o item 2 não fecha e o erro é achável por qualquer revisor com acesso aos dados | 1 h |
| 2 | Figs. 4 e 5 estão **com legendas em português** num artigo em inglês (`fig_modos_sim_real.svg`, `fig_estado_inicial.svg`) | 1 h |
| 3 | `images/fig_latencia_metodo.svg` já existe, pronta, e **não está no artigo** — entra como painel do método | 30 min |
| 4 | Limitações: latência é **só do braço** (a mão nunca foi caracterizada), um objeto por arquétipo, sem repetição, sem forças/torques/reality gap | 1 h |
| 5 | Posicionamento vs. trabalhos anteriores: a tríade (abstração única + sincronização automática + bidirecionalidade). Precisa de passada de literatura | 3–4 h |
| 6 | Refazer a Fig. 8 com os dados reanalisados: painel do método + dispersão por junta | 2 h |

### 3.3 Precisa de bancada — **em ordem de prioridade**

| Prio | Bloco | Por quê nessa posição |
|---|---|---|
| **1** | **E — preensão** (§6) | **Único item do parecer que a reanálise não alcança.** E é o mais barato: ~90 min, zero código, zero ferramenta nova |
| 2 | B — Sim-to-Real (§5) | A direção que o artigo não mede. Exige a correção do §5.1 |
| 3 | D — sincronização inicial (§7) | Barato, objetiva a Fig. 5 |
| 4 | A — Real-to-Sim repetido (§4) | **Rebaixado**: a reanálise já entrega 17 medições. Só vale para repetibilidade da MESMA trajetória |
| 5 | C — ServoJ perfil casado (§8) | Opcional; mexe no laço crítico de segurança |

### 3.4 Não fazer

Forças de contato, torques e quantificação formal do *reality gap*. O próprio parecer
concede que fiquem como limitação.

---

## 4. Pré-voo (uma vez, no início do dia)

1. Controlador CR10 em **REMOTE** (`Settings → Operate Mode`, ou `http://192.168.5.2`).
2. Nenhuma outra sessão conectada ao robô — o `latency_probe` abre mais uma.
3. Mão COVVI energizada: `ping 192.168.5.103`.
4. Diretório da campanha, **fora** de `sensors/Data/` (que é `.gitignore`d):
   ```bash
   export ART=~/artigo_dados
   mkdir -p "$ART"/{A_real2sim,B_sim2real,C_servoj,D_sync,E_grasp}
   ```
   O `latency_probe` grava em `$TOUCH_PACK_DATA_DIR/latency/`; cada bloco define esse
   env var. Ao final, copie os `*_raw.csv` e `*_result.json` para `data/latency/` e
   rode o `latency_report` de novo.
5. Fumaça, com o robô parado (vai reclamar de amplitude — é o esperado):
   ```bash
   TOUCH_PACK_DATA_DIR=$ART/smoke ros2 run touch_pack latency_probe --ros-args \
       -p robot_ip:=192.168.5.2 -p duration_s:=10.0
   ```

---

## 5. Bloco B — Sim-to-Real (prioridade 2)

O comando nasce no gêmeo e vai aos dois destinos a partir do mesmo instante:
trajetória completa para o Gazebo, `MovJ` por pose para o CR10.

### 5.1 ⚠️ Bug que contamina este bloco — corrigir ANTES de coletar

O detector automático de drag (`palpation_gui.py:6166`) liga o drag sozinho quando vê
as juntas se moverem mais de 0,8° com **2 s** sem comando vindo do PC
(`DRAG_SILENCE_S = 2.0`, `:6106`). E `_run_movement_once` **não atualiza**
`self._last_robot_cmd_t` ao enviar o `MovJ` (`:4735-4740`) — só o jog por slider atualiza.

Resultado: ~2–3 s depois do início da execução, o detector conclui "movimento sem
comando do PC" e liga o drag. A partir daí `_mirror_poll_loop` testa
`if self._drag_enabled:` **antes** de `if self._exec_movement_id is not None:`
(`:3090` vs `:3147`) e passa a espelhar real→sim — a contaminação que este bloco
existe para evitar.

**Correção mínima**, em `_run_movement_once`, logo após `drv.mov_j_joint_deg(...)`:
```python
self._last_robot_cmd_t = time.monotonic()
```

**Enquanto não corrigir:** o sintoma é visível — o botão vira `✋ Drag (auto)` e a
barra de status mostra *"Physical drag detected"*. Se acontecer numa captura,
**descarte a captura**.

### 5.2 Preparar o movimento (uma vez; salvo em `~/.config/touch_pack/poses.json`)

Aba **Poses & Motions**, robô em `MIRROR`, **drag desligado**:

1. Capture 4 poses com **⌨ Sim**, bem separadas, dentro do envelope seguro:

   | Pose | joint1 | joint2 | joint3 | joint4 | joint5 | joint6 |
   |---|---|---|---|---|---|---|
   | P0 | 0 | 0 | −90 | 0 | 90 | 0 |
   | P1 | +25 | −10 | −75 | 0 | 90 | 0 |
   | P2 | 0 | −25 | −60 | +20 | 75 | 0 |
   | P3 | −25 | −10 | −75 | −20 | 105 | 0 |

   Valores de partida — ajuste ao espaço livre real, mas **anote os usados**.
2. **＋ Novo movimento** → `artigo_s2r` → P0→P1→P2→P3→P0.
3. `dur_s = 2.5` s, `speed_pct = 30`. Uma passagem = 12,5 s.

> O perfil do Gazebo (interpolação do `joint_trajectory_controller`) e o do CR10
> (`MovJ` com `SpeedFactor`) **não são o mesmo perfil** — declarar como limitação. Por
> isso a métrica principal aqui é o **atraso de início de movimento**, não o RMSE ao
> longo da trajetória.

### 5.3 Capturar (5 repetições)

```bash
TOUCH_PACK_DATA_DIR=$ART/B_sim2real ros2 run touch_pack latency_probe --ros-args \
    -p direction:=sim_to_real -p robot_ip:=192.168.5.2 -p duration_s:=28.0
```
Assim que aparecer `Capturando por 28 s`, aperte **↻ Loop** em `artigo_s2r`. Duas
passagens cabem na janela. Ao fim, **■ Stop**. Repita 5× → 10 passagens → **40
transições de pose**.

O probe vai avisar `ATENÇÃO: você pediu 'sim_to_real' mas o sinal indica ...` —
**ignore**. A correlação cruzada compara perfis diferentes aqui. O que vale é o
`*_raw.csv`.

---

## 6. Bloco E — Taxa de sucesso de preensão (PRIORIDADE 1)

Troca as duas fotos da Fig. 9 por taxa de sucesso com intervalo de confiança. Sem
código: comando pela GUI e uma planilha.

### 6.1 Montagem

- **Gabarito.** Marque com fita a posição exata do objeto na bancada, com o contorno
  da base e uma marca de orientação. Todas as tentativas partem da mesma geometria —
  sem isso a taxa mede a sua pontaria, não a preensão.
- **Poses salvas.** Na aba **Poses & Motions**, para cada objeto:
  `P_approach` (mão aberta na posição de pega) e `P_lift` (mesma pose, +100 mm em Z).
  Monte um movimento com as duas.
- **Fotografe o gabarito montado** — vira figura de método ou material suplementar.

### 6.2 Condições

| Cond. | Objeto | Grip COVVI | `eci_id` |
|---|---|---|---|
| G1 | garrafa (a mesma da Fig. 9a) | **Power** | 2 |
| G2 | cubo pequeno (o mesmo da Fig. 9b) | **Prec. Closed** | 5 |

Opcionais, se sobrar tempo (fortalecem a generalização sem bancada nova):
G3 = lata cilíndrica / **Cylinder** (8) · G4 = cartão / **Key** (6).

### 6.3 Sequência da tentativa

1. Objeto no gabarito. Mão em **Glove** (aberta).
2. Executa o movimento até `P_approach`.
3. Aba **Manual Control**, card *COVVI Grips*: seleciona o grip da condição, **✓ Apply**.
4. Aguarda 2 s (fechamento).
5. Executa até `P_lift`. **Segura 5 s.**
6. *(robustez, opcional)* rotaciona `joint6` ±45° e volta.
7. Desce, aplica **Glove**, remove o objeto, recoloca no gabarito.

### 6.4 Critério de sucesso — fixar ANTES de começar, nunca depois

> **Sucesso** = o objeto permanece preso durante os 5 s de sustentação a 100 mm, sem
> escorregar mais de 10 mm em relação à mão e sem cair.

**N = 20 por condição.** Com 20/20 o intervalo de Wilson 95 % é **[83,9 %, 100 %]** —
suficiente para o artigo. Com menos de 20 o intervalo fica largo demais para valer a pena.

### 6.5 Planilha

`$ART/E_grasp/grasp_trials.csv`:
```csv
trial,condicao,objeto,grip,eci_id,sucesso,modo_falha,observacao
1,G1,garrafa,Power,2,1,,
2,G1,garrafa,Power,2,0,escorregou_na_subida,
```
`modo_falha` ∈ {`sem_contato`, `escorregou_no_fechamento`, `escorregou_na_subida`,
`escorregou_na_rotacao`, `objeto_deslocado_na_aproximacao`}.

Os modos de falha valem tanto quanto a taxa: são eles que sustentam a seção de
limitações e apontam se o limite é geometria, força de fechamento ou complacência.

### 6.6 Fotos novas para a Fig. 9

- **JPEG, não HEIC.** `images/` já tem `.HEIC` que o LaTeX não lê — o artigo usa as
  versões `.jpg` convertidas. Configure a câmera para JPEG e evite a conversão.
- **Mesma posição, altura e enquadramento de câmera nos dois (ou quatro) painéis.**
  Painéis comparáveis são metade do que o revisor pediu ao falar da Fig. 9. Um tripé,
  ou uma marca de fita no chão, resolve.
- **Fotografe durante os 5 s de sustentação, com o objeto no ar.** Uma foto com o
  objeto apoiado na mesa não prova preensão — prova contato.
- **Fundo neutro e limpo.** Tire da cena tudo que não for braço, mão e objeto.
- **Uma foto por condição, no mesmo instante do ensaio** (meio da sustentação).
- Se fizer G3/G4, a Fig. 9 vira um painel 2×2 — planeje o enquadramento para os
  quatro ficarem iguais.
- Fotografe também **uma falha representativa**, se ocorrer. Vale mais para a seção
  de limitações do que qualquer parágrafo.

---

## 7. Bloco D — Sincronização inicial (Fig. 5)

Troca o "cerca de três segundos" por curva de convergência medida e erro residual por
junta, em 10 subidas com poses de partida diferentes. O `real_pose_sync` roda em
**todo** launch, independente do `control_mode` (`tactile_cell.launch.py:497,635`).

Para cada uma das 10 repetições:

1. Leve o braço real a uma pose diferente. Anote.
2. **Terminal 2 primeiro** (o probe precisa gravar antes de o Gazebo subir):
   ```bash
   TOUCH_PACK_DATA_DIR=$ART/D_sync ros2 run touch_pack latency_probe --ros-args \
       -p robot_ip:=192.168.5.2 -p duration_s:=60.0
   ```
3. **Terminal 1**, imediatamente depois:
   ```bash
   ros2 launch touch_pack tactile_cell.launch.py \
       end_effector:=hand control_mode:=sim_only robot_ip:=192.168.5.2
   ```
4. Deixe o probe fechar sozinho, depois derrube o launch.

O braço real fica **parado** durante a sincronização — só o gêmeo se move. O número de
latência dessas capturas **não tem significado e deve ser descartado**. O que importa é
o `*_raw.csv`: nele está a série do Gazebo convergindo sobre a série constante do real.

---

## 8. Blocos A e C (prioridade baixa)

### 8.1 Bloco A — Real-to-Sim repetido

**Rebaixado**: o §1.1 já entrega 17 medições em 6 juntas e duas ordens de grandeza de
amplitude. O único ganho restante é repetir a *mesma* trajetória — e uma trajetória
conduzida à mão não repete.

Se for fazer: launch em `control_mode:=mirror`, **✋ Drag ON**, e
```bash
TOUCH_PACK_DATA_DIR=$ART/A_real2sim ros2 run touch_pack latency_probe --ros-args \
    -p direction:=real_to_sim -p robot_ip:=192.168.5.2 -p duration_s:=20.0
```
Matriz sugerida, 2 repetições cada: T1 joint1 ±25° lento · T2 joint1 ±25° rápido ·
T3 joint5 ±25° lento · T4 joint1+2+3 ±15° · T5 joint4 ±5° (baixa amplitude de propósito).
Aceite: `peak_corr ≥ 0,95` e `movement_amp_deg ≥ 2,0` (T5: `≥ 0,5`).

> **Automação possível.** `_toggle_drag` é só uma flag de software
> (`palpation_gui.py:4550-4571`) — não põe o CR10 em modo drag; quem libera os motores
> é o botão físico do braço. Com a flag ligada e o botão **não** pressionado, o braço
> continua sob servo, e um `MovJ` enviado direto pelo driver o move enquanto o
> `_mirror_poll_loop` republica o feedback de 125 Hz no Gazebo (`:3091-3126`) — o mesmo
> ramo de código do drag teach, com trajetória repetível. Guarda: `_mirror_movj_send`
> **desliga** o drag (`:2999-3001`), então o runner tem de chamar
> `drv.mov_j_joint_deg()` direto.

### 8.2 Bloco C — Sim-to-Real com perfil casado

O `mirror_node` só faz `ServoJ` streaming com a fase publicada em `/palpation/status`
diferente de `IDLE`/`DONE`/`ABORTED` (`mirror_node.py:274-280`). No `end_effector:=hand`
o modo Palpação é bloqueado (`palpation_gui.py:1927`) e o explorer publica `IDLE` a
10 Hz sem parar (`tactile_explorer.py:1788`). Contorno sem alterar código:

```bash
# T1 — plataforma SEM mirror e SEM GUI
ros2 launch touch_pack tactile_cell.launch.py \
    end_effector:=hand control_mode:=sim_only no_gui:=true robot_ip:=192.168.5.2
# T2 — mirror com o status remapeado
ros2 run touch_pack mirror_node --ros-args \
    -p robot_ip:=192.168.5.2 -r /palpation/status:=/latency/phase
# T3 — CONFIRME que sim e real concordam antes de armar o ServoJ
ros2 topic echo /joint_states --once
```

> ⚠️ **Segurança.** O primeiro `ServoJ` descarrega a diferença sim↔real inteira num
> único tick de 30 ms, **sem rampa** — é o que o pré-home da GUI existe para evitar
> (`palpation_gui.py:3172-3180`). Só publique a fase ativa depois de confirmar
> divergência **< 0,3°** em todas as juntas. Mão no botão de emergência.

```bash
# T3 — arma o ServoJ
ros2 topic pub -r 10 /latency/phase touch_pack_msgs/msg/PalpationStatus "{phase: 'TRANSIT'}"
# T4 — captura
TOUCH_PACK_DATA_DIR=$ART/C_servoj ros2 run touch_pack latency_probe --ros-args \
    -p direction:=sim_to_real -p robot_ip:=192.168.5.2 -p duration_s:=20.0
# T5 — move o gêmeo (primeiro ponto = pose ATUAL, senão o braço salta)
ros2 topic pub --once /cr10_group_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory \
  "{joint_names: [joint1,joint2,joint3,joint4,joint5,joint6],
    points: [
      {positions: [ 0.35, 0.0, -1.57, 0.0, 1.57, 0.0], time_from_start: {sec: 4}},
      {positions: [-0.35, 0.0, -1.57, 0.0, 1.57, 0.0], time_from_start: {sec: 9}},
      {positions: [ 0.35, 0.0, -1.57, 0.0, 1.57, 0.0], time_from_start: {sec: 14}},
      {positions: [ 0.00, 0.0, -1.57, 0.0, 1.57, 0.0], time_from_start: {sec: 18}}]}"
```
Ao terminar, **mate o T3 antes de tudo** (fase volta a `IDLE`, o ServoJ para).

---

## 9. Análise

Tudo é pós-processamento dos `*_raw.csv`, pelo `latency_report`. Ele já faz, por par
(captura, junta): estimativa do atraso por correlação cruzada com refino parabólico,
**compensação do atraso**, e então MAE, RMSE, erro máximo e correlação de Pearson.

A compensação é o ponto todo: sem ela o "erro" é dominado pelos ~71 ms e não diz nada
sobre fidelidade cinemática.

**Ainda não implementado no `latency_report`** (some quando os blocos rodarem):

- **Bloco B** — atraso de **início de movimento**: para cada transição de pose, o
  instante em que |velocidade| de cada série cruza 0,5 °/s pela primeira vez. 40
  eventos → média ± desvio. O RMSE ao longo da trajetória vai separado e rotulado como
  *discrepância de perfil*, não como erro de rastreamento.
- **Bloco D** — tempo até o erro sim↔real cair abaixo de 0,5° e nele permanecer, e erro
  residual por junta na média dos 2 s finais.

**Tabelas do artigo:**

| Tabela | Fonte | Status |
|---|---|---|
| Latência por condição | `latency_tables.tex` | ✅ pronta |
| Fidelidade cinemática por junta | `latency_tables.tex` | ✅ pronta |
| Preensão: N, sucessos, taxa, IC 95 % Wilson, modos de falha | Bloco E | ⬜ falta coletar |
| Sincronização inicial: tempo de acomodação, erro residual | Bloco D | ⬜ falta coletar |

---

## 10. Ferramentas

| O quê | Onde | Status |
|---|---|---|
| `latency_report` — reanálise, resumo CSV e tabelas LaTeX | `touch_pack/latency_report.py` | ✅ escrito e testado (`test/test_latency_report.py`) |
| **Correção do §5.1** — `_last_robot_cmd_t` em `_run_movement_once` | `palpation_gui.py` | ⬜ **bloqueia o Bloco B**. Uma linha |
| Métricas de início de movimento (Bloco B) e de acomodação (Bloco D) | `latency_report.py` | ⬜ só necessário se os blocos rodarem |
| Runner de campanha na GUI (aba automatizando A/B/C) | módulos novos | ⬜ ~1–1,5 dia. Só se pagar em campanha repetida |
| Probe de latência da **mão** (`/joint_states` × `DigitPosnAll`) | nó novo | ⬜ fora de escopo. A latência publicada é só do braço |

---

## 11. Orçamento de bancada

| Prio | Bloco | Tempo |
|---|---|---|
| — | Pré-voo | 20 min |
| **1** | **E — preensão (40 tentativas + fotos)** | **90 min** |
| 2 | B — Sim-to-Real (5 × 28 s + montar poses) | 60 min |
| 3 | D — sincronização (10 × 60 s) | 40 min |
| 4 | A — Real-to-Sim repetido | 45 min |
| 5 | C — ServoJ | 45 min |

**Se der para fazer só uma coisa: o Bloco E.** É o único item do parecer que a
reanálise não alcança, e é o mais barato de todos.
