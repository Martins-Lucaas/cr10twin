# Diagnóstico de grasp — sessão de 2026-11-17 (trabalho em andamento)

Estado: **nenhum objeto foi pego com sucesso ainda.** Os agentes foram interrompidos antes de relatar resultado final;
os patches em `*.patch` são parciais e NÃO foram validados em ciclo completo (exceto onde indicado).

## O que já está aplicado no repositório (main, não commitado)
- `grasp_ml_pack/poses.py`: `_OBJ_IN_HAND['frasco']` = `(-0.010, 0.045, 0.050)` (antes `(-0.025, 0.060, 0.072)`).
  Com isso os dedos passam a raspar o frasco (objeto desloca ~5–7 mm), mas **não há pega** (ver abaixo).
- `scripts/test_9cycles.py`: agora espera `CYCLE_DONE` em `/cell/status` (antes contava "ciclo iniciado" como sucesso),
  `TIMEOUT_S = 90`.
- `install/` reconstruído com `colcon build --packages-select grasp_ml_pack --symlink-install`
  (o install antigo fazia `test_perfect_grasp` e `test_fk_urdf` falharem).

## Fatos medidos (confiáveis)
1. O braço chega ao `q_pick` planejado com erro <0,1°. Não é problema de rastreamento.
2. O objeto, no frame real da mão (`cage_check._T_hand_base_world`), coincide com `_OBJ_IN_HAND`. O planejamento é
   coerente com o Gazebo; o que falha é a geometria escolhida.
3. **Raio real do frasco = 25 mm (Ø50)**, h=90 mm (SDF em `conveyor_controller.py`). Comentários com "Ø84" são
   antigos e errados. Tubo r=12 mm h=120; ampola r=5 mm h=75.
4. Frasco, geometria antiga: pontas fechavam a 58–66 mm do eixo (contato ≈ 36 mm); objeto 0 mm de deslocamento.
5. Frasco, geometria nova: fechamento chega ao alvo (driver 1.0; polegar 0.90) com **lag = 0 em todos os 15 passos** →
   a detecção de contato do PerfectGrasp nunca dispara → `HOLD_SQUEEZE` aperta no vazio → SLIP (z=0,851 após lift).
6. As pontas dos dedos fazem um arco em "fist": `hand_fk` indicador (y=0.156,z=0.002) → apex (0.101,0.042) → fim
   (0.057,0.007). O centro `(y=0.045,z=0.050)` fica FORA e acima do arco. A cavidade real fica perto de
   `palm_center=(0.005, 0.077, -0.007)`, i.e. y≈0.075, z≈0 (não validado em sim).
7. O polegar (x≈+0.047) fica acima do topo do frasco (±45 mm): não faz oposição.
8. Física NÃO é o gargalo (agente "física"): objetos mu=mu2=2,5, torsional 0,8; mão mu 2,5, kp 5e4; falange p=3,
   limite 1 N·m. Sem contato real, nenhum parâmetro de física/fechamento muda o resultado.
9. Ciclo completo leva ~65 s de parede (sim <1x); o executor fica `busy` durante isso.
10. Baseline `test_9cycles` (antes de qualquer correção de geometria): frasco 3/3 `grasp_fail`, tubo #1 `grasp_fail`
    (rodada interrompida).

## Hipóteses/patches parciais dos agentes (não validados)
- `frasco_a.patch` (lateral): `_OBJ_IN_HAND['frasco']=(0.030, 0.052, 0.0475)` para o polegar entrar na faixa.
  Contém uma linha `DBG` de log em `perfect_grasp.py` (remover). Sem resultado de ciclo.
- `frasco_b.patch` (top-down): `_APPROACH_DIR['frasco']=(0,0,-1)`, `_Rtcp_palm_down(finger_dir=(1,0,0))`,
  `_PALM_NORMAL`, `_OBJ_IN_HAND=(0.060,0.110,0.060)`, HAND_GRIPS frasco = Thumb 200/Index 30/resto 0 (pinça), poses
  cacheadas regeneradas. Sem resultado de ciclo.
- `tubo.patch`: via PRE antes do APPROACH (o sweep de juntas HOME→APPROACH derrubava o tubo: z 0.866→0.818),
  `_OBJ_IN_HAND['tubo']=(-0.010,0.058,0.026)`, Rotate do claw 40→0, poses cacheadas regeneradas.
  Achado independente e provavelmente válido: **o sweep HOME→APPROACH derruba o tubo antes do pick**.
- `ampola.patch`: detector HSV: máscara do topo (`mask[:140,:]=0`, anel azul do braço dava falso "tubo"),
  kernels 3×3 e área mínima 60 para ampola (era apagada pelo open 7×7); `_OBJ_IN_HAND['ampola']` z 0.0552→0.0450;
  Index do fingertip 109→16. Achado independente: **o detector não via a ampola** ("Nenhum objeto válido detectado").

## Próximos passos sugeridos
1. Frasco: usar a cavidade da mão como alvo (y≈0.075, z≈0 no frame da mão) e varrer com `hand_fk` incluindo o
   polegar; só então rodar sim. Se o lag continuar 0, não avançar ao lift: avançar o braço alguns mm e refechar.
2. Aplicar um a um os achados independentes (via PRE do tubo; detector da ampola) e validar cada um com
   `OBJS=<obj> REPS=3 python3 ferramentas/cycles.py`.
3. Rodar `test_9cycles.py` completo no repositório principal só depois do item 1/2.

## Ferramentas (`ferramentas/`)
Scripts para N simulações isoladas em paralelo (workspace overlay + `ROS_DOMAIN_ID` + porta do Gazebo próprios):
`setup_ws.sh`, `run_sim.sh NOME DOMAIN`, `stop_sim.sh DOMAIN` (≥11), `cycles.py` (N ciclos por objeto, env
`OBJS`,`REPS`), `diag.py` (1 ciclo + posições de falanges/objeto a 20 Hz). Os scripts têm caminhos do scratchpad
antigo (variável `$ORCH`/`S`): ajuste antes de reusar. `BRIEF.md` é o briefing dado aos agentes.
Cuidado: `pkill -f` com padrão presente na própria linha de comando mata o shell (código 144).
`ponytail` não existe nesta máquina.
