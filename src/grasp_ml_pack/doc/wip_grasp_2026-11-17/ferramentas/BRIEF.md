# Briefing comum — grasp_ml_pack (célula CR10 + mão COVVI no Gazebo)

Objetivo geral: o grasp 100% por CONTATO FÍSICO (sem attach cinemático, o usuário rejeita) deve levantar e entregar
os 3 objetos: frasco (cilindro r=25 mm, h=90 mm, palm_grip), tubo (r=12 mm, h=120 mm, claw_grip), ampola
(r=5 mm, h=75 mm, fingertip_grip). Sucesso = ciclo termina com `success=true` em /cell/status (CYCLE_DONE) sem
"SLIP", e o objeto vai para a caixa. Responda em português.

## Regras (obrigatórias)
- NÃO edite nada em /home/lucas-lpc/cr10twin. Trabalhe SÓ no seu workspace isolado (ver abaixo).
- Use SOMENTE seu ROS_DOMAIN_ID (e os scripts abaixo). Nunca use `pkill -f` genérico; pare sua sim com stop_sim.sh.
- Cada ciclo leva ~65 s de parede (sim <1x). Use poucos ciclos por hipótese (1 para triagem, 3 para confirmar).
- Orçamento: no máx. ~8 iterações de hipótese. Pare antes se atingir 3/3 ciclos com sucesso.
- Antes de explorar código leia /home/lucas-lpc/cr10twin/graphify-out/GRAPH_REPORT.md (grafo do projeto). A ferramenta
  "ponytail" não foi encontrada na máquina; ignore-a se não existir.
- Entregável final (resposta): (1) tabela hipótese → resultado (nº de ciclos ok), (2) o PATCH final como
  `diff -u` dos arquivos alterados contra /home/lucas-lpc/cr10twin/src/grasp_ml_pack (salve também em
  <ws>/final.patch), (3) causa raiz em 3-5 linhas, (4) o que NÃO funcionou. Seja factual: se não conseguiu, diga.

## Ferramentas (pasta ORCH=/tmp/claude-1000/-home-lucas-lpc-cr10twin/847d668c-3d80-4485-b424-058577a0b830/scratchpad/orch)
- `$ORCH/setup_ws.sh NOME` cria $ORCH/ws_NOME/src/grasp_ml_pack (cópia) + build overlay (symlink-install: editar a cópia
  vale direto; precisa REINICIAR a sim para recarregar).
- `$ORCH/run_sim.sh NOME DOMAIN` sobe a sim headless (~55 s). `$ORCH/stop_sim.sh DOMAIN` para (só seu domínio, ≥11).
- Em QUALQUER shell que fale com sua sim: `source /opt/ros/humble/setup.bash; source /home/lucas-lpc/cr10twin/install/setup.bash;
  source $ORCH/ws_NOME/install/setup.bash; export ROS_DOMAIN_ID=DOMAIN GAZEBO_MASTER_URI=http://127.0.0.1:$((11345+DOMAIN))`
- `OBJS=frasco REPS=3 CSV=/tmp/x.csv python3 $ORCH/cycles.py` roda REPS ciclos e imprime success/grasp_fail por ciclo.
- `python3 $ORCH/diag.py OBJ saida.json` roda 1 ciclo e grava, a ~20 Hz, estado, pose do `pick_object`, juntas do braço e
  posição das falanges (cr10_covvi::_<dedo>_{proximal,distal}_l1). Use para ver ONDE os dedos ficam vs o objeto.
- Log da sim: $ORCH/ws_NOME/launch.log (grep grasp_executor: F1..F7, perfect_grasp, SLIP, CAGE).

## Fatos já medidos (confiáveis)
- O braço chega EXATAMENTE ao q_pick planejado (erro <0,1°). O problema é a geometria mão↔objeto, não rastreamento.
- `poses._OBJ_IN_HAND[obj]` = centro do objeto no frame hand_base_link na pose de PICK (x,y,z). Em `_T_hand_base_world`
  (cage_check.py) está o frame real da mão; o Gazebo bate com ele (objeto medido == _OBJ_IN_HAND).
- Frame da mão (frasco/tubo laterais): os dedos curvam no plano (y,z) da mão; x da mão é a direção ao longo do cilindro
  (vertical no mundo). Origem das falanges em repouso: indicador x≈+0.023, médio +0.003, anelar −0.016, mínimo −0.033,
  polegar x≈+0.046..0.077 (y≈0.02..0.08). Com fechamento máximo as pontas ficam em y≈0.06, z≈0.01..0.02 (fist).
- `kinematics.hand_fk(cfg)` dá 'tip_<Dedo>' no frame da mão (modelo; serve p/ varrer posições candidatas por geometria
  ANTES de gastar um ciclo de sim). Distância de contato ≈ raio_obj + 3 mm (pele) + ~8 mm (meia ponta).
- Fechamento: grasp_executor F2 usa `PerfectGrasp.close_until_contact` (rampa de 0.06 rad/passo, detecta contato por LAG da
  falange; após contato alvo = contato + HOLD_SQUEEZE_RAD 0.15). Alvos máximos já em slider 200 (driver 1.0).
- Estado atual: frasco com `_OBJ_IN_HAND=(-0.010,0.045,0.050)` já faz os dedos TOCAREM (objeto desloca ~7 mm) mas é
  empurrado em vez de preso, nenhum contato detectado, lift falha (SLIP). O polegar (x≈0.047) fica fora da faixa do frasco
  (±45 mm), sem oposição. Antes disso (0.060 em y, 0.072 em z) nenhum dedo tocava.
- Ciclo falha por SLIP se o objeto não subir ≥5 cm após o lift (z do pick_object em /gazebo/model_states).
- Parâmetros de física/contato estão em launch/conveyor_cell.launch.py (kp/kd/mu da mão) e nos SDFs em
  grasp_ml_pack/conveyor_controller.py (massa, atrito mu, torsional dos objetos).
- Arquivos chave: poses.py (_OBJ_IN_HAND, _APPROACH_DIR/_APPROACH_DIST, HAND_GRIPS/HAND_PRESHAPE sliders 0..200,
  _Rtcp_lateral/_Rtcp_palm_down), grasp_executor.py (_run_cycle F1..F7), perfect_grasp.py, cage_check.py, collision.py.
