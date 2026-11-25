# Roteiro pós-coleta: trava e correções do rehab_pack

Faça na ordem. Se um passo falhar, pare.

1. Feche o touch_pack. Confira que nada ficou: `pgrep -af install/touch_pack/lib`.
2. Suíte (domínio isolado, sem hardware):
   ```bash
   export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1 CR10_PACK_LOCK_DIR=$(mktemp -d)
   (cd src/touch_pack && PYTHONPATH=$PWD:$PYTHONPATH python3 -m pytest -q test)
   ```
   Os testes do rehab rodam no passo 4, depois do build (precisam do pacote instalado).
3. Build num terminal novo: `colcon build --packages-select touch_pack rehab_pack`.
4. `source install/setup.bash` e, com o ambiente do passo 2, um arquivo por vez em
   `src/rehab_pack/test`. `test_gui_smoke.py` precisa de tela.
5. Trava, com o CR10 desconectado e **em outro terminal sem `CR10_PACK_LOCK_DIR`**:
   - touch_pack aberto: o launch do rehab tem de recusar com "touch_pack (ou hand_pack/grasp_ml_pack) está aberto".
   - rehab aberto: o launch do touch_pack tem de recusar.
   - `kill -9` no que estiver aberto: o outro tem de abrir.
6. Painel do rehab (sim): lista suspensa aberta + Espaço envia PARAR; roda do mouse não
   move junta; Home pede confirmação; duplo clique em Desabilitar não reabilita; fechar
   durante um jog para o braço; `kill -STOP` no rehab_gui com o braço ATIVO leva a
   ABORTADO em ~1 s (`kill -CONT` depois).
7. Jog no Gazebo: partida suave e recusa de cotovelo junto à mesa.
8. Bancada (só o operador move o braço): meça o trecho simulado → robô com o touch_pack
   em MIRROR (`latency_probe`, `direction:=sim_to_real`; ele toma a trava do touch_pack,
   então não roda com o rehab aberto). Some o trecho força → simulado em `LOOP_DELAY_S` e confira os eixos e o
   sinal da FA7155 antes de qualquer voluntário.
9. `code-review-graph build`.

## Commits sugeridos

1. `touch_pack: real_pose_sync e setup` — as alterações que já existiam antes.
2. `trava entre touch_pack e rehab_pack` — pack_lock, launches, mains, serial exclusiva, docker, ADR.
3. `rehab_pack: segurança do espelho, admitância, jog e painel`.
4. `rehab_pack: dados em REHAB/` — runs_dir, meta, relatórios do touch_pack.
5. `remove touch_pack/admittance`.
6. `docs: restaura ADRs e roteiro`.
