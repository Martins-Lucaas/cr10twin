# Auditoria touch_pack, 24/11/2026 (AGENTE 1)

Base: git archive de 893e34d em /tmp/wsA_base. Ambiente: WSL Ubuntu-22.04, ROS 2 Humble, overlay /tmp/wsA, ROS_DOMAIN_ID=71. Sem hardware, Gazebo ou firmware do CR10.

## Vermelho e verde por arquivo novo

| Arquivo | Falham na base | Passam agora |
|---|---|---|
| test_driver_confirmacao.py | 37 | 46 |
| test_explorer_seguranca.py | 38 | 46 |
| test_espelho_gui_seguranca.py | 35 | 42 |
| test_dados_gui_sensores.py | 43 | 45 |
| test_revisao_cruzada.py | 30 mutações | 52 |
| test_gen_tcp_meshes.py | 1 | 2 (com OCP) |

A diferença são controles (comportamento legítimo preservado) e os itens "não reproduzido" ou "já corrigido". Suíte completa (`python3 -m pytest -p no:cacheprovider test`, com test_palp_char): base 1161 passam, 6 skips, 0 falhas, 82,8 s; atual **1408 passam, 8 skips, 0 falhas** (com /tmp/wsA_opt: 1410 passam, 6 skips).

Skips no ambiente normal: 8. Nenhum por esta mudança.

- test_gen_tcp_meshes x2 (sem OCP): rodados com `cadquery-ocp-novtk` em /tmp/wsA_opt (pip --target; venv indisponível, sem python3-venv). Passam os 2; na base o da peça avulsa falha.
- test_ft_arrow x2: o cliente de fábrica existe no Windows desta máquina, mas o caminho `C:\Program Files (x86)\费波斯六维力客户端\OBJ` não resolve no Linux. Rodados no WSL com cópia do OBJ sob esse nome: 20 passam, 0 skips.
- test_imu_node_e2e x4: BLOQUEADO. PyQt6 e ahrs instalados em /tmp/wsA_opt; falta `~/WBAN-Inercial/Software main/Sensores Inerciais Nova Interface/core` (imu_node.py:45), repositório à parte, ausente do WSL. E, com ele, o teste faz multicast na rede e põe em RUN qualquer ESP real ligado: exige bancada sem sensores.

Flakes: `test_slide_pacing`, `test_explorer_logic` (hold_exits, hold_times_out, hold_aborts_when_force_freezes) e `test_logger_columns` corrigidos: relógio virtual nos dois primeiros, espera do casamento DDS no terceiro. Com 16 processos queimando CPU: 6 de 6 repetições limpas.

XR-CRUZ: os defeitos nasceram na fase 1, então a base não os tem. A prova de vermelho é por mutação: 30 reversões do conserto numa cópia (/tmp/wsA_mut), 30 pegas por test_revisao_cruzada.py (52 testes). T40: 8 mutações de produção, 8 pegas pelos testes convertidos.

Testes que continuam estruturais (T40), um a um:

- `test_mixin_is_actually_mixed_in`, `test_mixin_does_not_shadow_the_host`, `test_no_method_was_lost_in_the_split`, `test_every_self_reference_in_the_mixin_resolves`: a propriedade É a estrutura da classe (MRO, sombra, método perdido); exercitar cada botão não a garante.
- `test_load_cell_group_left_the_host`, `test_shared_gui_constants_have_a_home_of_their_own`: onde o símbolo mora; duplicar passaria em qualquer teste de comportamento.
- test_explorer_split, todos: idem para as fatias do explorer (constante em dois lugares, import do host).
- `test_mixin_declares_no_import_from_the_host`: ciclo de import, invisível enquanto a ordem de carga calhar certa.
- test_attack_modes_gui, test_hand_connect_ready, test_sensors_detach: extraem o método por AST, mas executam o corpo de produção; são comportamentais.
- test_sim_wrist_rigidity: o objeto testado é o arquivo world/URDF.
- test_adc_frame_integrity e test_align_attack: falso positivo da lista original; leem arquivo gerado pelo próprio teste.

Atenção: com ROS sourced, um `pytest.importorskip` em nível de módulo que chegue a pular zera a coleta da sessão inteira (launch_testing). Restam skips de módulo por tkinter, serial e xacro em touch_pack e rehab_pack; só disparam se a dependência declarada faltar.

## Itens

| ID | Agente | Evidência | Causa | Correção | Teste (test_*.py) | Estado |
|---|---|---|---|---|---|---|
| XC1 | 1 | IK da pose tombada a 25/40 graus | solver por candidatos trocava o ramo do cotovelo | solução contínua a partir da junta atual; salto de ramo recusado | explorer_seguranca: test_ik_do_ataque_nao_troca_de_ramo, test_giro_com_salto_de_ramo_e_recusado | corrigido |
| XC2 | 1 | mirror_node e gui_robot limpavam o erro e reenviavam ServoJ | rearme automático no laço | recusa trava e congela; rearme só por ação do operador | espelho_gui_seguranca: test_mirror_recusa_de_servoj_trava_sem_limpar_erro, test_gui_recusa_de_servoj_nao_rearma_e_congela | corrigido |
| XC3 | 1 | socket falso: -10000 virava True | retorno ignorado | True só com código 0; GUI não anuncia sem confirmação | driver_confirmacao: test_estop_so_confirma_com_zero; espelho_gui_seguranca: test_estop_sem_confirmacao_nao_anuncia_braco_desabilitado | corrigido |
| XC4 | 1 | socket cai na espera do lock | atributo None | confere o socket depois do lock | driver_confirmacao: test_estop_com_socket_caindo_durante_a_espera_do_lock | corrigido |
| XC5 | 1 | |F|>100 N descartada | filtro de plausibilidade engolia a batida | força alta dispara a guarda | explorer_seguranca: test_batida_acima_de_100N_dispara_a_guarda, test_nao_finito_nao_vira_leitura | corrigido |
| XC6 | 1 | Loop de Poses recomeçava ao soltar o E-STOP | evento de parada não acionado | E-STOP e Stop acionam; recusa encerra o loop | espelho_gui_seguranca: test_loop_de_poses_morre_no_estop, test_movj_recusado_encerra_o_loop, test_stop_palpacao_tambem_para_as_poses | corrigido |
| XC7 | 1 | Start pendente sobrevivia ao Stop no pré-home | worker não consultava cancelamento | _prehome_cancel no laço e na chegada | espelho_gui_seguranca: test_stop_e_estop_cancelam_o_start_pendente, test_stop_depois_da_chegada_ainda_cancela | corrigido |
| XC8 | 1 | Poses e palpação simultâneas | portões ausentes | recusa mútua | espelho_gui_seguranca: test_poses_recusadas_com_palpacao_ou_estop, test_start_recusado_com_poses_rodando | corrigido |
| T01 | 1 | HOLD/MANUAL acumulava avanço | sem orçamento de curso | aborta sem resposta de força | explorer_seguranca: test_hold_com_braco_parado_aborta_antes_de_acumular_curso, test_hold_em_contato_mole_que_responde_nao_aborta | corrigido |
| T02 | 1 | reconexão sem comparar com a pose real | retomada direta | exige sincronia | espelho_gui_seguranca: test_mirror_retomada_compara_com_a_pose_real, test_gui_espelho_nao_comeca_dessincronizado | corrigido |
| T03 | 1 | FREEZE em MATRIX_MAP aliviava e subia | _matrix_bail não via o freeze | congela no lugar | explorer_seguranca: test_freeze_no_matrix_map_nao_alivia_nem_sobe, test_freeze_interrompe_o_alivio | corrigido |
| T04 | 1 | HOME/MovJ sem guarda | laço sem força | guarda nos movimentos articulares | explorer_seguranca: test_home_com_forca_para_no_lugar, test_giro_do_ataque_com_forca_para | corrigido |
| T05 | 1 | silêncio = sucesso, respostas desalinhadas | sem confirmação/associação | timeout levanta, atrasadas descartadas, movimento nunca reenviado | driver_confirmacao: test_movimento_sem_resposta_levanta_e_nao_reenvia, test_respostas_atrasadas_nao_trocam_de_comando | corrigido |
| T06 | 1 | /clock zero aceitava Start antigo | idade com now=0 | relógio zero/reiniciado tratado | explorer_seguranca: test_idade_do_start | corrigido |
| T07 | 1 | só Fz vigiada | guarda escalar | força lateral na guarda | explorer_seguranca: test_colisao_lateral_dispara_com_fz_zero, test_atrito_de_deslize_nao_dispara | corrigido |
| T08 | 1 | done por tempo | sem comparar pose | chegada conferida pela pose | explorer_seguranca: test_transito_descartado_pelo_jtc_nao_e_done, test_home_final_falha_nao_vira_done | corrigido |
| T09 | 1 | falha ao sair do drag deixava colisão 0 | sem restauração na exceção | restaura sempre | driver_confirmacao: test_arrasto_que_falha_restaura_colisao, test_excecao_no_meio_do_arrasto_restaura_colisao | corrigido |
| T10 | 1 | sem E-STOP com no_gui | mirror_node não assinava | assina freeze/E-STOP | espelho_gui_seguranca: test_mirror_freeze_e_estop_sem_gui | corrigido |
| T11 | 1 | README sem home_deg | mensagem zerada aceita | exemplos corrigidos e entrada validada | explorer_seguranca: test_exemplos_do_readme_passam_na_validacao, test_start_invalido_e_recusado | corrigido |
| T12 | 1 | FROZEN sem saída no MIRROR | portões só IDLE/DONE/ABORTED | Stop em FROZEN leva a IDLE sem mover | explorer_seguranca: test_stop_encerra_o_frozen_sem_mover; espelho_gui_seguranca: test_soltar_estop_em_frozen_devolve_o_explorer_a_idle | corrigido |
| T13 | 1 | 50 de 500 spikes | best-effort depth 1 | reliable depth 1000 | dados_gui_sensores: test_rajada_de_spikes_chega_inteira | corrigido |
| T14 | 1 | pausa pulava ciclos | t0 não deslocava | t0 desloca | explorer_seguranca: test_pausa_nao_consome_ciclos_da_onda | corrigido |
| T15 | 1 | 34/40 leituras falhavam | parcial descartado | reassembla entre chamadas | driver_confirmacao: test_porta_lenta_entrega_todo_pacote, test_pacote_fragmentado_e_retomado | corrigido |
| T16 | 1 | reconexão perdia feedback_port | argumento esquecido | mesmo argumento | espelho_gui_seguranca: test_reconexao_leva_a_mesma_porta_de_feedback | corrigido |
| T17 | 1 | cabeçalho falso aceito | verificação tautológica | confere TestValue | driver_confirmacao: test_cabecalho_falso_nao_vira_pose | corrigido |
| T18 | 1 | payload do launch ignorado | parâmetro não lido | chega ao driver | espelho_gui_seguranca: test_mirror_payload_do_launch_chega_ao_driver | corrigido |
| T19 | 1 | NaN/inf aceitos | sem isfinite | recusa no driver, nó e GUI | driver_confirmacao: test_movimento_recusa_nao_finito; explorer_seguranca: test_setpoint_manual_nao_finito_e_recusado | corrigido |
| T20 | 1 | dois drivers | reconexão não cancelada | Connect cancela; perdedor fechado | espelho_gui_seguranca: test_connect_durante_reconexao_nao_cria_segundo_driver, test_driver_da_reconexao_cancelada_e_fechado | corrigido |
| T21 | 1 | IMU voltava com referência antiga | zero não refeito | refaz o zero e segura a pose | dados_gui_sensores: test_volta_do_sensor_nao_persegue_a_mao | corrigido |
| T22 | 1 | campo vazio travava Start | float sem proteção | valida antes | espelho_gui_seguranca: test_campo_vazio_nao_trava_o_start | corrigido |
| T23 | 1 | fechar não parava o robô primeiro | stop no fim | freeze e stop primeiro | espelho_gui_seguranca: test_fechar_a_janela_para_o_robo_primeiro, test_fechar_sobrevive_a_falha_na_parada, test_fechar_congela_tambem_o_pre_home_pendente | corrigido |
| T24 | 1 | Tk fora da thread principal | acesso em thread ROS/worker | cache e root.after | espelho_gui_seguranca: test_status_de_outra_thread_passa_pelo_after, test_movj_do_jog_nao_le_variavel_tk_fora_da_thread; dados_gui_sensores: test_mensagem_de_imu_nao_toca_variavel_tk | corrigido |
| T25 | 1 | abrir câmera bloqueava | VideoCapture na thread Tk | abre na captura | dados_gui_sensores: test_camera_fora_do_ar_nao_segura_a_thread_do_tk, test_camera_presa_no_read_nao_tem_a_captura_solta_por_baixo | corrigido |
| T26 | 1 | J6 absoluto da câmera | sem rampa | parte da pose atual, rampa, bloqueios | dados_gui_sensores: test_j6_da_camera_parte_de_onde_esta_e_em_rampa, test_j6_da_camera_respeita_os_bloqueios | corrigido |
| T27 | 1 | respawn desligava a mão | sem conferir estado | não age se reconectou | dados_gui_sensores: test_respawn_da_mao_nao_desliga_quem_reconectou | corrigido |
| T28 | 1 | término atrasado fechava o CSV | sem correlação | PalpationStatus.run_id | dados_gui_sensores: test_fim_atrasado_do_run_anterior_nao_fecha_o_novo, test_gui_ignora_fim_de_outro_run | corrigido |
| T29 | 1 | logger reiniciado truncava | open w | abertura exclusiva | dados_gui_sensores: test_start_latched_repetido_nao_trunca_o_run, test_sensors_csv_nao_sobrescreve | corrigido |
| T30 | 1 | sensors.csv repetia frame | lia o último | cada linha leva o próprio frame | dados_gui_sensores: test_cada_linha_leva_as_tensoes_do_proprio_frame | corrigido |
| T31 | 1 | logger reserva sem sensor | sem -p sensor | passa o sensor | dados_gui_sensores: test_logger_reserva_sobe_com_o_sensor_da_gui | corrigido |
| T32 | 1 | poses.json perdido | escrita direta | tmp + os.replace | espelho_gui_seguranca: test_queda_no_meio_do_save_preserva_o_anterior, test_poses_json_ilegivel_nao_e_apagado | corrigido |
| T33 | 1 | coluna singular sempre 0 | DLS nunca lança | sigma mínimo < lambda | dados_gui_sensores: test_singularidade_do_punho_e_marcada | corrigido |
| T34 | 1 | mu com J6 girado | um só eixo | hipotenusa lateral; K no eixo de descida | dados_gui_sensores: test_mu_com_j6_girada, test_k_com_ataque_inclinado | corrigido |
| T35 | 1 | tabela LaTeX fixa | literais | números da campanha | dados_gui_sensores: test_tabela_latex_usa_a_campanha_atual | corrigido |
| T36 | 1 | curva com sinal trocado | t + shift | t - shift; valor do atraso intocado | dados_gui_sensores: test_curva_deslocada_casa_com_o_sim | corrigido |
| T37 | 1 | tara com janela velha | buffer sem validade | idade máxima 0,5 s; queda esquece a janela | dados_gui_sensores: test_tara_recusa_janela_de_antes_da_queda, test_buraco_na_linha_esquece_a_janela | corrigido |
| T38 | 1 | ft_probe sem exclusividade | open simples | exclusive=True | dados_gui_sensores: test_probe_abre_exclusivo | corrigido |
| T39 | 1 | ft_probe só escutava | célula é polled | --mode polled ou stream | dados_gui_sensores: test_probe_pergunta_a_celula_polled | corrigido |
| T40 | 1 | 11 arquivos liam o fonte | teste textual | convertidos em comportamentais sobre o código de produção (explorer, GUI e driver reais, relógio virtual); estruturais restantes justificados abaixo | test_fmod_phase, test_fmod_ilc, test_hold_dwell_window, test_servoj_descent, test_matrix_grid_order, test_gui_split, test_audit_20260824, test_estop_reconnect, test_prehome_before_start, test_mode_feature_contract | corrigido |
| T41 | 1 | teste testava cópia | regra espelhada | qs_ramp_step extraído e usado por nó e teste | test_no_overshoot | corrigido |
| T42 | 1 | sem pandas a coleta caía | import no topo | dependência declarada e skip por marca. O primeiro conserto (importorskip de módulo) foi reprovado na prova: com launch_testing zerava a coleta da sessão | dados_gui_sensores: test_sem_pandas_a_coleta_dos_outros_continua | corrigido |
| T43 | 1 | 1 falha, 13 erros, 12 skips relatados | cra_description não instalado | nenhuma: no overlay completo a base dá 1161 passam, 6 skips, 0 falhas | suíte completa | não reproduzido (ambiente) |
| T44 | 1 | test_palp_char fora da suíte | idem T42 | incluído | suíte completa | corrigido |
| C01 | 1 | ADR 0001 x slide_force_hz | decisão não registrada | emenda; funcionalidade mantida | documento | corrigido |
| C02 | 1 | README divergente | texto antigo | atualizado | explorer_seguranca: test_exemplos_do_readme_passam_na_validacao | corrigido |
| C04 | 1 | oito funções sem uso e órfãs | restos | removidas após conferir callbacks, entry points, launch, config, dinâmico | suíte completa e tabela C07 | corrigido |
| C06 | 1 | 1115 testes lidos por AST; research_lab.world é o único world carregado | world sem defeito: XML válido, dt 4 ms, tampo kp 1e6 | publisher de test_logger_columns alinhado ao QoS novo | suíte completa | corrigido |
| C07 | 1 | 213 candidatos | quase todos callbacks | 199 mantidos, 14 removidos | suíte completa | corrigido |
| C08 | 1 | relatórios touch_*.md | - | todos os itens nesta tabela | - | corrigido |
| XT-21c | 1 | enable/soltar E-STOP sem Stop antes do Continue | fila antiga | Stop antes do Continue | driver_confirmacao: test_enable_esvazia_a_fila_antes_do_continue | corrigido (firmware não verificado) |
| XT-ServoJ | 1 | só o primeiro ACK era lido | parse parcial | todos os ACKs | driver_confirmacao: test_servoj_ve_erro_em_qualquer_ack_do_buffer | corrigido |
| XT-24a | 1 | DONE com home falha | retorno ignorado | estado correto | explorer_seguranca: test_home_final_falha_nao_vira_done | corrigido |
| XT-24b | 1 | import falho mandava q URDF como DOBOT | fallback silencioso | import levanta | driver_confirmacao: test_sem_conversao_o_driver_nao_importa | corrigido |
| XT-24c | 1 | TCP da ponteira com a mão | sem condição | cfg touch_tool | test_real_driver_proto | corrigido |
| XT-vel | 1 | velocidade sem teto | - | 30 mm/s | explorer_seguranca: test_velocidade_do_start_tem_teto | corrigido |
| XT-JS | 1 | /joint_states NaN/parcial | - | recusado | explorer_seguranca: test_joint_states_com_nan_ou_parcial_nao_vira_pose | corrigido |
| XT-align | 1 | align do run anterior | - | zerado | explorer_seguranca: test_align_do_run_anterior_nao_sobrevive | corrigido |
| XT-12 | 1 | rates com z constante derrubava tudo | min() vazio | NaN e isolamento | dados_gui_sensores: test_rates_com_z_constante_e_run_ruim_isolado | corrigido |
| XT-12b | 1 | delay() não monotônico (suspeita) | - | ordenação estável mantida | dados_gui_sensores: test_atraso_com_comandos_fora_de_ordem | não reproduzido (verde na base) |
| XT-13 | 1 | GPU perdida caía no raster exato | - | malha reduzida | dados_gui_sensores: test_gpu_perdida_vai_para_a_malha_reduzida_e_nao_para_o_raster, test_gpu_recusada_nao_recarrega_em_laco | corrigido |
| XT-14 | 1 | TOTAL do TCP impresso para texturas_v2 | peça avulsa tem base 0 própria | só ft6 | test_gen_tcp_meshes: test_peca_avulsa_nao_imprime_o_tcp (falha na base), test_pilha_completa_imprime_o_tcp | corrigido; decisão do usuário: versionar ponteira_texturas_v2.step (ausente da árvore; o teste usa sólido sintético) |
| XT-15 | 1 | código morto da fatia 2 | - | removido; savgol_filter, wrap180, twist_deg ficam (só testes usam) | tabela C07 | corrigido |
| XT-18 | 1 | zero não limpava o buffer | - | limpa | dados_gui_sensores: test_zero_no_sensor_descarta_as_amostras_antigas | corrigido |
| XT-21 | 1 | polled perdia interval_s | - | mantém | dados_gui_sensores: test_trocar_para_polled_mantem_o_passo | corrigido |
| XT-22 | 1 | taxa morta publicada | - | zerada na queda | dados_gui_sensores: test_queda_do_link_zera_a_taxa_publicada | corrigido |
| XT-23 | 1 | porta fechada como busy | RuntimeError genérico | LineBusyError | dados_gui_sensores: test_polled_porta_fechada_nao_passa_por_linha_ocupada | corrigido |
| XT-24d | 1 | gravação de IMU morria calada | - | avisa e limita a fila | dados_gui_sensores: test_gravacao_de_imu_com_disco_cheio_avisa_e_nao_acumula, test_campo_vazio_na_gravacao_de_imu_avisa | corrigido |
| XT-25 | 1 | título do relatório, thread daemon, agrupados | - | corrigidos | dados_gui_sensores: test_titulo_do_relatorio_usa_a_pasta_do_run, test_fechar_o_logger_espera_o_relatorio | corrigido |
| XT-inf | 1 | inf na célula | - | isfinite explícito | dados_gui_sensores: test_amostra_nao_finita_nao_entra_no_filtro | já corrigido (inf); NaN agora coberto |
| XT-yaml | 1 | tactile_hand_controllers.yaml não carregado | tirado de propósito: tactile_cell.launch deixou de apontar para ele no commit 70aabbf (31/08), junto com o plugin mimic que desestabilizava o modelo; o modo hand usa hand_pack/config/cr10_covvi_controllers.yaml (launch:175), e a GUI publica as 32 juntas que esse arquivo lista | nenhuma; controlador ativo intocado | inspeção: só o glob de setup.py o instala e grasp_ml_pack/doc/covvi_control.md o cita | arquivo morto confirmado; decisão do usuário: apagar ou guardar (e corrigir covvi_control.md, que ainda o descreve como ativo) |
| XT-adm | 1 | admittance_controller nunca spawnado | uso futuro documentado | mantido | - | não reproduzido (intencional) |
| XT-cam | 1 | rastreio com assistente IMU aberto | - | portão | dados_gui_sensores: test_rastreio_recusado_com_o_assistente_aberto | corrigido |
| XT-azero | 1 | auto-zero da célula | launches passam false | nenhuma | - | não reproduzido |
| XR-STOP | 1 | stop_motion mandava Continue depois de Stop recusado e engolia a falha | erro só no log | repete o Stop uma vez e levanta, sem Continue | test_driver_confirmacao: test_stop_nao_confirmado_nao_manda_continue, test_stop_confirmado_esvazia_e_libera_a_fila | corrigido (lado rehab: ver revisão cruzada) |
| XR-CRUZ-1 | 1 | ALTO: robot_mode vazio com E-STOP travado | cedia por _estop_engaged | cede só enquanto EmergencyStop(1) espera o lock; até EmergencyStop(0) mudo cede | test_revisao_cruzada: test_robot_mode_responde_com_estop_travado, test_rearme_completa_com_controlador_lento, test_estop_fura_ate_o_rearme_mudo | corrigido |
| XR-CRUZ-2 | 1 | ALTO: resposta perdida envenenava o comando | dívida por nome, sem fim | chave nome+args com validade de 2 s; E-STOP/rearme isentos; EmergencyStop(0) e (1) nunca se confirmam | test_revisao_cruzada: test_movj_perdido_nao_envenena_os_seguintes, test_movj_identico_volta_a_confirmar_quando_a_divida_vence, test_estop_e_rearme_nunca_ficam_envenenados, test_atrasada_de_verdade_nao_confirma_outro_comando, test_atrasada_descartada_e_a_verdadeira_aceita | corrigido |
| XR-CRUZ-3 | 1 | MÉDIO: RTF < 0,9 abortava HOME | prazo de parede | espera enquanto o braço anda (> 0,002 rad), teto de parede 9x | test_revisao_cruzada: test_home_com_rtf_baixo_chega, test_braco_parado_no_meio_ainda_aborta_logo, test_braco_que_rasteja_tem_teto_de_parede | corrigido |
| XR-CRUZ-4 | 1 | MÉDIO: pausa do Gazebo virava salto de ServoJ | sem idade nem teto | /joint_states > 0,3 s ou passo acima de 180 graus/s x dt (dt até 0,1 s) trava até o operador | test_revisao_cruzada: test_mirror_para_com_joint_states_velho, test_mirror_recusa_salto, test_gui_pausa_e_salto_travam_ate_o_operador, test_gui_velocidade_legitima_passa | corrigido |
| XR-CRUZ-5 | 1 | MÉDIO: follow_acc salvo sem efeito | cache só no trace | preenchido no fim da aba | test_revisao_cruzada: test_follow_acc_salvo_vale_sem_tocar_na_caixa | corrigido |
| XR-CRUZ-6 | 1 | MÉDIO: freeze durante connect/enable perdido | driver ainda não publicado | reconferido sob lock e EmergencyStop | test_revisao_cruzada: test_freeze_durante_a_conexao_nao_se_perde | corrigido |
| XR-CRUZ-7 | 1 | MÉDIO: freeze no meio do rearme visto tarde | executor de uma thread | contador de freeze, grupos próprios, MultiThreadedExecutor | test_revisao_cruzada: test_freeze_no_meio_do_rearme_vence, test_freeze_e_atendido_com_o_rearme_bloqueado | corrigido |
| XR-CRUZ-8 | 1 | BAIXO: lock 1 s no streaming; ACK de erro drenado; E-STOP conferido antes do lock; E-STOP durante rearme | - | prazo 0,2 s com ServoJ fluindo; erro levantado no próximo ServoJ; reconferência dentro do lock; geração de E-STOP | test_revisao_cruzada: test_consulta_muda_nao_segura_o_servoj, test_ack_de_erro_drenado_aparece_no_proximo_servoj, test_estop_pedido_na_espera_do_lock_barra_o_movimento, test_estop_durante_o_rearme_vence | corrigido |
| XR-CRUZ-9 | 1 | BAIXO: logger reiniciado não gravava o resto | start latched recusado | pasta <run_id>_contK, sem sobrescrever | test_revisao_cruzada: test_logger_reiniciado_grava_o_resto_em_continuacao | corrigido |
| XR-CRUZ-10 | 1 | BAIXO: Tk parado em stop_motion/emergency_stop | E/S na thread Tk | call_bounded (0,25 s); resultado tardio por after; só anuncia o confirmado | test_revisao_cruzada: test_estop_lento_nao_prende_o_tk, test_stop_so_anuncia_parada_confirmada | corrigido |
| XR-CRUZ-11 | 1 | BAIXO: timeout isolado de feedback virava E-STOP | sem tolerância | 0,5 s de tolerância antes do 1º ServoJ | test_revisao_cruzada: test_mirror_tolera_feedback_isolado_mas_nao_o_mudo, test_gui_tolera_feedback_isolado_mas_nao_o_mudo | corrigido |
| XR-CRUZ-12 | 1 | MÉDIO: E-STOP travado + queda da conexão = rearme impossível até reiniciar o mirror_node | driver morto ficava em _driver | o pedido de rearme descarta o driver morto, abre outro e solta o E-STOP com release_emergency_stop; freeze vence | test_revisao_cruzada: test_rearme_recria_o_driver_morto_e_solta_o_estop, test_freeze_vence_o_rearme_com_driver_novo | corrigido |
| XR-CRUZ-13 | 1 | BAIXO: _sync_fail_t sobrevivia ao fim da fase | só zerava no sucesso | zerado fora de fase ativa, em FROZEN/pausa e ao religar | test_revisao_cruzada: test_mirror_zera_a_tolerancia_no_fim_da_fase, test_gui_zera_a_tolerancia_no_fim_da_fase | corrigido |

Os testes citados falham na base e passam agora, salvo controles e itens marcados "não reproduzido"/"já corrigido".

## Contratos que afetam rehab_pack

- `stop_motion()` levanta quando o Stop não é confirmado.
- Falha de feedback por tempo: `CR10FeedbackTimeout`, conexão preservada. Keepalive pula com envio recente.
- `emergency_stop()` só devolve True com código 0.
- Feedback exige TestValue: `struct.pack_into('<Q', p, 48, 0x0123456789ABCDEF)`.
- ServoJ com qualquer código não zero levanta; enable levanta se SpeedFactor ou SetCollisionLevel não confirmarem.
- Removidos do driver: prepare_servoj, halt, sync, mov_j_cartesian, mov_l_cartesian, rel_movl_user, read_tcp_force, get_angle_deg, tool_do, resample_trajectory, FEEDBACK_TCP_FORCE_OFFSET. Novo em cfg: touch_tool=True.
- `PalpationStatus.run_id` novo: recompilar touch_pack_msgs.

## Testes puros para o CI (contrato d)

Rodam inteiros com Python sem rclpy, Gazebo ou hardware (medido, nenhum skip): test_controller_config, test_driver_confirmacao, test_explorer_split, test_ft_charts, test_ft_frame, test_ft_hkvl56, test_ft_modbus, test_ft_polled, test_ft_stats, test_ft_tcp, test_gui_split, test_hand_camera_teleop, test_hand_connect_ready, test_imu_math, test_imu_recorder, test_kinematics, test_manip3d, test_pack_lock, test_palp_char, test_plane_probe, test_real_driver_proto, test_slide_force_loop, test_tcp_calib, test_tool_select, test_touch_sensor_spec.

Pedem as bibliotecas Python do ambiente medido (numpy, scipy, pandas, pyserial, matplotlib, Pillow, tkinter). test_gen_tcp_meshes só roda com OCP. test_vtk_render faz core dump sem display: não pôr no CI. Os outros três arquivos novos exigem rclpy.

## Fechamento

Conferido por script: XC1 a XC8, T01 a T44, C01, C02, C04, C06, C07, C08, todos os XT e XR-CRUZ-1 a 13 têm linha única, estado final e teste. Sem teste próprio, por natureza: C01 (documento), C04/C06/C07/XT-15 (remoção ou leitura: suíte completa), C08 (índice), T43/T44 (suíte completa), XT-yaml, XT-adm e XT-azero (inspeção, nada mudou).

## Não verificado

Gazebo, bancada, GUI ao vivo, câmera, mão; Stop/Continue e códigos de ServoJ no firmware; ft_probe polled na célula real.
