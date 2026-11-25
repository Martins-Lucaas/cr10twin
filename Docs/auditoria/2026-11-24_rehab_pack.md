# Auditoria 2026-11-24 — rehab_pack (agente 2)

Base: HEAD 893e34d. WSL Ubuntu-22.04, ROS 2 Humble, cópia em `/tmp/wsB`; base em `/tmp/wsB_base`.
Sem Gazebo, CR10 ou FA7155: nada foi conectado a hardware.

Baseline: 294 passaram, 1 falhou (R28), 0 skips. Os 2 skips históricos eram só pandas/trimesh
ausentes; instalados, rodam e passam.

Vermelho-antes: os testes novos rodados na base dão 48 falhas (C), 27 (D) e 9 (G).
V/V = falhou na base e passa agora. V/V(api) = na base falhou por falta da interface nova, não
pelo comportamento. Guarda = passa nos dois lados (protege, não prova).
C = `test_auditoria_controle.py`, D = `test_auditoria_dados.py`, G = `test_gui_smoke.py`, em
`src/rehab_pack/test/`. Agente 2 em tudo, salvo R11 e XR14.

## Itens

| ID | Evidência / causa | Correção | Teste | Estado |
|---|---|---|---|---|
| XC9 | salto de 7° do sim ia num só ServoJ; `/joint_states` velho seguia em ATIVO | `RealLink.stream` recusa passo acima do limite e NaN; o espelho trava até Reset e rearma a menos de 3° com amostras frescas; a admitância aborta com `travado` ou js vencido | C `test_xc9_*`: degrau, rampa e js vencido V/V; primeira amostra V/V(api); retomada guarda | corrigido |
| R01 | combined_gui, manual_control_node, grasp_executor e 2 launches sem a trava | tomam a do touch_pack; publicador de fora em T_ARM_CMD recusa enable/jog e aborta em ATIVO | C `test_r01_*` V/V | corrigido |
| R02 | J6 no limite girava o mastro 15° | giro acima de 2° vira LIMITE | C `test_r02` V/V | corrigido |
| R03 | valor meio digitado em J1 ia ao mexer J2 | só a junta mexida muda; as outras seguem o último alvo aceito | G `test_r03` V/V | corrigido |
| R04 | habilitava no meio da tara | T_TARED = 0 no início; cancela ao sair de PARADO; recusa tara dupla | C `test_r04_*` V/V | corrigido |
| R05 | tara válida após reinício/re-zero da célula | silêncio ou `tare_result` ok invalida; em ATIVO aborta | C `test_r05_*` V/V | corrigido |
| R06 | tara guardava a orientação do sim | usa a do robô real; mudar `conectado` invalida | C `test_r06` V/V | corrigido |
| R07 | `ft_sign` 0/0,5/NaN aceitos | só ±1; yaw finito (nó e launch) | C `test_r07_*` V/V (yaw inf no nó: guarda) | corrigido |
| R08 | `servoj_period_s:=0` aceito | faixa 0,02–0,1 s, só leitura; `traj_lead_s` >= `dt_s` | C `test_r08_*` V/V | corrigido |
| R09 | NaN/incompleto/vencido em `/joint_states` | filtrado nos 2 nós; idade conferida em enable/jog | C `test_r09_*` V/V | corrigido |
| R10 | referência fora movia o robô sem força | referência recusada; clamp só com K > 0, nunca puxa para dentro | C `test_r10_*` V/V (1 de 2 casos do núcleo é guarda) | corrigido |
| R11 | `real_driver.py`, contrato (a) | ver registro do touch_pack | — | agente 1 |
| R12 | IK sem convergir 26,3 ms (mediana, n=200, WSL, HOME) | IK_ITERS = 5: 7,1 ms; alvo próximo 1,93 → 2,07 ms; não envia, vira LIMITE | C `test_r12` V/V | corrigido |
| R13 | um timeout de 50 ms derrubava a conexão | até 5 seguidos: q_real = None (para seguro), conexão mantida; depois, perda | C `test_r13_*` V/V | corrigido |
| R14 | PARAMS apagava o inicial do meta | meta guarda o inicial; mudanças em events | D `test_r14` V/V | corrigido |
| R15 | pico pela média do tick (14 N vira 2,08) | pico do `wrench.csv` | D `test_r15` V/V | corrigido |
| R16 | logger morto e painel "gravando" | heartbeat `/rehab/session/recording`; "GRAVAÇÃO PERDIDA" | G `test_r16_*` V/V; D `test_r16` V/V(api) | corrigido |
| R17 | aborto perdia motivo e TASK_END | ABORT;motivo; espera de 0,5 s antes de fechar | C e D `test_r17_*` V/V | corrigido |
| R18 | disco cheio deixava estado inválido | limpa ao abrir; fecha como `erro_disco`; JSON atômico | D `test_r18_*` V/V (no meio: V/V pelo OSError solto) | corrigido |
| R19 | proveniência velha ou congelada | idade conferida; evento ROBO e `robo_mudou` | D `test_r19_*` V/V | corrigido |
| R20 | marcador sem sessão dizia "registrado" | avisa que não gravou | G `test_r20` V/V | corrigido |
| R21 | fechar o painel não encerrava a tarefa; alvo herdado | `task_stop` ao fechar; alvo limpo no início | G e D `test_r21` V/V | corrigido |
| R22 | % do túnel com tarefa anterior à sessão | conta da tarefa | D `test_r22` V/V | corrigido |
| R23 | túnel concluía por atalho | progresso só dentro e no ritmo do cursor | D `test_r23_*`: atalhos V/V; percursos legítimos guarda | corrigido |
| R24 | ruído contava 48 saídas | histerese de 2 mm | D `test_r24` V/V | corrigido |
| R25 | tarefa com admitância morta | não inicia e encerra | G `test_r25` V/V | corrigido |
| R26 | parâmetros da tarefa não gravados | em TASK_START; troca de assistência vira PARAMS | D e G `test_r26` V/V | corrigido |
| R27 | CI sem rehab | job roda o rehab inteiro, a trava e testes puros do touch; instala xvfb, xauth e trimesh | comandos rodados localmente | corrigido; `xvfb-run` e o job no GitHub não rodaram |
| R28 | teste vermelho na base | contrato atual: dq vem da posição; o assert exigia o campo velocity e foi atualizado | o próprio | corrigido (teste desatualizado) |
| R29 | `package.xml` sem launch, launch_ros, builtin_interfaces, gazebo_msgs | acrescentados; comentário corrigido | `rosdep resolve` dos 4; colcon build | corrigido |
| C03 | README divergente | t_s, tara, sim/real, M, Fz, travas novas | — | corrigido |
| C04 | `score_text` x3 | removidos | grep | corrigido |
| C05 | `.mcp.json` com caminho pessoal | `code-review-graph` pelo PATH | `serve` + initialize respondeu no Windows | corrigido; Linux não verificado (executável ausente no WSL) |
| C06 | `test_world.py`, `rehab_lab.world` | `test_world.py` lido até `_folga_minima`, o resto executado; asserts conferem valor e distância | passa, 0 skips | nada a corrigir |
| C07 | 34 candidatos | tabela abaixo | — | corrigido |
| C08 | relatórios `rehab_controle` e `rehab_gui` | XR abaixo | — | ver XR |

## XR (fora de R01–R29)

| ID | Item | Estado |
|---|---|---|
| XR14 | controle 14, feedback desalinhado | agente 1 |
| XR15 | LIMITE com o real atrasado virava ABORTADO | corrigido, C `test_xr15` V/V |
| XR16 | InvalidHandle ao encerrar o espelho | corrigido, C `test_xr16` V/V |
| XR16b | gui 16: git dentro do serviço de início | corrigido, D `test_xr16` V/V; duração do git não medida |
| XR17 | docs e docstrings (controle e gui 17) | corrigido |
| XR18 | sessão livre nascia `centro_fora` | corrigido, G `test_xr18` V/V |
| XR18b | `check_sim_ft.py --sinal` prometia o sinal | corrigido: o script calcula o `sim_sign` pela FK, `test_bancada_e_ponte.py` `test_sinal_*`; não rodado contra o Gazebo |
| XR19 | NaN no resumo; ID `___` | corrigido, D `test_xr19_*` V/V |
| XR19b | `connect_once`, `read_q` | mantidos: testes existentes os usam; parâmetros e import mortos removidos |
| XR20 | código morto (gui 20) | removido; o serviço `S_HOME` do nó fica (README) |
| XR21 | JSON não objeto em T_REAL_STATUS derrubava o nó | corrigido, C `test_status_*` V/V |
| XR22 | fake_bench e GuiBridge (`connect_real`, `start_session`, `set_params`) sem teste | corrigido: `test_bancada_e_ponte.py` (14 testes; 8 falham no HEAD, 6 já passavam: cobertura) |
| XR25 | um `skip` de módulo sob launch_testing zerava a coleta (sem DISPLAY: "1 skipped" para a suíte toda) | corrigido: hook em `test/conftest.py`, `test_skip_de_modulo_*`; sem DISPLAY agora 360 passam e 1 pula |
| XR26 | contratos novos do driver (`CR10FeedbackTimeout`, TestValue no byte 48, ServoJ com código não zero, enable sem confirmação) | compatível: TestValue posto no fake_bench; provado pelos `test_bancada_*` contra o driver real |
| XR23 | eixos/sinal da FA7155, `LOOP_DELAY_S`, resposta do CR10 | pendente: exige bancada |
| XR24 | o git rastreia `Docs/` e `docs/` (no Linux são duas pastas) | pendente: decisão do usuário |

## Revisão cruzada do agente 1 (25/11/2026)

Prova de vermelho por mutação em cópia: 26 reversões, 26 pegas (R1–R5: 13; R6–R7: 13) por `test/test_revisao_cruzada.py` (35 testes).

| # | Achado | Correção | Estado |
|---|---|---|---|
| XR-CRUZ-R1 | ALTA: `halt()` engolia o Stop não confirmado | `halt()` devolve sucesso e escala para `emergency_stop()`; aviso `parada` no status, primeiro em `real_ok`, no cartão e no banner; Reset não apaga; o Conectar do operador solta o E-STOP que o espelho acionou | corrigido, `test_r1_*` e `test_parada_nao_confirmada_*` (GUI) |
| XR-CRUZ-R2 | MÉDIA: passo máximo fixo 1,44° | 1,44° + 24°/s x atraso do laço, dt até JS_STALE_S (folga até 3,12° a 30 ms; ver R6); no 1º ponto não há atraso | corrigido, `test_r2_*`; `test_xc9_*` seguem |
| XR-CRUZ-R3 | MÉDIA: PARADO limpava `halted` | só o evento `RESET`, publicado pelo serviço de Reset da admitância; o assert XC9 passou a usar esse evento | corrigido, `test_r3_*` |
| XR-CRUZ-R4 | MÉDIA: zero de hardware não invalidava a tara | assina `/load_cell/tared`: False, e a volta a True (auto-tare), invalidam | corrigido, `test_r4_*` |
| XR-CRUZ-R5 | BAIXA: `finish()` da tara cancelada fechava a seguinte | a janela é o token | corrigido, `test_r5_*` |
| XR-CRUZ-R6 | ponto de 3,12° em 30 ms (~104°/s) aceito em R2 | nenhum ponto passa de 1,44°: o atraso tolerado (até 0,1 s) vira folga, recuperada por recorte proporcional e que some 0,72° por ponto; além dela trava. O laço deixa ao menos um período entre pontos | corrigido, `test_r6_*` e `test_r2_*` |
| XR-CRUZ-R7 | `halt()` segurava o lock do espelho | parada em thread, com `halted`/`parando` já postos sob o lock; ServoJ, Conectar e admitância recusados enquanto dura; Desconectar e encerrar esperam por ela | corrigido, `test_r7_*` |

## C07 — candidatos a código morto

Uso gerado por script (`git grep`) e revisto.

| Símbolo | Definição (HEAD) | Uso | Decisão |
|---|---|---|---|
| generate_launch_description | launch/joystick_cell.launch.py:144 | `test_auditoria_controle.py:572` `mod.generate_launch_description()` | vivo: entrada do launch |
| _on_params | rehab_pack/admittance_node.py:187 | `admittance_node.py:160` `self.add_on_set_parameters_callback(self._on_params)` | vivo: chamada ou argumento |
| _on_real | rehab_pack/admittance_node.py:259 | `admittance_node.py:153` `self.create_subscription(String, T_REAL_STATUS, self._on_real, 5)` | vivo: callback ROS |
| _fk_z | rehab_pack/admittance_node.py:314 | `admittance_node.py:394` `ok, motivo, dur = plan_jog(self.q, goal, self._fk_z, speed_pct=pct,` | vivo: chamada ou argumento |
| _on_ref | rehab_pack/admittance_node.py:365 | `admittance_node.py:151` `self.create_subscription(PointStamped, T_REFERENCE, self._on_ref, 10)` | vivo: callback ROS |
| _srv_estop | rehab_pack/admittance_node.py:433 | `admittance_node.py:157` `self.create_service(Trigger, S_ESTOP, self._srv_estop)` | vivo: callback ROS |
| _srv_reset | rehab_pack/admittance_node.py:438 | `admittance_node.py:158` `self.create_service(Trigger, S_RESET, self._srv_reset)` | vivo: callback ROS |
| save_home | rehab_pack/constants.py:152 | `rehab_gui.py:900` `C.save_home([math.radians(g) for g in deg])` | vivo: chamada ou argumento |
| _on_wrench | rehab_pack/force_source.py:125 | `force_source.py:118` `self.create_subscription(WrenchStamped, topic, self._on_wrench,` | vivo: callback ROS |
| _on_tare | rehab_pack/force_source.py:139 | `force_source.py:143` `self.create_service(Trigger, S_TARE, self._on_tare,` | vivo: callback ROS |
| _on_tick | rehab_pack/gui_bridge.py:109 | `gui_bridge.py:52` `n.create_subscription(Float64MultiArray, C.T_TICK, self._on_tick, 10)` | vivo: callback ROS |
| _on_params | rehab_pack/gui_bridge.py:113 | `gui_bridge.py:53` `n.create_subscription(String, C.T_PARAMS, self._on_params, _QOS_LATCHED)` | vivo: callback ROS |
| default_factory | rehab_pack/real_link.py:110 | `real_link.py:144` `def __init__(self, ip: str, period_s: float = 0.030, factory=default_factory,` | vivo: chamada ou argumento |
| _on_state | rehab_pack/real_mirror.py:108 | `real_mirror.py:97` `self.create_subscription(String, T_STATE, self._on_state, latched)` | vivo: callback ROS |
| _on_tick | rehab_pack/real_mirror.py:114 | `real_mirror.py:92` `self.create_subscription(Float64MultiArray, T_TICK, self._on_tick, 10)` | vivo: callback ROS |
| _srv_connect | rehab_pack/real_mirror.py:125 | `real_mirror.py:94` `self.create_service(SetBool, S_REAL_CONNECT, self._srv_connect)` | vivo: callback ROS |
| _srv_mode | rehab_pack/real_mirror.py:143 | `real_mirror.py:95` `self.create_service(SetBool, S_REAL_MODE, self._srv_mode)` | vivo: callback ROS |
| _connect_loop | rehab_pack/real_mirror.py:170 | `real_mirror.py:100` `for target, name in ((self._connect_loop, 'conectar'),` | vivo: thread |
| _servoj_loop | rehab_pack/real_mirror.py:204 | `real_mirror.py:101` `(self._servoj_loop, 'servoj'),` | vivo: thread |
| _status_loop | rehab_pack/real_mirror.py:245 | `real_mirror.py:102` `(self._status_loop, 'status'))]` | vivo: thread |
| _cv_resize | rehab_pack/rehab_gui.py:611 | `rehab_gui.py:548` `self.cv.bind('<Configure>', self._cv_resize)` | vivo: Tk |
| _space_on_widget | rehab_pack/rehab_gui.py:1051 | `rehab_gui.py:140` `root.bind_class(cls, '<KeyPress-space>', self._space_on_widget)` | vivo: Tk |
| watch | rehab_pack/rehab_gui.py:1546 | `rehab_gui.py:1609` `root.after(200, watch)` | vivo: Tk |
| _meta | rehab_pack/session_logger.py:252 | `session_logger.py:274` `self.create_subscription(String, T_SESSION_META, self._meta, latch)` | vivo: callback ROS |
| _params | rehab_pack/session_logger.py:258 | `session_logger.py:275` `self.create_subscription(String, T_PARAMS, self._params, latch)` | vivo: callback ROS |
| _alvo | rehab_pack/session_logger.py:273 | `session_logger.py:279` `self.create_subscription(PointStamped, T_TARGET, self._alvo, 10)` | vivo: callback ROS |
| _evento | rehab_pack/session_logger.py:307 | `session_logger.py:273` `self.create_subscription(String, T_EVENT, self._evento, 50)` | vivo: callback ROS |
| _emg | rehab_pack/session_logger.py:313 | `session_logger.py:281` `self.create_subscription(Float32MultiArray, T_EMG, self._emg, 50)` | vivo: callback ROS |
| _srv_stop | rehab_pack/session_logger.py:354 | `session_logger.py:286` `self.create_service(Trigger, S_SESSION_STOP, self._srv_stop)` | vivo: callback ROS |
| score_text | rehab_pack/tasks.py:222 | NENHUMA | removido (sem uso) |
| score_text | rehab_pack/tasks.py:332 | NENHUMA | removido (sem uso) |
| score_text | rehab_pack/tasks.py:368 | NENHUMA | removido (sem uso) |
| _accept | scripts/fake_bench.py:76 | `fake_bench.py:73` `threading.Thread(target=self._accept, args=(s, fn), daemon=True).start()` | vivo: thread |
| _dash | scripts/fake_bench.py:102 | `fake_bench.py:68` `for port, fn in ((29999, self._dash), (30004, self._feed)):` | vivo: thread |

## Testes rodados

- `python3 -m pytest -q -rs src/rehab_pack/test`, overlay recompilado e DISPLAY do WSLg (no CI entra `xvfb-run -a`): 401 passaram, 0 skips. Após XR-CRUZ-R: 437 passaram, 0 skips; sem DISPLAY 395 e 1 skip (`test_gui_smoke`).
- Lista do passo "smoke tests" do `ci.yml` (25 arquivos puros do registro touch): 407 passaram; passo rehab_pack: 437 (após XR-CRUZ-R).
- Contrato (d): conciliado com a lista do registro touch; CI instala scipy, pandas e pillow. O Actions em si não rodou.
- `arm_intruders` por tick: mediana 0,02 ms, máx. 0,17 ms (n=200, WSL).
- `touch_pack/test` completo: 35 falhas com o touch_pack no meio da edição e msgs antigas no
  overlay; não analisadas aqui (agente 1).
