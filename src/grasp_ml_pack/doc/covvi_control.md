# Controle da mão COVVI — melhorias

Estado: `grasp_executor` → `hand_ik` → `perfect_grasp.close_until_contact`
(rampa incremental + parada por *lag* articular) → `FollowJointTrajectory` no
`hand_position_controller` (JTC, `open_loop_control: true`). Real:
`SetCurrentGrip` (grip de fábrica) ou `SetDigitPosn` (posição por dedo).

## Pegar objetos no Gazebo — preensão POR FÍSICA (tactile cell)

Na célula tátil (`end_effector:=hand`) a mão agora segura o objeto por
**atrito de contato real** — sem attach. O que faltava e foi resolvido em
`tactile_cell.launch.py` + `hand_pack/urdf_helpers.py` +
`touch_pack/config/tactile_hand_controllers.yaml`:

| # | Problema | Correção aplicada |
|---|---|---|
| 1 | `<mimic>` (tag nativa do gazebo_ros2_control) impõe a junta por SetPosition — cinemático, torque zero. As falanges distais são arrastadas mas não empurram. | `strip_mimic_joints_from_ros2_control` tira as 26 juntas mimic do `<ros2_control>`; `inject_mimic_joint_plugins` re-injeta o `libgazebo_mimic_joint_plugin.so` (roboticsgroup) — **`<hasPID>` + `<maxEffort>` nas 10 juntas do caminho de contato** (`*_proximal_j01`, `*_distal_j01`), que aí seguem a mestre com FORÇA. As outras 16 seguem cinemáticas (baratas). |
| 2 | `hand_position_controller` (JTC, `open_loop_control: true`) comandava as 31 juntas para ângulos fixos, sem ver o contato. | `tactile_hand_controllers.yaml`: JTC comanda **só os 6 drivers**, `open_loop_control: false` (novo alvo parte do estado medido). Driver `effort` cai de 8→1 N·m (8 esmagava). `palpation_gui._publish_sim_hand` também manda só os 6 drivers. |
| 3 | Contato mole: falange `kp=5e4`, objeto `kp=1e5`, `world iters=80`. Objeto rígido sob comando de posição afunda e escapa. | Falange de contato `kp=1e6 kd=100 minDepth=1e-4` (par das amostras complacentes); `pick_object` `kp=1e6 kd=100 mu=1.4 min_depth=1e-4 massa=0.06`; `research_lab.world` `iters 80→150`. |

Sequência de preensão (manual, pela GUI — aba "Mão"):
1. **preshape** — slider/preset "Apontar" ou um grip aberto (ex. `Cylinder` parcial).
2. **fechar PASSANDO do objeto** — preset "Fechar" ou grip `Power`/`Cylinder`.
   Os drivers travam no objeto no limite de `effort`; o PID das mimic de
   contato mantém a força. É o mesmo truque do Robotiq no `ws_fruit_sorting`.
3. **dwell ~0.5 s** — deixa o cone de atrito assentar.
4. **subir o braço devagar** (≤ 5 cm/s) pela aba "3D Manipulation" / MoveIt.

### Checklist de tuning no Linux (o Gazebo tem de estar rodando — nada disto foi testado no Windows)

Ordem sugerida, uma variável por vez, observando `pick_object` no lift:

1. **`libgazebo_mimic_joint_plugin.so`** vem do pacote **`gazebo_mimic_plugin`**
   deste workspace (`src/gazebo_mimic_plugin/`) — reescrita ROS-free do
   roboticsgroup (o upstream é catkin/ROS 1 e não builda no Humble). O
   `colcon build` o compila e o hook do `gazebo_ros` põe `<prefix>/lib` no
   `GAZEBO_PLUGIN_PATH`. Confirmar depois do build:
   `ros2 pkg prefix gazebo_mimic_plugin` e
   `find $(ros2 pkg prefix gazebo_mimic_plugin) -name 'libgazebo_mimic_joint_plugin.so'`.
   Se sumir do path em runtime, Gazebo loga o erro e segue — a mão fecha
   mas não segura.
2. **`maxEffort` das mimic de contato** (`inject_mimic_joint_plugins`,
   default 1.0 N·m) e **`effort` dos drivers** (`_stabilize_hand_joints`,
   default 1.0): sobem juntos se a mão não aperta o bastante, descem se o
   objeto salta/penetra. Alvo ≈ força de ponta de dedo 5–15 N.
3. **Ganhos do PID das mimic** (`p=40, i=0, d=0.2`): se as falanges
   oscilam no contato, subir `d` ou baixar `p`; se seguem mole/atrasado,
   subir `p`.
4. **`mu` do `pick_object` vs. `mu1` das falanges** (`_build_hand_suffix`,
   grip = 2.5): o do objeto tem de ser ≥ o do dedo. Subir o do objeto
   antes de mexer em qualquer outra coisa se ele **desliza** da mão.
5. **`kp/kd` de contato**: se o objeto **vibra/pula** parado na mão,
   baixar `kp` para 3e5 ou subir `world iters` para 200; se **penetra**
   visível, subir `kp` e baixar `min_depth`.
6. **`max_step_size` do `research_lab.world`** (0.004): se 1–5 não
   estabilizam, baixar para 0.002 e `real_time_update_rate` para 500
   (custa RTF). É o último recurso — o comentário no world já registra.
7. **Alternativa se o plugin roboticsgroup for inviável**: trocar o
   `<mimic>` fixo de `*_proximal_j01`/`*_distal_j01` por junta revolute
   passiva com `<dynamics><spring_stiffness>`/`<spring_reference>` e
   comandar só os drivers — complacência subatuada real (o distal
   back-drive no contato), sem pacote extra. Perde a razão de acoplamento
   fixa; ganha física mais fiel.

### Fallback — attach cinemático (mantido, DESLIGADO por default)

O nó **`kinematic_attacher`** (`touch_pack`, console_script) continua no
pacote para demos de pick-and-place em que a força de preensão **não** é o
objeto de estudo — cola `pick_object` a `::hand_base_link` via
`/gazebo/set_entity_state`. `tactile_cell.launch.py` **não** o sobe mais.
Para usar:
```
ros2 run touch_pack kinematic_attacher
ros2 service call /kinematic_attach/attach std_srvs/srv/Trigger   # feche a mão antes
ros2 service call /kinematic_attach/detach std_srvs/srv/Trigger   # solta
```
O `grasp_executor` (conveyor cell) ainda usa a cópia inline do attach —
migrar essa célula para a preensão-por-física acima é o follow-up.

## Aplicado nesta rodada (aditivo, sem tocar no caminho de comando)

| item | o quê | arquivos |
|---|---|---|
| P0 #3, #4 | Nó **`grasp_sense`** (só leitura da COVVI): `DigitTouchAllMsg` + `DigitStatusAllMsg` → `/grasp/holding`, `/grasp/slip`, `/grasp/contact_count`, `/grasp/fault`. O `grasp_executor` consome: **gate de fault antes de fechar**, **confirmação de posse após o lift** (holding=False → ciclo = falha), **slip no trânsito** → falha. Sem o nó no ar, comportamento anterior (só geométrico). | novo `grasp_sense.py`, `grasp_executor.py`, `setup.py` |
| P1 #5 | `hand_ik(grasp_type, obj_diameter, primitive=None)` — com `primitive` (`cylinder`/`box`/`sphere`, vindo do `object_detector`) o fechamento vira **diferencial por dedo** (envolver × opor). `primitive=None` = comportamento anterior, sem regressão. | `kinematics.py` |

`grasp_sense` params: `eci_prefix` (`/covvi/hand`), `touch_on` (12/255),
`min_digits` (2), `slip_window_s` (0.4). Suba-o no launch ao lado do
`grasp_executor` quando o ECI estiver ativo.

## A fazer com a mão real na frente (precisa de calibração/hardware)

### P0 #1 — fechar pelo tátil, não pelo lag
`close_until_contact` para o dedo quando `commanded − actual > LAG_THRESHOLD`.
Na mão real o sinal direto é `DigitStatusAllMsg.{dedo}_touch` / `_stall` e
`DigitTouchAllMsg.{dedo}_touch` (0–255). Trocar o critério de parada por dedo
para `status.{d}_touch or touch_val[d] >= touch_on`, com o *lag* só de reserva.
Barato — o `grasp_sense` já assina os dois streams; falta ligar o critério
dentro de `PerfectGrasp` (passar um `contact_fn(digit)` injetado, como o
`send_hand_fn` já é).

### P0 #2 — setpoint de força de preensão por dedo
Depois do contato, um P pequeno por dedo mantendo `DigitTouch[d]` num alvo
leve (ex. 30–60/255), com `MotorCurrentAllMsg` como teto duro (abort/relaxa se
a corrente de qualquer dedo passar de X). **Precisa da relação tátil→força**
(0–255 não tem mapa em Newton documentado): meça pressionando cada dedo contra
uma balança/célula em algumas posições e ajuste uma reta. Ganho conservador,
saída de `SetDigitPosn` limitada a poucos passos por ciclo.

### P1 #6 — usar os 14 grips de fábrica
`SetCurrentGrip(grip_id)` roda o fechamento proporcional do firmware (com
tratamento de corrente/contato próprio) — melhor para a mão real que
`SetDigitPosn` cru. Tabela `objeto/tipo → CurrentGripID` (os IDs 1–14 já estão
listados em `palpation_gui.py`), com `SetDigitPosn` só no ajuste fino. Para o
sim, mapear cada `CurrentGripID` a um preshape em `HAND_CONFIGS`. Testar cada
grip uma vez na mão real e anotar o preshape resultante de `DigitPosnAllMsg`.

### P2 #8 — caminho de servo a 50–100 Hz
`SetDigitPosn` está com debounce de 60 ms (~16 Hz) — serve para preshape, não
para o servo de força do P0 #2. Usar o stream `SetRealtimeCfg` (que a GUI já
liga) para os dígitos que estão regulando.

### P2 #9 — fidelidade dos mimics do twin
Os 25 mimics seguem os 6 drivers por razão constante. Os dedos reais são
4-barras: razão distal/proximal não-linear, back-drive no contato. Varrer cada
driver 0→máx na mão real, gravar `DigitPosnAllMsg`, ajustar as razões (ou um
polinômio de grau 2) por dedo, e reportar o RMSE de ângulo sim×real.

### P2 #10 — `hand_position_controller`
`open_loop_control: true` + `goal_time: 0.0` no `cr10_covvi_controllers.yaml`.
Ok no sim; se o mirror da mão real algum dia passar por esse controlador ele
não rastreia (sem correção por feedback). Verificar o caminho do mirror; se
for por ele, `open_loop_control: false` + afinar os ganhos/effort.
