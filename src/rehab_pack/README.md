# rehab_pack

Reabilitação de membro superior com o CR10. O voluntário segura um **mastro**
(`cad/Joystick.step`) preso à célula FA7155 e guia o robô por **admitância**
cartesiana. Independente de `touch_pack`, `hand_pack` e `grasp_ml_pack`.

Padrão: **simulação (Gazebo Classic)**, com a força vinda de uma "mão virtual"
na GUI ou do plugin F/T do Gazebo. O CR10 e a FA7155 reais se ligam do mesmo
jeito que no `touch_pack` (ver "Robô e célula reais"). Sem EMG real, sem RV.

## Como rodar

```bash
cd ~/cr10twin
colcon build --packages-select rehab_pack --symlink-install   # 1ª vez / após mudar setup.py
source /opt/ros/humble/setup.bash && source install/setup.bash

ros2 launch rehab_pack joystick_cell.launch.py               # Gazebo + controle + logger + painel
ros2 launch rehab_pack joystick_cell.launch.py force_source:=sim sim_ft:=true
ros2 launch rehab_pack joystick_cell.launch.py gui:=false     # sem janela do Gazebo
ros2 launch rehab_pack joystick_cell.launch.py painel:=false  # sem o painel (testes sem DISPLAY)
```

Argumentos: `force_source:=virtual|sim|real` (padrão `virtual`; `sim` exige
`sim_ft:=true`), `admittance`, `logger`, `painel`, `gui`, `sim_ft`.

Outro Gazebo aberto ocupa a porta 11345 e o `gzserver` novo morre com código
255. Feche o outro, ou isole este com `GAZEBO_MASTER_URI=http://localhost:11346`
e outro `ROS_DOMAIN_ID`.

Fluxo de uma sessão: **Tarar** (mastro solto) → **Habilitar** → publicar meta →
**Iniciar sessão** → tarefa → **Encerrar sessão**. **PARAR** (tecla Espaço) em
qualquer momento trava o braço em `ABORTADO`.

## Pilha mecânica (eixo +Z do flange, mm)

| mm | peça | material |
|---|---|---|
| 0 … 6 | flange fixo FA7155 | alumínio |
| 0,4 … 36,9 | célula FA7155 (300 g de catálogo) | — |
| 37,2 … 39,2 | placa 110 × 50 × 2 | impressa, 950 kg/m³ |
| 39,2 … 189,2 | empunhadura ⌀35 × 150, oca | **silicone/borracha**, 1100 kg/m³ |

`tcp_link` = centro da empunhadura (Z = 114,2 mm do flange), a 77 mm da face da
célula: a força medida na célula precisa ser levada até ali (M = r × F).
Pose inicial (`HOME_Q_RAD`, em `constants.py`): **J5 = +90°**, empunhadura para
cima, TCP em ≈ (0,55; 0; 0,25) m no frame `world`.

## Controle de admitância

`M·a + B·v + K·(x − x_ref) = F`, 3 eixos, tick de 30 ms. Núcleo em
`admittance.py`, cinemática em `kinematics.py` (cadeia `world → tcp_link` lida do
URDF gerado), comando do braço por `/cr10_group_controller/joint_trajectory`
(um ponto por tick). A orientação do mastro é mantida fixa: a tara vale a
sessão toda.

Estados: `PARADO`, `ATIVO`, `SEM_FORCA`, `LIMITE`, `ABORTADO`. Segurança:
|F| > limite (≤ 15 N) → `ABORTADO` travado; força com mais de 0,5 s → v = 0;
z do TCP ≥ 0,10 m; resíduo da IK > 5 mm → `LIMITE`. Parâmetros por
`ros2 param set /rehab_admittance ...` (rejeita `abort_force_n` > 15 e
combinações instáveis).

Serviços: `/rehab/ft/tare`, `/rehab/admittance/enable`, `/rehab/estop`,
`/rehab/admittance/reset`, `/rehab/home`. Nomes de todos os tópicos, serviços e
colunas: `rehab_pack/constants.py` (o contrato).

## Robô e célula reais (mesmo sistema do touch_pack)

**Estado: implementado e testado em simulação, NUNCA testado com o CR10 nem a
FA7155 de verdade.** Faça o primeiro ensaio sem pessoa, com a mão no E-Stop físico.

```bash
# célula FA7155 (nó ft_receiver do touch_pack) + CR10 espelhado, como no touch_pack
ros2 launch rehab_pack joystick_cell.launch.py control_mode:=mirror force_source:=real \
    robot_ip:=192.168.5.2 ft_port:=/dev/ttyUSB0
```

- **Célula:** `force_source:=real` sobe o `ft_receiver` do `touch_pack` com os
  mesmos argumentos (`ft_port`, `ft_mode` = polled, `ft_baud` = 1 Mbps,
  `ft_autozero`, `ft_filter`). Ele é o único dono da porta serial e publica
  `/ft_sensor/wrench`; o `rehab_force_source` o tara e o leva ao frame do mundo.
- **Robô:** `control_mode:=mirror` sobe o `rehab_mirror`, que usa o
  `CR10RealDriver` do `touch_pack` (TCP 29999, feedback 30004). O IP vem de
  `robot_ip`, senão de `~/.config/touch_pack/robot.json` (o mesmo da GUI do
  touch_pack), senão 192.168.5.2. O braço simulado manda ServoJ a 33 Hz ao real.
  O `real_pose_sync` leva o simulado até a pose do real antes de armar.
- **Sem hardware:** `robot_ip:=sim` usa um robô real simulado (segue o ServoJ),
  para ensaiar o fluxo todo.

**Intertravamentos (modo mirror):** a admitância só habilita com o robô conectado,
sem E-Stop e com sim e real a menos de 3°; durante o movimento tolera 8° de
atraso. Perdeu qualquer condição em `ATIVO` → `ABORTADO`, e o `rehab_mirror`
manda `Stop()` + `Continue()` ao robô. O espelho só corre com a admitância `ATIVA`.
`/rehab/home` é recusado nesse modo: posicione o robô real pelo painel de juntas
(aba 5, preset HOME). O painel mostra o estado do robô real e da célula no cartão Conexões.
O padrão é `control_mode:=sim_only`: nada move o robô real sem pedir.

**Pré-voo (somente leitura):** `ros2 run rehab_pack rehab_preflight --robot-ip 192.168.5.2 --ft-port /dev/ttyUSB0`
confere, sem enviar nada ao robô: se o `install` é o do `src` (não uma cópia
antiga), o driver e os executáveis do `touch_pack`, se o PC está na sub-rede do robô,
se as portas 29999/30004 respondem, o grupo `dialout`, o `pyserial` e o conversor
USB-RS485 da FA7155. Termina com código 1 se houver FALHA.

**Antes do primeiro ensaio com pessoa (não verificado):**
1. Confirme o sentido dos eixos: com o mastro solto e tarado, empurre em +X do
   mundo e veja `/rehab/ft/wrench` e o TCP irem para +X; ajuste `real_sign`.
   O `rehab_force_source` assume que os eixos da FA7155 coincidem com os do
   `load_cell_link` do URDF (conector em +X).
2. Meça a latência do laço (força → sim → ServoJ → real): ela inclui o JTC e o
   ServoJ, maior que a do controle direto. Comece com B alto (perfil Resistivo).
3. Confirme que o E-Stop físico corta o robô com o mirror armado.

## Painel (`rehab_gui`)

Sobe junto com o launch (desligue com `painel:=false`), ou `ros2 run rehab_pack rehab_gui`. Abre mesmo sem os
outros nós (LEDs "fora do ar"). Mesma família visual do `touch_pack`
(ttk `clam`), com tema em `gui_theme.py` e gráficos em `gui_charts.py`.
Cabeçalho: LEDs por nó, estado do braço, |F| ao vivo e **PARAR**. A faixa abaixo
diz o que fazer em cada estado. Abas: 1 Sessão, 2 Admitância (perfis e margem
de estabilidade), 3 Tarefas (centro-fora, túnel, alcance), 4 Gráficos (10 s),
5 Mão e juntas: *Mão virtual* (W/A/S/D/Q/E ou arrastar, só com `force_source:=virtual`)
e *Posicionar juntas* (ver abaixo).
Atalhos: 1–5 trocam de aba, Espaço = PARAR.

### Posicionar as juntas (aba 5 › Posicionar juntas)

Sliders e campos numéricos para as 6 juntas, com a posição **atual**, a do robô
**real** (no modo mirror) e o alvo. Os alvos nascem iguais à posição atual. "Ir para
a posição" move em até 12°/s (mínimo 1,5 s), só com o braço `PARADO`; o estado
passa a `POSICIONANDO` e volta sozinho a `PARADO`. **PARAR** interrompe e cancela a
trajetória. Com o robô real conectado pede confirmação. O nó recusa: alvo fora dos
limites de software (J2/J4/J5 ±170°, J3 ±160°), TCP abaixo de 10 cm ou célula
abaixo de 5 cm da mesa em QUALQUER ponto do caminho, e qualquer comando fora de
`PARADO`. **Não confere colisão do cotovelo com a mesa, a cadeira ou o voluntário:**
mova em etapas e com a mão no E-Stop. Preset HOME = mastro para cima (J5 = +90°).

## Coleta de dados

`rehab_logger` grava cada sessão em `sensors/Data/REHAB/<id>/<AAAAMMDD_HHMMSS>_<tarefa>/`
(ou em `$REHAB_PACK_DATA_DIR`). Publique `/rehab/session/meta` (subject_id,
lado_paretico, tarefa, notas) e chame `/rehab/session/start`; ele falha sem
`subject_id` ou com sessão aberta. Se a admitância for a `ABORTADO`, a sessão
fecha sozinha com status `abortada`.

- `samples.csv`: uma linha por tick (33 Hz): `TICK_FIELDS`, alvo (NaN sem alvo) e q1..q6.
- `wrench.csv`: força e torque na taxa nativa, com o frame.
- `events.csv`: `t_s, tipo, detalhe` (inclui `PARAMS` quando um parâmetro muda).
- `emg.csv`: só com `emg_channels` > 0 (ponto de extensão; sem driver de EMG).
- `meta.json`: esquema 1, commit do git, parâmetros, fonte de força, massas do mastro, status.
- `summary.json`: por alcance (tempo, caminho, retilinearidade, picos de velocidade) e totais.

O `t_s` dos ticks é tempo simulado; o `/clock` do Gazebo sai a 10 Hz, então os
nós de controle usam timer de relógio de parede e interpolam o `t_s`.

## Laboratório (`rehab_lab.world`)

Sala de reabilitação só de primitivas SDF (sem `model://` nem http), com **duas
paredes**: a do fundo (x = −1,65) e a da direita do voluntário (`wall_right_volunteer`,
y = +2,25; ele senta virado para −x). O lado −y fica aberto, com fundo de céu. Para
manter a outra parede, restaure `wall_right` do git e espelhe os pássaros. Tem janelas e porta, piso vinílico, cadeira do voluntário com manequim em
(1,20; 0) virado para o robô, cadeira de rodas, barras paralelas, espaldar,
tablado, bolas suíças, rack de halteres, cones, mesa do pesquisador, TV com a
"tela do jogo", quadro branco, armário e plantas. A mesa do robô tem tampo de
madeira, estrutura de aço, prateleira com o controlador, placa de alumínio sob
a base, fita de segurança, botão de emergência e LED na borda.

O laboratório é "vivo": 9 `<actor>` animados, só visuais (sem colisão nem física),
a mais de 1,6 m do robô e longe do voluntário. Uma terapeuta de jaleco anda entre
os cones e para para observar o voluntário; a TV roda o jogo centro-fora (cursor
ciano, alvo que acende e pisca); o ventilador de teto gira; o relógio tem ponteiro
de segundos real; nuvens e pássaros passam pelas janelas; uma bola suíça rola sob o
quadro branco e um monitor mostra o traço de ECG. RTF ≈ 1,0 com o robô carregado.
Só aparece depois de reiniciar o Gazebo: uma instância já aberta mantém o mundo antigo. Física (ODE
4 ms), piso e mesa 0,60 × 1,10 m com topo em z = 0,75 m são idênticos à versão
mínima. Só piso, mesa e paredes colidem; a decoração é visual.

## Regenerar meshes e inércias

```bash
python3 scripts/gen_joystick_meshes.py   # precisa do OCP
```

## Testes

```bash
python3 -m pytest src/rehab_pack/test -q   # a GUI pula sem DISPLAY
python3 scripts/check_sim_ft.py            # com o Gazebo no ar e sim_ft:=true
```

## Pendências e limites conhecidos

- **Pese a empunhadura pronta** e corrija a massa no URDF: 69,4 g é a densidade
  de catálogo do silicone sobre o volume do CAD (oco). Entra na compensação de peso.
- **Furação da placa:** os 2 furos ⌀5 a ±22 mm do centro não coincidem com
  nenhum padrão da face móvel da FA7155 (M5 a ±17,7 mm e M6 em (0, ±25) e
  (±11, 0)). No simulador a junta é fixa; para montar de verdade o CAD precisa de ajuste.
- `sim_sign = +1` é inferido pelo peso do mastro (Fz = −0,77 N); o teste com
  `/apply_link_wrench` não moveu o sensor. Confirme com uma força real em modo `sim`.
- O estado `LIMITE` (resíduo da IK) não foi exercitado em simulação.
- `sim_ft:=true` preserva a junta célula→placa fora do lumping do ODE; em HOME
  a J4 ficou estável (pico-a-pico < 1e-3 rad), mas valide antes de ensaios.
- A rugosidade da borracha só existe como atrito (mu 1,2) e acabamento fosco.
- Fora do escopo: driver de EMG, RV/Unity e ensaios com pacientes. O hardware
  real (CR10 e FA7155) está implementado mas sem teste em bancada (ver acima).
