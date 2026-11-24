# rehab_pack

Reabilitação de membro superior com o CR10. O voluntário segura um **mastro**
(`cad/Joystick.step`) preso à célula FA7155 e guia o robô por **admitância**
cartesiana. Usa do `touch_pack` só o driver do CR10 e o `ft_receiver`.

Padrão: CR10 simulado (Gazebo Classic) e força da **célula FA7155 real**. Sem célula o
braço simulado ainda se move pelas juntas (aba 5), mas a admitância não habilita. O
CR10 real se liga no cabeçalho do painel, como no `touch_pack`. Sem EMG real, sem RV.

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

Argumentos: `force_source:=real|sim` (padrão `real`; `sim` exige
`sim_ft:=true`), `admittance`, `logger`, `painel`, `gui`, `sim_ft`.

Outro Gazebo aberto ocupa a porta 11345 e o `gzserver` novo morre com código
255. Feche o outro, ou isole este com `GAZEBO_MASTER_URI=http://localhost:11346`
e outro `ROS_DOMAIN_ID`.

Fluxo de uma sessão: **Tarar** (mastro solto) → **Habilitar** → publicar meta →
**Iniciar sessão** → tarefa → **Encerrar sessão**. **PARAR** (tecla Espaço) em
qualquer momento trava o braço em `ABORTADO`.

**Painel:** se ele travar ou morrer por mais de 1 s com o braço se movendo, a admitância
vai a `ABORTADO`. Vale só se o painel estava vivo ao iniciar o movimento.

**Trava:** com o `touch_pack` aberto o rehab não inicia, e vice-versa (`docs/adr/0003`).

## Pilha mecânica (eixo +Z do flange, mm)

| mm | peça | material |
|---|---|---|
| 0 … 6 | flange fixo FA7155 | alumínio |
| 0,4 … 36,9 | célula FA7155 (300 g de catálogo) | — |
| 37,2 … 39,2 | placa 110 × 50 × 2 | impressa, 950 kg/m³ |
| 39,2 … 174,2 | empunhadura ⌀35 × 135 (medida) | **silicone/borracha**, 276,5 g (pesada) |

`tcp_link` = centro da empunhadura (Z = 106,7 mm do flange), a 70 mm da face da
célula: a força medida na célula precisa ser levada até ali (M = r × F).
Pose inicial (`HOME_Q_RAD`, em `constants.py`): **J5 = +90°**, empunhadura para
cima, TCP em ≈ (0,55; 0; 0,25) m no frame `world`.

## Controle de admitância

`M·a + B·v + K·(x − x_ref) = F`, 3 eixos, tick de 30 ms, Euler implícito. A força
de cada tick é a média das amostras que chegaram nele (anti-aliasing 400 → 33 Hz).

**Admitância variável.** Com a velocidade medida (do CR10 real em MIRROR, senão do
simulado), o nó observa a fração da energia que o robô devolve à mão. Acima de 0,30
sobe M e B juntos (até 8×, M/B constante) e relaxa em 3 s. Ideia de Dimeas &
Aspragathos (2016), Landi et al. (2017) e Ferraguti et al. (2019); é heurística, não
prova de passividade. Limiares ajustados só em simulação. Movimento voluntário
acima de ~1 Hz também pode ativá-la. `adm_scale` no `samples.csv` registra a escala
(1 = nominal).

**Limite de M e B.** `arm_stiffness_limit` dá a maior rigidez do braço com a malha
estável para o atraso `LOOP_DELAY_S` = 0,12 s (**estimado: meça**). O nó recusa
combinações abaixo de 100 N/m e a GUI avisa abaixo de 300 N/m.

Núcleo em
`admittance.py`, cinemática em `kinematics.py` (cadeia `world → tcp_link` lida do
URDF gerado), comando do braço por `/cr10_group_controller/joint_trajectory`
(um ponto por tick). A orientação do mastro é mantida fixa: a tara vale a
sessão toda.

Estados: `PARADO`, `ATIVO`, `SEM_FORCA`, `LIMITE`, `ABORTADO`, `POSICIONANDO`. Segurança:
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
  `ft_autozero`, `ft_filter`; aqui ambos nascem `false`: o auto-zero come força fraca
  sustentada e o One-Euro atrasa a malha ~80 ms). Ele é o único dono da porta serial e publica
  `/ft_sensor/wrench`; o `rehab_force_source` o tara e o leva ao frame do mundo.
- **Robô:** o `rehab_mirror` sobe sempre; no cabeçalho do painel: IP, Conectar e modo
  SIM_ONLY/MIRROR, como no touch_pack. `control_mode:=mirror` não conecta ao subir: use Conectar. Ele usa o
  `CR10RealDriver` do `touch_pack` (TCP 29999, feedback 30004). O IP vem de
  `robot_ip`, senão de `~/.config/touch_pack/robot.json` (o mesmo da GUI do
  touch_pack), senão 192.168.5.2. O braço simulado manda ServoJ a 33 Hz ao real.
  Em MIRROR e PARADO, o `rehab_mirror` leva o simulado até a pose do real; só arma
  abaixo de 3°. O rehab só lê o `robot.json`, nunca o altera.
- **Sem hardware:** `robot_ip:=sim` usa um robô real simulado (segue o ServoJ),
  para ensaiar o fluxo todo.

**Intertravamentos (modo mirror):** a admitância só habilita com o robô conectado,
sem E-Stop (inclusive o físico: RobotMode 5, 7 ou 8) e com sim e real a menos de 3°; durante o movimento tolera 8° de
atraso. Perdeu qualquer condição em `ATIVO` → `ABORTADO`, e o `rehab_mirror`
manda `Stop()` + `Continue()` ao robô. O espelho só corre com a admitância `ATIVA`.
O painel mostra o estado do robô real e da célula no cartão Conexões.
O padrão é `control_mode:=sim_only`: nada move o robô real sem pedir.

**Força simulada nunca move o robô real:** com MIRROR e fonte diferente de `real` o launch
falha, a tela recusa e o nó não habilita (e aborta). Exceção: `robot_ip:=sim`. Habilitar exige força recente e ≤ 3 N; pico de
UMA amostra acima do limite aborta; célula muda em MIRROR aborta (não retoma sozinha);
dois nós publicando `/rehab/ft/wrench` recusam; tara só em PARADO/ABORTADO, obrigatória
com a célula real e refeita se ela girar mais de 5°. Torque no centro da empunhadura
acima de 2 N·m aborta (`abort_torque_nm`, PROVISÓRIO: sem norma). Em ATIVO a célula
fica ≥ 5 cm e cotovelo/punho ≥ 10 cm sobre a mesa (senão `LIMITE`); coxas e cadeira
NÃO são conferidas. Ordem com paciente: braço apoiado (tipoia/apoio), mastro solto,
tarar, habilitar, só então segurar. Ao armar, a diferença sim−real some em 1,5 s. PARAR vai
também direto ao `rehab_mirror`. Conexão que cai só volta pelo operador.

**Pré-voo (somente leitura):** `ros2 run rehab_pack rehab_preflight --robot-ip 192.168.5.2 --ft-port /dev/ttyUSB0`
confere, sem enviar nada ao robô: se o `install` é o do `src` (não uma cópia
antiga), o driver e os executáveis do `touch_pack`, se o PC está na sub-rede do robô,
se as portas 29999/30004 respondem, o grupo `dialout`, o `pyserial` e o conversor
USB-RS485 da FA7155. Termina com código 1 se houver FALHA.

**Antes do primeiro ensaio com pessoa (não verificado):**
1. Confirme o sentido dos eixos: com o mastro solto e tarado, empurre em +X do
   mundo e veja `/rehab/ft/wrench` e o TCP irem para +X; ajuste `ft_sign` e
   `ft_yaw_deg` no launch.
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
5 Juntas.
Atalhos: 1–5 trocam de aba, Espaço = PARAR.

### Juntas (aba 5)

Como no `touch_pack`: o braço segue o slider na hora (agrupa 80 ms); digitando, confirme
com Enter. Em MIRROR o robô REAL também segue. Velocidade em % (100 % = 24°/s). **Home**
vai à pose salva; **Salvar Home** grava a pose atual e **Capturar do robô** a do CR10 real
em `~/.config/rehab_pack/home_pose.json`, que também é a pose inicial do Gazebo. Só em
`PARADO`/`POSICIONANDO`. Recusa limite de junta, TCP abaixo de 10 cm ou célula abaixo de
5 cm no caminho e cotovelo/punho junto à mesa; **não confere cadeira nem pessoas**. Em MIRROR
exige força recente da célula. PARAR interrompe.

## Bancada falsa (tudo menos o robô)

`scripts/fake_bench.py` fala os protocolos do CR10 (TCP 29999/30004) e da FA7155 (Modbus
numa pty), para rodar o `CR10RealDriver` e o `ft_receiver` verdadeiros sem hardware:
`robot_ip:=127.0.0.1 ft_port:=<porta impressa>`. Não simula dinâmica nem tempos do firmware.

## Coleta de dados

`rehab_logger` grava cada sessão em `sensors/Data/REHAB/<id>/<AAAAMMDD_HHMMSS>_<tarefa>/`
(ou em `$REHAB_PACK_DATA_DIR`, ou `$TOUCH_PACK_DATA_DIR/REHAB`). Publique `/rehab/session/meta` (subject_id,
lado_paretico, tarefa, notas) e chame `/rehab/session/start`; ele falha sem
`subject_id` ou com sessão aberta. Se a admitância for a `ABORTADO`, a sessão
fecha sozinha com status `abortada`.

- `samples.csv`: uma linha por tick (33 Hz): `TICK_FIELDS`, alvo (NaN sem alvo), q1..q6
  do simulado e q1..q6 do CR10 real (NaN sem robô). Posição e TCP são do simulado.
- `wrench.csv`: força e torque na taxa nativa, com o frame.
- `events.csv`: `t_s, tipo, detalhe` (inclui `PARAMS` quando um parâmetro muda).
- `emg.csv`: só com `emg_channels` > 0 (ponto de extensão; sem driver de EMG).
- `meta.json`: esquema 2, commit do git, parâmetros, fonte de força, massas do mastro, status.
- `summary.json`: por alcance (tempo, caminho, retilinearidade, picos de velocidade) e totais.

O `t_s` dos ticks é tempo simulado; o `/clock` do Gazebo sai a 10 Hz, então os
nós de controle usam timer de relógio de parede e interpolam o `t_s`.

## Laboratório (`rehab_lab.world`)

Sala de reabilitação só de primitivas SDF (sem `model://` nem http), com **duas
paredes**: a do fundo (x = −1,65) e a da direita do voluntário (`wall_right_volunteer`,
y = +2,25; a cadeira é virada para −x). O lado −y fica aberto, com fundo de céu. Para
manter a outra parede, restaure `wall_right` do git e espelhe os pássaros. Tem janelas e porta, piso vinílico, cadeira do voluntário (vazia) em
(1,20; 0) virada para o robô, cadeira de rodas, barras paralelas, espaldar,
tablado, bolas suíças, rack de halteres, cones, mesa do pesquisador, TV com a
"tela do jogo", quadro branco, armário e plantas. A mesa do robô tem tampo de
madeira, estrutura de aço, prateleira com o controlador, placa de alumínio sob
a base, fita de segurança, botão de emergência e LED na borda.

O laboratório tem 5 `<actor>` só visuais (jogo na TV, relógio, nuvens e pássaros).
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

- **Centro de massa do mastro:** a massa foi pesada (285,9 g), mas o CoM e a inércia
  supõem cilindro uniforme. Meça o ponto de equilíbrio (esperado: 67 mm da face da placa).
- **`cad/Joystick.step` desatualizado:** traz 150 mm de empunhadura; a peça tem 135.
- **Furação da placa:** a peça real é aparafusada direto na FA7155 (operador, 24/11/2026),
  mas o `cad/Joystick.step` não tem essa furação: os 2 furos ⌀5 a ±22 mm não coincidem
  com a face móvel (M5 a ±17,7 mm; M6 em (0, ±25) e (±11, 0)). O CAD está diferente da peça.
- `sim_sign = +1` é inferido pelo peso do mastro (Fz = −0,77 N); o teste com
  `/apply_link_wrench` não moveu o sensor. Confirme com uma força real em modo `sim`.
- O estado `LIMITE` (resíduo da IK) não foi exercitado em simulação.
- `sim_ft:=true` preserva a junta célula→placa fora do lumping do ODE; em HOME
  a J4 ficou estável (pico-a-pico < 1e-3 rad), mas valide antes de ensaios.
- A rugosidade da borracha só existe como atrito (mu 1,2) e acabamento fosco.
- Fora do escopo: driver de EMG, RV/Unity e ensaios com pacientes. O hardware
  real (CR10 e FA7155) está implementado mas sem teste em bancada (ver acima).
