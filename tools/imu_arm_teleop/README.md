# imu_arm_teleop

Bridge standalone (fora do workspace ROS) que usa os 6 ESPs do **WBAN-Inercial**
para teleoperar o CR10 do **touch_pack** por posição cartesiana. Não é um nó
ROS2 — é um script Python isolado que importa diretamente os módulos de fusão
do WBAN (`core.communications`, `core.dataprocessing`) e os de cinemática do
touch_pack (`manip3d`, `kinematics`, `real_driver`).

## Mapeamento (fase 1 — só posição)

`shoulder → arm → forearm` formam uma cadeia de elos (comprimento configurável)
cuja ponta dá a posição da "mão" relativa ao `trunk` (referência). O
deslocamento dessa posição desde o momento do `start`/`z` é escalado
(`motion_scale`) e somado à pose TCP inicial do robô; a IK diferencial do
touch_pack (`manip3d.ik_step`) resolve as 6 juntas. **A orientação do TCP fica
travada** na pose inicial — seguir a orientação da mão fica para depois de
validar a posição com segurança.

## Pré-requisitos

1. Os 6 ESPs calibrados (AG, e MAG se for ligar o magnetômetro) via
   `core/sensorcalibration.py` do WBAN-Inercial.
2. `python3 -m venv .venv && source .venv/bin/activate && pip install -r requirements.txt`
   (venv **isolado** — não instale isso no ambiente do ROS2, que precisa de
   `numpy<2`; este bridge não depende do ROS2, então não tem esse problema).
3. Copie `config.example.json` → `config.json` e ajuste:
   - `segment_to_sensor`: qual ESP (1–6, o ID que ele mesmo anuncia) está em
     qual lugar do braço. Sobra 1 sensor spare se você só usar os 5 nomes.
   - `chain[].length_m`: comprimento real de cada elo (meça no corpo).
   - `tcp_end_effector`: `"touch_tool"` ou `"hand"` — **tem que bater com a
     ferramenta de verdade montada no flange agora**, senão a pose "home"
     capturada e os alvos da IK ficam em referenciais diferentes (gera deriva
     constante mesmo com os sensores parados).
   - `limb_axis_local`: eixo local do sensor que aponta "para fora" do
     tronco, ao longo do membro. Depende de como o sensor é vestido — valide
     em dry-run antes de confiar.

## Uso

```bash
source .venv/bin/activate
python3 imu_arm_teleop.py --config config.json            # dry-run (não toca o robô)
python3 imu_arm_teleop.py --config config.json --live      # braço real
```

Depois que o processo imprimir "Pronto", digite no terminal:

- `start` — começa a seguir os sensores (zera a referência nesse instante).
- `z` — re-zera sem reiniciar (ver limitação de deriva abaixo).
- `stop` — para e desconecta.

## Segurança

- `dry_run=True` por padrão; `--live` é obrigatório pra tocar o robô real.
- Alvo cartesiano travado numa caixa de `safety_box_m` em torno da pose
  inicial — por eixo, não em raio.
- Se `arm` ou `forearm` ficarem sem dado novo por mais que `stale_s`, o braço
  **congela** (não teleopera com dado obsoleto) e loga um aviso.
- Encerrar o processo (`stop`, Ctrl+C, ou matar o processo) simplesmente para
  de enviar ServoJ — o comportamento do controlador Dobot ao parar de receber
  pontos é o mesmo já assumido pelo resto do touch_pack (mirror_node,
  tactile_explorer): não há um "stop" explícito adicional enviado ao
  controlador além do que `CR10RealDriver.stop()`/`close()` já fazem no
  `shutdown()`.

## Limitações conhecidas (leia antes de usar com o braço real)

- **Deriva de heading**: `core/dataprocessing.py` do WBAN está forçando o
  magnetômetro a zero (ver o comentário `Forçando zero` no método
  `convert_data`), então a fusão Madgwick roda só com Acc+Gyro — sem
  referência absoluta de rumo. O yaw de cada sensor deriva com o tempo, e
  isso contamina a posição estimada da mão. Prefira sessões curtas; use `z`
  para re-zerar com frequência.
- **Qual interface de rede o WBAN escolhe**: `core/utils.get_default_ip()`
  prefere `172.30.x.x`/`172.31.x.x` e, na ausência, cai pra primeira interface
  compatível que encontrar entre as redes privadas comuns — com Ethernet e o
  hotspot Wi-Fi ativos ao mesmo tempo, a escolha **não é garantida** ser a
  interface Wi-Fi onde os ESPs estão. Se a descoberta multicast não achar os
  sensores, desligue a Ethernet temporariamente (o hotspot não depende dela
  pra falar com os ESPs, só pra NAT de internet) e rode de novo.
- **Eixos/escala não calibrados**: `limb_axis_local`, `r_human_to_robot` e
  `motion_scale` em `config.example.json` são valores de partida, não
  medidos. Valide em `--dry-run` (observe os logs/alvos) movendo um eixo por
  vez antes de ligar `--live`.
- **Fase 1 é só posição**: orientação do TCP fixa; não espere a mão "virar"
  o efetuador.
