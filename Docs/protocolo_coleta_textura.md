# Protocolo de coleta de textura — modo SLIDE

**Projeto:** cr10twin / `touch_pack` · **Braço:** Dobot CR10 · **Sensor tátil:** matriz
piezorresistiva 5×5 + modelo de Izhikevich (STM32) · **Referência:** Gupta et al.,
*Spatio-temporal encoding improves neuromorphic tactile texture classification*,
IEEE Sensors Journal, 2021 (`Docs/Gupta, A. K. 2021.pdf`).

**Data:** 03/09/2026 · **Status:** proposta de bancada, pendente de execução.

---

## 1. Por que o setup atual não serve para textura

O que existe hoje na bancada foi construído para **palpação por indentação**, e faz
isso bem. Para textura ele tem três problemas, e todos são de hardware:

| # | Problema | Consequência na coleta |
|---|---|---|
| 1 | A ponteira D termina numa **face plana perpendicular ao eixo da ferramenta**, e o `HOME` do explorer exige o TCP apontando para baixo. O contato é uma placa rígida 22,4 × 24,9 mm apoiada de chapa sobre a amostra. | Não há um ponto de contato definido. Qualquer desalinhamento de décimos de grau transfere a carga toda para uma aresta, e o laminado passa a ler um gradiente de montagem, não a textura. |
| 2 | **Não há pele.** O laminado toca a amostra direto (a 0,5 mm de flex + piezorresistivo + flex). Gupta cobriu o sensor com uma camada macia (fita VHB 3M) exatamente para *difundir a força e proteger o sensor*. | Sem a camada macia não existe a filtragem espacial que faz o sinal ser textura e não impacto pontual; e o laminado desliza abrasivamente contra bucha e juta. |
| 3 | Contra superfície **rígida** a reserva de contato durante o `SLIDING` é de poucos micrometros — o próprio código diz isso (`_SLIDE_CONTACT_MIN_FRAC`, comentário em `tactile_explorer.py`): a fase não regula força, e a única reserva é a indentação `alvo/K` que o `HOLD` deixou. | Com `K` da ordem de centenas de N/mm, 1 N de setpoint deixa ~5 µm de reserva. Qualquer não-paralelismo da amostra ao longo de 90 mm perde o contato no meio do curso. |

A capa de silicone resolve o item 3 de quebra: ela derruba `K` de centenas de N/mm
para a ordem de **poucos N/mm**, e a mesma 1 N de setpoint passa a deixar
**centenas de micrometros** de reserva de indentação. É a diferença entre um curso
de 90 mm que depende do calço e um que se sustenta sozinho.

---

## 2. Parâmetros da referência (Gupta et al., 2021, §II-B e §II-D)

Para reproduzir, e não apenas "inspirar-se":

| Parâmetro | Valor no artigo |
|---|---|
| Força normal de contato | **1 N**, mantida por controle de força em malha fechada |
| Fase de estabilização (*hold*) | **5 s** com movimento relativo congelado |
| Distância de deslize | **90 mm** |
| Velocidades de deslize | **5, 10 e 15 mm/s** |
| Estímulos | **8 texturas naturais** (3 cerâmicas de piso, papelão corrugado, piso de borracha, tapete têxtil, Scotch-Brite, isopor) |
| Fixação | plataforma impressa, fita dupla-face, **plataforma parafusada à mesa** |
| Montagem do sensor | *cuff* de dedo impresso em material macio, sobre mão antropomórfica (iLimb) |
| Proteção / difusão da força | camada macia transparente (fita VHB, 3M) sobre o sensor |
| Braço | UR-10 |
| Aquisição | 16 taxels a 1 kHz |

As quatro fases do artigo — **Contact → Hold → Sliding → Retract** — são exatamente
as fases `DESCENDING → HOLD → SLIDING → RETRACT` do `tactile_explorer`. O modo
`SLIDE` da GUI já executa esse ciclo; o que falta é a ponta que toca.

---

## 3. O que precisa ser fabricado

> **REVISÃO 03/09/2026 — a ponteira H foi descartada.** A decisão foi manter a
> **ponteira D** e fabricar apenas a capa de silicone para ela. Justificativa: dos
> três problemas do §1, o único que a ponteira nova resolvia e a capa não era o
> ângulo de contato — e o sensor do próprio Gupta era **plano**, com a camada macia
> (fita VHB) fazendo todo o trabalho. Pior: com a faceta a 25° o ponto de contato sai
> até 12 mm do eixo, criando um momento que a célula axial de 1 eixo não enxerga e
> converte em erro de força normal. A capa de silicone sobre a D custa um molde, não
> mexe em URDF, kinematics, compensação de gravidade nem na HOME, e entrega a
> proteção do laminado, a difusão da força e a reserva de indentação do `SLIDING`.
> **Os §§3.1 e 3.2 abaixo ficam como registro da alternativa avaliada; o que vale é
> o §3.2-bis.** Ver `Docs/molde_capa_silicone_D.md`.

### 3.1 ~~Ponteira H — corpo de dedo com faceta plana inclinada~~ (descartada)

O laminado 5×5 é **rígido e plano**, 17 × 19,47 mm. Ele não se conforma a uma calota
esférica: qualquer tentativa de curvá-lo em dois eixos delamina o flex ou descola os
eletrodos. A saída é separar as duas funções:

- **a faceta é plana** — é onde o laminado é colado/fixado, e ele fica inteiro apoiado;
- **a curvatura de dedo vem da capa de silicone** — que é o que efetivamente toca a
  amostra, exatamente como a fita VHB do Gupta faz o papel da pele.

Geometria proposta (`cad/step/ponteira_H_finger_5x5.step`, a modelar):

| Trecho (Z do ombro da haste) | Descrição |
|---|---|
| 0 … 10 mm | Luva sobre a haste do `touch_tool`: furo ⌀10,2 mm, externo ⌀15,2 mm — **mesma interface da ponteira D e da G** (`CYLINDRICAL_SURFACE` R5,1 / R7,6 nos STEP existentes), então a ponteira continua hot-swap |
| 10 … 14 mm | Cone de transição ⌀15,2 → ⌀20 |
| 14 … 17 mm | **Rasgo de retenção** da capa: ⌀18 × 3 mm de largura, para o lábio da capa de silicone travar e não escorregar durante o arrasto |
| 17 … ~44 mm | Corpo do dedo, ⌀20 mm |
| face distal | **Plano inclinado 25° em relação ao plano XY**, com bolso rebaixado 0,6 mm de 17,4 × 19,9 mm para o laminado, canal de saída do flex de 6 × 1 mm pelo lado posterior, e faixa rebaixada 0,5 mm × 6 mm de largura ao redor do corpo para a fita de fixação ficar embutida |
| arestas | Aresta de ataque (a mais baixa da faceta) com raio **R6**; demais arestas **R3** — nenhuma quina viva sob o silicone |

**Por que 25°.** É o ângulo em que um dedo humano encosta numa mesa em exploração
tangencial. Ele garante que o contato comece pela aresta de ataque e cresça como
mancha à medida que o silicone deforma — que é o regime em que a matriz de taxels vê
um gradiente espacial, e não uma pancada uniforme. Com a faceta perpendicular
(0°) todos os 25 taxels veem a mesma coisa, e o ganho do *pooling* espacial que é
a tese do Gupta desaparece.

**Direção do deslize.** A faceta inclina no eixo **X da ferramenta**. O deslize deve
ser comandado em **`+X` ou `-X`** (`slide_dir`), nunca em Y — em Y o dedo arrasta de
través e a aresta lateral rasga a capa. Convencionar: **`-X` = ladeira abaixo**
(aresta de ataque na frente) é a direção de referência; `+X` fica para o ensaio de
simetria de direção.

**Consequência no TCP.** Trocar a ponteira **muda a altura do TCP** e o ponto de
contato deixa de estar no eixo. Isso toca três arquivos, como o próprio URDF avisa:

1. `src/touch_pack/urdf/touch_tool_tcp.urdf` — fonte da verdade;
2. `touch_pack/kinematics.py` → `T_TOUCH_TOOL_ATTACH`;
3. `src/touch_pack/config/tactile_controllers.yaml` → bloco `gravity_compensation.CoG`.

E **re-ensinar a HOME** antes do primeiro run: o carimbo `tool_tcp_mm`
(`constants.tool_stamp`) vai avisar na tela que as poses salvas foram ensinadas com
outra ferramenta, mas ele é diagnóstico, não corretivo.

### 3.2 Capa de silicone (Dragon Skin) e o molde

**Material:** Dragon Skin (Smooth-On, cura por platina, 1:1 em volume).

- **Dragon Skin 10 Medium** (Shore A 10) — mais próximo da polpa do dedo, transmite
  bem a vibração de alta frequência. É a recomendação para o conjunto de estímulos
  escolhido (tecidos, bucha, materiais naturais), onde a informação está na
  micro-vibração e não na macro-geometria.
- **Dragon Skin 20** (Shore A 20) se a capa de A10 mostrar histerese visível entre
  ida e volta do deslize, ou se rasgar na bucha verde.

**Espessura de parede: 1,5 mm** sobre a faceta do sensor. É o compromisso: abaixo de
1 mm a capa rasga na bucha e transmite impacto pontual; acima de 2 mm ela vira um
filtro passa-baixas e apaga justamente a alta frequência que distingue as texturas.
Nas laterais e no lábio de retenção a parede sobe para 2,5 mm, onde a resistência ao
rasgo importa mais que a fidelidade.

**Molde de 3 peças** (`cad/step/molde_H_*.step`, a modelar), no mesmo estilo dos
`molde_corpo_1` / `molde_tampa` que já existem na pasta (corpo cilíndrico, pinos de
registro ⌀3, parede de 6 mm):

| Peça | Função |
|---|---|
| `molde_H_casca_A` / `molde_H_casca_B` | Concha externa, partida no plano **Y = 0** (o plano que contém o eixo da ferramenta e a normal da faceta). A forma externa não tem contra-saída nesse plano, então as duas metades saem retas. Pinos de registro ⌀3 e orelhas com furo M3 para o grampo. |
| `molde_H_macho` | Réplica da ponteira H **com o bolso do sensor preenchido** (um bloco sólido 17,4 × 19,9 × 0,6 no lugar do laminado). É ele que define o *interior* da capa, e é por isso que ele não pode ser a ponteira real: o interior da capa tem que casar com o dedo **já com o laminado instalado**. |

**Canal de injeção** (⌀4) no topo, entrando pela região que vira o lábio; **dois
respiros** ⌀1,5 nos dois pontos mais altos da cavidade quando o molde está na posição
de vazamento. Impressão em PLA ou PETG, camada 0,15 mm, e **desmoldante obrigatório**
(Ease Release 200 ou álcool polivinílico) — Dragon Skin adere a PLA impresso.

> **Cuidado de inibição:** silicone de platina é inibido por enxofre, estanho e
> algumas resinas. Não usar luva de látex, massa de modelar, nem peça impressa em
> resina não pós-curada em contato com a mistura.

### 3.3 Fixação do laminado na ponteira

O laminado entra no bolso de 0,6 mm e é preso com **fita adesiva de poliimida
(Kapton), 12 mm**, em duas voltas na faixa rebaixada do corpo, com o flex saindo pelo
canal posterior. A fita **não pode passar por cima da área ativa** — ela muda a
rigidez local e o taxel coberto passa a ler a fita. Se a fita sozinha não segurar
sob arrasto, colar apenas as **duas bordas** do laminado (não a face inteira) com
cianoacrilato gel, deixando a área ativa livre.

Alívio de tração no flex: uma volta de fita prendendo o cabo ao corpo do
`touch_tool`, **antes** da luva, para que qualquer puxão vá para o corpo e não para
a solda do flex.

### 3.4 Placa de amostras

Réplica funcional da plataforma do Gupta:

- Placa de MDF ou alumínio de **150 × 300 mm**, plana, **parafusada à mesa** (não
  apenas apoiada — 90 mm de arrasto a 1 N move qualquer coisa solta).
- Amostras coladas com **fita dupla-face** em janelas de **110 × 60 mm**, deixando
  10 mm de folga em cada extremidade do curso de 90 mm.
- **Nivelamento:** a placa precisa estar paralela ao plano de deslize dentro de
  **0,3°**. A 90 mm de curso, 0,3° são 0,47 mm de deriva em Z — dentro da reserva de
  indentação da capa de silicone (~0,3–0,5 mm a 1 N), mas já perto do limite. Usar a
  **calibração dinâmica do ângulo de ataque** (`plane_probe`, 4 toques, raio 15 mm,
  1 N) antes de cada sessão, e o calço para o que sobrar: acima de 10° o software
  recusa (`_SLIDE_SLOPE_MAX_DEG`) e acima de ~1° o lugar de corrigir é o calço.
- Amostras têxteis (juta, malha) precisam ficar **tensionadas**, não onduladas — uma
  moldura de retenção nas quatro bordas resolve.

### 3.5 Decisão de sensor de força: célula axial × FA7155

Esta é a decisão mais importante do documento, e ela muda o que a coleta consegue medir.

| | Célula axial 100 kg (XIAO + HX711) | FA7155 6 eixos (RS485) |
|---|---|---|
| Eixos | 1 (normal) | 6 (Fx, Fy, Fz, Mx, My, Mz) |
| Taxa entregue | ~24 Hz | **1 kHz** (exemplar da bancada, 1 Mbps) |
| Ruído σ em repouso | 23 mN | a re-medir |
| Comprimento do TCP | 162,2 mm | 67,7 mm |
| Mede **atrito**? | **Não** | **Sim** — Fx/Fy durante o `SLIDING` |

Para textura, **a força tangencial é sinal, não ruído**: o coeficiente de atrito e a
sua modulação ao longo do curso separam bucha de juta antes mesmo de o modelo
neuromórfico opinar. E 24 Hz não amostram nada de vibração — a 10 mm/s, 24 Hz
resolve uma feição a cada 0,42 mm; a bucha verde tem estrutura bem abaixo disso.

**Recomendação: rodar as coletas de textura com `force_sensor:=ft6`.** O custo é
real e precisa ser aceito conscientemente:

- a pilha mecânica muda (a FA7155 é 94,5 mm mais curta) — os três arquivos do §3.1
  mudam de novo, e o `PROBE_ALIGN_RETRACT_MM` de 20 mm, dimensionado para a
  ferramenta longa, continua válido (folgado) para a curta;
- `FORCE_NOISE_SIGMA_N = 0,023 N` foi medido no HX711 e **precisa ser re-medido**
  com a FA7155 — é ele que fixa o menor setpoint perseguível, e 1 N do Gupta é
  perto do piso;
- confirmar `ft_force_sign` na bancada com `scripts/ft_probe.py --zero` **antes do
  primeiro run**: sinal invertido faz o corte de segurança ver tração onde há
  compressão.

Se por qualquer motivo a coleta tiver que sair com a célula axial, ela ainda vale —
o sinal de textura vem do laminado a 1 kHz, não da célula — mas o eixo de atrito
fica perdido e não há como recuperá-lo depois.

---

## 4. Conjunto de estímulos

Oito texturas naturais, como no artigo, escolhidas para **variar muito entre si** em
atrito, rugosidade e regularidade espacial — não para serem parecidas.

| # | Textura | Escala espacial dominante | Papel no conjunto |
|---|---|---|---|
| T1 | Bucha verde de cozinha (Scotch-Brite / similar) | fibras ~0,1 mm, emaranhado aleatório | alto atrito, aleatória — é a mesma do Gupta (h) |
| T2 | Bucha de espuma (lado amarelo macio) | poros ~0,5–1 mm | mesma família de T1, com módulo muito menor: testa se o sistema separa rugosidade de complacência |
| T3 | Palha de aço / esfregão metálico fino | ~0,05 mm | extremo fino, alto atrito |
| T4 | Juta / aniagem | trama ~1 mm, **periódica** | regularidade espacial forte, tecido — a mesma do artigo |
| T5 | Jeans (sarja) | trama ~0,5 mm, diagonal | periódica fina e **anisotrópica** — habilita o ensaio de direção |
| T6 | Feltro ou lã | fibras curtas, sem periodicidade | baixo atrito, macio, quase sem estrutura |
| T7 | Couro (ou sintético com grão) | grão ~0,3 mm irregular | superfície lisa mas texturada, atrito médio |
| T8 | Cortiça ou papelão corrugado | 0,2 mm (cortiça) / ~5 mm (corrugado) | referência de baixa/alta escala; o corrugado é o único com feição macroscópica |

**Controle (T0):** placa de acrílico lisa. Não é textura — é a **linha de base**.
Todo run de T0 que produzir estrutura no sinal está denunciando vibração do braço,
ressonância da ferramenta ou artefato do laminado, não a amostra. Coletar T0 no
início e no fim de cada sessão.

**Registro obrigatório por amostra:** foto macro com escala (régua no quadro), lote /
procedência, data de colagem na placa, e o número de runs já feitos sobre ela —
bucha e palha de aço **se desgastam** ao longo de algumas dezenas de passadas, e um
conjunto coletado ao longo de duas semanas pode ter uma deriva de material que o
classificador vai aprender como se fosse a textura.

---

## 5. Matriz experimental

```
8 texturas × 3 velocidades (5, 10, 15 mm/s) × 20 repetições = 480 runs
+ T0 controle × 3 velocidades × 5 repetições               =  15 runs
                                                            ───────────
                                                             495 runs
```

**Tempo estimado por run** (força 1 N, curso 90 mm):

| Fase | 5 mm/s | 10 mm/s | 15 mm/s |
|---|---|---|---|
| Trânsito + descida | ~25 s | ~25 s | ~25 s |
| HOLD (5 s + assentamento) | ~8 s | ~8 s | ~8 s |
| SLIDING (90 mm) | 18 s | 9 s | 6 s |
| Retração + retorno | ~20 s | ~20 s | ~20 s |
| **Total** | **~71 s** | **~62 s** | **~59 s** |

≈ **8,5 h de braço em movimento**, sem contar troca de amostra. Planejar **4 sessões
de meio período**, com o T0 abrindo e fechando cada uma.

**Ordem dos runs:** aleatorizar **velocidade dentro de cada bloco de textura**, e
aleatorizar a ordem das texturas entre sessões. Não coletar todas as repetições de
uma condição em sequência — deriva térmica do laminado e da célula ficaria
perfeitamente correlacionada com a condição, e o classificador aprenderia a deriva.

**Posição de partida do deslize:** deslocar o ponto de início em ±3 mm no eixo
transversal a cada repetição (seis posições, ciclando). Isso evita que 20 passadas
cavem o mesmo sulco na amostra e evita que o modelo aprenda um defeito local em vez
da textura.

**Ensaio complementar de direção (opcional, depois do conjunto principal):**
T4/T5/T8 (as periódicas) × `+X` e `-X` × 10 repetições a 10 mm/s = 60 runs.
Responde se a codificação é invariante ao sentido do arrasto.

---

## 6. Preparação da bancada (checklist de sessão)

Nada disto é opcional; a ordem importa.

1. **Ferramenta montada e declarada.** Ponteira H com capa de silicone e laminado
   fixado. Conferir que `touch_tool_tcp.urdf`, `kinematics.T_TOUCH_TOOL_ATTACH` e
   `tactile_controllers.yaml` descrevem **esta** pilha. `check_urdf` no arquivo.
2. **Re-ensinar a HOME** e apagar o contato aprendido antigo (desmarcar *Home
   conhecida* na GUI, que dispara `/palpation/forget_contact`). O carimbo
   `tool_tcp_mm` vai reclamar das poses velhas — é para reclamar mesmo.
3. **Sensor de força.** `ft_probe.py --list` → `--raw` → `--zero`. Apertar a ponteira
   com o dedo e confirmar que a compressão sai **positiva**; ajustar
   `ft_force_sign` se não. Registrar σ em repouso (2000+ quadros) e atualizar
   `FORCE_NOISE_SIGMA_N` se ele mudou.
4. **Sensor tátil.** Aba *Sensores* da GUI: os 25 taxels respondendo, sem taxel morto
   nem saturado. Um toque leve com cotonete em cada canto confirma a orientação
   (o firmware entrega o quadro girado 180°; `taxel_frame_to_physical` corrige, mas
   confira na tela).
5. **Placa parafusada**, amostra colada, moldura tensionando o têxtil.
6. **Calibração do ângulo de ataque** sobre a amostra: 4 pontos, raio 15 mm, 1 N.
   Registrar a inclinação medida. **Acima de 1°, calçar** — não seguir.
7. **Tare** da célula com a ferramenta no ar, na pose de partida (não na HOME
   genérica: a gravidade projeta diferente em cada pose).
8. **T0 de abertura** — um run de controle na placa lisa antes da primeira amostra.

---

## 7. Execução

Modo `SLIDE`, pela GUI (aba *Palpação*) ou pelo tópico. Pelo terminal, um run
completo do protocolo Gupta a 10 mm/s:

```bash
ros2 topic pub --once /palpation/start touch_pack_msgs/msg/PalpationStart \
  "{mode: 'SLIDE',
    force_n: 1.0,
    depth_mm: 15.0,
    approach_speed_mms: 20.0,
    speed_mms: 10.0,
    slide_dist_mm: 90.0,
    slide_dir: '-X',
    repeats: 1,
    speed_factor_pct: 10.0,
    hold_stable_s: 5.0,
    hold_timeout_s: 30.0}"
```

Com o launch subido em espelhamento e a célula de 6 eixos:

```bash
ros2 launch touch_pack tactile_cell.launch.py \
    end_effector:=touch_tool sensor:='5' \
    force_sensor:=ft6 \
    control_mode:=mirror robot_ip:=192.168.5.2
```

**Notas sobre os parâmetros:**

- `force_n: 1.0` é o **mínimo selecionável na GUI** (o campo é inteiro, 1–10 N) e é
  exatamente o valor do artigo. A meia-banda do `HOLD` nesse alvo é
  `max(4σ, 5% do alvo)` = **0,092 N** (o piso de ruído ganha), ou seja ±9,2 % —
  mais larga do que se gostaria, e é o argumento mais forte para re-medir σ com a
  FA7155: um σ menor aperta a banda.
- `depth_mm: 15.0` é **curso máximo de segurança**, não profundidade alvo. Com a capa
  de silicone e 1 N, a penetração real fica na casa de 0,3–0,5 mm.
- `hold_stable_s: 5.0` reproduz o *hold* de 5 s do artigo.
- `slide_dist_mm: 90.0` — conferir que a placa tem 110 mm de janela e que o curso não
  atravessa singularidade do punho. Fazer **um ensaio a seco** (sem contato, 20 mm
  acima) antes da primeira amostra de cada janela nova.
- **Não** usar `repeats > 1` para as 20 repetições: entre repetições o braço volta à
  HOME e reencontra o contato, mas o run vira **um único CSV** e as passadas ficam
  concatenadas. Um run por passada mantém uma pasta por amostra e torna trivial
  descartar uma passada ruim.

---

## 8. Dados gerados

Cada run cria `sensors/Data/SLIDE/<AAAAMMDD_HHMMSS>/` com:

| Arquivo | Fonte | O que interessa aqui |
|---|---|---|
| `samples.csv` | `palpation_logger` | `t_rel_s, phase, force_net_n, taxel_0..taxel_24, n_RA, n_SA, cn_*` — filtrar `phase == 3` (`SLIDING`) recorta exatamente a passada |
| `sensors.csv` | GUI | **stream de 1 kHz**: `v00..v44` dos taxels + força. É este o arquivo que reproduz a amostragem do artigo |
| `adc.csv`, `spikes.csv`, `cuneiformes.csv` | GUI | linhas cruas do firmware; `spikes.csv` é o trem RA/SA |
| `params.json` | logger | parâmetros do run + impressão da calibração |
| `summary.json`, `plot.png` | `palpation_report` | métricas por fase |

**O que o esquema atual não registra e precisa ser anotado à parte** (planilha da
sessão, uma linha por run):

`run_id · textura · velocidade · direção · repetição nº · posição transversal ·
ângulo medido na calibração de plano · capa de silicone (lote/nº de runs) ·
temperatura ambiente · operador · aceito/descartado + motivo`

Sugestão de melhoria de software: acrescentar um campo livre `sample_label` à
`PalpationStart` e gravá-lo no `params.json`. Sem isso, a associação run↔textura vive
fora do repositório, que é onde ela se perde.

**Nota sobre `lc_voltage_raw_v` / `lc_voltage_v` com a FA7155:** essas colunas
carregam a força do eixo de controle **em newtons**, não em volts. Documentado, mas
fácil de esquecer no pós-processamento.

---

## 9. Critérios de aceite de um run

Descartar (e re-coletar) quando:

- `outcome != 'ok'` no `summary.json`;
- o log reportou **perda de contato** no `SLIDING` por trecho contínuo > 5 mm
  (`_SLIDE_LOST_BUDGET_M`) — a guarda só avisa, não aborta, de propósito, mas para
  textura um trecho longo sem contato invalida a passada;
- a força ao final do `HOLD` ficou fora de **1,00 ± 0,10 N**;
- o `impact_peak_n` do primeiro contato passou de **0,3 N** (a capa de silicone deve
  manter o pico bem abaixo disso; um pico alto denuncia que a capa saiu do lugar ou
  que o `crawl_v_ms` está destoando);
- qualquer taxel saturado ou travado em zero durante a passada;
- inclinação do plano medida > 1° e não corrigida por calço.

Registrar o motivo. Uma taxa de descarte acima de ~10 % não é azar: é a bancada
avisando que alguma coisa da lista do §6 não está firme.

---

## 10. Segurança

- Compressão **positiva** por convenção; `FORCE_ABORT_LIMIT_N = 15 N` cancela a
  medição; setpoint saturado em 10 N. Leitura envelhecida (> 0,5 s) aborta as fases
  de força.
- `SpeedFactor` forçado a 10 % durante a palpação.
- Durante o `SLIDING` a força **não é regulada**, só monitorada — é o comportamento
  pretendido. A guarda geométrica `_SLIDE_MAX_SINK_M` (10 mm) é o que impede um
  mergulho numa borda da amostra.
- E-STOP no cabeçalho da GUI. `STOP` em qualquer fase alivia o contato, sobe em +Z e
  volta para a HOME.
- **Ensaio a seco obrigatório** para cada nova janela de amostra, 20 mm acima da
  placa, antes do primeiro contato.

---

## 11. Pendências antes da primeira coleta

| # | Item | Onde |
|---|---|---|
| 1 | Modelar e imprimir a **ponteira H** | `cad/step/ponteira_H_finger_5x5.step` |
| 2 | Modelar, imprimir e vazar o **molde da capa** | `cad/step/molde_H_casca_A/B.step`, `molde_H_macho.step` |
| 3 | Atualizar **URDF + kinematics + gravity comp** para a nova pilha | 3 arquivos do §3.1 |
| 4 | Regerar meshes/inércias | `scripts/gen_tcp_meshes_from_step.py` |
| 5 | Decidir e montar o **sensor de força** (recomendado: `ft6`) | §3.5 |
| 6 | Re-medir `FORCE_NOISE_SIGMA_N` com a FA7155 | `constants.py` |
| 7 | Construir e nivelar a **placa de amostras** | §3.4 |
| 8 | Campo `sample_label` na `PalpationStart` (opcional, mas recomendado) | `touch_pack_msgs` |
| 9 | Re-ensinar HOME e apagar contato aprendido | GUI |

---

*Documento gerado a partir do estado do repositório em 03/09/2026 (`touch_pack`
`README.md`, `constants.py`, `tactile_explorer.py`, `urdf/touch_tool_tcp.urdf`,
`cad/step/`) e do protocolo de Gupta et al. 2021.*
