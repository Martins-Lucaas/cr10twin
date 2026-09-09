# Plano de análise — Palpação robótica (resposta aos revisores)

Coleta ENCERRADA em 09/09/2026: dois materiais (`glass`, `silicon`), seis
setpoints de 0,5 a 5 N, cinco repetições cada — 60 ciclos. Os dados estão em
`sensors/Data/glass/` e `sensors/Data/silicon/`, versionados no repositório.
Daqui para a frente é só análise; não haverá coleta adicional.

Leia primeiro a **§2**, que é o estado real dos dados — inclusive o artefato
do ciclo 5, que invalida todo número de repetibilidade tirado direto do
`summary.json`. Depois a **§3**, que é a lista do que rodar.

Escrito contra o código em `HEAD = 641e081` com a célula FA7155 (FT6).

---

## 1. Antes de ligar o braço: uma decisão que não dá para adiar

**A configuração descrita no artigo não existe mais, e não é recuperável do repositório.**

O `main.tex` é de 30/07/2026. O primeiro commit do repositório é de 26/08/2026. O código que produziu os números publicados está fora do controle de versão, e o commit mais antigo que existe já traz parâmetros diferentes dos que a Seção V descreve.

Pior: a **lei de controle mudou**. A Eq. (2) do artigo,

> Δx = clip( ρ·e·β / K̂ , ±ΔF_max/K̂ )

foi substituída em `f934d4e` (31/08/2026) por uma **rampa a velocidade constante**, em que a rigidez não é mais um ganho — ela entra apenas em dois limitadores que *encurtam* um passo fixo (teto por ΔF projetado e não-ultrapassagem). O docstring do `_qs_regulate` diz o motivo em uma frase: *"o overshoot dela era estrutural, porque K_est é uma EMA do trecho JÁ percorrido de um contato que enrijece e o passo err/K_est atravessava o alvo."*

Ou seja: a própria mudança de lei foi motivada exatamente pelo problema que o Revisor 2 aponta no item 3. Isso é uma boa notícia para a resposta, mas obriga a reescrever a Seção IV-C.

### Comparação parâmetro a parâmetro

| Grandeza | Artigo (config. "final") | `HEAD` hoje | Onde |
|---|---|---|---|
| Lei do passo | Δx = ρ·e·β/K̂, ρ = 0,7 | Rampa a v constante; K só limita | `_qs_regulate` |
| Banda τ_abs | 0,10 N (sweep) / 0,15 N (10 ciclos) | **0,02 N** | `HOLD_TOL_N` |
| Banda τ_pct | — | 1 % do setpoint | `HOLD_TOL_PCT` |
| Teto ΔF por passo | 0,7 N | **0,1 N** | `_QS_RAMP_DF_CAP_N` |
| Teto de passo | 20 µm | 100 µm (raramente morde) | `_QS_DX_MAX_M` |
| Não-ultrapassagem | — | 0,9·\|e\|/k_upper | `_QS_NO_CROSS_FRAC` |
| Rastejo pré-contato | 1 mm/s fixo | **v = 0,12/(T_halt·K)**, ~40 µm/s a 35,6 N/mm | `crawl_v_ms` |
| Limiar de contato | "pequeno" | 0,12 N | `CONTACT_ON_N` |
| Célula | HX711 + axial 100 kg, ~24 Hz | **FA7155, 400 N, ~400 Hz** | `force_sensor:=ft6` |
| σ da célula (cru) | 0,112 N | **0,0219 N** | `FORCE_NOISE_SIGMA_N` |
| σ visto pela malha | ~0,112 N | **0,0015 N** (One-Euro ligado) | `FORCE_CTRL_SIGMA_N` |
| Comprimento do TCP | 162,2 mm | **67,7 mm** | `tool_tcp_mm()` |

> **Verificar antes de escrever a resposta:** o artigo afirma ΔF_max = 0,7 N e deduz daí o teto de 20 µm (0,7 / 35,6 = 19,7 µm — internamente consistente). Mas o commit mais antigo do repositório tem `_QS_DF_MAX_N = 0,2` e `_QS_RELAX = 0,7`. É plausível que 0,7 seja **ρ transcrito como ΔF_max**, e que os 20 µm tenham sido derivados desse erro. Confira nos `params.json` das aquisições originais antes de citar qualquer um dos dois números na carta-resposta.

### O que isso implica

Recoletar hoje **não** reproduz a "configuração final" do artigo — produz uma **terceira** configuração. Só existem dois caminhos, e um deles está fora do seu escopo:

- **(A) Recoletar tudo no sistema atual e reescrever a Seção IV-C.** ✅ Recomendado, e é o que este plano detalha.
- **(B) Reverter o código à configuração publicada.** ❌ Impossível: esse código não está no git, e além disso violaria sua restrição de não mexer em software.

O caminho (A) tem uma vantagem que vale destacar na carta-resposta: **as duas campanhas passam a compartilhar uma única configuração**, o que responde o item 2 do Revisor 2 de forma completa, e não apenas atenuada.

---

## 2. O que foi coletado (09/09/2026) — campanha FECHADA

Decisão tomada: a análise segue **com estes dados**, sem coleta adicional.

**2 materiais × 6 setpoints × 5 repetições = 60 ciclos**, todos com um primeiro
contato independente. Modo MANUAL, sem escada, sem modulação, `depth_mm = 40`,
`hold_stable_s = 5,0 s`, célula FA7155.

| Material | Setpoints (N) | `hold_tol` | Runs |
|---|---|---|---|
| `glass` | 0,5 / 1 / 2 / 3 / 4 / 5 | **0,05 N** (uniforme) | `20260909_1128` a `_1143` |
| `silicon` | 0,5 / 1 / 2 / 3 / 4 / 5 | **0,02 N** em 0,5/1/2 · **0,05 N** em 3/4/5 | `20260909_1157` a `_1212` |

Local no repositório: `sensors/Data/glass/<SP>N/` e `sensors/Data/silicon/<SP>N/`,
um diretório de run por setpoint, cada um com os 5 ciclos.

### 2.1 O resultado que já está ganho: overshoot

Vem da fase DESCENDING e **não é afetado** pelo artefato da §2.2.

| | Artigo publicado | Coleta de 09/09 |
|---|---|---|
| Mediana | 0,35 N | 0,000 – 0,254 N |
| Ciclos acima de 1 N | 4 de 53 | **0 de 60** |
| Máximo | **7,9 N** | **0,294 N** |

27× de redução no pior caso, com mais contatos que a campanha original. A
previsão da §4 (o orçamento de impacto do `crawl_v_ms`) se confirma.

### 2.2 ⚠️ ARTEFATO DO CICLO 5 — corrigir na análise antes de qualquer número

**Presente em 12 de 12 runs.** O recuo de fim de run começa *antes* de a fase
deixar de ser HOLD, e a descarga a zero cai dentro da janela de medição de 5 s.

Assinatura, em `glass/5N` (médias por segundo do HOLD):

```
ciclo 4:  4.95 5.06 5.01 5.00 4.99 4.98 4.97 4.96
ciclo 5:  4.93 5.06 5.02 5.00 4.99 4.98 4.98 4.97 2.28   ← descarga
```

E as médias da janela por ciclo:

```
glass/5N    4.978  4.978  4.983  4.988  | 4.569
silicon/2N  1.995  1.986  1.981  1.987  | 1.852
```

Os ciclos 1–4 estão cravados; o 5 sempre cai, e sempre para baixo. O
`min_n` da janela do ciclo 5 é **−0,002 N** (`glass`) — a força chega a zero
dentro dela.

**Consequência: todo número de repetibilidade que sai direto do
`summary.json` está errado.** O CV de ~3,6 % é artefato, não física — os
ciclos 1–4 dispersam 5–10 mN, ou seja, na mesma ordem dos 4–17 mN publicados.

**Receita da janela limpa** (a aplicar na outra máquina, sobre `samples.csv`):

1. Selecione as amostras do ciclo com `phase == 2` (HOLD; ver `PHASE_CODES`).
2. Percorra de trás para frente descartando enquanto `|F − setpoint| > 5·τ`.
3. Da amostra restante mais recente, tome os últimos 5 s.
4. Média e σ dessa janela são a colocação e o ruído assentado do ciclo.

O passo 2 é o que remove a descarga. Sem ele, o ciclo 5 de cada run tem de ser
descartado — 12 dos 60 ciclos, e a repetibilidade cai para n = 4.

### 2.3 Duas limitações que a campanha de 2 materiais impõe

**(a) A regressão do quantum morreu.** `ΔF_q = K·Δx_min` precisa de uma faixa
de K para virar medida; com dois pontos, uma reta passa por qualquer par e não
testa nada. O Revisor 2, item 4, fica **sem resposta experimental** — resta
declarar o quantum como limite superior, exatamente como o artigo já faz, e
explicar por que não foi possível medi-lo.

Há um resto aproveitável: os runs de silicone a `hold_tol = 0,02` contra os a
`0,05` são dois pontos de uma varredura de banda acidental. Se algum dos de
0,02 N mostrar stall (janela fechando fora da banda) e os de 0,05 N não,
isso limita `Δx_min` entre os dois valores. É fraco, mas é o que há.

**(b) O phantom de tecido mole não foi atendido.** O Revisor 1 pediu
explicitamente. Se o silicone é Dragon Skin 10A de 5 mm, ele é ~10× mais
rígido que tecido mole, e acima de ~1,5 N o que se mede é a base rígida: 5 N a
~1,5 N/mm pedem 3,3 mm de indentação numa camada de 5 mm. **Isto tem de ser
declarado na conclusão**, não omitido — ver §9.

### 2.4 A não-uniformidade da banda no silicone

Três runs de `silicon` (0,5 / 1 / 2 N) usaram `hold_tol = 0,02 N` e os outros
três `0,05 N`. É a mesma ressalva do Revisor 2, item 2 — parâmetros diferentes
dentro da mesma campanha. Como não haverá recoleta, a saída é **reportar as
duas metades separadamente e dizer qual banda valeu em cada uma**, nunca
juntá-las numa estatística só.

---

## 3. Análise a fazer (na outra máquina)

Tudo abaixo é processamento dos CSVs já coletados — nenhuma bancada.

| # | O quê | Entrada | Responde |
|---|---|---|---|
| 1 | Repetibilidade com a janela limpa da §2.2 | `samples.csv` | Rev. 2/2 |
| 2 | Colocação e viés vs setpoint (ajuste afim, R²) | idem | Rev. 1 |
| 3 | Ruído assentado σ por setpoint | idem | Rev. 1 |
| 4 | **K de cada material**, do ramo de carga | idem, via FK sobre `q1..q6` | Rev. 1, Rev. 2/1 |
| 5 | Overshoot: distribuição por material e setpoint | `summary.json` (já correto) | Rev. 1, Rev. 2/3 |
| 6 | `T_halt` da regressão pico × K | itens 4 e 5 | Rev. 2/3 |

**O item 4 é pré-requisito dos itens 5 e 6, e é o que decide a campanha
inteira:** se vidro e silicone derem o mesmo K (ambos lendo a compliance da
pilha, ~35 N/mm), então há **um** nível de rigidez e não dois, e a resposta ao
Revisor 1 não se sustenta. Faça esse antes de escrever qualquer coisa.

Lembre de usar a **FK sobre `q1..q6`** (5 decimais em radianos, quantum de
1×10⁻⁵ rad) e não a coluna `tcp_z` (5 decimais em metros, quantum de 10 µm) —
ver §5.2.

## 4. Overshoot de primeiro contato (Revisor 1; Revisor 2, item 3)

### 4.1 O que mudou, e a previsão a testar

O artigo mede mediana 0,35 N, com 4/53 ciclos acima de 1 N e um pico de 7,9 N, e atribui isso ao atraso do pipeline com um rastejo de **1 mm/s fixo**.

O código atual **orça explicitamente esse transiente**: `crawl_v_ms` resolve `v = CONTACT_ON_N/(T_halt·K)` justamente para que o pico do primeiro impacto pare no limiar de contato. A previsão é forte e falsificável:

> **O pico do primeiro contato escala LINEARMENTE com o K real**, porque a
velocidade de rastejo é fixa (`crawl_v_ms(_K_RIGID_REF_NM)` = 50,4 µm/s,
independente da amostra): pico = v·T_halt·K = 4,29×10⁻⁶·K (K em N/m).
Com dois materiais isso dá dois pontos — suficiente para uma estimativa de
`T_halt`, insuficiente para testar a linearidade.

Isso é uma queda de ~3× na mediana e, mais importante, **elimina o mecanismo da cauda**: o pico deixa de escalar com a velocidade porque a velocidade passou a escalar com 1/K.

**Evidência preliminar já no repositório:** o run `sensors/Data/TOUCH/20260908_114619` (FA7155) registra em `summary.json` `overshoot_n = -0,012` na fase DESCENDING, com `max_n = 1,088` contra um setpoint de 1,1 N — ou seja, **nenhum overshoot**. E o `final_window` do HOLD tem `std_n = 0,005 N`, contra os 0,020 N publicados.

### 4.2 Como medir T_halt sem instrumento novo

O `_STREAM_HALT_LAT_S = 0,085 s` do código é, por confissão do próprio comentário, *"uma transposição, não uma medição direta"* — foi medido no executor da onda, não nesta cadeia. Medi-lo é exatamente o que o Revisor 2 pede no item 3.

**Método direto, usando só os dados que você já vai coletar.** Durante o primeiro contato o braço rasteja a `v` conhecida e constante. Do `samples.csv`:

1. Ache a amostra em que `force_net_n` cruza `CONTACT_ON_N` pela primeira vez no ciclo → instante do comando de halt.
2. Ache o pico de `force_net_n` na janela seguinte → `ΔF_pico`.
3. `T_halt = ΔF_pico / (v · K)`, com `v` = `crawl_v_ms(K)` e `K` medido da própria curva de carga.

Com dois materiais e dois K diferentes, **o mesmo T_halt tem de sair das duas contas**. Se sair, o modelo `v·T_halt·K` está validado e a explicação do overshoot deixa de ser conjectura; se não sair, você descobriu que o mecanismo é outro — e isso também é um resultado publicável.

**Decomposição (opcional, fortalece a discussão):**

- *Atraso do filtro:* o `samples.csv` traz `lc_voltage_raw_v` (força **crua**, sem One-Euro) na mesma linha que `force_net_n` (filtrada). Correlação cruzada entre as duas colunas dá o atraso do filtro diretamente. O código estima ~80 ms a 2 N/s.
- *Idade da pose:* coluna `pose_age_ms`, já gravada por amostra.
- *Transporte firmware→ROS:* `lc_t_us` é o carimbo do firmware, o único tempo da linha que não passou por dois saltos ROS. Regrida `t_unix` contra `lc_t_us` — a dispersão do resíduo é o jitter da cadeia.
- *Metade mecânica (comando→braço):* `ros2 run touch_pack latency_probe --ros-args -p direction:=sim_to_real -p robot_ip:=192.168.5.2 -p duration_s:=20.0`, com o braço em MIRROR sendo movido pela GUI.

### 4.3 O que reportar

Distribuição do pico por material (mediana, IQR, máximo, contagem acima de 1 N), o `T_halt` estimado pelas quatro vias, e a comparação lado a lado com a campanha publicada. Se a previsão se confirmar, a frase da conclusão deixa de ser "trabalho futuro" e vira "resolvido e medido".

---

## 5. O force quantum (Revisor 2, item 4)

### 5.1 Por que hoje é só um limite superior

`palpation_logger.py:618` grava `tcp_x/y/z` com `f'{v:.5f}'` — metros com 5 decimais, quantum de **10 µm**. Todo passo executado no hold registra exatamente um quantum, então Δx_min não é observável nesse canal. É exatamente o que o artigo diz, e está correto.

### 5.2 Duas saídas que **não** exigem mexer no código

**(a) Use as juntas, não o TCP.** A linha 615 grava `q1..q6` com 5 decimais **em radianos** — quantum de 1×10⁻⁵ rad. Refazendo a FK offline a partir dessas colunas, a resolução efetiva em z do TCP fica em ~3–7 µm por junta, e como várias juntas se movem juntas as quantizações não se alinham: o grid composto é bem mais fino que os 10 µm da coluna `tcp_z`. Continua sendo um limite, mas **2 a 3× mais apertado**, e de graça.

**(b) Transforme K na variável independente — esta é a boa.** Δx_min é uma propriedade do *braço*, não do contato. Logo:

> ΔF_q = K · Δx_min

~~Com quatro materiais cobrindo 0,3 a 35 N/mm, colete o menor degrau de força executado em cada material e plote contra K.~~ **Não é mais possível** — ver §2.3(a). Com dois pontos a regressão não testa nada. Se o modelo vale, os pontos caem numa **reta pela origem cujo coeficiente angular é Δx_min** — medido, não limitado. Um ajuste sobre duas décadas de K é bem condicionado; sobre um único ponto (o que o artigo tem) é impossível por construção.

**É a mesma campanha que o Revisor 1 já pediu.** O item 4 do Revisor 2 sai de graça do item que o Revisor 1 exigiu.

### 5.3 A varredura de banda — NÃO REALIZADA

Ficou fora da campanha. Era o experimento que mediria `Δx_min` pela fronteira de stall, com K fixo e `hold_tol` varrido em 0,02 / 0,04 / 0,07 / 0,10 / 0,15 N. A fronteira em que os stalls aparecem dá `Δx_min = τ_crítico / K` — uma segunda medida, por um mecanismo completamente diferente do de 5.2. Se as duas concordarem, o número é sólido.

> ⚠️ **Risco a antecipar:** com τ = 0,02 N e K = 35 N/mm, um Δx_min de 3 µm daria ΔF_q ≈ 0,105 N, cinco vezes a banda — o hold **não fecharia**. A não-ultrapassagem (`0,9·|e|/k_upper`) encolhe o passo perto do alvo e provavelmente salva a convergência, e o run de 08/09 fechou em 0,02 N. Mas se M1 der muitos `timeout` de HOLD, **não é bug: é o quantum aparecendo**, e a varredura 3.3 é justamente o instrumento para documentá-lo. Rode a 3.3 **antes** da campanha principal de M1 e escolha o τ de trabalho a partir dela.

---

## 6. Repetibilidade e configuração única (Revisor 2, item 2)

**Parcialmente atendido.** A campanha de 09/09 é única e homogênea em tudo,
menos na banda do silicone (§2.4). Mas são **5 repetições**, não as dez que o
revisor pediu textualmente — e, se o artefato do ciclo 5 for tratado por
descarte em vez da janela limpa, sobram 4.

Duas formas honestas de escrever isso:

- Reportar n = 5 e dizer que a campanha anterior de dez ciclos foi substituída
  por uma configuração única, que era o ponto do revisor;
- ou reportar o σ com o intervalo de confiança que n = 5 permite, deixando
  explícito que a incerteza sobre o próprio σ é de ordem ±35 %.

O formato do artigo continua bom: média por ciclo relativa à média do próprio
run, box + pontos sobrepostos, σ intra-run impresso, e o *bias* mantido
separado da *dispersão*.

---

## 7. Análise — colunas e receitas

Tudo sai de `sensors/Data/<MODO>/<run_id>/`:

| Arquivo | Uso |
|---|---|
| `samples.csv` | Série principal. Colunas: `t_rel_s, t_unix, mode, cycle, phase, setpoint_n, force_net_n, lc_seq, lc_t_us, lc_voltage_raw_v, q1..q6, tcp_x/y/z, pose_age_ms, taxel_*` |
| `summary.json` | Já calcula por ciclo e por fase: `overshoot_n`, `final_window{mean,std,min,max}`, `duration_s` |
| `params.json` | Configuração do run. **Confira `hold_tol_n`, `force_sensor`, `tool_tcp_mm` em todo run** |

**Receitas:**

- **Placement / bias:** média de `force_net_n` na `final_window` do HOLD − `setpoint_n`. Já está no `summary.json`.
- **Ruído assentado:** `final_window.std_n`.
- **Repetibilidade:** σ das médias por ciclo dentro do run; CV = σ/setpoint.
- **Overshoot:** `summary.json → cycles[i].DESCENDING.overshoot_n`, e confira contra o `max_n` da mesma fase.
- **K:** regressão de `force_net_n` contra a indentação, com a indentação vinda da **FK sobre `q1..q6`**, não de `tcp_z`. Ajuste acima de 1 N em M1/M2 (para comparar com o publicado) **e** ajuste a curva inteira em M3/M4, onde o "toe" é o regime de operação.
- **Δx_min:** menor degrau executado por material → reta contra K (§5.2); e fronteira de stall (§5.3).
- **`lc_seq`:** confira continuidade. Salto = amostra perdida; descarte a janela.

---

## 8. Segurança

A malha atual é mais conservadora que a do artigo em todos os eixos, e nada nela deve ser tocado:

- Teto duro 15 N (`FORCE_ABORT_LIMIT_N`); parada com margem em 12 N (`_FORCE_SAFE_LIMIT_N`), com alívio ativo (`_relieve_contact`) antes do abort.
- Setpoint máximo pela GUI: 10 N.
- FA7155: fundo de escala 400 N, sobrecarga segura 300 % (1200 N). Os 15 N são 3,75 % do FS — folga enorme.
- E-STOP com trava e `/palpation/freeze` (congela no lugar, sem homing).

**Para os phantoms moles**, o risco não é força e sim **profundidade**: o abort de 15 N nunca dispara a 0,3 N/mm (precisaria de 50 mm de indentação). O que protege ali é o `depth_mm` que você configurar. **Defina-o pela espessura do phantom, não pelo setpoint**, e confirme que o `budget` aborta antes de a ponteira encontrar o fundo rígido.

---

## 9. O que muda no artigo

| Seção | Ação |
|---|---|
| IV-C | **Reescrever.** Substituir a Eq. (2) pela rampa a velocidade constante e pelos dois limitadores. Explicar que a lei anterior foi trocada porque seu overshoot era estrutural — isso *é* a resposta ao item 3 do Revisor 2 |
| IV-B | Documentar `crawl_v_ms`: a velocidade de rastejo é derivada do orçamento de impacto, não escolhida |
| V-B | Uma campanha, uma configuração. Remover a ressalva |
| VI-C | Overshoot medido com T_halt medido por quatro vias independentes |
| VI (novo) | Dependência da rigidez: quatro materiais, 100× em K |
| VI-D | Quantum **medido** pela regressão contra K, com a fronteira de stall como confirmação |
| Tabela I | Refazer inteira com os números novos |
| VII | Separar o que foi demonstrado do que continua futuro. **Limitar a validação a bancada** e condicionar uso em tecido a phantoms biologicamente representativos, como o Revisor 1 pede |

**Sobre a conclusão** (pedido explícito de ambos): a formulação honesta é que a validação cobre palpação robótica em bancada sobre contatos de 0,3 a 35 N/mm, incluindo um phantom de tecido mole *mecanicamente* representativo, e que aplicação clínica permanece condicionada a validação em phantoms *biologicamente* representativos — que é coisa diferente e você não terá.

---

## 10. Ordem de execução (análise)

1. **K de vidro e silicone** (§3, item 4). Decide se a campanha tem uma ou
   duas rigidezes — e portanto se responde ao Revisor 1. Antes de tudo.
2. Repetibilidade com a janela limpa da §2.2, por material e por setpoint.
3. Colocação, viés e ruído assentado.
4. Overshoot por material; `T_halt` da regressão pico × K.
5. Refazer a Tabela I com os números novos.
6. Reescrever IV-C (a lei de controle mudou — §1) e a conclusão (§9).

**Pendências que não são análise:**

- Confirmar nos `params.json` das aquisições ORIGINAIS qual era o ΔF_max —
  0,7 N ou 0,2 N (§1). A carta-resposta vai citar esse número.
- Decidir a redação sobre o phantom mole (§2.3b) e sobre o quantum que não
  pôde ser medido (§2.3a).
