# Na fase SLIDING a profundidade fica travada e a força varia livremente

No modo SLIDE, a variação da força normal ao longo do curso é o sinal medido: a textura da superfície. Por isso `_phase_sliding` (`src/touch_pack/touch_pack/tactile_explorer.py`) trava a **profundidade** na indentação que o HOLD deixou (`_Z_CORR_GAIN * depth_err`) e não fecha malha de força. Regular a força ali apagaria justamente a grandeza que o ensaio existe para medir.

## Consequences

- Em runs SLIDE, `mae_vs_setpoint_n`, `std_n` e as excursões de força do SLIDING são dados, não erro de rastreamento.
- Perda de contato real (F→0 por curso longo) é problema de montagem/alinhamento: a resposta é `probe_align`, não realimentação de força.
- A regulação de força por realimentação continua valendo em DESCENDING e HOLD.

## Considered Options

- Malha de força `Δx = relax·(F* − F)/K_est` durante o SLIDING. Foi rejeitada porque achata a textura (veio de tratar um MAE de 0,388 N como falha de controle).

## Emenda (23/11/2026): malha lenta de força opcional, `slide_force_hz`

**Decisão.** `_phase_sliding` soma à trava de profundidade uma correção lenta `d`, com `ḋ = clip(2π·slide_force_hz/K · (F* − F))`. O parâmetro vale 0,5 Hz por padrão e 0,5 Hz é o teto; `0` volta ao comportamento original deste ADR (profundidade travada, força livre). Feita a pedido do usuário; a trava de profundidade continua existindo, agora em torno do plano corrigido.

**Justificativa.** Num curso longo o plano declarado nunca coincide com a amostra: em contato rígido (5–30 N/mm medidos) décimos de milímetro de desalinhamento viram newtons de deriva, e a célula sai da faixa do ensaio. A malha corrige essa tendência (abaixo de ~0,5 Hz) e deixa passar a textura, que está acima de 2 Hz nas velocidades usadas. Medido em 23/11/2026: atraso comando→pose ~40 ms mais filtro ~50 ms; a 0,5 Hz sobra margem de fase acima de 55° mesmo com K errado por 2×.

**Limites fixos.** Para dentro 0,1 mm/s e no máximo 0,3 mm abaixo do plano do HOLD; para fora 0,5 mm/s e no máximo 1 mm. `K` é a EMA do HOLD com piso de 5 N/mm, e sem estimativa a malha fica inerte. Saturação, força excessiva e força lateral seguem com as guardas próprias.

**Consequências.**

- Conteúdo de força abaixo de ~0,5 Hz no SLIDING passa a ser **regulado**, não medido: ondulação longa da amostra (comprimento maior que `v/0,5 Hz`, 20 mm a 10 mm/s) sai atenuada em `force_net_n` e aparece em `tcp_z`.
- Para medir forma ou ondulação por força, rode com `-p slide_force_hz:=0.0`. Só nesse caso vale a primeira consequência acima (MAE e excursões do SLIDING como dado puro).
- Runs com e sem a malha não são comparáveis em baixa frequência. O valor do parâmetro não vai para o `params.json`; o log do explorer (`malha de força … Hz`) é o registro.
- A rejeição em "Considered Options" continua: o que foi descartado é a malha rápida de força, que achata a textura.
