# Na fase SLIDING a profundidade fica travada e a força varia livremente

No modo SLIDE, a variação da força normal ao longo do curso é o sinal medido: a textura da superfície. Por isso `_phase_sliding` (`src/touch_pack/touch_pack/tactile_explorer.py`) trava a **profundidade** na indentação que o HOLD deixou (`_Z_CORR_GAIN * depth_err`) e não fecha malha de força. Regular a força ali apagaria justamente a grandeza que o ensaio existe para medir.

## Consequences

- Em runs SLIDE, `mae_vs_setpoint_n`, `std_n` e as excursões de força do SLIDING são dados, não erro de rastreamento.
- Perda de contato real (F→0 por curso longo) é problema de montagem/alinhamento: a resposta é `probe_align`, não realimentação de força.
- A regulação de força por realimentação continua valendo em DESCENDING e HOLD.

## Considered Options

- Malha de força `Δx = relax·(F* − F)/K_est` durante o SLIDING. Foi rejeitada porque achata a textura (veio de tratar um MAE de 0,388 N como falha de controle).
