# Revisão cruzada do rehab_pack (agente 1, só leitura)

Linhas e situação conferidas na árvore de trabalho de 25/11/2026. "Aberto" quer
dizer: o código lido hoje ainda tem o defeito. Nada do rehab foi editado aqui.

| # | Sev. | Onde | Cenário | Correção | Situação |
|---|---|---|---|---|---|
| 1 | ALTA | `scripts/fake_bench.py:95` | o feedback falso não trazia o TestValue; o driver novo lia 0 de 10 pacotes | `pack_into('<Q', p, 48, 0x0123456789ABCDEF)` | corrigido pelo agente 2 |
| 2 | ALTA | `real_link.halt` (`real_link.py:~253`), `real_mirror.py:228-231` e `_travar` (`:134`) | `stop_motion` agora levanta quando o Stop não é confirmado (antes mandava Continue e engolia a falha). `halt()` só registra no log e o laço chama uma vez por transição: em ABORTADO o braço pode seguir andando sem ninguém saber | `halt()` devolver sucesso; na falha, `drv.emergency_stop()` e mostrar "parada não confirmada" | driver corrigido (XR-STOP no registro touch); lado rehab ABERTO |
| 3 | MÉDIA | `real_link.py:151,239` | `max_step_rad` é fixo (24°/s x 0,030 s x 2 = 1,44°). Um buraco de 70 ms no laço em jog de 100 % já vale mais que isso: StepTooLarge e trava (provado) | escalar pelo tempo desde o último ponto, limitado a JS_STALE_S | ABERTO |
| 4 | MÉDIA | `real_mirror.py:119-123` | qualquer mensagem PARADO limpa `halted`, sem Reset do operador (provado): a trava "dê Reset para liberar" se solta com a próxima republicação do estado | limpar só na transição ABORTADO para PARADO, ou num reset explícito | ABERTO |
| 5 | MÉDIA | `force_source.py:157-199` | o `zero` de hardware do ft_receiver publica `/load_cell/tared`=False; o rehab não escuta, e a tara calculada sobre o zero antigo continua valendo | assinar `/load_cell/tared` e chamar `_invalidate` | ABERTO (nenhuma referência ao tópico no pacote) |
| 6 | BAIXA | `force_source.py:188-213` | tara cancelada no meio da janela e outra pedida logo depois: o `finish()` da primeira fecha a janela da segunda (provado) | token por janela | ABERTO |
| 7 | BAIXA | `real_link.py:50-78` | um timeout isolado de feedback abortava dizendo "sim e real diferem" | tratar `CR10FeedbackTimeout` à parte | corrigido pelo agente 2 (tipo próprio) |
| 8 | BAIXA | `test/conftest.py:12-15` | skip de módulo zerava a coleta sob launch_testing (exit 5) | hook de importação | corrigido pelo agente 2 |

Conferido sem achado: nenhum assert enfraquecido; `prepare_servoj`/`halt`/`sync`
sem uso em produção além dos citados; dependências declaradas; a trava entre
pacotes é pega por todo publicador do braço; TunnelTask, disco cheio, jog e
saturação; nenhum deadlock encontrado.
