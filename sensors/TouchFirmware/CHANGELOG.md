# Firmware do sensor tátil 5×5 + HX711 — correções da cadeia USB

Ponto de partida: a versão que rodava na bancada em 14/08/2026. O protocolo de
linha (`ADC,…`, `RA,…`, `SA,…`, `CN_*`, `FORCE,…`) e toda a lógica neural
(Izhikevich, template sináptico, cuneiformes) estão **inalterados** — o
`touch_pack/touch_source.py` continua valendo sem mudança.

A evidência que motivou a revisão já existia no lado do PC:
`touch_source.py:344` mantém um contador `frames_bad` com o comentário *"Frame
truncado — o firmware perdeu bytes na saída"*. Esse contador existe porque
dispara.

## O orçamento de dados

Relógio: HSE 8 MHz → PLL(M=8, N=336, P=2) → **168 MHz**. APB1 = 42 MHz, clock
dos timers = 84 MHz. TIM6 com prescaler 83 → 1 MHz; período 199 → trigger a
cada **200 µs**. Cada trigger converte as 5 colunas → ISR a 5 kHz → um frame
5×5 completo a cada **1 ms**.

| Fonte | Taxa | Bytes/linha | Vazão |
| --- | --- | --- | --- |
| `ADC,…` | 1000/s | ~143 | **~143 KB/s** |
| `CN_MM/RA/SA` | até 3000/s | ~18 | até ~54 KB/s |
| `FORCE,…` | 100/s | ~35 | ~3,5 KB/s |

Contra isso, o dreno enviava no máximo 64 bytes por transferência, uma de cada
vez.

---

## [1] `packet[]` sobrescrito com a transferência em voo — corrupção

**O defeito principal.** `CDC_Transmit_FS` apenas guarda o ponteiro
(`USBD_CDC_SetTxBuffer`) e retorna; no OTG_FS do F7 os bytes só são empurrados
para a FIFO depois, dentro da interrupção. O buffer precisa ficar intacto até
`TxState` voltar a zero.

A versão antiga enchia `packet` **antes** de saber se podia transmitir:

```c
while (temp_tail != usb_head && len < USB_PACKET_SIZE)
    packet[len++] = usb_tx_buffer[temp_tail];   // escreve primeiro
...
if (CDC_Transmit_FS(packet, len) == USBD_OK)    // pergunta depois
    usb_tail = temp_tail;
```

Como `usb_tail` já havia avançado na chamada bem-sucedida anterior, a iteração
seguinte do laço principal sobrescrevia `packet` com os bytes **seguintes**
enquanto a transferência anterior ainda o estava lendo. O host recebia as duas
metades misturadas. Em seguida `CDC_Transmit_FS` devolvia `USBD_BUSY`,
`usb_tail` não avançava, e os mesmos bytes eram reenviados — **corrompidos e
duplicados**.

**Correção:** `usb_tx_ready()` (que consulta `hcdc->TxState`) é chamado
*antes* de tocar em `packet`.

## [2] Ring buffer sem seção crítica

`usb_buffer_write` tem dois produtores concorrentes:

| Contexto | Chamadas |
| --- | --- |
| ISR (`DMA2_Stream0` → `HAL_ADC_ConvCpltCallback` → `update_taxels`) | `send_adc_frame`, `CN_*` |
| laço principal | `process_spikes`, `FORCE`, boot, `STAT` |

O par `usb_tx_buffer[usb_head] = c; usb_head = next;` não é atômico: uma ISR
caindo entre as duas instruções escrevia no mesmo índice e tinha o seu byte
sobrescrito — caracteres de uma linha apareciam no meio de outra.

**Correção:** `irq_save()`/`irq_restore()`, que preservam o `PRIMASK` do
chamador em vez de chamar `__enable_irq()` cego. Isso importa porque a leitura
do HX711 também usa seção crítica; a versão antiga reabilitaria as
interrupções ao sair da região interna.

## [3] Escrita all-or-nothing

A versão antiga copiava byte a byte e dava `break` ao encher, **cortando a
linha no meio**. Agora a mensagem só entra se couber inteira; se não couber é
descartada por completo e contabilizada. Frame que chega, chega íntegro.

## [4] Chunk de transmissão: 64 → 512 B

`USB_PACKET_SIZE` (64) é o *wMaxPacketSize* do endpoint bulk em Full Speed —
não é o teto de uma **transferência**. `CDC_Transmit_FS` aceita centenas de
bytes e a própria pilha os fatia em pacotes de 64. Usar 64 como tamanho de
transferência limitava a vazão a ~1 transferência por polling do host.

Novo `USB_TX_CHUNK = 512` (8 pacotes por transferência) e ring de 8192 B.
Transferências múltiplas de 64 são encurtadas em 1 byte para dispensar o
pacote de comprimento zero (ZLP) que fecharia a transferência.

## [5] Descarte deixou de ser mudo

Nova linha periódica `STAT,drop=<n>,t=<us>` (1 Hz, `USB_STAT_PERIOD_MS`).
Antes, uma perda no firmware e uma perda no link eram indistinguíveis do lado
do PC. `touch_source.py` ignora prefixos desconhecidos, então a linha é
inofensiva para quem não a consome.

## [6] HX711: temporização do PD_SCK

Datasheet: nível alto tem mínimo de **0,2 µs** (T2) e **máximo de 60 µs** —
acima disso o chip entra em power-down. Dois `HAL_GPIO_WritePin` consecutivos
a 168 MHz davam ~40 ns, **abaixo do mínimo**: leituras podiam sair com bits
errados.

Além disso, os 25 pulsos rodavam dentro de **um único** `__disable_irq()`,
bloqueando a ISR do ADC de 5 kHz. Deixá-los completamente livres também não
serve: uma ISR longa (o `update_taxels` chega a montar linhas) seguraria o SCK
alto por mais de 60 µs.

**Correção:** `hx711_pulse()` — atraso por contador de ciclos do DWT (60
ciclos ≈ 0,36 µs) e janela sem interrupção **por pulso**, não por leitura.

## [7] HX711: gate de amostragem truncado

`(uint32_t)(1000.0f / 80.0f)` = **12**, não 12,5 → amostragem real a 83,3 Hz
enquanto o `hx711_alpha` do IIR era calculado para 80 Hz. Quem dita a cadência
é o próprio HX711 (~80 SPS pelo pino RATE); o gate por milissegundo foi
removido e a leitura passou a acontecer quando `HX711_IsReady()` avisa.

`HX711_ReadAverage` tinha o mesmo tipo de erro — o laço gastava a iteração sem
ler quando o chip não estava pronto, então pedir 200 amostras rendia bem menos
e a tara saía de uma média mais pobre que a pedida. A função foi substituída
pela máquina de estados de `[12]`, que conta amostras de verdade.

## [8] Timestamp de 32 bits dava a volta a cada 71,6 min

TIM2 conta a 1 MHz num registrador de 32 bits → volta em 4294,97 s. Gravações
longas viam o tempo voltar a zero. `micros64()` estende para 64 bits por
software (chamado a cada frame, muito acima da taxa mínima de uma vez por
volta). O valor continua sendo microssegundos decimais, então os regex
`t=(\d+)` do PC seguem casando.

**Fica DESLIGADO por padrão (`TS_64BIT 0`).** `touch_pack_msgs/msg/TouchFrame.msg`
declara `uint32 t_us`, e a GUI republica cada frame ADC nesse campo. O rclpy
rejeita valor acima de `2^32-1` com `AssertionError` — que
`palpation_gui._publish_tactile_line` **não captura** (ela trata só
`ValueError`/`IndexError`). Ligar isto faria a republicação em ROS morrer
depois de 71,6 min de uptime do MCU: quebra silenciosa e tardia, pior que o
wraparound que se queria consertar. Verificado na bancada — a atribuição de
`4294967396` levanta a exceção.

O parser de texto (`touch_source.py`) aceita o valor largo sem mudança; quem
não aceita é a mensagem ROS. Para ligar, nesta ordem: alargar `t_us` para
`uint64` no `.msg`, reconstruir `touch_pack_msgs`, conferir os consumidores de
`touch_t_us` no `palpation_logger`, e só então trocar para 1.

## [9] `snprintf` de 25 inteiros dentro da ISR

`send_adc_frame` rodava a 1 kHz **dentro da ISR do ADC**, que dispara a cada
200 µs. Substituído por um formatador decimal direto; a saída é byte-a-byte
idêntica (há teste comparando com o `snprintf` original).

## [10] Retorno de `snprintf` não clampeado

`snprintf` devolve o tamanho que a string **teria**, que pode passar do
buffer. Esse valor ia direto para `memcpy`/`usb_buffer_write`, lendo além do
array. `snprintf_len()` clampa. `process_spikes` também passou a verificar
`batch_count` antes do `memcpy` — a versão antiga nunca comparava com o
tamanho de `batch_msg`.

## [11] `dwt_delay_cycles` podia travar para sempre — **regressão minha**

Introduzido pela correção `[6]` e corrigido depois de uma sessão de
diagnóstico na bancada. Sintoma: **o dispositivo enumera e não envia um byte
sequer, nem o banner de boot.**

O contador de ciclos do DWT não liga sozinho. No Cortex-M7 o bloco é CoreSight
e tem *Lock Access Register*: sem escrever `0xC5ACCE55` em `DWT->LAR`, as
escritas em `DWT->CTRL` são **ignoradas** e `CYCCNT` fica em zero. Com o
contador parado,

```c
while ((DWT->CYCCNT - start) < cycles) { __NOP(); }
```

nunca termina. Esse atraso é chamado por `hx711_pulse()`, que roda na tara,
que rodava no boot **antes** do laço principal. Resultado: o firmware travava
antes de transmitir qualquer coisa, enquanto o USB seguia enumerando — porque
enumeração é interrupção e não depende do laço.

Três defesas, e nenhuma sozinha bastava:

1. escrever a chave no LAR antes de habilitar o `CYCCNT`;
2. **verificar** que o contador anda, e cair num laço de reserva por `NOP`
   quando não anda;
3. teto de iterações no caminho do DWT, para o caso de o contador congelar no
   meio (depurador anexado).

## [12] O boot bloqueava no HX711 antes de o USB falar

O defeito estrutural que tornou `[11]` tão difícil de diagnosticar — e que
existia desde a versão original.

A tara rodava **antes** do `while(1)`, somando 200 leituras de uma vez.
Enquanto durava, `usb_buffer_process()` não era chamado: o banner de boot
ficava no buffer sem sair. Se a célula não respondesse, o mudo durava toda a
janela de espera; se algo lá dentro travasse, o mudo era permanente. Do lado
do PC os três casos — placa morta, célula ausente, firmware travado — são
**indistinguíveis**.

Agora a tara é uma máquina de estados no laço (`HX711_TareStep`): uma amostra
por volta, só quando o chip avisa, com desistência por tempo
(`HX711_TARE_WINDOW_MS`) que reporta `HX711 TARE TIMEOUT,<obtidas>,<pedidas>`.
O USB fala desde o primeiro milissegundo, aconteça o que acontecer com a
célula — inclusive ela não estar conectada.

`HX711_Read` também passou a ter timeout em **tempo** (`HAL_GetTick`), não em
contagem de iterações: o teto antigo de 1.000.000 de voltas valia uma duração
diferente conforme o clock, e ninguém sabia dizer qual.

**Regra que vale para o futuro:** nada entre o `MX_USB_DEVICE_Init()` e o
`while(1)` pode bloquear. Um dispositivo que não consegue falar não consegue
ser diagnosticado.

---

## NÃO corrigido, de propósito

### `select_row()` ignora o parâmetro `row`

```c
void select_row(uint8_t row) {   // `row` nunca é lido
    ... escreve row_mask ...
    row_mask = row5(row_mask);   // roda uma máscara estática
}
```

A máscara escrita e o `row_idx` usado para indexar `taxels[]` avançam de forma
independente. Traçando o boot: `row_mask` começa em `0b11110` (linha 0);
`select_row(0)` no `main` escreve essa máscara e roda para `0b11101`; a
primeira ISR chama `select_row(0)` e escreve `0b11101` — **linha 1** — mas
guarda os dados como linha 0.

Amarrar a máscara a `row` mudaria qual taxel físico responde por cada índice
do frame 5×5, invalidando a calibração e todas as gravações existentes. É uma
correção de bancada, com o sensor na mão e um estímulo conhecido, não de
escrivaninha. `row_masks[ROWS]` está declarado e nunca usado — provavelmente
era esta a intenção original.

### Variáveis mortas

`hx711_ready`, `grams`, `last_force_time`, `taxels[].I`, `I_inh` global
(sombreado por um local), `g_syn_RA`/`g_syn_SA` e `I_inh_buffer`/`I_inh_index`
não são lidos. Não são defeitos e removê-los alargaria o diff sem necessidade.

### Vazão residual

Mesmo com o chunk de 512 B, ~143 KB/s de ASCII sobre CDC FS é exigente. Se o
`STAT,drop=` continuar subindo, as saídas em ordem de custo são: reduzir a
taxa de frames ADC, ou trocar a linha ASCII por binário (25×`uint16` + tempo
≈ 58 B contra ~143 B) — esta última exige mudar o parser do PC junto.

---

## Testes

```
cd sensors/TouchFirmware
make check    # sintaxe, -Wall -Wextra, zero avisos
make test     # 13 grupos contra stubs da HAL em test/stub/
```

Cobrem: formatadores decimais (incluindo `UINT64_MAX`), igualdade byte-a-byte
do frame ADC com o `snprintf` original, escrita all-or-nothing e o contador de
descarte, ausência de escrita em `packet[]` com transferência em voo, ausência
de transferências múltiplas de 64, wraparound do ring, monotonicidade do
timestamp através de duas voltas do TIM2, clamp do `snprintf`, preservação dos
dados enquanto o CDC não está configurado, **terminação do atraso do HX711 nos
três estados possíveis do `CYCCNT`** (parado, normal, congelado no meio) e
**desistência da tara sem célula conectada**.

**Não substituem a bancada.** Timing do HX711, comportamento real do endpoint
USB e a taxa de frames medida pelo PC só se verificam com o hardware ligado —
o número a acompanhar é `frames_ok` vs `frames_bad` no `touch_source.py`, ao
lado do `STAT,drop=`.
