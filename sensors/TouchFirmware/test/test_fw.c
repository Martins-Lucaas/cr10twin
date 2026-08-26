/* Harness de host: inclui o firmware inteiro e exercita a cadeia USB. */
/* O LAR do DWT vive num endereco absoluto do Cortex-M7; no PC isso seria um
 * acesso invalido, entao o harness aponta o macro para uma variavel. */
extern volatile unsigned int stub_dwt_lar;
#define DWT_LAR_ADDR stub_dwt_lar
#define main firmware_main
#include "../main.c"
#undef main

#include <assert.h>
#include <stdio.h>

/* ---- implementação dos stubs ---- */
static GPIO_TypeDef g_a, g_b, g_c, g_f;
GPIO_TypeDef *GPIOA = &g_a, *GPIOB = &g_b, *GPIOC = &g_c, *GPIOF = &g_f;
uint32_t stub_tim_counter = 0;
volatile unsigned int stub_dwt_lar = 0;
static CoreDebug_Type cd; static DWT_Type dw;
CoreDebug_Type *CoreDebug = &cd; DWT_Type *DWT = &dw;

void HAL_GPIO_Init(GPIO_TypeDef *p, GPIO_InitTypeDef *i) { (void)p;(void)i; }
void HAL_GPIO_WritePin(GPIO_TypeDef *p, uint16_t n, GPIO_PinState s) { (void)p;(void)n;(void)s; }
static int dout_level = 0;
GPIO_PinState HAL_GPIO_ReadPin(GPIO_TypeDef *p, uint16_t n) { (void)p;(void)n; return dout_level ? GPIO_PIN_SET : GPIO_PIN_RESET; }
void HAL_DMA_Init(DMA_HandleTypeDef *h) { (void)h; }
void HAL_DMA_IRQHandler(DMA_HandleTypeDef *h) { (void)h; }
void HAL_ADC_Init(ADC_HandleTypeDef *h) { (void)h; }
void HAL_ADC_ConfigChannel(ADC_HandleTypeDef *h, ADC_ChannelConfTypeDef *c) { (void)h;(void)c; }
void HAL_ADC_Start_DMA(ADC_HandleTypeDef *h, uint32_t *b, uint32_t n) { (void)h;(void)b;(void)n; }
void HAL_TIM_Base_Init(TIM_HandleTypeDef *h) { (void)h; }
void HAL_TIM_Base_Start(TIM_HandleTypeDef *h) { (void)h; }
void HAL_TIMEx_MasterConfigSynchronization(TIM_HandleTypeDef *h, TIM_MasterConfigTypeDef *c) { (void)h;(void)c; }
void HAL_RCC_OscConfig(RCC_OscInitTypeDef *o) { (void)o; }
void HAL_RCC_ClockConfig(RCC_ClkInitTypeDef *c, uint32_t f) { (void)c;(void)f; }
void HAL_NVIC_SetPriority(IRQn_Type i, uint32_t a, uint32_t b) { (void)i;(void)a;(void)b; }
void HAL_NVIC_EnableIRQ(IRQn_Type i) { (void)i; }
void HAL_Init(void) {}
void HAL_Delay(uint32_t ms) { (void)ms; }
uint32_t stub_tick = 0;
uint32_t HAL_GetTick(void) { return stub_tick; }
void MX_USB_DEVICE_Init(void) {}
uint32_t __get_PRIMASK(void) { return 0; }
void __disable_irq(void) {}
void __enable_irq(void) {}

USBD_HandleTypeDef hUsbDeviceFS;
static USBD_CDC_HandleTypeDef cdc_ctx;

/* Canal fake: acumula tudo o que "sai" pelo USB. */
static uint8_t wire[1 << 20];
static size_t wire_len = 0;
static int cdc_force_busy = 0;

uint8_t CDC_Transmit_FS(uint8_t *buf, uint16_t len)
{
    if (cdc_force_busy || cdc_ctx.TxState != 0U) return USBD_BUSY;
    memcpy(wire + wire_len, buf, len);
    wire_len += len;
    cdc_ctx.TxState = 1U;          /* transferência em voo */
    return USBD_OK;
}
static void usb_complete(void) { cdc_ctx.TxState = 0U; }

static void usb_setup(void)
{
    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    hUsbDeviceFS.pClassData = &cdc_ctx;
    cdc_ctx.TxState = 0U;
    usb_head = usb_tail = 0; usb_dropped = 0;
    wire_len = 0; cdc_force_busy = 0;
}

/* Drena tudo, simulando o host completando cada transferência. */
static void drain_all(void)
{
    for (int i = 0; i < 100000 && usb_tail != usb_head; i++) {
        usb_buffer_process();
        usb_complete();
    }
}

static int fails = 0;
#define CHECK(c, msg) do { if (!(c)) { printf("  FALHOU: %s\n", msg); fails++; } } while (0)

/* ---------------- testes ---------------- */

static void t_formatters(void)
{
    char b[32]; uint16_t n;
    n = u32_to_dec(0, b);      b[n] = 0; CHECK(!strcmp(b, "0"), "u32 0");
    n = u32_to_dec(4095, b);   b[n] = 0; CHECK(!strcmp(b, "4095"), "u32 4095");
    n = u32_to_dec(4294967295u, b); b[n] = 0; CHECK(!strcmp(b, "4294967295"), "u32 max");
    n = u64_to_dec(0, b);      b[n] = 0; CHECK(!strcmp(b, "0"), "u64 0");
    n = u64_to_dec(4294967296ULL, b); b[n] = 0; CHECK(!strcmp(b, "4294967296"), "u64 2^32");
    n = u64_to_dec(18446744073709551615ULL, b); b[n] = 0;
    CHECK(!strcmp(b, "18446744073709551615"), "u64 max");
    printf("[ok] formatadores decimais\n");
}

/* O ADC deve sair EXATAMENTE como o snprintf antigo produzia. */
static void t_adc_frame_matches_old_format(void)
{
    usb_setup();
    for (int i = 0; i < NUM_TAXELS; i++) last_adc[i] = (uint16_t)(i * 170 + 3);
    send_adc_frame(1234567ULL);
    drain_all();
    wire[wire_len] = 0;

    char expect[512];
    int n = snprintf(expect, sizeof(expect),
        "ADC,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,t=%lu\r\n",
        last_adc[0],last_adc[1],last_adc[2],last_adc[3],last_adc[4],
        last_adc[5],last_adc[6],last_adc[7],last_adc[8],last_adc[9],
        last_adc[10],last_adc[11],last_adc[12],last_adc[13],last_adc[14],
        last_adc[15],last_adc[16],last_adc[17],last_adc[18],last_adc[19],
        last_adc[20],last_adc[21],last_adc[22],last_adc[23],last_adc[24],
        (unsigned long)1234567UL);
    (void)n;
    CHECK(!strcmp((char *)wire, expect), "linha ADC != formato antigo");
    if (strcmp((char *)wire, expect)) { printf("   novo: %s   velho: %s", wire, expect); }
    printf("[ok] frame ADC byte-a-byte igual ao snprintf original\n");
}

/* Buffer cheio descarta a linha INTEIRA — nunca meia linha. */
static void t_write_is_all_or_nothing(void)
{
    usb_setup();
    char big[USB_TX_BUFFER_SIZE];
    memset(big, 'x', sizeof(big));

    CHECK(usb_buffer_write(big, USB_TX_BUFFER_SIZE - 1) == true, "deveria caber SIZE-1");
    CHECK(usb_dropped == 0, "nao deveria descartar ainda");
    CHECK(usb_buffer_write("ABC", 3) == false, "deveria recusar, buffer cheio");
    CHECK(usb_dropped == 1, "contador de descarte");

    /* nada de 'A','B','C' entrou no ring */
    int found = 0;
    for (int i = 0; i < USB_TX_BUFFER_SIZE; i++) if (usb_tx_buffer[i] == 'A') found = 1;
    CHECK(!found, "linha recusada deixou bytes no buffer");
    printf("[ok] escrita all-or-nothing + contador de descarte\n");
}

/* packet[] nao pode ser tocado enquanto a transferencia esta em voo. */
static void t_no_inflight_overwrite(void)
{
    usb_setup();
    for (int i = 0; i < 40; i++) usb_buffer_write("0123456789ABCDEF", 16);  /* 640 B */

    usb_buffer_process();               /* 1a transferencia sai */
    size_t after_first = wire_len;
    CHECK(after_first > 0, "primeira transferencia deveria sair");

    /* Sem completar: toda chamada extra tem de ser no-op. */
    for (int i = 0; i < 10; i++) usb_buffer_process();
    CHECK(wire_len == after_first, "transmitiu com transferencia em voo");

    usb_complete();
    drain_all();
    CHECK(wire_len == 640, "bytes entregues != bytes escritos");

    for (size_t i = 0; i < wire_len; i++)
        CHECK(wire[i] == "0123456789ABCDEF"[i % 16], "sequencia corrompida");
    printf("[ok] nenhuma escrita em packet[] com transferencia em voo\n");
}

/* Nenhuma transferencia pode ser multiplo de 64 (evita o ZLP). */
static void t_never_multiple_of_max_packet(void)
{
    usb_setup();
    for (int i = 0; i < 200; i++) usb_buffer_write("0123456789ABCDEF", 16); /* 3200 B */

    int seen = 0;
    while (usb_tail != usb_head) {
        size_t before = wire_len;
        usb_buffer_process();
        size_t len = wire_len - before;
        if (len > 0) {
            CHECK(len % USB_PACKET_SIZE != 0, "transferencia multipla de 64 (precisaria de ZLP)");
            CHECK(len <= USB_TX_CHUNK, "transferencia acima do chunk");
            seen++;
        }
        usb_complete();
    }
    CHECK(seen > 1, "deveria ter varias transferencias");
    CHECK(wire_len == 3200, "total entregue errado");
    printf("[ok] ZLP evitado (%d transferencias, nenhuma multipla de 64)\n", seen);
}

/* Escrita que cruza o fim do ring tem de sair contigua e na ordem. */
static void t_ring_wraparound(void)
{
    usb_setup();
    /* empurra head/tail para perto do fim */
    for (int i = 0; i < 500; i++) { usb_buffer_write("0123456789ABCDEF", 16); }
    drain_all();
    CHECK(wire_len == 8000, "pre-carga");
    wire_len = 0;

    const char *pat = "WRAPPED-LINE-0123456789\r\n";
    uint16_t plen = (uint16_t)strlen(pat);
    for (int i = 0; i < 50; i++) CHECK(usb_buffer_write(pat, plen), "escrita apos wrap");
    drain_all();
    wire[wire_len] = 0;
    CHECK(wire_len == (size_t)plen * 50, "tamanho apos wrap");
    for (int i = 0; i < 50; i++)
        CHECK(!memcmp(wire + i * plen, pat, plen), "conteudo apos wrap");
    printf("[ok] wraparound do ring preserva ordem e conteudo\n");
}

/* micros64 continua monotonico depois da volta do TIM2 de 32 bits.
 * So faz sentido com TS_64BIT ligado; com ele desligado o teste verifica o
 * contrato oposto — que o valor eh o contador cru de 32 bits. */
#if TS_64BIT
static void t_timestamp_never_wraps(void)
{
    tim2_hi = 0; tim2_last = 0;
    stub_tim_counter = 0;                uint64_t a = micros64();
    stub_tim_counter = 4294967000u;      uint64_t b = micros64();
    stub_tim_counter = 100u;             uint64_t c = micros64();   /* deu a volta */
    stub_tim_counter = 4294967000u;      uint64_t d = micros64();
    stub_tim_counter = 50u;              uint64_t e = micros64();   /* segunda volta */

    CHECK(a < b, "monotonico antes da volta");
    CHECK(c > b, "monotonico ATRAVES da volta");
    CHECK(c == 4294967296ULL + 100ULL, "valor apos 1a volta");
    CHECK(d > c && e > d, "monotonico na 2a volta");
    CHECK(e == 2ULL * 4294967296ULL + 50ULL, "valor apos 2a volta");
    printf("[ok] timestamp de 64 bits monotonico atraves das voltas\n");
}
#else
static void t_timestamp_never_wraps(void)
{
    stub_tim_counter = 4294967000u;
    CHECK(micros64() == 4294967000ULL, "TS_64BIT=0 deve devolver o contador cru");
    stub_tim_counter = 100u;
    CHECK(micros64() == 100ULL, "TS_64BIT=0 da a volta, por contrato");
    printf("[ok] TS_64BIT=0: timestamp cru de 32 bits (cabe no uint32 do TouchFrame)\n");
}
#endif

/* snprintf_len nunca deixa passar um comprimento maior que o buffer. */
static void t_snprintf_len_clamps(void)
{
    char small[8];
    int raw = snprintf(small, sizeof(small), "%s", "muito-mais-longo-que-8");
    CHECK(raw > (int)sizeof(small), "premissa: snprintf devolve o tamanho ideal");
    CHECK(snprintf_len(raw, sizeof(small)) == sizeof(small) - 1, "clamp do snprintf");
    CHECK(snprintf_len(-1, sizeof(small)) == 0, "erro do snprintf vira 0");
    printf("[ok] retorno de snprintf clampeado\n");
}

/* CDC nao configurado: nada sai e nada se perde. */
static void t_not_configured_keeps_data(void)
{
    usb_setup();
    hUsbDeviceFS.dev_state = 0;   /* nao configurado */
    usb_buffer_write("HELLO", 5);
    for (int i = 0; i < 10; i++) usb_buffer_process();
    CHECK(wire_len == 0, "transmitiu sem estar configurado");
    CHECK(usb_tail != usb_head, "descartou dado do buffer");

    hUsbDeviceFS.dev_state = USBD_STATE_CONFIGURED;
    drain_all();
    wire[wire_len] = 0;
    CHECK(!strcmp((char *)wire, "HELLO"), "dado nao sobreviveu a reconexao");
    printf("[ok] dado preservado ate o CDC configurar\n");
}

/* O atraso do HX711 NUNCA pode ser infinito. Um DWT que ignora a habilitação
 * (Cortex-M7 sem a chave do LAR) deixa CYCCNT em zero, e o laço antigo
 * `while (CYCCNT - start < n)` travava para sempre — dentro do boot, o que
 * fazia o dispositivo enumerar e ficar mudo. Aqui os dois caminhos são
 * exercitados; se algum não terminar, o teste trava e o timeout do harness
 * pega. */
static void t_delay_always_terminates(void)
{
    /* CYCCNT congelado: dwt_delay_init tem de DETECTAR e cair na reserva. */
    dw.CYCCNT = 0;
    dwt_delay_init();
    CHECK(dwt_ok == false, "DWT congelado deveria ser detectado");
    for (int i = 0; i < 200; i++) dwt_delay_cycles(HX711_SCK_DELAY_CYC);
    printf("[ok] CYCCNT congelado: detectado, reserva por NOP termina\n");

    /* Agora um CYCCNT que anda: o caminho normal. */
    dwt_ok = true;
    dw.CYCCNT = 0;
    for (int i = 0; i < 200; i++) { dw.CYCCNT += 1000; dwt_delay_cycles(HX711_SCK_DELAY_CYC); }
    printf("[ok] CYCCNT normal: caminho do DWT termina\n");

    /* CYCCNT que PARA no meio (depurador anexado): o teto de iterações salva. */
    dwt_ok = true;
    dw.CYCCNT = 5;
    for (int i = 0; i < 50; i++) dwt_delay_cycles(HX711_SCK_DELAY_CYC);
    printf("[ok] CYCCNT que para no meio: teto de iteracoes termina\n");
}

/* A tara não pode bloquear o laço: sem HX711 respondendo, ela tem de desistir
 * sozinha e o USB tem de seguir falando o tempo todo. */
static void t_tare_never_blocks(void)
{
    usb_setup();
    tare_done = false; tare_taken = 0; tare_sum = 0; tare_t0_ms = 0;
    dout_level = 1;                 /* DOUT alto = chip NUNCA pronto */

    stub_tick = 0;
    for (int i = 0; i < 5; i++) { HX711_TareStep(); CHECK(!tare_done, "tara cedo demais"); }

    stub_tick = HX711_TARE_DELAY_MS + HX711_TARE_WINDOW_MS + 1;
    HX711_TareStep();
    CHECK(tare_done, "tara deveria ter desistido por tempo");
    CHECK(hx711_offset == 0, "sem amostras, offset deve ser 0");

    drain_all();
    wire[wire_len] = 0;
    CHECK(strstr((char *)wire, "HX711 TARE TIMEOUT,0,200,") != NULL,
          "desistencia deveria ser reportada pelo USB");
    printf("[ok] tara desiste sozinha e avisa (celula ausente)\n");
    dout_level = 0;
}

int main(void)
{
    printf("== firmware TouchFirmware/main.c ==\n");
    t_formatters();
    t_adc_frame_matches_old_format();
    t_write_is_all_or_nothing();
    t_no_inflight_overwrite();
    t_never_multiple_of_max_packet();
    t_ring_wraparound();
    t_timestamp_never_wraps();
    t_snprintf_len_clamps();
    t_not_configured_keeps_data();
    t_delay_always_terminates();
    t_tare_never_blocks();
    printf(fails ? "\n%d VERIFICACAO(OES) FALHARAM\n" : "\nTodas as verificacoes passaram\n", fails);
    return fails ? 1 : 0;
}
