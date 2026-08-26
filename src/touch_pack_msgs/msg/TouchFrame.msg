# Um frame de taxels do sensor de toque COM o carimbo do firmware
# (/touch_sensor/frame, republicado pela GUI a partir da linha
# "ADC,v0,...,vN-1,t=micros" do STM32).
#
# Substitui o Int32MultiArray de /touch_sensor/adc, que só carregava os valores
# e descartava o `t=` da própria linha — o mesmo problema do lado da célula
# (ver LoadCellSample). `rows`/`cols` viajam junto para que o consumidor não
# precise adivinhar a grade a partir do comprimento.

uint32 t_us       # micros() do STM32 no instante do frame, uint32 com wraparound
int32[] taxels    # rows*cols leituras do ADC (0..4095), varredura linha a linha
uint16 rows
uint16 cols
