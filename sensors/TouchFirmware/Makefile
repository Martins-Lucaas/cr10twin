# Testes de host do firmware do sensor tátil.
#
# NÃO compila o firmware para o alvo — isso é trabalho do CubeIDE/PlatformIO.
# O que roda aqui é a cadeia que não depende de hardware (ring buffer USB,
# formatadores decimais, relógio de 64 bits), contra stubs da HAL em test/stub.
#
#   make test     compila e roda
#   make check    só a checagem sintática, com -Wall -Wextra
#   make clean

CC      ?= gcc
CFLAGS  ?= -std=gnu11 -O1 -Wall -Wextra -Wno-unused-parameter
INCLUDE  = -Itest/stub

.PHONY: test check clean

test: build/test_fw
	./build/test_fw

build/test_fw: test/test_fw.c main.c test/stub/*.h
	@mkdir -p build
	$(CC) $(CFLAGS) -Wno-format-truncation $(INCLUDE) -o $@ test/test_fw.c -lm

check:
	$(CC) -fsyntax-only $(CFLAGS) $(INCLUDE) main.c && echo "main.c: sintaxe OK"

clean:
	rm -rf build
