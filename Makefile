CC = gcc
CFLAGS = -Wall -Wextra -O3 -std=c99 -Iinclude
LDFLAGS = -lm

SRC = src/core_math.c src/prng.c src/min_heap.c src/noise.c src/terrain.c src/cost_model.c src/search.c src/wasm_bridge.c
TEST_SRC = tests/test_runner.c tests/test_prng.c tests/test_min_heap.c tests/test_math.c \
           tests/test_noise.c tests/test_terrain.c tests/test_cost_model.c tests/test_search.c \
           tests/test_wasm_bridge.c

TARGET = bin/master_tests.exe

all: $(TARGET)

$(TARGET): $(SRC) $(TEST_SRC)
	@if not exist bin mkdir bin
	$(CC) $(CFLAGS) $(SRC) $(TEST_SRC) -o $(TARGET) $(LDFLAGS)

test: $(TARGET)
	@$(TARGET)

clean:
	@if exist bin rmdir /s /q bin

.PHONY: all test clean
