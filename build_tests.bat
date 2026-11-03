@echo off
if not exist bin mkdir bin
gcc -Wall -Wextra -O3 -std=c99 -Iinclude ^
    src/core_math.c src/prng.c src/min_heap.c src/noise.c src/terrain.c src/cost_model.c src/search.c src/wasm_bridge.c ^
    tests/test_runner.c tests/test_prng.c tests/test_min_heap.c tests/test_math.c ^
    tests/test_noise.c tests/test_terrain.c tests/test_cost_model.c tests/test_search.c tests/test_wasm_bridge.c ^
    -o bin/master_tests.exe -lm
if %ERRORLEVEL% equ 0 (
    echo Compilation successful! Running master tests...
    bin\master_tests.exe
) else (
    echo Compilation failed!
    exit /b %ERRORLEVEL%
)
