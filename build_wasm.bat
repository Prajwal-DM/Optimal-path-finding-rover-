@echo off
if not exist web mkdir web

echo Setting up Emscripten environment...
call emsdk\emsdk_env.bat

echo Compiling C Core to WebAssembly (O3 Optimized, Zero-Copy Exports)...
call emcc -O3 -std=c99 -Iinclude ^
    src/core_math.c src/prng.c src/min_heap.c src/noise.c src/terrain.c src/cost_model.c src/search.c src/wasm_bridge.c ^
    -s WASM=1 ^
    -s EXPORTED_RUNTIME_METHODS="['cwrap','getValue','HEAPF32','HEAPU8','HEAPU32']" ^
    -s EXPORTED_FUNCTIONS="['_wasm_engine_init','_wasm_terrain_generate','_wasm_search_start','_wasm_search_step','_wasm_search_solve','_wasm_get_heightmap_ptr','_wasm_get_passable_mask_ptr','_wasm_get_cells_ptr','_wasm_get_path_ptr','_wasm_get_path_length','_wasm_get_stats_ptr','_wasm_get_grid_width','_wasm_get_grid_height','_wasm_get_grid_cell_size','_wasm_get_cell_flags','_wasm_get_cell_height','_wasm_get_cell_slope','_wasm_get_cell_roughness','_wasm_get_cell_terrain_type','_wasm_get_interpolated_height','_wasm_get_terrain_normal','_wasm_get_path_waypoint','_wasm_get_terrain_normal_ptr','_wasm_get_terrain_slope_at','_wasm_get_path_waypoint_ptr','_wasm_smooth_path','_wasm_get_smoothed_path_ptr','_wasm_get_smoothed_path_length','_wasm_get_smoothed_waypoint_ptr']" ^
    -s ALLOW_MEMORY_GROWTH=1 ^
    -o web/rover_engine.js

if %ERRORLEVEL% equ 0 (
    echo WebAssembly compilation SUCCESSFUL! Output in web/rover_engine.js and web/rover_engine.wasm
) else (
    echo WebAssembly compilation FAILED!
    exit /b %ERRORLEVEL%
)
