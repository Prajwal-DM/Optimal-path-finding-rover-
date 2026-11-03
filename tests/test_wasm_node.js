/**
 * @file test_wasm_node.js
 * @brief Headless Node.js test script verifying the compiled WebAssembly binary.
 */

const path = require('path');
const Module = require('../web/rover_engine.js');

function run() {
    return new Promise((resolve, reject) => {
        Module.onRuntimeInitialized = () => {
            try {
                console.log("====================================================");
                console.log(" HEADLESS WEBASSEMBLY VERIFICATION (NODE.JS)        ");
                console.log("====================================================");
                console.log("[WASM] WebAssembly module loaded and initialized!");
                executeTests();
                resolve();
            } catch (err) {
                reject(err);
            }
        };
    });
}

function executeTests() {

    const wasm_engine_init = Module.cwrap('wasm_engine_init', 'number', ['number', 'number']);
    const wasm_terrain_generate = Module.cwrap('wasm_terrain_generate', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number']);
    const wasm_search_start = Module.cwrap('wasm_search_start', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number']);
    const wasm_search_step = Module.cwrap('wasm_search_step', 'number', ['number']);
    const wasm_search_solve = Module.cwrap('wasm_search_solve', 'number', []);
    const wasm_get_heightmap_ptr = Module.cwrap('wasm_get_heightmap_ptr', 'number', []);
    const wasm_get_passable_mask_ptr = Module.cwrap('wasm_get_passable_mask_ptr', 'number', []);
    const wasm_get_path_ptr = Module.cwrap('wasm_get_path_ptr', 'number', []);
    const wasm_get_path_length = Module.cwrap('wasm_get_path_length', 'number', []);
    const wasm_get_stats_ptr = Module.cwrap('wasm_get_stats_ptr', 'number', []);

    // 1. Initialize
    console.log("[TEST] Initializing 64x64 engine...");
    let ret = wasm_engine_init(64, 64);
    if (ret !== 0) throw new Error(`wasm_engine_init failed with code ${ret}`);

    // 2. Generate terrain
    console.log("[TEST] Generating procedural terrain from seed 1337...");
    ret = wasm_terrain_generate(1337, 2.5, 18.0, 0.035, 5, 0.5, 2.0, 30.0);
    if (ret !== 0) throw new Error(`wasm_terrain_generate failed with code ${ret}`);

    // Verify zero-copy pointers
    const hPtr = wasm_get_heightmap_ptr();
    const maskPtr = wasm_get_passable_mask_ptr();
    console.log(`[WASM Memory] Heightmap pointer: 0x${hPtr.toString(16)}, Passable Mask: 0x${maskPtr.toString(16)}`);

    const heightmap = new Float32Array(Module.HEAPF32.buffer, hPtr, 64 * 64);
    const mask = new Uint8Array(Module.HEAPU8.buffer, maskPtr, 64 * 64);

    let passableCount = 0;
    for (let i = 0; i < 64 * 64; ++i) {
        if (mask[i] === 1) passableCount++;
    }
    console.log(`[TEST] Generated 4096 cells | Passable: ${passableCount} (${((passableCount / 4096) * 100).toFixed(1)}%)`);

    // 3. Search start: A* from (5, 5) to (50, 50)
    console.log("[TEST] Launching A* search with octile heuristic...");
    ret = wasm_search_start(5, 5, 50, 50, 0 /* A* */, 1.0, 1.0, 2.5, 0.25);
    if (ret !== 0) throw new Error(`wasm_search_start failed with code ${ret}`);

    // 4. Stepped execution
    let status = 1; // SEARCH_RUNNING
    let steps = 0;
    while (status === 1 && steps < 500) {
        status = wasm_search_step(20);
        steps++;
    }

    console.log(`[TEST] Search finished after ${steps} discrete step calls with status ${status} (2=FOUND)`);
    if (status !== 2) throw new Error(`Search failed to find path, status: ${status}`);

    const pathLen = wasm_get_path_length();
    const pathPtr = wasm_get_path_ptr();
    const statsPtr = wasm_get_stats_ptr();

    const nodes = Module.getValue(statsPtr + 0, 'i32');
    const heap = Module.getValue(statsPtr + 4, 'i32');
    const distanceMeters = Module.getValue(statsPtr + 12, 'float');
    const totalCost = Module.getValue(statsPtr + 16, 'float');
    const fuelCost = Module.getValue(statsPtr + 20, 'float');
    const riskCost = Module.getValue(statsPtr + 24, 'float');
    const timeMs = Module.getValue(statsPtr + 28, 'double');

    const wasm_get_smoothed_path_length = Module.cwrap('wasm_get_smoothed_path_length', 'number', []);
    const smoothLen = wasm_get_smoothed_path_length();

    console.log("\n--- TELEMETRY REPORT ---");
    console.log(`Path Length:      ${pathLen} waypoints (${distanceMeters.toFixed(1)} meters)`);
    console.log(`Smoothed Path:    ${smoothLen} waypoints (${((1.0 - smoothLen / pathLen) * 100.0).toFixed(1)}% LOS shortcutting)`);
    console.log(`Total Path Cost:  ${totalCost.toFixed(2)} (Fuel: ${fuelCost.toFixed(2)} | Risk: ${riskCost.toFixed(2)})`);
    console.log(`Nodes Expanded:   ${nodes}`);
    console.log(`Peak Heap Size:   ${heap}`);
    console.log(`Compute Time:     ${timeMs.toFixed(2)} ms`);
    console.log("------------------------");
    console.log(">>> HEADLESS WEBASSEMBLY VERIFICATION PASSED! <<<\n");
}

run().catch(err => {
    console.error("FAILED:", err);
    process.exit(1);
});
