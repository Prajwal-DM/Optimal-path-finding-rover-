/**
 * @file test_wasm_physics.js
 * @brief Automated headless test verifying the WebAssembly 3D rover physics pipeline.
 */

const Module = require('../web/rover_engine.js');

function run() {
    return new Promise((resolve, reject) => {
        Module.onRuntimeInitialized = () => {
            try {
                console.log("====================================================");
                console.log(" PHASE 5: WEBASSEMBLY ROVER 3D PHYSICS VERIFICATION ");
                console.log("====================================================");
                executePhysicsTests();
                resolve();
            } catch (err) {
                reject(err);
            }
        };
    });
}

function executePhysicsTests() {
    const init = Module.cwrap('wasm_engine_init', 'number', ['number', 'number']);
    const gen = Module.cwrap('wasm_terrain_generate', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number']);
    const start = Module.cwrap('wasm_search_start', 'number', ['number', 'number', 'number', 'number', 'number', 'number', 'number', 'number', 'number']);
    const solve = Module.cwrap('wasm_search_solve', 'number', []);
    const getPathLen = Module.cwrap('wasm_get_path_length', 'number', []);
    const getWaypointPtr = Module.cwrap('wasm_get_path_waypoint_ptr', 'number', ['number']);
    const getInterpHeight = Module.cwrap('wasm_get_interpolated_height', 'number', ['number', 'number']);
    const getNormalPtr = Module.cwrap('wasm_get_terrain_normal_ptr', 'number', ['number', 'number']);
    const getSlopeAt = Module.cwrap('wasm_get_terrain_slope_at', 'number', ['number', 'number']);

    console.log("[TEST] Initializing engine & generating terrain (seed 42)...");
    init(64, 64);
    gen(42, 2.5, 18.0, 0.035, 5, 0.5, 2.0, 30.0);

    console.log("[TEST] Solving A* path from (5, 5) to (50, 50)...");
    start(5, 5, 50, 50, 0, 1.0, 1.0, 2.5, 0.25);
    const status = solve();
    if (status !== 2) throw new Error(`Search failed with status ${status}`);

    const pathLen = getPathLen();
    console.log(`[TEST] Solved path found with ${pathLen} waypoints.`);
    if (pathLen < 2) throw new Error("Expected path length >= 2");

    // 1. Verify Waypoint Extraction
    console.log("[TEST] Verifying continuous waypoint extraction from C...");
    const waypoints = [];
    for (let i = 0; i < pathLen; ++i) {
        const ptr = getWaypointPtr(i);
        const x = Module.HEAPF32[ptr >> 2];
        const y = Module.HEAPF32[(ptr >> 2) + 1];
        const z = Module.HEAPF32[(ptr >> 2) + 2];
        waypoints.push({ x, y, z });
    }

    const first = waypoints[0];
    const last = waypoints[pathLen - 1];
    console.log(`[TEST] Start Waypoint: (${first.x.toFixed(1)}, ${first.y.toFixed(2)}, ${first.z.toFixed(1)})`);
    console.log(`[TEST] Goal Waypoint:  (${last.x.toFixed(1)}, ${last.y.toFixed(2)}, ${last.z.toFixed(1)})`);

    // 2. Verify Continuous Elevation & Bilinear Interpolation
    console.log("[TEST] Verifying continuous bilinear height interpolation across waypoints...");
    for (let i = 0; i < pathLen - 1; ++i) {
        const p0 = waypoints[i];
        const p1 = waypoints[i + 1];

        // Sample midpoint
        const midX = (p0.x + p1.x) * 0.5;
        const midZ = (p0.z + p1.z) * 0.5;
        const midH = getInterpHeight(midX, midZ);

        if (isNaN(midH)) throw new Error(`NaN height at (${midX}, ${midZ})`);
        // Height should be smoothly bounded by neighborhood
        const minLocal = Math.min(p0.y, p1.y) - 2.5;
        const maxLocal = Math.max(p0.y, p1.y) + 2.5;
        if (midH < minLocal || midH > maxLocal) {
            throw new Error(`Height spike at (${midX}, ${midZ}): midH=${midH}, expected [${minLocal}, ${maxLocal}]`);
        }
    }
    console.log("   -> Continuous height interpolation verified with 0 spikes or discontinuities.");

    // 3. Verify Surface Normal Vector & Slope in C
    console.log("[TEST] Verifying 3D Surface Normal vectors & local slope calculation in C...");
    for (let i = 0; i < pathLen; ++i) {
        const p = waypoints[i];
        const normPtr = getNormalPtr(p.x, p.z);
        const nx = Module.HEAPF32[normPtr >> 2];
        const ny = Module.HEAPF32[(normPtr >> 2) + 1];
        const nz = Module.HEAPF32[(normPtr >> 2) + 2];

        const len = Math.hypot(nx, ny, nz);
        if (Math.abs(len - 1.0) > 1e-4) {
            throw new Error(`Normal vector not normalized at index ${i}: len=${len}`);
        }
        if (ny <= 0) {
            throw new Error(`Normal Y must be positive (upward facing), got ny=${ny}`);
        }

        const slopeDeg = getSlopeAt(p.x, p.z);
        if (slopeDeg < 0.0 || slopeDeg > 90.0) {
            throw new Error(`Invalid slope at index ${i}: ${slopeDeg}`);
        }

        const expectedSlope = Math.acos(Math.max(-1.0, Math.min(1.0, ny))) * (180.0 / Math.PI);
        if (Math.abs(slopeDeg - expectedSlope) > 0.05) {
            throw new Error(`Slope mismatch at index ${i}: getSlope=${slopeDeg} vs acos(ny)=${expectedSlope}`);
        }
    }
    console.log("   -> Surface normals unit length, upward pointing, and slope exact match with acos(Ny).");

    // 4. Verify Orthonormal Rover Basis & Attitude Angles
    console.log("[TEST] Verifying Orthonormal Rover Basis & Pitch/Roll attitude math...");
    for (let i = 0; i < pathLen - 1; ++i) {
        const p0 = waypoints[i];
        const p1 = waypoints[i + 1];

        const tangent = {
            x: p1.x - p0.x,
            y: p1.y - p0.y,
            z: p1.z - p0.z
        };
        const tLen = Math.hypot(tangent.x, tangent.y, tangent.z);
        tangent.x /= tLen; tangent.y /= tLen; tangent.z /= tLen;

        const normPtr = getNormalPtr(p0.x, p0.z);
        const up = {
            x: Module.HEAPF32[normPtr >> 2],
            y: Module.HEAPF32[(normPtr >> 2) + 1],
            z: Module.HEAPF32[(normPtr >> 2) + 2]
        };

        // Forward projected onto surface
        const dotTU = tangent.x * up.x + tangent.y * up.y + tangent.z * up.z;
        let fwd = {
            x: tangent.x - up.x * dotTU,
            y: tangent.y - up.y * dotTU,
            z: tangent.z - up.z * dotTU
        };
        const fLen = Math.hypot(fwd.x, fwd.y, fwd.z);
        fwd.x /= fLen; fwd.y /= fLen; fwd.z /= fLen;

        // Right = cross(Forward, Up)
        const right = {
            x: fwd.y * up.z - fwd.z * up.y,
            y: fwd.z * up.x - fwd.x * up.z,
            z: fwd.x * up.y - fwd.y * up.x
        };
        const rLen = Math.hypot(right.x, right.y, right.z);

        if (Math.abs(rLen - 1.0) > 1e-4) throw new Error("Right vector not unit length");

        // Pitch & Roll
        const pitch = Math.asin(Math.max(-1.0, Math.min(1.0, fwd.y))) * (180.0 / Math.PI);
        const roll = Math.asin(Math.max(-1.0, Math.min(1.0, right.y))) * (180.0 / Math.PI);

        // Path is on passable terrain so pitch and roll must be within traversable bounds
        if (Math.abs(pitch) > 35.0) {
            throw new Error(`Pitch angle exceeded threshold: ${pitch}`);
        }
        if (Math.abs(roll) > 35.0) {
            throw new Error(`Roll angle exceeded threshold: ${roll}`);
        }
    }
    console.log("   -> Rover orientation basis orthonormal; pitch and roll strictly within safe traversable envelope.");

    // 5. Verify C Path Smoothing via WebAssembly
    console.log("[TEST] Verifying C Line-of-Sight Path Smoothing via WebAssembly...");
    const getSmoothedPathLen = Module.cwrap('wasm_get_smoothed_path_length', 'number', []);
    const getSmoothedWaypointPtr = Module.cwrap('wasm_get_smoothed_waypoint_ptr', 'number', ['number']);

    const smoothLen = getSmoothedPathLen();
    console.log(`[TEST] Raw Waypoints: ${pathLen} | Smoothed Waypoints: ${smoothLen}`);
    if (smoothLen <= 0 || smoothLen > pathLen) {
        throw new Error(`Invalid smoothed path length: ${smoothLen} (raw: ${pathLen})`);
    }

    const startSmoothedPtr = getSmoothedWaypointPtr(0);
    const startSmoothedX = Module.HEAPF32[startSmoothedPtr >> 2];
    const startSmoothedZ = Module.HEAPF32[(startSmoothedPtr >> 2) + 2];
    if (Math.abs(startSmoothedX - first.x) > 1e-4 || Math.abs(startSmoothedZ - first.z) > 1e-4) {
        throw new Error("Smoothed start point differs from raw start point");
    }

    const goalSmoothedPtr = getSmoothedWaypointPtr(smoothLen - 1);
    const goalSmoothedX = Module.HEAPF32[goalSmoothedPtr >> 2];
    const goalSmoothedZ = Module.HEAPF32[(goalSmoothedPtr >> 2) + 2];
    if (Math.abs(goalSmoothedX - last.x) > 1e-4 || Math.abs(goalSmoothedZ - last.z) > 1e-4) {
        throw new Error("Smoothed goal point differs from raw goal point");
    }
    const reduction = ((1.0 - smoothLen / pathLen) * 100.0).toFixed(1);
    console.log(`   -> C Line-of-Sight Path Smoothing compressed route by ${reduction}%!`);

    // 6. Verify Fuel and Risk Weight Ranges [0.0 to 5.0]
    console.log("[TEST] Verifying Fuel and Risk Weight extremes (0.0 to 5.0)...");
    const weightConfigs = [
        { alpha: 0.0, beta: 0.0, desc: "Zero Fuel & Zero Risk (Shortest Path)" },
        { alpha: 0.0, beta: 5.0, desc: "Zero Fuel & Max Risk (Ultra Safe Route)" },
        { alpha: 5.0, beta: 0.0, desc: "Max Fuel & Zero Risk (Energy Minimizer)" },
        { alpha: 5.0, beta: 5.0, desc: "Max Fuel & Max Risk (Balanced High Penalties)" }
    ];

    for (const cfg of weightConfigs) {
        start(5, 5, 50, 50, 0, cfg.alpha, cfg.beta, 2.5, 0.25);
        const st = solve();
        const pLen = getPathLen();
        if (st !== 2 || pLen <= 0) {
            throw new Error(`Failed to solve path for ${cfg.desc} (alpha=${cfg.alpha}, beta=${cfg.beta})`);
        }
        console.log(`   -> ${cfg.desc}: Solved path found with ${pLen} nodes.`);
    }

    console.log("\n----------------------------------------------------");
    console.log(">>> PHASE 6 PHYSICS & SMOOTHING VERIFICATION PASSED! <<<");
    console.log("====================================================\n");
}

run().catch(err => {
    console.error("FAILED:", err);
    process.exit(1);
});
