/**
 * @file test_wasm_bridge.c
 * @brief Unit tests for WebAssembly export API and zero-copy shared memory contracts.
 */

#include "wasm_bridge.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

int test_wasm_bridge_lifecycle(void) {
    printf("[TEST] WebAssembly Bridge Lifecycle (Init, Gen, Step, Pointers)... ");

    /* 1. Initialize engine at 32x32 */
    int ret = wasm_engine_init(32, 32);
    if (ret != 0) {
        printf("FAILED wasm_engine_init: %d\n", ret);
        return 1;
    }

    if (wasm_get_grid_width() != 32 || wasm_get_grid_height() != 32) {
        printf("FAILED grid dimensions\n");
        return 1;
    }

    /* 2. Generate terrain */
    ret = wasm_terrain_generate(42, 2.5f, 18.0f, 0.035f, 5, 0.5f, 2.0f, 30.0f);
    if (ret != 0) {
        printf("FAILED wasm_terrain_generate: %d\n", ret);
        return 1;
    }

    /* Verify pointers are non-null and point to valid memory */
    const float *h_ptr = wasm_get_heightmap_ptr();
    const uint8_t *mask_ptr = wasm_get_passable_mask_ptr();
    const CellData *cells_ptr = wasm_get_cells_ptr();

    if (!h_ptr || !mask_ptr || !cells_ptr) {
        printf("FAILED: null pointer export\n");
        return 1;
    }

    /* 3. Start search */
    ret = wasm_search_start(2, 2, 28, 28, 0 /* A* */, 1.0f, 1.0f, 2.5f, 0.25f);
    if (ret != 0) {
        printf("FAILED wasm_search_start: %d\n", ret);
        return 1;
    }

    /* 4. Step search */
    int status = SEARCH_RUNNING;
    int steps = 0;
    while (status == SEARCH_RUNNING && steps < 1000) {
        status = wasm_search_step(10);
        steps++;
    }

    if (status != SEARCH_FOUND) {
        printf("FAILED: search not completed (status=%d)\n", status);
        return 1;
    }

    /* 5. Path and stats exports */
    uint32_t path_len = wasm_get_path_length();
    const uint32_t *path_ptr = wasm_get_path_ptr();
    const SearchStats *stats_ptr = wasm_get_stats_ptr();

    if (!path_ptr || path_len < 2 || !stats_ptr) {
        printf("FAILED: invalid path or stats export\n");
        return 1;
    }

    /* 6. Test 3D orientation & normal calculation */
    float nx, ny, nz;
    wasm_get_terrain_normal(10.0f, 10.0f, &nx, &ny, &nz);
    float normal_len = sqrtf(nx * nx + ny * ny + nz * nz);
    if (fabsf(normal_len - 1.0f) > 1e-4f) {
        printf("FAILED: surface normal not unit length: %f\n", normal_len);
        return 1;
    }
    if (ny <= 0.0f) {
        printf("FAILED: normal Y <= 0: %f\n", ny);
        return 1;
    }

    /* 7. Test 3D waypoint coordinate retrieval */
    float wp_x, wp_y, wp_z;
    ret = wasm_get_path_waypoint(0, &wp_x, &wp_y, &wp_z);
    if (ret != 0) {
        printf("FAILED wasm_get_path_waypoint: %d\n", ret);
        return 1;
    }

    /* 8. Test Vector Pointer Helpers & Continuous Slope */
    const float *norm_vec = wasm_get_terrain_normal_ptr(10.0f, 10.0f);
    if (!norm_vec || fabsf(norm_vec[0] - nx) > 1e-5f || fabsf(norm_vec[1] - ny) > 1e-5f || fabsf(norm_vec[2] - nz) > 1e-5f) {
        printf("FAILED wasm_get_terrain_normal_ptr mismatch\n");
        return 1;
    }

    float slope = wasm_get_terrain_slope_at(10.0f, 10.0f);
    if (slope < 0.0f || slope > 90.0f) {
        printf("FAILED wasm_get_terrain_slope_at out of range: %f\n", slope);
        return 1;
    }

    const float *wp_vec = wasm_get_path_waypoint_ptr(0);
    if (!wp_vec || fabsf(wp_vec[0] - wp_x) > 1e-5f || fabsf(wp_vec[1] - wp_y) > 1e-5f || fabsf(wp_vec[2] - wp_z) > 1e-5f) {
        printf("FAILED wasm_get_path_waypoint_ptr mismatch\n");
        return 1;
    }

    printf("PASSED (Zero-copy pointers, 3D Normals, Stepping API, and Telemetry validated)\n");
    return 0;
}
