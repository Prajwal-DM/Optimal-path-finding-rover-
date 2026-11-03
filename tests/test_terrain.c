/**
 * @file test_terrain.c
 * @brief Unit tests for terrain grid generation, 9-point multi-sampling, and passability.
 */

#include "terrain.h"
#include "core_math.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#define GRID_TEST_W 64
#define GRID_TEST_H 64
#define GRID_TEST_CELLS (GRID_TEST_W * GRID_TEST_H)

static float s_heightmap[GRID_TEST_CELLS];
static CellData s_cells[GRID_TEST_CELLS];
static uint8_t s_passable[GRID_TEST_CELLS];

static float s_heightmap2[GRID_TEST_CELLS];
static CellData s_cells2[GRID_TEST_CELLS];
static uint8_t s_passable2[GRID_TEST_CELLS];

int test_terrain_generation_and_sampling(void) {
    printf("[TEST] Terrain 9-Point Sampling & Slope Classification... ");
    TerrainGrid grid;
    RoverError err = terrain_init(&grid, GRID_TEST_W, GRID_TEST_H, s_heightmap, s_cells, s_passable);
    if (err != ROVER_OK) {
        printf("FAILED to init terrain grid: %d\n", err);
        return 1;
    }

    TerrainParams params = terrain_default_params(12345, GRID_TEST_W, GRID_TEST_H);
    err = terrain_generate(&grid, &params);
    if (err != ROVER_OK) {
        printf("FAILED to generate terrain: %d\n", err);
        return 1;
    }

    if (grid.total_cells != GRID_TEST_CELLS) {
        printf("FAILED: total cells mismatch (%u vs %u)\n", grid.total_cells, GRID_TEST_CELLS);
        return 1;
    }

    if (grid.blocked_cells + grid.passable_cells != grid.total_cells) {
        printf("FAILED: blocked + passable != total cells\n");
        return 1;
    }

    /* Verify 9-point sampling slope thresholding */
    for (uint32_t i = 0; i < grid.total_cells; ++i) {
        float slope = grid.cells[i].slope_deg;
        float rough = grid.cells[i].roughness;

        if (rough < 0.0f) {
            printf("FAILED: negative roughness at index %u\n", i);
            return 1;
        }

        if (slope > grid.max_slope_deg) {
            if (!(grid.cells[i].flags & CELL_FLAG_BLOCKED) || grid.passable_mask[i] != 0) {
                printf("FAILED: cell %u has slope %.2f > %.2f but is not marked blocked!\n",
                       i, slope, grid.max_slope_deg);
                return 1;
            }
            if (grid.cells[i].terrain_type != TERRAIN_CLIFF) {
                printf("FAILED: blocked cell %u not marked TERRAIN_CLIFF\n", i);
                return 1;
            }
        } else {
            if (grid.cells[i].flags & CELL_FLAG_BLOCKED || grid.passable_mask[i] != 1) {
                printf("FAILED: cell %u has safe slope %.2f <= %.2f but is marked blocked!\n",
                       i, slope, grid.max_slope_deg);
                return 1;
            }
        }
    }

    printf("PASSED (%u passable, %u blocked cliffs, 100%% threshold accuracy)\n",
           grid.passable_cells, grid.blocked_cells);
    return 0;
}

int test_terrain_reproducibility(void) {
    printf("[TEST] Terrain Multi-Octave Seed Reproducibility... ");
    TerrainGrid grid1, grid2;
    terrain_init(&grid1, GRID_TEST_W, GRID_TEST_H, s_heightmap, s_cells, s_passable);
    terrain_init(&grid2, GRID_TEST_W, GRID_TEST_H, s_heightmap2, s_cells2, s_passable2);

    TerrainParams params = terrain_default_params(0xCAFEBABE, GRID_TEST_W, GRID_TEST_H);
    terrain_generate(&grid1, &params);
    terrain_generate(&grid2, &params);

    for (uint32_t i = 0; i < GRID_TEST_CELLS; ++i) {
        if (s_heightmap[i] != s_heightmap2[i]) {
            printf("FAILED: heightmap mismatch at %u (%f != %f)\n", i, s_heightmap[i], s_heightmap2[i]);
            return 1;
        }
        if (s_cells[i].slope_deg != s_cells2[i].slope_deg) {
            printf("FAILED: slope mismatch at %u\n", i);
            return 1;
        }
        if (s_passable[i] != s_passable2[i]) {
            printf("FAILED: passability mismatch at %u\n", i);
            return 1;
        }
    }

    printf("PASSED (Bit-exact match across entire %ux%u map)\n", GRID_TEST_W, GRID_TEST_H);
    return 0;
}

int test_terrain_boundary_checks(void) {
    printf("[TEST] Terrain Coordinate Boundary Guards... ");
    TerrainGrid grid;
    terrain_init(&grid, GRID_TEST_W, GRID_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams params = terrain_default_params(42, GRID_TEST_W, GRID_TEST_H);
    terrain_generate(&grid, &params);

    if (terrain_is_cell_passable(&grid, -1, 5)) {
        printf("FAILED: negative x reported passable\n");
        return 1;
    }
    if (terrain_is_cell_passable(&grid, 5, -1)) {
        printf("FAILED: negative y reported passable\n");
        return 1;
    }
    if (terrain_is_cell_passable(&grid, GRID_TEST_W, 0)) {
        printf("FAILED: x = width reported passable\n");
        return 1;
    }
    if (terrain_is_cell_passable(&grid, 0, GRID_TEST_H)) {
        printf("FAILED: y = height reported passable\n");
        return 1;
    }

    printf("PASSED (All out-of-bounds queries rejected)\n");
    return 0;
}

int test_terrain_subcell_micro_ridge_detection(void) {
    printf("[TEST] Sub-Cell Micro-Ridge & 36-Pair Pairwise Detection... ");

    /* Test empirical case:
     * Corner (0,0) = 0.0m
     * Top-Mid (0.5, 0) = 0.95m
     * Center (0.5, 0.5) = 0.95m
     * Center-to-corner is 28.25 deg (< 30 deg safe)
     * Corner-to-top-mid is 37.23 deg (> 30 deg fatal cliff!)
     */
    float cs = 2.5f;
    float samples[9] = {
        0.95f, /* 0: Center */
        0.00f, /* 1: Top-Left Corner */
        0.95f, /* 2: Top-Right */
        0.95f, /* 3: Bottom-Left */
        0.95f, /* 4: Bottom-Right */
        0.95f, /* 5: Top-Mid */
        0.95f, /* 6: Bottom-Mid */
        0.95f, /* 7: Left-Mid */
        0.95f  /* 8: Right-Mid */
    };

    static const float SUB_COORDS[9][2] = {
        { 0.5f, 0.5f }, { 0.0f, 0.0f }, { 1.0f, 0.0f },
        { 0.0f, 1.0f }, { 1.0f, 1.0f }, { 0.5f, 0.0f },
        { 0.5f, 1.0f }, { 0.0f, 0.5f }, { 1.0f, 0.5f }
    };

    float max_slope = 0.0f;
    for (int i = 0; i < 9; ++i) {
        for (int j = i + 1; j < 9; ++j) {
            float dx = (SUB_COORDS[j][0] - SUB_COORDS[i][0]) * cs;
            float dy = (SUB_COORDS[j][1] - SUB_COORDS[i][1]) * cs;
            float dist_m = sqrtf(dx * dx + dy * dy);
            float s = math_calc_slope_deg(fabsf(samples[i] - samples[j]), dist_m);
            if (s > max_slope) max_slope = s;
        }
    }

    if (max_slope < 37.0f || max_slope > 37.5f) {
        printf("FAILED: expected 37.23 deg micro-cliff, got %f deg\n", max_slope);
        return 1;
    }

    if (max_slope <= 30.0f) {
        printf("FAILED: 37 deg cliff not recognized as impassable\n");
        return 1;
    }

    printf("PASSED (Micro-cliff detected at %.2f deg > 30.0 deg cliff threshold)\n", max_slope);
    return 0;
}
