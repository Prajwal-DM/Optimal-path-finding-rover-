/**
 * @file test_cost_model.c
 * @brief Unit tests for physical cost model, corner-cutting, and admissible heuristic.
 */

#include "cost_model.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

#define TEST_GRID_SIZE 32
#define TEST_TOTAL (TEST_GRID_SIZE * TEST_GRID_SIZE)

static float s_cost_heightmap[TEST_TOTAL];
static CellData s_cost_cells[TEST_TOTAL];
static uint8_t s_cost_passable[TEST_TOTAL];

int test_cost_corner_cutting_prevention(void) {
    printf("[TEST] Diagonal Corner-Cutting Prevention... ");
    TerrainGrid grid;
    terrain_init(&grid, TEST_GRID_SIZE, TEST_GRID_SIZE, s_cost_heightmap, s_cost_cells, s_cost_passable);
    TerrainParams p = terrain_default_params(42, TEST_GRID_SIZE, TEST_GRID_SIZE);
    terrain_generate(&grid, &p);

    /* Construct a controlled 2x2 corner scenario:
     * u = (5, 5), v = (6, 6) [diagonal move]
     * Cardinal 1: (6, 5)
     * Cardinal 2: (5, 6)
     */
    uint32_t u_idx = 5 * TEST_GRID_SIZE + 5;
    uint32_t v_idx = 6 * TEST_GRID_SIZE + 6;
    uint32_t c1_idx = 5 * TEST_GRID_SIZE + 6;
    uint32_t c2_idx = 6 * TEST_GRID_SIZE + 5;

    /* Ensure endpoints are passable */
    grid.cells[u_idx].flags &= ~CELL_FLAG_BLOCKED;
    grid.passable_mask[u_idx] = 1;
    grid.cells[v_idx].flags &= ~CELL_FLAG_BLOCKED;
    grid.passable_mask[v_idx] = 1;

    /* Case A: Block cardinal 1 */
    grid.cells[c1_idx].flags |= CELL_FLAG_BLOCKED;
    grid.passable_mask[c1_idx] = 0;
    grid.cells[c2_idx].flags &= ~CELL_FLAG_BLOCKED;
    grid.passable_mask[c2_idx] = 1;

    CostParams cp = cost_default_params();
    if (cost_is_move_legal(&grid, 5, 5, 6, 6)) {
        printf("FAILED: diagonal move allowed when cardinal (5, 6) is blocked!\n");
        return 1;
    }
    float cost_blocked = cost_compute_edge(&grid, 5, 5, 6, 6, &cp, NULL, NULL);
    if (cost_blocked < COST_INFINITE) {
        printf("FAILED: cost not infinite when cutting blocked corner (got %f)\n", cost_blocked);
        return 1;
    }

    /* Case B: Block cardinal 2 instead */
    grid.cells[c1_idx].flags &= ~CELL_FLAG_BLOCKED;
    grid.passable_mask[c1_idx] = 1;
    grid.cells[c2_idx].flags |= CELL_FLAG_BLOCKED;
    grid.passable_mask[c2_idx] = 0;

    if (cost_is_move_legal(&grid, 5, 5, 6, 6)) {
        printf("FAILED: diagonal move allowed when cardinal (6, 5) is blocked!\n");
        return 1;
    }

    /* Case C: Unblock both cardinals -> Move becomes legal */
    grid.cells[c2_idx].flags &= ~CELL_FLAG_BLOCKED;
    grid.passable_mask[c2_idx] = 1;

    if (!cost_is_move_legal(&grid, 5, 5, 6, 6)) {
        printf("FAILED: diagonal move rejected when both cardinal neighbors are clear!\n");
        return 1;
    }
    float cost_clear = cost_compute_edge(&grid, 5, 5, 6, 6, &cp, NULL, NULL);
    if (cost_clear >= COST_INFINITE || cost_clear <= 0.0f) {
        printf("FAILED: invalid cost for clear diagonal move: %f\n", cost_clear);
        return 1;
    }

    printf("PASSED (Strict no-corner-cutting enforced)\n");
    return 0;
}

int test_cost_admissibility_and_positivity(void) {
    printf("[TEST] Cost Strict Positivity & Mathematical Heuristic Admissibility... ");
    TerrainGrid grid;
    terrain_init(&grid, TEST_GRID_SIZE, TEST_GRID_SIZE, s_cost_heightmap, s_cost_cells, s_cost_passable);
    TerrainParams p = terrain_default_params(7890, TEST_GRID_SIZE, TEST_GRID_SIZE);
    terrain_generate(&grid, &p);

    CostParams cp = cost_default_params();
    uint32_t verified_edges = 0;

    /* 8 neighbor offsets */
    static const int DX[8] = { 1, -1, 0,  0, 1, -1,  1, -1 };
    static const int DY[8] = { 0,  0, 1, -1, 1,  1, -1, -1 };

    for (uint32_t y = 1; y < TEST_GRID_SIZE - 1; ++y) {
        for (uint32_t x = 1; x < TEST_GRID_SIZE - 1; ++x) {
            uint32_t u_idx = y * TEST_GRID_SIZE + x;
            if (!terrain_is_cell_passable(&grid, (int32_t)x, (int32_t)y)) continue;

            for (int k = 0; k < 8; ++k) {
                uint32_t vx = (uint32_t)((int)x + DX[k]);
                uint32_t vy = (uint32_t)((int)y + DY[k]);
                uint32_t v_idx = vy * TEST_GRID_SIZE + vx;

                if (!cost_is_move_legal(&grid, x, y, vx, vy)) continue;

                float fuel = 0.0f;
                float risk = 0.0f;
                float edge_cost = cost_compute_edge(&grid, x, y, vx, vy, &cp, &fuel, &risk);

                /* 1. Strict positivity */
                if (edge_cost <= 0.0f || fuel <= 0.0f) {
                    printf("FAILED: non-positive edge cost (%f) or fuel (%f) at (%u,%u)->(%u,%u)\n",
                           edge_cost, fuel, x, y, vx, vy);
                    return 1;
                }

                /* 2. Admissibility condition: h(u -> v) <= cost(u -> v) */
                float h_step = cost_heuristic(&grid, u_idx, v_idx, &cp);
                if (h_step > edge_cost + 1e-5f) {
                    printf("FAILED: Heuristic overestimation! h(%f) > cost(%f) at (%u,%u)->(%u,%u)\n",
                           h_step, edge_cost, x, y, vx, vy);
                    return 1;
                }

                verified_edges++;
            }
        }
    }

    if (verified_edges == 0) {
        printf("FAILED: no valid edges tested\n");
        return 1;
    }

    printf("PASSED (%u legal edges verified: all strictly positive & h(u->v) <= cost(u->v))\n",
           verified_edges);
    return 0;
}

int test_cost_ascent_and_downhill_mechanics(void) {
    printf("[TEST] Ascent Penalty (k) & Downhill Cost Floor (f_min)... ");
    TerrainGrid grid;
    terrain_init(&grid, TEST_GRID_SIZE, TEST_GRID_SIZE, s_cost_heightmap, s_cost_cells, s_cost_passable);
    TerrainParams p = terrain_default_params(111, TEST_GRID_SIZE, TEST_GRID_SIZE);
    terrain_generate(&grid, &p);

    CostParams cp = cost_default_params();

    /* Synthetic test: 3 adjacent cells */
    uint32_t idx_a = 10 * TEST_GRID_SIZE + 10;
    uint32_t idx_b = 10 * TEST_GRID_SIZE + 11;
    uint32_t idx_c = 10 * TEST_GRID_SIZE + 12;

    /* Make all 3 flat terrain */
    grid.cells[idx_a].flags = 0;
    grid.cells[idx_a].terrain_type = TERRAIN_FLAT;
    grid.cells[idx_a].slope_deg = 5.0f;
    grid.cells[idx_a].roughness = 0.1f;
    grid.passable_mask[idx_a] = 1;

    grid.cells[idx_b].flags = 0;
    grid.cells[idx_b].terrain_type = TERRAIN_FLAT;
    grid.cells[idx_b].slope_deg = 5.0f;
    grid.cells[idx_b].roughness = 0.1f;
    grid.passable_mask[idx_b] = 1;

    grid.cells[idx_c].flags = 0;
    grid.cells[idx_c].terrain_type = TERRAIN_FLAT;
    grid.cells[idx_c].slope_deg = 5.0f;
    grid.cells[idx_c].roughness = 0.1f;
    grid.passable_mask[idx_c] = 1;

    /* Scenario 1: Flat move */
    grid.cells[idx_a].height = 10.0f;
    grid.cells[idx_b].height = 10.0f;
    float cost_flat = cost_compute_edge(&grid, 10, 10, 11, 10, &cp, NULL, NULL);

    /* Scenario 2: Steep uphill move (delta_h = +3.0m over 2.5m horizontal) */
    grid.cells[idx_b].height = 13.0f;
    float cost_uphill = cost_compute_edge(&grid, 10, 10, 11, 10, &cp, NULL, NULL);

    if (cost_uphill <= cost_flat) {
        printf("FAILED: uphill cost (%f) <= flat cost (%f)\n", cost_uphill, cost_flat);
        return 1;
    }

    /* Scenario 3: Extreme downhill move (delta_h = -20.0m) -> Must clamp to f_min floor */
    grid.cells[idx_b].height = -10.0f;
    float fuel_downhill = 0.0f;
    cost_compute_edge(&grid, 10, 10, 11, 10, &cp, &fuel_downhill, NULL);

    float expected_min_fuel = grid.cell_size * 1.0f * cp.downhill_fmin;
    if (fabsf(fuel_downhill - expected_min_fuel) > 1e-4f) {
        printf("FAILED: extreme downhill did not clamp to f_min floor: got %f, expected %f\n",
               fuel_downhill, expected_min_fuel);
        return 1;
    }

    printf("PASSED (Uphill penalized by k=2.5, Downhill clamped to f_min=0.25)\n");
    return 0;
}

int test_cost_ascii_preview(void) {
    printf("[TEST] ASCII Terrain Map Visualization & Export... ");
    TerrainGrid grid;
    terrain_init(&grid, 30, 15, s_cost_heightmap, s_cost_cells, s_cost_passable);
    TerrainParams p = terrain_default_params(2026, 30, 15);
    terrain_generate(&grid, &p);

    char ascii_buf[2048];
    cost_render_ascii_terrain(&grid, ascii_buf, sizeof(ascii_buf));

    if (ascii_buf[0] == '\0') {
        printf("FAILED: empty ASCII buffer\n");
        return 1;
    }

    printf("PASSED\n\n--- ASCII MAP SAMPLE ---\n%s------------------------\n", ascii_buf);
    return 0;
}

int test_cost_guards_and_extreme_parameters(void) {
    printf("[TEST] Cost Parameter Division-by-Zero Guards & Heuristic Bounds... ");
    TerrainGrid grid;
    terrain_init(&grid, 16, 16, s_cost_heightmap, s_cost_cells, s_cost_passable);
    TerrainParams p = terrain_default_params(555, 16, 16);
    terrain_generate(&grid, &p);

    /* 1. Test division-by-zero resilience in cost_cell_risk */
    CostParams extreme_cp = cost_default_params();
    extreme_cp.max_expected_rough = 0.0f; /* Malformed zero divisor */
    grid.max_slope_deg = 0.0f;            /* Malformed zero divisor */

    float risk = cost_cell_risk(&grid, 5, 5, &extreme_cp);
    if (isnan(risk) || isinf(risk)) {
        printf("FAILED: cost_cell_risk produced NaN/Inf on zero divisors: %f\n", risk);
        return 1;
    }

    /* 2. Test bounds checks in cost_heuristic */
    float h_out = cost_heuristic(&grid, 999999, 5, &extreme_cp);
    if (h_out != 0.0f) {
        printf("FAILED: cost_heuristic did not return 0.0f on out-of-bounds start node: %f\n", h_out);
        return 1;
    }
    float h_out2 = cost_heuristic(&grid, 5, 999999, &extreme_cp);
    if (h_out2 != 0.0f) {
        printf("FAILED: cost_heuristic did not return 0.0f on out-of-bounds goal node: %f\n", h_out2);
        return 1;
    }

    printf("PASSED (Zero-divisor guards prevented NaN, Invalid indices safely bounded)\n");
    return 0;
}
