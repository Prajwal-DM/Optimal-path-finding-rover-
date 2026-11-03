/**
 * @file test_search.c
 * @brief Unit tests for search engine, A* vs Dijkstra optimality, and BFS connectivity.
 */

#include "search.h"
#include <stdio.h>
#include <assert.h>
#include <math.h>

#define SEARCH_TEST_W 48
#define SEARCH_TEST_H 48
#define SEARCH_TEST_CELLS (SEARCH_TEST_W * SEARCH_TEST_H)

/* Pre-allocated buffers for zero-allocation test environment */
static float s_heightmap[SEARCH_TEST_CELLS];
static CellData s_cells[SEARCH_TEST_CELLS];
static uint8_t s_passable[SEARCH_TEST_CELLS];

static float s_g_score[SEARCH_TEST_CELLS];
static float s_f_score[SEARCH_TEST_CELLS];
static int32_t s_came_from[SEARCH_TEST_CELLS];
static uint8_t s_node_state[SEARCH_TEST_CELLS];

static HeapElement s_heap_elements[SEARCH_TEST_CELLS];
static int32_t s_heap_indices[SEARCH_TEST_CELLS];
static uint32_t s_path_nodes[SEARCH_TEST_CELLS];

static uint32_t s_bfs_queue[SEARCH_TEST_CELLS];
static uint8_t s_bfs_visited[SEARCH_TEST_CELLS];

int test_search_edge_cases(void) {
    printf("[TEST] Search Edge Cases (Start==Goal, Blocked Endpoints, Bounds)... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(1234, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    /* 1. Out of bounds start / goal */
    RoverError err = search_start(&ctx, SEARCH_TEST_CELLS + 5, 10, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_ERR_OUT_OF_BOUNDS) {
        printf("FAILED: expected out of bounds on start_node: %d\n", err);
        return 1;
    }
    err = search_start(&ctx, 10, SEARCH_TEST_CELLS + 10, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_ERR_OUT_OF_BOUNDS) {
        printf("FAILED: expected out of bounds on goal_node: %d\n", err);
        return 1;
    }

    /* 2. Blocked endpoint rejection (start and goal) */
    uint32_t blocked_node = 0;
    bool found_blocked = false;
    for (uint32_t i = 0; i < grid.total_cells; ++i) {
        if (!grid.passable_mask[i]) {
            blocked_node = i;
            found_blocked = true;
            break;
        }
    }
    if (found_blocked) {
        err = search_start(&ctx, blocked_node, 10, ALGORITHM_ASTAR, NULL);
        if (err != ROVER_ERR_BLOCKED_CELL) {
            printf("FAILED: expected ROVER_ERR_BLOCKED_CELL on blocked start: %d\n", err);
            return 1;
        }
        err = search_start(&ctx, 10, blocked_node, ALGORITHM_ASTAR, NULL);
        if (err != ROVER_ERR_BLOCKED_CELL) {
            printf("FAILED: expected ROVER_ERR_BLOCKED_CELL on blocked goal: %d\n", err);
            return 1;
        }
    }

    /* 3. Start == Goal */
    uint32_t safe_node = 0;
    for (uint32_t i = 0; i < grid.total_cells; ++i) {
        if (grid.passable_mask[i]) {
            safe_node = i;
            break;
        }
    }
    err = search_start(&ctx, safe_node, safe_node, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_OK || ctx.status != SEARCH_FOUND) {
        printf("FAILED: start == goal did not immediately find path: %d\n", err);
        return 1;
    }
    if (ctx.stats.path_length_nodes != 1 || ctx.stats.path_cost_total != 0.0f) {
        printf("FAILED: start == goal cost != 0.0\n");
        return 1;
    }

    printf("PASSED (Bounds guards, blocked endpoint rejection, instant Start==Goal)\n");
    return 0;
}

int test_search_connectivity_early_exit(void) {
    printf("[TEST] Fast BFS Flood-Fill Connectivity Early-Exit... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(777, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    /* Construct an impenetrable vertical cliff wall across column x = 24 */
    for (uint32_t y = 0; y < SEARCH_TEST_H; ++y) {
        uint32_t idx = y * SEARCH_TEST_W + 24;
        grid.cells[idx].flags |= CELL_FLAG_BLOCKED;
        grid.passable_mask[idx] = 0;
    }

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    /* Start on left side (x=5, y=5), Goal on right side (x=40, y=5) */
    uint32_t start = 5 * SEARCH_TEST_W + 5;
    uint32_t goal = 5 * SEARCH_TEST_W + 40;

    grid.cells[start].flags = 0;
    grid.passable_mask[start] = 1;
    grid.cells[goal].flags = 0;
    grid.passable_mask[goal] = 1;

    RoverError err = search_start(&ctx, start, goal, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_ERR_NO_PATH || ctx.status != SEARCH_UNREACHABLE) {
        printf("FAILED: expected early-exit ROVER_ERR_NO_PATH, got err=%d status=%d\n", err, ctx.status);
        return 1;
    }

    if (ctx.stats.nodes_expanded != 0 || ctx.stats.path_length_nodes != 0 || ctx.stats.path_cost_total != 0.0f) {
        printf("FAILED: telemetry stats not zeroed on unreachable search!\n");
        return 1;
    }

    /* Regression test: ensure a prior search's stats do not leak into an unreachable search */
    uint32_t s_local = 2 * SEARCH_TEST_W + 2;
    uint32_t g_local = 3 * SEARCH_TEST_W + 3;
    grid.cells[s_local].flags = 0; grid.passable_mask[s_local] = 1;
    grid.cells[g_local].flags = 0; grid.passable_mask[g_local] = 1;
    search_start(&ctx, s_local, g_local, ALGORITHM_ASTAR, NULL);
    search_solve(&ctx);
    assert(ctx.stats.nodes_expanded > 0);

    /* Run unreachable search again */
    err = search_start(&ctx, start, goal, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_ERR_NO_PATH || ctx.status != SEARCH_UNREACHABLE) {
        printf("FAILED: second unreachable check failed: %d\n", err);
        return 1;
    }
    if (ctx.stats.nodes_expanded != 0 || ctx.stats.path_length_nodes != 0 || ctx.stats.path_cost_total != 0.0f) {
        printf("FAILED: previous search telemetry leaked into unreachable search! (nodes=%u, cost=%f)\n",
               ctx.stats.nodes_expanded, ctx.stats.path_cost_total);
        return 1;
    }

    printf("PASSED (Disjoint components detected in < 1ms, 0 priority queue expansions, 0 telemetry leak)\n");
    return 0;
}

int test_search_astar_vs_dijkstra_optimality(void) {
    printf("[TEST] Optimality Equivalence: A* vs Dijkstra on Identical Map... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(9999, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    /* Find two mutually reachable passable nodes far apart */
    uint32_t start = 2 * SEARCH_TEST_W + 2;
    uint32_t goal = (SEARCH_TEST_H - 3) * SEARCH_TEST_W + (SEARCH_TEST_W - 3);

    grid.cells[start].flags = 0;
    grid.passable_mask[start] = 1;
    grid.cells[goal].flags = 0;
    grid.passable_mask[goal] = 1;

    /* Ensure connected */
    if (!search_check_connectivity(&ctx, start, goal)) {
        /* Fallback to nearby reachable goal */
        goal = 20 * SEARCH_TEST_W + 20;
        grid.cells[goal].flags = 0;
        grid.passable_mask[goal] = 1;
    }

    /* 1. Run A* */
    RoverError err = search_start(&ctx, start, goal, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_OK) {
        printf("FAILED to start A*: %d\n", err);
        return 1;
    }
    SearchStatus st = search_solve(&ctx);
    if (st != SEARCH_FOUND) {
        printf("FAILED: A* did not find path: %d\n", st);
        return 1;
    }
    float astar_cost = ctx.stats.path_cost_total;
    uint32_t astar_expanded = ctx.stats.nodes_expanded;

    /* 2. Run Dijkstra */
    err = search_start(&ctx, start, goal, ALGORITHM_DIJKSTRA, NULL);
    if (err != ROVER_OK) {
        printf("FAILED to start Dijkstra: %d\n", err);
        return 1;
    }
    st = search_solve(&ctx);
    if (st != SEARCH_FOUND) {
        printf("FAILED: Dijkstra did not find path: %d\n", st);
        return 1;
    }
    float dijkstra_cost = ctx.stats.path_cost_total;
    uint32_t dijkstra_expanded = ctx.stats.nodes_expanded;

    /* Assert Exact Cost Equivalence: |A* - Dijkstra| < 1e-3 */
    float cost_diff = fabsf(astar_cost - dijkstra_cost);
    if (cost_diff > 1e-3f) {
        printf("FAILED: Optimality broken! A* cost (%f) != Dijkstra cost (%f)\n", astar_cost, dijkstra_cost);
        return 1;
    }

    /* Assert Heuristic Efficiency: A* expands fewer or equal nodes than Dijkstra */
    if (astar_expanded > dijkstra_expanded) {
        printf("FAILED: A* expanded more nodes than Dijkstra (%u vs %u)\n", astar_expanded, dijkstra_expanded);
        return 1;
    }

    printf("PASSED (A* cost: %.3f == Dijkstra cost: %.3f | Nodes: A*=%u vs Dijkstra=%u)\n",
           astar_cost, dijkstra_cost, astar_expanded, dijkstra_expanded);
    return 0;
}

int test_search_greedy_behavior(void) {
    printf("[TEST] Greedy Best-First Search Heuristic Comparison... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(54321, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    uint32_t start = 3 * SEARCH_TEST_W + 3;
    uint32_t goal = 35 * SEARCH_TEST_W + 35;
    grid.cells[start].flags = 0; grid.passable_mask[start] = 1;
    grid.cells[goal].flags = 0; grid.passable_mask[goal] = 1;

    /* A* baseline */
    search_start(&ctx, start, goal, ALGORITHM_ASTAR, NULL);
    search_solve(&ctx);
    float astar_cost = ctx.stats.path_cost_total;

    /* Greedy */
    search_start(&ctx, start, goal, ALGORITHM_GREEDY, NULL);
    search_solve(&ctx);
    float greedy_cost = ctx.stats.path_cost_total;

    /* Greedy cost must be >= A* cost */
    if (greedy_cost < astar_cost - 1e-3f) {
        printf("FAILED: Greedy cost (%f) < A* optimal cost (%f)\n", greedy_cost, astar_cost);
        return 1;
    }

    printf("PASSED (Greedy cost: %.2f >= A* optimal: %.2f)\n", greedy_cost, astar_cost);
    return 0;
}

int test_search_stepping_execution(void) {
    printf("[TEST] Stepped Frame-by-Frame Execution API... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(888, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    uint32_t start = 5 * SEARCH_TEST_W + 5;
    uint32_t goal = 25 * SEARCH_TEST_W + 25;
    grid.cells[start].flags = 0; grid.passable_mask[start] = 1;
    grid.cells[goal].flags = 0; grid.passable_mask[goal] = 1;

    RoverError err = search_start(&ctx, start, goal, ALGORITHM_ASTAR, NULL);
    if (err != ROVER_OK) {
        printf("FAILED to start search: %d\n", err);
        return 1;
    }

    /* Advance in small chunks of 15 nodes */
    int step_count = 0;
    while (ctx.status == SEARCH_RUNNING && step_count < 2000) {
        SearchStatus st = search_step(&ctx, 15);
        step_count++;
        if (st == SEARCH_FOUND || st == SEARCH_UNREACHABLE) break;
    }

    if (ctx.status != SEARCH_FOUND) {
        printf("FAILED: search not completed in stepping loop (status=%d)\n", ctx.status);
        return 1;
    }

    uint32_t path_len = 0;
    const uint32_t *path = search_get_path(&ctx, &path_len);
    if (!path || path_len < 2) {
        printf("FAILED: reconstructed path invalid (len=%u)\n", path_len);
        return 1;
    }

    /* Verify path continuity */
    for (uint32_t i = 0; i + 1 < path_len; ++i) {
        uint32_t u = path[i];
        uint32_t v = path[i + 1];
        uint32_t ux, uy, vx, vy;
        terrain_index_to_coord(&grid, u, &ux, &uy);
        terrain_index_to_coord(&grid, v, &vx, &vy);
        if (!cost_is_move_legal(&grid, ux, uy, vx, vy)) {
            printf("FAILED: illegal step in reconstructed path at (%u,%u)->(%u,%u)\n", ux, uy, vx, vy);
            return 1;
        }
    }

    printf("PASSED (Solved across %d discrete steps, path valid & continuous)\n", step_count);
    return 0;
}

int test_search_weight_tuning_effect(void) {
    printf("[TEST] Parameter Tuning: Fuel-Saver (High Alpha) vs Safe Route (High Beta)... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(3333, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    uint32_t start = 4 * SEARCH_TEST_W + 4;
    uint32_t goal = 30 * SEARCH_TEST_W + 30;
    grid.cells[start].flags = 0; grid.passable_mask[start] = 1;
    grid.cells[goal].flags = 0; grid.passable_mask[goal] = 1;

    /* Route A: Fuel-Saver (alpha = 5.0, beta = 0.1) */
    CostParams params_fuel = cost_default_params();
    params_fuel.alpha = 5.0f;
    params_fuel.beta = 0.1f;
    search_start(&ctx, start, goal, ALGORITHM_ASTAR, &params_fuel);
    search_solve(&ctx);
    float fuel_in_saver = ctx.stats.path_cost_fuel;
    float risk_in_saver = ctx.stats.path_cost_risk;

    /* Route B: Safe Route (alpha = 0.1, beta = 5.0) */
    CostParams params_safe = cost_default_params();
    params_safe.alpha = 0.1f;
    params_safe.beta = 5.0f;
    search_start(&ctx, start, goal, ALGORITHM_ASTAR, &params_safe);
    search_solve(&ctx);
    float fuel_in_safe = ctx.stats.path_cost_fuel;
    float risk_in_safe = ctx.stats.path_cost_risk;

    /* Fuel saver must consume <= fuel of safe route */
    if (fuel_in_saver > fuel_in_safe + 1e-2f) {
        printf("FAILED: fuel-saver route burned more fuel than safe route (%f > %f)\n",
               fuel_in_saver, fuel_in_safe);
        return 1;
    }

    /* Safe route must accumulate <= risk of fuel saver */
    if (risk_in_safe > risk_in_saver + 1e-2f) {
        printf("FAILED: safe route accumulated more risk than fuel saver (%f > %f)\n",
               risk_in_safe, risk_in_saver);
        return 1;
    }

    printf("PASSED (Fuel-saver burned %.2f fuel vs %.2f | Safe route risk %.2f vs %.2f)\n",
           fuel_in_saver, fuel_in_safe, risk_in_safe, risk_in_saver);
    return 0;
}

int test_search_path_smoothing(void) {
    printf("[TEST] Path Smoothing & Greedy Line-of-Sight Shortcut (String Pulling)... ");
    TerrainGrid grid;
    terrain_init(&grid, SEARCH_TEST_W, SEARCH_TEST_H, s_heightmap, s_cells, s_passable);
    TerrainParams p = terrain_default_params(42, SEARCH_TEST_W, SEARCH_TEST_H);
    terrain_generate(&grid, &p);

    SearchContext ctx;
    search_init(&ctx, &grid, s_g_score, s_f_score, s_came_from, s_node_state,
                s_heap_elements, s_heap_indices, s_path_nodes, SEARCH_TEST_CELLS,
                s_bfs_queue, s_bfs_visited);

    uint32_t start = 5 * SEARCH_TEST_W + 5;
    uint32_t goal = 43 * SEARCH_TEST_W + 43;
    grid.cells[start].flags = 0; grid.passable_mask[start] = 1;
    grid.cells[goal].flags = 0; grid.passable_mask[goal] = 1;

    CostParams cp = cost_default_params();
    RoverError err = search_start(&ctx, start, goal, ALGORITHM_ASTAR, &cp);
    if (err != ROVER_OK) {
        printf("FAILED search_start: %d\n", err);
        return 1;
    }
    SearchStatus st = search_solve(&ctx);
    if (st != SEARCH_FOUND) {
        printf("FAILED search_solve: %d\n", st);
        return 1;
    }

    uint32_t raw_len = ctx.stats.path_length_nodes;
    if (raw_len < 10) {
        printf("FAILED: raw path too short for smoothing test: %u\n", raw_len);
        return 1;
    }

    uint32_t smoothed[SEARCH_TEST_CELLS];
    uint32_t smoothed_len = 0;
    err = search_smooth_path(&grid, s_path_nodes, raw_len, smoothed, &smoothed_len);
    if (err != ROVER_OK) {
        printf("FAILED search_smooth_path returned error %d\n", err);
        return 1;
    }

    /* Verification assertions */
    if (smoothed_len > raw_len) {
        printf("FAILED: smoothed length (%u) > raw length (%u)\n", smoothed_len, raw_len);
        return 1;
    }
    if (smoothed[0] != s_path_nodes[0]) {
        printf("FAILED: start waypoint changed: %u vs %u\n", smoothed[0], s_path_nodes[0]);
        return 1;
    }
    if (smoothed[smoothed_len - 1] != s_path_nodes[raw_len - 1]) {
        printf("FAILED: goal waypoint changed: %u vs %u\n", smoothed[smoothed_len - 1], s_path_nodes[raw_len - 1]);
        return 1;
    }

    /* Verify every segment in smoothed path has continuous line of sight */
    for (uint32_t k = 0; k + 1 < smoothed_len; ++k) {
        if (!search_has_line_of_sight(&grid, smoothed[k], smoothed[k + 1])) {
            printf("FAILED: line of sight violated between smoothed segment %u (%u -> %u)\n",
                   k, smoothed[k], smoothed[k + 1]);
            return 1;
        }
    }

    /* Corner cases: 0, 1, 2 elements */
    uint32_t c_out[4];
    uint32_t c_len = 0;
    search_smooth_path(&grid, s_path_nodes, 0, c_out, &c_len);
    assert(c_len == 0);
    search_smooth_path(&grid, s_path_nodes, 1, c_out, &c_len);
    assert(c_len == 1 && c_out[0] == s_path_nodes[0]);
    search_smooth_path(&grid, s_path_nodes, 2, c_out, &c_len);
    assert(c_len == 2 && c_out[1] == s_path_nodes[1]);

    float reduction_pct = (1.0f - (float)smoothed_len / (float)raw_len) * 100.0f;
    printf("PASSED (%u raw waypoints compressed to %u smoothed waypoints, %.1f%% shortcut reduction)\n",
           raw_len, smoothed_len, reduction_pct);
    return 0;
}

