/**
 * @file test_runner.c
 * @brief Master CLI test runner across Phase 1, Phase 2, and Phase 3.
 */

#include <stdio.h>
#include <time.h>

/* Phase 1 Tests */
int test_math_octile_distance(void);
int test_math_slope_calculation(void);
int test_prng_golden_vectors(void);
int test_prng_determinism(void);
int test_prng_range_and_distribution(void);
int test_prng_extreme_int_ranges(void);
int test_heap_empty_and_bounds_errors(void);
int test_heap_clear_and_reuse(void);
int test_heap_telemetry_stats(void);
int test_heap_basic_order(void);
int test_heap_decrease_key(void);
int test_heap_stress_random(void);

/* Phase 2 Tests */
int test_noise_bounds_and_continuity(void);
int test_noise_determinism_golden(void);
int test_terrain_generation_and_sampling(void);
int test_terrain_reproducibility(void);
int test_terrain_boundary_checks(void);
int test_terrain_subcell_micro_ridge_detection(void);
int test_cost_corner_cutting_prevention(void);
int test_cost_admissibility_and_positivity(void);
int test_cost_ascent_and_downhill_mechanics(void);
int test_cost_ascii_preview(void);
int test_cost_guards_and_extreme_parameters(void);

/* Phase 3 Tests */
int test_search_edge_cases(void);
int test_search_connectivity_early_exit(void);
int test_search_astar_vs_dijkstra_optimality(void);
int test_search_greedy_behavior(void);
int test_search_stepping_execution(void);
int test_search_weight_tuning_effect(void);
int test_search_path_smoothing(void);

/* Phase 4 Tests */
int test_wasm_bridge_lifecycle(void);

int main(void) {
    printf("====================================================\n");
    printf(" ROVER PATHFINDING ENGINE - MASTER TEST HARNESS      \n");
    printf(" Phase 1, 2 & 3: DSA, Terrain, Cost Model & A* Core \n");
    printf("====================================================\n\n");

    clock_t start_time = clock();
    int failed = 0;
    int total = 0;

    #define RUN_TEST(fn) do { \
        total++; \
        if (fn() != 0) { \
            failed++; \
        } \
    } while(0)

    printf("--- [PHASE 1: DSA, PRNG & SCALAR MATH] ---\n");
    RUN_TEST(test_math_octile_distance);
    RUN_TEST(test_math_slope_calculation);
    RUN_TEST(test_prng_golden_vectors);
    RUN_TEST(test_prng_determinism);
    RUN_TEST(test_prng_range_and_distribution);
    RUN_TEST(test_prng_extreme_int_ranges);
    RUN_TEST(test_heap_empty_and_bounds_errors);
    RUN_TEST(test_heap_clear_and_reuse);
    RUN_TEST(test_heap_telemetry_stats);
    RUN_TEST(test_heap_basic_order);
    RUN_TEST(test_heap_decrease_key);
    RUN_TEST(test_heap_stress_random);

    printf("\n--- [PHASE 2: NOISE, TERRAIN & COST MODEL] ---\n");
    RUN_TEST(test_noise_bounds_and_continuity);
    RUN_TEST(test_noise_determinism_golden);
    RUN_TEST(test_terrain_generation_and_sampling);
    RUN_TEST(test_terrain_reproducibility);
    RUN_TEST(test_terrain_boundary_checks);
    RUN_TEST(test_terrain_subcell_micro_ridge_detection);
    RUN_TEST(test_cost_corner_cutting_prevention);
    RUN_TEST(test_cost_admissibility_and_positivity);
    RUN_TEST(test_cost_ascent_and_downhill_mechanics);
    RUN_TEST(test_cost_ascii_preview);
    RUN_TEST(test_cost_guards_and_extreme_parameters);

    printf("\n--- [PHASE 3: SEARCH ENGINE, OPTIMALITY & STEPPING] ---\n");
    RUN_TEST(test_search_edge_cases);
    RUN_TEST(test_search_connectivity_early_exit);
    RUN_TEST(test_search_astar_vs_dijkstra_optimality);
    RUN_TEST(test_search_greedy_behavior);
    RUN_TEST(test_search_stepping_execution);
    RUN_TEST(test_search_weight_tuning_effect);
    RUN_TEST(test_search_path_smoothing);

    printf("\n--- [PHASE 4: WEBASSEMBLY ZERO-COPY MEMORY BRIDGE] ---\n");
    RUN_TEST(test_wasm_bridge_lifecycle);

    clock_t end_time = clock();
    double elapsed_ms = (double)(end_time - start_time) * 1000.0 / CLOCKS_PER_SEC;

    printf("\n----------------------------------------------------\n");
    if (failed == 0) {
        printf(">>> ALL %d TESTS PASSED SUCCESSFULLY! (%.2f ms) <<<\n", total, elapsed_ms);
    } else {
        printf(">>> %d of %d TESTS FAILED! (%.2f ms) <<<\n", failed, total, elapsed_ms);
    }
    printf("====================================================\n");

    return (failed == 0) ? 0 : 1;
}
