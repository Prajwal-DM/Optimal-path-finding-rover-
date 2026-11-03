/**
 * @file search.h
 * @brief Search Engine Core implementing stepped A*, Dijkstra, Greedy Best-First,
 * and early-exit BFS connectivity pre-checking. Zero allocations in search loop.
 */

#ifndef SEARCH_H
#define SEARCH_H

#include "common.h"
#include "terrain.h"
#include "cost_model.h"
#include "min_heap.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Detailed profiling telemetry and DSA metrics.
 */
typedef struct {
    uint32_t nodes_expanded;        /* Total nodes popped from open set */
    uint32_t max_heap_size;         /* Peak size of binary min-heap */
    uint32_t path_length_nodes;     /* Number of waypoints in path */
    float path_distance_meters;     /* Total physical ground distance in meters */
    float path_cost_total;          /* Total traversal scalar cost */
    float path_cost_fuel;           /* Isolated fuel consumption component */
    float path_cost_risk;           /* Isolated terrain risk component */
    double search_time_ms;          /* Elapsed execution time in milliseconds */
} SearchStats;

/**
 * @brief Search context state machine for single-run or stepped execution.
 */
typedef struct {
    const TerrainGrid *grid;
    CostParams cost_params;
    SearchAlgorithm algorithm;
    SearchStatus status;

    uint32_t start_node;
    uint32_t goal_node;

    /* Flat per-node search buffers (size: grid->total_cells) */
    float *g_score;
    float *f_score;
    int32_t *came_from;
    uint8_t *node_state;            /* 0 = unvisited, CELL_FLAG_OPEN, CELL_FLAG_CLOSED */

    /* Binary Min-Heap Priority Queue */
    MinHeap open_heap;

    /* Reconstructed path buffer */
    uint32_t *path_nodes;
    uint32_t path_capacity;

    /* Flat buffer for BFS connectivity checks */
    uint32_t *bfs_queue;
    uint8_t *bfs_visited;

    /* Telemetry */
    SearchStats stats;
} SearchContext;

/**
 * @brief Initialize search context with static or pre-allocated flat arrays.
 */
RoverError search_init(SearchContext *ctx,
                       const TerrainGrid *grid,
                       float *g_score_buf,
                       float *f_score_buf,
                       int32_t *came_from_buf,
                       uint8_t *node_state_buf,
                       HeapElement *heap_elements_buf,
                       int32_t *heap_indices_buf,
                       uint32_t *path_buf,
                       uint32_t path_capacity,
                       uint32_t *bfs_queue_buf,
                       uint8_t *bfs_visited_buf);

/**
 * @brief Fast BFS flood fill to determine if start and goal are in the same connected component.
 * @return true if connected, false if completely blocked by cliffs/obstacles.
 */
bool search_check_connectivity(SearchContext *ctx, uint32_t start, uint32_t goal);

/**
 * @brief Begin a new search from start to goal using the chosen algorithm.
 * Validates endpoints and runs fast connectivity check before setting status to SEARCH_RUNNING.
 */
RoverError search_start(SearchContext *ctx,
                        uint32_t start_node,
                        uint32_t goal_node,
                        SearchAlgorithm algorithm,
                        const CostParams *cost_params);

/**
 * @brief Advance the search by expanding up to max_expansions nodes.
 * Enables 60 FPS stepping and live visualization in browser/Three.js.
 * @return Current SearchStatus (SEARCH_RUNNING, SEARCH_FOUND, SEARCH_UNREACHABLE)
 */
SearchStatus search_step(SearchContext *ctx, uint32_t max_expansions);

/**
 * @brief Execute the search synchronously to completion in a single call.
 */
SearchStatus search_solve(SearchContext *ctx);

/**
 * @brief Get the reconstructed path as an array of 1D node indices.
 * @param out_count Pointer to receive number of nodes in path
 * @return Pointer to path buffer, or NULL if no path found
 */
const uint32_t *search_get_path(const SearchContext *ctx, uint32_t *out_count);

/**
 * @brief Get current telemetry stats.
 */
const SearchStats *search_get_stats(const SearchContext *ctx);

/**
 * @brief Test whether a direct line of sight exists between two grid nodes across passable terrain.
 * Checks continuous ray sub-cell samples for obstacle collision and slope threshold violation.
 */
bool search_has_line_of_sight(const TerrainGrid *grid, uint32_t u, uint32_t v);

/**
 * @brief Smooth a discrete path using greedy line-of-sight shortcutting (string pulling).
 * Eliminates jagged 45/90 degree zig-zags while guaranteeing safe traversability.
 * @param grid Pointer to terrain grid
 * @param raw_path Array of discrete path node indices
 * @param raw_len Number of nodes in raw_path
 * @param out_smoothed Buffer to receive smoothed node indices (must hold at least raw_len)
 * @param out_smoothed_len Pointer to receive number of nodes in smoothed path
 * @return ROVER_OK on success, or error code
 */
RoverError search_smooth_path(const TerrainGrid *grid,
                              const uint32_t *raw_path,
                              uint32_t raw_len,
                              uint32_t *out_smoothed,
                              uint32_t *out_smoothed_len);

#ifdef __cplusplus
}
#endif

#endif /* SEARCH_H */
