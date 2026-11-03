/**
 * @file cost_model.h
 * @brief Direction-dependent physical edge cost calculations and admissible heuristic.
 */

#ifndef COST_MODEL_H
#define COST_MODEL_H

#include "terrain.h"
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define COST_INFINITE 1e9f

/**
 * @brief Adjustable weights and physical tuning parameters for edge costs.
 */
typedef struct {
    float alpha;            /* Fuel consumption weight (default: 1.0f) */
    float beta;             /* Terrain risk weight (default: 1.0f) */
    float ascent_penalty_k; /* Steep climbing penalty k (default: 2.5f) */
    float downhill_fmin;    /* Minimum fuel cost multiplier floor f_min (default: 0.25f) */
    float risk_weight_w1;   /* Slope weight in risk formula (default: 0.7f) */
    float risk_weight_w2;   /* Roughness weight in risk formula (default: 0.3f) */
    float max_expected_rough; /* Normalization factor for roughness (default: 1.2m) */
} CostParams;

/**
 * @brief Default cost tuning parameters.
 */
CostParams cost_default_params(void);

/**
 * @brief Get friction / resistance multiplier tau(v) for a terrain type.
 */
float cost_terrain_multiplier(TerrainType type);

/**
 * @brief Compute the normalized risk factor for a destination cell v in [0.0f, 1.0f].
 */
float cost_cell_risk(const TerrainGrid *grid, uint32_t x, uint32_t y, const CostParams *params);

/**
 * @brief Validate whether moving from cell u to neighbor v is legal.
 * Enforces boundary checks, blocked cell checks, and diagonal corner-cutting prevention.
 */
bool cost_is_move_legal(const TerrainGrid *grid,
                        uint32_t u_x, uint32_t u_y,
                        uint32_t v_x, uint32_t v_y);

/**
 * @brief Calculate direction-dependent edge cost from cell u to neighbor v.
 *
 * @param grid Terrain grid
 * @param u_x, u_y Starting cell coordinates
 * @param v_x, v_y Destination neighbor coordinates
 * @param params Tuning parameters (alpha, beta, k, f_min)
 * @param out_fuel Optional pointer to receive isolated fuel cost component
 * @param out_risk Optional pointer to receive isolated risk cost component
 * @return Total scalar traversal cost, or COST_INFINITE if move is illegal
 */
float cost_compute_edge(const TerrainGrid *grid,
                        uint32_t u_x, uint32_t u_y,
                        uint32_t v_x, uint32_t v_y,
                        const CostParams *params,
                        float *out_fuel,
                        float *out_risk);

/**
 * @brief Calculate edge cost by 1D node indices.
 */
float cost_compute_edge_by_index(const TerrainGrid *grid,
                                uint32_t u_idx,
                                uint32_t v_idx,
                                const CostParams *params,
                                float *out_fuel,
                                float *out_risk);

/**
 * @brief Compute the provably admissible and consistent heuristic cost h(n) to goal.
 * Formula: h(n) = alpha * f_min * octile_distance_scaled(n, goal, cell_size).
 */
float cost_heuristic(const TerrainGrid *grid,
                     uint32_t u_idx,
                     uint32_t goal_idx,
                     const CostParams *params);

/**
 * @brief Render an ASCII representation of the terrain into a string buffer for CLI inspection.
 * @param grid Terrain grid
 * @param buffer Output char buffer
 * @param buf_size Size of output buffer
 */
void cost_render_ascii_terrain(const TerrainGrid *grid, char *buffer, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif /* COST_MODEL_H */
