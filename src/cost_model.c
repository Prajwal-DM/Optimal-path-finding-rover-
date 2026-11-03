/**
 * @file cost_model.c
 * @brief Direction-dependent physical edge cost calculations and admissible heuristic.
 */

#include "cost_model.h"
#include "core_math.h"
#include <stdio.h>
#include <math.h>

CostParams cost_default_params(void) {
    CostParams p;
    p.alpha = 1.0f;
    p.beta = 1.0f;
    p.ascent_penalty_k = ASCENT_PENALTY_K;       /* 2.5f */
    p.downhill_fmin = DOWN_HILL_COST_FLOOR;       /* 0.25f */
    p.risk_weight_w1 = 0.70f;                    /* 70% slope impact */
    p.risk_weight_w2 = 0.30f;                    /* 30% roughness impact */
    p.max_expected_rough = 1.20f;                /* 1.2m roughness normalizer */
    return p;
}

float cost_terrain_multiplier(TerrainType type) {
    switch (type) {
        case TERRAIN_FLAT:  return 1.00f;
        case TERRAIN_SAND:  return 1.35f;
        case TERRAIN_ROUGH: return 1.65f;
        case TERRAIN_ROCK:  return 2.10f;
        case TERRAIN_CLIFF:
        default:            return COST_INFINITE;
    }
}

float cost_cell_risk(const TerrainGrid *grid, uint32_t x, uint32_t y, const CostParams *params) {
    if (!grid || !params || x >= grid->width || y >= grid->height) {
        return 1.0f;
    }

    uint32_t idx = y * grid->width + x;
    const CellData *cell = &grid->cells[idx];

    if (cell->flags & CELL_FLAG_BLOCKED) {
        return 1.0f;
    }

    /* Guard against zero or negative configuration divisors to prevent NaN */
    float max_slope = (grid->max_slope_deg > 1e-4f) ? grid->max_slope_deg : 30.0f;
    float max_rough = (params->max_expected_rough > 1e-4f) ? params->max_expected_rough : 1.20f;

    float norm_slope = math_clampf(cell->slope_deg / max_slope, 0.0f, 1.0f);
    float norm_rough = math_clampf(cell->roughness / max_rough, 0.0f, 1.0f);

    float risk = params->risk_weight_w1 * norm_slope + params->risk_weight_w2 * norm_rough;
    return math_clampf(risk, 0.0f, 1.0f);
}

bool cost_is_move_legal(const TerrainGrid *grid,
                        uint32_t u_x, uint32_t u_y,
                        uint32_t v_x, uint32_t v_y) {
    if (!grid) return false;

    /* 1. Both endpoints must be strictly passable */
    if (!terrain_is_cell_passable(grid, (int32_t)u_x, (int32_t)u_y)) return false;
    if (!terrain_is_cell_passable(grid, (int32_t)v_x, (int32_t)v_y)) return false;

    /* 2. Must be adjacent 8-connected neighbors */
    int32_t dx = (int32_t)v_x - (int32_t)u_x;
    int32_t dy = (int32_t)v_y - (int32_t)u_y;
    if (dx < -1 || dx > 1 || dy < -1 || dy > 1 || (dx == 0 && dy == 0)) {
        return false;
    }

    /* 3. Corner-cutting prevention: for diagonal moves, both cardinal neighbors must be passable */
    if (dx != 0 && dy != 0) {
        if (!terrain_is_cell_passable(grid, (int32_t)u_x + dx, (int32_t)u_y)) {
            return false;
        }
        if (!terrain_is_cell_passable(grid, (int32_t)u_x, (int32_t)u_y + dy)) {
            return false;
        }
    }

    return true;
}

float cost_compute_edge(const TerrainGrid *grid,
                        uint32_t u_x, uint32_t u_y,
                        uint32_t v_x, uint32_t v_y,
                        const CostParams *params,
                        float *out_fuel,
                        float *out_risk) {
    if (!grid || !params) return COST_INFINITE;

    if (!cost_is_move_legal(grid, u_x, u_y, v_x, v_y)) {
        return COST_INFINITE;
    }

    int32_t dx = math_absi((int32_t)v_x - (int32_t)u_x);
    int32_t dy = math_absi((int32_t)v_y - (int32_t)u_y);

    /* Move distance d in physical meters */
    float d = ((dx + dy) == 1) ? grid->cell_size : (MATH_SQRT2 * grid->cell_size);

    uint32_t u_idx = u_y * grid->width + u_x;
    uint32_t v_idx = v_y * grid->width + v_x;

    const CellData *u_cell = &grid->cells[u_idx];
    const CellData *v_cell = &grid->cells[v_idx];

    /* Elevation difference and physical gradient */
    float delta_h = v_cell->height - u_cell->height;
    float gradient = delta_h / d;

    /* Terrain resistance multiplier tau */
    float tau = cost_terrain_multiplier((TerrainType)v_cell->terrain_type);

    /* Fuel calculation with ascent penalty k and downhill floor f_min */
    float climb_factor;
    if (gradient >= 0.0f) {
        climb_factor = 1.0f + params->ascent_penalty_k * gradient;
    } else {
        float discounted = 1.0f + 0.5f * gradient;
        climb_factor = math_maxf(params->downhill_fmin, discounted);
    }
    float fuel = d * tau * climb_factor;

    /* Risk calculation */
    float risk_factor = cost_cell_risk(grid, v_x, v_y, params);
    float risk_component = risk_factor * d;

    /* Combined scalar cost */
    float total_cost = params->alpha * fuel + params->beta * risk_component;

    /* Guard against zero or non-positive cost if alpha and/or beta are 0.0 */
    if (total_cost < 0.0001f * d) {
        total_cost = 0.0001f * d;
    }

    if (out_fuel) *out_fuel = fuel;
    if (out_risk) *out_risk = risk_component;

    return total_cost;
}

float cost_compute_edge_by_index(const TerrainGrid *grid,
                                 uint32_t u_idx,
                                 uint32_t v_idx,
                                 const CostParams *params,
                                 float *out_fuel,
                                 float *out_risk) {
    if (!grid) return COST_INFINITE;
    uint32_t u_x, u_y, v_x, v_y;
    terrain_index_to_coord(grid, u_idx, &u_x, &u_y);
    terrain_index_to_coord(grid, v_idx, &v_x, &v_y);
    return cost_compute_edge(grid, u_x, u_y, v_x, v_y, params, out_fuel, out_risk);
}

float cost_heuristic(const TerrainGrid *grid,
                     uint32_t u_idx,
                     uint32_t goal_idx,
                     const CostParams *params) {
    if (!grid || !params) return 0.0f;
    if (u_idx >= grid->total_cells || goal_idx >= grid->total_cells) return 0.0f;

    uint32_t u_x, u_y, g_x, g_y;
    terrain_index_to_coord(grid, u_idx, &u_x, &u_y);
    terrain_index_to_coord(grid, goal_idx, &g_x, &g_y);

    return math_admissible_heuristic((int32_t)u_x, (int32_t)u_y,
                                    (int32_t)g_x, (int32_t)g_y,
                                    grid->cell_size,
                                    params->alpha,
                                    params->downhill_fmin);
}

void cost_render_ascii_terrain(const TerrainGrid *grid, char *buffer, size_t buf_size) {
    if (!grid || !buffer || buf_size == 0) return;

    size_t written = 0;
    int n = snprintf(buffer + written, buf_size - written,
                     "Terrain Grid: %ux%u | Total: %u | Passable: %u (%.1f%%) | Blocked: %u\n",
                     grid->width, grid->height, grid->total_cells,
                     grid->passable_cells,
                     (float)grid->passable_cells * 100.0f / (float)grid->total_cells,
                     grid->blocked_cells);
    if (n > 0) written += (size_t)n;

    /* Render sample window up to 40x20 for clean console display */
    uint32_t render_w = (grid->width > 40) ? 40 : grid->width;
    uint32_t render_h = (grid->height > 20) ? 20 : grid->height;

    for (uint32_t y = 0; y < render_h && written + render_w + 2 < buf_size; ++y) {
        for (uint32_t x = 0; x < render_w && written + 2 < buf_size; ++x) {
            uint32_t idx = y * grid->width + x;
            char ch = '.';
            if (grid->cells[idx].flags & CELL_FLAG_BLOCKED) {
                ch = '#'; /* Cliff */
            } else {
                switch ((TerrainType)grid->cells[idx].terrain_type) {
                    case TERRAIN_FLAT:  ch = '.'; break;
                    case TERRAIN_SAND:  ch = '~'; break;
                    case TERRAIN_ROUGH: ch = '*'; break;
                    case TERRAIN_ROCK:  ch = '^'; break;
                    default:            ch = '?'; break;
                }
            }
            buffer[written++] = ch;
        }
        buffer[written++] = '\n';
    }
    buffer[written] = '\0';
}
