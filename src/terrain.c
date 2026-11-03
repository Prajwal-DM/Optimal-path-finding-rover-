/**
 * @file terrain.c
 * @brief Procedural terrain generation and multi-sample cell analysis implementation.
 */

#include "terrain.h"
#include "core_math.h"
#include <string.h>
#include <math.h>

TerrainParams terrain_default_params(uint32_t seed, uint32_t width, uint32_t height) {
    TerrainParams p;
    p.seed = seed;
    p.width = (width > MAX_GRID_WIDTH) ? MAX_GRID_WIDTH : ((width == 0) ? 64 : width);
    p.height = (height > MAX_GRID_HEIGHT) ? MAX_GRID_HEIGHT : ((height == 0) ? 64 : height);
    p.cell_size = DEFAULT_CELL_SIZE;      /* 2.5 meters */
    p.height_scale = 18.0f;               /* +/- 9m amplitude */
    p.frequency = 0.035f;                 /* Smooth rolling hills */
    p.octaves = 5;
    p.persistence = 0.5f;
    p.lacunarity = 2.0f;
    p.max_slope_deg = DEFAULT_MAX_SLOPE_DEG; /* 30.0 degrees */
    return p;
}

RoverError terrain_init(TerrainGrid *grid,
                        uint32_t width,
                        uint32_t height,
                        float *heightmap_buffer,
                        CellData *cells_buffer,
                        uint8_t *passable_mask_buffer) {
    if (!grid || !heightmap_buffer || !cells_buffer || !passable_mask_buffer) {
        return ROVER_ERR_INVALID_PARAM;
    }
    if (width == 0 || width > MAX_GRID_WIDTH || height == 0 || height > MAX_GRID_HEIGHT) {
        return ROVER_ERR_INVALID_PARAM;
    }

    grid->width = width;
    grid->height = height;
    grid->cell_size = DEFAULT_CELL_SIZE;
    grid->max_slope_deg = DEFAULT_MAX_SLOPE_DEG;
    grid->heightmap = heightmap_buffer;
    grid->cells = cells_buffer;
    grid->passable_mask = passable_mask_buffer;
    grid->total_cells = width * height;
    grid->blocked_cells = 0;
    grid->passable_cells = 0;

    memset(grid->heightmap, 0, grid->total_cells * sizeof(float));
    memset(grid->cells, 0, grid->total_cells * sizeof(CellData));
    memset(grid->passable_mask, 0, grid->total_cells * sizeof(uint8_t));

    return ROVER_OK;
}

RoverError terrain_generate(TerrainGrid *grid, const TerrainParams *params) {
    if (!grid || !params) return ROVER_ERR_INVALID_PARAM;
    if (params->width != grid->width || params->height != grid->height) {
        return ROVER_ERR_INVALID_PARAM;
    }

    grid->params = *params;
    grid->cell_size = params->cell_size;
    grid->max_slope_deg = params->max_slope_deg;

    PerlinNoise noise;
    noise_init(&noise, (uint64_t)params->seed);

    uint32_t w = grid->width;
    uint32_t h = grid->height;
    float cs = params->cell_size;

    grid->blocked_cells = 0;
    grid->passable_cells = 0;

    for (uint32_t y = 0; y < h; ++y) {
        for (uint32_t x = 0; x < w; ++x) {
            uint32_t idx = y * w + x;

            /* Continuous coordinates in cell units */
            float fx = (float)x;
            float fy = (float)y;

            /* 9-point sub-cell sampling positions:
             * 0: Center
             * 1: Top-Left, 2: Top-Right, 3: Bottom-Left, 4: Bottom-Right
             * 5: Top, 6: Bottom, 7: Left, 8: Right
             */
            float samples[9];
            samples[0] = noise_sample_fbm(&noise, fx + 0.5f, fy + 0.5f,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[1] = noise_sample_fbm(&noise, fx, fy,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[2] = noise_sample_fbm(&noise, fx + 1.0f, fy,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[3] = noise_sample_fbm(&noise, fx, fy + 1.0f,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[4] = noise_sample_fbm(&noise, fx + 1.0f, fy + 1.0f,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[5] = noise_sample_fbm(&noise, fx + 0.5f, fy,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[6] = noise_sample_fbm(&noise, fx + 0.5f, fy + 1.0f,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[7] = noise_sample_fbm(&noise, fx, fy + 0.5f,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            samples[8] = noise_sample_fbm(&noise, fx + 1.0f, fy + 0.5f,
                                          params->octaves, params->frequency,
                                          params->persistence, params->lacunarity) * params->height_scale;

            /* 1. Mean height */
            float sum = 0.0f;
            for (int i = 0; i < 9; ++i) {
                sum += samples[i];
            }
            float mean_h = sum / 9.0f;

            /* 2. Unbiased Sample Standard Deviation (Bessel's correction with N - 1 = 8 degrees of freedom) */
            float sum_sq_diff = 0.0f;
            for (int i = 0; i < 9; ++i) {
                float diff = samples[i] - mean_h;
                sum_sq_diff += diff * diff;
            }
            float roughness = sqrtf(sum_sq_diff / 8.0f);

            /* 3. Comprehensive Maximum Internal Slope across all C(9, 2) = 36 sample pairs
             * Evaluates all sub-cell micro-ridges, corner-to-edge drops, and diagonals.
             */
            static const float SUB_COORDS[9][2] = {
                { 0.5f, 0.5f }, /* 0: Center */
                { 0.0f, 0.0f }, /* 1: Top-Left */
                { 1.0f, 0.0f }, /* 2: Top-Right */
                { 0.0f, 1.0f }, /* 3: Bottom-Left */
                { 1.0f, 1.0f }, /* 4: Bottom-Right */
                { 0.5f, 0.0f }, /* 5: Top-Mid */
                { 0.5f, 1.0f }, /* 6: Bottom-Mid */
                { 0.0f, 0.5f }, /* 7: Left-Mid */
                { 1.0f, 0.5f }  /* 8: Right-Mid */
            };

            float max_slope = 0.0f;
            for (int i = 0; i < 9; ++i) {
                for (int j = i + 1; j < 9; ++j) {
                    float dx = (SUB_COORDS[j][0] - SUB_COORDS[i][0]) * cs;
                    float dy = (SUB_COORDS[j][1] - SUB_COORDS[i][1]) * cs;
                    float dist_meters = sqrtf(dx * dx + dy * dy);
                    float slope_ij = math_calc_slope_deg(fabsf(samples[i] - samples[j]), dist_meters);
                    if (slope_ij > max_slope) {
                        max_slope = slope_ij;
                    }
                }
            }

            /* Store in flat arrays */
            grid->heightmap[idx] = mean_h;
            grid->cells[idx].height = mean_h;
            grid->cells[idx].slope_deg = max_slope;
            grid->cells[idx].roughness = roughness;

            /* Passability and terrain classification */
            if (max_slope > params->max_slope_deg) {
                grid->cells[idx].flags = CELL_FLAG_BLOCKED;
                grid->cells[idx].terrain_type = (uint8_t)TERRAIN_CLIFF;
                grid->passable_mask[idx] = 0;
                grid->blocked_cells++;
            } else {
                grid->cells[idx].flags = 0;
                grid->passable_mask[idx] = 1;
                grid->passable_cells++;

                if (max_slope > 20.0f || roughness > 0.60f) {
                    grid->cells[idx].terrain_type = (uint8_t)TERRAIN_ROCK;
                } else if (max_slope > 10.0f || roughness > 0.30f) {
                    grid->cells[idx].terrain_type = (uint8_t)TERRAIN_ROUGH;
                } else if (mean_h < -0.15f * params->height_scale) {
                    grid->cells[idx].terrain_type = (uint8_t)TERRAIN_SAND;
                } else {
                    grid->cells[idx].terrain_type = (uint8_t)TERRAIN_FLAT;
                }
            }
        }
    }

    return ROVER_OK;
}

bool terrain_is_cell_passable(const TerrainGrid *grid, int32_t x, int32_t y) {
    if (!grid || x < 0 || x >= (int32_t)grid->width || y < 0 || y >= (int32_t)grid->height) {
        return false;
    }
    uint32_t idx = (uint32_t)y * grid->width + (uint32_t)x;
    return (grid->passable_mask[idx] != 0);
}

const CellData *terrain_get_cell(const TerrainGrid *grid, int32_t x, int32_t y) {
    if (!grid || x < 0 || x >= (int32_t)grid->width || y < 0 || y >= (int32_t)grid->height) {
        return NULL;
    }
    uint32_t idx = (uint32_t)y * grid->width + (uint32_t)x;
    return &grid->cells[idx];
}
