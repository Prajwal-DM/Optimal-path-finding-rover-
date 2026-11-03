/**
 * @file terrain.h
 * @brief Procedural terrain generation, multi-sampling cell analysis, and grid storage.
 */

#ifndef TERRAIN_H
#define TERRAIN_H

#include "common.h"
#include "noise.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Attributes computed for each grid cell via 9-point sub-cell sampling.
 */
typedef struct {
    float height;          /* Mean elevation across 9 samples (meters) */
    float slope_deg;       /* Maximum internal slope angle (degrees) */
    float roughness;       /* Height sample standard deviation (meters) */
    uint8_t terrain_type;  /* TerrainType enum */
    uint8_t flags;         /* CELL_FLAG_BLOCKED, etc. */
    uint8_t reserved[2];   /* Alignment padding */
} CellData;

/**
 * @brief Parameters for procedural terrain generation.
 */
typedef struct {
    uint32_t seed;
    uint32_t width;
    uint32_t height;
    float cell_size;        /* Width of a cell in meters (default: 2.5m) */
    float height_scale;     /* Peak-to-valley height amplitude in meters (default: 18.0m) */
    float frequency;        /* Base spatial noise frequency (default: 0.035f) */
    int octaves;            /* Fractal octaves (default: 5) */
    float persistence;      /* Octave persistence (default: 0.5f) */
    float lacunarity;       /* Octave lacunarity (default: 2.0f) */
    float max_slope_deg;    /* Impassable cliff threshold (default: 30.0f deg) */
} TerrainParams;

/**
 * @brief Complete terrain grid holding raw elevation buffers and cell analysis.
 * Pointers can be exported directly into WebAssembly memory buffers.
 */
typedef struct {
    uint32_t width;
    uint32_t height;
    float cell_size;
    float max_slope_deg;
    TerrainParams params;

    /* Pre-allocated flat continuous arrays */
    float *heightmap;       /* Mean height of each cell: size width * height */
    CellData *cells;        /* Cell analysis data: size width * height */
    uint8_t *passable_mask; /* 1 = passable, 0 = blocked: size width * height */

    /* Total counts */
    uint32_t total_cells;
    uint32_t blocked_cells;
    uint32_t passable_cells;
} TerrainGrid;

/**
 * @brief Get default procedural generation parameters.
 */
TerrainParams terrain_default_params(uint32_t seed, uint32_t width, uint32_t height);

/**
 * @brief Initialize a terrain grid structure with externally allocated or static buffers.
 */
RoverError terrain_init(TerrainGrid *grid,
                        uint32_t width,
                        uint32_t height,
                        float *heightmap_buffer,
                        CellData *cells_buffer,
                        uint8_t *passable_mask_buffer);

/**
 * @brief Generate procedural terrain using multi-octave fractal noise and 9-point cell analysis.
 */
RoverError terrain_generate(TerrainGrid *grid, const TerrainParams *params);

/**
 * @brief Check if cell (x, y) is inside grid bounds and non-blocked.
 */
bool terrain_is_cell_passable(const TerrainGrid *grid, int32_t x, int32_t y);

/**
 * @brief Safe accessor for cell data by coordinate.
 */
const CellData *terrain_get_cell(const TerrainGrid *grid, int32_t x, int32_t y);

/**
 * @brief Fast inline index lookup: y * width + x.
 */
static inline uint32_t terrain_coord_to_index(const TerrainGrid *grid, uint32_t x, uint32_t y) {
    return y * grid->width + x;
}

/**
 * @brief Convert index back to (x, y).
 */
static inline void terrain_index_to_coord(const TerrainGrid *grid, uint32_t index, uint32_t *out_x, uint32_t *out_y) {
    if (out_x) *out_x = index % grid->width;
    if (out_y) *out_y = index / grid->width;
}

#ifdef __cplusplus
}
#endif

#endif /* TERRAIN_H */
